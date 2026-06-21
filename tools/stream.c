/*
 * stream.c — Step 3b: isochronous streaming engine for the RigKontrol 3.
 *
 * Opens the device, runs the EP1 handshake (GET_DEVICE_INFO + AUDIO_PARAMS),
 * then drives the isoc pipes via IOKit async transfers on a CFRunLoop:
 *   - capture  (EP 0x82 IN):  deinterleave with the mode-2 codec, write WAV
 *   - playback (EP 0x06 OUT): interleave a 440 Hz test tone (with --play)
 *
 * First run capture-only to learn the per-frame byte counts (frActCount),
 * which tells us the real isoc frame granularity on this device. Heavy
 * diagnostics are printed for the first few callbacks.
 *
 * Usage:  ./build/stream <seconds> [--play] [--rate N]
 */

#include "../usb_iokit.h"
#include "../caiaq_protocol.h"
#include "../codec_mode2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <CoreFoundation/CoreFoundation.h>

#define N_XFERS         8     /* in-flight capture transfers */
#define N_OUT_XFERS     8     /* in-flight playback transfers (free-running chain) */
#define FRAMES_PER_XFER 64    /* microframes per transfer (high-speed: 64 = 8ms) */
#define FRAME_BYTES     512   /* isoc maxPacketSize (per microframe) */
#define XFER_BUF        (FRAMES_PER_XFER * FRAME_BYTES)

/*
 * frame_start is numbered in 1ms USB frames, but the isoc frame list counts
 * microframes on this high-speed device (8 microframes per ms frame). So a
 * transfer of FRAMES_PER_XFER microframes spans FRAMES_PER_XFER/8 ms-frames;
 * consecutive transfers must advance frame_start by that, not by the
 * microframe count, or we leave 7ms gaps (the bug seen in capture v1).
 */
#define FRAME_STRIDE    (FRAMES_PER_XFER / 8)

/* playback scheduling window ahead of the bus frame (ms-frames). */
#define OUT_MIN_LEAD    24  /* ms-frames ahead of bus */
#define OUT_MAX_LEAD    48  /* drop only if absurdly far ahead */

struct iso_xfer {
	struct stream *s;
	int index;
	int busy;                /* for the output free-list */
	uint8_t buf[XFER_BUF];
	IOUSBIsocFrame frames[FRAMES_PER_XFER];
	UInt64 frame_start;
};

struct stream {
	struct rk3_usb *usb;
	struct caiaq_codec codec;
	int bpp;
	int rate;
	int do_play;
	int silence;     /* play digital zero instead of the tone */
	int pb_chk0;     /* experiment: playback check byte at i%4==0 (capture-style) */
	int const_cad;   /* experiment: steady N-frames-per-microframe OUT cadence */
	int off;         /* experiment: initial audio-byte phase offset into the tone */
	double freq;     /* test-tone frequency (Hz) */
	int running;

	/* capture accumulation (S24_3BE as received) */
	uint8_t *cap;
	size_t   cap_len, cap_cap;

	/* playback tone ring (S24_3BE), looped */
	uint8_t *tone;
	size_t   tone_size;
	int      tone_pos;

	struct iso_xfer in[N_XFERS];
	struct iso_xfer out[N_OUT_XFERS];

	int out_acc;       /* sample-rate accumulator for fallback sizing */

	/*
	 * Cadence queue: each capture completion pushes the device's actual
	 * per-microframe byte counts; the OUT chain pops them so playback replays
	 * the exact bursty 88/0 structure the device expects (a time-delayed
	 * mirror, like Linux read_completed). FIFO, single producer/consumer on
	 * the CFRunLoop thread, so no locking.
	 */
#define PAT_RING 256
	int pat[PAT_RING][FRAMES_PER_XFER];
	int pat_head, pat_tail;          /* head = next write, tail = next read */
	int last_pat[FRAMES_PER_XFER];   /* reused if the queue underruns */
	int out_started;                 /* OUT chain primed yet */
	int out_underruns;

	int in_errs, out_errs;
	int out_done_count, in_count, out_drops;
	int len_hist[FRAME_BYTES + 1];   /* histogram of OUT frame byte-counts */
	int busy_now, busy_max;
	int dbg, out_dbg;  /* callbacks left to log verbosely */
};

static int pat_count(struct stream *s) { return s->pat_head - s->pat_tail; }

static void pat_push(struct stream *s, struct iso_xfer *x)
{
	if (pat_count(s) >= PAT_RING)
		s->pat_tail++;                       /* drop oldest on overflow */
	int *slot = s->pat[s->pat_head % PAT_RING];
	for (int f = 0; f < FRAMES_PER_XFER; f++)
		slot[f] = x->frames[f].frActCount;
	s->pat_head++;
}

/*
 * Synthesize the device's bursty cadence for one transfer when no real captured
 * pattern is available (startup / underrun). A non-zero packet carries bpp/8
 * stereo frames; we need rate/8000 frames per microframe on average, so emit a
 * full bpp packet whenever that many frames have accumulated, else a 0-length
 * frame. This reproduces the 88/0 structure (with periodic doubles) the device
 * expects, at the exact sample rate.
 */
static void pat_synth(struct stream *s, int *lens)
{
	int pkt_frames = s->bpp / 8;          /* stereo frames per non-zero packet */
	int need = s->rate / 8000;            /* avg frames per microframe */
	int rem  = s->rate % 8000;
	for (int f = 0; f < FRAMES_PER_XFER; f++) {
		s->out_acc += need * 8000 + rem;  /* frames*8000, fixed-point */
		if (s->out_acc >= pkt_frames * 8000) {
			s->out_acc -= pkt_frames * 8000;
			lens[f] = s->bpp;
		} else {
			lens[f] = 0;
		}
	}
}

/*
 * Steady cadence: every microframe carries (rate/8000) stereo frames (plus an
 * occasional extra via the rate accumulator so the long-run rate is exact). No
 * zero-length frames. Each frame is 8 USB bytes in mode-2 (3 audio + 1 check
 * per 4-byte group, 2 ch * (3+1) = 8). This feeds the no-sync OUT endpoint the
 * constant per-microframe rate its DAC time-slots expect.
 */
static void pat_const(struct stream *s, int *lens)
{
	int base = s->rate / 8000;
	int rem  = s->rate % 8000;
	for (int f = 0; f < FRAMES_PER_XFER; f++) {
		int nf = base;
		s->out_acc += rem;
		if (s->out_acc >= 8000) { s->out_acc -= 8000; nf++; }
		lens[f] = nf * 8;            /* USB bytes for nf stereo frames */
	}
}

/* Fill lens[] with the next captured cadence pattern, else synthesize one. */
static void pat_pop(struct stream *s, int *lens)
{
	if (pat_count(s) > 0) {
		int *slot = s->pat[s->pat_tail % PAT_RING];
		for (int f = 0; f < FRAMES_PER_XFER; f++)
			lens[f] = slot[f];
		s->pat_tail++;
	} else {
		s->out_underruns++;
		pat_synth(s, lens);
	}
}

/* ---- WAV (24-bit PCM, little-endian) -------------------------------- */

static void put_le32(FILE *f, uint32_t v)
{
	fputc(v & 0xff, f); fputc((v >> 8) & 0xff, f);
	fputc((v >> 16) & 0xff, f); fputc((v >> 24) & 0xff, f);
}
static void put_le16(FILE *f, uint16_t v)
{
	fputc(v & 0xff, f); fputc((v >> 8) & 0xff, f);
}

/* Input is S24_3BE stereo; WAV wants S24 little-endian, so swap each sample. */
static int write_wav(const char *path, const uint8_t *be, size_t len, int rate)
{
	size_t nsamp = len / 3;                 /* 3 bytes per mono sample */
	uint32_t data_bytes = (uint32_t)(nsamp * 3);
	FILE *f = fopen(path, "wb");
	if (!f) { perror("fopen"); return -1; }

	fwrite("RIFF", 1, 4, f); put_le32(f, 36 + data_bytes);
	fwrite("WAVE", 1, 4, f);
	fwrite("fmt ", 1, 4, f); put_le32(f, 16);
	put_le16(f, 1);                         /* PCM */
	put_le16(f, 2);                         /* stereo */
	put_le32(f, rate);
	put_le32(f, rate * 2 * 3);              /* byte rate */
	put_le16(f, 2 * 3);                     /* block align */
	put_le16(f, 24);                        /* bits */
	fwrite("data", 1, 4, f); put_le32(f, data_bytes);

	for (size_t i = 0; i + 3 <= len; i += 3) {
		fputc(be[i + 2], f); fputc(be[i + 1], f); fputc(be[i], f);
	}
	fclose(f);
	printf("wrote %s (%zu stereo frames, %.2f s)\n",
	       path, nsamp / 2, (double)nsamp / 2.0 / rate);
	return 0;
}

static void cap_append(struct stream *s, const uint8_t *p, size_t n)
{
	if (s->cap_len + n > s->cap_cap) {
		size_t nc = (s->cap_cap ? s->cap_cap * 2 : (1 << 20));
		while (nc < s->cap_len + n) nc *= 2;
		s->cap = realloc(s->cap, nc);
		s->cap_cap = nc;
	}
	memcpy(s->cap + s->cap_len, p, n);
	s->cap_len += n;
}

/* ---- isoc completion callbacks -------------------------------------- */

static void resubmit_in(struct iso_xfer *x);
static int submit_out(struct iso_xfer *x);
static void fill_out(struct stream *s, struct iso_xfer *x);
static void out_done(void *refcon, IOReturn result, void *arg0);

/*
 * Self-healing schedule: keep transfers a few ms-frames ahead of the bus.
 * If a callback fell behind and `want` is already in the past, re-anchor to
 * the current bus frame (+margin), accepting a small audio gap rather than
 * losing the transfer to kIOReturnIsoTooOld forever.
 */
#define SCHED_MARGIN 4
static UInt64 schedule_frame(struct stream *s, UInt64 want)
{
	UInt64 cur = 0;
	AbsoluteTime at;
	(*s->usb->intf)->GetBusFrameNumber(s->usb->intf, &cur, &at);
	if (want < cur + SCHED_MARGIN)
		want = cur + SCHED_MARGIN;
	return want;
}

static void in_done(void *refcon, IOReturn result, void *arg0)
{
	struct iso_xfer *x = refcon;
	struct stream *s = x->s;
	(void)arg0;

	if (!s->running)
		return;

	s->in_count++;

	if (s->dbg > 0) {
		printf("[in #%d] result=0x%08x frames:", x->index, result);
		for (int f = 0; f < FRAMES_PER_XFER; f++)
			printf(" %u/%u", x->frames[f].frActCount,
			       x->frames[f].frReqCount);
		printf("\n");
	}

	uint8_t scratch[XFER_BUF];
	for (int f = 0; f < FRAMES_PER_XFER; f++) {
		int act = x->frames[f].frActCount;
		if (act <= 0)
			continue;
		int pos = 0;
		caiaq_capture_mode2(&s->codec, x->buf + f * FRAME_BYTES, act,
				    scratch, &pos, sizeof(scratch));
		if (pos > 0)
			cap_append(s, scratch, pos);
	}

	/* record this capture's per-microframe cadence for playback to replay */
	if (s->do_play)
		pat_push(s, x);

	if (s->dbg > 0)
		s->dbg--;

	resubmit_in(x);
}

/*
 * Fill one OUT transfer with a CONSTANT, gapless cadence: every microframe
 * carries rate/8000 stereo frames (plus an occasional extra via the rate
 * accumulator, so the long-run rate is exact). This is what the device's OUT
 * endpoint wants on every microframe — no zero-length frames, no dependence on
 * the bursty IN cadence. caiaq_playback_mode2 lays out audio + check bytes.
 */
static void fill_out(struct stream *s, struct iso_xfer *x)
{
	int lens[FRAMES_PER_XFER];
	if (s->const_cad)
		pat_const(s, lens);   /* steady N-frames/microframe (no zeros) */
	else
		pat_pop(s, lens);     /* exact captured cadence: shape + rate-locked */
	/*
	 * The pipe rejects a transfer ending in 0-length frame(s). The captured
	 * cadence often ends in zeros, so move the last non-zero packet to the final
	 * slot (preserves the total byte count; interior/leading zeros are fine).
	 */
	if (lens[FRAMES_PER_XFER - 1] == 0) {
		int j = FRAMES_PER_XFER - 2;
		while (j >= 0 && lens[j] == 0)
			j--;
		if (j >= 0) {
			lens[FRAMES_PER_XFER - 1] = lens[j];
			lens[j] = 0;
		}
	}
	/*
	 * IOKit isoc packs frame data CONTIGUOUSLY by frReqCount, so microframe f's
	 * bytes go at the running offset, NOT f*FRAME_BYTES. (Capture gets away with
	 * f*FRAME_BYTES only because every capture frReqCount == FRAME_BYTES.)
	 */
	int off = 0;
	for (int f = 0; f < FRAMES_PER_XFER; f++) {
		int len = lens[f];
		if (len > FRAME_BYTES) len = FRAME_BYTES;
		s->len_hist[len]++;
		x->frames[f].frReqCount = len;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
		if (len > 0)
			caiaq_playback_mode2(&s->codec, x->buf + off, len,
					     s->tone, &s->tone_pos, (int)s->tone_size);
		off += len;
	}
}

/*
 * Submit an already-filled OUT transfer at x->frame_start, SELF-HEALING: if the
 * pipe rejects the frame (kIOReturnBadArgument when the slot isn't acceptable),
 * bump the start ahead of the current bus frame and retry. A submit that never
 * succeeds would silently drop the buffer out of the chain (no completion
 * callback fires), so retrying here is what keeps the chain alive. The data is
 * NOT re-filled on retry, so the cadence queue is popped exactly once per call.
 */
static int submit_out(struct iso_xfer *x)
{
	struct stream *s = x->s;
	for (int tries = 0; tries < 8; tries++) {
		IOReturn kr = (*s->usb->intf)->WriteIsochPipeAsync(s->usb->intf,
				s->usb->pipe_iso_out, x->buf, x->frame_start,
				FRAMES_PER_XFER, x->frames, out_done, x);
		if (kr == kIOReturnSuccess)
			return 0;
		s->out_errs++;
		UInt64 cur = 0;
		AbsoluteTime at;
		(*s->usb->intf)->GetBusFrameNumber(s->usb->intf, &cur, &at);
		UInt64 floor = cur + OUT_MIN_LEAD;
		x->frame_start = (x->frame_start < floor)
				 ? floor : x->frame_start + FRAME_STRIDE;
	}
	s->out_drops++;   /* gave up: this slot leaves the chain */
	return -1;
}

/*
 * Free-running OUT chain (same discipline as capture's resubmit_in): on each
 * completion advance frame_start by N_OUT_XFERS*FRAME_STRIDE, refill with the
 * next cadence pattern, and resubmit.
 */
static void out_done(void *refcon, IOReturn result, void *arg0)
{
	struct iso_xfer *x = refcon;
	struct stream *s = x->s;
	(void)arg0;
	s->out_done_count++;
	if (!s->running)
		return;
	if (s->out_dbg > 0) {
		s->out_dbg--;
		printf("[out #%d] result=0x%08x frame=%llu\n",
		       x->index, result, x->frame_start);
	}
	x->frame_start += (UInt64)N_OUT_XFERS * FRAME_STRIDE;
	fill_out(s, x);
	submit_out(x);
}

static void resubmit_in(struct iso_xfer *x)
{
	struct stream *s = x->s;
	x->frame_start = schedule_frame(s, x->frame_start + N_XFERS * FRAME_STRIDE);
	for (int f = 0; f < FRAMES_PER_XFER; f++) {
		x->frames[f].frReqCount = FRAME_BYTES;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
	}
	IOReturn kr = (*s->usb->intf)->ReadIsochPipeAsync(s->usb->intf,
			s->usb->pipe_iso_in, x->buf, x->frame_start,
			FRAMES_PER_XFER, x->frames, in_done, x);
	if (kr != kIOReturnSuccess && s->running) {
		s->in_errs++;
		/* re-anchor hard and retry once */
		x->frame_start = schedule_frame(s, 0);
		(*s->usb->intf)->ReadIsochPipeAsync(s->usb->intf,
			s->usb->pipe_iso_in, x->buf, x->frame_start,
			FRAMES_PER_XFER, x->frames, in_done, x);
	}
}

/* ---- tone generation ------------------------------------------------ */

static void make_tone(struct stream *s)
{
	int frames = s->rate;                   /* 1 second, loops */
	s->tone_size = (size_t)frames * 2 * 3;  /* stereo S24_3BE */
	s->tone = malloc(s->tone_size);
	/*
	 * The device expects the OUT audio stream at a 4-byte (BYTES_PER_SAMPLE+1)
	 * phase: Linux seeds audio_out_buf_pos = out_pos = BYTES_PER_SAMPLE+1 in
	 * snd_usb_caiaq_pcm_prepare. Without this offset every sample is byte-rotated
	 * and a real signal plays as a distorted buzz (verified: --off 0 buzzes,
	 * --off 4 is clean). --off overrides it for experiments.
	 */
	s->tone_pos = (int)(((size_t)s->off) % s->tone_size);
	for (int n = 0; n < frames; n++) {
		double t = (double)n / s->rate;
		int32_t v = s->silence ? 0 :
			(int32_t)(0.2 * 8388607.0 * sin(2 * M_PI * s->freq * t));
		for (int ch = 0; ch < 2; ch++) {
			size_t o = ((size_t)n * 2 + ch) * 3;
			s->tone[o + 0] = (v >> 16) & 0xff; /* big-endian */
			s->tone[o + 1] = (v >> 8) & 0xff;
			s->tone[o + 2] = v & 0xff;
		}
	}
}

/* ---- stop timer ----------------------------------------------------- */

static void stop_cb(CFRunLoopTimerRef t, void *info)
{
	struct stream *s = info;
	(void)t;
	s->running = 0;
	CFRunLoopStop(CFRunLoopGetCurrent());
}

int main(int argc, char **argv)
{
	struct rk3_usb u;
	struct stream s;
	struct caiaq_device_spec spec;
	double seconds = 2.0;

	memset(&s, 0, sizeof(s));
	s.rate = 48000;
	s.freq = 440.0;
	s.off = 4;          /* Linux out_pos = BYTES_PER_SAMPLE + 1; OUT stream phase */
	s.dbg = 0;
	s.out_dbg = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--play")) s.do_play = 1;
		else if (!strcmp(argv[i], "--chk0")) s.pb_chk0 = 1;
		else if (!strcmp(argv[i], "--const")) s.const_cad = 1;
		else if (!strcmp(argv[i], "--off") && i + 1 < argc) s.off = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--silence")) { s.do_play = 1; s.silence = 1; }
		else if (!strcmp(argv[i], "--freq") && i + 1 < argc) s.freq = atof(argv[++i]);
		else if (!strcmp(argv[i], "--rate") && i + 1 < argc) s.rate = atoi(argv[++i]);
		else seconds = atof(argv[i]);
	}

	if (rk3_usb_open(&u) != 0)
		return 1;
	s.usb = &u;

	if (caiaq_get_device_info(&u, &spec) != 0) { rk3_usb_close(&u); return 1; }
	int n_streams = spec.num_analog_audio_in / 2;
	if (n_streams < 1) n_streams = 1;
	s.bpp = caiaq_compute_bpp(s.rate, spec.data_alignment, n_streams);

	printf("rate=%d bpp=%d data_alignment=%u  capture%s for %.1fs\n",
	       s.rate, s.bpp, spec.data_alignment,
	       s.do_play ? "+playback" : "", seconds);

	if (caiaq_set_audio_params(&u, s.rate, 24, s.bpp) != 0) {
		rk3_usb_close(&u);
		return 1;
	}

	caiaq_codec_reset(&s.codec);
	if (s.pb_chk0)
		s.codec.pb_chk = 0;   /* experiment: capture-style OUT layout */
	if (s.do_play)
		make_tone(&s);

	/* async event source on this run loop */
	CFRunLoopSourceRef src = NULL;
	IOReturn kr = (*u.intf)->CreateInterfaceAsyncEventSource(u.intf, &src);
	if (kr != kIOReturnSuccess || !src) {
		fprintf(stderr, "CreateInterfaceAsyncEventSource 0x%08x\n", kr);
		rk3_usb_close(&u);
		return 1;
	}
	CFRunLoopAddSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);

	UInt64 frame = 0;
	AbsoluteTime atTime;
	(*u.intf)->GetBusFrameNumber(u.intf, &frame, &atTime);
	UInt64 start = frame + 10; /* a few frames in the future */
	printf("bus frame %llu, scheduling from %llu\n", frame, start);

	s.running = 1;

	/* prime capture transfers */
	for (int i = 0; i < N_XFERS; i++) {
		s.in[i].s = &s;
		s.in[i].index = i;
		s.in[i].frame_start = start + (UInt64)i * FRAME_STRIDE;
		for (int f = 0; f < FRAMES_PER_XFER; f++)
			s.in[i].frames[f].frReqCount = FRAME_BYTES;
		kr = (*u.intf)->ReadIsochPipeAsync(u.intf, u.pipe_iso_in,
				s.in[i].buf, s.in[i].frame_start,
				FRAMES_PER_XFER, s.in[i].frames, in_done, &s.in[i]);
		if (kr != kIOReturnSuccess)
			fprintf(stderr, "ReadIsochPipeAsync(prime %d) 0x%08x\n", i, kr);
	}

	/* Prime the free-running OUT chain at startup (reliable scheduling — the
	 * pipe rejects mid-stream bursts). It starts on synthetic bursty cadence
	 * and switches to the exact captured cadence as the queue fills. */
	if (s.do_play) {
		UInt64 out_start = start;
		for (int i = 0; i < N_OUT_XFERS; i++) {
			s.out[i].s = &s;
			s.out[i].index = i;
			s.out[i].frame_start = out_start + (UInt64)i * FRAME_STRIDE;
			fill_out(&s, &s.out[i]);
			if (submit_out(&s.out[i]) != 0)
				fprintf(stderr, "submit_out(prime %d) failed\n", i);
		}
	}

	/* stop timer */
	CFRunLoopTimerContext ctx = { 0, &s, NULL, NULL, NULL };
	CFRunLoopTimerRef timer = CFRunLoopTimerCreate(NULL,
			CFAbsoluteTimeGetCurrent() + seconds, 0, 0, 0,
			stop_cb, &ctx);
	CFRunLoopAddTimer(CFRunLoopGetCurrent(), timer, kCFRunLoopDefaultMode);

	CFRunLoopRun();

	/* teardown */
	(*u.intf)->AbortPipe(u.intf, u.pipe_iso_in);
	if (s.do_play)
		(*u.intf)->AbortPipe(u.intf, u.pipe_iso_out);

	printf("captured %zu bytes (%zu stereo frames), input_panic=%d output_panic=%d, "
	       "resched in=%d out=%d\n",
	       s.cap_len, s.cap_len / 6, s.codec.input_panic, s.codec.output_panic,
	       s.in_errs, s.out_errs);
	if (s.do_play)
		printf("in_completions=%d out_completed=%d out_retries=%d out_drops=%d "
		       "out_underruns=%d\n",
		       s.in_count, s.out_done_count, s.out_errs, s.out_drops,
		       s.out_underruns);

	if (s.do_play) {
		/* OUT byte-rate audit: how many audio frames/sec did we actually
		 * send vs the device's DAC rate? (overfeed -> pitch sharp + drops) */
		long long out_audio_bytes = 0, out_pkts = 0;
		for (int L = 0; L <= FRAME_BYTES; L++) {
			if (!s.len_hist[L]) continue;
			int chk = (L + 1) / 4;                 /* check bytes in an L-byte pkt */
			out_audio_bytes += (long long)(L - chk) * s.len_hist[L];
			out_pkts += s.len_hist[L];
			printf("  len=%d  count=%d  (audio=%d B/pkt)\n",
			       L, s.len_hist[L], L - chk);
		}
		/* actual streamed duration = microframes sent / 8000 (not wall-clock). */
		double dur = out_pkts / 8000.0;
		printf("OUT audio: %lld bytes => %.1f stereo frames/s "
		       "(target %d; %+.2f%%), %lld pkts over %.2fs\n",
		       out_audio_bytes, out_audio_bytes / 6.0 / dur, s.rate,
		       (out_audio_bytes / 6.0 / dur / s.rate - 1.0) * 100.0,
		       out_pkts, dur);
	}

	if (s.cap_len > 0)
		write_wav("capture.wav", s.cap, s.cap_len, s.rate);
	free(s.cap);
	free(s.tone);
	rk3_usb_close(&u);
	return 0;
}
