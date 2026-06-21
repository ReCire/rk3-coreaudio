/* rk3_engine.c — see rk3_engine.h */

#include "rk3_engine.h"

#include <os/log.h>
#include <string.h>
#include <mach/mach_time.h>
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>

static os_log_t eng_log(void)
{
	static os_log_t l;
	static dispatch_once_t once;
	dispatch_once(&once, ^{ l = os_log_create("com.mihai.rk3", "engine"); });
	return l;
}
#define ELOG(fmt, ...) os_log(eng_log(), "[rk3] " fmt, ##__VA_ARGS__)

/* ---- ring helpers (single producer + single consumer per ring) ------- */

static inline unsigned ring_avail(unsigned w, unsigned r)
{
	return (w - r + RK3_RING_BYTES) % RK3_RING_BYTES;   /* bytes readable */
}
static inline unsigned ring_space(unsigned w, unsigned r)
{
	/* keep one byte free so full != empty */
	return RK3_RING_BYTES - 1 - ring_avail(w, r);
}

/* ---- cadence mirror (engine-thread only) ----------------------------- */

static int pat_count(struct rk3_engine *e) { return e->pat_head - e->pat_tail; }

static void pat_push(struct rk3_engine *e, struct rk3_iso_xfer *x)
{
	if (pat_count(e) >= RK3_PAT_RING)
		e->pat_tail++;
	int *slot = e->pat[e->pat_head % RK3_PAT_RING];
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
		slot[f] = x->frames[f].frActCount;
	e->pat_head++;
}

/* Synthesize the device's bursty 88/0 cadence for one transfer (startup). */
static void pat_synth(struct rk3_engine *e, int *lens)
{
	int pkt_frames = e->bpp / 8;
	int need = e->rate / 8000;
	int rem  = e->rate % 8000;
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		e->out_acc += need * 8000 + rem;
		if (e->out_acc >= pkt_frames * 8000) {
			e->out_acc -= pkt_frames * 8000;
			lens[f] = e->bpp;
		} else {
			lens[f] = 0;
		}
	}
}

static void pat_pop(struct rk3_engine *e, int *lens)
{
	if (pat_count(e) > 0) {
		int *slot = e->pat[e->pat_tail % RK3_PAT_RING];
		for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
			lens[f] = slot[f];
		e->pat_tail++;
	} else {
		pat_synth(e, lens);
	}
}

/*
 * Pop one OUT transfer's worth (RK3_OUT_FRAMES_PER_XFER microframes) of cadence,
 * refilling the IN-sized staging block as it drains. RK3_OUT_FRAMES_PER_XFER
 * divides RK3_FRAMES_PER_XFER, so each captured IN block feeds an exact integer
 * number of OUT transfers — the OUT chain stays rate-locked to the device clock
 * even though it ships smaller transfers than capture.
 */
static void pat_pop_out(struct rk3_engine *e, int *lens)
{
	if (e->pat_stage_off >= RK3_FRAMES_PER_XFER) {
		pat_pop(e, e->pat_stage);
		e->pat_stage_off = 0;
	}
	for (int f = 0; f < RK3_OUT_FRAMES_PER_XFER; f++)
		lens[f] = e->pat_stage[e->pat_stage_off + f];
	e->pat_stage_off += RK3_OUT_FRAMES_PER_XFER;
}

/* ---- isoc completion callbacks (ported from stream.c) ---------------- */

static void resubmit_in(struct rk3_iso_xfer *x);
static int  submit_out(struct rk3_iso_xfer *x);
static void fill_out(struct rk3_engine *e, struct rk3_iso_xfer *x);
static void out_done(void *refcon, IOReturn result, void *arg0);

#define SCHED_MARGIN 4
static UInt64 schedule_frame(struct rk3_engine *e, UInt64 want)
{
	UInt64 cur = 0;
	AbsoluteTime at;
	(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &cur, &at);
	if (want < cur + SCHED_MARGIN)
		want = cur + SCHED_MARGIN;
	return want;
}

static void in_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)result; (void)arg0;

	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;

	/* Deinterleave each microframe into a scratch then push to the cap ring. */
	uint8_t scratch[RK3_XFER_BUF];
	int pos = 0;
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		int act = x->frames[f].frActCount;
		if (act <= 0)
			continue;
		caiaq_capture_mode2(&e->codec, x->buf + f * RK3_FRAME_BYTES, act,
				    scratch, &pos, sizeof(scratch));
	}
	if (pos > 0) {
		/* device clock: count frames actually produced, even if the ring drops them */
		atomic_fetch_add_explicit(&e->dev_frames,
				(unsigned)(pos / RK3_BYTES_PER_FRAME), memory_order_relaxed);
		unsigned w = atomic_load_explicit(&e->cap_w, memory_order_relaxed);
		unsigned r = atomic_load_explicit(&e->cap_r, memory_order_acquire);
		if (ring_space(w, r) < (unsigned)pos) {
			atomic_fetch_add_explicit(&e->cap_overruns, 1,
						  memory_order_relaxed);
		} else {
			unsigned first = RK3_RING_BYTES - w;
			if (first > (unsigned)pos) first = pos;
			memcpy(e->cap_buf + w, scratch, first);
			if ((unsigned)pos > first)
				memcpy(e->cap_buf, scratch + first, pos - first);
			atomic_store_explicit(&e->cap_w,
				(w + pos) % RK3_RING_BYTES, memory_order_release);
		}
	}

	/* mirror this capture's per-microframe cadence for playback */
	pat_push(e, x);

	resubmit_in(x);
}

/*
 * Fill one OUT transfer from the playback ring. Pops the captured cadence
 * (rate-locked to the device), guarantees a non-zero trailing frame, and packs
 * the audio CONTIGUOUSLY by frReqCount via caiaq_playback_mode2.
 *
 * Startup gate: until CoreAudio has buffered RK3_OUT_CUSHION (pb_started, set in
 * write_output), the engine sends digital silence and does NOT touch pb_r. Once
 * started, pb_r is read from the ring at RK3_OUT_PHASE (set in write_output) and
 * advances continuously, preserving the proven 4-byte OUT phase across packets.
 */
static void fill_out(struct rk3_engine *e, struct rk3_iso_xfer *x)
{
	int lens[RK3_OUT_FRAMES_PER_XFER];
	pat_pop_out(e, lens);

	if (lens[RK3_OUT_FRAMES_PER_XFER - 1] == 0) {
		int j = RK3_OUT_FRAMES_PER_XFER - 2;
		while (j >= 0 && lens[j] == 0)
			j--;
		if (j >= 0) {
			lens[RK3_OUT_FRAMES_PER_XFER - 1] = lens[j];
			lens[j] = 0;
		}
	}

	int started = atomic_load_explicit(&e->pb_started, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned avail = started ? ring_avail(w, r) : 0;

	int off = 0;
	for (int f = 0; f < RK3_OUT_FRAMES_PER_XFER; f++) {
		int len = lens[f];
		if (len > RK3_FRAME_BYTES) len = RK3_FRAME_BYTES;
		x->frames[f].frReqCount = len;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
		if (len > 0) {
			int audio = len - (len + 1) / 4;   /* audio bytes in pkt */
			if (avail < (unsigned)audio) {
				/* not started yet, or underrun: emit silence */
				if (started)
					atomic_fetch_add_explicit(&e->pb_underruns,
							1, memory_order_relaxed);
				caiaq_playback_mode2(&e->codec, x->buf + off, len,
						     NULL, NULL, 0);
			} else {
				caiaq_playback_mode2(&e->codec, x->buf + off, len,
						     e->pb_buf, (int *)&r,
						     RK3_RING_BYTES);
				avail -= audio;
			}
		}
		off += len;
	}
	if (started) {
		atomic_store_explicit(&e->pb_r, r, memory_order_release);
		/* windowed-min out_fill telemetry: post-consume trough (this is the
		 * sample right after the OUT bite, i.e. closest to underrun). */
		unsigned wf = atomic_load_explicit(&e->pb_w, memory_order_relaxed);
		unsigned of = ring_avail(wf, r) / RK3_BYTES_PER_FRAME;
		if (of < e->out_fill_min) e->out_fill_min = of;
	}
}

static int submit_out(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	for (int tries = 0; tries < 8; tries++) {
		IOReturn kr = (*e->usb.intf)->WriteIsochPipeAsync(e->usb.intf,
				e->usb.pipe_iso_out, x->buf, x->frame_start,
				RK3_OUT_FRAMES_PER_XFER, x->frames, out_done, x);
		if (kr == kIOReturnSuccess)
			return 0;
		e->out_errs++;
		UInt64 cur = 0;
		AbsoluteTime at;
		(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &cur, &at);
		UInt64 floor = cur + RK3_OUT_MIN_LEAD;
		x->frame_start = (x->frame_start < floor)
				 ? floor : x->frame_start + RK3_OUT_FRAME_STRIDE;
	}
	e->out_drops++;
	return -1;
}

static void out_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)result; (void)arg0;
	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;
	x->frame_start += (UInt64)RK3_N_OUT_XFERS * RK3_OUT_FRAME_STRIDE;
	fill_out(e, x);
	submit_out(x);

	/*
	 * Drift-vs-jitter probe. out_done fires once per OUT transfer (every
	 * RK3_OUT_FRAME_STRIDE ms-frame = 8000/RK3_OUT_FRAMES_PER_XFER per sec)
	 * regardless of N_OUT_XFERS, so tick every (8000/OUT_FPX) calls ≈ 1 second. Log
	 * the OUT ring fill, the CAPTURE ring fill (so we catch a silent input-latency
	 * balloon — the IN ring growing if CoreAudio drains it slower than the device
	 * fills it), and the CUMULATIVE measured device rate from a FIXED anchor
	 * (converges as the run lengthens). Monotonic out_fill decline => drift; noisy
	 * around a stable mean with underruns on dips => jitter/margin.
	 */
	if (e->dbg_host0 == 0) {
		e->dbg_host0 = mach_absolute_time();
		e->dbg_devframes0 = atomic_load_explicit(&e->dev_frames, memory_order_relaxed);
	}
	if (++e->dbg_ctr >= (unsigned)(8000 / RK3_OUT_FRAMES_PER_XFER)) {
		e->dbg_ctr = 0;
		static mach_timebase_info_data_t tb;
		if (tb.denom == 0) mach_timebase_info(&tb);
		UInt64 now = mach_absolute_time();
		double elapsed_s = (double)(now - e->dbg_host0) * tb.numer / tb.denom / 1e9;
		unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
		unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
		unsigned fill = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
		unsigned cw = atomic_load_explicit(&e->cap_w, memory_order_acquire);
		unsigned cr = atomic_load_explicit(&e->cap_r, memory_order_relaxed);
		unsigned capfill = ring_avail(cw, cr) / RK3_BYTES_PER_FRAME;
		unsigned long long df = atomic_load_explicit(&e->dev_frames, memory_order_relaxed);
		double rate = elapsed_s > 0 ?
			(double)(df - e->dbg_devframes0) / elapsed_s : 0.0;
		unsigned ofmin = e->out_fill_min, cfmin = e->cap_fill_min;
		e->out_fill_min = ~0u;
		e->cap_fill_min = ~0u;
		ELOG("clk: out_fill=%u (min %u) cap_fill=%u (min %u)  dev_rate=%.2f Hz (nominal %d)  underruns=%u",
		     fill, ofmin == ~0u ? 0 : ofmin, capfill, cfmin == ~0u ? 0 : cfmin,
		     rate, e->rate, atomic_load(&e->pb_underruns));
	}
}

static void resubmit_in(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	x->frame_start = schedule_frame(e, x->frame_start + RK3_N_XFERS * RK3_FRAME_STRIDE);
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		x->frames[f].frReqCount = RK3_FRAME_BYTES;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
	}
	IOReturn kr = (*e->usb.intf)->ReadIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_in, x->buf, x->frame_start,
			RK3_FRAMES_PER_XFER, x->frames, in_done, x);
	if (kr != kIOReturnSuccess &&
	    atomic_load_explicit(&e->running, memory_order_acquire)) {
		e->in_errs++;
		x->frame_start = schedule_frame(e, 0);
		(*e->usb.intf)->ReadIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_in, x->buf, x->frame_start,
			RK3_FRAMES_PER_XFER, x->frames, in_done, x);
	}
}

/* ---- engine thread --------------------------------------------------- */

static void prime_chains(struct rk3_engine *e)
{
	UInt64 frame = 0;
	AbsoluteTime atTime;
	(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &frame, &atTime);
	UInt64 start = frame + 10;

	for (int i = 0; i < RK3_N_XFERS; i++) {
		e->in[i].e = e;
		e->in[i].index = i;
		e->in[i].frame_start = start + (UInt64)i * RK3_FRAME_STRIDE;
		for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
			e->in[i].frames[f].frReqCount = RK3_FRAME_BYTES;
		IOReturn kr = (*e->usb.intf)->ReadIsochPipeAsync(e->usb.intf,
				e->usb.pipe_iso_in, e->in[i].buf,
				e->in[i].frame_start, RK3_FRAMES_PER_XFER,
				e->in[i].frames, in_done, &e->in[i]);
		if (kr != kIOReturnSuccess)
			ELOG("prime IN %d: 0x%08x", i, kr);
	}

	for (int i = 0; i < RK3_N_OUT_XFERS; i++) {
		e->out[i].e = e;
		e->out[i].index = i;
		e->out[i].frame_start = start + (UInt64)i * RK3_OUT_FRAME_STRIDE;
		fill_out(e, &e->out[i]);
		if (submit_out(&e->out[i]) != 0)
			ELOG("prime OUT %d failed", i);
	}
}

/*
 * Promote the engine's USB/CFRunLoop thread to time-constraint (real-time)
 * scheduling so isoc completion callbacks are serviced on time even under DAW
 * load. Insurance for the smaller OUT transfers (higher completion rate): the
 * storm risk is a default-priority thread falling behind the completion stream.
 * preemptible=1 so we still yield to CoreAudio's own real-time IO thread.
 * Best-effort; a failure here is non-fatal (logged, engine runs default-sched).
 */
static void engine_set_realtime(void)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	double abs_per_ms = 1e6 * (double)tb.denom / (double)tb.numer;
	thread_time_constraint_policy_data_t pol;
	pol.period      = 0;                              /* aperiodic (event-driven) */
	pol.computation = (uint32_t)(0.5 * abs_per_ms);  /* ~0.5 ms to service a cb */
	pol.constraint  = (uint32_t)(2.0 * abs_per_ms);  /* deadline ~2 ms */
	pol.preemptible = 1;
	kern_return_t kr = thread_policy_set(mach_thread_self(),
			THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
			THREAD_TIME_CONSTRAINT_POLICY_COUNT);
	if (kr != KERN_SUCCESS)
		ELOG("thread_policy_set RT failed: 0x%x", kr);
}

static void *engine_thread(void *arg)
{
	struct rk3_engine *e = arg;
	e->runloop = CFRunLoopGetCurrent();
	engine_set_realtime();

	CFRunLoopSourceRef src = NULL;
	IOReturn kr = (*e->usb.intf)->CreateInterfaceAsyncEventSource(e->usb.intf, &src);
	if (kr != kIOReturnSuccess || !src) {
		ELOG("CreateInterfaceAsyncEventSource 0x%08x", kr);
		e->start_rc = -1;
		atomic_store_explicit(&e->thread_ready, 1, memory_order_release);
		return NULL;
	}
	CFRunLoopAddSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);

	atomic_store_explicit(&e->running, 1, memory_order_release);
	prime_chains(e);

	e->start_rc = 0;
	atomic_store_explicit(&e->thread_ready, 1, memory_order_release);

	ELOG("engine running (rate=%d bpp=%d)", e->rate, e->bpp);
	CFRunLoopRun();   /* until rk3_engine_stop() calls CFRunLoopStop */

	/* teardown on the engine thread */
	(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_in);
	(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_out);
	CFRunLoopRemoveSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);
	CFRelease(src);
	ELOG("engine stopped (cap_overruns=%u pb_underruns=%u pb_overruns=%u "
	     "in_errs=%d out_errs=%d)",
	     atomic_load(&e->cap_overruns), atomic_load(&e->pb_underruns),
	     atomic_load(&e->pb_overruns), e->in_errs, e->out_errs);
	return NULL;
}

/* ---- public API ------------------------------------------------------ */

int rk3_engine_start(struct rk3_engine *e, int rate)
{
	struct caiaq_device_spec spec;

	/* preserve the caller-zeroed rings/diagnostics; (re)init transient state */
	atomic_store(&e->running, 0);
	atomic_store(&e->thread_ready, 0);
	e->rate = rate;
	e->pat_head = e->pat_tail = e->out_acc = 0;
	e->pat_stage_off = RK3_FRAMES_PER_XFER;   /* force a pop on first OUT fill */
	e->in_errs = e->out_errs = e->out_drops = 0;

	if (rk3_usb_open(&e->usb) != 0)
		return -1;

	if (caiaq_get_device_info(&e->usb, &spec) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}
	int n_streams = spec.num_analog_audio_in / 2;
	if (n_streams < 1) n_streams = 1;
	e->bpp = caiaq_compute_bpp(rate, spec.data_alignment, n_streams);

	if (caiaq_set_audio_params(&e->usb, rate, 24, e->bpp) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}

	caiaq_codec_reset(&e->codec);

	/*
	 * Capture ring empty. Playback ring empty + ungated: the engine sends
	 * silence on OUT until write_output has buffered RK3_OUT_CUSHION, then it
	 * sets pb_r = RK3_OUT_PHASE and pb_started (preserving the 4-byte phase).
	 * pb_w/pb_r both start at 0 (a frame boundary) so CoreAudio's audio is
	 * frame-aligned from ring byte 0.
	 */
	atomic_store(&e->cap_w, 0);
	atomic_store(&e->cap_r, 0);
	atomic_store(&e->cap_primed, 0);
	e->cap_settle_left = RK3_CAP_SETTLE_FRAMES;
	e->cap_measure_left = RK3_CAP_MEASURE_FRAMES;
	e->cap_meas_min = ~0u;
	e->out_fill_min = ~0u;
	e->cap_fill_min = ~0u;
	atomic_store(&e->pb_w, 0);
	atomic_store(&e->pb_r, 0);
	atomic_store(&e->pb_started, 0);
	atomic_store(&e->cap_overruns, 0);
	atomic_store(&e->pb_underruns, 0);
	atomic_store(&e->pb_overruns, 0);

	if (pthread_create(&e->thread, NULL, engine_thread, e) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}

	/* wait for the thread to prime the chains (or fail) */
	while (!atomic_load_explicit(&e->thread_ready, memory_order_acquire))
		usleep(1000);

	if (e->start_rc != 0) {
		rk3_engine_stop(e);
		return -1;
	}
	return 0;
}

void rk3_engine_stop(struct rk3_engine *e)
{
	if (!atomic_load(&e->running) && !e->runloop)
		return;
	atomic_store_explicit(&e->running, 0, memory_order_release);
	if (e->runloop) {
		CFRunLoopStop(e->runloop);
		pthread_join(e->thread, NULL);
		e->runloop = NULL;
	}
	rk3_usb_close(&e->usb);
}

/* ---- Float32 <-> S24_3BE bridges (CoreAudio IO thread) --------------- */

static inline float s24be_to_f32(const uint8_t *p)
{
	int32_t v = (int32_t)((uint32_t)p[0] << 24 |
			      (uint32_t)p[1] << 16 |
			      (uint32_t)p[2] << 8);   /* sign in MSB */
	return (float)(v >> 8) / 8388608.0f;
}
static inline void f32_to_s24be(float f, uint8_t *p)
{
	if (f > 1.0f) f = 1.0f;
	if (f < -1.0f) f = -1.0f;
	int32_t v = (int32_t)(f * 8388607.0f);
	p[0] = (uint8_t)(v >> 16);
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)v;
}

void rk3_engine_read_input(struct rk3_engine *e, float *dst, int nframes)
{
	int need = nframes * RK3_BYTES_PER_FRAME;
	unsigned r = atomic_load_explicit(&e->cap_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->cap_w, memory_order_acquire);

	/*
	 * One-time startup capture drain. The capture standing fill is an uncontrolled
	 * startup offset (~490 frames ≈ 10 ms of hidden input latency) that the device's
	 * initial clock transient builds up over the first ~1 s — so we cannot drain at
	 * pb_started (too early, fill not yet established). Wait RK3_CAP_SETTLE_FRAMES
	 * past pb_started, then collapse the fill to RK3_CAP_FILL_SETPOINT once. The
	 * dropped amount is a frame multiple, so cap_r stays frame-aligned. Runs exactly
	 * once (cap_primed). read_input is the sole cap_r writer, so this is SPSC-safe.
	 */
	if (!atomic_load_explicit(&e->cap_primed, memory_order_relaxed) &&
	    atomic_load_explicit(&e->pb_started, memory_order_acquire)) {
		if (e->cap_settle_left > 0) {
			e->cap_settle_left -= nframes;   /* wait out the IN startup transient */
		} else if (e->cap_measure_left > 0) {
			unsigned hf = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
			if (hf < e->cap_meas_min) e->cap_meas_min = hf;  /* find the trough */
			e->cap_measure_left -= nframes;
		} else {
			if (e->cap_meas_min != ~0u && e->cap_meas_min > RK3_CAP_FILL_FLOOR) {
				unsigned drop = (e->cap_meas_min - RK3_CAP_FILL_FLOOR)
						* RK3_BYTES_PER_FRAME;
				r = (r + drop) % RK3_RING_BYTES;   /* trough -> floor */
			}
			atomic_store_explicit(&e->cap_primed, 1, memory_order_relaxed);
		}
	}

	unsigned avail = ring_avail(w, r);
	{                                   /* windowed-min cap_fill telemetry */
		unsigned af = avail / RK3_BYTES_PER_FRAME;
		if (af < e->cap_fill_min) e->cap_fill_min = af;
	}
	int got = (avail < (unsigned)need) ? (int)avail : need;
	got -= got % RK3_BYTES_PER_FRAME;

	int i = 0;
	for (; i < got / RK3_BYTES_PER_FRAME; i++) {
		dst[i * 2 + 0] = s24be_to_f32(&e->cap_buf[r]); r = (r + 3) % RK3_RING_BYTES;
		dst[i * 2 + 1] = s24be_to_f32(&e->cap_buf[r]); r = (r + 3) % RK3_RING_BYTES;
	}
	atomic_store_explicit(&e->cap_r, r, memory_order_release);
	for (; i < nframes; i++) {          /* zero-fill underrun */
		dst[i * 2 + 0] = 0.0f;
		dst[i * 2 + 1] = 0.0f;
	}
}

void rk3_engine_write_output(struct rk3_engine *e, const float *src, int nframes)
{
	int want = nframes * RK3_BYTES_PER_FRAME;
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_relaxed);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_acquire);
	if (ring_space(w, r) < (unsigned)want) {
		atomic_fetch_add_explicit(&e->pb_overruns, 1, memory_order_relaxed);
		return;                      /* ring full: drop this block */
	}
	for (int i = 0; i < nframes; i++) {
		f32_to_s24be(src[i * 2 + 0], &e->pb_buf[w]); w = (w + 3) % RK3_RING_BYTES;
		f32_to_s24be(src[i * 2 + 1], &e->pb_buf[w]); w = (w + 3) % RK3_RING_BYTES;
	}
	atomic_store_explicit(&e->pb_w, w, memory_order_release);

	/*
	 * Startup gate: once the cushion is buffered, hold the read pointer
	 * RK3_OUT_PHASE bytes in (the proven 4-byte OUT rotation) and release the
	 * engine to start playout. Set pb_r before pb_started (release) so fill_out,
	 * which reads pb_started with acquire, sees the seeded pb_r.
	 */
	if (!atomic_load_explicit(&e->pb_started, memory_order_relaxed) &&
	    w >= (unsigned)RK3_OUT_CUSHION) {
		atomic_store_explicit(&e->pb_r, RK3_OUT_PHASE, memory_order_relaxed);
		atomic_store_explicit(&e->pb_started, 1, memory_order_release);
	}
}

int rk3_engine_out_fill_frames(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->pb_started, memory_order_acquire))
		return -1;
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	return (int)(ring_avail(w, r) / RK3_BYTES_PER_FRAME);
}
