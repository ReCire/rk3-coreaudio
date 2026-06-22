/* rk3_engine.c — see rk3_engine.h */

#include "rk3_engine.h"
#include "rk3_midi.h"          /* RK3_MIDI_UDP_PORT */

#include <os/log.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
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

	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
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
	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
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
	    atomic_load_explicit(&e->audio_running, memory_order_acquire)) {
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

/* ---- MIDI relay: EP 0x81 control reports -> loopback UDP -> helper --- */

static void midi_arm_read(struct rk3_engine *e);

/* Open a UDP socket connected to the helper's loopback port. Best-effort. */
static void midi_relay_open(struct rk3_engine *e)
{
	e->midi_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (e->midi_fd < 0) {
		ELOG("midi relay: socket failed errno=%d (no MIDI this session)", errno);
		return;
	}
	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(RK3_MIDI_UDP_PORT);
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 */
	/* connect() a datagram socket so later sends need no address and a dead
	 * peer surfaces as a send error we can ignore rather than a routing cost. */
	if (connect(e->midi_fd, (struct sockaddr *)&to, sizeof(to)) != 0) {
		ELOG("midi relay: connect failed errno=%d (no MIDI this session)", errno);
		close(e->midi_fd);
		e->midi_fd = -1;
		return;
	}
	ELOG("midi relay: forwarding EP1 control reports to 127.0.0.1:%d", RK3_MIDI_UDP_PORT);
}

/*
 * Number of completions between rate checks, and the minimum time they must
 * span. Normal control traffic is ~100/s (AUTO_MSG analog interval + buttons on
 * change), so RK3_MIDI_CB_WINDOW completions should take seconds; anything that
 * does them in under RK3_MIDI_CB_MIN_MS is a tight re-arm loop, not real input.
 */
#define RK3_MIDI_CB_WINDOW   256
#define RK3_MIDI_CB_MIN_MS   50.0

/* Async completion for the EP 0x81 read: forward the report, then re-arm. */
static void midi_read_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_engine *e = refcon;
	if (!atomic_load_explicit(&e->midi_running, memory_order_acquire))
		return;
	if (result == kIOReturnAborted)
		return;                       /* teardown: stop the chain */

	/*
	 * Device-gone detection. When the RK3 is unplugged, the armed read
	 * completes with a not-attached/not-responding code. Mark the engine USB
	 * dead and stop the chain (no re-arm) so the plugin supervisor can close
	 * and reopen the engine on the next attach. Not a storm-latch: a fresh
	 * open should re-arm normally.
	 */
	if (result == kIOReturnNoDevice || result == kIOReturnNotResponding ||
	    result == kIOReturnNotAttached) {
		ELOG("midi relay: device gone (result=0x%08x) — USB dead, stopping relay",
		     result);
		atomic_store_explicit(&e->usb_dead, 1, memory_order_release);
		atomic_store_explicit(&e->midi_running, 0, memory_order_release);
		return;
	}

	if (result == kIOReturnSuccess) {
		UInt32 n = (UInt32)(uintptr_t)arg0;   /* bytes read */
		if (n > 0 && e->midi_fd >= 0) {
			if (send(e->midi_fd, e->midi_buf, n, 0) < 0 &&
			    e->midi_send_errs < 4) {
				ELOG("midi relay: send errno=%d (helper not running?)", errno);
				e->midi_send_errs++;
			}
		}
	}

	/*
	 * STORM GUARD — DO NOT REMOVE. A self-resubmitting async read must be
	 * protected against tight-looping: if a completion fires immediately and
	 * repeatedly (an immediate error, a pipe stall, or a 0-byte success),
	 * re-arming with no delay pegs this CFRunLoop thread and cascades coreaudiod
	 * to a whole-machine 100% CPU storm (required a reboot, 2026-06-21). MIDI is
	 * best-effort, so if completions outrun a sane control rate we DISABLE the
	 * relay (logging the last result so we can see why) instead of re-arming.
	 * This bounds any pathology to RK3_MIDI_CB_WINDOW cheap iterations.
	 */
	if (++e->midi_cb_ctr >= RK3_MIDI_CB_WINDOW) {
		static mach_timebase_info_data_t tb;
		if (tb.denom == 0)
			mach_timebase_info(&tb);
		UInt64 now = mach_absolute_time();
		double ms = e->midi_cb_t0
			? (double)(now - e->midi_cb_t0) * tb.numer / tb.denom / 1e6
			: 1e9;
		if (ms < RK3_MIDI_CB_MIN_MS) {
			ELOG("midi relay: completion storm (%d cbs in %.2f ms, last "
			     "result=0x%08x) — DISABLING relay to protect audio",
			     RK3_MIDI_CB_WINDOW, ms, result);
			atomic_store_explicit(&e->midi_running, 0, memory_order_release);
			e->midi_storm_tripped = 1;  /* latch: sentinel poller won't re-arm
			                             * until the sentinel is rm'd + re-touched */
			return;               /* no re-arm: relay stays off this session */
		}
		e->midi_cb_t0 = now;
		e->midi_cb_ctr = 0;
	}

	/* Non-abort result within the sane rate: keep reading. */
	midi_arm_read(e);
}

static void midi_arm_read(struct rk3_engine *e)
{
	IOReturn kr = (*e->usb.intf)->ReadPipeAsync(e->usb.intf, e->usb.pipe_cmd_in,
			e->midi_buf, sizeof(e->midi_buf), midi_read_done, e);
	if (kr != kIOReturnSuccess)
		ELOG("midi relay: ReadPipeAsync arm failed 0x%08x", kr);
}

/* Bring the relay up on the engine thread: open the socket, arm the EP1 read,
 * then tell the device to auto-report buttons/pedal. Arm BEFORE AUTO_MSG so we
 * never miss the first report. All steps are non-fatal to audio. The sentinel
 * policy (see midi_sentinel_tick) decides WHEN this runs; it just does the work. */
static void midi_relay_start(struct rk3_engine *e)
{
	midi_relay_open(e);
	e->midi_cb_ctr = 0;
	e->midi_cb_t0 = mach_absolute_time();
	e->midi_send_errs = 0;
	atomic_store_explicit(&e->midi_running, 1, memory_order_release);
	midi_arm_read(e);

	/* snd_usb_caiaq_set_auto_msg: RK3 = (digital=1, analog=10, erp=0). */
	uint8_t am[3] = { 1, 10, 0 };
	if (caiaq_send_command(&e->usb, EP1_CMD_AUTO_MSG, am, sizeof(am)) != 0)
		ELOG("midi relay: AUTO_MSG failed (controls won't auto-report)");
}

static void midi_relay_stop(struct rk3_engine *e)
{
	int was_on = atomic_load_explicit(&e->midi_running, memory_order_acquire);
	atomic_store_explicit(&e->midi_running, 0, memory_order_release);
	if (was_on)
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_cmd_in);
	if (e->midi_fd >= 0) {
		close(e->midi_fd);
		e->midi_fd = -1;
	}
}

/*
 * Opt-in sentinel: the relay runs only while this file exists. The default-
 * installed driver is byte-for-byte the known-good audio path until the user
 * deliberately `touch`es it, and `rm`ing it disarms instantly — both WITHOUT a
 * reinstall, because we poll it live on the engine run loop (below) rather than
 * checking once at startup. coreaudiod (euid 202) can stat a file the user made
 * in /tmp. MIDI is independent of audio: the relay arms as soon as the sentinel
 * appears, whether or not anything is streaming.
 */
#define RK3_MIDI_SENTINEL "/tmp/rk3_midi_on"

/*
 * Poll the sentinel ~1 Hz on the engine run loop and toggle the relay to match.
 * Runs on the engine thread (same thread as midi_read_done), so midi_storm_tripped
 * and the relay calls need no locking. The storm latch is cleared only when the
 * sentinel is absent, so after a storm trip the user must rm + re-touch to re-arm
 * (a pathological device cannot loop the relay back on every second).
 */
static void midi_sentinel_tick(CFRunLoopTimerRef timer, void *info)
{
	(void)timer;
	struct rk3_engine *e = info;
	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;
	int want = (access(RK3_MIDI_SENTINEL, F_OK) == 0);
	int on   = atomic_load_explicit(&e->midi_running, memory_order_acquire);

	if (!want) {
		if (on)
			midi_relay_stop(e);
		e->midi_storm_tripped = 0;     /* sentinel gone: allow a future re-arm */
	} else if (!on && !e->midi_storm_tripped) {
		ELOG("midi relay: %s present — arming foot-controls relay", RK3_MIDI_SENTINEL);
		midi_relay_start(e);
	}
}

/* ---- LED / 7-segment listener: loopback UDP -> EP1_CMD_WRITE_IO ------- */

/* Async WRITE_IO completion: clear the in-flight flag, count / log the result. */
static void led_write_done(void *refcon, IOReturn result, void *arg0)
{
	(void)arg0;
	struct rk3_engine *e = refcon;
	e->led_busy = 0;
	if (result != kIOReturnSuccess) {
		if (e->led_errs < 4) {
			ELOG("led: WRITE_IO async failed 0x%08x (errs=%d)",
			     result, e->led_errs + 1);
			e->led_errs++;
		}
	} else {
		e->led_writes++;
	}
}

/*
 * One datagram arrived on the LED port: it IS a control_state buffer (the RK3
 * protocol always sends the whole buffer). Forward it verbatim as a WRITE_IO on
 * bulk EP 0x01 OUT. Runs on the engine run loop, so it is strictly sequential
 * with the isoc completions and the MIDI read — no locking, no concurrent USB
 * access. recv once per callback; CFSocket re-fires while datagrams remain queued.
 *
 * The write MUST be asynchronous: a synchronous WritePipeTO here blocks this run
 * loop, freezing the isoc servicing and melting the audio stream (observed
 * 2026-06-21 — OUT ring ballooned to full, dev_rate collapsed). WritePipeAsync
 * returns immediately and completes via led_write_done. Best-effort throughout:
 * never disturbs audio. led_wbuf must persist for the duration of the async call.
 */
static void led_recv(CFSocketRef s, CFSocketCallBackType type,
		     CFDataRef address, const void *data, void *info)
{
	(void)type; (void)address; (void)data;
	struct rk3_engine *e = info;
	uint8_t buf[EP1_BUFSIZE];
	ssize_t n = recv(CFSocketGetNative(s), buf, sizeof(buf), 0);
	if (n <= 0)
		return;
	if (e->led_busy)
		return;                       /* a write is still in flight; drop */
	int len = (int)n;
	if (len > EP1_BUFSIZE - 1)
		len = EP1_BUFSIZE - 1;        /* control_state can't exceed the cmd payload */

	/* Always emit a full EP1_BUFSIZE frame (cmd + 63 zero-padded control_state),
	 * matching Linux control.c which sends sizeof(control_state) capped to 63.
	 * The RK3 firmware ignores short frames (a 7-byte write returns success but
	 * lights nothing), so pad the unused tail with zeros. */
	e->led_wbuf[0] = EP1_CMD_WRITE_IO;
	memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
	memcpy(e->led_wbuf + 1, buf, len);
	e->led_busy = 1;
	IOReturn kr = (*e->usb.intf)->WritePipeAsync(e->usb.intf, e->usb.pipe_cmd_out,
			e->led_wbuf, (UInt32)EP1_BUFSIZE, led_write_done, e);
	if (kr != kIOReturnSuccess) {
		e->led_busy = 0;
		if (e->led_errs < 4) {
			ELOG("led: WritePipeAsync arm failed 0x%08x", kr);
			e->led_errs++;
		}
	}
}

/*
 * Bind the loopback UDP LED port and add it to the engine run loop. Non-fatal:
 * if the coreaudiod sandbox blocks inbound UDP (only outbound is proven), we log
 * and continue with audio + MIDI unaffected — LED control is simply unavailable
 * this session. Whether bind() is permitted is the open question this answers.
 */
static void led_listen_start(struct rk3_engine *e)
{
	CFSocketContext ctx = { 0, e, NULL, NULL, NULL };
	e->led_sock = CFSocketCreate(NULL, AF_INET, SOCK_DGRAM, IPPROTO_UDP,
			kCFSocketReadCallBack, led_recv, &ctx);
	if (!e->led_sock) {
		ELOG("led: CFSocketCreate failed — LED control unavailable this session");
		return;
	}
	int fd = CFSocketGetNative(e->led_sock);
	int yes = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	struct sockaddr_in a;
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(RK3_LED_UDP_PORT);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 only */
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		ELOG("led: bind 127.0.0.1:%d failed errno=%d — LED control unavailable "
		     "(coreaudiod sandbox may block inbound UDP)", RK3_LED_UDP_PORT, errno);
		CFRelease(e->led_sock);     /* closes the native fd (close-on-invalidate) */
		e->led_sock = NULL;
		return;
	}

	e->led_src = CFSocketCreateRunLoopSource(NULL, e->led_sock, 0);
	if (!e->led_src) {
		ELOG("led: CFSocketCreateRunLoopSource failed — LED control unavailable");
		CFRelease(e->led_sock);
		e->led_sock = NULL;
		return;
	}
	CFRunLoopAddSource(CFRunLoopGetCurrent(), e->led_src, kCFRunLoopDefaultMode);
	ELOG("led: listening on 127.0.0.1:%d (control_state -> WRITE_IO)", RK3_LED_UDP_PORT);
}

static void led_listen_stop(struct rk3_engine *e)
{
	if (e->led_src) {
		CFRunLoopRemoveSource(CFRunLoopGetCurrent(), e->led_src,
				      kCFRunLoopDefaultMode);
		CFRelease(e->led_src);
		e->led_src = NULL;
	}
	if (e->led_sock) {
		CFSocketInvalidate(e->led_sock);   /* closes the native fd */
		CFRelease(e->led_sock);
		e->led_sock = NULL;
	}
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

	/* Poll the MIDI sentinel ~1 Hz so `touch`/`rm` arm/disarm the relay live,
	 * independent of audio. The relay itself runs on this same run loop. */
	CFRunLoopTimerContext tctx = { 0, e, NULL, NULL, NULL };
	CFRunLoopTimerRef midi_timer = CFRunLoopTimerCreate(NULL,
			CFAbsoluteTimeGetCurrent() + 1.0, 1.0, 0, 0,
			midi_sentinel_tick, &tctx);
	if (midi_timer)
		CFRunLoopAddTimer(CFRunLoopGetCurrent(), midi_timer, kCFRunLoopDefaultMode);

	/* LED / 7-segment control listener. Passive (acts only on inbound datagrams),
	 * no callback-rate risk, so it runs unconditionally — no opt-in sentinel. */
	led_listen_start(e);

	atomic_store_explicit(&e->running, 1, memory_order_release);
	e->start_rc = 0;
	atomic_store_explicit(&e->thread_ready, 1, memory_order_release);

	ELOG("engine USB up (rate=%d bpp=%d) — MIDI relay follows %s; audio idle",
	     e->rate, e->bpp, RK3_MIDI_SENTINEL);
	CFRunLoopRun();   /* until rk3_engine_close() calls CFRunLoopStop */

	/* teardown on the engine thread (audio already stopped by close) */
	led_listen_stop(e);
	midi_relay_stop(e);
	if (midi_timer) {
		CFRunLoopRemoveTimer(CFRunLoopGetCurrent(), midi_timer,
				     kCFRunLoopDefaultMode);
		CFRelease(midi_timer);
	}
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

/*
 * Run `work` on the engine thread's run loop and block (bounded) until it
 * completes. The isoc prime/abort must touch the IOUSBLib async machinery from
 * the run-loop thread, but StartIO/StopIO call in on CoreAudio's thread — this
 * marshals the work across. Bounded wait so a wedged engine can't hang StartIO.
 */
static bool engine_sync(struct rk3_engine *e, int max_ms, void (^work)(void))
{
	if (!e->runloop || !atomic_load_explicit(&e->running, memory_order_acquire))
		return false;
	atomic_int done;
	atomic_init(&done, 0);
	atomic_int *donep = &done;
	CFRunLoopPerformBlock(e->runloop, kCFRunLoopDefaultMode, ^{
		work();
		atomic_store_explicit(donep, 1, memory_order_release);
	});
	CFRunLoopWakeUp(e->runloop);
	for (int i = 0; i < max_ms * 2; i++) {    /* usleep(500) => 2 iters/ms */
		if (atomic_load_explicit(&done, memory_order_acquire))
			return true;
		usleep(500);
	}
	ELOG("engine_sync: timed out (%d ms) waiting for engine thread", max_ms);
	return false;
}

/* Reset all audio-streaming state (rings, cadence mirror, diagnostics, clock
 * telemetry) for a fresh audio_start. Does NOT touch USB / thread / MIDI. */
static void reset_audio_state(struct rk3_engine *e)
{
	e->pat_head = e->pat_tail = e->out_acc = 0;
	e->pat_stage_off = RK3_FRAMES_PER_XFER;   /* force a pop on first OUT fill */
	e->in_errs = e->out_errs = e->out_drops = 0;

	/*
	 * Capture ring empty. Playback ring empty + ungated: the engine sends
	 * silence on OUT until write_output has buffered RK3_OUT_CUSHION, then it
	 * sets pb_r = RK3_OUT_PHASE and pb_started (preserving the 4-byte phase).
	 */
	atomic_store(&e->cap_w, 0);
	atomic_store(&e->cap_r, 0);
	atomic_store(&e->cap_primed, 0);
	e->cap_settle_left = e->cap_settle_init;
	e->cap_measure_left = e->cap_measure_init;
	e->cap_meas_min = ~0u;
	e->out_fill_min = ~0u;
	e->cap_fill_min = ~0u;
	atomic_store(&e->pb_w, 0);
	atomic_store(&e->pb_r, 0);
	atomic_store(&e->pb_started, 0);
	atomic_store(&e->cap_overruns, 0);
	atomic_store(&e->pb_underruns, 0);
	atomic_store(&e->pb_overruns, 0);

	atomic_store(&e->dev_frames, 0);
	e->dbg_ctr = 0;
	e->dbg_host0 = 0;
	e->dbg_devframes0 = 0;
}

/* ---- public API ------------------------------------------------------ */

/* Recompute the rate-scaled fill constants from e->rate. The reference values are
 * tuned at 48 kHz and describe fixed amounts of time / producer-consumer bite, so
 * they scale linearly with the rate. Called at open and on every set_rate. */
static void engine_apply_rate_scale(struct rk3_engine *e)
{
	e->out_cushion       = RK3_SCALE_TO_RATE(RK3_OUT_CUSHION, e->rate);
	e->out_cushion      -= e->out_cushion % RK3_BYTES_PER_FRAME;  /* keep frame-aligned */
	e->out_fill_setpoint = RK3_SCALE_TO_RATE(RK3_OUT_FILL_SETPOINT, e->rate);
	e->cap_fill_floor    = RK3_SCALE_TO_RATE(RK3_CAP_FILL_FLOOR, e->rate);
	e->cap_settle_init   = RK3_SCALE_TO_RATE(RK3_CAP_SETTLE_FRAMES, e->rate);
	e->cap_measure_init  = RK3_SCALE_TO_RATE(RK3_CAP_MEASURE_FRAMES, e->rate);
}

/* Rates the caiaq protocol + this engine accept (caiaq_set_audio_params codes). */
static int rate_supported(int rate)
{
	return rate == 44100 || rate == 48000 || rate == 88200 ||
	       rate == 96000 || rate == 192000;
}

int rk3_engine_set_rate(struct rk3_engine *e, int rate)
{
	if (!rate_supported(rate)) {
		ELOG("set_rate: unsupported rate %d", rate);
		return -1;
	}
	if (atomic_load_explicit(&e->audio_running, memory_order_acquire)) {
		ELOG("set_rate: refused while audio running");
		return -1;
	}
	e->rate = rate;
	e->bpp = caiaq_compute_bpp(rate, e->data_alignment, e->n_streams);
	engine_apply_rate_scale(e);
	ELOG("set_rate: rate=%d bpp=%d (takes effect at next audio_start)", e->rate, e->bpp);
	return 0;
}

int rk3_engine_out_fill_setpoint(const struct rk3_engine *e)
{
	return e->out_fill_setpoint;
}

int rk3_engine_open(struct rk3_engine *e, int rate)
{
	struct caiaq_device_spec spec;

	atomic_store(&e->running, 0);
	atomic_store(&e->audio_running, 0);
	atomic_store(&e->usb_dead, 0);
	atomic_store(&e->thread_ready, 0);
	e->runloop = NULL;
	e->rate = rate;
	e->midi_fd = -1;
	e->midi_send_errs = 0;
	e->midi_cb_ctr = 0;
	e->midi_cb_t0 = 0;
	e->midi_storm_tripped = 0;
	atomic_store(&e->midi_running, 0);
	e->led_sock = NULL;
	e->led_src = NULL;
	e->led_writes = 0;
	e->led_errs = 0;
	e->led_busy = 0;

	if (rk3_usb_open(&e->usb) != 0)
		return -1;

	/* Handshake: identifies the device and yields data_alignment for bpp.
	 * AUDIO_PARAMS is deferred to audio_start (no audio format set while idle,
	 * mirroring the proven standalone MIDI path). */
	if (caiaq_get_device_info(&e->usb, &spec) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}
	int n_streams = spec.num_analog_audio_in / 2;
	if (n_streams < 1) n_streams = 1;
	e->data_alignment = spec.data_alignment;
	e->n_streams = n_streams;
	e->bpp = caiaq_compute_bpp(rate, spec.data_alignment, n_streams);
	engine_apply_rate_scale(e);

	if (pthread_create(&e->thread, NULL, engine_thread, e) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}

	/* wait for the thread to come up (or fail to create its event source) */
	while (!atomic_load_explicit(&e->thread_ready, memory_order_acquire))
		usleep(1000);

	if (e->start_rc != 0) {
		rk3_engine_close(e);
		return -1;
	}
	return 0;
}

int rk3_engine_audio_start(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->running, memory_order_acquire)) {
		ELOG("audio_start: engine not open");
		return -1;
	}
	if (atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return 0;                         /* already streaming */

	/* Rings/diagnostics are reset here (no consumer yet: StartIO hasn't returned
	 * so no DoIOOperation runs). The USB command I/O (AUDIO_PARAMS, which reads a
	 * reply on EP 0x81) and the isoc prime are done on the engine thread so they
	 * never run concurrently with the engine's own USB traffic. */
	reset_audio_state(e);
	atomic_store_explicit(&e->audio_running, 1, memory_order_release);

	__block int params_rc = -1;
	bool ok = engine_sync(e, 9000, ^{
		/*
		 * Quiesce the MIDI relay's async EP 0x81 read first: AUDIO_PARAMS reads
		 * its ack on the SAME bulk-in pipe, and two readers on one pipe conflict.
		 * We're on the engine thread, so this is strictly sequential; the aborted
		 * read's stale completion is harmless (its midi_running guard returns it).
		 */
		int relay_was_on = atomic_load_explicit(&e->midi_running, memory_order_acquire);
		if (relay_was_on) {
			atomic_store_explicit(&e->midi_running, 0, memory_order_release);
			(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_cmd_in);
		}

		/*
		 * AUDIO_PARAMS reads its ack on EP 0x81 (pipe_cmd_in) — the SAME pipe the
		 * MIDI relay's async read was just aborted on. AbortPipe cancels the
		 * in-flight transfer but does NOT resync the endpoint data toggle, so the
		 * device's ack can arrive on a toggle the host no longer expects, the read
		 * times out, and params_rc comes back -1. That's the recurring
		 * "audio_start: failed (params_rc=-1)" that previously needed a reboot
		 * (device_info never hits it — it runs before the relay is ever armed).
		 *
		 * Recover deterministically: ClearPipeStallBothEnds resyncs host+device
		 * toggles for the pipe, and we retry a few times before declaring failure.
		 */
		for (int attempt = 0; attempt < 3; attempt++) {
			(*e->usb.intf)->ClearPipeStallBothEnds(e->usb.intf, e->usb.pipe_cmd_in);
			params_rc = caiaq_set_audio_params(&e->usb, e->rate, 24, e->bpp);
			if (params_rc == 0)
				break;
			ELOG("audio_start: AUDIO_PARAMS attempt %d failed; retrying after pipe resync",
			     attempt + 1);
		}
		if (params_rc == 0) {
			caiaq_codec_reset(&e->codec);
			prime_chains(e);
		}

		/* Re-arm MIDI so the controls keep working alongside audio (or, on a
		 * params failure, while idle — MIDI is independent of audio). */
		if (relay_was_on) {
			atomic_store_explicit(&e->midi_running, 1, memory_order_release);
			midi_arm_read(e);
		}
	});

	if (!ok || params_rc != 0) {
		atomic_store_explicit(&e->audio_running, 0, memory_order_release);
		ELOG("audio_start: failed (sync_ok=%d params_rc=%d)", ok, params_rc);
		return -1;
	}
	ELOG("audio_start: isoc chains primed (rate=%d bpp=%d)", e->rate, e->bpp);
	return 0;
}

void rk3_engine_audio_stop(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;
	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return;
	/* stop resubmits first, then abort in-flight transfers on the engine thread */
	atomic_store_explicit(&e->audio_running, 0, memory_order_release);
	engine_sync(e, 1000, ^{
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_in);
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_out);
	});
	ELOG("audio_stop: isoc chains aborted (USB + MIDI stay up)");
}

void rk3_engine_close(struct rk3_engine *e)
{
	rk3_engine_audio_stop(e);            /* no-op if not streaming */
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

int rk3_engine_usb_dead(struct rk3_engine *e)
{
	return atomic_load_explicit(&e->usb_dead, memory_order_acquire);
}

/* ---- composites (preserve the original one-shot API for the tools) --- */

int rk3_engine_start(struct rk3_engine *e, int rate)
{
	int rc = rk3_engine_open(e, rate);
	if (rc != 0)
		return rc;
	rc = rk3_engine_audio_start(e);
	if (rc != 0) {
		rk3_engine_close(e);
		return rc;
	}
	return 0;
}

void rk3_engine_stop(struct rk3_engine *e)
{
	rk3_engine_audio_stop(e);
	rk3_engine_close(e);
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
			if (e->cap_meas_min != ~0u &&
			    e->cap_meas_min > (unsigned)e->cap_fill_floor) {
				unsigned drop = (e->cap_meas_min - e->cap_fill_floor)
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
	    w >= (unsigned)e->out_cushion) {
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
