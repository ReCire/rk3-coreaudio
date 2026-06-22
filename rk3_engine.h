/*
 * rk3_engine.h — RigKontrol 3 isochronous streaming engine (reusable).
 *
 * Extracts the hardware-validated isoc transport from tools/stream.c into a
 * module the CoreAudio HAL plugin (or the stream tool) can drive. The engine
 * owns the USB device and runs all isoc I/O on its OWN thread with a dedicated
 * CFRunLoop. It exchanges audio with the caller through two single-producer/
 * single-consumer byte rings carrying native S24_3BE stereo frames:
 *
 *   capture ring   — engine writes (EP 0x82 IN completion), caller reads.
 *   playback ring  — caller writes, engine reads (EP 0x06 OUT fill).
 *
 * The caller side (CoreAudio's real-time IO thread) uses the float bridges
 * rk3_engine_read_input / rk3_engine_write_output, which convert between
 * Float32 (what the HAL exchanges) and S24_3BE (the wire format).
 *
 * Transport discipline (capture chain, free-running OUT chain, microframe
 * frame_start stride, contiguous OUT packing, 0-length-trailing-frame fix,
 * cadence mirror, and the critical OUT 4-byte phase) is preserved exactly from
 * stream.c — see HANDOFF.md "hard-won transport findings". Do not regress it.
 */

#ifndef RK3_ENGINE_H
#define RK3_ENGINE_H

#include "usb_iokit.h"
#include "caiaq_protocol.h"
#include "codec_mode2.h"

#include <CoreFoundation/CoreFoundation.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

/* Transport constants (identical to stream.c). */
#define RK3_N_XFERS         8     /* in-flight capture transfers */
/*
 * In-flight playback transfers. The OUT pipeline latency = N_OUT_XFERS *
 * (FRAMES_PER_XFER/8) ms (each finished buffer is rescheduled N_OUT_XFERS*stride
 * ahead). Reducing this is the SAFE latency lever: it does NOT change the isoc
 * completion rate (so no extra helper load, unlike shrinking FRAMES_PER_XFER) and
 * worst case is an underrun, not a coreaudiod storm. 8 (64ms) was bring-up; Step 5
 * tries 4 (32ms pipeline). Floor is 2 (double-buffer minimum; 1 can't refill in flight).
 * Sweep @64-frame buffer / cushion 320: N_OUT=3 → 35.5ms; N_OUT=2 → 30.0ms, pb_underruns=0
 * over 8s (clean, tight). N_OUT=2 is the resilience floor (8ms refill window, no margin for
 * helper stalls under real DAW load) — validate under Reaper load before trusting it; revert
 * to 3 if it glitches. NOTE: at cushion 320 the ~0.01% host/device drift drains the ring in
 * ~67s, so without device-clock sync this config drift-glitches ~once/min (an 8s test can't
 * see it). Clock sync is the prerequisite for shipping this aggressive config.
 */
#define RK3_N_OUT_XFERS     2
/*
 * Microframes per transfer (8 microframes = 1 ms). This is the isoc granularity
 * and the dominant latency lever: the OUT pipeline = N_OUT_XFERS * FRAMES_PER_XFER/8
 * ms of audio is always scheduled ahead (each finished buffer is rescheduled
 * N_OUT_XFERS*stride ahead). 64 (8ms) = ~64ms OUT pipeline; KNOWN-GOOD (measured
 * 81-84ms round-trip, installs cleanly, never hangs).
 *
 * Step 5 WARNING: setting this to 16 (2ms) stormed coreaudiod and the whole audio
 * subsystem hard enough to require a reboot (2026-06-21). Root cause is the isoc
 * completion-callback RATE: smaller transfers => 4x more completions/sec on the
 * sandboxed helper's CFRunLoop, which overloaded it and pegged CPU. The handshake
 * already has timeouts, so this is NOT a blocking-IO hang. Do NOT shrink this
 * without first proving the helper can sustain the higher callback rate. Cut the
 * OUT pipeline via N_OUT_XFERS instead (fewer in-flight transfers, SAME 8ms
 * completion rate => no extra helper load; worst case is an underrun, not a storm).
 *
 * 2026-06-22 — 64->32 (Gemini #2, the biggest single latency lever): IN completions
 * 125->250/s. With OUT_FPX still 32 (250/s) that's 500/s total, between the proven-
 * safe 375/s and the proven-fatal ~1000/s (global FPX=16). De-risked the TRANSPORT
 * half first in tools/stream.c: --in-fpx 32 / --in-fpx 16 --out-fpx 16 streamed the
 * DEVICE clean up to 1000/s (0 errs/drops, correct rate, bursty cadence click-free).
 * So the device is not the limiter — the only residual risk is whether the sandboxed
 * coreaudiod runloop sustains 500/s (untestable outside coreaudiod). Cuts the IN
 * pipeline ~4ms AND halves the 384-frame capture-bite WALL (-> lower cap floor).
 * Revert to 64 (one line) if the install storms. updateFrequency mid-flight is a
 * NO-OP on this Apple Silicon IOUSBLib shim, so smaller transfers are the ONLY
 * remaining latency lever (see HANDOFF_LATENCY / the stream.c gate result).
 */
#define RK3_FRAMES_PER_XFER 32
#define RK3_FRAME_BYTES     512   /* isoc maxPacketSize per microframe */
#define RK3_XFER_BUF        (RK3_FRAMES_PER_XFER * RK3_FRAME_BYTES)
#define RK3_FRAME_STRIDE    (RK3_FRAMES_PER_XFER / 8)  /* ms-frames per xfer */
/*
 * OUT transfer granularity, split from the IN/global RK3_FRAMES_PER_XFER so the
 * CAPTURE chain keeps its proven 8 ms cadence while OUT uses smaller transfers to
 * cut latency (the OUT pipeline is the dominant term). Must DIVIDE
 * RK3_FRAMES_PER_XFER evenly and not exceed it: the cadence mirror pops one
 * IN-sized block and feeds it to OUT transfers in RK3_OUT_FRAMES_PER_XFER chunks,
 * and the shared rk3_iso_xfer arrays are sized for the larger (IN) value.
 *
 * 32 (4 ms) halves BOTH the OUT pipeline (N_OUT_XFERS*OUT_STRIDE) and the consumer
 * bite vs 64. Callback-rate safety (this is what stormed coreaudiod before): OUT
 * completions = 8000/OUT_FPX per sec.
 *
 * 2026-06-22 — OUT_FPX 32 -> 16 (THE STORM GAMBLE, user-approved). With IN now at
 * FPX=32 (250/s), OUT=16 is 500/s OUT + 250/s IN = 750/s total. This is the
 * UNTESTED middle of the storm window: 375/s & 500/s are PROVEN SAFE, ~1000/s
 * (global FPX=16) is PROVEN FATAL (helper storm -> reboot). 16 is the floor of the
 * window (the static asserts also forbid below 16 since FPX=32). Cuts the OUT
 * pipeline N_OUT_XFERS*OUT_STRIDE from 2*4=8 ms to 2*2=4 ms (~4 ms off round-trip,
 * 16.98 -> ~13 ms target). If the helper storms at 750/s: idle the device + escape
 * IMMEDIATELY (do NOT killall coreaudiod while wedged), then REVERT to 32. Do NOT
 * drop OUT_FPX below 16 (static asserts forbid it at FPX=32, and it would push the
 * rate toward the fatal ceiling).
 */
#define RK3_OUT_FRAMES_PER_XFER 16
#define RK3_OUT_XFER_BUF        (RK3_OUT_FRAMES_PER_XFER * RK3_FRAME_BYTES)
#define RK3_OUT_FRAME_STRIDE    (RK3_OUT_FRAMES_PER_XFER / 8)
_Static_assert(RK3_FRAMES_PER_XFER % RK3_OUT_FRAMES_PER_XFER == 0,
	"OUT_FPX must divide FPX evenly (cadence mirror feeds OUT in OUT_FPX chunks)");
_Static_assert(RK3_OUT_FRAMES_PER_XFER <= RK3_FRAMES_PER_XFER,
	"iso_xfer arrays are sized for FPX; OUT_FPX must not exceed it");
#define RK3_OUT_MIN_LEAD    24
#define RK3_PAT_RING        256

/*
 * Low-latency isoch transport (NI's macOS driver uses this; see HANDOFF_LATENCY).
 * RK3_USE_LOWLAT=1 swaps the basic Read/Write IsochPipeAsync calls for the
 * LowLatency{Read,Write}IsochPipeAsync family with LowLatencyCreateBuffer'd
 * kernel-shared buffers. Step 1 is a 1:1 swap (RK3_LL_UPDATE_FREQ=0 => frame
 * list updated only at transfer completion, i.e. behaviorally identical to the
 * basic API) — a no-regression checkpoint that establishes the LL path inside
 * coreaudiod. The latency win comes later by raising RK3_LL_UPDATE_FREQ to 1 and
 * driving fill/consume from a bounded run-loop timer. Set to 0 to fall back to
 * the proven basic transport (one-line rebuild) if a regression shows up.
 *
 * Proven equivalent to the basic path in tools/stream.c --lowlat (clean tone,
 * 0 errors, identical OUT byte-rate) before being brought in here.
 */
#define RK3_USE_LOWLAT      0   /* TEMP: reverted after a coreaudiod storm on the
                                 * first LL engine install (2026-06-22). Basic path
                                 * is the safe known-good. Re-enable once the LL
                                 * storm is diagnosed (see HANDOFF_LATENCY notes). */
#define RK3_LL_UPDATE_FREQ  0   /* 0 = update at completion only (1:1 mode) */

#if RK3_USE_LOWLAT
typedef IOUSBLowLatencyIsocFrame rk3_isoc_frame_t;
#else
typedef IOUSBIsocFrame           rk3_isoc_frame_t;
#endif

/* Audio shape: RK3 is stereo S24_3BE => 6 bytes per stereo frame. */
#define RK3_BYTES_PER_FRAME 6
#define RK3_CHANNELS        2

/*
 * Ring capacity in bytes; a multiple of RK3_BYTES_PER_FRAME so whole frames
 * never straddle the wrap. ~273 ms at 48 kHz — generous elastic cushion for a
 * first version (clock sync is Step 5).
 */
#define RK3_RING_BYTES      (RK3_BYTES_PER_FRAME * 65536)

/* OUT 4-byte audio phase (Linux out_pos = BYTES_PER_SAMPLE + 1). DO NOT change. */
#define RK3_OUT_PHASE       4
/*
 * OUT-ring fill control. Two distinct knobs — do not conflate them:
 *
 *  - RK3_OUT_CUSHION (bytes, frame-aligned): the INITIAL standing fill the engine
 *    buffers on the OUT ring before it begins playout (the startup gate in
 *    write_output, which then seeds pb_r = RK3_OUT_PHASE). Its only job is to
 *    survive the STARTUP TRANSIENT: for the first ~3 s CoreAudio's producer lags
 *    the device consumer and the fill dips by ~260 frames (measured at a 64-frame
 *    host buffer). The cushion must exceed that dip so the ring does not underrun
 *    before steady state. It does NOT set the steady-state fill.
 *
 *  - RK3_OUT_FILL_SETPOINT (frames): the STEADY-STATE standing fill, held by the
 *    driver's rate servo (Drv_GetZeroTimeStamp). This is what actually sets OUT
 *    latency, and — unlike before — it is STABLE run-to-run.
 *
 * History (do not re-derive): before the servo, the steady fill was uncontrolled.
 * RK3_OUT_CUSHION only seeds the INITIAL value; the steady value was then set by
 * the startup transient PLUS an unbounded underrun self-heal "balloon" (a stalled
 * consumer + a still-producing CoreAudio ratchets the fill up and it never comes
 * back down). That made latency wander 30–69 ms run-to-run; the "30 ms" wins were
 * lucky runs. The servo fixes this at the root: it trims the perceived device
 * clock so the average fill converges to the setpoint, draining balloons and
 * absorbing the ~0.002 % residual drift click-free (no samples added/dropped — the
 * ring simply holds a different amount). See Drv_GetZeroTimeStamp.
 *
 * 512 initial cushion covers the ~260-frame startup dip with margin; the servo
 * then eases the ring down to RK3_OUT_FILL_SETPOINT.
 */
#define RK3_OUT_CUSHION       (RK3_BYTES_PER_FRAME * 512)
/*
 * Servo steady-state target for the MEAN OUT-ring fill (frames). The hard lower
 * bound is set by the CONSUMER BITE: each out_done pops a whole 8 ms isoc transfer
 * (~384 frames at 48 k) out of the ring in one go, while CoreAudio refills it in
 * small (64-frame) chunks. So the post-pop trough is roughly mean − 384/2 ≈
 * mean − 192; for the trough to clear zero with jitter margin the MEAN must be
 * ≳ 256. Telemetry proved this empirically: setpoint 192 (mean below ~256) made
 * the consumer chronically underrun the remainder of its own bite → 19 underruns
 * in 20 s and HIGHER latency (self-heal balloons), while ~256 ran clean.
 *
 * 272 holds the mean just above the floor (post-pop trough ≈ 80 frames of jitter
 * margin) → underrun-free and run-to-run stable. ~5.7 ms of OUT ring; the bulk of
 * the round-trip is the isoc pipeline + safety offset, not this. Going lower
 * trades clicks for a couple ms (bad for guitar) and the real sub-30 ms lever is
 * a smaller isoc transfer — which storms the audio subsystem, so it is gated.
 *
 * 2026-06-22 — re-swept after OUT_FPX 32->16 halved the consumer bite (~96 frames
 * now, not 384). 224 was stale (tuned at the old bite). Swept 192/176/160: 176 ran
 * CLEAN (post-drain out_fill trough ~71-76, pb_underruns=0) and recovered ~1.9 ms
 * (14.31 -> 12.38 ms loopback); 160 gave NO further fill reduction (the new consumer-
 * bite floor), so 176 is the settled sweet spot with clear margin. Revert to 224 if
 * a real DAW load ever underruns here.
 */
#define RK3_OUT_FILL_SETPOINT 176

/*
 * One-time startup drain target for the CAPTURE ring (frames). Capture is the
 * mirror of the OUT consumer-bite problem: the device delivers a whole IN isoc
 * transfer (RK3_FRAMES_PER_XFER microframes ≈ 384 frames at 48k) into the ring in
 * one go while CoreAudio drains it 64 frames at a time, so cap_fill sawtooths ±~192
 * around its mean. That mean is an uncontrolled startup OFFSET — rates are matched
 * (the OUT servo trims the shared CoreAudio clock to the device, so the IN side is
 * rate-matched for free and does NOT self-correct), and telemetry showed it sitting
 * ~490 frames ≈ 10 ms of hidden INPUT latency. We collapse it ONCE, when playout
 * begins, down to this setpoint. Floor = the IN producer bite: IN stays at
 * 64-microframe transfers (shrinking it would raise the isoc callback rate = storm
 * risk), so the bite is ~384 and the mean must stay ≳ 192 + jitter margin. 256
 * reclaims ~5 ms. Draining to V leaves the mean ≈ V (the drain instant is a random
 * phase of the ±192 sawtooth), so the trough lands ≈ V−192; raise V if it underruns.
 *
 * TIMING: the fill is NOT established at pb_started (which flips in the first ~100 ms
 * once the OUT cushion buffers). The device over-delivers during the initial clock
 * transient (dev_rate 48201 → 48013 over ~1 s), and that excess is what we want to
 * drop — so the drain waits RK3_CAP_SETTLE_FRAMES after pb_started before measuring.
 *
 * TROUGH-TARGETED DRAIN: an earlier version drained "to a setpoint at one instant",
 * but the drain fired at an arbitrary phase of the ±192 sawtooth, so the resulting
 * mean was unpredictable (landed ~400 in practice — barely below the ~490 we started
 * from). Instead, after the settle we MEASURE the windowed minimum of cap_fill over
 * RK3_CAP_MEASURE_FRAMES (the actual sawtooth TROUGH, post-transient), then drop
 * exactly (trough − RK3_CAP_FILL_FLOOR) frames once. That lands the steady-state
 * trough on the floor deterministically, so the mean settles to floor + ~½·bite and
 * we keep a known underrun margin regardless of phase. Floor is bounded below by the
 * IN producer bite (~384 frames at 64-microframe transfers): the trough cannot be
 * pushed under it without input underruns, so RK3_CAP_FILL_FLOOR is the jitter margin
 * we want to preserve below the trough, not the mean. Watch cap_fill_min in the log
 * and pb in_errs/underruns; raise the floor if the trough dips toward 0.
 * Caveat: the drain is a one-time splice (input discontinuity, a single click) ~5.5 s
 * into the stream. Startup-only; revisit if it bothers recording.
 *
 * 2026-06-22 — 208 -> 140 after IN dropped to RK3_FRAMES_PER_XFER=32: the IN producer
 * bite HALVED (384 -> ~192 frames), so the sawtooth is now ±~96 not ±192 and the old
 * 208 floor (sized for the 384 bite) left ~2-3 ms of cap-ring slack un-reclaimed.
 * Halved jitter depth means a lower trough floor keeps the same underrun margin. This
 * is a fill constant (zero callback-rate / storm risk); worst case is a capture click,
 * not a storm. The drain still fires ~5.5 s in, so measure with `iotest 10 --latency`
 * (the MIN over impulses = the post-drain steady latency) and watch cap_fill (min) in
 * the log — raise the floor if the post-drain trough dips toward 0.
 *
 * 2026-06-22 (Tier 3) — 140 -> 120: reclaim ~20 frames (~0.4 ms) of IN standing fill.
 * This is the RISKY notch (handoff flagged it): it eats into the already-thin post-drain
 * capture trough, so the failure mode is a CAPTURE underrun (input click), not a storm.
 * MEASURE: watch cap_fill (min) + in_errs/in underruns in the driver-log across a full
 * 10 s run; if the trough dips toward 0 or input clicks, REVERT to 140 (one line).
 */
#define RK3_CAP_FILL_FLOOR    120
#define RK3_CAP_SETTLE_FRAMES  (5 * 48000)         /* ~5 s: the IN transient (dev_rate 48200->48013
                                                    * AND the cap balloon) is still building at 2 s;
                                                    * measuring then is garbage. Wait it out fully. */
#define RK3_CAP_MEASURE_FRAMES (48000 / 2)         /* ~0.5 s window to find the (now stable) trough */

/* The constants above are all expressed at the 48 kHz reference rate. They are
 * fixed amounts of time / producer-consumer bite, so they scale linearly with the
 * actual sample rate. Scale a 48k-reference frame count to the running rate. */
#define RK3_RATE_REF 48000
#define RK3_SCALE_TO_RATE(base48k, rate) \
	((int)(((long long)(base48k) * (rate)) / RK3_RATE_REF))

struct rk3_engine;

struct rk3_iso_xfer {
	struct rk3_engine *e;
	int index;
#if RK3_USE_LOWLAT
	/* kernel-shared, from LowLatencyCreateBuffer; sized per direction (IN uses
	 * RK3_XFER_BUF / RK3_FRAMES_PER_XFER, OUT the smaller OUT_ values). */
	uint8_t *buf;
	rk3_isoc_frame_t *frames;
#else
	uint8_t buf[RK3_XFER_BUF];
	rk3_isoc_frame_t frames[RK3_FRAMES_PER_XFER];
#endif
	UInt64 frame_start;
};

struct rk3_engine {
	struct rk3_usb usb;
	struct caiaq_codec codec;
	int rate;
	int bpp;
	/*
	 * Saved from the GET_DEVICE_INFO handshake so bpp can be recomputed when the
	 * rate changes (rk3_engine_set_rate) without re-running the handshake.
	 */
	int data_alignment;
	int n_streams;
	/*
	 * Rate-scaled copies of the fill-control constants (all of RK3_OUT_CUSHION /
	 * RK3_OUT_FILL_SETPOINT / RK3_CAP_FILL_FLOOR / RK3_CAP_SETTLE_FRAMES /
	 * RK3_CAP_MEASURE_FRAMES are tuned in absolute frames at 48 kHz; they describe
	 * fixed amounts of TIME and producer/consumer bites that both scale linearly
	 * with the sample rate). Recomputed by engine_apply_rate_scale at open and on
	 * every rk3_engine_set_rate so behaviour matches at 44.1/48/96/192 kHz.
	 */
	int out_cushion;        /* bytes (frame-aligned): initial OUT standing fill */
	int out_fill_setpoint;  /* frames: servo steady-state OUT fill target */
	int cap_fill_floor;     /* frames: capture-drain trough floor */
	int cap_settle_init;    /* frames: settle wait before measuring the cap trough */
	int cap_measure_init;   /* frames: cap-trough measurement window */

	atomic_int running;        /* 1 while the engine thread/runloop is alive */
	atomic_int audio_running;  /* 1 while the isoc audio chains should resubmit.
	                            * Distinct from `running`: USB + the MIDI relay
	                            * stay up (running=1) across StartIO/StopIO so the
	                            * foot controls work even with no audio streaming;
	                            * only the isoc chains toggle with audio_running. */
	atomic_int usb_dead;       /* set when a USB completion reports device-gone;
	                            * the plugin supervisor watches this to recycle. */
	pthread_t thread;
	CFRunLoopRef runloop;      /* engine thread's run loop (for stop) */
	atomic_int thread_ready;   /* set once the thread is up (or failed) */
	int start_rc;              /* startup result reported back to caller */

	struct rk3_iso_xfer in[RK3_N_XFERS];
	struct rk3_iso_xfer out[RK3_N_OUT_XFERS];

	/* capture ring: engine producer (cap_w), caller consumer (cap_r). */
	uint8_t cap_buf[RK3_RING_BYTES];
	atomic_uint cap_w, cap_r;
	atomic_int  cap_primed;    /* 0 until the one-time startup capture drain runs */
	int         cap_settle_left;  /* frames left to wait post-pb_started before measuring */
	int         cap_measure_left; /* frames left in the trough-measurement window */
	unsigned    cap_meas_min;     /* windowed-min cap_fill during measurement (frames) */

	/* windowed-min ring fills (frames), reset each telemetry tick — the true
	 * sawtooth troughs, i.e. how close each ring gets to underrun. */
	unsigned    out_fill_min;
	unsigned    cap_fill_min;

	/* playback ring: caller producer (pb_w), engine consumer (pb_r). */
	uint8_t pb_buf[RK3_RING_BYTES];
	atomic_uint pb_w, pb_r;
	atomic_int  pb_started;    /* 0 until CoreAudio buffers OUT_CUSHION */

	/* OUT cadence mirror (engine-thread only; no locking). */
	int pat[RK3_PAT_RING][RK3_FRAMES_PER_XFER];
	int pat_head, pat_tail;
	int out_acc;
	/* Staging block for feeding one IN-sized cadence block to OUT transfers in
	 * RK3_OUT_FRAMES_PER_XFER chunks. pat_stage_off == RK3_FRAMES_PER_XFER means
	 * the stage is drained and the next pat_pop_out refills it. */
	int pat_stage[RK3_FRAMES_PER_XFER];
	int pat_stage_off;

	/* diagnostics */
	atomic_uint cap_overruns, pb_underruns, pb_overruns;
	int in_errs, out_errs, out_drops;

	/* clock/fill telemetry: total stereo frames captured (device clock), and a
	 * once-per-second log tick of the OUT ring fill level (drift vs jitter probe). */
	atomic_ullong dev_frames;
	unsigned dbg_ctr;
	UInt64 dbg_host0;
	unsigned long long dbg_devframes0;

	/*
	 * MIDI relay (engine thread only). The RK3 foot buttons + pedal arrive as
	 * caiaq reports on bulk EP 0x81 IN; we self-resubmit an async read on
	 * pipe_cmd_in and forward each raw report over loopback UDP to the user-space
	 * MIDI helper (the coreaudiod sandbox blocks CoreMIDI directly). Best-effort:
	 * every failure here is logged and ignored — it must never disturb audio.
	 */
	int        midi_fd;            /* connected UDP socket, -1 if disabled */
	atomic_int midi_running;       /* gates the self-resubmitting EP1 read */
	int        midi_send_errs;     /* relay send failures (logged, capped) */
	uint8_t    midi_buf[512];      /* async bulk-read buffer for EP 0x81 */
	unsigned   midi_cb_ctr;        /* completions since last rate-warn window */
	UInt64     midi_cb_t0;         /* mach time at last rate-warn window start */
	/*
	 * Throttled re-arm (the storm de-risk; replaces the old disable-latch).
	 * midi_arm_read stamps midi_last_arm; midi_read_done re-arms inline only if
	 * >= RK3_MIDI_REARM_MIN_MS has elapsed, else it defers the re-arm to
	 * midi_rearm_timer. This caps the EP 0x81 re-arm rate to <= ~1 kHz no matter
	 * how fast completions fire (0-byte success, stall, immediate error), so a
	 * runaway can never peg the run loop / storm coreaudiod — while real input
	 * (~100/s, >=10 ms apart) always re-arms inline with zero added latency. */
	UInt64            midi_last_arm;    /* mach time of the last ReadPipeAsync arm */
	CFRunLoopTimerRef midi_rearm_timer; /* one-shot deferred re-arm (engine run loop) */
	int        midi_storm_tripped; /* retained for telemetry; no longer latches off */

	/*
	 * LED / 7-segment listener (engine thread only). The engine binds a loopback
	 * UDP socket on its run loop; each datagram is a raw control_state buffer that
	 * is written straight to the device via EP1_CMD_WRITE_IO. Stateless (the
	 * caller always sends the whole buffer, as the RK3 protocol requires) and
	 * best-effort — every failure is logged and ignored so it cannot disturb
	 * audio. There is no self-resubmit loop here (CFSocket fires only on real
	 * inbound datagrams), so no storm guard is needed.
	 */
	CFSocketRef        led_sock;   /* wraps the bound UDP fd, NULL if unavailable */
	CFRunLoopSourceRef led_src;    /* run-loop source for led_sock */
	unsigned           led_writes; /* WRITE_IO commands completed (telemetry) */
	int                led_errs;   /* WRITE_IO failures (logged, capped) */
	/*
	 * The WRITE_IO is issued ASYNCHRONOUSLY (WritePipeAsync), never synchronously:
	 * a blocking WritePipeTO on this run-loop thread freezes the isoc servicing and
	 * melts the audio stream (the OUT ring balloons, dev_rate collapses — observed
	 * 2026-06-21). led_wbuf holds the command for the duration of the async call
	 * (cmd byte + control_state) and led_busy serializes to one in-flight write;
	 * datagrams arriving while busy are dropped (best-effort, sub-ms writes so the
	 * 0.4 s walk never collides).
	 */
	uint8_t            led_wbuf[EP1_BUFSIZE];
	int                led_busy;
};

/*
 * Single-owner lifecycle. Because all four endpoints share one USB interface
 * and macOS lets only one process open it, the engine (inside coreaudiod) is the
 * sole owner of the interface for its whole life — MIDI and audio both. The
 * interface is acquired ONCE at rk3_engine_open and held until rk3_engine_close;
 * audio streaming is layered on top with rk3_engine_audio_start/stop. This makes
 * the foot-controls MIDI relay work with no audio streaming, and removes the
 * interface-handoff race entirely (there is never a second USB owner).
 *
 *   open        -> claim USB, run the EP1 handshake (device-info / bpp), start the
 *                  engine thread + run loop, and arm the foot-controls MIDI relay
 *                  unconditionally (always-on; no audio params or isoc chains yet).
 *   audio_start -> set AUDIO_PARAMS, reset the rings, prime the isoc chains. Call
 *                  from CoreAudio StartIO.
 *   audio_stop  -> abort the isoc pipes; USB + the MIDI relay stay up. Call from
 *                  CoreAudio StopIO.
 *   close       -> abort everything, join the thread, release USB.
 *
 * open/audio_start return 0 on success, negative on error (device absent, claimed,
 * or handshake/params failure). audio_start/stop are safe no-ops if the engine is
 * not open. The marshaling of prime/abort onto the engine thread is internal.
 */
int  rk3_engine_open(struct rk3_engine *e, int rate);
void rk3_engine_close(struct rk3_engine *e);
int  rk3_engine_audio_start(struct rk3_engine *e);
void rk3_engine_audio_stop(struct rk3_engine *e);

/*
 * Change the audio sample rate. Recomputes bpp (from the rate + the device_spec
 * saved at open) and the rate-scaled fill constants; the new params take effect at
 * the next rk3_engine_audio_start (which re-sends AUDIO_PARAMS). MUST be called
 * with audio stopped (the CoreAudio rate-change handshake stops IO first). Returns
 * 0 on success, -1 if the rate is not one the device supports.
 */
int  rk3_engine_set_rate(struct rk3_engine *e, int rate);

/* Servo steady-state OUT-fill target in frames, rate-scaled (RK3_OUT_FILL_SETPOINT
 * at 48 kHz). Read by the driver's rate servo + latency reporting. */
int  rk3_engine_out_fill_setpoint(const struct rk3_engine *e);

/*
 * 1 if a USB completion reported the device gone (unplugged) since open. The
 * plugin supervisor polls this to close + reopen the engine on the next attach.
 */
int  rk3_engine_usb_dead(struct rk3_engine *e);

/*
 * Convenience composites (open+audio_start / audio_stop+close) preserving the
 * original one-shot API for the stream/test tools. The plugin uses the split API.
 */
int  rk3_engine_start(struct rk3_engine *e, int rate);
void rk3_engine_stop(struct rk3_engine *e);

/*
 * Real-time bridges, called from CoreAudio's IO thread. nframes = stereo frames;
 * buffers are Float32 interleaved L,R. read_input zero-fills on underrun;
 * write_output drops on overrun (both counted in diagnostics).
 */
void rk3_engine_read_input(struct rk3_engine *e, float *dst, int nframes);
void rk3_engine_write_output(struct rk3_engine *e, const float *src, int nframes);

/*
 * Current OUT-ring standing fill in stereo frames, or -1 if playout has not
 * started yet (still in the startup gate). Read by the driver's rate servo
 * (Drv_GetZeroTimeStamp) to hold the fill at RK3_OUT_FILL_SETPOINT. Safe to call
 * from the CoreAudio IO thread; reads the same atomics as the float bridges.
 */
int rk3_engine_out_fill_frames(struct rk3_engine *e);

#endif /* RK3_ENGINE_H */
