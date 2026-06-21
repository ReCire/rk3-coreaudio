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
 */
#define RK3_FRAMES_PER_XFER 64
#define RK3_FRAME_BYTES     512   /* isoc maxPacketSize per microframe */
#define RK3_XFER_BUF        (RK3_FRAMES_PER_XFER * RK3_FRAME_BYTES)
#define RK3_FRAME_STRIDE    (RK3_FRAMES_PER_XFER / 8)  /* ms-frames per xfer */
#define RK3_OUT_MIN_LEAD    24
#define RK3_PAT_RING        256

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
 * Playback cushion (bytes, frame-aligned): the engine sends silence on the OUT
 * pipe until CoreAudio has buffered this much, then starts playout with the read
 * pointer held OUT_PHASE bytes into the audio. The cushion is a STANDING fill of
 * the OUT ring (producer=CoreAudio and consumer=USB are rate-matched), so it is
 * pure added output latency — it must absorb host-thread jitter and host/device
 * clock drift until real device-clock sync exists.
 *
 * Step 5 latency sweep: 4096 (~85 ms) was the glitch-free v1. 768 at N_OUT=4 → 59.8 ms
 * clean; 1024 at N_OUT=3 → 51.7 ms clean. Those were measured with iotest's DEFAULT
 * ~512 host IO buffer, whose 512-frame producer sawtooth dominated the cushion floor.
 *
 * Buffer-size cycle: with a 64-frame host buffer (Reaper-like; iotest --buf 64) the
 * producer becomes smooth, so the floor is set by the CONSUMER bite — fill_out pops one
 * OUT transfer ≈ FRAMES_PER_XFER/8 = 8 ms ≈ 384 frames per out_done. Floor is therefore
 * ~384 + jitter ≈ 512 frames (~11 ms), independent of host buffer. Measured @64-frame
 * buffer, N_OUT=3: 512 → 46.2 ms tight; 384 → 36.75 ms clean but tail widening (mean 40,
 * max 44.5) = nearing the floor. Now probing 320 → if pb_underruns climb, 384 is the
 * floor and we ship that; if still 0, try 256.
 * NOTE: only valid when the host actually uses a small buffer (Reaper 64; NOT the ~512
 * default — at the default this would underrun, so don't ship <768 without confirming).
 */
#define RK3_OUT_CUSHION     (RK3_BYTES_PER_FRAME * 320)

struct rk3_engine;

struct rk3_iso_xfer {
	struct rk3_engine *e;
	int index;
	uint8_t buf[RK3_XFER_BUF];
	IOUSBIsocFrame frames[RK3_FRAMES_PER_XFER];
	UInt64 frame_start;
};

struct rk3_engine {
	struct rk3_usb usb;
	struct caiaq_codec codec;
	int rate;
	int bpp;

	atomic_int running;        /* 1 while the isoc chains should resubmit */
	pthread_t thread;
	CFRunLoopRef runloop;      /* engine thread's run loop (for stop) */
	atomic_int thread_ready;   /* set once the chains are primed */
	int start_rc;              /* startup result reported back to caller */

	struct rk3_iso_xfer in[RK3_N_XFERS];
	struct rk3_iso_xfer out[RK3_N_OUT_XFERS];

	/* capture ring: engine producer (cap_w), caller consumer (cap_r). */
	uint8_t cap_buf[RK3_RING_BYTES];
	atomic_uint cap_w, cap_r;

	/* playback ring: caller producer (pb_w), engine consumer (pb_r). */
	uint8_t pb_buf[RK3_RING_BYTES];
	atomic_uint pb_w, pb_r;
	atomic_int  pb_started;    /* 0 until CoreAudio buffers OUT_CUSHION */

	/* OUT cadence mirror (engine-thread only; no locking). */
	int pat[RK3_PAT_RING][RK3_FRAMES_PER_XFER];
	int pat_head, pat_tail;
	int out_acc;

	/* diagnostics */
	atomic_uint cap_overruns, pb_underruns, pb_overruns;
	int in_errs, out_errs, out_drops;

	/* clock/fill telemetry: total stereo frames captured (device clock), and a
	 * once-per-second log tick of the OUT ring fill level (drift vs jitter probe). */
	atomic_ullong dev_frames;
	unsigned dbg_ctr;
	UInt64 dbg_host0;
	unsigned long long dbg_devframes0;
};

/*
 * Open the RK3, run the EP1 handshake + AUDIO_PARAMS for `rate`, prime the isoc
 * chains, and start the engine thread. Returns 0 on success, negative on error
 * (device absent, claimed, or handshake failure). On failure the engine is left
 * closed and may be retried.
 */
int  rk3_engine_start(struct rk3_engine *e, int rate);

/* Stop the chains, abort the pipes, join the thread, close USB. Idempotent. */
void rk3_engine_stop(struct rk3_engine *e);

/*
 * Real-time bridges, called from CoreAudio's IO thread. nframes = stereo frames;
 * buffers are Float32 interleaved L,R. read_input zero-fills on underrun;
 * write_output drops on overrun (both counted in diagnostics).
 */
void rk3_engine_read_input(struct rk3_engine *e, float *dst, int nframes);
void rk3_engine_write_output(struct rk3_engine *e, const float *src, int nframes);

#endif /* RK3_ENGINE_H */
