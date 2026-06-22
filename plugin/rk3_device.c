/*
 * rk3_device.c — RigKontrol 3 AudioServerPlugin (CoreAudio HAL driver).
 *
 * Publishes one 2-in/2-out 48 kHz Float32 device backed by the IOUSBLib isoc
 * engine (rk3_engine.{h,c}), which owns the USB device directly inside
 * coreaudiod (sandbox de-risk passed 2026-06-21: euid-202 daemon can open the
 * RK3). The HAL object model (plugin -> device -> input/output streams) follows
 * Apple's NullAudio sample; IO bridges Float32 <-> the engine's S24_3BE rings.
 *
 * Clocking (v1): host-clock zero-timestamps at the nominal rate (NullAudio
 * model). The engine's rings absorb the small device/host drift. Proper device-
 * clock sync + latency tuning is Step 5.
 *
 * Install: /Library/Audio/Plug-Ins/HAL/RigKontrol3.driver ; sudo killall coreaudiod.
 */

#include "../rk3_engine.h"

#include <CoreAudio/AudioServerPlugIn.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <os/log.h>
#include <string.h>
#include <unistd.h>

/* ---- logging --------------------------------------------------------- */

static os_log_t dev_log(void)
{
	static os_log_t l;
	static dispatch_once_t once;
	dispatch_once(&once, ^{ l = os_log_create("com.mihai.rk3", "driver"); });
	return l;
}
#define DLOG(fmt, ...) os_log(dev_log(), "[rk3drv] " fmt, ##__VA_ARGS__)

/* ---- object IDs ------------------------------------------------------ */

enum {
	kObjID_PlugIn        = kAudioObjectPlugInObject,   /* == 1 */
	kObjID_Device        = 2,
	kObjID_Stream_Input  = 3,
	kObjID_Stream_Output = 4,
};

#define kDevice_UID          "RigKontrol3:17cc:1940"
#define kDevice_ModelUID     "RigKontrol3:Model"
#define kDefaultSampleRate   48000.0
/*
 * Sample rates published to CoreAudio. The caiaq protocol also supports 88200, but
 * we expose the four the user wants. The engine sets the matching AUDIO_PARAMS and
 * scales its fill constants by rate (rk3_engine_set_rate); the rate-change handshake
 * (SetPropertyData NominalSampleRate -> RequestDeviceConfigurationChange ->
 * Drv_PerformDeviceConfigChange) reopens audio at the new rate, even mid-session.
 */
static const Float64 kSupportedRates[] = { 44100.0, 48000.0, 96000.0, 192000.0 };
#define kNumSupportedRates ((UInt32)(sizeof(kSupportedRates)/sizeof(kSupportedRates[0])))
#define kRingFrames          8192      /* zero-timestamp period (frames) */
#define kSafetyOffsetFrames  256  /* tried 128: round-trip got WORSE (59.8→62.3ms, phase shifted later) — does not shrink physical path, only scheduling phase. Keep 256. */

/*
 * Fill-servo constants (see Drv_GetZeroTimeStamp). The OUT ring is a pure
 * integrator: d(fill)/dt = perceived_rate - device_rate. We set
 *   perceived_rate = device_rate - GAIN*(fill - setpoint)
 * by trimming the zero-timestamp slope, giving first-order convergence of the
 * standing fill to RK3_OUT_FILL_SETPOINT with time constant ~1/GAIN seconds.
 * GetZeroTimeStamp is the servo's sample point (called once per kRingFrames, so
 * ~6 Hz here): GAIN*period << 2 keeps the discrete loop well damped. The trim is
 * clamped so the perceived rate never deviates more than ±1% from nominal (a
 * balloon then drains at up to 1%*Fs ≈ 480 frames/s; inaudible — no resampling).
 */
#define kFillServoGain     0.75   /* 1/sec: fill-error -> rate-trim convergence rate */
#define kFillServoTrimMax  0.01   /* clamp |perceived-rate trim| to 1% (inaudible; no resample) */
/*
 * The raw OUT-ring fill is a sawtooth (consumer pops a whole isoc transfer per
 * out_done while CoreAudio dribbles it back in), so a single sample is noisy and
 * phase-dependent. The servo controls an EMA of it instead. alpha at the ~6 Hz
 * zero-timestamp rate gives a ~1 s smoothing time constant — long enough to reject
 * the sawtooth, short enough to track the startup transient and balloons.
 * gain 2.0 + alpha 0.15 overshot the startup drain and crushed the troughs to ~10
 * (→ 19 underruns); 0.75 + 0.10 is gentler and calmer.
 */
#define kFillServoAlpha    0.10

/* ---- driver state ---------------------------------------------------- */

static AudioServerPlugInDriverInterface gInterface;
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;
static UInt32 gRefCount = 0;
static AudioServerPlugInHostRef gHost = NULL;

static pthread_mutex_t gStateMutex = PTHREAD_MUTEX_INITIALIZER;
static struct rk3_engine gEngine;       /* zero-initialized */
static UInt32 gIOCount = 0;             /* StartIO/StopIO refcount */

/*
 * Current nominal sample rate (the live device format). Read on the IO/property
 * threads; only changed inside the config-change handshake (Drv_PerformDeviceConfig
 * Change), which the HAL runs with IO stopped. Validated against kSupportedRates.
 */
static Float64 gSampleRate = kDefaultSampleRate;

static Boolean rate_is_supported(Float64 r)
{
	for (UInt32 i = 0; i < kNumSupportedRates; i++)
		if (kSupportedRates[i] == r) return true;
	return false;
}

/*
 * Single-owner USB lifecycle. The engine claims the RK3 interface as soon as the
 * plugin loads — not only while audio streams — so the foot-controls MIDI relay
 * works with no audio. A supervisor thread (spawned in Initialize) opens the
 * engine, retries while the device is absent/claimed, and recycles it if the
 * device is unplugged. StartIO/StopIO then merely layer the isoc audio chains on
 * top via rk3_engine_audio_start/stop. All gEngine/gEngineOpen/gIOCount access is
 * under gStateMutex; the supervisor and StartIO both open only under that lock, so
 * the device is never opened twice. */
static int       gEngineOpen = 0;       /* engine currently holds USB (under mutex) */
static pthread_t gSupervisor;
static int       gSupervisorUp = 0;     /* supervisor thread spawned (under mutex) */

/*
 * Zero-timestamp clock + fill servo. The timeline is a line through the anchor
 * point (gAnchorHostTime, gAnchorSampleTime) with slope gTicksPerFrame. The servo
 * adjusts the slope (gServoTrim) to hold the engine's OUT-ring fill at the
 * setpoint; each adjustment re-pivots the anchor around "now" so the
 * sampleTime<->hostTime mapping stays continuous (no timestamp jump). All written
 * and read only on the CoreAudio IO thread (Drv_GetZeroTimeStamp), so no locking.
 */
static Float64 gNominalTicksPerFrame = 0.0; /* host ticks/frame at the nominal rate */
static Float64 gTicksPerFrame        = 0.0; /* current servo-trimmed ticks/frame */
static Float64 gServoTrim            = 0.0; /* fractional perceived-rate trim, clamped */
static Float64 gFillAvg              = -1.0; /* EMA of OUT-ring fill (frames); <0 = uninit */
static UInt64  gAnchorHostTime       = 0;   /* host time of gAnchorSampleTime */
static Float64 gAnchorSampleTime     = 0.0; /* sample time at the anchor */
static UInt64  gNumberTimeStamps     = 0;   /* last reported period index (monotonic) */

static const AudioStreamBasicDescription kFormat = {
	.mSampleRate       = kDefaultSampleRate,
	.mFormatID         = kAudioFormatLinearPCM,
	.mFormatFlags      = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
	.mBytesPerPacket   = 8,
	.mFramesPerPacket  = 1,
	.mBytesPerFrame    = 8,
	.mChannelsPerFrame = 2,
	.mBitsPerChannel   = 32,
};

/* kFormat at the given sample rate (only mSampleRate varies — Float32 stereo). */
static AudioStreamBasicDescription format_at_rate(Float64 rate)
{
	AudioStreamBasicDescription f = kFormat;
	f.mSampleRate = rate;
	return f;
}

/* ===================================================================== *
 *  Engine USB supervisor (single-owner lifecycle)
 * ===================================================================== */

/* Open the engine. MUST be called with gStateMutex held and gEngineOpen == 0.
 * rk3_engine_open is fast when the device is absent (find_device misses), so
 * holding the lock across it is cheap. Returns 1 on success. */
static int engine_open_locked(void)
{
	memset(&gEngine, 0, sizeof(gEngine));
	if (rk3_engine_open(&gEngine, (int)gSampleRate) != 0)
		return 0;
	gEngineOpen = 1;
	DLOG("engine open: USB claimed (MIDI relay live; audio idle)");
	return 1;
}

/*
 * Acquire the RK3 as soon as the plugin loads and keep it: retry while it is
 * absent/claimed, and recycle the engine if it is unplugged (usb_dead) while
 * audio is idle. Removal during active audio is left to coreaudiod's StopIO
 * (recycled on the next tick once gIOCount returns to 0) so we never tear down
 * the engine under a live IO cycle.
 */
static void *engine_supervisor(void *arg)
{
	(void)arg;
	for (;;) {
		pthread_mutex_lock(&gStateMutex);
		if (!gEngineOpen) {
			engine_open_locked();          /* quietly retry if it fails */
		} else if (gIOCount == 0 && rk3_engine_usb_dead(&gEngine)) {
			DLOG("supervisor: device gone — closing engine; reopen on attach");
			rk3_engine_close(&gEngine);
			gEngineOpen = 0;
		}
		pthread_mutex_unlock(&gStateMutex);
		usleep(2 * 1000 * 1000);               /* 2 s poll */
	}
	return NULL;
}

/* ===================================================================== *
 *  IUnknown / factory
 * ===================================================================== */

static HRESULT Drv_QueryInterface(void *inDriver, REFIID inUUID, LPVOID *outIf)
{
	(void)inDriver;
	CFUUIDRef req  = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
	CFUUIDRef base = CFUUIDCreateFromString(NULL,
			CFSTR("00000000-0000-0000-C000-000000000046"));
	CFUUIDRef drv  = CFUUIDCreateFromString(NULL,
			CFSTR("EEA5773D-CC43-49F1-8E00-8F96E7D23B17"));
	HRESULT r = E_NOINTERFACE;
	if (req && (CFEqual(req, base) || CFEqual(req, drv))) {
		gRefCount++;
		*outIf = gDriverRef;
		r = S_OK;
	}
	if (req)  CFRelease(req);
	if (base) CFRelease(base);
	if (drv)  CFRelease(drv);
	return r;
}
static ULONG Drv_AddRef(void *d)  { (void)d; return ++gRefCount; }
static ULONG Drv_Release(void *d) { (void)d; return gRefCount ? --gRefCount : 0; }

/* Recompute the zero-timestamp clock slope for a sample rate (host ticks/frame).
 * Called at Initialize and whenever the nominal rate changes. */
static void recompute_clock_for_rate(Float64 rate)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	/* host ticks per second = 1e9 * denom/numer ; per frame = /rate */
	double ticksPerSec = 1.0e9 * (double)tb.denom / (double)tb.numer;
	gNominalTicksPerFrame = ticksPerSec / rate;
	gTicksPerFrame = gNominalTicksPerFrame;
}

static OSStatus Drv_Initialize(AudioServerPlugInDriverRef inDriver,
			       AudioServerPlugInHostRef inHost)
{
	(void)inDriver;
	gHost = inHost;
	recompute_clock_for_rate(gSampleRate);
	DLOG("Initialize: rate=%.0f nominalTicksPerFrame=%.3f",
	     gSampleRate, gNominalTicksPerFrame);

	/* Spawn the USB supervisor so the engine claims the RK3 (and the MIDI relay
	 * goes live) without waiting for an audio client. */
	pthread_mutex_lock(&gStateMutex);
	if (!gSupervisorUp) {
		if (pthread_create(&gSupervisor, NULL, engine_supervisor, NULL) == 0)
			gSupervisorUp = 1;
		else
			DLOG("Initialize: supervisor thread create failed (MIDI needs audio)");
	}
	pthread_mutex_unlock(&gStateMutex);
	return 0;
}

/* device-management ops we don't support (single hard-coded device) */
static OSStatus Drv_CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc,
		const AudioServerPlugInClientInfo *c, AudioObjectID *out)
{ (void)d;(void)desc;(void)c;(void)out; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Drv_DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID o)
{ (void)d;(void)o; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Drv_AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID o,
		const AudioServerPlugInClientInfo *c)
{ (void)d;(void)o;(void)c; return 0; }
static OSStatus Drv_RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID o,
		const AudioServerPlugInClientInfo *c)
{ (void)d;(void)o;(void)c; return 0; }
/*
 * Apply a pending sample-rate change. The HAL calls this with IO stopped after a
 * SetPropertyData(NominalSampleRate) asked for a RequestDeviceConfigurationChange;
 * the requested rate is carried in inChangeAction (encoded as an integer Hz). We
 * update the live rate, retune the engine (new AUDIO_PARAMS/bpp take effect at the
 * next StartIO) and recompute the clock slope. The HAL re-reads the device/stream
 * formats afterwards and notifies clients.
 */
static OSStatus Drv_PerformDeviceConfigChange(AudioServerPlugInDriverRef d,
		AudioObjectID o, UInt64 a, void *i)
{
	(void)d;(void)i;
	if (o != kObjID_Device)
		return 0;
	Float64 newRate = (Float64)a;
	if (!rate_is_supported(newRate)) {
		DLOG("PerformConfigChange: ignoring unsupported rate %llu", (unsigned long long)a);
		return kAudioHardwareIllegalOperationError;
	}
	pthread_mutex_lock(&gStateMutex);
	gSampleRate = newRate;
	recompute_clock_for_rate(gSampleRate);
	if (gEngineOpen) {
		/* Audio is stopped here (HAL stops IO before a config change), so the new
		 * rate is applied to the idle engine; it re-sends AUDIO_PARAMS at StartIO. */
		if (rk3_engine_set_rate(&gEngine, (int)gSampleRate) != 0)
			DLOG("PerformConfigChange: engine set_rate(%.0f) failed", gSampleRate);
	}
	pthread_mutex_unlock(&gStateMutex);
	DLOG("PerformConfigChange: rate now %.0f Hz", gSampleRate);
	return 0;
}
static OSStatus Drv_AbortDeviceConfigChange(AudioServerPlugInDriverRef d,
		AudioObjectID o, UInt64 a, void *i)
{ (void)d;(void)o;(void)a;(void)i; return 0; }

/* ===================================================================== *
 *  Property helpers
 * ===================================================================== */

#define SET_CFSTRING(out, sz, str) do {                       \
	if (out) *(CFStringRef *)(out) = CFStringCreateCopy(NULL, (str)); \
	*(sz) = sizeof(CFStringRef);                           \
} while (0)

static Boolean obj_is_stream(AudioObjectID o)
{ return o == kObjID_Stream_Input || o == kObjID_Stream_Output; }

/* ---- HasProperty ----------------------------------------------------- */

static Boolean Drv_HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a)
{
	(void)d;(void)c;
	switch (o) {
	case kObjID_PlugIn:
		switch (a->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyDeviceList:
		case kAudioPlugInPropertyTranslateUIDToDevice:
		case kAudioPlugInPropertyResourceBundle:
			return true;
		}
		return false;
	case kObjID_Device:
		switch (a->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyName:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioDevicePropertyDeviceUID:
		case kAudioDevicePropertyModelUID:
		case kAudioDevicePropertyTransportType:
		case kAudioDevicePropertyRelatedDevices:
		case kAudioDevicePropertyClockDomain:
		case kAudioDevicePropertyDeviceIsAlive:
		case kAudioDevicePropertyDeviceIsRunning:
		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		case kAudioDevicePropertyLatency:
		case kAudioDevicePropertyStreams:
		case kAudioObjectPropertyControlList:
		case kAudioDevicePropertySafetyOffset:
		case kAudioDevicePropertyNominalSampleRate:
		case kAudioDevicePropertyAvailableNominalSampleRates:
		case kAudioDevicePropertyIsHidden:
		case kAudioDevicePropertyZeroTimeStampPeriod:
		case kAudioDevicePropertyPreferredChannelsForStereo:
			return true;
		}
		return false;
	default:
		if (obj_is_stream(o)) {
			switch (a->mSelector) {
			case kAudioObjectPropertyBaseClass:
			case kAudioObjectPropertyClass:
			case kAudioObjectPropertyOwner:
			case kAudioStreamPropertyIsActive:
			case kAudioStreamPropertyDirection:
			case kAudioStreamPropertyTerminalType:
			case kAudioStreamPropertyStartingChannel:
			case kAudioStreamPropertyLatency:
			case kAudioStreamPropertyVirtualFormat:
			case kAudioStreamPropertyPhysicalFormat:
			case kAudioStreamPropertyAvailableVirtualFormats:
			case kAudioStreamPropertyAvailablePhysicalFormats:
				return true;
			}
		}
		return false;
	}
}

static OSStatus Drv_IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a, Boolean *out)
{
	(void)d;(void)o;(void)c;
	switch (a->mSelector) {
	case kAudioStreamPropertyIsActive:
	case kAudioDevicePropertyNominalSampleRate:
		*out = true; break;
	default:
		*out = false; break;
	}
	return 0;
}

/* ---- GetPropertyDataSize -------------------------------------------- */

static OSStatus Drv_GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 *outSize)
{
	(void)d;(void)c;(void)qs;(void)q;
	switch (a->mSelector) {
	case kAudioObjectPropertyBaseClass:
	case kAudioObjectPropertyClass:
		*outSize = sizeof(AudioClassID); return 0;
	case kAudioObjectPropertyOwner:
		*outSize = sizeof(AudioObjectID); return 0;
	case kAudioObjectPropertyName:
	case kAudioObjectPropertyManufacturer:
	case kAudioDevicePropertyDeviceUID:
	case kAudioDevicePropertyModelUID:
		*outSize = sizeof(CFStringRef); return 0;
	case kAudioObjectPropertyOwnedObjects:
		if (o == kObjID_PlugIn) { *outSize = sizeof(AudioObjectID); return 0; }
		if (o == kObjID_Device) { *outSize = 2 * sizeof(AudioObjectID); return 0; }
		*outSize = 0; return 0;
	case kAudioPlugInPropertyDeviceList:
		*outSize = sizeof(AudioObjectID); return 0;
	case kAudioPlugInPropertyTranslateUIDToDevice:
		*outSize = sizeof(AudioObjectID); return 0;
	case kAudioPlugInPropertyResourceBundle:
		*outSize = sizeof(CFStringRef); return 0;
	case kAudioDevicePropertyTransportType:
	case kAudioDevicePropertyClockDomain:
	case kAudioDevicePropertyDeviceIsAlive:
	case kAudioDevicePropertyDeviceIsRunning:
	case kAudioDevicePropertyDeviceCanBeDefaultDevice:
	case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
	case kAudioDevicePropertyLatency:
	case kAudioDevicePropertySafetyOffset:
	case kAudioDevicePropertyZeroTimeStampPeriod:
	case kAudioDevicePropertyIsHidden:
	case kAudioStreamPropertyIsActive:
	case kAudioStreamPropertyDirection:
	case kAudioStreamPropertyTerminalType:
	case kAudioStreamPropertyStartingChannel:
		*outSize = sizeof(UInt32); return 0;
	case kAudioDevicePropertyNominalSampleRate:
		*outSize = sizeof(Float64); return 0;
	case kAudioDevicePropertyAvailableNominalSampleRates:
		*outSize = kNumSupportedRates * sizeof(AudioValueRange); return 0;
	case kAudioDevicePropertyPreferredChannelsForStereo:
		*outSize = 2 * sizeof(UInt32); return 0;
	case kAudioDevicePropertyStreams:
		if (a->mScope == kAudioObjectPropertyScopeInput ||
		    a->mScope == kAudioObjectPropertyScopeOutput)
			*outSize = sizeof(AudioObjectID);
		else
			*outSize = 2 * sizeof(AudioObjectID);
		return 0;
	case kAudioObjectPropertyControlList:
	case kAudioDevicePropertyRelatedDevices:
		*outSize = 0; return 0;
	case kAudioStreamPropertyVirtualFormat:
	case kAudioStreamPropertyPhysicalFormat:
		*outSize = sizeof(AudioStreamBasicDescription); return 0;
	case kAudioStreamPropertyAvailableVirtualFormats:
	case kAudioStreamPropertyAvailablePhysicalFormats:
		*outSize = kNumSupportedRates * sizeof(AudioStreamRangedDescription);
		return 0;
	}
	return kAudioHardwareUnknownPropertyError;
}

/* ---- GetPropertyData ------------------------------------------------ */

static OSStatus Drv_GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 inSize, UInt32 *outSize, void *out)
{
	(void)d;(void)c;(void)inSize;(void)qs;
	switch (a->mSelector) {

	case kAudioObjectPropertyBaseClass:
		*outSize = sizeof(AudioClassID);
		*(AudioClassID *)out =
			(o == kObjID_PlugIn) ? kAudioObjectClassID :
			(o == kObjID_Device) ? kAudioObjectClassID :
			kAudioObjectClassID;
		return 0;
	case kAudioObjectPropertyClass:
		*outSize = sizeof(AudioClassID);
		*(AudioClassID *)out =
			(o == kObjID_PlugIn) ? kAudioPlugInClassID :
			(o == kObjID_Device) ? kAudioDeviceClassID :
			kAudioStreamClassID;
		return 0;
	case kAudioObjectPropertyOwner:
		*outSize = sizeof(AudioObjectID);
		*(AudioObjectID *)out =
			(o == kObjID_PlugIn) ? kAudioObjectUnknown : kObjID_Device;
		return 0;
	case kAudioObjectPropertyManufacturer:
		SET_CFSTRING(out, outSize,
			CFSTR("Native Instruments (mihai userspace port)"));
		return 0;

	/* ---- plugin ---- */
	case kAudioObjectPropertyOwnedObjects:
		if (o == kObjID_PlugIn) {
			*outSize = sizeof(AudioObjectID);
			if (out) ((AudioObjectID *)out)[0] = kObjID_Device;
		} else if (o == kObjID_Device) {
			UInt32 n = inSize / sizeof(AudioObjectID);
			AudioObjectID ids[2] = { kObjID_Stream_Input, kObjID_Stream_Output };
			if (n > 2) n = 2;
			*outSize = n * sizeof(AudioObjectID);
			if (out) memcpy(out, ids, *outSize);
		} else {
			*outSize = 0;
		}
		return 0;
	case kAudioPlugInPropertyDeviceList:
		*outSize = sizeof(AudioObjectID);
		if (out) *(AudioObjectID *)out = kObjID_Device;
		return 0;
	case kAudioPlugInPropertyTranslateUIDToDevice: {
		CFStringRef uid = q ? *(CFStringRef *)q : NULL;
		*outSize = sizeof(AudioObjectID);
		*(AudioObjectID *)out =
			(uid && CFStringCompare(uid, CFSTR(kDevice_UID), 0) ==
				kCFCompareEqualTo) ? kObjID_Device : kAudioObjectUnknown;
		return 0;
	}
	case kAudioPlugInPropertyResourceBundle:
		SET_CFSTRING(out, outSize, CFSTR(""));
		return 0;

	/* ---- device ---- */
	case kAudioObjectPropertyName:
		SET_CFSTRING(out, outSize, CFSTR("RigKontrol 3"));
		return 0;
	case kAudioDevicePropertyDeviceUID:
		SET_CFSTRING(out, outSize, CFSTR(kDevice_UID));
		return 0;
	case kAudioDevicePropertyModelUID:
		SET_CFSTRING(out, outSize, CFSTR(kDevice_ModelUID));
		return 0;
	case kAudioDevicePropertyTransportType:
		*outSize = sizeof(UInt32);
		*(UInt32 *)out = kAudioDeviceTransportTypeUSB;
		return 0;
	case kAudioDevicePropertyClockDomain:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 0; return 0;
	case kAudioDevicePropertyDeviceIsAlive:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 1; return 0;
	case kAudioDevicePropertyDeviceIsRunning:
		pthread_mutex_lock(&gStateMutex);
		*(UInt32 *)out = (gIOCount > 0) ? 1 : 0;
		pthread_mutex_unlock(&gStateMutex);
		*outSize = sizeof(UInt32);
		return 0;
	case kAudioDevicePropertyDeviceCanBeDefaultDevice:
	case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 1; return 0;
	case kAudioDevicePropertyLatency: {
		/*
		 * Honest presentation latency the host (Reaper PDC + display) can
		 * see. CoreAudio adds buffer + SafetyOffset on top. Reported once,
		 * on the device object (scope-aware); streams return 0 so the host
		 * does not double-count. One isoc transfer = FRAMES_PER_XFER
		 * microframes of audio (8 microframes = 1 ms => * rate/8000); OUT
		 * and IN now use different transfer sizes (RK3_OUT_FRAMES_PER_XFER
		 * vs RK3_FRAMES_PER_XFER), so account for them separately.
		 */
		UInt32 frames = 0;
		if (o == kObjID_Device) {
			if (a->mScope == kAudioObjectPropertyScopeOutput) {
				UInt32 outXfer = (UInt32)(RK3_OUT_FRAMES_PER_XFER
						* (gSampleRate / 8000.0));
				/* servo-held standing OUT-ring fill (steady state =
				 * setpoint, NOT the initial-only cushion) + the
				 * N_OUT_XFERS-deep isoc pipeline (each refilled buffer
				 * is rescheduled N_OUT_XFERS * stride ahead before it is
				 * heard). The setpoint is rate-scaled by the engine. */
				frames = rk3_engine_out_fill_setpoint(&gEngine)
				       + RK3_N_OUT_XFERS * outXfer;
			} else {
				/* one capture transfer before samples surface */
				frames = (UInt32)(RK3_FRAMES_PER_XFER
						* (gSampleRate / 8000.0));
			}
		}
		*outSize = sizeof(UInt32); *(UInt32 *)out = frames; return 0;
	}
	case kAudioDevicePropertySafetyOffset:
		*outSize = sizeof(UInt32); *(UInt32 *)out = kSafetyOffsetFrames; return 0;
	case kAudioDevicePropertyZeroTimeStampPeriod:
		*outSize = sizeof(UInt32); *(UInt32 *)out = kRingFrames; return 0;
	case kAudioDevicePropertyIsHidden:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 0; return 0;
	case kAudioDevicePropertyNominalSampleRate:
		*outSize = sizeof(Float64); *(Float64 *)out = gSampleRate; return 0;
	case kAudioDevicePropertyAvailableNominalSampleRates: {
		AudioValueRange *rng = out;
		UInt32 avail = inSize / sizeof(AudioValueRange);
		UInt32 n = kNumSupportedRates;
		if (out && n > avail) n = avail;
		for (UInt32 k = 0; out && k < n; k++) {
			rng[k].mMinimum = kSupportedRates[k];
			rng[k].mMaximum = kSupportedRates[k];
		}
		*outSize = (out ? n : kNumSupportedRates) * sizeof(AudioValueRange);
		return 0;
	}
	case kAudioDevicePropertyPreferredChannelsForStereo: {
		*outSize = 2 * sizeof(UInt32);
		UInt32 *ch = out; ch[0] = 1; ch[1] = 2; return 0;
	}
	case kAudioDevicePropertyStreams: {
		AudioObjectID ids[2]; UInt32 n = 0;
		if (a->mScope == kAudioObjectPropertyScopeInput ||
		    a->mScope == kAudioObjectPropertyScopeGlobal)
			ids[n++] = kObjID_Stream_Input;
		if (a->mScope == kAudioObjectPropertyScopeOutput ||
		    a->mScope == kAudioObjectPropertyScopeGlobal)
			ids[n++] = kObjID_Stream_Output;
		UInt32 avail = inSize / sizeof(AudioObjectID);
		if (n > avail) n = avail;
		*outSize = n * sizeof(AudioObjectID);
		if (out) memcpy(out, ids, *outSize);
		return 0;
	}
	case kAudioObjectPropertyControlList:
	case kAudioDevicePropertyRelatedDevices:
		*outSize = 0; return 0;

	/* ---- stream ---- */
	case kAudioStreamPropertyIsActive:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 1; return 0;
	case kAudioStreamPropertyDirection:
		*outSize = sizeof(UInt32);
		*(UInt32 *)out = (o == kObjID_Stream_Input) ? 1 : 0;  /* 1=input */
		return 0;
	case kAudioStreamPropertyTerminalType:
		*outSize = sizeof(UInt32);
		*(UInt32 *)out = (o == kObjID_Stream_Input)
			? kAudioStreamTerminalTypeMicrophone
			: kAudioStreamTerminalTypeSpeaker;
		return 0;
	case kAudioStreamPropertyStartingChannel:
		*outSize = sizeof(UInt32); *(UInt32 *)out = 1; return 0;
	case kAudioStreamPropertyVirtualFormat:
	case kAudioStreamPropertyPhysicalFormat:
		*outSize = sizeof(AudioStreamBasicDescription);
		*(AudioStreamBasicDescription *)out = format_at_rate(gSampleRate);
		return 0;
	case kAudioStreamPropertyAvailableVirtualFormats:
	case kAudioStreamPropertyAvailablePhysicalFormats: {
		AudioStreamRangedDescription *rd = out;
		UInt32 avail = inSize / sizeof(AudioStreamRangedDescription);
		UInt32 n = kNumSupportedRates;
		if (out && n > avail) n = avail;
		for (UInt32 k = 0; out && k < n; k++) {
			rd[k].mFormat = format_at_rate(kSupportedRates[k]);
			rd[k].mSampleRateRange.mMinimum = kSupportedRates[k];
			rd[k].mSampleRateRange.mMaximum = kSupportedRates[k];
		}
		*outSize = (out ? n : kNumSupportedRates)
			 * sizeof(AudioStreamRangedDescription);
		return 0;
	}
	}
	return kAudioHardwareUnknownPropertyError;
}

static OSStatus Drv_SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 inSize, const void *in)
{
	(void)d;(void)o;(void)c;(void)qs;(void)q;
	switch (a->mSelector) {
	case kAudioDevicePropertyNominalSampleRate: {
		if (inSize < sizeof(Float64))
			return kAudioHardwareBadPropertySizeError;
		Float64 req = *(const Float64 *)in;
		if (!rate_is_supported(req))
			return kAudioHardwareIllegalOperationError;
		if (req == gSampleRate)
			return 0;                       /* no change */
		/* Defer the actual switch to the HAL's config-change cycle: it stops IO,
		 * then calls Drv_PerformDeviceConfigChange where we apply the new rate.
		 * The requested rate rides along in the change-action argument. */
		if (gHost && gHost->RequestDeviceConfigurationChange)
			gHost->RequestDeviceConfigurationChange(gHost, kObjID_Device,
					(UInt64)req, NULL);
		return 0;
	}
	case kAudioStreamPropertyIsActive:
		return 0;   /* always active */
	}
	return kAudioHardwareUnsupportedOperationError;
}

/* ===================================================================== *
 *  IO
 * ===================================================================== */

static OSStatus Drv_StartIO(AudioServerPlugInDriverRef d, AudioObjectID o, UInt32 c)
{
	(void)d;(void)o;(void)c;
	OSStatus rc = 0;
	pthread_mutex_lock(&gStateMutex);
	if (gIOCount == 0) {
		/* The supervisor normally has the engine open already; if not (device
		 * just appeared and we beat the 2 s poll), open it now under the lock —
		 * same lock the supervisor opens under, so no double-open race. */
		if (!gEngineOpen)
			engine_open_locked();
		if (!gEngineOpen) {
			DLOG("StartIO: engine not open (device absent/claimed?)");
			rc = kAudioHardwareNotRunningError;
		} else if (rk3_engine_audio_start(&gEngine) != 0) {
			DLOG("StartIO: audio_start FAILED");
			rc = kAudioHardwareNotRunningError;
		} else {
			gAnchorHostTime = mach_absolute_time();
			gAnchorSampleTime = 0.0;
			gTicksPerFrame = gNominalTicksPerFrame;
			gServoTrim = 0.0;
			gFillAvg = -1.0;
			gNumberTimeStamps = 0;
			gIOCount = 1;
			DLOG("StartIO: audio running");
		}
	} else {
		gIOCount++;
	}
	pthread_mutex_unlock(&gStateMutex);
	return rc;
}

static OSStatus Drv_StopIO(AudioServerPlugInDriverRef d, AudioObjectID o, UInt32 c)
{
	(void)d;(void)o;(void)c;
	pthread_mutex_lock(&gStateMutex);
	if (gIOCount > 0 && --gIOCount == 0) {
		/* Stop only the audio chains; the engine keeps USB + the MIDI relay up. */
		rk3_engine_audio_stop(&gEngine);
		DLOG("StopIO: audio stopped (engine + MIDI stay up)");
	}
	pthread_mutex_unlock(&gStateMutex);
	return 0;
}

static OSStatus Drv_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, Float64 *outSampleTime, UInt64 *outHostTime, UInt64 *outSeed)
{
	(void)d;(void)o;(void)c;
	UInt64 now = mach_absolute_time();

	/*
	 * Canonical stepwise zero-timestamp: the anchor (gAnchorHostTime,
	 * gAnchorSampleTime) IS the last reported timestamp and is held CONSTANT for
	 * the whole period; it only steps forward when `now` crosses into a new
	 * period. Returning a constant pair within a period and a strictly increasing
	 * pair across periods is what CoreAudio requires — an earlier version
	 * re-pivoted on every call, which made the reported hostTime non-monotonic
	 * whenever the servo changed the rate and spun the IO thread to 100% CPU.
	 *
	 * The rate (gTicksPerFrame) is only changed AT a period boundary, pivoting
	 * around the freshly reported timestamp, so a rate change rotates only the
	 * FUTURE slope and never moves an already-reported timestamp. This keeps the
	 * servo correct AND the timeline monotonic.
	 */
	Float64 sampleNow = gAnchorSampleTime +
		(Float64)(now - gAnchorHostTime) / gTicksPerFrame;
	UInt64 period = (UInt64)(sampleNow / (Float64)kRingFrames);

	if (period > gNumberTimeStamps) {
		/* Crossed into a new period: advance the anchor to that boundary on the
		 * current line (newSample >= old anchor, so the cast is non-negative and
		 * the new host time is strictly later — monotonic). */
		Float64 newSample = (Float64)(period * kRingFrames);
		UInt64  newHost   = gAnchorHostTime +
			(UInt64)((newSample - gAnchorSampleTime) * gTicksPerFrame);
		gAnchorSampleTime = newSample;
		gAnchorHostTime   = newHost;
		gNumberTimeStamps = period;

		/*
		 * Fill servo (runs once per period, ~6 Hz): trim the perceived device
		 * rate so the OUT-ring standing fill converges to RK3_OUT_FILL_SETPOINT.
		 * fill < 0 = playout not started yet, leave the trim unchanged. A positive
		 * fill error (ring too full) slows the perceived rate so CoreAudio
		 * produces a touch slower and the device drains the excess; negative
		 * speeds it up. Click-free: nothing is resampled, the ring just settles.
		 */
		int fill = rk3_engine_out_fill_frames(&gEngine);
		if (fill >= 0) {
			/*
			 * EMA-smooth the noisy sawtooth fill before acting on it.
			 * SOFT-START: seed the EMA to the SETPOINT, not to the first
			 * observed fill. The ring starts at RK3_OUT_CUSHION (well above
			 * setpoint), so seeding to the real fill made err huge at t0 and
			 * the servo slammed a big NEGATIVE trim, slowing CoreAudio right
			 * as the device consumes fast at startup -> OUT drained to 0 ->
			 * underruns (seen once we lowered the setpoint). Seeding to the
			 * setpoint makes err start ~0; the EMA then tracks the real fill
			 * as the cushion drains gently. Steady state is unaffected (the
			 * EMA converges to the real fill regardless of its seed).
			 */
			int setpoint = rk3_engine_out_fill_setpoint(&gEngine);
			if (gFillAvg < 0.0) gFillAvg = (Float64)setpoint;
			else gFillAvg += kFillServoAlpha * ((Float64)fill - gFillAvg);

			Float64 err = gFillAvg - (Float64)setpoint;
			Float64 trim = -(kFillServoGain * err) / gSampleRate;
			if (trim >  kFillServoTrimMax) trim =  kFillServoTrimMax;
			if (trim < -kFillServoTrimMax) trim = -kFillServoTrimMax;
			gServoTrim = trim;
			gTicksPerFrame = gNominalTicksPerFrame / (1.0 + gServoTrim);

			static unsigned servo_ctr;
			if (++servo_ctr >= 6) {   /* ~once/sec */
				servo_ctr = 0;
				DLOG("servo: fill=%d avg=%.0f setpoint=%d trim=%+.4f%%",
				     fill, gFillAvg, setpoint,
				     gServoTrim * 100.0);
			}
		}
	}

	*outSampleTime = gAnchorSampleTime;
	*outHostTime   = gAnchorHostTime;
	*outSeed       = 1;
	return 0;
}

static OSStatus Drv_WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, Boolean *outWill, Boolean *outInPlace)
{
	(void)d;(void)o;(void)c;
	Boolean will = (op == kAudioServerPlugInIOOperationReadInput ||
			op == kAudioServerPlugInIOOperationWriteMix);
	if (outWill)    *outWill = will;
	if (outInPlace) *outInPlace = true;
	return 0;
}

static OSStatus Drv_BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, UInt32 nf, const AudioServerPlugInIOCycleInfo *i)
{ (void)d;(void)o;(void)c;(void)op;(void)nf;(void)i; return 0; }

static OSStatus Drv_DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		AudioObjectID s, UInt32 c, UInt32 op, UInt32 nf,
		const AudioServerPlugInIOCycleInfo *i, void *mainBuf, void *secBuf)
{
	(void)d;(void)o;(void)s;(void)c;(void)i;(void)secBuf;
	if (op == kAudioServerPlugInIOOperationReadInput)
		rk3_engine_read_input(&gEngine, (float *)mainBuf, (int)nf);
	else if (op == kAudioServerPlugInIOOperationWriteMix)
		rk3_engine_write_output(&gEngine, (const float *)mainBuf, (int)nf);
	return 0;
}

static OSStatus Drv_EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, UInt32 nf, const AudioServerPlugInIOCycleInfo *i)
{ (void)d;(void)o;(void)c;(void)op;(void)nf;(void)i; return 0; }

/* ===================================================================== *
 *  Factory
 * ===================================================================== */

void *RK3DeviceFactory(CFAllocatorRef allocator, CFUUIDRef typeID);
void *RK3DeviceFactory(CFAllocatorRef allocator, CFUUIDRef typeID)
{
	(void)allocator; (void)typeID;
	gInterface = (AudioServerPlugInDriverInterface){
		._reserved = NULL,
		.QueryInterface = Drv_QueryInterface,
		.AddRef = Drv_AddRef,
		.Release = Drv_Release,
		.Initialize = Drv_Initialize,
		.CreateDevice = Drv_CreateDevice,
		.DestroyDevice = Drv_DestroyDevice,
		.AddDeviceClient = Drv_AddDeviceClient,
		.RemoveDeviceClient = Drv_RemoveDeviceClient,
		.PerformDeviceConfigurationChange = Drv_PerformDeviceConfigChange,
		.AbortDeviceConfigurationChange = Drv_AbortDeviceConfigChange,
		.HasProperty = Drv_HasProperty,
		.IsPropertySettable = Drv_IsPropertySettable,
		.GetPropertyDataSize = Drv_GetPropertyDataSize,
		.GetPropertyData = Drv_GetPropertyData,
		.SetPropertyData = Drv_SetPropertyData,
		.StartIO = Drv_StartIO,
		.StopIO = Drv_StopIO,
		.GetZeroTimeStamp = Drv_GetZeroTimeStamp,
		.WillDoIOOperation = Drv_WillDoIOOperation,
		.BeginIOOperation = Drv_BeginIOOperation,
		.DoIOOperation = Drv_DoIOOperation,
		.EndIOOperation = Drv_EndIOOperation,
	};
	gRefCount = 1;
	DLOG("RK3DeviceFactory: instantiated");
	return gDriverRef;
}
