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
#define kSampleRate          48000.0
#define kRingFrames          8192      /* zero-timestamp period (frames) */
#define kSafetyOffsetFrames  256  /* tried 128: round-trip got WORSE (59.8→62.3ms, phase shifted later) — does not shrink physical path, only scheduling phase. Keep 256. */

/* ---- driver state ---------------------------------------------------- */

static AudioServerPlugInDriverInterface gInterface;
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;
static UInt32 gRefCount = 0;
static AudioServerPlugInHostRef gHost = NULL;

static pthread_mutex_t gStateMutex = PTHREAD_MUTEX_INITIALIZER;
static struct rk3_engine gEngine;       /* zero-initialized */
static UInt32 gIOCount = 0;             /* StartIO/StopIO refcount */

/* zero-timestamp host clock */
static Float64 gHostTicksPerFrame = 0.0;
static UInt64  gAnchorHostTime = 0;
static UInt64  gNumberTimeStamps = 0;

static const AudioStreamBasicDescription kFormat = {
	.mSampleRate       = kSampleRate,
	.mFormatID         = kAudioFormatLinearPCM,
	.mFormatFlags      = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
	.mBytesPerPacket   = 8,
	.mFramesPerPacket  = 1,
	.mBytesPerFrame    = 8,
	.mChannelsPerFrame = 2,
	.mBitsPerChannel   = 32,
};

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

static OSStatus Drv_Initialize(AudioServerPlugInDriverRef inDriver,
			       AudioServerPlugInHostRef inHost)
{
	(void)inDriver;
	gHost = inHost;
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	/* host ticks per second = 1e9 * denom/numer ; per frame = /rate */
	double ticksPerSec = 1.0e9 * (double)tb.denom / (double)tb.numer;
	gHostTicksPerFrame = ticksPerSec / kSampleRate;
	DLOG("Initialize: hostTicksPerFrame=%.3f", gHostTicksPerFrame);
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
static OSStatus Drv_PerformDeviceConfigChange(AudioServerPlugInDriverRef d,
		AudioObjectID o, UInt64 a, void *i)
{ (void)d;(void)o;(void)a;(void)i; return 0; }
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
		*outSize = sizeof(AudioValueRange); return 0;
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
		*outSize = sizeof(AudioStreamRangedDescription); return 0;
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
		 * does not double-count. One isoc transfer = RK3_FRAMES_PER_XFER
		 * microframes of audio (8 microframes = 1 ms => * rate/8000).
		 */
		UInt32 frames = 0;
		if (o == kObjID_Device) {
			UInt32 xferFrames =
				(UInt32)(RK3_FRAMES_PER_XFER * (kSampleRate / 8000.0));
			if (a->mScope == kAudioObjectPropertyScopeOutput)
				/* standing OUT ring cushion + the N_OUT_XFERS-deep isoc
				 * pipeline (each refilled buffer is rescheduled
				 * N_OUT_XFERS * stride ahead before it is heard) */
				frames = RK3_OUT_CUSHION / RK3_BYTES_PER_FRAME
				       + RK3_N_OUT_XFERS * xferFrames;
			else
				/* one capture transfer before samples surface */
				frames = xferFrames;
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
		*outSize = sizeof(Float64); *(Float64 *)out = kSampleRate; return 0;
	case kAudioDevicePropertyAvailableNominalSampleRates: {
		*outSize = sizeof(AudioValueRange);
		AudioValueRange *rng = out;
		rng->mMinimum = kSampleRate; rng->mMaximum = kSampleRate;
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
		*(AudioStreamBasicDescription *)out = kFormat;
		return 0;
	case kAudioStreamPropertyAvailableVirtualFormats:
	case kAudioStreamPropertyAvailablePhysicalFormats: {
		*outSize = sizeof(AudioStreamRangedDescription);
		AudioStreamRangedDescription *rd = out;
		rd->mFormat = kFormat;
		rd->mSampleRateRange.mMinimum = kSampleRate;
		rd->mSampleRateRange.mMaximum = kSampleRate;
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
	case kAudioDevicePropertyNominalSampleRate:
		/* single supported rate; accept the matching value, ignore others */
		if (inSize >= sizeof(Float64) && *(const Float64 *)in == kSampleRate)
			return 0;
		return kAudioHardwareIllegalOperationError;
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
		memset(&gEngine, 0, sizeof(gEngine));
		if (rk3_engine_start(&gEngine, (int)kSampleRate) != 0) {
			DLOG("StartIO: engine_start FAILED (device absent/claimed?)");
			rc = kAudioHardwareNotRunningError;
		} else {
			gAnchorHostTime = mach_absolute_time();
			gNumberTimeStamps = 0;
			gIOCount = 1;
			DLOG("StartIO: engine running");
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
		rk3_engine_stop(&gEngine);
		DLOG("StopIO: engine stopped");
	}
	pthread_mutex_unlock(&gStateMutex);
	return 0;
}

static OSStatus Drv_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, Float64 *outSampleTime, UInt64 *outHostTime, UInt64 *outSeed)
{
	(void)d;(void)o;(void)c;
	UInt64 now = mach_absolute_time();
	Float64 framesSinceAnchor = (Float64)(now - gAnchorHostTime) / gHostTicksPerFrame;
	UInt64 periods = (UInt64)(framesSinceAnchor / (Float64)kRingFrames);
	if (periods > gNumberTimeStamps)
		gNumberTimeStamps = periods;
	*outSampleTime = (Float64)(gNumberTimeStamps * kRingFrames);
	*outHostTime   = gAnchorHostTime +
		(UInt64)((Float64)(gNumberTimeStamps * kRingFrames) * gHostTicksPerFrame);
	*outSeed = 1;
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
