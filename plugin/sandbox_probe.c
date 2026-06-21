/*
 * sandbox_probe.c — Step-4 de-risk: can sandboxed coreaudiod open the RK3 USB
 * interface from inside a HAL plugin?
 *
 * This is a throwaway AudioServerPlugin bundle. It implements just enough of the
 * AudioServerPlugInDriverInterface to be loaded and Initialize()d by coreaudiod,
 * publishing ZERO audio devices. The interesting part runs in Initialize(): it
 * performs the exact IOKit/IOUSBLib open sequence used by usb_iokit.c
 * (find -> open device -> SetConfiguration -> open interface -> USBInterfaceOpen
 * -> SetAlternateInterface) and logs every IOReturn via os_log.
 *
 * Read the result with:
 *   log show --last 2m --predicate 'subsystem == "com.mihai.rk3.probe"' --info
 * or live:
 *   log stream --predicate 'subsystem == "com.mihai.rk3.probe"' --info
 *
 * Interpreting the critical codes:
 *   USBDeviceOpen / USBInterfaceOpen == 0 (kIOReturnSuccess)  -> sandbox ALLOWS USB.
 *       => Step 4 can own USB directly inside coreaudiod. No helper daemon needed.
 *   == 0xe00002e2 (kIOReturnNotPermitted) or a sandbox denial in the log
 *       => coreaudiod is blocked; fall back to a helper daemon + shared-memory ring.
 *   == 0xe00002c5 (kIOReturnExclusiveAccess) / 0xe00002be (kIOReturnBusy)
 *       => something else holds the device (e.g. a stray ./build/stream); not a
 *          sandbox verdict — close the other owner and re-run.
 */

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <os/log.h>
#include <pthread.h>

#define RK3_VID 0x17cc
#define RK3_PID 0x1940
#define STREAMING_ALT 1

static os_log_t gRK3Log;
static void rk3_log_init(void) { gRK3Log = os_log_create("com.mihai.rk3.probe", "usb"); }
static os_log_t rk3_log(void)
{
	static pthread_once_t once = PTHREAD_ONCE_INIT;
	pthread_once(&once, rk3_log_init);
	return gRK3Log;
}

#define PLOG(fmt, ...) os_log(rk3_log(), "[RK3probe] " fmt, ##__VA_ARGS__)

/* ---- the actual de-risk: attempt the USB open from inside coreaudiod ---- */

static void probe_usb_open(void)
{
	IOReturn kr;

	PLOG("=== RK3 USB sandbox probe START (pid=%d, euid=%d) ===",
	     getpid(), geteuid());

	/* 1. match VID/PID */
	CFMutableDictionaryRef matching =
		IOServiceMatching(kIOUSBDeviceClassName);
	if (!matching) {
		PLOG("FAIL: IOServiceMatching returned NULL");
		return;
	}
	SInt32 vid = RK3_VID, pid = RK3_PID;
	CFNumberRef vidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &vid);
	CFNumberRef pidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &pid);
	CFDictionarySetValue(matching, CFSTR(kUSBVendorID), vidRef);
	CFDictionarySetValue(matching, CFSTR(kUSBProductID), pidRef);
	CFRelease(vidRef);
	CFRelease(pidRef);

	io_iterator_t iter = 0;
	kr = IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iter);
	if (kr != KERN_SUCCESS) {
		PLOG("FAIL: IOServiceGetMatchingServices kr=0x%08x", kr);
		return;
	}
	io_service_t svc = IOIteratorNext(iter);
	IOObjectRelease(iter);
	if (!svc) {
		PLOG("FAIL: device not found (RK3 unplugged?) — "
		     "cannot tell sandbox verdict, re-run with device present");
		return;
	}
	PLOG("OK: matched RK3 device service");

	/* 2. device plug-in interface */
	IOCFPlugInInterface **plugIn = NULL;
	IOUSBDeviceInterface300 **dev = NULL;
	SInt32 score = 0;
	kr = IOCreatePlugInInterfaceForService(svc,
			kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID,
			&plugIn, &score);
	IOObjectRelease(svc);
	if (kr != kIOReturnSuccess || !plugIn) {
		PLOG("FAIL (TELLING): IOCreatePlugInInterfaceForService "
		     "kr=0x%08x — sandbox often blocks the user-client here", kr);
		return;
	}
	(*plugIn)->QueryInterface(plugIn,
			CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID300),
			(LPVOID *)&dev);
	IODestroyPlugInInterface(plugIn);
	if (!dev) {
		PLOG("FAIL: QueryInterface(device) returned NULL");
		return;
	}
	PLOG("OK: got IOUSBDeviceInterface300");

	/* 3. THE critical call: open the device */
	kr = (*dev)->USBDeviceOpen(dev);
	if (kr != kIOReturnSuccess) {
		PLOG("FAIL (CRITICAL): USBDeviceOpen kr=0x%08x — "
		     "0xe00002e2=NotPermitted(sandbox), 0xe00002c5=ExclusiveAccess,"
		     " 0xe00002be=Busy", kr);
		(*dev)->Release(dev);
		return;
	}
	PLOG("OK (CRITICAL): USBDeviceOpen SUCCEEDED — coreaudiod CAN open USB");

	/* 4. configure */
	IOUSBConfigurationDescriptorPtr cfg = NULL;
	kr = (*dev)->GetConfigurationDescriptorPtr(dev, 0, &cfg);
	if (kr != kIOReturnSuccess || !cfg) {
		PLOG("FAIL: GetConfigurationDescriptorPtr kr=0x%08x", kr);
		goto close_dev;
	}
	kr = (*dev)->SetConfiguration(dev, cfg->bConfigurationValue);
	if (kr != kIOReturnSuccess) {
		PLOG("FAIL: SetConfiguration kr=0x%08x", kr);
		goto close_dev;
	}
	PLOG("OK: SetConfiguration(%u)", cfg->bConfigurationValue);

	/* 5. open interface #0 */
	IOUSBFindInterfaceRequest req = {
		.bInterfaceClass    = kIOUSBFindInterfaceDontCare,
		.bInterfaceSubClass = kIOUSBFindInterfaceDontCare,
		.bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
		.bAlternateSetting  = kIOUSBFindInterfaceDontCare,
	};
	io_iterator_t iiter = 0;
	kr = (*dev)->CreateInterfaceIterator(dev, &req, &iiter);
	if (kr != kIOReturnSuccess) {
		PLOG("FAIL: CreateInterfaceIterator kr=0x%08x", kr);
		goto close_dev;
	}
	io_service_t isvc = IOIteratorNext(iiter);
	IOObjectRelease(iiter);
	if (!isvc) {
		PLOG("FAIL: no interface nub (not configured?)");
		goto close_dev;
	}
	IOCFPlugInInterface **iplugIn = NULL;
	IOUSBInterfaceInterface300 **intf = NULL;
	kr = IOCreatePlugInInterfaceForService(isvc,
			kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID,
			&iplugIn, &score);
	IOObjectRelease(isvc);
	if (kr != kIOReturnSuccess || !iplugIn) {
		PLOG("FAIL (TELLING): interface IOCreatePlugInInterfaceForService "
		     "kr=0x%08x", kr);
		goto close_dev;
	}
	(*iplugIn)->QueryInterface(iplugIn,
			CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID300),
			(LPVOID *)&intf);
	IODestroyPlugInInterface(iplugIn);
	if (!intf) {
		PLOG("FAIL: QueryInterface(interface) returned NULL");
		goto close_dev;
	}

	/* 6. THE second critical call: open the interface */
	kr = (*intf)->USBInterfaceOpen(intf);
	if (kr != kIOReturnSuccess) {
		PLOG("FAIL (CRITICAL): USBInterfaceOpen kr=0x%08x", kr);
		(*intf)->Release(intf);
		goto close_dev;
	}
	PLOG("OK (CRITICAL): USBInterfaceOpen SUCCEEDED");

	/* 7. alt-set 1 (streaming) — proves we reach the isoc-capable alt */
	kr = (*intf)->SetAlternateInterface(intf, STREAMING_ALT);
	PLOG("%s: SetAlternateInterface(%d) kr=0x%08x",
	     kr == kIOReturnSuccess ? "OK" : "FAIL", STREAMING_ALT, kr);

	(*intf)->USBInterfaceClose(intf);
	(*intf)->Release(intf);
	PLOG("=== VERDICT: coreaudiod CAN fully open + claim the RK3. "
	     "Step 4 may own USB directly. ===");

close_dev:
	(*dev)->USBDeviceClose(dev);
	(*dev)->Release(dev);
}

/* ===================================================================== *
 *  Minimal AudioServerPlugInDriverInterface (publishes zero devices)
 * ===================================================================== */

static UInt32 gRefCount = 0;
static AudioServerPlugInDriverInterface gInterface;
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;

static HRESULT Probe_QueryInterface(void *inDriver, REFIID inUUID,
				    LPVOID *outInterface)
{
	(void)inDriver;
	CFUUIDRef req = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
	CFUUIDRef base = CFUUIDCreateFromString(NULL,
		CFSTR("00000000-0000-0000-C000-000000000046")); /* IUnknown */
	CFUUIDRef drv = CFUUIDCreateFromString(NULL,
		CFSTR("EEA5773D-CC43-49F1-8E00-8F96E7D23B17")); /* AudioServerPlugInDriverInterfaceUUID */
	HRESULT result = E_NOINTERFACE;
	if (req && (CFEqual(req, base) || CFEqual(req, drv))) {
		gRefCount++;
		*outInterface = gDriverRef;
		result = S_OK;
	}
	if (req) CFRelease(req);
	if (base) CFRelease(base);
	if (drv) CFRelease(drv);
	return result;
}

static ULONG Probe_AddRef(void *inDriver) { (void)inDriver; return ++gRefCount; }
static ULONG Probe_Release(void *inDriver) { (void)inDriver; return gRefCount ? --gRefCount : 0; }

static OSStatus Probe_Initialize(AudioServerPlugInDriverRef inDriver,
				 AudioServerPlugInHostRef inHost)
{
	(void)inDriver; (void)inHost;
	probe_usb_open();           /* <<< the de-risk runs here */
	return 0;
}

/* --- everything below: stubs. We own no devices. --- */

static OSStatus Stub_CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef a,
		const AudioServerPlugInClientInfo *c, AudioObjectID *o)
{ (void)d;(void)a;(void)c;(void)o; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Stub_DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID o)
{ (void)d;(void)o; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Stub_AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID o,
		const AudioServerPlugInClientInfo *c)
{ (void)d;(void)o;(void)c; return 0; }
static OSStatus Stub_RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID o,
		const AudioServerPlugInClientInfo *c)
{ (void)d;(void)o;(void)c; return 0; }
static OSStatus Stub_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef d,
		AudioObjectID o, UInt64 a, void *i)
{ (void)d;(void)o;(void)a;(void)i; return 0; }
static OSStatus Stub_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef d,
		AudioObjectID o, UInt64 a, void *i)
{ (void)d;(void)o;(void)a;(void)i; return 0; }

static Boolean Stub_HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a)
{
	(void)d;(void)c;
	if (o == kAudioObjectPlugInObject &&
	    (a->mSelector == kAudioObjectPropertyOwnedObjects ||
	     a->mSelector == kAudioPlugInPropertyDeviceList ||
	     a->mSelector == kAudioObjectPropertyManufacturer ||
	     a->mSelector == kAudioObjectPropertyClass ||
	     a->mSelector == kAudioObjectPropertyBaseClass))
		return true;
	return false;
}
static OSStatus Stub_IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a, Boolean *out)
{ (void)d;(void)o;(void)c;(void)a; if (out) *out = false; return 0; }

static OSStatus Stub_GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 *outSize)
{
	(void)d;(void)o;(void)c;(void)qs;(void)q;
	if (!outSize) return kAudioHardwareIllegalOperationError;
	switch (a->mSelector) {
	case kAudioObjectPropertyOwnedObjects:
	case kAudioPlugInPropertyDeviceList:
		*outSize = 0;               /* zero devices */
		return 0;
	case kAudioObjectPropertyClass:
	case kAudioObjectPropertyBaseClass:
		*outSize = sizeof(AudioClassID);
		return 0;
	case kAudioObjectPropertyManufacturer:
		*outSize = sizeof(CFStringRef);
		return 0;
	default:
		return kAudioHardwareUnknownPropertyError;
	}
}

static OSStatus Stub_GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 inSize, UInt32 *outSize, void *out)
{
	(void)d;(void)o;(void)c;(void)qs;(void)q;(void)inSize;
	if (!outSize) return kAudioHardwareIllegalOperationError;
	switch (a->mSelector) {
	case kAudioObjectPropertyOwnedObjects:
	case kAudioPlugInPropertyDeviceList:
		*outSize = 0;
		return 0;
	case kAudioObjectPropertyClass:
		*outSize = sizeof(AudioClassID);
		if (out) *(AudioClassID *)out = kAudioPlugInClassID;
		return 0;
	case kAudioObjectPropertyBaseClass:
		*outSize = sizeof(AudioClassID);
		if (out) *(AudioClassID *)out = kAudioObjectClassID;
		return 0;
	case kAudioObjectPropertyManufacturer:
		*outSize = sizeof(CFStringRef);
		if (out) *(CFStringRef *)out = CFSTR("mihai");
		return 0;
	default:
		return kAudioHardwareUnknownPropertyError;
	}
}

static OSStatus Stub_SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
		pid_t c, const AudioObjectPropertyAddress *a,
		UInt32 qs, const void *q, UInt32 inSize, const void *in)
{ (void)d;(void)o;(void)c;(void)a;(void)qs;(void)q;(void)inSize;(void)in;
  return kAudioHardwareUnsupportedOperationError; }

static OSStatus Stub_StartIO(AudioServerPlugInDriverRef d, AudioObjectID o, UInt32 c)
{ (void)d;(void)o;(void)c; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Stub_StopIO(AudioServerPlugInDriverRef d, AudioObjectID o, UInt32 c)
{ (void)d;(void)o;(void)c; return kAudioHardwareUnsupportedOperationError; }
static OSStatus Stub_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, Float64 *st, UInt64 *htns, UInt64 *seed)
{ (void)d;(void)o;(void)c;(void)st;(void)htns;(void)seed;
  return kAudioHardwareUnsupportedOperationError; }
static OSStatus Stub_WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, Boolean *will, Boolean *inPlace)
{ (void)d;(void)o;(void)c;(void)op; if (will)*will=false; if (inPlace)*inPlace=false; return 0; }
static OSStatus Stub_BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, UInt32 nf, const AudioServerPlugInIOCycleInfo *i)
{ (void)d;(void)o;(void)c;(void)op;(void)nf;(void)i; return 0; }
static OSStatus Stub_DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		AudioObjectID s, UInt32 c, UInt32 op, UInt32 nf,
		const AudioServerPlugInIOCycleInfo *i, void *mb, void *sb)
{ (void)d;(void)o;(void)s;(void)c;(void)op;(void)nf;(void)i;(void)mb;(void)sb; return 0; }
static OSStatus Stub_EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID o,
		UInt32 c, UInt32 op, UInt32 nf, const AudioServerPlugInIOCycleInfo *i)
{ (void)d;(void)o;(void)c;(void)op;(void)nf;(void)i; return 0; }

/* ---- the COM factory entry point (named in Info.plist) ---- */

void *RK3ProbeFactory(CFAllocatorRef allocator, CFUUIDRef typeID);
void *RK3ProbeFactory(CFAllocatorRef allocator, CFUUIDRef typeID)
{
	(void)allocator; (void)typeID;
	gInterface = (AudioServerPlugInDriverInterface){
		._reserved = NULL,
		.QueryInterface = Probe_QueryInterface,
		.AddRef = Probe_AddRef,
		.Release = Probe_Release,
		.Initialize = Probe_Initialize,
		.CreateDevice = Stub_CreateDevice,
		.DestroyDevice = Stub_DestroyDevice,
		.AddDeviceClient = Stub_AddDeviceClient,
		.RemoveDeviceClient = Stub_RemoveDeviceClient,
		.PerformDeviceConfigurationChange = Stub_PerformDeviceConfigurationChange,
		.AbortDeviceConfigurationChange = Stub_AbortDeviceConfigurationChange,
		.HasProperty = Stub_HasProperty,
		.IsPropertySettable = Stub_IsPropertySettable,
		.GetPropertyDataSize = Stub_GetPropertyDataSize,
		.GetPropertyData = Stub_GetPropertyData,
		.SetPropertyData = Stub_SetPropertyData,
		.StartIO = Stub_StartIO,
		.StopIO = Stub_StopIO,
		.GetZeroTimeStamp = Stub_GetZeroTimeStamp,
		.WillDoIOOperation = Stub_WillDoIOOperation,
		.BeginIOOperation = Stub_BeginIOOperation,
		.DoIOOperation = Stub_DoIOOperation,
		.EndIOOperation = Stub_EndIOOperation,
	};
	gRefCount = 1;
	PLOG("RK3ProbeFactory called — plugin instantiated by coreaudiod");
	return gDriverRef;
}
