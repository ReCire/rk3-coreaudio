/*
 * midi_probe.c — Phase-2 de-risk, ROUND 2: which IPC egress survives the
 * coreaudiod sandbox?
 *
 * Round 1 verdict (recorded): MIDIClientCreate from inside coreaudiod fails with
 * st=-10833 — the sandbox blocks the MIDIServer connection, so the engine CANNOT
 * own the CoreMIDI source. The fallback is a normal-user helper (LaunchAgent)
 * that owns CoreMIDI; the engine (which owns the USB interface while streaming)
 * RELAYS the raw control reports to it. This round answers the only remaining
 * unknown for that design: can coreaudiod open a socket and send to the helper?
 *
 * Tests, in order of preference:
 *   1. UDP loopback   socket(AF_INET, SOCK_DGRAM) -> sendto 127.0.0.1:RK3_PORT
 *   2. UNIX datagram  socket(AF_UNIX, SOCK_DGRAM) -> sendto /tmp/rk3midi.sock
 * For each we log the call result + errno. A sandbox DENIAL shows up as the
 * socket()/sendto() failing with EPERM (errno 1). ENOENT/ECONNREFUSED on the
 * UNIX path is NOT a denial — it just means no listener yet (transport viable).
 *
 * Read the result with:
 *   make midi-probe-log
 *
 * Interpreting:
 *   UDP sendto OK (errno 0)  -> use UDP loopback engine->helper. Simplest. DONE.
 *   UDP blocked, UNIX OK     -> use a UNIX datagram socket instead.
 *   both blocked (EPERM)     -> need an XPC service / mach relay (heavier).
 */

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <os/log.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>

#define RK3_PORT 17890

static os_log_t gLog;
static void log_init(void) { gLog = os_log_create("com.mihai.rk3.midiprobe", "midi"); }
static os_log_t mlog(void)
{
	static pthread_once_t once = PTHREAD_ONCE_INIT;
	pthread_once(&once, log_init);
	return gLog;
}
#define PLOG(fmt, ...) os_log(mlog(), "[RK3midiprobe] " fmt, ##__VA_ARGS__)

/* ---- the actual de-risk: which IPC egress works from coreaudiod? ---- */

static void probe_udp(void)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		PLOG("UDP  FAIL: socket(AF_INET) errno=%d (%s)%s",
		     errno, strerror(errno),
		     errno == EPERM ? " — SANDBOX DENIAL" : "");
		return;
	}
	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(RK3_PORT);
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 */
	const char payload[] = "RK3PROBE";
	ssize_t n = sendto(fd, payload, sizeof(payload), 0,
			   (struct sockaddr *)&to, sizeof(to));
	if (n < 0)
		PLOG("UDP  FAIL: sendto 127.0.0.1:%d errno=%d (%s)%s",
		     RK3_PORT, errno, strerror(errno),
		     errno == EPERM ? " — SANDBOX DENIAL" : "");
	else
		PLOG("UDP  OK: sent %zd bytes to 127.0.0.1:%d — loopback UDP USABLE",
		     n, RK3_PORT);
	close(fd);
}

static void probe_unix(void)
{
	int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (fd < 0) {
		PLOG("UNIX FAIL: socket(AF_UNIX) errno=%d (%s)%s",
		     errno, strerror(errno),
		     errno == EPERM ? " — SANDBOX DENIAL" : "");
		return;
	}
	struct sockaddr_un to;
	memset(&to, 0, sizeof(to));
	to.sun_family = AF_UNIX;
	strncpy(to.sun_path, "/tmp/rk3midi.sock", sizeof(to.sun_path) - 1);
	const char payload[] = "RK3PROBE";
	ssize_t n = sendto(fd, payload, sizeof(payload), 0,
			   (struct sockaddr *)&to, sizeof(to));
	if (n < 0)
		PLOG("UNIX %s: sendto %s errno=%d (%s)%s",
		     errno == EPERM ? "FAIL" : "OK(no-listener)",
		     to.sun_path, errno, strerror(errno),
		     errno == EPERM ? " — SANDBOX DENIAL"
		     : " — socket allowed; only missing a listener (transport USABLE)");
	else
		PLOG("UNIX OK: sent %zd bytes to %s", n, to.sun_path);
	close(fd);
}

static void probe_midi(void)
{
	PLOG("=== RK3 IPC-transport sandbox probe START (pid=%d, euid=%d) ===",
	     getpid(), geteuid());
	probe_udp();
	probe_unix();
	PLOG("=== transport probe DONE — pick the first USABLE transport above ===");
}

/* ===================================================================== *
 *  Minimal AudioServerPlugInDriverInterface (publishes zero devices)
 * ===================================================================== */

static UInt32 gRefCount = 0;
static AudioServerPlugInDriverInterface gInterface;
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;

static HRESULT Probe_QueryInterface(void *inDriver, REFIID inUUID, LPVOID *outInterface)
{
	(void)inDriver;
	CFUUIDRef req = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
	CFUUIDRef base = CFUUIDCreateFromString(NULL,
		CFSTR("00000000-0000-0000-C000-000000000046")); /* IUnknown */
	CFUUIDRef drv = CFUUIDCreateFromString(NULL,
		CFSTR("EEA5773D-CC43-49F1-8E00-8F96E7D23B17")); /* AudioServerPlugInDriverInterface */
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
	probe_midi();               /* <<< the de-risk runs here */
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
		*outSize = 0;
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

/* ---- the COM factory entry point (named in midi_probe.plist) ---- */

void *RK3MidiProbeFactory(CFAllocatorRef allocator, CFUUIDRef typeID);
void *RK3MidiProbeFactory(CFAllocatorRef allocator, CFUUIDRef typeID)
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
	PLOG("RK3MidiProbeFactory called — plugin instantiated by coreaudiod");
	return gDriverRef;
}
