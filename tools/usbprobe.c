/*
 * usbprobe.c — RigKontrol 3 USB enumeration probe (macOS / IOKit)
 *
 * Step 1 of the rk3-coreaudio plan: locate the RigKontrol 3
 * (VID 0x17cc / PID 0x1940), open it, and dump every interface /
 * alternate setting / endpoint so we can confirm:
 *   - bulk EP 0x01 (command + MIDI),
 *   - isoch EP 0x02 IN  (audio capture),
 *   - isoch EP 0x06 OUT (audio playback),
 *   - which alternate setting exposes the isoch endpoints,
 *   - that we can actually open + claim the interface on this macOS.
 *
 * This corresponds to caiaq device.c:84 (id table) and device.c:446
 * (usb_set_interface(usb_dev, 0, 1)) in the Linux driver.
 *
 * Build:  make usbprobe   (see ../Makefile)
 * Run:    ./usbprobe      (RK3 must be plugged in)
 *
 * Read-only / diagnostic: it does not stream or reconfigure the device
 * beyond opening the interface and setting the alternate setting.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mach/mach.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>

#define RK3_VID 0x17cc
#define RK3_PID 0x1940

static const char *xfer_type_str(UInt8 t)
{
	switch (t) {
	case kUSBControl:	return "control";
	case kUSBIsoc:		return "isoc";
	case kUSBBulk:		return "bulk";
	case kUSBInterrupt:	return "interrupt";
	default:		return "?";
	}
}

static const char *dir_str(UInt8 d)
{
	switch (d) {
	case kUSBOut:	return "OUT";
	case kUSBIn:	return "IN";
	case kUSBNone:	return "none";
	default:	return "?";
	}
}

/* Walk every alternate setting of one interface and print its endpoints. */
static void dump_interface(IOUSBInterfaceInterface300 **intf)
{
	IOReturn kr;
	UInt8 ifNum = 0, altCur = 0, nEP = 0, cls = 0, sub = 0, proto = 0;

	(*intf)->GetInterfaceNumber(intf, &ifNum);
	(*intf)->GetAlternateSetting(intf, &altCur);
	(*intf)->GetInterfaceClass(intf, &cls);
	(*intf)->GetInterfaceSubClass(intf, &sub);
	(*intf)->GetInterfaceProtocol(intf, &proto);

	printf("  Interface #%u (class %u/%u/%u), current alt %u\n",
	       ifNum, cls, sub, proto, altCur);

	/* Try opening so we can probe endpoints / claim it. */
	kr = (*intf)->USBInterfaceOpen(intf);
	if (kr != kIOReturnSuccess) {
		printf("    [!] USBInterfaceOpen failed: 0x%08x "
		       "(may be claimed by the system) — descriptors only\n", kr);
	}

	/* Enumerate alternate settings 0..7 (RK3 has very few). */
	for (UInt8 alt = 0; alt < 8; alt++) {
		kr = (*intf)->SetAlternateInterface(intf, alt);
		if (kr != kIOReturnSuccess)
			continue; /* alt setting doesn't exist */

		(*intf)->GetNumEndpoints(intf, &nEP);
		printf("    alt %u: %u endpoint(s)\n", alt, nEP);

		/* Pipe 0 is the default control pipe; data pipes start at 1. */
		for (UInt8 pipe = 1; pipe <= nEP; pipe++) {
			UInt8 direction = 0, number = 0, transferType = 0, interval = 0;
			UInt16 maxPacketSize = 0;

			kr = (*intf)->GetPipeProperties(intf, pipe, &direction,
							&number, &transferType,
							&maxPacketSize, &interval);
			if (kr != kIOReturnSuccess) {
				printf("      pipe %u: GetPipeProperties 0x%08x\n",
				       pipe, kr);
				continue;
			}

			printf("      pipe %u -> EP 0x%02x %-3s %-9s "
			       "maxPacket %u interval %u\n",
			       pipe,
			       (unsigned)(number | (direction == kUSBIn ? 0x80 : 0)),
			       dir_str(direction),
			       xfer_type_str(transferType),
			       maxPacketSize, interval);
		}
	}

	/* Leave it on the streaming alt setting (1) like the Linux driver does. */
	kr = (*intf)->SetAlternateInterface(intf, 1);
	printf("    SetAlternateInterface(1) -> 0x%08x %s\n",
	       kr, kr == kIOReturnSuccess ? "(OK, streaming alt claimable)" : "");

	(*intf)->USBInterfaceClose(intf);
}

static void probe_device(io_service_t usbDevice)
{
	IOReturn kr;
	IOCFPlugInInterface **plugIn = NULL;
	IOUSBDeviceInterface300 **dev = NULL;
	SInt32 score = 0;

	kr = IOCreatePlugInInterfaceForService(usbDevice,
			kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID,
			&plugIn, &score);
	if (kr != kIOReturnSuccess || !plugIn) {
		printf("IOCreatePlugInInterfaceForService failed: 0x%08x\n", kr);
		return;
	}

	(*plugIn)->QueryInterface(plugIn,
			CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID300),
			(LPVOID *)&dev);
	IODestroyPlugInInterface(plugIn);
	if (!dev) {
		printf("QueryInterface(device) failed\n");
		return;
	}

	UInt16 vid = 0, pid = 0, rel = 0;
	(*dev)->GetDeviceVendor(dev, &vid);
	(*dev)->GetDeviceProduct(dev, &pid);
	(*dev)->GetDeviceReleaseNumber(dev, &rel);
	printf("Found device VID 0x%04x PID 0x%04x bcdDevice 0x%04x\n",
	       vid, pid, rel);

	kr = (*dev)->USBDeviceOpen(dev);
	if (kr != kIOReturnSuccess) {
		printf("[!] USBDeviceOpen failed: 0x%08x "
		       "(another driver/process owns it?)\n", kr);
		/* Still attempt interface iteration for descriptor info. */
	} else {
		printf("USBDeviceOpen OK (device not claimed by anything else)\n");
	}

	/*
	 * Interface nubs only exist once the device is configured. macOS
	 * won't auto-configure a device it has no class driver for, so the
	 * interface iterator comes back empty until we set the configuration
	 * explicitly. Use the first configuration's bConfigurationValue.
	 */
	UInt8 numConfig = 0;
	(*dev)->GetNumberOfConfigurations(dev, &numConfig);
	printf("Device has %u configuration(s)\n", numConfig);

	if (numConfig > 0) {
		IOUSBConfigurationDescriptorPtr cfg = NULL;
		kr = (*dev)->GetConfigurationDescriptorPtr(dev, 0, &cfg);
		if (kr == kIOReturnSuccess && cfg) {
			printf("config 0: bConfigurationValue %u, %u interface(s), "
			       "maxPower %u\n",
			       cfg->bConfigurationValue, cfg->bNumInterfaces,
			       cfg->MaxPower);
			kr = (*dev)->SetConfiguration(dev, cfg->bConfigurationValue);
			printf("SetConfiguration(%u) -> 0x%08x %s\n",
			       cfg->bConfigurationValue, kr,
			       kr == kIOReturnSuccess ? "OK" : "(failed)");
		} else {
			printf("GetConfigurationDescriptorPtr failed: 0x%08x\n", kr);
		}
	}

	/* Match-any request to iterate every interface. */
	IOUSBFindInterfaceRequest req = {
		.bInterfaceClass    = kIOUSBFindInterfaceDontCare,
		.bInterfaceSubClass = kIOUSBFindInterfaceDontCare,
		.bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
		.bAlternateSetting  = kIOUSBFindInterfaceDontCare,
	};

	io_iterator_t iter = 0;
	kr = (*dev)->CreateInterfaceIterator(dev, &req, &iter);
	if (kr != kIOReturnSuccess) {
		printf("CreateInterfaceIterator failed: 0x%08x\n", kr);
		(*dev)->USBDeviceClose(dev);
		(*dev)->Release(dev);
		return;
	}

	io_service_t usbInterface;
	while ((usbInterface = IOIteratorNext(iter))) {
		IOCFPlugInInterface **iplug = NULL;
		IOUSBInterfaceInterface300 **intf = NULL;
		SInt32 iscore = 0;

		kr = IOCreatePlugInInterfaceForService(usbInterface,
				kIOUSBInterfaceUserClientTypeID,
				kIOCFPlugInInterfaceID, &iplug, &iscore);
		IOObjectRelease(usbInterface);
		if (kr != kIOReturnSuccess || !iplug)
			continue;

		(*iplug)->QueryInterface(iplug,
				CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID300),
				(LPVOID *)&intf);
		IODestroyPlugInInterface(iplug);
		if (!intf)
			continue;

		dump_interface(intf);
		(*intf)->Release(intf);
	}

	IOObjectRelease(iter);
	(*dev)->USBDeviceClose(dev);
	(*dev)->Release(dev);
}

int main(void)
{
	printf("RigKontrol 3 USB probe — looking for VID 0x%04x PID 0x%04x\n",
	       RK3_VID, RK3_PID);

	CFMutableDictionaryRef matching = IOServiceMatching(kIOUSBDeviceClassName);
	if (!matching) {
		fprintf(stderr, "IOServiceMatching returned NULL\n");
		return 1;
	}

	SInt32 vid = RK3_VID, pid = RK3_PID;
	CFNumberRef vidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &vid);
	CFNumberRef pidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &pid);
	CFDictionarySetValue(matching, CFSTR(kUSBVendorID), vidRef);
	CFDictionarySetValue(matching, CFSTR(kUSBProductID), pidRef);
	CFRelease(vidRef);
	CFRelease(pidRef);

	io_iterator_t iter = 0;
	kern_return_t kr = IOServiceGetMatchingServices(kIOMainPortDefault,
							matching, &iter);
	if (kr != KERN_SUCCESS) {
		fprintf(stderr, "IOServiceGetMatchingServices failed: 0x%08x\n", kr);
		return 1;
	}

	io_service_t usbDevice;
	int found = 0;
	while ((usbDevice = IOIteratorNext(iter))) {
		found++;
		probe_device(usbDevice);
		IOObjectRelease(usbDevice);
	}
	IOObjectRelease(iter);

	if (!found) {
		printf("\nNo RigKontrol 3 found. Plug it in and re-run.\n");
		printf("(Tip: `system_profiler SPUSBDataType` to list USB devices.)\n");
		return 2;
	}

	printf("\nDone. Confirm above: bulk EP 0x01, isoc EP 0x02 IN, "
	       "isoc EP 0x06 OUT, and that alt 1 is claimable.\n");
	return 0;
}
