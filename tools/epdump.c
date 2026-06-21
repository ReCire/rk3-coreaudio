/*
 * epdump.c — dump the RK3 configuration descriptor, decoding each isoc
 * endpoint's bmAttributes synchronization type (bits 2..3):
 *   00 no sync, 01 asynchronous, 10 adaptive, 11 synchronous.
 * This tells us how EP 0x06 OUT derives its DAC clock.
 */

#include "../usb_iokit.h"
#include <stdio.h>

static const char *synctype(uint8_t attr)
{
	switch ((attr >> 2) & 3) {
	case 0: return "no-sync";
	case 1: return "ASYNC";
	case 2: return "ADAPTIVE";
	case 3: return "SYNC";
	}
	return "?";
}

int main(void)
{
	struct rk3_usb u;
	if (rk3_usb_open(&u) != 0)
		return 1;

	IOUSBConfigurationDescriptorPtr cfg = NULL;
	IOReturn kr = (*u.dev)->GetConfigurationDescriptorPtr(u.dev, 0, &cfg);
	if (kr != kIOReturnSuccess || !cfg) {
		fprintf(stderr, "GetConfigurationDescriptorPtr 0x%08x\n", kr);
		rk3_usb_close(&u);
		return 1;
	}

	const uint8_t *p = (const uint8_t *)cfg;
	int total = (cfg->wTotalLength & 0xff) | (cfg->wTotalLength >> 8 << 8);
	printf("config wTotalLength=%d\n", total);

	for (int i = 0; i < total; ) {
		uint8_t len = p[i], type = p[i + 1];
		if (len == 0) break;
		if (type == 0x04) {                 /* INTERFACE */
			printf("INTERFACE #%u alt %u, %u endpoints, class %u/%u/%u\n",
			       p[i + 2], p[i + 3], p[i + 4], p[i + 5], p[i + 6], p[i + 7]);
		} else if (type == 0x05) {          /* ENDPOINT */
			uint8_t addr = p[i + 2], attr = p[i + 3];
			uint16_t mps = p[i + 4] | (p[i + 5] << 8);
			uint8_t interval = p[i + 6];
			const char *tt = (attr & 3) == 1 ? "isoc" :
					 (attr & 3) == 2 ? "bulk" :
					 (attr & 3) == 3 ? "intr" : "ctrl";
			printf("  EP 0x%02x %s attr=0x%02x %s%s maxPkt=%u(x%u) interval=%u\n",
			       addr, (addr & 0x80) ? "IN " : "OUT", attr, tt,
			       (attr & 3) == 1 ? "/" : "",
			       mps & 0x7ff, ((mps >> 11) & 3) + 1, interval);
			if ((attr & 3) == 1)
				printf("       isoc sync=%s usage=%u\n",
				       synctype(attr), (attr >> 4) & 3);
		}
		i += len;
	}

	rk3_usb_close(&u);
	return 0;
}
