/* rk3_midi.c — see rk3_midi.h */

#include "rk3_midi.h"
#include "caiaq_protocol.h"   /* EP1_CMD_READ_IO / EP1_CMD_READ_ANALOG */

#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- mapping (edit here) -------------------------------------------- */

#define RK3_MIDI_CHANNEL  0          /* 0 = MIDI channel 1 */
#define RK3_N_BUTTONS     9          /* keycode_rk3 has 9 (input.c:33) */

/* Indexed by the firmware's IO bit (0-based). The RK3 reports its 9 foot
 * switches in a scrambled order vs the silkscreen labels, established on real
 * hardware: physical P1..P9 fire bits 5,6,7,8,1,2,3,4,9 (1-based). This table
 * sends each one out as CC(19 + physical_label), so button N -> CC(19+N):
 *   bit0->P5(CC24) bit1->P6(CC25) bit2->P7(CC26) bit3->P8(CC27)
 *   bit4->P1(CC20) bit5->P2(CC21) bit6->P3(CC22) bit7->P4(CC23) bit8->P9(CC28)
 * press => 127, release => 0. */
static const uint8_t kButtonCC[RK3_N_BUTTONS] = {
	24, 25, 26, 27, 20, 21, 22, 23, 28,
};

/* The RK3 analog read reports 3 axes (ABS_X/Y/Z, input.c:226). On this unit
 * the expression pedal sits on axis 2; axes 0/1 read flat (nothing plugged).
 * 0 = axis disabled (no CC emitted). */
#define RK3_N_ANALOG  3
static const uint8_t kAnalogCC[RK3_N_ANALOG] = {
	0, 0, 11,
};
/* Pedal calibration: the expression input does NOT swing the full 0..1023; it
 * was measured ~40 (heel/rest) to ~270 (toe/full). Map that range to 0..127.
 * Adjust to your pedal if it doesn't reach full travel. */
#define RK3_PEDAL_RAW_MIN  40
#define RK3_PEDAL_RAW_MAX  270

/* ---- state ---------------------------------------------------------- */

struct rk3_midi {
	MIDIClientRef   client;
	MIDIEndpointRef source;

	int     have_buttons;
	uint8_t button_state[RK3_N_BUTTONS];   /* last sent 0/1 */

	int     have_analog;
	uint8_t analog_cc_val[RK3_N_ANALOG];   /* last sent 0..127 */
};

/* ---- CoreMIDI send --------------------------------------------------- */

static void send_cc(struct rk3_midi *m, uint8_t cc, uint8_t val)
{
	Byte msg[3] = { (Byte)(0xB0 | (RK3_MIDI_CHANNEL & 0x0f)),
			(Byte)(cc & 0x7f), (Byte)(val & 0x7f) };

	Byte storage[64];
	MIDIPacketList *list = (MIDIPacketList *)storage;
	MIDIPacket *pkt = MIDIPacketListInit(list);
	/* timestamp 0 => deliver now */
	pkt = MIDIPacketListAdd(list, sizeof(storage), pkt, 0, sizeof(msg), msg);
	if (pkt)
		MIDIReceived(m->source, list);
}

/* ---- decoders (ported from Linux input.c) --------------------------- */

/* payload = EP1 buffer without the leading command byte. */
static void handle_io(struct rk3_midi *m, const uint8_t *p, int len,
		      int verbose)
{
	for (int i = 0; i < RK3_N_BUTTONS; i++) {
		int byte = i / 8;
		if (byte >= len)
			break;
		uint8_t bit = (p[byte] >> (i % 8)) & 1;   /* input.c:317 */

		if (!m->have_buttons || bit != m->button_state[i]) {
			m->button_state[i] = bit;
			send_cc(m, kButtonCC[i], bit ? 127 : 0);
			if (verbose)
				printf("  button %d %-3s -> CC%u %d\n",
				       kButtonCC[i] - 19, bit ? "ON" : "off",
				       kButtonCC[i], bit ? 127 : 0);
		}
	}
	m->have_buttons = 1;
}

static void handle_analog(struct rk3_midi *m, const uint8_t *p, int len,
			  int verbose)
{
	for (int a = 0; a < RK3_N_ANALOG; a++) {
		if (kAnalogCC[a] == 0)                     /* axis disabled */
			continue;
		int off = a * 2;                          /* input.c:208 */
		if (off + 1 >= len)
			break;
		int raw = (p[off] << 8) | p[off + 1];     /* big-endian */

		int clamped = raw;
		if (clamped < RK3_PEDAL_RAW_MIN)
			clamped = RK3_PEDAL_RAW_MIN;
		if (clamped > RK3_PEDAL_RAW_MAX)
			clamped = RK3_PEDAL_RAW_MAX;
		uint8_t val = (uint8_t)((clamped - RK3_PEDAL_RAW_MIN) * 127 /
					(RK3_PEDAL_RAW_MAX - RK3_PEDAL_RAW_MIN));

		if (!m->have_analog || val != m->analog_cc_val[a]) {
			m->analog_cc_val[a] = val;
			send_cc(m, kAnalogCC[a], val);
			if (verbose)
				printf("  analog%d raw=%4d -> CC%u %d\n",
				       a, raw, kAnalogCC[a], val);
		}
	}
	m->have_analog = 1;
}

/* ---- public --------------------------------------------------------- */

void rk3_midi_dispatch(struct rk3_midi *m, const uint8_t *buf, int len,
		       int verbose)
{
	if (!m || len < 1)
		return;

	switch (buf[0]) {
	case EP1_CMD_READ_IO:
		handle_io(m, buf + 1, len - 1, verbose);
		break;
	case EP1_CMD_READ_ANALOG:
		handle_analog(m, buf + 1, len - 1, verbose);
		break;
	default:
		break;   /* device-info / midi / audio replies: not ours */
	}
}

struct rk3_midi *rk3_midi_open(const char *name)
{
	struct rk3_midi *m = calloc(1, sizeof(*m));
	if (!m)
		return NULL;

	CFStringRef cfname = CFStringCreateWithCString(
		NULL, name, kCFStringEncodingUTF8);

	OSStatus st = MIDIClientCreate(cfname, NULL, NULL, &m->client);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_open: MIDIClientCreate failed (%d)\n",
			(int)st);
		goto fail;
	}

	st = MIDISourceCreate(m->client, cfname, &m->source);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_open: MIDISourceCreate failed (%d)\n",
			(int)st);
		goto fail;
	}

	CFRelease(cfname);
	return m;

fail:
	if (cfname)
		CFRelease(cfname);
	rk3_midi_close(m);
	return NULL;
}

void rk3_midi_close(struct rk3_midi *m)
{
	if (!m)
		return;
	if (m->source)
		MIDIEndpointDispose(m->source);
	if (m->client)
		MIDIClientDispose(m->client);
	free(m);
}
