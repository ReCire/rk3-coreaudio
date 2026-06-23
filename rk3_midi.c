/* rk3_midi.c — see rk3_midi.h */

#include "rk3_midi.h"
#include "caiaq_protocol.h"   /* EP1_CMD_READ_IO / EP1_CMD_READ_ANALOG */

#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

/* The RK3 analog read reports 3 axes (ABS_X/Y/Z, input.c:226):
 *   axis 2 -> the built-in expression treadle
 *   axis 0 / axis 1 -> the two rear external pedal jacks (EXP 1 / EXP 2)
 * Map each axis to a CC; 0 = axis disabled (no CC emitted). */
#define RK3_N_ANALOG  3
static const uint8_t kAnalogCC[RK3_N_ANALOG] = {
	12, 13, 11,
};
/* Per-axis raw calibration: [rest/heel .. full/toe] mapped to 0..127.
 * The built-in treadle (axis 2) was measured ~40..270 — it does NOT swing the
 * full 0..1023. External pedals vary by model and by the pedal's own polarity
 * switch, so the rear jacks default to the full range; run the helper (or the
 * standalone tool) with -v to read the raw axis values and set these to the
 * rest/full extremes you actually observe. */
static const int kAnalogRawMin[RK3_N_ANALOG] = {    0,    0,  40 };
static const int kAnalogRawMax[RK3_N_ANALOG] = { 1023, 1023, 270 };

/* ---- state ---------------------------------------------------------- */

struct rk3_midi {
	MIDIClientRef   client;
	MIDIEndpointRef source;

	int     have_buttons;
	uint8_t button_state[RK3_N_BUTTONS];   /* last sent 0/1 */

	int     have_analog;
	uint8_t analog_cc_val[RK3_N_ANALOG];   /* last sent 0..127 */
	int     analog_raw[RK3_N_ANALOG];      /* last raw, for -v on-change print */

	/* MIDI Beat Clock -> display decimal-point flash (rk3_midi_enable_clock_sync) */
	MIDIEndpointRef dest;                  /* virtual destination the DAW clocks  */
	int      led_fd;                       /* connected UDP -> engine LED port; -1 */
	unsigned clk;                          /* 0xF8 pulses since last downbeat reset */
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
		int off = a * 2;                          /* input.c:208 */
		if (off + 1 >= len)
			break;
		int raw = (p[off] << 8) | p[off + 1];     /* big-endian */

		/* Print every axis on raw change (even disabled ones) so the pedal
		 * jacks can be probed/calibrated with -v. Threshold drops ADC jitter. */
		if (verbose && (!m->have_analog ||
				abs(raw - m->analog_raw[a]) > 3)) {
			printf("  analog%d raw=%4d%s\n", a, raw,
			       kAnalogCC[a] ? "" : "  (axis disabled, no CC)");
			m->analog_raw[a] = raw;
		}

		if (kAnalogCC[a] == 0)                     /* axis disabled */
			continue;

		int lo = kAnalogRawMin[a], hi = kAnalogRawMax[a];
		int clamped = raw;
		if (clamped < lo) clamped = lo;
		if (clamped > hi) clamped = hi;
		uint8_t val = (hi > lo)
			? (uint8_t)((clamped - lo) * 127 / (hi - lo)) : 0;

		if (!m->have_analog || val != m->analog_cc_val[a]) {
			m->analog_cc_val[a] = val;
			send_cc(m, kAnalogCC[a], val);
			if (verbose)
				printf("    analog%d -> CC%u %d\n",
				       a, kAnalogCC[a], val);
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

/* ---- MIDI Beat Clock -> display decimal-point flash ------------------ */

/* "Con" glyphs (see rk3_engine.c): C=0x39, o=0x5C, n=0x54; bit7 = decimal pt.
 * Send a 3-byte control_state to the engine's LED port; the engine zero-pads to
 * a full frame and relays it to WRITE_IO. `on` lights the dot after the 'n'. */
static void led_flash_dot(struct rk3_midi *m, int on)
{
	if (m->led_fd < 0)
		return;
	uint8_t f[3] = { 0x39, 0x5C, (uint8_t)(on ? (0x54 | 0x80) : 0x54) };
	(void)send(m->led_fd, f, sizeof(f), 0);   /* best-effort; engine may drop */
}

/* CoreMIDI delivers clock on its own thread. System Real-Time bytes (>=0xF8)
 * are single-byte and never collide with data bytes (<0x80), so a flat scan of
 * every byte is safe. 24 clocks = one beat: dot ON at the beat, OFF ~2 clocks
 * later (a brief flash whose length tracks tempo but stays short). */
static void clock_read(const MIDIPacketList *pl, void *refCon, void *srcRef)
{
	(void)srcRef;
	struct rk3_midi *m = refCon;
	const MIDIPacket *p = &pl->packet[0];
	for (unsigned i = 0; i < pl->numPackets; i++) {
		for (UInt16 j = 0; j < p->length; j++) {
			switch (p->data[j]) {
			case 0xF8: {                       /* timing clock */
				unsigned phase = m->clk % 24;
				if (phase == 0)
					led_flash_dot(m, 1);
				else if (phase == 2)
					led_flash_dot(m, 0);
				m->clk++;
				break;
			}
			case 0xFA:                         /* start    */
			case 0xFB:                         /* continue */
				m->clk = 0;                /* align the downbeat */
				break;
			case 0xFC:                         /* stop     */
				m->clk = 0;
				led_flash_dot(m, 0);       /* leave a clean "Con" */
				break;
			default:
				break;
			}
		}
		p = MIDIPacketNext(p);
	}
}

int rk3_midi_enable_clock_sync(struct rk3_midi *m, const char *dest_name)
{
	if (!m)
		return -1;

	/* Connected UDP socket so led_flash_dot() can just send(). */
	m->led_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (m->led_fd >= 0) {
		struct sockaddr_in a;
		memset(&a, 0, sizeof(a));
		a.sin_family = AF_INET;
		a.sin_port = htons(RK3_LED_UDP_PORT);
		a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (connect(m->led_fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
			close(m->led_fd);
			m->led_fd = -1;
		}
	}

	CFStringRef cn = CFStringCreateWithCString(NULL, dest_name,
						   kCFStringEncodingUTF8);
	OSStatus st = MIDIDestinationCreate(m->client, cn, clock_read, m, &m->dest);
	if (cn)
		CFRelease(cn);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_enable_clock_sync: "
			"MIDIDestinationCreate failed (%d)\n", (int)st);
		return -1;
	}
	return 0;
}

struct rk3_midi *rk3_midi_open(const char *name)
{
	struct rk3_midi *m = calloc(1, sizeof(*m));
	if (!m)
		return NULL;
	m->led_fd = -1;                  /* calloc leaves 0 (= stdin); mark unset */

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
	if (m->led_fd >= 0)
		close(m->led_fd);
	if (m->dest)
		MIDIEndpointDispose(m->dest);
	if (m->source)
		MIDIEndpointDispose(m->source);
	if (m->client)
		MIDIClientDispose(m->client);
	free(m);
}
