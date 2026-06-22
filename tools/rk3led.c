/*
 * rk3led.c — standalone RigKontrol 3 LED / 7-segment display driver.
 *
 * The RK3 "display" is FOUR 7-segment digits plus 8 numbered LEDs and a pedal
 * LED. Every control is a boolean bit (no brightness on the RK3). You keep a
 * 6-byte control_state, set/clear bits, and send the WHOLE buffer each time via
 * EP1_CMD_WRITE_IO (0x5) on bulk EP 0x01 OUT. (The RK3 uses WRITE_IO, NOT
 * DIMM_LEDS — see Linux control.c control_put() default case.)
 *
 * Bit layout (from Linux control.c rk3_controller[]); bit pos -> byte pos/8,
 * mask 1<<(pos%8):
 *   byte0 = 7-seg digit 1   bits 0..7 = segments a,b,c,d,e,f,g,p(decimal point)
 *   byte1 = 7-seg digit 2   "
 *   byte2 = 7-seg digit 3   "
 *   byte3 = 7-seg digit 4   "
 *   byte4 = round LEDs      bits 0..7 = LED 1 .. LED 8
 *   byte5 = pedal LED       bit 0     = LED pedal
 *
 *        aaa
 *       f   b
 *       f   b
 *        ggg
 *       e   c
 *       e   c
 *        ddd  p
 *
 * THIS IS A HARDWARE-VALIDATION TOOL: the byte map above is reverse-engineered
 * from the Linux driver and has NOT been confirmed against real RK3 hardware on
 * this port. Use --walk to light every bit in order and check it against the
 * silkscreen; use --raw to poke exact bytes if the segment/digit order is off.
 *
 * Two transports, auto-selected:
 *   - DIRECT (default when the device is free): open the USB interface and send
 *     EP1_CMD_WRITE_IO ourselves. Only works when the audio driver is NOT loaded
 *     (the installed engine claims the one shared interface for life).
 *   - REMOTE (fallback / --remote): the engine owns USB, so we send the 6-byte
 *     control_state as a loopback UDP datagram to the engine's LED listener
 *     (127.0.0.1:RK3_LED_UDP_PORT); the engine relays it to the device. This is
 *     how you drive the LEDs while the driver is loaded (audio may be streaming).
 * Default: try DIRECT, fall back to REMOTE if the interface is claimed.
 *
 * Build: make rk3led
 * Run:
 *   ./build/rk3led --show 1234       4-digit string (0-9 A-F - _ space . dp)
 *   ./build/rk3led --show "8.8.8.8."  decimal points: a '.' after a digit lights p
 *   ./build/rk3led --leds 0xFF        round LEDs bitmask (bit0=LED1 .. bit7=LED8)
 *   ./build/rk3led --pedal on         pedal LED on/off
 *   ./build/rk3led --raw b0 b1 b2 b3 b4 b5   send 6 raw control_state bytes (hex)
 *   ./build/rk3led --walk             light every bit 0..47 in turn (~0.4s each)
 *   ./build/rk3led --off              all dark
 * Options combine; with no display option it just sends all-off.
 */

#include "../usb_iokit.h"
#include "../caiaq_protocol.h"
#include "../rk3_midi.h"          /* RK3_LED_UDP_PORT */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

/* control_state: 4 digit bytes + LED byte + pedal byte. */
#define RK3_CTRL_BYTES 6

/* 7-segment masks, bit0..6 = a,b,c,d,e,f,g, bit7 = decimal point. */
#define SEG_A 0x01
#define SEG_B 0x02
#define SEG_C 0x04
#define SEG_D 0x08
#define SEG_E 0x10
#define SEG_F 0x20
#define SEG_G 0x40
#define SEG_P 0x80

/* Font for 0-9, A-F, plus '-', '_', blank. Index by the helper below. */
static uint8_t seg_font(char c)
{
	switch (c) {
	case '0': return SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F;
	case '1': return SEG_B|SEG_C;
	case '2': return SEG_A|SEG_B|SEG_G|SEG_E|SEG_D;
	case '3': return SEG_A|SEG_B|SEG_G|SEG_C|SEG_D;
	case '4': return SEG_F|SEG_G|SEG_B|SEG_C;
	case '5': return SEG_A|SEG_F|SEG_G|SEG_C|SEG_D;
	case '6': return SEG_A|SEG_F|SEG_G|SEG_E|SEG_D|SEG_C;
	case '7': return SEG_A|SEG_B|SEG_C;
	case '8': return SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G;
	case '9': return SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G;
	case 'a': case 'A': return SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G;
	case 'b': case 'B': return SEG_F|SEG_E|SEG_G|SEG_C|SEG_D;        /* lowercase b */
	case 'c': case 'C': return SEG_A|SEG_F|SEG_E|SEG_D;
	case 'd': case 'D': return SEG_B|SEG_C|SEG_G|SEG_E|SEG_D;        /* lowercase d */
	case 'e': case 'E': return SEG_A|SEG_F|SEG_G|SEG_E|SEG_D;
	case 'f': case 'F': return SEG_A|SEG_F|SEG_G|SEG_E;
	case '-':           return SEG_G;
	case '_':           return SEG_D;
	case ' ':           return 0;
	default:            return 0;
	}
}

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* A LED sink is either a direct USB interface or a UDP socket to the engine. */
struct led_sink {
	struct rk3_usb *usb;   /* non-NULL => direct WRITE_IO */
	int             fd;    /* >=0      => loopback UDP to the engine listener */
};

static int send_state(struct led_sink *s, const uint8_t st[RK3_CTRL_BYTES])
{
	if (s->fd >= 0) {
		if (send(s->fd, st, RK3_CTRL_BYTES, 0) < 0) {
			fprintf(stderr, "UDP send failed: errno=%d\n", errno);
			return -1;
		}
		return 0;
	}
	int rc = caiaq_send_command(s->usb, EP1_CMD_WRITE_IO, st, RK3_CTRL_BYTES);
	if (rc != 0)
		fprintf(stderr, "WRITE_IO failed (rc=%d)\n", rc);
	return rc;
}

/* Open a connected UDP socket to the engine's loopback LED listener. */
static int open_remote(void)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		fprintf(stderr, "socket() failed: errno=%d\n", errno);
		return -1;
	}
	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(RK3_LED_UDP_PORT);
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (connect(fd, (struct sockaddr *)&to, sizeof(to)) != 0) {
		fprintf(stderr, "connect() failed: errno=%d\n", errno);
		close(fd);
		return -1;
	}
	return fd;
}

static void sleep_ms(unsigned ms)
{
	struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
	nanosleep(&ts, NULL);
}

/*
 * Parse a display string into the 4 digit bytes. A '.' immediately after a
 * character lights that digit's decimal point (so "8.8.8.8." fills all). Only
 * the first 4 displayable characters are used.
 */
static void parse_show(const char *s, uint8_t digits[4])
{
	int d = 0;
	digits[0] = digits[1] = digits[2] = digits[3] = 0;
	for (const char *p = s; *p && d < 4; p++) {
		if (*p == '.') {
			/* decimal point applies to the last emitted digit */
			if (d > 0)
				digits[d - 1] |= SEG_P;
			continue;
		}
		digits[d++] = seg_font(*p);
	}
}

static void usage(void)
{
	fprintf(stderr,
	    "usage: rk3led [--show STR] [--leds MASK] [--pedal on|off]\n"
	    "              [--raw b0 b1 b2 b3 b4 b5] [--walk] [--off]\n"
	    "  --show STR    up to 4 of 0-9 A-F - _ space; '.' lights prev digit's dp\n"
	    "  --leds MASK   round LEDs bitmask, bit0=LED1..bit7=LED8 (e.g. 0xFF, 255)\n"
	    "  --pedal STATE pedal LED on/off\n"
	    "  --raw 6 hex   send exact control_state bytes (overrides everything)\n"
	    "  --walk        light every bit 0..47 in sequence to map the hardware\n"
	    "  --off         all dark (default with no display option)\n"
	    "  --remote      force loopback-UDP to the engine (driver loaded)\n"
	    "  --local       force direct USB (driver NOT loaded; default tries this first)\n");
}

int main(int argc, char **argv)
{
	uint8_t st[RK3_CTRL_BYTES] = { 0 };
	uint8_t raw[RK3_CTRL_BYTES];
	int have_raw = 0, do_walk = 0;
	int force_remote = 0, force_local = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--remote")) {
			force_remote = 1;
		} else if (!strcmp(argv[i], "--local")) {
			force_local = 1;
		} else if (!strcmp(argv[i], "--show") && i + 1 < argc) {
			uint8_t dg[4];
			parse_show(argv[++i], dg);
			memcpy(st, dg, 4);
		} else if (!strcmp(argv[i], "--leds") && i + 1 < argc) {
			st[4] = (uint8_t)strtoul(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--pedal") && i + 1 < argc) {
			const char *v = argv[++i];
			st[5] = (!strcasecmp(v, "on") || !strcmp(v, "1")) ? 0x01 : 0x00;
		} else if (!strcmp(argv[i], "--raw")) {
			if (i + RK3_CTRL_BYTES >= argc) {
				fprintf(stderr, "--raw needs 6 hex bytes\n");
				return 2;
			}
			for (int b = 0; b < RK3_CTRL_BYTES; b++)
				raw[b] = (uint8_t)strtoul(argv[++i], NULL, 16);
			have_raw = 1;
		} else if (!strcmp(argv[i], "--walk")) {
			do_walk = 1;
		} else if (!strcmp(argv[i], "--off")) {
			memset(st, 0, sizeof(st));
		} else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage();
			return 0;
		} else {
			fprintf(stderr, "unknown arg: %s\n", argv[i]);
			usage();
			return 2;
		}
	}

	struct rk3_usb u;
	struct caiaq_device_spec spec;
	struct led_sink sink = { .usb = NULL, .fd = -1 };

	/*
	 * Pick the transport. DIRECT needs the USB interface to be free (driver not
	 * loaded). REMOTE sends datagrams to the engine's listener. Default: try
	 * DIRECT, and if the interface is busy/absent fall back to REMOTE so the same
	 * command works whether or not the driver is loaded.
	 */
	if (!force_remote && rk3_usb_open(&u) == 0) {
		if (caiaq_get_device_info(&u, &spec) != 0) {
			rk3_usb_close(&u);
			return 1;
		}
		sink.usb = &u;
		printf("transport: direct USB (fw %u, hw_subtype %u)\n",
		       spec.fw_version, spec.hw_subtype);
	} else {
		if (force_local) {
			fprintf(stderr, "--local requested but the USB interface is "
				"unavailable (driver loaded? device unplugged?)\n");
			return 1;
		}
		sink.fd = open_remote();
		if (sink.fd < 0)
			return 1;
		printf("transport: loopback UDP -> engine 127.0.0.1:%d\n",
		       RK3_LED_UDP_PORT);
	}

	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	if (do_walk) {
		/* Light each of the 48 bits alone, in order, so you can match the
		 * physical element to byte/bit and confirm the documented map. */
		printf("walk: lighting bit 0..47 (~0.4s each). Ctrl-C to stop.\n");
		for (int bit = 0; bit < RK3_CTRL_BYTES * 8 && !g_stop; bit++) {
			uint8_t w[RK3_CTRL_BYTES] = { 0 };
			w[bit / 8] = (uint8_t)(1u << (bit % 8));
			printf("  bit %2d -> byte%d bit%d  (state %02x %02x %02x %02x %02x %02x)\n",
			       bit, bit / 8, bit % 8,
			       w[0], w[1], w[2], w[3], w[4], w[5]);
			fflush(stdout);
			if (send_state(&sink, w) != 0)
				break;
			sleep_ms(400);
		}
		/* leave it dark */
		uint8_t z[RK3_CTRL_BYTES] = { 0 };
		send_state(&sink, z);
		if (sink.usb) rk3_usb_close(&u); else close(sink.fd);
		return 0;
	}

	const uint8_t *out = have_raw ? raw : st;
	printf("sending control_state: %02x %02x %02x %02x %02x %02x\n",
	       out[0], out[1], out[2], out[3], out[4], out[5]);
	int rc = send_state(&sink, out);

	if (sink.usb) rk3_usb_close(&u); else close(sink.fd);
	return rc == 0 ? 0 : 1;
}
