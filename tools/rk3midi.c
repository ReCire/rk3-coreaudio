/*
 * rk3midi.c — standalone "RigKontrol 3 controls -> MIDI" bridge.
 *
 * Opens the RK3 over USB, runs the device-info handshake, tells the device to
 * auto-report its digital (buttons) and analog (pedal) state, then polls bulk
 * EP 0x81 IN and turns each control change into MIDI on a CoreMIDI virtual
 * source named "RigKontrol 3". Point Reaper (or any DAW) at that MIDI input.
 *
 * NOTE: this owns the whole USB interface, so it cannot run at the same time
 * as the audio driver (coreaudiod claims the interface while streaming). It is
 * the hardware-validation step; simultaneous audio+MIDI lives in the engine.
 *
 * Build: make rk3midi   Run: ./build/rk3midi [-q]   (-q = quiet, no console log)
 */

#include "../usb_iokit.h"
#include "../caiaq_protocol.h"
#include "../rk3_midi.h"

#include <stdio.h>
#include <string.h>
#include <signal.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Mirror of snd_usb_caiaq_set_auto_msg (device.c:311): tell the device to
 * push digital/analog/erp reports automatically. RK3 uses (1, 10, 0). */
static int set_auto_msg(struct rk3_usb *u, int digital, int analog, int erp)
{
	uint8_t tmp[3] = { (uint8_t)digital, (uint8_t)analog, (uint8_t)erp };
	return caiaq_send_command(u, EP1_CMD_AUTO_MSG, tmp, sizeof(tmp));
}

int main(int argc, char **argv)
{
	int verbose = 1;
	for (int i = 1; i < argc; i++)
		if (strcmp(argv[i], "-q") == 0)
			verbose = 0;

	struct rk3_usb u;
	struct caiaq_device_spec spec;

	printf("RigKontrol 3 -> MIDI bridge\n");

	if (rk3_usb_open(&u) != 0)
		return 1;

	/* Handshake so the device is in a known state (also confirms it's ours). */
	if (caiaq_get_device_info(&u, &spec) != 0) {
		rk3_usb_close(&u);
		return 1;
	}
	printf("device: fw %u, %u buttons-region, %u analog-in, midi %u/%u\n",
	       spec.fw_version, spec.num_digital_in,
	       spec.num_analog_in, spec.num_midi_in, spec.num_midi_out);

	struct rk3_midi *m = rk3_midi_open("RigKontrol 3");
	if (!m) {
		rk3_usb_close(&u);
		return 1;
	}

	if (set_auto_msg(&u, 1, 10, 0) != 0)
		fprintf(stderr, "warning: AUTO_MSG command failed\n");

	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);
	printf("Publishing CoreMIDI source \"RigKontrol 3\". "
	       "Press buttons / move pedal. Ctrl-C to quit.\n");

	/* Poll EP1 IN. Short timeout so SIGINT is responsive; timeouts are normal
	 * (the device only sends on change / at the auto-report interval). */
	while (!g_stop) {
		uint8_t buf[EP1_BUFSIZE];
		size_t len = sizeof(buf);
		int rc = rk3_bulk_read(&u, buf, &len, 100);
		if (rc == RK3_ETIMEOUT)
			continue;
		if (rc != 0)
			break;
		if (len >= 1)
			rk3_midi_dispatch(m, buf, (int)len, verbose);
	}

	printf("\nshutting down\n");
	rk3_midi_close(m);
	rk3_usb_close(&u);
	return 0;
}
