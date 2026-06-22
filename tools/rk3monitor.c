/*
 * rk3monitor.c — standalone hardware direct-monitoring sweep tool.
 *
 * NI's RK3 supports hardware direct monitoring: the device routes its analog
 * input straight to its analog output internally, bypassing the host round-trip
 * for near-zero (<1ms) monitoring latency. RE'd from NIUSBAudioDriver.kext:
 *   setDirectMonitoringMode -> EP1 cmd 0x0e, one-byte payload (the "mode").
 *   getCurrentDirectMonitoringMode -> EP1 cmd 0x12, returns the current mode.
 * The mode-byte VALUES are not documented anywhere (not in the Linux driver),
 * so this tool exists to discover them empirically: plug a guitar into the RK3
 * input + headphones/monitors into its output, then sweep modes and listen.
 *
 * This owns the whole USB interface, so it CANNOT run while the audio driver
 * streams (coreaudiod claims the interface). It's the hardware-validation step;
 * once the right mode is known, the command moves into the engine.
 *
 * Usage:
 *   rk3monitor                 interactive: type a mode number + Enter to set it,
 *                              'g' to query current, 'q' to quit.
 *   rk3monitor <mode>          one-shot: set <mode>, read it back, exit.
 *   rk3monitor --hold <mode>   set <mode>, then stay alive until Ctrl-C.
 *   rk3monitor --sweep         auto-step modes 0..7, ~3s each, announce each.
 *   --params                   negotiate AUDIO_PARAMS (48k/24) first; try this
 *                              if monitoring is silent (codec may need configuring).
 *
 * Build: make rk3monitor
 */

#include "../usb_iokit.h"
#include "../caiaq_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* NOTE: the 0x12 (getCurrentDirectMonitoringMode) query STALLS the EP1 pipe on
 * RK3 firmware 11 — the device doesn't implement it. We only WRITE (0x0e); the
 * USB layer now self-clears a stall, but we avoid 0x12 entirely. Judge the
 * effect by EAR, not by readback. */
static int set_mode(struct rk3_usb *u, int mode)
{
	if (caiaq_set_direct_mon(u, (uint8_t)mode) != 0) {
		fprintf(stderr, "  set mode %d: FAILED (EP1 0x0e write error)\n", mode);
		return -1;
	}
	printf("  set mode %d (0x0e accepted) — listen now\n", mode);
	fflush(stdout);
	return 0;
}

int main(int argc, char **argv)
{
	/* params defaults ON: direct monitoring (mode >=1) is REJECTED by the
	 * device until AUDIO_PARAMS is negotiated (confirmed on fw 11). */
	int hold = 0, sweep = 0, params = 1, oneshot_mode = -1;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--hold") == 0 && i + 1 < argc)
			{ hold = 1; oneshot_mode = atoi(argv[++i]); }
		else if (strcmp(argv[i], "--sweep") == 0)
			sweep = 1;
		else if (strcmp(argv[i], "--params") == 0)
			params = 1;
		else if (strcmp(argv[i], "--no-params") == 0)
			params = 0;
		else if (argv[i][0] != '-')
			oneshot_mode = atoi(argv[i]);
		else {
			fprintf(stderr, "unknown arg: %s\n", argv[i]);
			return 2;
		}
	}

	struct rk3_usb u;
	struct caiaq_device_spec spec;

	printf("RigKontrol 3 — direct-monitoring (EP1 0x0e) sweep tool\n");
	printf("Plug guitar -> RK3 input, headphones/monitors -> RK3 output, then listen.\n\n");

	if (rk3_usb_open(&u) != 0)
		return 1;

	if (caiaq_get_device_info(&u, &spec) != 0) {
		rk3_usb_close(&u);
		return 1;
	}
	printf("device: fw %u, data_alignment %u\n", spec.fw_version, spec.data_alignment);

	if (params) {
		int bpp = caiaq_compute_bpp(48000, spec.data_alignment, 1);
		if (caiaq_set_audio_params(&u, 48000, 24, bpp) == 0)
			printf("audio params negotiated (48k/24, bpp=%d)\n", bpp);
		else
			printf("audio params negotiation failed (continuing anyway)\n");
	}

	printf("\n");

	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	int rc = 0;

	if (sweep) {
		printf("Auto-sweep modes 0..3 (~3s each). Listen for input passing to output.\n");
		printf("(fw 11: 0=off, 1=on are valid; >=2 stall — kept in sweep as a probe.)\n");
		for (int m = 0; m <= 3 && !g_stop; m++) {
			printf("[mode %d]\n", m);
			set_mode(&u, m);
			for (int t = 0; t < 30 && !g_stop; t++)
				usleep(100000);
		}
		printf("sweep done; restoring mode 0 (off)\n");
		set_mode(&u, 0);
	} else if (oneshot_mode >= 0 && !hold) {
		rc = set_mode(&u, oneshot_mode);
	} else if (hold) {
		set_mode(&u, oneshot_mode);
		printf("holding mode %d — play and listen. Ctrl-C to quit.\n", oneshot_mode);
		while (!g_stop)
			usleep(200000);
		printf("\nrestoring mode 0 (off)\n");
		set_mode(&u, 0);
	} else {
		printf("Interactive. Type a mode number + Enter to set it; 'q' = quit.\n");
		char line[64];
		while (!g_stop) {
			printf("mode> ");
			fflush(stdout);
			if (!fgets(line, sizeof(line), stdin))
				break;
			if (line[0] == 'q')
				break;
			if (line[0] == '\n')
				continue;
			set_mode(&u, atoi(line));
		}
		printf("restoring mode 0 (off)\n");
		set_mode(&u, 0);
	}

	rk3_usb_close(&u);
	return rc;
}
