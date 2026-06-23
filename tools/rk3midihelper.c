/*
 * rk3midihelper.c — user-space MIDI helper for the RigKontrol 3 driver.
 *
 * The coreaudiod sandbox blocks CoreMIDI (MIDIClientCreate -> -10833, confirmed
 * on hardware), so the audio engine cannot publish the foot controls as MIDI
 * itself. Instead the engine — which owns the USB interface while streaming —
 * forwards each raw caiaq EP1 control report over loopback UDP to THIS process,
 * a normal-user daemon that owns the CoreMIDI virtual source "RigKontrol 3" and
 * runs the exact same decode->CC logic as the standalone tools/rk3midi.c.
 *
 * It does NOT touch USB, so it coexists with the audio driver. It keeps the
 * CoreMIDI source up for its whole lifetime (so DAWs always see the input);
 * reports only flow while audio is streaming (the engine owns USB only then).
 *
 * Run manually for a quick test:   ./build/rk3midihelper [-v]
 * Or install it as a LaunchAgent:  make install-midihelper
 *
 * Build: make midihelper
 */

#include "../rk3_midi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define RK3_CAL_MS 6000          /* pedal-sweep capture window (ms) */

static volatile sig_atomic_t g_stop = 0;
static volatile sig_atomic_t g_reload = 0;
static void on_stop(int sig)   { (void)sig; g_stop = 1; }
static void on_reload(int sig) { (void)sig; g_reload = 1; }

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* Bind a UDP socket to 127.0.0.1:<port>. Returns fd or -1. */
static int bind_udp_loopback(int port)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
		return -1;
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	struct sockaddr_in a;
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 only */
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		close(fd);
		return -1;
	}
	return fd;
}

int main(int argc, char **argv)
{
	int verbose = 0;
	for (int i = 1; i < argc; i++)
		if (strcmp(argv[i], "-v") == 0)
			verbose = 1;

	struct rk3_midi *m = rk3_midi_open("RigKontrol 3");
	if (!m)
		return 1;

	/* Apply ~/.config/rk3/midi.conf if present (overrides the built-in
	 * defaults). Re-applied live on SIGHUP (see tools/rk3-reload-config). */
	rk3_midi_load_config(m, NULL);

	/* Publish a destination too: clock it from the DAW to blink the display's
	 * decimal point on the beat. Non-fatal — controls/MIDI still work without it. */
	if (rk3_midi_enable_clock_sync(m, "RigKontrol 3") != 0)
		fprintf(stderr, "warning: clock-sync destination unavailable; "
				"tempo flash disabled\n");

	int fd = bind_udp_loopback(RK3_MIDI_UDP_PORT);
	if (fd < 0) {
		perror("bind (midi port)");
		rk3_midi_close(m);
		return 1;
	}
	/* Wake recv() ~every 15 ms even when idle so HOLD/TAP timers can fire and
	 * the calibration window can close (the device only sends on change). */
	struct timeval tv = { 0, 15000 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

	/* Optional control port for rk3-calibrate. Non-fatal if it can't bind. */
	int cfd = bind_udp_loopback(RK3_CTL_UDP_PORT);
	if (cfd < 0)
		fprintf(stderr, "warning: control port %d unavailable; "
				"calibration disabled\n", RK3_CTL_UDP_PORT);

	/* sigaction WITHOUT SA_RESTART so a blocked recv() returns EINTR and the
	 * loop promptly sees g_stop / g_reload (signal() defaults to SA_RESTART). */
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	sa.sa_handler = on_stop;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sa.sa_handler = on_reload;
	sigaction(SIGHUP, &sa, NULL);    /* reload ~/.config/rk3/midi.conf */

	printf("RigKontrol 3 MIDI helper: CoreMIDI source + clock destination up; "
	       "listening on udp 127.0.0.1:%d. Play audio + use the controls.\n"
	       "Send MIDI Beat Clock to the 'RigKontrol 3' output to flash the "
	       "display on the beat. SIGHUP reloads the config.\n",
	       RK3_MIDI_UDP_PORT);
	fflush(stdout);

	int cal_axis = -1;
	uint64_t cal_deadline = 0;

	while (!g_stop) {
		if (g_reload) {
			g_reload = 0;
			fprintf(stderr, "rk3_midi: reloading config (SIGHUP)\n");
			rk3_midi_load_config(m, NULL);
		}

		uint8_t buf[512];
		ssize_t n = recv(fd, buf, sizeof(buf), 0);   /* blocks up to ~15 ms */
		if (n >= 1) {
			rk3_midi_dispatch(m, buf, (int)n, verbose);
		} else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
			   errno != EINTR) {
			perror("recv");
			break;
		}

		/* calibration requests: "CAL <axis>" (0=exp1, 1=exp2, 2=treadle) */
		if (cfd >= 0) {
			char cbuf[64];
			ssize_t cn = recv(cfd, cbuf, sizeof(cbuf) - 1, MSG_DONTWAIT);
			int ax;
			if (cn > 0) {
				cbuf[cn] = '\0';
				if (sscanf(cbuf, "CAL %d", &ax) == 1 &&
				    ax >= 0 && ax < 3) {
					rk3_midi_calib_begin(m, ax);
					cal_axis = ax;
					cal_deadline = now_ms() + RK3_CAL_MS;
					fprintf(stderr, "rk3_midi: calibrating axis %d — "
						"sweep the pedal fully for %d s...\n",
						ax, RK3_CAL_MS / 1000);
				}
			}
		}

		rk3_midi_tick(m, verbose);             /* HOLD/TAP long-press timers */

		if (cal_axis >= 0 && now_ms() >= cal_deadline) {
			rk3_midi_calib_finish(m, cal_axis, NULL, NULL);
			cal_axis = -1;
		}
	}

	printf("\nshutting down\n");
	close(fd);
	if (cfd >= 0)
		close(cfd);
	rk3_midi_close(m);
	return 0;
}
