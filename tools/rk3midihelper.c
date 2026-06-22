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
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) { (void)sig; g_stop = 1; }

int main(int argc, char **argv)
{
	int verbose = 0;
	for (int i = 1; i < argc; i++)
		if (strcmp(argv[i], "-v") == 0)
			verbose = 1;

	struct rk3_midi *m = rk3_midi_open("RigKontrol 3");
	if (!m)
		return 1;

	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		perror("socket");
		rk3_midi_close(m);
		return 1;
	}
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	struct sockaddr_in a;
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(RK3_MIDI_UDP_PORT);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 only */
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		perror("bind");
		close(fd);
		rk3_midi_close(m);
		return 1;
	}

	/* Use sigaction WITHOUT SA_RESTART (signal() defaults to SA_RESTART on
	 * macOS/BSD, which auto-restarts the blocking recv() so the g_stop flag is
	 * never re-checked — Ctrl-C then appears to hang). With sa_flags=0 the recv()
	 * returns EINTR on a signal and the loop exits cleanly. */
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_sig;
	sa.sa_flags = 0;                 /* no SA_RESTART: let recv() see EINTR */
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	printf("RigKontrol 3 MIDI helper: CoreMIDI source up; "
	       "listening on udp 127.0.0.1:%d. Play audio + use the controls.\n",
	       RK3_MIDI_UDP_PORT);
	fflush(stdout);

	while (!g_stop) {
		uint8_t buf[512];
		ssize_t n = recv(fd, buf, sizeof(buf), 0);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			perror("recv");
			break;
		}
		if (n >= 1)
			rk3_midi_dispatch(m, buf, (int)n, verbose);
	}

	printf("\nshutting down\n");
	close(fd);
	rk3_midi_close(m);
	return 0;
}
