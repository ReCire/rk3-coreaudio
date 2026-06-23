/*
 * rk3_midi.h — translate RigKontrol 3 physical controls into MIDI and
 * publish them as a CoreMIDI *virtual source* (so DAWs like Reaper see an
 * "RigKontrol 3" MIDI input).
 *
 * The RK3 does NOT emit MIDI on the wire. Its 9 foot buttons and expression
 * pedal arrive as caiaq vendor messages on bulk EP 0x81 IN (the same channel
 * the audio handshake uses), tagged EP1_CMD_READ_IO / EP1_CMD_READ_ANALOG.
 * This module decodes those payloads (logic ported from the Linux input.c)
 * and synthesizes Control Change messages.
 *
 * Transport-agnostic: feed it the raw EP1 reply buffers via rk3_midi_dispatch().
 * It owns only the CoreMIDI side, so it can be driven from a standalone tool
 * (tools/rk3midi.c) or, later, from inside the audio engine.
 *
 * Default mapping (edit the tables in rk3_midi.c):
 *   button 1..9  -> CC 20..28, value 127 on press / 0 on release, channel 1
 *   axis2 (built-in treadle) -> CC 11 (expression), 0..127, channel 1
 *   axis0 / axis1 (rear EXP pedal jacks) -> CC 12 / CC 13, 0..127, channel 1
 *   (run the tool/helper with -v to print raw axes for calibration)
 */

#ifndef RK3_MIDI_H
#define RK3_MIDI_H

#include <stdint.h>

/*
 * Loopback UDP port the audio engine forwards raw EP1 control reports to, and
 * the MIDI helper (tools/rk3midihelper.c) listens on. The coreaudiod sandbox
 * blocks CoreMIDI directly (MIDIClientCreate -> -10833, confirmed on hardware),
 * so the engine — which owns the USB interface while streaming — relays the
 * reports to a normal-user helper that owns the CoreMIDI source. Loopback UDP
 * is the transport: it is the one egress confirmed allowed from coreaudiod's
 * sandbox (the AF_UNIX datagram path is also allowed but UDP is simpler).
 */
#define RK3_MIDI_UDP_PORT 17890

/*
 * Loopback UDP port the audio engine LISTENS on for LED / 7-segment display
 * updates. The engine owns the one shared USB interface for life, so a user CLI
 * (tools/rk3led.c) cannot drive the LEDs directly while the driver is loaded; it
 * sends the 6-byte control_state as a datagram here and the engine relays it to
 * the device via EP1_CMD_WRITE_IO on its own run-loop thread. Mirror image of the
 * MIDI relay (which SENDS from the engine): here the engine RECEIVES. Whether the
 * coreaudiod sandbox permits bind()/recv (vs. the proven outbound send) is gated
 * empirically — the engine logs bind success/failure and is non-fatal either way.
 */
#define RK3_LED_UDP_PORT 17891

struct rk3_midi;

/*
 * Create a CoreMIDI client and a virtual source named `name` (e.g.
 * "RigKontrol 3"). Returns NULL on failure (message to stderr).
 */
struct rk3_midi *rk3_midi_open(const char *name);

/*
 * Also publish a virtual *destination* named `dest_name` that the DAW can send
 * MIDI Beat Clock to, and blink the RK3 display's decimal point on every beat.
 * Counts 0xF8 clock pulses (24 per quarter note), aligns the downbeat on
 * Start/Continue (0xFA/0xFB), clears the dot on Stop (0xFC). The beat flash is
 * authored as a "Con." control_state and sent to the audio engine over loopback
 * UDP (RK3_LED_UDP_PORT) — the engine owns USB and relays it to WRITE_IO.
 *
 * Optional; only the installed helper needs it (the standalone tool drives the
 * device directly). Returns 0 on success, -1 if the destination can't be made.
 */
int rk3_midi_enable_clock_sync(struct rk3_midi *m, const char *dest_name);

/* Tear down the virtual source/destination and client. Safe on NULL. */
void rk3_midi_close(struct rk3_midi *m);

/*
 * Feed one EP1 IN reply (including the leading command byte). Dispatches on
 * buf[0]: READ_IO -> buttons, READ_ANALOG -> pedal. Other commands ignored.
 * `verbose` (nonzero) prints decoded values to stdout (for the tool / probing).
 */
void rk3_midi_dispatch(struct rk3_midi *m, const uint8_t *buf, int len,
		       int verbose);

#endif /* RK3_MIDI_H */
