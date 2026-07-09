# rk3-coreaudio

A userspace **macOS audio + MIDI driver for the Native Instruments RigKontrol 3**
(USB VID `0x17cc` / PID `0x1940`), built for Apple Silicon.

<img width="900" height="687" alt="image" src="https://github.com/user-attachments/assets/88b9b720-f3f7-4763-ba35-023e423bfd19" />


NI never shipped an Apple-Silicon driver for the RigKontrol 3 — the official
package is a 32-bit-era kext that no longer loads on modern macOS. This project
reimplements the device protocol (originally from the Linux `snd-usb-caiaq`
kernel driver) as a modern **CoreAudio AudioServer plugin** plus a small CoreMIDI
helper, so the RK3 works as a normal sound card and MIDI foot controller again.

What you get:

- **2-in / 2-out USB audio** at 44.1 / 48 / 96 / 192 kHz, switchable live in
  Audio MIDI Setup. Appears as a standard CoreAudio device usable by Reaper,
  the system, or any audio app.
- **Round-trip latency** — ~12–15 ms loopback at 48 kHz / 64-frame buffer. This
  is **not yet low and needs improvement** (see `docs/HANDOFF_LATENCY2.md`).
- **MIDI foot controls** — the 9 foot buttons and the pedals are published as a
  CoreMIDI source named **"RigKontrol 3"** (buttons → CC20–28; built-in treadle →
  CC11; the two rear expression-pedal jacks → CC12 / CC13). Always on; works with
  or without audio streaming. The display shows `Con`. The mapping is
  **configurable** without a UI — see *MIDI mapping config* below.
- **Tempo flash** — the helper also exposes a CoreMIDI *destination* named
  **"RigKontrol 3"**. Send it MIDI Beat Clock from your DAW and the display's
  decimal point blinks on the beat (`Con` → `Con.`), aligned to the transport's
  Start/Continue and cleared on Stop.
- **LED / 7-segment control** over a loopback socket (see `tools/rk3led`).

> **Unofficial / unaffiliated.** "RigKontrol" and "Native Instruments" are
> trademarks of Native Instruments GmbH. This project is an independent,
> community-built driver and is **not** produced, endorsed by, or affiliated
> with Native Instruments. Use at your own risk.

## Requirements

- Apple Silicon Mac (built with `-mmacosx-version-min=12.0`; macOS 12 Monterey or
  later).
- A RigKontrol 3 unit.
- For building from source: the Xcode Command Line Tools (`xcode-select --install`).

## Install from a prebuilt release

> The driver bundle is **ad-hoc signed**, not notarized. macOS quarantines files
> downloaded from the internet, and sandboxed `coreaudiod` refuses to load a
> quarantined ad-hoc bundle — so you must strip the quarantine attribute. The
> `install.sh` script does this for you.

1. Download `rk3-coreaudio-macos.zip` from the
   [Releases](../../releases) page and unzip it.
2. In Terminal, from the unzipped folder:

   ```sh
   ./install.sh
   ```

   It will (with `sudo`, so you'll be prompted for your password):
   - strip the `com.apple.quarantine` attribute,
   - copy `RigKontrol3.driver` to `/Library/Audio/Plug-Ins/HAL/`,
   - re-sign it in place (required — see note below),
   - install the MIDI helper to `/usr/local/bin` + a per-user LaunchAgent,
   - restart `coreaudiod`.

3. Plug in the RigKontrol 3. It should appear in **Audio MIDI Setup** as a
   2-in/2-out device, the display should read `CON`, and a **"RigKontrol 3"**
   MIDI source should be available to your DAW.

To remove everything:

```sh
./uninstall.sh
```

## Build from source

```sh
git clone <this repo>
cd rk3-coreaudio

make test            # codec unit test, no hardware needed
make driver          # build build/RigKontrol3.driver
sudo make install-driver
make midihelper && sudo make install-midihelper   # MIDI foot controls
```

Useful targets (see the `Makefile` for the full list):

| Target | What it does |
| --- | --- |
| `make driver` / `sudo make install-driver` | build / install the audio driver |
| `make midihelper` / `sudo make install-midihelper` | build / install the CoreMIDI helper |
| `make uninstall-driver`, `uninstall-midihelper` | remove them |
| `make driver-log`, `midihelper-log` | tail the live os_log / helper log |
| `make all` | build the hardware probe/diagnostic tools |
| `make iotest` | build the loopback audio test tool |
| `make test` | run the codec unit test (no hardware) |

## MIDI mapping config

The built-in mapping (buttons → CC20–28, treadle → CC11, rear jacks → CC12/13,
all momentary) works out of the box. To change it, drop a config file at
`~/.config/rk3/midi.conf` — the helper reads it at startup and again whenever you
run `rk3-reload-config`, so edits apply live without a restart:

```sh
mkdir -p ~/.config/rk3
cp docs/midi.conf.example ~/.config/rk3/midi.conf
$EDITOR ~/.config/rk3/midi.conf
rk3-reload-config
```

A missing file is fine — everything falls back to the built-in default, and bad
lines are logged (`make midihelper-log`) and skipped. The file lets you set:

- **Output channel** (`channel 1..16`).
- **Per-button behaviour** by the *printed* P1..P9 label (firmware bit-scramble is
  handled for you): `momentary` (default), `toggle` (press flips a latch),
  `hold` (momentary CC plus a second CC when held past `time` ms), `tap` (quick
  tap vs hold fire different CCs, mutually exclusive), or `off`.
- **Per-pedal mapping** for `treadle` / `exp1` / `exp2`: continuous `cc`, raw-ADC
  `calibMin`/`calibMax`, and an optional derived on/off `switch` CC with
  `son`/`soff` hysteresis thresholds.

Calibrate a pedal's travel with `rk3-calibrate <1|2|3>` (1 = treadle, 2 = EXP1,
3 = EXP2): it asks the running helper to capture the raw min/max while you sweep
the pedal, then writes `calibMin`/`calibMax` into the config and applies them
live. See `docs/midi.conf.example` for the full annotated syntax.

(`rk3-reload-config` and `rk3-calibrate` are installed to `/usr/local/bin` by
`make install-midihelper`.)

### Why the in-place re-sign matters

A HAL plugin is loaded by a **sandboxed** `coreaudiod` helper. If you copy a new
bundle *over* an existing one, stale code pages remain and the helper is
`SIGKILL`ed at `dlopen` with a code-signature/invalid-page error — even though
`codesign -vv` reports "valid on disk". The install path always does
`rm -rf` → copy → `codesign --force --sign -` **in place** to avoid this. Keep
that ordering if you script your own install.

## Troubleshooting

- **Device doesn't appear / no audio:** `make driver-log` and look for USB claim
  or codesign errors. Unplug/replug the RK3 if the interface got wedged.
- **`coreaudiod` pegged at 100% CPU:** quit audio apps and reboot; never
  `killall coreaudiod` or replug the device mid-stream while it's wedged.
- **No MIDI:** confirm the helper is running (`make midihelper-log`), and that
  the "RigKontrol 3" source is enabled in your DAW. MIDI works with or without
  an active audio stream.

## How it works

- `usb_iokit.c`, `caiaq_protocol.c`, `codec_mode2.c` — USB transport (IOUSBLib),
  the caiaq vendor protocol, and the mode-2 sample codec.
- `rk3_engine.c` — the isochronous audio engine (capture + free-running output
  rings, the OUT 4-byte phase seed, the fill servo) plus the MIDI/LED relay.
- `plugin/rk3_device.c` — the AudioServerPlugin (device, streams, volume/mute,
  multi-rate, clock).
- `rk3_midi.c` + `tools/rk3midihelper.c` — decode foot controls → CC and publish
  the CoreMIDI source from a normal-user LaunchAgent (the sandboxed driver
  cannot publish CoreMIDI itself). The helper also publishes the matching
  destination and turns incoming MIDI Beat Clock into the display's beat flash,
  which it sends to the engine over the same loopback LED socket.

To use the tempo flash, enable the **"RigKontrol 3"** output in your DAW and turn
on its "send MIDI clock" option (in Reaper: *Preferences → MIDI Devices*), then
start playback.

Design notes and the reverse-engineering history are in [`docs/`](docs/).

## License

GPL-2.0. This driver reimplements protocol logic derived from the
GPL-2.0-licensed Linux `snd-usb-caiaq` driver; see [LICENSE](LICENSE).
