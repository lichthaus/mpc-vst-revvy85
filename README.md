# Revvy85

An effect plugin for Akai MPC standalones (Gen 2, 64-bit) that emulates the hardware of a 1985
digital reverb, the Yamaha REV7, and runs **your own dump of its V1.0 program EPROM** on it: an
HD6303R host CPU, two YM3804 DSPs and the YM3807 modulation chip, at the unit's 31.25 kHz.
The programs, the display and the keys are the firmware's own.

Not affiliated with or endorsed by Yamaha. REV7 is used here only to say which unit's firmware
the plugin runs.

**No firmware is included**, here or in any release. You need a REV7 and an EPROM reader.

## Your EPROM

1. Read the program EPROM (a 27256, 32 KB) of your own unit with an EPROM reader. Read it
   more than once and compare the reads.
2. Name the file `rev7_v1.0.bin` and copy it into `Content/roms/` inside the installed plugin
   folder (`VST2 Plugins/Lichthaus - VST - Revvy85/Content/roms/`), or add it in the Open MPC
   installer's "Your ROMs and files" step.
3. Load the plugin. It boots the firmware for about 5 seconds, then the display shows the
   current program.

Without the file the plugin passes audio through and the display says `Firmware missing`.
Only V1.0 has been tested (CRC-32 `43e5df55`). Another dump still runs, but the display shows
`Unknown EPROM` for a few seconds after loading, and some programs may sound wrong, because
two fitted constants (below) are only applied to the V1.0 DSP programs.

## Controls

PANEL page, laid out like the unit's front panel:

- **Display window:** the 2 x 16 LCD, the red MEMORY digits, the input meter (dB below the
  A/D's full scale) and the MONO / BP indicators.
- **Knob row:** INPUT LEVEL, the input filters (FILTER: HP, LP; SEM: FREQ, RES, MODE, MIX),
  MIXING and VOLUME. Touch a knob, then turn the MPC's data wheel. The MONO and SEM BP
  switches sit beside them.
- **Keys, as on the unit:** REV1-REV4, E/R1, E/R2, OTHERS, USER MEMORY; REV TIME, F1, INITIAL
  DELAY, F2, 1ST REF, F3, - and +; the keypad (0-9, ., MEMORY, STORE, RECALL, CLEAR, ENTER);
  MUTE, OUT PHASE, MIDI CTRL, BYPASS. A tap presses the key once; the LEDs follow the firmware.
- **MEMORY digits:** touch them and turn the data wheel to pick a memory (the number blinks).
  Push the wheel, or double-tap the digits, to recall it. Without a push the digits go back to
  the current memory after 4 s.
- **Q-Links** (four knobs, banked): bank 1 Input Level, Mixing, Volume, Headroom; bank 2 HP
  Freq, LP Freq, SEM Freq, SEM Res; bank 3 SEM Mode, SEM Mix, SEM BP, Mono.

LEVELS page: Program (memory 1-90, sent as a MIDI program change; it follows the unit when you
change programs from the keys), Mixing, Input Level, Volume, Headroom (operating level into
the emulated converters), Stages (On: the fitted analogue path; Off: digital only), Mono,
SEM BP, and the display as text.

The unit's 4 KB battery RAM (user memories, the current program and its edits, the MIDI table)
is saved with the MPC project; loading the project boots the firmware with it.

## What is emulated, and what is not

- **Emulated from the firmware's point of view:** the host CPU, the LCD, keys and LEDs, the
  DSPs' serial program loading and their datapath, the MIDI input, the battery RAM.
- **Fitted to measurements of one unit:** the analogue input/output path
  (`Content/output-stage.txt`, used when Stages is On), the YM3807 modulation chip (a
  behavioural model: triangle LFOs, delay and amplitude modulation) and two constants (a comb
  state scale in the reverb program, and the modulation depths). Reverbs, delays, early
  reflections, flange and chorus amplitude match the reference unit closely. Open: TREMOLO /
  SYMPHONIC and the chorus delay depth at fast rates.
- **Balance:** the model's left D/A word is exactly 4x the right one in every program; the
  plugin scales the left channel down by 4.
- **MUTE / BYPASS:** follow the firmware's relay outputs. **OUT PHASE** inverts the reverb.
- **The unit's input EQ is replaced** by a 12 dB/oct high-pass (off, then 20 Hz-2 kHz) and
  low-pass (off, then 18 kHz-500 Hz) and a SEM-style state-variable filter (30 Hz-12 kHz,
  resonance up to Q 25, MODE low-pass -> notch -> high-pass, BP for band-pass, MIX 0 = off),
  all before the A/D so they shape the reverb only.

CPU: about 6 % of one Cortex-A76 core (p99, Large Hall) on an MPC Key 37.

## Building

Needs the [MPC Plugin Builder SDK](https://github.com/inmusic-software/mpc-plugin-builder-sdk)
(its `tools/build.py` downloads Zig by itself), Python 3, [uv](https://docs.astral.sh/uv/) for
the skin generator, and macOS for the skin's fonts.

```sh
MPC_SDK=/path/to/mpc-plugin-builder-sdk plugin/Revvy85/make.sh [path/to/rev7_v1.0.bin]
```

The plugin folder is `plugin/Revvy85/build/Lichthaus - VST - Revvy85/`. With an EPROM given,
`make.sh` also runs the SDK's `host_sim` test, then removes the EPROM from the build output.
Never commit or publish a firmware file; `.gitignore` keeps `*.bin` out.

## Licence

MIT (`LICENSE`). The MPC Plugin Builder SDK is MIT, copyright inMusic Brands, Inc.
