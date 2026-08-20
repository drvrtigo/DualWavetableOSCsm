# DualWavetableOSC-SM

A dual-channel wavetable oscillator firmware for the **Electrosmith Daisy Patch SM**, designed as a Eurorack module. Each channel outputs an independent wavetable voice with per-channel morph control, sub-oscillator, optional harmonic interval layering, analog oscillator drift, and a calibrated 1V/oct pitch tracking system.

*This is a work in progress; no guarantees this readme is fully accurate*

---

## Features

- **Dual independent oscillator channels** — each with its own V/oct CV input, morph knob, and sub-oscillator level
- **Wavetable morphing** — smooth bilinear interpolation across a multi-frame wavetable bank stored in SDRAM
- **Interval mode** — toggle to layer a second harmonically-quantized oscillator voice per channel (±octave, ±fifth, unison)
- **Sub-oscillator** — square wave at one octave below the main pitch, independently leveled per channel
- **Analog drift simulation** — per-voice low-rate jitter applied to oscillator frequencies for natural detuning
- **Moog ladder filter** — per-channel highpass-complemented filter with frequency tracking and mild saturation via `Overdrive`
- **Soft takeover** — pickup-style parameter locking with animated LED feedback to prevent value jumps when exiting Shift mode
- **Shift mode** — hold the Shift button to access per-channel octave transpose (±2 oct, 5 steps) and interval select via the existing knobs
- **Calibrated inputs** — uses a `CalibrationRuntime` system for accurate ADC-mapped V/oct pitch and knob readings

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| Platform | Electrosmith Daisy Patch SM |
| CV Inputs | CV_5, CV_7 (V/oct Ch1/Ch2); CV_6, CV_8 (shape CV Ch1/Ch2) |
| Knob Inputs | CV_1/CV_2 (shape/morph), CV_3/CV_4 (sub level) |
| CV Output | CV_OUT_2 (LED driver via voltage) |
| Buttons | B7 (Shift), B8 (Interval Mode toggle) |
| Memory | SDRAM (wavetable bank loaded at boot) |

---

## Signal Path

```
V/oct CV → CalibrationRuntime → pitch Hz
                                  │
              ┌───────────────────┼────────────────────┐
              ▼                   ▼                    ▼
       WavetableOsc[0]    WavetableOsc[1]         Oscillator (sub)
       (center voice)     (interval voice,        (square, oct down)
                           interval mode only)
              └───────────────────┼────────────────────┘
                                  ▼
                            Mix + level scale
                                  ▼
                          MoogLadder filter
                          (freq-tracked cutoff)
                                  ▼
                        Overdrive + output clip
                                  ▼
                            Audio Out [c]
```

---

## Controls

### Normal Mode

| Control | Function |
|---------|----------|
| Knob 1 / Knob 2 | Wavetable morph position (Ch1 / Ch2) |
| Knob 3 / Knob 4 | Sub-oscillator level (Ch1 / Ch2) |
| CV_5 / CV_7 | 1V/oct pitch input (Ch1 / Ch2) |
| CV_6 / CV_8 | Morph CV modulation (Ch1 / Ch2) |
| Toggle (B8) | Switch between Single and Interval mode |
| LED (CV_OUT_2) | Indicates mode: ~2.3V = single, ~4.6V = interval |

### Shift Mode (hold B7 ≥ 400ms)

| Control | Function |
|---------|----------|
| Knob 1 / Knob 2 | Octave transpose Ch1 / Ch2 (–2 to +2, 5 steps) |
| Knob 3 / Knob 4 | Harmonic interval Ch1 / Ch2 (–oct, –5th, unison, +5th, +oct) |
| LED | Animates to show which parameter is active; blinks on soft pickup |

---

## Project Structure

```
DualWavetableOSCsm/
├── main.cpp                  # Main firmware: audio callback, UI loop, oscillator logic
├── src/
│   └── calibration/          # CalibrationRuntime — ADC/CV calibration system
├── wavetables/
│   ├── wavetable_data_a.h    # Primary wavetable bank (SDRAM-loaded at boot)
│   └── wavetable_data_b.h    # Alternate wavetable bank (reserved)
├── Makefile                  # libDaisy/DaisySP build system
└── daisy_qspi.cfg            # OpenOCD config for QSPI flash programming
```

---

## Building & Flashing

This project uses the standard Electrosmith Makefile build system targeting libDaisy and DaisySP.

### Prerequisites

- [libDaisy](https://github.com/electro-smith/libDaisy) and [DaisySP](https://github.com/electro-smith/DaisySP) checked out as submodules or referenced in `lib/`
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- OpenOCD (for SWD flashing) or the Daisy bootloader (for DFU)

### Build

```bash
make
```

### Flash via SWD (ST-Link / J-Link)

```bash
make program
```

---

## Calibration

The `CalibrationRuntime` class (`src/calibration/`) maps raw Daisy ADC readings to calibrated CV voltages and knob values. Calibration data is stored in persistent flash. On first boot (or if no calibration data exists), the module uses default linear scaling.

To re-run calibration, refer to the calibration routine defined in `calibration_runtime.h`.

---

## Dependencies

- [libDaisy](https://github.com/electro-smith/libDaisy) — hardware abstraction for the Daisy platform
- [DaisySP](https://github.com/electro-smith/DaisySP) — DSP building blocks (`MoogLadder`, `Oscillator`, `Overdrive`, `Jitter`)

---

## License

This project does not currently specify a license. All rights reserved by the author unless otherwise noted.
