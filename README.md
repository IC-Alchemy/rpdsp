# rpdsp

Header-only, real-time C++17 audio DSP for Pico-DSP-Garden. Zero dependencies, no allocations in the audio path, and portable between desktop (x86/x64) and RP2040/RP2350. Every module is a standalone `.h`.

## Install

* **Arduino IDE:** Copy or symlink `rpdsp/` into your sketchbook's `libraries/` folder.
* **arduino-cli:** Pass `--library libraries/rpdsp` at compile time.

**Don't `#include <rpdsp.h>`**—it's just an Arduino discovery shim. Include the headers you need directly:

```cpp
#include <rpdsp/oscillator.h>
#include <rpdsp/filter.h>
```

## Quick start

```cpp
#include <rpdsp/oscillator.h>
#include <rpdsp/filter.h>

rpdsp::BSplineSawOsc osc;
rpdsp::StateVariableFilter filter;

void setup() {
  osc.prepare(48000.0f);
  osc.setFreq(220.0f);
  filter.prepare(48000.0f);
  filter.setCutoffResonance(1200.0f, 0.3f);
}

float renderSample() {
  return filter.process(osc.process()).lowpass;
}
```

Most modules follow the same lifecycle: construct, `prepare(sampleRate)` once, then `process(...)` per sample or block.

## What's in the box

* **Generators & Voices:** `oscillator.h` (naive + band-limited B-spline/sync/noise), `hypersaw.h`, `voice.h`, `waveguide.h` (Karplus-Strong).
* **Filters & Effects:** `filter.h` (one-pole, DC blocker, biquad, SVF), `ladder.h`, `effects.h`, `dark_reverb.h`, `delay_line.h`, `interpolation.h`.
* **Dynamics & Envelopes:** `dynamics.h` (envelope follower, mono/stereo compressor), `envelope.h` (self-contained `ADSR`).
* **Patchable Recipes (`DSPFunctions.h`):** Free-function DSP blocks (oscillators, filters, resonators, delays, granulator, freq shifter, LFOs, chaos) using caller-owned `float*` state. Optional class wrappers in `analog_adsr.h`, `bbd_delay.h`, `tape_delay.h`, `frequency_shifter.h`.
* **RP2350/RP2040 Hardware:** `hardware_interpolator.h` (integer-sample SIO wrappers).
* **Sequencing, UI & Utils:** `gate_pattern.h`, `rhythm_sequencer.h`, `parameter_track.h`, `scale_table.h`, `control_surface.h`, `knob_bank.h`, `pickup_knob.h`, `joystick_recorder.h`, `parameter_smoother.h`, `analysis.h`, `algorithm.h`, `realtime.h`, `config.h`.

Full API details, benchmarks, and hardware interpolator notes live in `Docs/`.

## Conventions & gotchas

* **Aliasing:** `SawOsc`, `SquareOsc`, and `TriangleOscillator` are naive. Use `BSpline*` or `HardSyncSaw` for band-limited output.
* **Defaults:** `RPDSP_BLOCK_SIZE` (`config.h`) must be `16`, `32`, or `64`. Default sample rate is `48000.0f`.
* **API exceptions:** Sample-rate-independent classes skip `prepare()`. A few use different method names (`LinearSmoother::next()`, `KarplusStrongVoice::pluck()`, `ParameterTrack::init()`, slider `scan()`). `StateVariableFilter` returns a struct, stereo reverbs return `std::array<float, 2>`, `Compressor::processStereo` modifies in place, and `hardware_interpolator.h` uses `int16_t`/`int32_t`. `SineOscillator` has no `setAmp()`, and `ADSR` is self-contained (doesn't pull in `config.h`).

## Free-function recipes (`DSPFunctions.h`)

No classes or `prepare()`—just pass a zero-init `float*` state array and `inc = freq / fs` (plus a `uint32_t*` RNG seed for random helpers). Timing-sensitive recipes default to 48 kHz; for other rates, precompute coefficients at setup/control rate:

```cpp
const auto comp = rpdsp::make_comp_feedback_coefficients(96000.0f);
float state[2]{};
float y = rpdsp::comp_feedback(input, 0.3f, 0.5f, comp, state);
```

Check the comments in `DSPFunctions.h` for exact state sizes and control bounds.
