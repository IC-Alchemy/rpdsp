# rpdsp

Header-only, real-time audio DSP for embedded C++17. Self-authored for
Pico-DSP-Garden — no external dependencies, no allocation in the audio path,
portable between host (x86/x64) and RP2040/RP2350.

Oscillators, filters, dynamics, effects, envelopes, a subtractive-synth voice,
sequencing/UI helpers, and RP2350 hardware-interpolator wrappers. Every module
is a plain `.h` you can include on its own.

## Installation

Arduino IDE: copy or symlink `rpdsp/` into your sketchbook's `libraries/`
directory. `arduino-cli`: pass `--library libraries/rpdsp` at compile time (no
copy needed) — this is how Pico-DSP-Garden's own examples consume it.

**Do not `#include <rpdsp.h>`.** The root `rpdsp.h` is an Arduino
library-discovery shim only, so `arduino-cli`/the IDE can detect the library
and add `src/` to the include path. Include the namespaced headers you need
directly:

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

Every module follows the same lifecycle: construct, `prepare(sampleRate)`
once, then call `process(...)` every sample (or every block). See
[conventions and exceptions](#conventions) below — a handful of modules
legitimately deviate from this shape, and it's worth knowing which before you
guess an API from habit.

## Module index

| Header | What's in it |
|---|---|
| `algorithm.h` | Free helper functions: `clamp`, `clamp01`, `lerp`, `wrap01`, `dbToGain`, `gainToDb`, `midiNoteToHz`, `softClip`, `fastTanh`, `toInt24x32`, equal-power pan, and coefficient helpers (`safeSampleRate`, `clampCutoff`, `onePoleSmooth`). Pulled in transitively by most other headers. |
| `analysis.h` | `ZeroCrossingPitchDetector`, `YinPitchDetector<WindowSize, MaxTau>`, `RmsPeakMeter`. |
| `analog_adsr.h` | `AnalogAdsr` — optional named wrapper around the `adsr_analog` recipe; owns gate/state and offers coefficient or seconds setters. |
| `bbd_delay.h` | `BbdDelay<Capacity>` — optional named wrapper around `delay_bbd`; owns fixed BBD storage. |
| `config.h` | `kDefaultSampleRate`, `kDefaultBlockSize` (`RPDSP_BLOCK_SIZE`, must be 16/32/64), `kPi`/`kTwoPi`. |
| `control_surface.h` | `MuxSliderScanner<N>`, `DirectAdcSliderScanner<N>`, `DebouncedButton`. |
| `delay_line.h` | `DelayLine<Capacity>` — circular buffer with linear/cubic fractional reads. |
| `DSPFunctions.h` | Free-function DSP recipes (`comp_feedback`, `filt_diodesvf`, `filt_vowel`, `osc_pdmorph`/`osc_fbfm`/`osc_chaosdrift`/`osc_morphtsq`/`osc_tzfm`/`osc_dsf`/`osc_formant`/`osc_revsync`, `res_tension`, `gtr_feedback`, `delay_bbd`/`delay_tape`, `fx_swarm`/`fx_diffuse`/`gran_cloud`/`fx_freqshift`, `ringmod_diode`, `pitch_octdown`, `adsr_analog`, `env_loopad`, `lfo_randcubic`, `chaos_lorenz`, `cv_wander`, `smooth_catchup`). Caller-owned `float*` state; `inc = freq/fs`. |
| `dynamics.h` | `EnvelopeFollower`, `CompressorStaticCurve`, `GainReductionSmoother`, `Compressor`. |
| `effects.h` | `Waveshaper`, `Delay<Capacity>`, `Chorus<Capacity>`, `CombFilter<Capacity>`, `AllpassFilter<Capacity>`, `SchroederReverb`, `StereoSchroederReverb`. |
| `envelope.h` | `ADSR`. |
| `filter.h` | `OnePoleLowpass`, `DcBlocker`, `BiquadLowpass`, `StateVariableFilter` (+ `StateVariableOutput`). |
| `frequency_shifter.h` | `FrequencyShifter` — optional named wrapper around `fx_freqshift`; accepts shift in Hz. |
| `gate_pattern.h` | `GatePattern<MaxSteps>` — step-mask trigger sequencer. |
| `hardware_interpolator.h` | RP2350/RP2040 SIO interpolator wrappers: `HardwareInterpolatorPool`, `HardwareWavefolder`, `HardwareBlend`, `HardwareClamp`, `HardwareOscillator`, `HardwareMorphOscillator`, `HardwareFoldClamp`. Integer (`int32_t`/`int16_t`) samples, not float. |
| `hypersaw.h` | `Hypersaw` — 7-voice Super Saw (Szabo model). |
| `joystick_recorder.h` | `JoystickRecorder<MaxPositions>` — record/loop an (x, y) motion path. |
| `knob_bank.h` | `KnobBank<NumBanks, NumKnobs>` — multi-bank pickup-aware parameter storage. |
| `ladder.h` | `LadderFilter` — 4-pole Huovilainen Moog ladder (LP/BP/HP, 12/24 dB/oct). |
| `oscillator.h` | `Phasor`, `SineOscillator`, `TriangleOscillator`, `SawOsc`, `SquareOsc` (naive/aliasing); `BSplineSawOsc`, `BSplineSquareOsc`, `HardSyncSaw` (band-limited); `NoiseOscillator`. |
| `parameter_smoother.h` | `LinearSmoother` — fixed-ramp-time parameter smoothing. |
| `parameter_track.h` | `ParameterTrack<T, MaxSteps>` — per-step automation storage for any type. |
| `pickup_knob.h` | `PickupKnob` — software takeover/pickup for a single control. |
| `realtime.h` | `zapDenormal`, `XorShift32` (fast deterministic PRNG). |
| `rhythm_sequencer.h` | `RhythmGateSequencer<MaxSteps>` — gate sequencer over a caller-owned pattern table. |
| `scale_table.h` | `kScaleTable`, `kScaleNames`, `scaleStepSemitones`, `scaleName` — 13 constexpr musical scales, no classes. |
| `tape_delay.h` | `TapeDelay<Capacity>` — optional named wrapper around `delay_tape`; owns fixed tape storage and protects its modulated read-head bounds. |
| `voice.h` | `TriggeredSynthVoice<MaxOscillators>` (+ `VoiceTrigger`, preset structs, `classicThreeSawSubtractivePreset()`, `noisePluckPreset()`). |
| `waveguide.h` | `KarplusStrongVoice<Capacity>` — Karplus-Strong plucked string. `ExtendedKarplusStrongVoice<Capacity>` — fractional-period tuning, brightness loop blend, pick-position tap, excitation modes, sympathetic string, body resonance. |

Full per-class API detail, costs, and caveats: [`Docs/algorithm_catalog.md`](../../Docs/algorithm_catalog.md).
Benchmarked host cost per algorithm: [`Docs/dsp_algorithm_benchmarks.md`](../../Docs/dsp_algorithm_benchmarks.md).
RP2350 hardware interpolator deep-dive: [`Docs/Hardware_Interpolators.md`](../../Docs/Hardware_Interpolators.md).

## Conventions

- **Lifecycle:** `prepare(sampleRate)` once, then `process(...)` per sample.
  Names are lowercase (`prepare`, not `Init`).
- **`SineOscillator` has no `setAmp`.** Scale the `process()` return value.
- **Phase convention:** `Phasor::process()` (and everything built on it)
  returns the phase/value *before* advancing.
- **Anti-aliasing:** the naive oscillators (`SawOsc`, `SquareOsc`,
  `TriangleOscillator`) alias at audible frequencies. Use the
  `SecondOrderBSpline*` classes or `HardSyncSaw` for band-limited output.
- **`RPDSP_BLOCK_SIZE`** (`config.h`) must be `16`, `32`, or `64`
  (`static_assert`-enforced). Default sample rate is `kDefaultSampleRate =
  48000.0f`; individual sketches may declare their own `SAMPLE_RATE`.

### Modules that don't follow `prepare()` → `process(float)`

Worth knowing before you guess an API from habit:

- **No `prepare()` needed** (sample-rate independent): `DelayLine`,
  `CombFilter`, `AllpassFilter`, `RmsPeakMeter`, `Waveshaper`,
  `DebouncedButton`, `JoystickRecorder`, `KnobBank`, `PickupKnob`,
  `RhythmGateSequencer`, `GatePattern`, `ParameterTrack`,
  `CompressorStaticCurve`. `NoiseOscillator` seeds via its constructor instead
  of a setter.
- **`process()` takes no sample argument** (self-contained generators):
  `ADSR::process()`, `TriggeredSynthVoice::process()`,
  `KarplusStrongVoice::process()`, `Hypersaw::process()`,
  `LinearSmoother::next()` (also renamed — no `process` at all).
- **`process()` returns a struct or array, not a float:**
  `StateVariableFilter::process()` → `StateVariableOutput{lowpass, bandpass,
  highpass}`; `StereoSchroederReverb::process(l, r)` → `std::array<float, 2>`.
- **Integer samples, not float:** everything in `hardware_interpolator.h`
  operates on `std::int32_t`/`std::int16_t`.
- **Callback-driven instead of `process()`:** `MuxSliderScanner::scan(...)`
  and `DirectAdcSliderScanner::scan(...)` take lambdas for
  select/delay/read.
- **Free functions, caller-owned `float*` state:** everything in
  `DSPFunctions.h` — oscillators, filters, resonators, delays, modulators,
  envelopes, modulation sources. No `prepare()`/class; pass a zero-init
  `float*` state array plus raw per-sample inputs (`inc = freq/fs`). RNG
  helpers (`lfo_randcubic`, `cv_wander`, `gran_cloud`) take a caller-owned
  non-zero-seeded `uint32_t*` LCG.
- **Optional named wrappers:** `TapeDelay<Capacity>`, `BbdDelay<Capacity>`,
  `FrequencyShifter`, and `AnalogAdsr` are separate headers that own the
  storage and conventional parameters for four recipes. They delegate to
  `DSPFunctions.h`; the patchable free-function recipe API remains intact.
  `TapeDelay` deliberately retains `delay_tape`'s 48 kHz wow/flutter law;
  `prepare(sampleRate)` is used for its milliseconds conversion.
- **Out-parameter style:** `HardwareOscillator::peekSample`/`nextSample` and
  `HardwareMorphOscillator::nextSample` write through a reference instead of
  returning a value.
- **Different trigger-method name:** `KarplusStrongVoice::pluck(freq, amp)`
  instead of `noteOn`/`trigger`; `ParameterTrack::init(...)` instead of
  `prepare`/`reset`.
- **`ADSR` is intentionally self-contained** — it defines its own local
  `kDefaultSampleRate`/`clamp01`/`lerp` rather than including `config.h` /
  `algorithm.h`. Don't assume it shares the shared constant if you change one.

## Gotchas

- **`Hypersaw::reseed()` matters for multiple instances.** The default PRNG
  seed is fixed, so two un-reseeded `Hypersaw`s produce identical phase
  spreads on every `trigger()`. Call `reseed()` once per instance with a
  distinct non-zero seed to decorrelate voices.
- **`ParameterTrack::resize()` growing after a shrink does not restore old
  history.** It only fills forward from the step count at the time of the
  call, using `defaultValue`, even if earlier steps held real data before a
  prior shrink.
- **`LadderFilter`'s block `process(float*, size_t)` overload is preferred**
  over the single-sample overload in the audio callback — it keeps filter
  state resident in FPU registers for the whole block rather than
  loading/storing every sample.
- **`LadderFilter::computeCoeffs` clamps cutoff to `[5, sampleRate * 0.425]`**
  — not `rpdsp::clampCutoff`'s usual 0.5 factor. This margin is required for
  the ladder's coefficient stability; don't substitute the shared helper.
- Two near-identical slider scanners exist in `control_surface.h`:
  `MuxSliderScanner` for one ADC pin behind a multiplexer, and
  `DirectAdcSliderScanner` for one dedicated ADC pin per slider. Pick based on
  your wiring, not naming — they share the same smoothing/quantization
  behavior.

## Host testing

`rpdsp` is portable C++17 and builds on the host (no Arduino/Pico headers
required for anything except `hardware_interpolator.h`, which self-guards via
`__has_include(<hardware/interp.h>)` and falls back to a software model off
target). From the repo root:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

`tests/` includes every header in a `test_compile_all` case, so a missing
transitive include fails the build before it ever reaches hardware.

## License

MIT — see the repository [LICENSE](../../LICENSE). Individual ported modules
(`ladder.h` from the Teensy Audio Library, `parameter_track.h` and
`scale_table.h` from Pico2Seq) carry attribution comments at the top of the
file; keep those if you redistribute.
