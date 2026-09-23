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
| `DSPFunctions.h` | Free-function DSP recipes (`comp_feedback`, `filt_diodesvf`, `filt_vowel`, `osc_pdmorph`/`osc_fbfm`/`osc_chaosdrift`/`osc_morphtsq`/`osc_tzfm`/`osc_dsf`/`osc_formant`/`osc_revsync`/`osc_prism`, `res_tension`/`res_braid`, `gtr_feedback`, `delay_bbd`/`delay_tape`, `fx_swarm`/`fx_diffuse`/`gran_cloud`/`fx_freqshift`/`fx_memoryfold`, `ringmod_diode`, `pitch_octdown`, `adsr_analog`, `env_loopad`, `lfo_randcubic`/`lfo_hesitate`, `chaos_lorenz`, `cv_wander`, `smooth_catchup`). Caller-owned `float*` state; `inc = freq/fs`. |
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
| `sitar.h` | `SitarStringVoice<Capacity>` — sitar physical model: Karplus-Strong course with jawari bridge buzz, taraf sympathetic resonator bank, two-resonator body, and meend pitch-slide. |
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
  helpers (`lfo_randcubic`, `cv_wander`, `gran_cloud`, `lfo_hesitate`) take a
  caller-owned `uint32_t*` LCG. Zero is a valid seed; use distinct seeds for
  distinct streams. State sizes and layouts are documented above each recipe.
- **Optional named wrappers:** `TapeDelay<Capacity>`, `BbdDelay<Capacity>`,
  `FrequencyShifter`, and `AnalogAdsr` are separate headers that own the
  storage and conventional parameters for four recipes. They delegate to
  `DSPFunctions.h`; the patchable free-function recipe API remains intact.
  `TapeDelay::prepare(sampleRate)` now configures milliseconds conversion,
  0.8 Hz wow, 6.3 Hz flutter, and the oxide filter time constant.
  `FrequencyShifter` caches its sine/cosine rotation in `setShiftHz()` and
  recomputes it in `prepare()`; neither wrapper evaluates trig/exp per sample.
- **Out-parameter style:** `HardwareOscillator::peekSample`/`nextSample` and
  `HardwareMorphOscillator::nextSample` write through a reference instead of
  returning a value.
- **Different trigger-method name:** `KarplusStrongVoice::pluck(freq, amp)`
  instead of `noteOn`/`trigger`; `ParameterTrack::init(...)` instead of
  `prepare`/`reset`.
- **`ADSR` is intentionally self-contained** — it defines its own local
  `kDefaultSampleRate`/`clamp01`/`lerp` rather than including `config.h` /
  `algorithm.h`. Don't assume it shares the shared constant if you change one.

### Recipe coefficients and compatibility

Existing recipe calls and state-array sizes remain valid. The short
`comp_feedback`, `filt_vowel`, and `delay_tape` calls keep their 48 kHz timing
defaults. To configure another rate, build coefficients at setup or when
controls change, then pass them immediately before the state pointer:

| Factory | Coefficient overload |
|---|---|
| `make_comp_feedback_coefficients(fs)` | `comp_feedback(x, threshold, amount, coeff, state)` |
| `make_filt_vowel_coefficients(vowel, fs)` | `filt_vowel(x, coeff, state)` |
| `make_delay_tape_coefficients(fs)` | `delay_tape(x, buffer, n, delay, wow, feedback, coeff, state)` |
| `make_fx_freqshift_coefficients(shiftHz / fs)` | `fx_freqshift(x, coeff, state)` |

For example, calculate these once and retain both coefficients and state:

```cpp
const auto compressor = rpdsp::make_comp_feedback_coefficients(96000.0f);
float compressorState[2]{};
// Per sample:
// float y = rpdsp::comp_feedback(input, 0.3f, 0.5f, compressor, compressorState);
```

`recipe_rate_at_sample_rate(coefficientAt48k, fs)` preserves a one-pole's
time constant. The coefficient structs expose their fields for custom
settings; observe each field's documented bounds when constructing them
manually. Factories use log/exp or trig and belong at setup/control rate.
Vowel coefficients scale the original Chamberlin tuning, which remains an
approximation. Other recipes with fixed follower/servo coefficients state
those per-sample rates in their comments.

Behavior corrections:

- `adsr_analog` releases immediately on gate-off, including during attack.
  Attack, decay, sustain and release are clamped to `[0,1]`.
- `delay_tape` clamps delay to `[1,n-2]` samples and limits wow to the room
  available on both sides. A wrapped read that rounds to `n` becomes index
  zero. Minimum buffer lengths are 2 for BBD, 3 for the diffuser, and 4 for
  tape/cloud; lengths above `2^24` are rejected because positions are floats.
  Invalid lengths return silence (dry input for the additive diffuser)
  without touching pointers. Valid calls require correctly sized storage;
  reset buffer and state together if its size changes.
- Oscillator fundamental increments are signed and clamped to `[-0.5,0.5]`.
  Phases use `wrap01`, including reverse motion, and sine cores use
  `sinNormalizedPhase`. This improves sine accuracy and can change the
  sound of strongly nonlinear/feedback patches. Warping, FM, sync, DSF,
  and folding still alias; oversampling is the caller's responsibility.
- The frequency-shifter carrier uses a true rotation instead of Euler's
  frequency-dependent angular error. Its existing polarity is preserved:
  positive shifts move partials down, negative shifts move them up. The
  coefficient factory gives the most accurate tuning. The original
  `fx_freqshift(x, inc, state)` call supports per-sample modulation using
  polynomial sine/cosine coefficients. The Hilbert pair's sideband
  rejection remains approximate, particularly near DC and Nyquist.
- Vowel 4 now reaches the exact final table entry. `gtr_feedback` needs
  four state floats; existing five-float arrays are still sufficient.

### Four additional patchable recipes

Each function below uses fixed caller-owned storage, needs no allocation,
and documents its state layout and control bounds in `DSPFunctions.h`.
Keep state between samples; zero it to reset. Pass finite controls and audio.

| Function | Sound or behavior | State |
|---|---|---|
| `osc_prism(inc, focus, spread, state)` | Move a spectral focus across six harmonics and widen it to blend neighbors. Partials fade near Nyquist. | `float[1]` |
| `res_braid(x, g1, g2, loss, couple, state)` | Two resonant modes exchange energy through a scattering junction, producing beating and split resonances. | `float[4]` |
| `fx_memoryfold(x, drive, memory, rate, state)` | A wavefolder with a moving bias that follows recent signal history. | `float[1]` |
| `lfo_hesitate(rate, linger, memory, rng, state)` | Correlated random targets with an adjustable pause followed by a smooth glide. | `float[4]` plus `uint32_t` RNG |

Example settings at 48 kHz, called once per sample. The two audio sources
are shown independently; send `osc`, `struck`, or `folded` to your mixer.
Compute the resonator tuning and time coefficients when controls change:

```cpp
#include <rpdsp/DSPFunctions.h>

float prismState[1]{}, braidState[4]{}, foldState[1]{}, gestureState[4]{};
uint32_t gestureSeed = 42;
const float fs = 48000.0f;
const float g1 = tanf(rpdsp::kPi * 220.0f / fs);
const float g2 = tanf(rpdsp::kPi * 331.0f / fs);
const float loss = 1.0f - expf(logf(0.001f) / (1.5f * fs)); // 1.5 s T60
const float follow = 1.0f - expf(-1.0f / (0.002f * fs));   // 2 ms memory

void renderExample(float impulse) { // e.g. one sample at 0.2, then zero
    float cv = rpdsp::lfo_hesitate(0.8f / fs, 0.3f, 0.4f,
                                 &gestureSeed, gestureState);
    float osc = rpdsp::osc_prism(110.0f / fs, 0.5f + 0.5f * cv,
                               0.35f, prismState);
    float struck = rpdsp::res_braid(impulse, g1, g2, loss, 0.002f, braidState);
    float folded = rpdsp::fx_memoryfold(osc, 3.0f, 0.65f, follow, foldState);
    // Route the desired signals to your output here.
}
```

`osc_prism` costs up to six polynomial sine evaluations per sample. Its
partial fade does not remove sidebands from rapid parameter modulation.
`fx_memoryfold` is a naive wavefolder and benefits from oversampling.
`res_braid` conserves energy in its rotations before applying loss;
continuous excitation can still build up a level above unity. Its `g1/g2`
describe the uncoupled pitches, which change when coupling is introduced.
Hardware CPU cost and sound still need measurement on the target board.

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
