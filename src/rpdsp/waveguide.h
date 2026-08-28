#pragma once

#include "algorithm.h"
#include "delay_line.h"
#include "realtime.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace rpdsp {

template <size_t Capacity>
class KarplusStrongVoice {
 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    reset();
  }

  void reset() {
    delay_.reset();
    periodSamples_ = Capacity / 2;
    last_ = 0.0f;
    active_ = false;
  }

  void setDecay(float decay) { decay_ = clamp(decay, 0.0f, 0.9999f); }

  void pluck(float frequencyHz, float amplitude = 1.0f) {
    const float freq = std::max(1.0f, frequencyHz);
    // Period length sets pitch; clamp to the fixed delay so plucks cannot allocate or overrun.
    periodSamples_ = static_cast<size_t>(clamp(sampleRate_ / freq, 2.0f, static_cast<float>(Capacity - 1)));
    const float amp = std::max(0.0f, amplitude);
    delay_.reset();
    // Initial noise is the classic string excitation; the feedback loop turns it into pitch.
    for (size_t i = 0; i < periodSamples_; ++i) {
      delay_.push(rng_.nextBipolar() * amp);
    }
    last_ = 0.0f;
    active_ = true;
  }

  float process() {
    if (!active_) {
      return 0.0f;
    }
    const float current = delay_.read(periodSamples_ - 1);
    // Two-point averaging is the string loss filter; decay controls how quickly energy dies.
    const float next = decay_ * 0.5f * (current + last_);
    delay_.push(next);
    last_ = current;
    if (std::fabs(next) < 1.0e-5f && std::fabs(current) < 1.0e-5f) {
      active_ = false;
    }
    return current;
  }

  [[nodiscard]] bool isActive() const { return active_; }

 private:
  float sampleRate_ = kDefaultSampleRate;
  float decay_ = 0.996f;
  float last_ = 0.0f;
  bool active_ = false;
  size_t periodSamples_ = Capacity / 2;
  DelayLine<Capacity> delay_;
  XorShift32 rng_{0xA511E9B3u};
};

// ---------------------------------------------------------------------------
// PluckedStringVoice — extended (Jaffe–Smith style) Karplus-Strong course.
//
// Everything KarplusStrongVoice cannot do, on a control (see
// AGENTS/plucked_expansion_plan.md for the design rationale):
//
//   * Fractional loop delay (interpolated read) + loop-filter phase-delay
//     compensation, so tuning stays accurate into the top octaves instead of
//     quantizing to whole samples.
//   * Decay expressed as T60 seconds; the per-period loop gain is derived
//     from the period, so "3 s sustain" means 3 s at every pitch.
//   * Adjustable loop damping: a one-pole lowpass (300 Hz..12 kHz) replaces
//     the fixed two-point averager. Dark nylon to glassy harpsichord.
//   * Pluck shaping at excitation time: pick position (feedforward comb at
//     pos*period — nulls the partials a pluck at that point cannot excite)
//     and pick hardness (one-pole lowpass on the noise burst, with makeup
//     gain so soft picks stay audible).
//   * Stiffness: two first-order allpasses inside the loop disperse upper
//     partials sharp, from perfectly harmonic to bell/piano-like.
//   * A two-string course: both strings run complete, independently excited
//     loops detuned +/- half of setDetuneCents() — 12-string shimmer and
//     slow chorusing sustain instead of a static tail.
//
// Realtime contract: fixed storage, no allocation. expf/powf appear only in
// the control-rate setters and pluck(); process() is two interpolated reads,
// two one-pole updates, four allpass updates, two pushes.
//
// Tuning note: the phase compensation uses the loop filters' DC group delay,
// which is accurate for fundamentals well below the damping cutoff. At the
// extreme corner (very dark + very high pitch) the loop delay clamps and the
// top notes ride sharp — musically moot, since that corner is also nearly
// silent at the fundamental.
//
// KarplusStrongVoice above is intentionally untouched: it is the simple,
// cheap classic, and existing worlds (SonicGarden PluckedGarden) keep their
// exact sound.
// ---------------------------------------------------------------------------
template <size_t Capacity>
class PluckedStringVoice {
 public:
  static constexpr int kNumStrings = 2;

  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    setBrightness(brightness_);
    setPickHardness(hardness_);
    reset();
  }

  void reset() {
    // Distinct seeds so the course's two strings get independent excitation
    // noise (a doubled course, not one string played twice). Seed pairs are
    // NOT interchangeable: all xorshift32 states share one orbit, and a
    // pathologically placed pair can produce anti-correlated streams whose
    // fundamentals cancel in the course sum (0xA511E9B3/0x7F4A7C15 does
    // exactly that — caught by the pick-position host test, which fails
    // loudly on such a pair; keep it passing when changing these).
    strings_[0].rng = XorShift32{0xA511E9B3u};
    strings_[1].rng = XorShift32{0x9E3779B9u};
    for (auto& s : strings_) {
      s.delay.reset();
      s.lp = 0.0f;
      s.apX1 = s.apY1 = s.apX2 = s.apY2 = 0.0f;
      s.active = false;
      s.quiet = 0;
      s.periodSamples = static_cast<float>(Capacity) * 0.5f;
      s.loopGain = 0.995f;
      s.loopDelay = s.periodSamples - 1.0f;
    }
  }

  // --- Control-rate setters (call from updateControls(), not per sample) ---

  // Time for the tail to fall 60 dB, independent of pitch. 0.05..10 s.
  void setDecayTimeSeconds(float t60) {
    t60_ = clamp(t60, 0.05f, 10.0f);
    for (auto& s : strings_) updateLoopGain(s);
  }

  // 0 = dark (300 Hz loop damping), 1 = bright (12 kHz). Exponential map.
  void setBrightness(float brightness01) {
    brightness_ = clamp01(brightness01);
    const float cutoffHz = 300.0f * powf(40.0f, brightness_);
    lpCoeff_ = 1.0f - expf(-kTwoPiLocal * cutoffHz / sampleRate_);
    for (auto& s : strings_) {
      updateLoopDelay(s);
      updateLoopGain(s);  // damping loss at the fundamental is compensated
    }
  }

  // Where the pick meets the string: 0.02 (bridge, thin/nasal) .. 0.5
  // (middle, hollow). Takes effect on the next pluck().
  void setPickPosition(float position01) {
    pickPosition_ = clamp(position01, 0.02f, 0.5f);
  }

  // 0 = soft felt (muted excitation), 1 = hard pick (full-bandwidth noise).
  // Takes effect on the next pluck().
  void setPickHardness(float hardness01) {
    hardness_ = clamp01(hardness01);
    const float cutoffHz = 500.0f * powf(24.0f, hardness_);
    excCoeff_ = 1.0f - expf(-kTwoPiLocal * cutoffHz / sampleRate_);
    // Makeup so a soft (heavily lowpassed) burst keeps comparable loudness.
    const float rms = sqrtf(excCoeff_ / (2.0f - excCoeff_));
    excMakeup_ = clamp(1.0f / (rms + 1.0e-6f) * 0.35f, 0.7f, 3.0f);
  }

  // 0 = perfectly harmonic, 1 = strongly dispersive (piano/bell sharpening).
  void setStiffness(float stiffness01) {
    apCoeff_ = -0.62f * clamp01(stiffness01);
    for (auto& s : strings_) updateLoopDelay(s);
  }

  // Spread between the two strings of the course, in cents (0..30). Each
  // string takes half the detune either side. Takes effect on next pluck().
  void setDetuneCents(float cents) { detuneCents_ = clamp(cents, 0.0f, 30.0f); }

  // --- Events ---------------------------------------------------------------

  void pluck(float frequencyHz, float amplitude = 1.0f) {
    const float freq = clamp(frequencyHz, 20.0f, sampleRate_ * 0.25f);
    const float amp = std::max(0.0f, amplitude);
    const float half = detuneCents_ * 0.5f;
    pluckString(strings_[0], freq * powf(2.0f, -half / 1200.0f), amp);
    pluckString(strings_[1], freq * powf(2.0f, half / 1200.0f), amp);
  }

  float process() {
    float out = 0.0f;
    for (auto& s : strings_) {
      if (!s.active) continue;
      const float y = s.delay.readLinear(s.loopDelay);
      // Loop damping: adjustable one-pole lowpass (the "string loss").
      s.lp += lpCoeff_ * (y - s.lp);
      // Dispersion: two first-order allpasses sharpen the upper partials.
      const float t = apCoeff_ * s.lp + s.apX1 - apCoeff_ * s.apY1;
      s.apX1 = s.lp;
      s.apY1 = t;
      const float u = apCoeff_ * t + s.apX2 - apCoeff_ * s.apY2;
      s.apX2 = t;
      s.apY2 = u;
      s.delay.push(u * s.loopGain);
      out += y;
      // Deactivate after a sustained stretch below audibility (also keeps
      // the recirculating values out of denormal territory).
      if (std::fabs(y) < 1.0e-5f) {
        if (++s.quiet > 4096) s.active = false;
      } else {
        s.quiet = 0;
      }
    }
    return out;
  }

  [[nodiscard]] bool isActive() const {
    return strings_[0].active || strings_[1].active;
  }

 private:
  static constexpr float kTwoPiLocal = 6.28318530718f;

  struct StringState {
    DelayLine<Capacity> delay;
    float periodSamples = 256.0f;  // placeholder until the first pluck
    float loopDelay = 2.0f;
    float loopGain = 0.995f;
    float lp = 0.0f;
    float apX1 = 0.0f, apY1 = 0.0f, apX2 = 0.0f, apY2 = 0.0f;
    int quiet = 0;
    bool active = false;
    XorShift32 rng{0xA511E9B3u};
  };

  // Per-period gain for the requested T60 at this string's period.
  void updateLoopGain(StringState& s) {
    // g^(T60*fs/P) = 10^-3  =>  g = 10^(-3*P/(T60*fs))
    const float exponent = -3.0f * s.periodSamples / (t60_ * sampleRate_);
    float g = powf(10.0f, exponent);
    // The loop damping lowpass also attenuates the fundamental once per
    // period; divide its magnitude there back out so the requested T60
    // survives dark brightness settings. Clamped: a very dark string may
    // decay faster than asked, but can never go unstable.
    const float w0 = kTwoPiLocal / s.periodSamples;
    const float re = 1.0f - (1.0f - lpCoeff_) * cosf(w0);
    const float im = (1.0f - lpCoeff_) * sinf(w0);
    const float dampingMag = lpCoeff_ / sqrtf(re * re + im * im);
    g /= std::max(dampingMag, 0.05f);
    s.loopGain = clamp(g, 0.0f, 0.99995f);
  }

  // Fractional read length = period minus the structural 1-sample delay of
  // the read/push loop and the DC group delay of the loop filters.
  void updateLoopDelay(StringState& s) {
    const float lpDelay = (1.0f - lpCoeff_) / lpCoeff_;
    const float apDelay =
        2.0f * (1.0f - apCoeff_) / (1.0f + apCoeff_);  // two stages
    s.loopDelay = clamp(s.periodSamples - 1.0f - lpDelay - apDelay, 2.0f,
                        static_cast<float>(Capacity - 2));
  }

  void pluckString(StringState& s, float freq, float amp) {
    s.periodSamples =
        clamp(sampleRate_ / freq, 4.0f, static_cast<float>(Capacity - 2));
    updateLoopGain(s);
    updateLoopDelay(s);
    s.delay.reset();
    s.lp = 0.0f;
    s.apX1 = s.apY1 = s.apX2 = s.apY2 = 0.0f;

    // Excitation: one period of noise, lowpassed by pick hardness, then a
    // CIRCULAR pick-position comb (x[i] - 0.9*x[(i - D) mod N]). Circular
    // matters: the burst becomes the string's periodic state, so wrapping
    // the tap applies the position notches to the whole excitation instead
    // of only its second half. Two passes through the delay line itself —
    // pass 1 pushes the raw burst, pass 2 reads it back at the two taps and
    // pushes the combed samples; the loop then only ever reads the combed
    // region (loopDelay < N), and the raw region is overwritten within the
    // first period. No scratch buffer, no allocation.
    const size_t burst = static_cast<size_t>(s.periodSamples);
    size_t combDelay = std::max<size_t>(
        1, static_cast<size_t>(pickPosition_ * s.periodSamples));
    // Pass-2 taps reach back burst+combDelay-1 samples; shrink the comb
    // delay for periods near capacity so reads stay in range.
    if (burst + combDelay > Capacity - 1) combDelay = Capacity - 1 - burst;
    float lpEx = 0.0f;
    const float gain = amp * excMakeup_;
    for (size_t i = 0; i < burst; ++i) {
      lpEx += excCoeff_ * (s.rng.nextBipolar() * gain - lpEx);
      s.delay.push(lpEx);
    }
    for (size_t i = 0; i < burst; ++i) {
      const float raw = s.delay.read(burst - 1);
      const float tap = s.delay.read(i >= combDelay ? burst + combDelay - 1
                                                    : combDelay - 1);
      s.delay.push(raw - 0.9f * tap);
    }
    s.quiet = 0;
    s.active = true;
  }

  float sampleRate_ = kDefaultSampleRate;
  float t60_ = 2.0f;
  float brightness_ = 0.7f;
  float pickPosition_ = 0.25f;
  float hardness_ = 0.8f;
  float detuneCents_ = 6.0f;
  float lpCoeff_ = 0.5f;
  float excCoeff_ = 0.8f;
  float excMakeup_ = 1.0f;
  float apCoeff_ = 0.0f;
  std::array<StringState, kNumStrings> strings_{};
};

// Extended Karplus-Strong voice: fractional-period tuning, blendable loop
// brightness, pick-position second tap, selectable excitation, a detuned
// sympathetic string, and a two-resonator body model. All memory is fixed
// (template capacity); all setters are control-rate and RT-safe.
template <size_t Capacity>
class ExtendedKarplusStrongVoice {
 public:
  // Burst shape used by pluck().
  enum class Excitation : std::uint8_t {
    Noise,          // classic white-noise burst (tone ignored)
    FilteredNoise,  // noise burst through a one-pole lowpass (tone = cutoff)
    Pulse,          // square burst; tone sets length from 1..period/2 samples
    Impulse         // single-sample click at both tap injection points (tone ignored)
  };

  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    updateBrightnessCoeff();
    updateBodyCoeffs();
    reset();
  }

  void reset() {
    delay_.reset();
    symDelay_.reset();
    periodSamples_ = static_cast<float>(Capacity / 2);
    symPeriodSamples_ = periodSamples_;
    last_ = 0.0f;
    symLast_ = 0.0f;
    lpState_ = 0.0f;
    symLpState_ = 0.0f;
    exciteLpState_ = 0.0f;
    body1_.reset();
    body2_.reset();
    active_ = false;
  }

  // Feedback gain; clamped like the classic voice.
  void setDecay(float decay) { decay_ = clamp(decay, 0.0f, 0.9999f); }

  // Loop damping blend: 0 = classic two-point average, 1 = nearly unfiltered
  // feedback (bright, long ring). Values between blend the two filters.
  void setBrightness(float brightness01) {
    brightness_ = clamp01(brightness01);
    updateBrightnessCoeff();
  }

  // Second output tap position as a fraction of the period, clamped to
  // [0.02, 0.5]; output is a 50/50 mix of the main and pick taps (comb timbre).
  void setPickPosition(float pos01) { pickPos_ = clamp(pos01, 0.02f, 0.5f); }

  void setExcitation(Excitation excitation) { excitation_ = excitation; }

  // Tone meaning depends on the excitation: lowpass cutoff for FilteredNoise,
  // burst length for Pulse; ignored for Noise and Impulse.
  void setExcitationTone(float tone01) { excitationTone_ = clamp01(tone01); }

  // Sympathetic string detune mapping: 0..1 -> 0..50 cents sharp. The second
  // string always runs above the main string; 0.5 is a mild 25-cent shimmer.
  void setSympatheticDetune(float detune01) {
    symDetuneCents_ = 50.0f * clamp01(detune01);
    updateSymPeriod();
  }

  void setSympatheticLevel(float level) { symLevel_ = clamp01(level); }

  // Body resonance: two parallel RBJ bandpass biquads (Q ~ 8) at bodyFreq and
  // bodyFreq * 2.756 (a non-harmonic wooden-ish partial). Coefficients are
  // recomputed here and in prepare(), never per sample.
  void setBodyFrequency(float hz) {
    bodyFreqHz_ = clamp(hz, 50.0f, 500.0f);
    updateBodyCoeffs();
  }

  void setBodyAmount(float amount01) { bodyAmount_ = clamp01(amount01); }

  void pluck(float frequencyHz, float amplitude = 1.0f) {
    const float freq = std::max(1.0f, frequencyHz);
    // Fractional period gives accurate tuning; clamped to the fixed delay.
    // Note: reading the loop with linear interpolation adds a small
    // frequency-dependent phase shift, so damping varies slightly with the
    // fractional part; we accept this rather than compensating the filter.
    periodSamples_ = clamp(sampleRate_ / freq, 2.0f, static_cast<float>(Capacity - 1));
    const float amp = std::max(0.0f, amplitude);
    updateSymPeriod();

    delay_.reset();
    symDelay_.reset();
    last_ = 0.0f;
    symLast_ = 0.0f;
    lpState_ = 0.0f;
    symLpState_ = 0.0f;
    exciteLpState_ = 0.0f;

    // Pulse burst length from tone: 1..period/2 samples of a square wave.
    const size_t pulseLen = static_cast<size_t>(
        1.0f + excitationTone_ * (periodSamples_ * 0.5f - 1.0f));
    // FilteredNoise one-pole: tone 0 -> ~200 Hz, tone 1 -> ~16 kHz (exp map).
    const float exciteCutoff = 200.0f * std::pow(80.0f, excitationTone_);
    const float exciteB =
        1.0f - std::exp(-kTwoPi * clampCutoff(exciteCutoff, sampleRate_) / sampleRate_);

    const size_t fillCount = static_cast<size_t>(periodSamples_);
    const size_t pickIndex =
        static_cast<size_t>(pickPos_ * periodSamples_);
    for (size_t i = 0; i < fillCount; ++i) {
      float sample = 0.0f;
      switch (excitation_) {
        case Excitation::Noise:
          sample = rng_.nextBipolar() * amp;
          break;
        case Excitation::FilteredNoise: {
          const float noise = rng_.nextBipolar() * amp;
          exciteLpState_ += exciteB * (noise - exciteLpState_);
          sample = exciteLpState_;
          break;
        }
        case Excitation::Pulse:
          sample = i < pulseLen ? amp : 0.0f;
          break;
        case Excitation::Impulse:
          // Click at the main and pick injection points only.
          sample = (i == 0 || i == pickIndex) ? amp : 0.0f;
          break;
      }
      delay_.push(sample);
      // Sympathetic string hears the same burst, scaled down.
      symDelay_.push(sample * 0.5f);
    }
    active_ = true;
  }

  float process() {
    if (!active_) {
      return 0.0f;
    }

    // Main string: fractional-period read, pick-position second tap.
    const float tapDelay = periodSamples_ - 1.0f;
    const float current = delay_.readLinear(tapDelay);
    const float pick = delay_.readLinear(tapDelay * pickPos_);
    const float stringOut = 0.5f * (current + pick);

    // Loop filter: classic two-point average blended with a one-pole lowpass
    // whose cutoff opens as brightness -> 1 (nearly no damping at b = 1).
    const float rawAvg = 0.5f * (current + last_);
    lpState_ = zapDenormal(lpState_ + lpCoeff_ * (current - lpState_));
    const float next = decay_ * lerp(rawAvg, lpState_, brightness_);
    delay_.push(next);
    last_ = current;

    // Sympathetic string mirrors the same loop with its own period.
    const float symTapDelay = symPeriodSamples_ - 1.0f;
    const float symCurrent = symDelay_.readLinear(symTapDelay);
    const float symRawAvg = 0.5f * (symCurrent + symLast_);
    symLpState_ = zapDenormal(symLpState_ + lpCoeff_ * (symCurrent - symLpState_));
    const float symNext = decay_ * lerp(symRawAvg, symLpState_, brightness_);
    symDelay_.push(symNext);
    symLast_ = symCurrent;

    // Silence detection watches both strings.
    if (std::fabs(next) < 1.0e-5f && std::fabs(current) < 1.0e-5f &&
        std::fabs(symNext) < 1.0e-5f && std::fabs(symCurrent) < 1.0e-5f) {
      active_ = false;
    }

    float out = stringOut + symLevel_ * symCurrent;

    // Body resonance is driven by the combined string output.
    if (bodyAmount_ > 0.0f) {
      const float body = body1_.process(out) + body2_.process(out);
      out += bodyAmount_ * body;
    }
    return out;
  }

  [[nodiscard]] bool isActive() const { return active_; }

 private:
  // Small RBJ bandpass biquad (constant 0 dB peak gain), Q fixed at ~8;
  // coefficients are set up at prepare/pluck/setter time, never per sample.
  struct BodyBiquad {
    void setFrequency(float hz, float sampleRate) {
      constexpr float kQ = 8.0f;
      const float w0 = kTwoPi * clampCutoff(hz, sampleRate) / sampleRate;
      const float alpha = std::sin(w0) / (2.0f * kQ);
      const float a0 = 1.0f + alpha;
      b0_ = alpha / a0;
      b1_ = 0.0f;
      b2_ = -alpha / a0;
      a1_ = (-2.0f * std::cos(w0)) / a0;
      a2_ = (1.0f - alpha) / a0;
    }

    void reset() { z1_ = 0.0f; z2_ = 0.0f; }

    float process(float input) {
      // Transposed direct form II, same convention as BiquadLowpass.
      const float out = (b0_ * input) + z1_;
      z1_ = (b1_ * input) - (a1_ * out) + z2_;
      z2_ = (b2_ * input) - (a2_ * out);
      return zapDenormal(out);
    }

    float b0_ = 0.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
  };

  // Brightness maps the loop one-pole exponentially: b=0 -> ~800 Hz, b=1 ->
  // just under Nyquist (near-transparent, longest ring).
  void updateBrightnessCoeff() {
    const float cutoff = 800.0f * std::pow(0.48f * sampleRate_ / 800.0f, brightness_);
    lpCoeff_ = 1.0f - std::exp(-kTwoPi * clampCutoff(cutoff, sampleRate_) / sampleRate_);
  }

  // Sympathetic period follows the main period, shortened by the cents detune.
  void updateSymPeriod() {
    const float ratio = std::pow(2.0f, symDetuneCents_ / 1200.0f);
    symPeriodSamples_ = clamp(periodSamples_ / ratio, 2.0f, static_cast<float>(Capacity - 1));
  }

  void updateBodyCoeffs() {
    body1_.setFrequency(bodyFreqHz_, sampleRate_);
    // Non-harmonic partial keeps the body from ringing exactly with the string.
    body2_.setFrequency(bodyFreqHz_ * 2.756f, sampleRate_);
  }

  static constexpr float kTwoPi = 6.28318530717958647692f;

  float sampleRate_ = kDefaultSampleRate;
  float decay_ = 0.996f;
  float brightness_ = 0.0f;
  float pickPos_ = 0.25f;
  float excitationTone_ = 0.5f;
  float symDetuneCents_ = 25.0f;
  float symLevel_ = 0.0f;
  float bodyFreqHz_ = 120.0f;
  float bodyAmount_ = 0.0f;
  Excitation excitation_ = Excitation::Noise;

  float periodSamples_ = Capacity / 2;
  float symPeriodSamples_ = Capacity / 2;
  float last_ = 0.0f;
  float symLast_ = 0.0f;
  float lpState_ = 0.0f;
  float symLpState_ = 0.0f;
  float lpCoeff_ = 0.1f;
  float exciteLpState_ = 0.0f;
  bool active_ = false;

  DelayLine<Capacity> delay_;
  DelayLine<Capacity> symDelay_;
  BodyBiquad body1_;
  BodyBiquad body2_;
  XorShift32 rng_{0xB7E15163u};
};

}  // namespace rpdsp
