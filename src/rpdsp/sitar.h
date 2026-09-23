#pragma once

#include "algorithm.h"
#include "delay_line.h"
#include "realtime.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace rpdsp {

// ---------------------------------------------------------------------------
// SitarStringVoice — plucked-string course with a jawari bridge, a taraf
// sympathetic resonator bank, and a two-resonator body.
//
// String core: the PluckedStringVoice architecture (fractional-delay
// Karplus-Strong course, T60 decay, pick position/hardness, dispersion,
// detuned pair) with three sitar-specific additions:
//
//   * Jawari: a memoryless, one-sided soft compression INSIDE the string
//     feedback loop. Loop readouts above the contact threshold (the string
//     buzzing against the curved bridge) are squashed by over/(1 + k*over),
//     which strictly removes energy — the loop can never be driven unstable.
//     Because the contact only engages while the string amplitude exceeds the
//     threshold, the buzz is strongest right after the pluck and cleans up
//     progressively as the string decays: an evolving spectrum, not static
//     distortion. A tiny one-pole DC blocker on the output removes the offset
//     the one-sided contact generates. Per-sample cost is one compare, plus
//     one divide only on contacting samples (rational knee — no tanhf/powf/
//     expf; same per-sample division budget as padeTanh).
//
//   * Taraf: five high-Q resonant modes (root, fifth, octave, octave+fifth,
//     two octaves above the played note) driven continuously by the string
//     output. They are never plucked; they ring with whatever the string
//     feeds them and keep sounding after the string itself dies. Pole radii
//     encode tarafDecaySeconds exactly, so "slow decay" holds at every pitch;
//     coefficients are rebuilt only when the pitch or decay time changes.
//     Note: modes tuned between string harmonics (the fifth at 1.5*f0) are
//     excited by the jawari buzz and the pluck transient, not by the harmonic
//     series itself — exactly like real sympathetic strings, which need
//     near-unison drive.
//
//   * Meend: slideTo() retunes an already-ringing string by slewing the
//     fractional loop delay (and the loop gain with it) over
//     setSlideTimeSeconds() — no delay reset, no retriggered excitation, no
//     interruption of the sympathetic bank.
//
// Body: the ExtendedKarplusStrongVoice pattern — two parallel RBJ bandpass
// biquads (Q ~ 8) at bodyFreq and bodyFreq*2.756, mixed at a modest amount so
// the string feels attached to an acoustic object.
//
// Realtime contract: fixed storage, no allocation; expf/powf/sinf/cosf only
// in control-rate setters, prepare(), pluck() and slideTo(). process() is two
// interpolated delay reads, contact compression, two one-pole updates, four
// allpass updates, two pushes, five taraf modes and two body biquads.
//
// Capacity: delay-line length in samples per string. Periods are clamped to
// [4, Capacity-2], so the lowest playable pitch is sampleRate/(Capacity-2)
// (about 24 Hz at 48 kHz with the firmware's 2048).
// ---------------------------------------------------------------------------
template <size_t Capacity>
class SitarStringVoice {
 public:
  static constexpr int kNumStrings = 2;
  static constexpr int kTarafModes = 5;

  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    resonatorQuietHold_ =
        std::max<size_t>(1, static_cast<size_t>(0.25f * sampleRate_));
    setBrightness(brightness_);
    setPickHardness(hardness_);
    setStiffness(stiffness_);
    setJawari(jawari_);
    setJawariThreshold(jawariThreshold_);
    setSlideTimeSeconds(slideSeconds_);
    updateBodyCoeffs();
    updateTarafCoeffs();
    reset();
  }

  void reset() {
    // Same seed discipline as PluckedStringVoice: distinct per-string seeds
    // for independent excitation noise; 0xA511E9B3/0x7F4A7C15 cancel in a
    // course sum, so do not "fix" the seed values.
    strings_[0].rng = XorShift32{0xA511E9B3u};
    strings_[1].rng = XorShift32{0x9E3779B9u};
    for (auto& s : strings_) {
      s.delay.reset();
      s.lp = 0.0f;
      s.apX1 = s.apY1 = s.apX2 = s.apY2 = 0.0f;
      s.active = false;
      s.quiet = 0;
      s.periodSamples = static_cast<float>(Capacity) * 0.5f;
      s.targetPeriod = s.periodSamples;
      s.loopDelay = s.periodSamples - 1.0f;
      s.loopGain = 0.995f;
      s.targetGain = s.loopGain;
    }
    dcX1_ = 0.0f;
    dcY1_ = 0.0f;
    body1_.reset();
    body2_.reset();
    for (auto& mode : taraf_) mode.reset();
    anyStringActive_ = false;
    sliding_ = false;
    // Counters start at the hold limit: nothing is ringing yet, so the voice
    // is inactive until the first pluck().
    tarafQuiet_ = resonatorQuietHold_;
    bodyQuiet_ = resonatorQuietHold_;
  }

  // --- Control-rate setters (call from updateControls(), not per sample) ---

  // Time for the string tail to fall 60 dB, independent of pitch. 0.05..10 s.
  void setDecayTimeSeconds(float t60) {
    t60_ = clamp(t60, 0.05f, 10.0f);
    for (auto& s : strings_) updateLoopGains(s);
  }

  // Loop damping: 0 = dark (300 Hz), 1 = very bright (12 kHz). Exponential map.
  void setBrightness(float brightness01) {
    brightness_ = clamp01(brightness01);
    const float cutoffHz = 300.0f * powf(40.0f, brightness_);
    lpCoeff_ = 1.0f - expf(-kTwoPi * cutoffHz / sampleRate_);
    updateStructuralDelay();
    for (auto& s : strings_) {
      updateLoopDelay(s);
      updateLoopGains(s);  // damping loss at the fundamental is compensated
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
    excCoeff_ = 1.0f - expf(-kTwoPi * cutoffHz / sampleRate_);
    // Makeup so a soft (heavily lowpassed) burst keeps comparable loudness.
    const float rms = sqrtf(excCoeff_ / (2.0f - excCoeff_));
    excMakeup_ = clamp(1.0f / (rms + 1.0e-6f) * 0.35f, 0.7f, 3.0f);
  }

  // 0 = perfectly harmonic, 1 = strongly dispersive. Sitar stays low.
  void setStiffness(float stiffness01) {
    stiffness_ = clamp01(stiffness01);
    apCoeff_ = -0.62f * stiffness_;
    updateStructuralDelay();
    for (auto& s : strings_) updateLoopDelay(s);
  }

  // Spread between the two strings of the course in cents (0..30); a sitar
  // course runs nearly in unison.
  void setDetuneCents(float cents) { detuneCents_ = clamp(cents, 0.0f, 30.0f); }

  // JAWARI: bridge contact amount, 0 = clean string, 1 = aggressive buzzing.
  void setJawari(float jawari01) {
    jawari_ = clamp01(jawari01);
    updateJawariCoeffs();
  }

  // Contact threshold (amplitude above which the string touches the bridge):
  // 0..1 maps to a threshold of 0.02..0.6. Lower = buzzes sooner and longer.
  void setJawariThreshold(float threshold01) {
    jawariThreshold_ = clamp01(threshold01);
    contactThreshold_ = 0.02f + 0.58f * jawariThreshold_;
  }

  // TARAF: sympathetic bank output level, 0 = no sympathetic contribution.
  void setTarafAmount(float amount01) { tarafAmount_ = clamp01(amount01); }

  // Sympathetic ring time after the drive stops. 0.05..12 s.
  void setTarafDecaySeconds(float t60) {
    tarafT60_ = clamp(t60, 0.05f, 12.0f);
    updateTarafCoeffs();
  }

  // Body resonance mix; 0 = disembodied string.
  void setBodyAmount(float amount01) { bodyAmount_ = clamp01(amount01); }

  // Low body resonance; the second body mode sits a non-harmonic 2.756x above
  // it. 50..500 Hz, coefficients rebuilt here — never per sample.
  void setBodyFrequency(float hz) {
    bodyFreqHz_ = clamp(hz, 50.0f, 500.0f);
    updateBodyCoeffs();
  }

  // Meend slide time: how long slideTo() takes to bend the ringing string.
  // 0.005..2 s; the default 0.12 s suits slow sitar meend gestures.
  void setSlideTimeSeconds(float seconds) {
    slideSeconds_ = clamp(seconds, 0.005f, 2.0f);
    // One-pole time constant = slideSeconds/4, so the bend audibly completes
    // in about the requested time.
    slewRetain_ = onePoleSmooth(250.0f * slideSeconds_, sampleRate_);
  }

  // --- Events ---------------------------------------------------------------

  // Pluck the course at frequencyHz. amplitude drives the excitation level
  // and leaves a subtle trace in jawari intensity (harder plucks buzz harder).
  void pluck(float frequencyHz, float amplitude = 1.0f) {
    const float freq = clamp(frequencyHz, 20.0f, sampleRate_ * 0.25f);
    const float amp = std::max(0.0f, amplitude);
    jawariVelocityScale_ = 0.8f + 0.2f * clamp01(amplitude);
    updateJawariCoeffs();
    const float half = detuneCents_ * 0.5f;
    pluckString(strings_[0], freq * powf(2.0f, -half / 1200.0f), amp);
    pluckString(strings_[1], freq * powf(2.0f, half / 1200.0f), amp);
    sliding_ = false;
    anyStringActive_ = true;
    // Something is about to ring: keep the resonator counters wide open so
    // isActive() cannot flicker off between the pluck and the sound.
    tarafQuiet_ = 0;
    bodyQuiet_ = 0;
    if (freq != tarafBaseFreq_) {
      tarafBaseFreq_ = freq;
      updateTarafCoeffs();
    }
  }

  // Meend: bend the ringing string to frequencyHz over the slide time without
  // resetting the delay or retriggering the excitation. Safe to call while
  // silent; the next pluck() overrides the slide.
  void slideTo(float frequencyHz) {
    const float freq = clamp(frequencyHz, 20.0f, sampleRate_ * 0.25f);
    const float half = detuneCents_ * 0.5f;
    strings_[0].targetPeriod = clamp(
        sampleRate_ / (freq * powf(2.0f, -half / 1200.0f)), 4.0f,
        static_cast<float>(Capacity - 2));
    strings_[1].targetPeriod = clamp(
        sampleRate_ / (freq * powf(2.0f, half / 1200.0f)), 4.0f,
        static_cast<float>(Capacity - 2));
    for (auto& s : strings_) updateLoopGains(s);
    sliding_ = true;
    // The bank follows the target pitch; coefficients are rebuilt here (control
    // rate), never while the delay slews.
    if (freq != tarafBaseFreq_) {
      tarafBaseFreq_ = freq;
      updateTarafCoeffs();
    }
  }

  float process() {
    if (!isActive()) {
      return 0.0f;
    }

    // --- Strings: read -> jawari contact -> loop damping -> delay -----------
    float stringSum = 0.0f;
    const bool contact = jawariK_ > 0.0f;
    const float slewStep = 1.0f - slewRetain_;
    bool stringsSettled = true;
    anyStringActive_ = false;
    for (auto& s : strings_) {
      if (!s.active) continue;
      anyStringActive_ = true;
      if (sliding_) {
        s.periodSamples += slewStep * (s.targetPeriod - s.periodSamples);
        s.loopGain += slewStep * (s.targetGain - s.loopGain);
        if (std::fabs(s.targetPeriod - s.periodSamples) > 0.01f) {
          stringsSettled = false;
        }
      }
      s.loopDelay = clamp(s.periodSamples - structuralDelay_, 2.0f,
                          static_cast<float>(Capacity - 2));
      const float y = s.delay.readLinear(s.loopDelay);
      float contacted = y;
      if (contact) {
        const float over = y - contactThreshold_;
        // One-sided soft compression: only excursions past the threshold are
        // squashed, and strictly downward — energy can only leave the loop
        // through the contact, never enter.
        if (over > 0.0f) {
          contacted = contactThreshold_ + over / (1.0f + jawariK_ * over);
        }
      }
      // Loop damping: adjustable one-pole lowpass (the "string loss").
      s.lp = zapDenormal(s.lp + lpCoeff_ * (contacted - s.lp));
      // Dispersion: two first-order allpasses sharpen the upper partials.
      const float t = apCoeff_ * s.lp + s.apX1 - apCoeff_ * s.apY1;
      s.apX1 = s.lp;
      s.apY1 = t;
      const float u = apCoeff_ * t + s.apX2 - apCoeff_ * s.apY2;
      s.apX2 = t;
      s.apY2 = u;
      s.delay.push(u * s.loopGain);
      stringSum += contacted;
      // Deactivate after a sustained stretch below audibility (also keeps
      // the recirculating values out of denormal territory).
      if (std::fabs(y) < 1.0e-5f) {
        if (++s.quiet > 4096) s.active = false;
      } else {
        s.quiet = 0;
      }
    }
    if (stringsSettled) sliding_ = false;

    // Tiny DC/high-pass correction for the one-sided contact offset.
    dcY1_ = zapDenormal(stringSum - dcX1_ + kDcBlockR * dcY1_);
    dcX1_ = stringSum;
    float out = dcY1_;

    // --- Body: two parallel RBJ bandpasses on the string output ------------
    float bodyMix = 0.0f;
    if (bodyAmount_ > 0.0f) {
      bodyMix = bodyAmount_ * (body1_.process(out) + body2_.process(out));
    }

    // --- Taraf: five continuously-excited sympathetic modes -----------------
    float tarafMix = 0.0f;
    if (tarafAmount_ > 0.0f) {
      float sum = 0.0f;
      for (int i = 0; i < kTarafModes; ++i) {
        sum += kTarafTapers[static_cast<size_t>(i)] *
               taraf_[static_cast<size_t>(i)].process(out);
      }
      tarafMix = tarafAmount_ * sum;
    }

    // Track resonator ringing so isActive() stays true while either bank is
    // audibly alive, even with the strings gone quiet.
    if (std::fabs(tarafMix) < 1.0e-4f) {
      if (tarafQuiet_ < resonatorQuietHold_) ++tarafQuiet_;
    } else {
      tarafQuiet_ = 0;
    }
    if (std::fabs(bodyMix) < 1.0e-4f) {
      if (bodyQuiet_ < resonatorQuietHold_) ++bodyQuiet_;
    } else {
      bodyQuiet_ = 0;
    }

    return out + bodyMix + tarafMix;
  }

  // True while the string, the body, or the sympathetic bank is still
  // audible: silent-voice optimizations must not stop processing during a
  // taraf tail.
  [[nodiscard]] bool isActive() const {
    return anyStringActive_ || tarafQuiet_ < resonatorQuietHold_ ||
           bodyQuiet_ < resonatorQuietHold_;
  }

 private:
  static constexpr float kTwoPi = 6.28318530718f;
  // One-pole DC-blocker coefficient (~4 Hz corner at 48 kHz).
  static constexpr float kDcBlockR = 0.9995f;
  // Fixed sympathetic intervals above the played note (v1; scale-aware tuning
  // is second generation) and their relative output levels.
  static constexpr std::array<float, kTarafModes> kTarafIntervals{
      1.0f, 1.5f, 2.0f, 3.0f, 4.0f};
  static constexpr std::array<float, kTarafModes> kTarafTapers{
      1.0f, 0.8f, 0.8f, 0.6f, 0.5f};

  struct StringState {
    DelayLine<Capacity> delay;
    float periodSamples = 256.0f;  // placeholder until the first pluck
    float targetPeriod = 256.0f;
    float loopDelay = 2.0f;
    float loopGain = 0.995f;
    float targetGain = 0.995f;
    float lp = 0.0f;
    float apX1 = 0.0f, apY1 = 0.0f, apX2 = 0.0f, apY2 = 0.0f;
    int quiet = 0;
    bool active = false;
    XorShift32 rng{0xA511E9B3u};
  };

  // Small RBJ bandpass biquad (constant 0 dB peak gain), same transposed
  // direct form II convention as ExtendedKarplusStrongVoice::BodyBiquad.
  // alpha is derived from an explicit pole radius so one decay time holds at
  // every mode frequency; coefficients are set up at control rate only.
  struct TarafResonator {
    void set(float frequencyHz, float t60, float sampleRate) {
      const float w0 = kTwoPi * clampCutoff(frequencyHz, sampleRate) / sampleRate;
      const float radius = powf(10.0f, -3.0f / (t60 * sampleRate));
      const float alpha = (1.0f - radius * radius) / (1.0f + radius * radius);
      const float a0 = 1.0f + alpha;
      b0_ = alpha / a0;
      b2_ = -b0_;
      a1_ = (-2.0f * cosf(w0)) / a0;
      a2_ = (1.0f - alpha) / a0;
    }

    void reset() { z1_ = 0.0f; z2_ = 0.0f; }

    float process(float input) {
      const float out = (b0_ * input) + z1_;
      z1_ = -(a1_ * out) + z2_;
      z2_ = (b2_ * input) - (a2_ * out);
      return zapDenormal(out);
    }

    float b0_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
  };

  // Body resonator biquad: constant 0 dB peak gain RBJ bandpass, Q fixed ~8;
  // coefficients set in prepare()/setBodyFrequency(), never per sample.
  struct BodyBiquad {
    void setFrequency(float hz, float sampleRate) {
      constexpr float kQ = 8.0f;
      const float w0 = kTwoPi * clampCutoff(hz, sampleRate) / sampleRate;
      const float alpha = sinf(w0) / (2.0f * kQ);
      const float a0 = 1.0f + alpha;
      b0_ = alpha / a0;
      b2_ = -b0_;
      a1_ = (-2.0f * cosf(w0)) / a0;
      a2_ = (1.0f - alpha) / a0;
    }

    void reset() { z1_ = 0.0f; z2_ = 0.0f; }

    float process(float input) {
      const float out = (b0_ * input) + z1_;
      z1_ = -(a1_ * out) + z2_;
      z2_ = (b2_ * input) - (a2_ * out);
      return zapDenormal(out);
    }

    float b0_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
  };

  // Per-period gain for the requested T60 at this period (PluckedStringVoice
  // math, factored so the slide can target a gain before it gets there).
  float loopGainForPeriod(float periodSamples) const {
    // g^(T60*fs/P) = 10^-3  =>  g = 10^(-3*P/(T60*fs))
    const float exponent = -3.0f * periodSamples / (t60_ * sampleRate_);
    float g = powf(10.0f, exponent);
    // The loop damping lowpass also attenuates the fundamental once per
    // period; divide its magnitude there back out so the requested T60
    // survives dark brightness settings. Clamped: a very dark string may
    // decay faster than asked, but can never go unstable.
    const float w0 = kTwoPi / periodSamples;
    const float re = 1.0f - (1.0f - lpCoeff_) * cosf(w0);
    const float im = (1.0f - lpCoeff_) * sinf(w0);
    const float dampingMag = lpCoeff_ / sqrtf(re * re + im * im);
    g /= std::max(dampingMag, 0.05f);
    return clamp(g, 0.0f, 0.99995f);
  }

  void updateLoopGains(StringState& s) const {
    s.loopGain = loopGainForPeriod(s.periodSamples);
    s.targetGain = loopGainForPeriod(s.targetPeriod);
  }

  // Fractional read length = period minus the structural 1-sample delay of
  // the read/push loop and the DC group delay of the loop filters.
  void updateStructuralDelay() {
    const float lpDelay = (1.0f - lpCoeff_) / lpCoeff_;
    const float apDelay =
        2.0f * (1.0f - apCoeff_) / (1.0f + apCoeff_);  // two stages
    structuralDelay_ = 1.0f + lpDelay + apDelay;
  }

  void updateLoopDelay(StringState& s) const {
    s.loopDelay = clamp(s.periodSamples - structuralDelay_, 2.0f,
                        static_cast<float>(Capacity - 2));
  }

  void updateJawariCoeffs() {
    // k sets the compression knee: at jawari = 1 the contact asymptote sits
    // 1/36 above the threshold — a hard buzz; jawari = 0 gives k = 0, an
    // exact bypass (skipped per sample). Velocity leaves a subtle trace.
    jawariK_ = 36.0f * jawari_ * jawariVelocityScale_;
  }

  void updateBodyCoeffs() {
    body1_.setFrequency(bodyFreqHz_, sampleRate_);
    // Non-harmonic partial keeps the body from ringing exactly with the string.
    body2_.setFrequency(bodyFreqHz_ * 2.756f, sampleRate_);
  }

  void updateTarafCoeffs() {
    for (size_t i = 0; i < static_cast<size_t>(kTarafModes); ++i) {
      taraf_[i].set(tarafBaseFreq_ * kTarafIntervals[i], tarafT60_, sampleRate_);
    }
  }

  // Same excitation as PluckedStringVoice::pluckString: one period of noise,
  // lowpassed by pick hardness, then a CIRCULAR pick-position comb applied
  // with two passes through the delay line itself. No scratch buffer.
  void pluckString(StringState& s, float freq, float amp) {
    s.periodSamples =
        clamp(sampleRate_ / freq, 4.0f, static_cast<float>(Capacity - 2));
    s.targetPeriod = s.periodSamples;
    updateLoopGains(s);
    updateLoopDelay(s);
    s.delay.reset();
    s.lp = 0.0f;
    s.apX1 = s.apY1 = s.apX2 = s.apY2 = 0.0f;

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

  // String (sitar defaults: hard bright excitation, bridge-side pick, long
  // decay, near-unison course, low dispersion, very bright loop damping).
  float t60_ = 5.0f;
  float brightness_ = 0.92f;
  float pickPosition_ = 0.12f;
  float hardness_ = 0.9f;
  float stiffness_ = 0.15f;
  float detuneCents_ = 3.0f;
  float lpCoeff_ = 0.5f;
  float excCoeff_ = 0.8f;
  float excMakeup_ = 1.0f;
  float apCoeff_ = -0.62f * 0.15f;
  float structuralDelay_ = 3.0f;

  // Jawari bridge contact.
  float jawari_ = 0.45f;
  float jawariThreshold_ = 0.3f;
  float jawariK_ = 16.2f;
  float jawariVelocityScale_ = 1.0f;
  float contactThreshold_ = 0.194f;

  // Taraf sympathetic bank.
  float tarafAmount_ = 0.35f;
  float tarafT60_ = 4.0f;
  float tarafBaseFreq_ = 220.0f;

  // Body.
  float bodyAmount_ = 0.25f;
  float bodyFreqHz_ = 130.0f;

  // Meend slide.
  float slideSeconds_ = 0.12f;
  float slewRetain_ = 0.999f;

  // Output DC correction.
  float dcX1_ = 0.0f;
  float dcY1_ = 0.0f;

  size_t resonatorQuietHold_ = 12000;
  size_t tarafQuiet_ = 12000;
  size_t bodyQuiet_ = 12000;
  bool anyStringActive_ = false;
  bool sliding_ = false;

  std::array<StringState, kNumStrings> strings_{};
  std::array<TarafResonator, static_cast<size_t>(kTarafModes)> taraf_{};
  BodyBiquad body1_;
  BodyBiquad body2_;
};

}  // namespace rpdsp
