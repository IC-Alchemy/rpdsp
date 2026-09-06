#pragma once

// Optional named-module wrapper for adsr_analog(). The recipe remains the
// low-level API; this class turns its caller-owned state and gate into a
// conventional named envelope interface. Setters/prepare() run at control
// rate; process() only advances the allocation-free recipe. Finite controls
// are expected. Gate-off releases on the next sample, even during attack.

#include "DSPFunctions.h"
#include "algorithm.h"

#include <array>
#include <cmath>

namespace rpdsp {

class AnalogAdsr {
 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    // Preserve time constants configured before prepare() or a rate change.
    // Explicit coefficient settings remain per-sample values.
    if (attackSeconds_ >= 0.0f) attack_ = secondsToCoefficient(attackSeconds_);
    if (decaySeconds_ >= 0.0f) decay_ = secondsToCoefficient(decaySeconds_);
    if (releaseSeconds_ >= 0.0f) release_ = secondsToCoefficient(releaseSeconds_);
    reset();
  }

  // Clear playback state while retaining all parameter settings.
  void reset() {
    state_.fill(0.0f);
    gate_ = false;
  }

  // Per-sample rates: 0 holds the stage, 1 completes its update immediately.
  // Calling a coefficient setter supersedes that stage's seconds setting.
  void setAttackCoefficient(float coefficient) {
    attack_ = clamp01(coefficient);
    attackSeconds_ = -1.0f;
  }
  void setDecayCoefficient(float coefficient) {
    decay_ = clamp01(coefficient);
    decaySeconds_ = -1.0f;
  }
  void setSustain(float level) { sustain_ = clamp01(level); }
  void setReleaseCoefficient(float coefficient) {
    release_ = clamp01(coefficient);
    releaseSeconds_ = -1.0f;
  }

  // Seconds are one-pole time constants, not exact stage durations. From
  // zero, attack reaches 1 after about 1.466 * tau (it aims at 1.3). Decay
  // and release remove about 63.2% of the remaining error per tau.
  // Non-positive times complete the stage update on the next sample.
  void setAttackSeconds(float seconds) {
    attackSeconds_ = fmaxf(seconds, 0.0f);
    attack_ = secondsToCoefficient(attackSeconds_);
  }
  void setDecaySeconds(float seconds) {
    decaySeconds_ = fmaxf(seconds, 0.0f);
    decay_ = secondsToCoefficient(decaySeconds_);
  }
  void setReleaseSeconds(float seconds) {
    releaseSeconds_ = fmaxf(seconds, 0.0f);
    release_ = secondsToCoefficient(releaseSeconds_);
  }

  void set(float attackSeconds, float decaySeconds, float sustain,
           float releaseSeconds) {
    setAttackSeconds(attackSeconds);
    setDecaySeconds(decaySeconds);
    setSustain(sustain);
    setReleaseSeconds(releaseSeconds);
  }

  // Event-style retrigger: start attack from the current level, including
  // adjacent notes with no processed gate-low sample. No level discontinuity
  // is introduced by trigger() itself; the next process() advances attack.
  void trigger() {
    gate_ = true;
    state_[2] = 1.0f;
  }
  void noteOn() { trigger(); }
  void noteOff() { gate_ = false; }

  float process() {
    return adsr_analog(gate_ ? 1.0f : 0.0f, attack_, decay_, sustain_,
                       release_, state_.data());
  }

  float process(bool gate) {
    // Level-style gate input: a held high gate does not repeatedly retrigger.
    gate_ = gate;
    return process();
  }

 private:
  float secondsToCoefficient(float seconds) const {
    // A non-positive time means complete the one-pole step in this sample.
    if (seconds <= 0.0f) {
      return 1.0f;
    }
    // expm1 avoids cancellation to zero for long time constants.
    return -std::expm1(-1.0f / (seconds * sampleRate_));
  }

  float sampleRate_ = kDefaultSampleRate;
  float attack_ = 0.01f;
  float decay_ = 0.001f;
  float sustain_ = 0.7f;
  float release_ = 0.001f;
  // A negative sentinel selects coefficient mode for each stage. The
  // defaults retain the recipe wrapper's original per-sample rates.
  float attackSeconds_ = -1.0f;
  float decaySeconds_ = -1.0f;
  float releaseSeconds_ = -1.0f;
  bool gate_ = false;
  // Recipe layout: level, previous gate, attack flag.
  std::array<float, 3> state_{};
};

}  // namespace rpdsp
