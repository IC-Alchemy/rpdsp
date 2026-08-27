#pragma once

// Optional named-module wrapper for adsr_analog(). The recipe remains the
// low-level API; this class turns its caller-owned state and gate into a
// conventional named envelope interface.

#include "DSPFunctions.h"
#include "algorithm.h"

#include <array>
#include <cmath>

namespace rpdsp {

class AnalogAdsr {
 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    reset();
  }

  void reset() {
    state_.fill(0.0f);
    gate_ = false;
  }

  void setAttackCoefficient(float coefficient) { attack_ = clamp01(coefficient); }
  void setDecayCoefficient(float coefficient) { decay_ = clamp01(coefficient); }
  void setSustain(float level) { sustain_ = clamp01(level); }
  void setReleaseCoefficient(float coefficient) { release_ = clamp01(coefficient); }

  void setAttackSeconds(float seconds) { setAttackCoefficient(secondsToCoefficient(seconds)); }
  void setDecaySeconds(float seconds) { setDecayCoefficient(secondsToCoefficient(seconds)); }
  void setReleaseSeconds(float seconds) { setReleaseCoefficient(secondsToCoefficient(seconds)); }

  void set(float attackSeconds, float decaySeconds, float sustain,
           float releaseSeconds) {
    setAttackSeconds(attackSeconds);
    setDecaySeconds(decaySeconds);
    setSustain(sustain);
    setReleaseSeconds(releaseSeconds);
  }

  void noteOn() { gate_ = true; }
  void noteOff() { gate_ = false; }

  float process() {
    return adsr_analog(gate_ ? 1.0f : 0.0f, attack_, decay_, sustain_,
                       release_, state_.data());
  }

  float process(bool gate) {
    gate_ = gate;
    return process();
  }

 private:
  float secondsToCoefficient(float seconds) const {
    // A non-positive time means complete the one-pole step in this sample.
    if (seconds <= 0.0f) {
      return 1.0f;
    }
    return 1.0f - std::exp(-1.0f / (seconds * sampleRate_));
  }

  float sampleRate_ = kDefaultSampleRate;
  float attack_ = 0.01f;
  float decay_ = 0.001f;
  float sustain_ = 0.7f;
  float release_ = 0.001f;
  bool gate_ = false;
  std::array<float, 3> state_{};
};

}  // namespace rpdsp
