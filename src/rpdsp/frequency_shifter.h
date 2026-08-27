#pragma once

// Optional named-module wrapper for fx_freqshift(). The recipe remains the
// low-level API; this class supplies the sample-rate conversion and state.

#include "DSPFunctions.h"
#include "algorithm.h"

#include <array>

namespace rpdsp {

class FrequencyShifter {
 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    reset();
  }

  void reset() { state_.fill(0.0f); }

  // Positive and negative values select opposite single-sidebands.
  void setShiftHz(float shiftHz) { shiftHz_ = shiftHz; }

  float process(float input) {
    return fx_freqshift(input, shiftHz_ / sampleRate_, state_.data());
  }

 private:
  float sampleRate_ = kDefaultSampleRate;
  float shiftHz_ = 0.0f;
  std::array<float, 35> state_{};
};

}  // namespace rpdsp
