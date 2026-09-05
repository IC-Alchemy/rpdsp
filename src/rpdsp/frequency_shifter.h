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
    coefficients_ = make_fx_freqshift_coefficients(shiftHz_ / sampleRate_);
    reset();
  }

  void reset() { state_.fill(0.0f); }

  // Control-rate setter; computes the carrier rotation once. Changes preserve
  // carrier phase. The applied shift is limited to +/- sampleRate/2.
  // Legacy convention: positive shifts down, negative shifts up.
  void setShiftHz(float shiftHz) {
    shiftHz_ = shiftHz;
    coefficients_ = make_fx_freqshift_coefficients(shiftHz_ / sampleRate_);
  }

  float process(float input) {
    return fx_freqshift(input, coefficients_, state_.data());
  }

 private:
  float sampleRate_ = kDefaultSampleRate;
  float shiftHz_ = 0.0f;
  FrequencyShiftCoefficients coefficients_{};
  std::array<float, 35> state_{};
};

}  // namespace rpdsp
