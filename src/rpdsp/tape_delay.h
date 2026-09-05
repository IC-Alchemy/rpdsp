#pragma once

// Optional named-module wrapper for delay_tape(). The recipe remains the
// low-level API; this class only owns its fixed storage and parameters.

#include "DSPFunctions.h"
#include "algorithm.h"

#include <array>
#include <cstddef>

namespace rpdsp {

template <size_t Capacity>
class TapeDelay {
  static_assert(Capacity > 6, "TapeDelay capacity must leave room for its modulated read head.");
  static_assert(Capacity <= 16777216, "TapeDelay float indices must represent every buffer slot.");

 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    coefficients_ = make_delay_tape_coefficients(sampleRate_);
    reset();
  }

  void reset() {
    buffer_.fill(0.0f);
    state_.fill(0.0f);
  }

  void setDelaySamples(float samples) { delaySamples_ = samples; }
  void setDelayMilliseconds(float milliseconds) {
    setDelaySamples(milliseconds * 0.001f * sampleRate_);
  }
  void setWowSamples(float samples) { wowSamples_ = samples; }
  void setFeedback(float feedback) { feedback_ = feedback; }

  float process(float input) {
    // The recipe clamps both read-head excursions, including when the user
    // changes delay/wow between samples. Timing coefficients are cached.
    return delay_tape(input, buffer_.data(), static_cast<int>(Capacity),
                      delaySamples_, wowSamples_, feedback_, coefficients_, state_.data());
  }

 private:
  float sampleRate_ = kDefaultSampleRate;
  float delaySamples_ = 1.0f;
  float wowSamples_ = 0.0f;
  float feedback_ = 0.0f;
  TapeDelayCoefficients coefficients_{};
  std::array<float, Capacity> buffer_{};
  std::array<float, 4> state_{};
};

}  // namespace rpdsp
