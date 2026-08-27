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

 public:
  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
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
    // The recipe requires wow + 3 < delay < Capacity - 2. Keep that safety
    // contract in the named wrapper without changing delay_tape() itself.
    const float maxDelay = static_cast<float>(Capacity - 3);
    const float wow = clamp(wowSamples_, 0.0f, maxDelay - 4.0f);
    const float delay = clamp(delaySamples_, wow + 4.0f, maxDelay);
    return delay_tape(input, buffer_.data(), static_cast<int>(Capacity),
                      delay, wow, feedback_, state_.data());
  }

 private:
  float sampleRate_ = kDefaultSampleRate;
  float delaySamples_ = 1.0f;
  float wowSamples_ = 0.0f;
  float feedback_ = 0.0f;
  std::array<float, Capacity> buffer_{};
  std::array<float, 4> state_{};
};

}  // namespace rpdsp
