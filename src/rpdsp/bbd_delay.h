#pragma once

// Optional named-module wrapper for delay_bbd(). The recipe remains the
// low-level API; this class only owns its fixed storage and parameters.

#include "DSPFunctions.h"
#include "algorithm.h"

#include <array>
#include <cstddef>

namespace rpdsp {

template <size_t Capacity>
class BbdDelay {
  static_assert(Capacity > 1, "BbdDelay capacity must be greater than one sample.");

 public:
  void prepare(float sampleRate) {
    (void)safeSampleRate(sampleRate);  // Kept for the standard module lifecycle.
    reset();
  }

  void reset() {
    buffer_.fill(0.0f);
    state_.fill(0.0f);
  }

  // Clock is the virtual BBD clock: 0.05..1.0, not Hz. Lower values lengthen
  // and darken the delay while sweeping the repeat pitch.
  void setClock(float clock) { clock_ = clamp(clock, 0.05f, 1.0f); }
  void setFeedback(float feedback) { feedback_ = feedback; }

  float process(float input) {
    return delay_bbd(input, buffer_.data(), static_cast<int>(Capacity),
                     clock_, feedback_, state_.data());
  }

 private:
  float clock_ = 1.0f;
  float feedback_ = 0.0f;
  std::array<float, Capacity> buffer_{};
  std::array<float, 3> state_{};
};

}  // namespace rpdsp
