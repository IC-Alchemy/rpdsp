#pragma once

#include <array>
#include <cstddef>

namespace rpdsp {

// Fixed-size, wrap-indexed automation track for a single parameter across up
// to MaxSteps sequencer steps. Mirrors GatePattern's step/length model (see
// gate_pattern.h) but stores a value of type T per step instead of a boolean
// gate, so it works for float/int/bool parameter automation alike.
//
// Ported from Pico2Seq's ParameterTrack<MAX_SIZE> (SequencerDefs.h), which
// was float-only and unnamespaced; behavior is preserved, just generalized
// over T and moved to size_t indexing to match the rest of rpdsp.
template <typename T, size_t MaxSteps>
class ParameterTrack {
 public:
  // Fills every step with defaultValue and sets the active length.
  void init(T defaultValue, size_t stepCount = MaxSteps) {
    defaultValue_ = defaultValue;
    stepCount_ = clampStepCount(stepCount);
    values_.fill(defaultValue);
  }

  // Returns the value for any step index, wrapping into the active length.
  [[nodiscard]] T getValue(size_t stepIndex) const {
    if (stepCount_ == 0)
      return defaultValue_;
    return values_[stepIndex % stepCount_];
  }

  // Sets the value for a step index, wrapping into the active length.
  void setValue(size_t stepIndex, T newValue) {
    if (stepCount_ == 0)
      return;
    values_[stepIndex % stepCount_] = newValue;
  }

  // Changes the active length. Growing fills every step from the *previous*
  // stepCount_ forward with defaultValue_, even if those slots held earlier
  // data from before a prior shrink -- shrink-then-grow does not restore old
  // history beyond the currently active length. This mirrors Pico2Seq's
  // original ParameterTrack::resize exactly (it only fills when newStepCount
  // > currentStepCount, using currentStepCount at call time as the start).
  void resize(size_t newStepCount) {
    const size_t clamped = clampStepCount(newStepCount);
    if (clamped > stepCount_) {
      for (size_t i = stepCount_; i < clamped; ++i)
        values_[i] = defaultValue_;
    }
    stepCount_ = clamped;
  }

  [[nodiscard]] size_t stepCount() const { return stepCount_; }
  [[nodiscard]] T defaultValue() const { return defaultValue_; }
  static constexpr size_t maxSteps() { return MaxSteps; }

  // Direct, non-wrapping access for callers that already know the index is
  // in range (e.g. UI grids drawing every step regardless of active length).
  [[nodiscard]] T rawValue(size_t index) const {
    return index < MaxSteps ? values_[index] : defaultValue_;
  }

 private:
  static size_t clampStepCount(size_t stepCount) {
    if (stepCount < 1)
      return 1;
    return stepCount > MaxSteps ? MaxSteps : stepCount;
  }

  std::array<T, MaxSteps> values_{};
  size_t stepCount_ = MaxSteps;
  T defaultValue_{};
};

}  // namespace rpdsp
