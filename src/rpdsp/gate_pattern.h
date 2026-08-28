// gate_pattern.h — step-mask storage for trigger/gate patterns
// ---------------------------------------------------------------------------
// STORAGE ONLY. This class holds a mask; it does not own a playhead, because
// uClock does (see the root AGENTS.md and rpdsp/transport.h). The caller
// derives the step from the tick and indexes in:
//
//   const auto t = rpdsp::transport::read();
//   const std::uint32_t perStep = rpdsp::transport::ticksPerDivision(
//       rpdsp::transport::Division::Sixteenth, rpdsp::transport::ppqn());
//   if (pattern.gateAt(rpdsp::transport::stepIndex(t.tick, perStep))) fire();
//
// An earlier revision carried a private step_ plus processClock(bool). That was
// a second step counter racing uClock's, so it is gone: gateAt() wraps a
// monotonically rising tick-derived index into the active length, which is what
// the old playhead was reconstructing the hard way.
//
// Two accessors, deliberately distinct:
//   stepEnabled(i)  raw mask lookup, no wrapping — for UI grids drawing all
//                   MaxSteps slots regardless of the active length.
//   gateAt(i)       length-wrapping lookup — for the audio path, fed a
//                   tick-derived step index that keeps counting past length().

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rpdsp {

// Fixed-size step mask for trigger/gate patterns.
template <size_t MaxSteps = 32>
class GatePattern {
 public:
  // Clears every step; length optionally resizes the active prefix.
  void reset(size_t length = MaxSteps) {
    length_ = clampLength(length);
    steps_.fill(false);
  }

  void setLength(size_t length) { length_ = clampLength(length); }
  [[nodiscard]] size_t length() const { return length_; }
  static constexpr size_t maxSteps() { return MaxSteps; }

  void setStep(size_t step, bool enabled) {
    if (step < MaxSteps)
      steps_[step] = enabled;
  }

  void toggleStep(size_t step) {
    if (step < MaxSteps)
      steps_[step] = !steps_[step];
  }

  // Raw mask lookup. No wrapping: out-of-range reads false, so a UI can walk
  // 0..MaxSteps without knowing the active length.
  [[nodiscard]] bool stepEnabled(size_t step) const {
    return step < MaxSteps ? steps_[step] : false;
  }

  // Length-wrapping lookup. Feed this a tick-derived step index — it rises
  // without bound, and the wrap here is what makes a length of 7 polymeter
  // against a 16-step display.
  [[nodiscard]] bool gateAt(size_t stepIndex) const {
    if (length_ == 0)
      return false;
    return steps_[stepIndex % length_];
  }

  [[nodiscard]] bool anyEnabled() const {
    for (size_t i = 0; i < length_; ++i)
      if (steps_[i]) return true;
    return false;
  }

  void loadMask(std::uint32_t mask, size_t length = MaxSteps) {
    // Bit 0 maps to step 0. Steps outside the active length are still loaded
    // so changing length later preserves the mask contents.
    setLength(length);
    for (size_t i = 0; i < MaxSteps; ++i)
      steps_[i] = (mask & (std::uint32_t{1} << i)) != 0;
  }

  // Copy a row out of an external table (RhythmGateSequencer's layout), so a
  // constexpr preset bank in flash can seed an editable pattern in RAM.
  void loadRow(const bool* row, size_t stepCount) {
    if (row == nullptr) return;
    const size_t n = stepCount > MaxSteps ? MaxSteps : stepCount;
    for (size_t i = 0; i < n; ++i) steps_[i] = row[i];
    for (size_t i = n; i < MaxSteps; ++i) steps_[i] = false;
  }

 private:
  static size_t clampLength(size_t length) {
    if (length < 1)
      return 1;
    return length > MaxSteps ? MaxSteps : length;
  }

  std::array<bool, MaxSteps> steps_{};
  size_t length_ = MaxSteps;
};

}  // namespace rpdsp
