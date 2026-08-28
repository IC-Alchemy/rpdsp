#pragma once

#include <cstdint>

namespace rpdsp {

// Allocation-free real-time budget tracker for an audio callback block.
// Caller supplies a monotonic tick count (e.g. micros(), a hardware cycle
// counter) at the start and end of each block; the meter publishes
// worst-case, rolling-average, and overrun count via plain members safe for
// a second core to poll (same volatile-publication convention as the rest
// of the cross-core state in this codebase — single writer on the audio
// core, single reader elsewhere, no locks).
class CallbackPerformanceMeter {
 public:
  // budgetTicks is the deadline for one block, in the same tick units as
  // beginBlock/endBlock (e.g. microseconds if using micros()). Pass 0 to
  // disable overrun counting.
  void prepare(std::uint32_t budgetTicks) {
    budgetTicks_ = budgetTicks;
    reset();
  }

  void reset() {
    startTicks_ = 0;
    lastTicks_ = 0;
    worstTicks_ = 0;
    averageTicks_ = 0.0f;
    overrunCount_ = 0;
  }

  void beginBlock(std::uint32_t nowTicks) { startTicks_ = nowTicks; }

  void endBlock(std::uint32_t nowTicks) {
    // Unsigned subtraction wraps correctly across tick-counter rollover.
    const std::uint32_t elapsed = nowTicks - startTicks_;
    lastTicks_ = elapsed;
    if (elapsed > worstTicks_) worstTicks_ = elapsed;
    averageTicks_ += (static_cast<float>(elapsed) - averageTicks_) * kAverageSmoothing;
    if (budgetTicks_ != 0 && elapsed > budgetTicks_) ++overrunCount_;
  }

  // Non-realtime accessors — poll these from the control core / UI code.
  std::uint32_t lastTicks() const { return lastTicks_; }
  std::uint32_t worstTicks() const { return worstTicks_; }
  float averageTicks() const { return averageTicks_; }
  std::uint32_t overrunCount() const { return overrunCount_; }
  std::uint32_t budgetTicks() const { return budgetTicks_; }

 private:
  static constexpr float kAverageSmoothing = 1.0f / 16.0f;

  std::uint32_t budgetTicks_ = 0;
  std::uint32_t startTicks_ = 0;
  volatile std::uint32_t lastTicks_ = 0;
  volatile std::uint32_t worstTicks_ = 0;
  volatile float averageTicks_ = 0.0f;
  volatile std::uint32_t overrunCount_ = 0;
};

// Deadline budget in microseconds for one block of blockSizeFrames at
// sampleRate, per Docs/realtime_rules.md's "Deadline Budget" table.
inline float callbackBudgetUs(int blockSizeFrames, float sampleRate) {
  return 1.0e6f * static_cast<float>(blockSizeFrames) / sampleRate;
}

}  // namespace rpdsp
