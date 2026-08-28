// transport.h — uClock's tick, published where DSP code can read it
// ---------------------------------------------------------------------------
// uClock is the timebase for this repo (see the root AGENTS.md). Its callback
// runs in a hardware-timer ISR, which is the wrong place to do musical work:
// a world's trigger action clears filter and delay state that Core 0 is
// concurrently reading inside processFrame(). So the ISR does one thing —
// publish its tick here — and the audio core derives everything from it.
//
// This header deliberately knows nothing about uClock. It is a plain counter
// plus the arithmetic every tempo-synced world needs, which keeps rpdsp
// portable and host-testable: the firmware feeds it from uClock's callback,
// the host suite feeds it synthetically.
//
//   Firmware (once, in setup):   rpdsp::transport::configure(PPQN, sampleRate)
//   Firmware (uClock callback):  rpdsp::transport::publish(tick, micros())
//   World (per frame):           const auto t = rpdsp::transport::tick();
//
// Threading: single writer (the clock ISR), many readers. tick and timestamp
// are published behind a sequence counter so a reader can detect the ISR
// landing between the two loads and retry — a torn pair would place an event
// in the wrong buffer.

#pragma once

#include "algorithm.h"

#include <cstddef>
#include <cstdint>

namespace rpdsp {
namespace transport {

// --- Musical divisions ------------------------------------------------------
// Ticks per division at a given PPQN. A quarter note is exactly `ppqn` ticks,
// so every entry below is a rational multiple of it: PPQN_384 makes all of
// them — including triplets and dotted values — exact integers.
enum class Division : std::uint8_t {
  Whole = 0,   // 1/1
  Half,        // 1/2
  Quarter,     // 1/4
  QuarterTrip, // 1/4T
  Eighth,      // 1/8
  EighthTrip,  // 1/8T
  Sixteenth,   // 1/16
  ThirtySecond,// 1/32
  Count
};

inline constexpr std::size_t kDivisionCount =
    static_cast<std::size_t>(Division::Count);

// Display labels; index with the same value used for Division.
inline constexpr const char* kDivisionNames[kDivisionCount] = {
    "1/1", "1/2", "1/4", "1/4T", "1/8", "1/8T", "1/16", "1/32"};

// Numerator/denominator of each division expressed in quarter notes:
// ticks = ppqn * num / den. Triplets are 2/3 of the straight value.
inline constexpr std::uint16_t kDivisionNumerator[kDivisionCount] =
    {4, 2, 1, 2, 1, 1, 1, 1};
inline constexpr std::uint16_t kDivisionDenominator[kDivisionCount] =
    {1, 1, 1, 3, 2, 3, 4, 8};

// Ticks per division at the given PPQN. Never returns 0, so callers can
// divide by it without guarding. constexpr so a sketch can initialise a
// namespace-scope constant with it and not depend on static init order.
[[nodiscard]] constexpr std::uint32_t ticksPerDivision(Division division,
                                                       std::uint32_t ppqn) {
  const std::size_t i = static_cast<std::size_t>(division);
  const std::size_t safe = i >= kDivisionCount ? kDivisionCount - 1 : i;
  const std::uint32_t ticks =
      (ppqn * kDivisionNumerator[safe]) / kDivisionDenominator[safe];
  return ticks == 0 ? 1u : ticks;
}

// Map a normalized [0,1] parameter onto a division, matching the rounding
// PicoOLED::formatParamValue and ParamCursor::snap use for `choices` lists so
// the label on screen and the division in the audio path never disagree.
[[nodiscard]] inline Division divisionFromNormalized(float normalized) {
  const float last = static_cast<float>(kDivisionCount - 1);
  const int index = static_cast<int>(clamp01(normalized) * last + 0.5f);
  return static_cast<Division>(index);
}

[[nodiscard]] inline const char* divisionName(Division division) {
  const std::size_t i = static_cast<std::size_t>(division);
  return kDivisionNames[i >= kDivisionCount ? kDivisionCount - 1 : i];
}

// --- Published clock state --------------------------------------------------

struct State {
  std::uint32_t tick = 0;        // uClock's output-PPQN tick counter
  std::uint32_t micros = 0;      // when that tick fired
  std::int8_t shuffleTicks = 0;  // uClock::getShuffleLength() at that tick
  bool running = false;
};

namespace detail {
// Single writer (clock ISR), many readers. The sequence counter is odd while
// a write is in flight, so readers spin instead of seeing a half-updated pair.
inline volatile std::uint32_t g_sequence = 0;
inline volatile std::uint32_t g_tick = 0;
inline volatile std::uint32_t g_micros = 0;
inline volatile std::int8_t g_shuffleTicks = 0;
inline volatile bool g_running = false;
inline std::uint32_t g_ppqn = 384;
inline float g_sampleRate = 48000.0f;
}  // namespace detail

// Call once at setup with the resolution passed to uClock.setOutputPPQN()
// and the audio sample rate.
inline void configure(std::uint32_t ppqn, float sampleRate) {
  detail::g_ppqn = ppqn == 0 ? 384u : ppqn;
  detail::g_sampleRate = sampleRate > 1.0f ? sampleRate : 48000.0f;
}

[[nodiscard]] inline std::uint32_t ppqn() { return detail::g_ppqn; }
[[nodiscard]] inline float sampleRate() { return detail::g_sampleRate; }

// Called from the clock ISR. Keep this the entire body of that callback.
inline void publish(std::uint32_t tick, std::uint32_t nowMicros,
                    std::int8_t shuffleTicks = 0) {
  detail::g_sequence = detail::g_sequence + 1;  // odd: write in flight
  detail::g_tick = tick;
  detail::g_micros = nowMicros;
  detail::g_shuffleTicks = shuffleTicks;
  detail::g_sequence = detail::g_sequence + 1;  // even: consistent again
}

inline void setRunning(bool running) { detail::g_running = running; }

// Reset to a stopped, tick-zero state. Setup/teardown only — not the audio path.
inline void reset() {
  detail::g_sequence = detail::g_sequence + 1;
  detail::g_tick = 0;
  detail::g_micros = 0;
  detail::g_shuffleTicks = 0;
  detail::g_running = false;
  detail::g_sequence = detail::g_sequence + 1;
}

// Coherent snapshot. Bounded retry: the ISR is two stores, so it cannot
// starve a reader, and the loop gives up rather than spinning in the audio
// path if something ever violates the single-writer rule.
[[nodiscard]] inline State read() {
  State out;
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint32_t before = detail::g_sequence;
    if ((before & 1u) != 0u) continue;  // write in flight
    out.tick = detail::g_tick;
    out.micros = detail::g_micros;
    out.shuffleTicks = detail::g_shuffleTicks;
    out.running = detail::g_running;
    if (detail::g_sequence == before) return out;
  }
  return out;
}

// Just the tick, for the common case.
[[nodiscard]] inline std::uint32_t tick() { return read().tick; }

// --- Tick arithmetic --------------------------------------------------------
// The house pattern from Examples/Arp: every musical decision is integer
// arithmetic on the tick. These are the three expressions worlds keep needing.

// Which step of the given division `tick` falls in.
[[nodiscard]] inline std::uint32_t stepIndex(std::uint32_t tick,
                                             std::uint32_t ticksPerStep) {
  return ticksPerStep == 0 ? 0 : tick / ticksPerStep;
}

// How far into the current step `tick` is, in ticks.
[[nodiscard]] inline std::uint32_t stepPhase(std::uint32_t tick,
                                             std::uint32_t ticksPerStep) {
  return ticksPerStep == 0 ? 0 : tick % ticksPerStep;
}

// Gate high for the first `gateFraction` of each step. Equivalent to the
// stepPhase < gateTicks comparison written out in Arp.ino.
[[nodiscard]] inline bool gateOpen(std::uint32_t tick,
                                   std::uint32_t ticksPerStep,
                                   float gateFraction) {
  if (ticksPerStep == 0) return false;
  const std::uint32_t gateTicks = static_cast<std::uint32_t>(
      static_cast<float>(ticksPerStep) * clamp01(gateFraction));
  return stepPhase(tick, ticksPerStep) < gateTicks;
}

// Samples per tick at the configured PPQN and tempo. Only needed to convert a
// tick timestamp into a frame offset; worlds that just want "did the step
// change" should compare stepIndex() instead of counting samples.
[[nodiscard]] inline float samplesPerTick(float bpm) {
  if (bpm <= 0.0f) return 0.0f;
  return (detail::g_sampleRate * 60.0f) /
         (bpm * static_cast<float>(detail::g_ppqn));
}

}  // namespace transport
}  // namespace rpdsp
