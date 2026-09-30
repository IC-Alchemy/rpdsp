#pragma once

#include <cstddef>

// Placement hook for the few hot functions a firmware wants to run from RAM
// instead of flash (currently DarkReverb's two process() overloads). Empty by
// default, so nothing changes for anyone who does not define it. Define it before
// the first rpdsp header is included, for example on a Pico SDK target:
//   #define RPDSP_HOT_FUNCTION __attribute__((section(".time_critical.rpdsp")))
// Template functions cannot be named in a wrapper, and an out-of-line callee is
// not moved just because its caller is, so check the linked map (the symbol's
// address should be in SRAM) rather than assuming.
#ifndef RPDSP_HOT_FUNCTION
#define RPDSP_HOT_FUNCTION
#endif

#ifndef RPDSP_BLOCK_SIZE
#define RPDSP_BLOCK_SIZE 32
#endif

namespace rpdsp {

// The portable DSP layer is tuned around short, predictable audio callbacks.
inline constexpr float kDefaultSampleRate = 48000.0f;
inline constexpr size_t kDefaultBlockSize = RPDSP_BLOCK_SIZE;
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;

// Keep host tests and firmware examples on the same small set of realtime block budgets.
static_assert(kDefaultBlockSize == 16 || kDefaultBlockSize == 32 || kDefaultBlockSize == 64,
              "RPDSP_BLOCK_SIZE must be 16, 32, or 64");

}  // namespace rpdsp
