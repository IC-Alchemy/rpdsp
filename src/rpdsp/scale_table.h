#pragma once

// Musical scale step -> semitone offset tables. Ported from Pico2Seq's
// lib/pico2seq-core/scales/scales.{h,cpp}; data is kept identical, just
// namespaced and made header-only (inline constexpr, C++17) to match the
// rest of rpdsp, with the mutable "currently selected scale" state left to
// the caller instead of living as a library-owned global.

#include <array>
#include <cstddef>

namespace rpdsp {

// Number of distinct scale definitions.
inline constexpr size_t kScaleCount = 13;
// Number of step-to-semitone entries per scale.
inline constexpr size_t kScaleSteps = 48;

using ScaleSteps = std::array<int, kScaleSteps>;

// Human-readable scale names, indexed the same as kScaleTable.
inline constexpr std::array<const char*, kScaleCount> kScaleNames = {
    "Ionian Major",
    "Dorian",
    "Phrygian",
    "Lydian",
    "Mixolydian",
    "Aeolian Minor",
    "Locrian",
    "Pentatonic Minor",
    "Phrygian Dominant",
    "Lydian Dominant",
    "Harmonic Minor",
    "Wholetone",
    "Chromatic",
};

// clang-format off
inline constexpr std::array<ScaleSteps, kScaleCount> kScaleTable = {{
    // Ionian (Major): 1-2-3-4-5-6-7
    ScaleSteps{0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24, 26,
     28, 29, 31, 33, 35, 36, 38, 40, 41, 43, 45, 47, 48, 50, 52, 53,
     55, 57, 59, 60, 62, 64, 66, 67, 69, 71, 72, 72, 72, 72, 72, 72},

    // Dorian: 1-2-b3-4-5-6-b7
    ScaleSteps{0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26,
     27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53,
     55, 56, 58, 60, 62, 63, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},

    // Phrygian: 1-b2-b3-4-5-b6-b7
    ScaleSteps{0, 1, 3, 5, 7, 8, 10, 12, 13, 15, 17, 19, 20, 22, 24, 25,
     27, 29, 31, 32, 34, 36, 37, 39, 41, 43, 44, 46, 48, 49, 51, 53,
     55, 56, 58, 60, 61, 63, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},

    // Lydian: 1-2-3-#4-5-6-7
    ScaleSteps{0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26,
     28, 30, 31, 33, 35, 36, 38, 40, 42, 43, 45, 47, 48, 50, 52, 54,
     55, 57, 59, 60, 62, 64, 66, 67, 69, 71, 72, 72, 72, 72, 72, 72},

    // Mixolydian: 1-2-3-4-5-6-b7
    ScaleSteps{0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26,
     28, 29, 31, 33, 34, 36, 38, 40, 41, 43, 45, 46, 48, 50, 52, 53,
     55, 57, 58, 60, 62, 64, 65, 67, 69, 70, 72, 72, 72, 72, 72, 72},

    // Aeolian (Natural Minor): 1-2-b3-4-5-b6-b7
    ScaleSteps{0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26,
     27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53,
     55, 56, 58, 60, 61, 63, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},

    // Locrian: 1-b2-b3-4-b5-b6-b7
    ScaleSteps{0, 1, 3, 5, 6, 8, 10, 12, 13, 15, 17, 19, 20, 22, 24, 25,
     27, 29, 31, 32, 34, 35, 37, 39, 41, 42, 44, 46, 48, 49, 51, 53,
     54, 56, 58, 60, 61, 63, 65, 66, 68, 70, 72, 72, 72, 72, 72, 72},

    // Pentatonic Minor: 1-b3-4-5-b7 (duplicated steps to fill grid)
    ScaleSteps{0, 0, 3, 3, 5, 5, 7, 7, 10, 10, 12, 12, 15, 15, 17, 17,
     19, 19, 22, 22, 24, 24, 27, 29, 29, 29, 32, 32, 34, 34, 36, 36,
     39, 39, 41, 41, 43, 43, 46, 46, 48, 48, 51, 53, 53, 53, 53, 53},

    // Phrygian Dominant (5th mode of harmonic minor: 1-b2-3-4-5-b6-b7)
    ScaleSteps{0, 1, 4, 5, 7, 8, 10, 12, 13, 16, 17, 19, 20, 22, 24, 25,
     28, 29, 31, 32, 34, 36, 37, 40, 41, 43, 44, 46, 48, 49, 52, 53,
     55, 56, 58, 60, 61, 64, 65, 67, 68, 70, 72, 72, 72, 72, 72, 72},

    // Lydian Dominant (4th mode of melodic minor: 1-2-3-#4-5-6-b7)
    ScaleSteps{0, 2, 4, 6, 7, 9, 10, 12, 14, 16, 18, 19, 21, 22, 24, 26,
     28, 30, 31, 33, 34, 36, 38, 40, 42, 43, 45, 46, 48, 50, 52, 54,
     55, 57, 58, 60, 62, 64, 66, 67, 69, 70, 72, 72, 72, 72, 72, 72},

    // Harmonic Minor: 1-2-b3-4-5-b6-7
    ScaleSteps{0, 2, 3, 5, 7, 8, 11, 12, 14, 15, 17, 19, 20, 23, 24, 26,
     27, 29, 31, 32, 35, 36, 38, 39, 41, 43, 44, 47, 48, 50, 51, 53,
     55, 56, 59, 60, 62, 63, 65, 67, 68, 71, 72, 72, 72, 72, 72, 72},

    // Whole Tone
    ScaleSteps{0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
     32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
     64, 66, 68, 70, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72, 72},

    // Chromatic
    ScaleSteps{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
     16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
     32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47},
}};
// clang-format on

// Looks up a semitone offset for (scaleIndex, stepIndex), clamping both to
// valid ranges so callers can't index out of bounds.
[[nodiscard]] inline int scaleStepSemitones(size_t scaleIndex, size_t stepIndex) {
  const size_t safeScale = scaleIndex >= kScaleCount ? kScaleCount - 1 : scaleIndex;
  const size_t safeStep = stepIndex >= kScaleSteps ? kScaleSteps - 1 : stepIndex;
  return kScaleTable[safeScale][safeStep];
}

// Human-readable name for a scale index, clamped to a valid entry.
[[nodiscard]] inline const char* scaleName(size_t scaleIndex) {
  return kScaleNames[scaleIndex >= kScaleCount ? kScaleCount - 1 : scaleIndex];
}

}  // namespace rpdsp
