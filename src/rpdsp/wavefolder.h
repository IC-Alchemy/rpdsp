#pragma once

#include "realtime.h"

#include <cmath>

namespace rpdsp {

// Triangle wavefolder. Amplitudes beyond +/-1 start folding back on
// themselves; gain scales the input (negative gain folds thru-zero) and offset
// shifts the input pre-gain for asymmetrical folding.
//
// Math ported from DaisySP's Wavefolder (Copyright (c) 2020 Electrosmith,
// Corp, Nick Donaldson, MIT license) into rpdsp conventions.
class Wavefolder {
 public:
  void setGain(float gain) { gain_ = gain; }
  void setOffset(float offset) { offset_ = offset; }

  float process(float input) {
    const float x = (input + offset_) * gain_;
    const float ft = std::floor((x + 1.0f) * 0.5f);
    const float sgn = static_cast<int>(ft) % 2 == 0 ? 1.0f : -1.0f;
    return zapDenormal(sgn * (x - 2.0f * ft));
  }

 private:
  float gain_ = 1.0f;
  float offset_ = 0.0f;
};

}  // namespace rpdsp
