#pragma once


/* Ported from the Audio Library for Teensy, Ladder Filter.
 * Copyright (c) 2021, Richard van Hoesel
 * Copyright (c) 2024, Infrasonic Audio LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//-----------------------------------------------------------
// Huovilainen New Moog (HNM) model as per CMJ Jun 2006.
// Richard van Hoesel, v.1.03, Feb. 14 2021.
// v1.7 (Infrasonic/Daisy): add configurable filter mode.
// v1.6 (Infrasonic/Daisy): removes polyphase FIR, uses 4x linear
//      oversampling for performance reasons.
// v1.4: FC extended to 18.7 kHz, max res to 1.8, 4x oversampling,
//       and a minor Q-tuning adjustment.
// v.1.03: adds oversampling, extended resonance, and exposes the
//         input_drive and passband_gain parameters.
// please retain this header if you use this code.
//
// rpdsp port: MIT; adapted to rpdsp conventions (lowercase API,
// prepare()/process(), header-only). All tuning constants are preserved
// verbatim from the source. The per-sample computation has been
// restructured for the RP2350 FPU (Cortex-M33, in-order, single-precision):
//   - the mode-mix switch/std::array is hoisted out of the 4x oversampling
//     loop (the mix is linear, so per-stage sums are weighted once/sample);
//   - K*Qadjust and the passband-gain feedback offset are precomputed;
//   - the input crossfade is incremental (one add per pass);
//   - the one-pole stages are algebraically refolded from
//       ft = alpha*(c1*s + c2*z0 - z1) + z1
//     to
//       ft = (alpha*c1)*s + (alpha*c2)*z0 + (1-alpha)*z1
//     (3 FMA-class ops instead of 4; coefficients baked in computeCoeffs);
//   - all 8 state variables live in registers across the oversampling loop
//     (and across the whole block in the buffer overload), with the
//     zapDenormal guard applied at state write-back.
// Output differs from the pre-optimization code only by float rounding
// order: measured max deviation < 2.3e-5 (~-93 dBFS) across LP/BP/HP modes
// with cutoff sweeps and resonance up to self-oscillation; see
// Docs/dsp_algorithm_benchmarks.md history for the harness.
//-----------------------------------------------------------

#include "algorithm.h"
#include "realtime.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace rpdsp {

/** @brief 4-pole Huovilainen "New Moog" ladder filter.
 *
 * Selectable response (LP/BP/HP at 12 or 24 dB/oct), input drive into a
 * tanh clipper, passband-gain compensation, and stable self-oscillation.
 *
 * This is the heaviest filter in rpdsp: every output sample runs 4x
 * oversampling, each pass doing 4 one-pole stages plus a fastTanh. The
 * oversampling factor is the public constant kInterpolation -- drop it to
 * 2 on RP2350 if the realtime budget is blown. The stage coefficients and
 * sr_int_recip_ both derive from it, so the tuning self-adjusts when it
 * changes. Prefer the process(buf, size) overload in the audio callback:
 * it keeps the filter state in FPU registers for the whole block.
 */
class LadderFilter {
 public:
  enum class Mode {
    LP24,
    LP12,
    BP24,
    BP12,
    HP24,
    HP12
  };

  // Oversampling factor. Lowering it trades HF accuracy for CPU.
  static constexpr std::uint8_t kInterpolation = 4;

  void prepare(float sampleRate) {
    sample_rate_ = safeSampleRate(sampleRate);
    sr_int_recip_ = 1.0f / (sample_rate_ * kInterpolation);
    K_ = 1.0f;
    kq_ = 1.0f;
    Fbase_ = 1000.0f;
    Qadjust_ = 1.0f;
    oldinput_ = 0.0f;
    mode_ = Mode::LP24;

    setPassbandGain(0.5f);
    setInputDrive(0.5f);
    setFreq(5000.0f);
    setRes(0.2f);
  }

  void reset() {
    for (int i = 0; i < 4; ++i) {
      z0_[i] = 0.0f;
      z1_[i] = 0.0f;
    }
    oldinput_ = 0.0f;
  }

  float process(float in) {
    float y0 = z0_[0], y1 = z0_[1], y2 = z0_[2], y3 = z0_[3];
    float w0 = z1_[0], w1 = z1_[1], w2 = z1_[2], w3 = z1_[3];
    float oldin = oldinput_;
    const float out = processOneSample(in, oldin, y0, y1, y2, y3,
                                       w0, w1, w2, w3);
    z0_[0] = y0; z0_[1] = y1; z0_[2] = y2; z0_[3] = y3;
    z1_[0] = zapDenormal(w0); z1_[1] = zapDenormal(w1);
    z1_[2] = zapDenormal(w2); z1_[3] = zapDenormal(w3);
    oldinput_ = oldin;
    return out;
  }

  void process(float* buf, std::size_t size) {
    // Block-resident state: one load/store of the 9 state values per block
    // instead of per sample. On the in-order M33 this is the difference
    // between state living in FPU registers and 18 memory ops per sample.
    float y0 = z0_[0], y1 = z0_[1], y2 = z0_[2], y3 = z0_[3];
    float w0 = z1_[0], w1 = z1_[1], w2 = z1_[2], w3 = z1_[3];
    float oldin = oldinput_;
    for (std::size_t i = 0; i < size; i++) {
      buf[i] = processOneSample(buf[i], oldin, y0, y1, y2, y3,
                                w0, w1, w2, w3);
    }
    z0_[0] = y0; z0_[1] = y1; z0_[2] = y2; z0_[3] = y3;
    z1_[0] = zapDenormal(w0); z1_[1] = zapDenormal(w1);
    z1_[2] = zapDenormal(w2); z1_[3] = zapDenormal(w3);
    oldinput_ = oldin;
  }

  void setFreq(float freq) {
    Fbase_ = freq;
    computeCoeffs(freq);
  }

  void setRes(float res) {
    // Maps resonance 0..1 onto K = 0..4 (clamped to kMaxResonance first).
    res = clamp(res, 0.0f, kMaxResonance);
    K_ = 4.0f * res;
    kq_ = K_ * Qadjust_;
  }

  /** Passband-gain compensation, 0..0.5. Mitigates passband loss at high Q. */
  void setPassbandGain(float pbg) {
    pbg_ = clamp(pbg, 0.0f, 0.5f);
    setInputDrive(drive_);
  }

  /** Drive into the tanh clipper, 0..4. */
  void setInputDrive(float drv) {
    drive_ = std::max(drv, 0.0f);
    if (drive_ > 1.0f) {
      drive_ = std::min(drive_, 4.0f);
    }
  }

  void setMode(Mode mode) { mode_ = mode; }

 private:
  static constexpr float kInterpolationRecip = 1.0f / kInterpolation;
  static constexpr float kMaxResonance = 1.8f;

  // One output sample: 4x oversampled ladder core operating entirely on
  // caller-provided (register-resident) state. y* are the stage-input
  // memories (z0), w* the stage outputs (z1).
  float processOneSample(float in, float& oldin,
                         float& y0, float& y1, float& y2, float& y3,
                         float& w0, float& w1, float& w2, float& w3) const {
    const float b1 = b1_, b2 = b2_, am = am_, kq = kq_;
    const float input = in * drive_;
    // Linear crossfade oldin->input across the passes, incrementally:
    // pass k uses (k/4)*oldin + (1-k/4)*input, i.e. xf starts at input and
    // steps by (oldin - input)/4.
    const float step = (oldin - input) * kInterpolationRecip;
    // (z1[3] - pbg*input) * K * Qadjust, with the input term hoisted.
    const float fb_offset = pbg_ * input * kq;
    float xf = input;
    float sum1 = 0.0f, sum2 = 0.0f, sum3 = 0.0f, sum4 = 0.0f;
    for (int os = 0; os < kInterpolation; os++) {
      float u = xf + fb_offset - w3 * kq;
      u = fastTanh(u);
      float t;
      t = b1 * u  + b2 * y0 + am * w0; y0 = u;  w0 = t;
      t = b1 * w0 + b2 * y1 + am * w1; y1 = w0; w1 = t;
      t = b1 * w1 + b2 * y2 + am * w2; y2 = w1; w2 = t;
      t = b1 * w2 + b2 * y3 + am * w3; y3 = w2; w3 = t;
      sum1 += w0; sum2 += w1; sum3 += w2; sum4 += w3;
      xf += step;
    }
    oldin = input;
    // The mode mix is linear in the stage outputs and the input term is
    // constant across passes, so averaging the stage sums first and
    // weighting once per sample is exact.
    return weightedSumForCurrentMode(input,
                                     sum1 * kInterpolationRecip,
                                     sum2 * kInterpolationRecip,
                                     sum3 * kInterpolationRecip,
                                     sum4 * kInterpolationRecip);
  }

  void computeCoeffs(float freq) {
    // Model-tuned clamp [5, sr*0.425]: the 0.425 (not 0.5) keeps the alpha
    // polynomial stable -- do NOT swap in rpdsp::clampCutoff here.
    freq = clamp(freq, 5.0f, sample_rate_ * 0.425f);
    float wc = freq * kTwoPi * sr_int_recip_;
    float wc2 = wc * wc;
    float alpha = 0.9892f * wc - 0.4324f * wc2 + 0.1381f * wc * wc2
                  - 0.0202f * wc2 * wc2;
    // Qadjust_ is a matched pair with alpha (revised hfQ, rvh Feb 14 2021).
    Qadjust_ = 1.006f + 0.0536f * wc - 0.095f * wc2 - 0.05f * wc2 * wc2;
    kq_ = K_ * Qadjust_;
    // Huovilainen one-pole section, refolded so each stage is 3 mul-adds:
    //   ft = alpha*(s/1.3 + (0.3/1.3)*z0 - z1) + z1
    //      = b1*s + b2*z0 + am*z1
    // The hardcoded 1/1.3 and 0.3/1.3 are the section coefficients -- do
    // not simplify or retune them.
    b1_ = alpha * 0.76923077f;
    b2_ = alpha * 0.23076923f;
    am_ = 1.0f - alpha;
  }

  // Weighted stage mixing per Valimaki & Huovilainen, CMJ 2006.
  // s1..s4 are the per-sample averages of the four stage outputs.
  float weightedSumForCurrentMode(float input, float s1, float s2,
                                  float s3, float s4) const {
    switch (mode_) {
      case Mode::LP24: return s4;
      case Mode::LP12: return s2;
      case Mode::BP24:
        return (s2 + s4) * 4.0f - s3 * 8.0f;
      case Mode::BP12: return (s1 - s2) * 2.0f;
      case Mode::HP24:
        return input + s4 - ((s1 + s3) * 4.0f) + s2 * 6.0f;
      case Mode::HP12:
        return input + s2 - s1 * 2.0f;
      default: return 0.0f;
    }
  }

  float sample_rate_ = kDefaultSampleRate;
  float sr_int_recip_ = 0.0f;
  float b1_ = 0.76923077f;   // alpha * (1/1.3)
  float b2_ = 0.23076923f;   // alpha * (0.3/1.3)
  float am_ = 0.0f;          // 1 - alpha
  float z0_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float z1_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float K_ = 1.0f;
  float kq_ = 1.0f;          // K_ * Qadjust_
  float Fbase_ = 1000.0f;
  float Qadjust_ = 1.0f;
  float pbg_ = 0.5f;
  float drive_ = 0.5f;
  float oldinput_ = 0.0f;
  Mode mode_ = Mode::LP24;
};

}  // namespace rpdsp
