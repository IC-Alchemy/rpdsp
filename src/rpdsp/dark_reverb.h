#pragma once

// DarkReverb: long, dark stereo reverb sized for Cortex-M33 microcontrollers
// (RP2350 and other single-precision FPv5 parts).
//
// Signal flow (L+R summed to mono in, stereo out):
//
//   in -> 2:1 half-band -> low cut -> 4 allpass diffusers --+-----------+
//                                                           v           v
//   +-> [stage 0] -> [stage 1] -> [stage 2] -> [stage 3] --+           |
//   +-------------------------------------------------------+           |
//   stage k = decay gain -> modulated allpass -> allpass -> lowpass -> delay
//   (the diffused input enters stages 0 and 2, stage 0 also holds a 1 Hz
//    subsonic trap, and stereo taps are read from inside the four delays)
//   -> 1:2 half-band -> dry/wet mix
//
// The tail length comes from the decay gains, not from delay memory, so a
// small buffer can ring for minutes. Slowly modulated allpasses in every
// stage keep the modes moving so a long tail does not ring metallically.
//
// What keeps it cheap on an M33:
// - The tank runs at half the host rate. A dark tail has nothing to lose
//   above ~10 kHz at 48 kHz, and every delay covers twice the time per byte.
//   The elliptic half-band filters use three allpass multiplies per tank
//   sample and pass 0-9.6 kHz (at 48 kHz) with >53 dB image/alias rejection.
// - All sixteen delay lines share one power-of-two ring buffer with a single
//   pointer that moves once per tank sample. Every tap is (pointer +
//   compile-time constant) & mask: no per-line indices and no wrap branches.
// - Default storage is IEEE half precision, half the RAM of float. Cortex-M
//   FPUs (FPv4/FPv5) convert with one VCVTB instruction each way, so a
//   load is LDRH+VMOV+VCVTB. Fixed-point Q15 would cost the same but loses
//   about half an LSB per write whether it truncates or rounds, which
//   shortens quiet tails or leaves limit cycles. Half's error is relative
//   (11 significant bits, round to nearest even), so the decay rate does not
//   depend on level, and subnormals carry the tail down to about -140 dBFS.
//   DarkReverbStorage::Float gives 24-bit precision at twice the RAM.
// - Setters compute all coefficients; the audio path has no libm call and
//   no divide.
// - Two magic-circle quadrature LFOs cost two multiply-adds each per tank
//   sample, with no sin().
// - The block API runs decimation, tank and interpolation as three passes
//   over chunks of 16 tank samples, which keeps more state in FPU registers.
//
// Original baseline cost (arm-none-eabi-gcc 13.2, -mcpu=cortex-m33 -O2,
// static count of the block loops): 269 instructions per stereo output sample with Half storage,
// 232 with Float. Estimated 270-375 cycles, i.e. 9-12% of one 150 MHz
// RP2350 core at 48 kHz. Not yet measured on hardware.
//
// RAM: 2 bytes (Half) or 4 bytes (Float) per Capacity sample, plus 248
// bytes of state. Capacity counts tank samples, and the tank runs at
// sampleRate / 2:
//   DarkReverb<8192>  = 16 KB, ring 0.31 s at 48 kHz (smaller, grainier)
//   DarkReverb<16384> = 32 KB, ring 0.64 s at 48 kHz (default)
//   DarkReverb<32768> = 64 KB, ring 1.28 s at 48 kHz (densest modes)
// Delay lengths scale with Capacity, not with the sample rate. At 96 kHz,
// use twice the Capacity for the same ring time.
//
// Construct statically (the buffer is a member), prepare(sampleRate) once,
// then call process(left, right) per sample or the block overload.

#include "algorithm.h"
#include "config.h"
#include "realtime.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#if defined(__GNUC__)
#define RPDSP_DARK_REVERB_INLINE inline __attribute__((always_inline))
#else
#define RPDSP_DARK_REVERB_INLINE inline
#endif

// Every Cortex-M FPU (FPv4-SP on M4F, FPv5 on M7/M33) has VCVTB for
// half<->single conversion. GCC only exposes __fp16 with -mfp16-format=ieee,
// which Arduino and the Pico SDK do not pass, so use the instruction
// directly. Softfp and hard-float ABIs both have the S registers.
#if defined(__GNUC__) && defined(__arm__) && defined(__ARM_ARCH_PROFILE) && \
    (__ARM_ARCH_PROFILE == 'M') && defined(__ARM_FP) && (__ARM_FP & 4)
#define RPDSP_DARK_REVERB_VCVTB 1
#else
#define RPDSP_DARK_REVERB_VCVTB 0
#endif

namespace rpdsp {

enum class DarkReverbStorage {
  Half,   // IEEE binary16, 2 bytes per sample (default)
  Float,  // binary32, 4 bytes per sample
};

namespace dark_reverb_detail {

// Portable binary32 -> binary16, round to nearest even. Bit-exact with
// hardware IEEE conversion for every non-NaN input (checked against x86
// F16C over all 2^32 patterns); NaN becomes the default quiet NaN.
inline std::uint16_t floatToHalfSoftware(float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof bits);
  const std::uint32_t sign = (bits >> 16) & 0x8000u;
  bits &= 0x7FFFFFFFu;
  if (bits >= 0x47800000u) {  // |value| >= 65536, inf or NaN
    return static_cast<std::uint16_t>(sign | (bits > 0x7F800000u ? 0x7E00u : 0x7C00u));
  }
  if (bits < 0x38800000u) {  // below 2^-14: half subnormal or zero
    // Adding 0.5 lines the ten subnormal bits up at the bottom of the float
    // mantissa, and the FPU's own round-to-nearest-even rounds them.
    float aligned;
    std::memcpy(&aligned, &bits, sizeof aligned);
    aligned += 0.5f;
    std::uint32_t alignedBits;
    std::memcpy(&alignedBits, &aligned, sizeof alignedBits);
    return static_cast<std::uint16_t>(sign | (alignedBits - 0x3F000000u));
  }
  // Normal: rebias the exponent (-112 << 23) and round the 13 dropped bits
  // to nearest even. A carry out of the mantissa bumps the exponent, up to
  // infinity for values that round past 65504.
  bits += 0xC8000FFFu + ((bits >> 13) & 1u);
  return static_cast<std::uint16_t>(sign | (bits >> 13));
}

// Portable binary16 -> binary32 (exact).
inline float halfToFloatSoftware(std::uint16_t half) {
  std::uint32_t bits = static_cast<std::uint32_t>(half & 0x7FFFu) << 13;
  const std::uint32_t exponent = bits & 0x0F800000u;
  bits += 0x38000000u;  // rebias exponent: (127 - 15) << 23
  if (exponent == 0x0F800000u) {
    bits += 0x38000000u;  // inf/NaN keep an all-ones exponent
  } else if (exponent == 0) {
    // Zero/subnormal: add one to the exponent, then subtract 2^-14 so the FPU
    // renormalises.
    bits += 0x00800000u;
    float value;
    std::memcpy(&value, &bits, sizeof value);
    value -= 6.103515625e-05f;
    std::memcpy(&bits, &value, sizeof bits);
  }
  bits |= static_cast<std::uint32_t>(half & 0x8000u) << 16;
  float out;
  std::memcpy(&out, &bits, sizeof out);
  return out;
}

inline std::uint16_t floatToHalf(float value) {
#if RPDSP_DARK_REVERB_VCVTB
  // VCVTB writes the low half of the S register; STRH ignores the top half.
  float converted;
  __asm__("vcvtb.f16.f32 %0, %1" : "=t"(converted) : "t"(value));
  std::uint32_t bits;
  __asm__("vmov %0, %1" : "=r"(bits) : "t"(converted));
  return static_cast<std::uint16_t>(bits);
#else
  return floatToHalfSoftware(value);
#endif
}

inline float halfToFloat(std::uint16_t half) {
#if RPDSP_DARK_REVERB_VCVTB
  float out;
  __asm__("vmov %0, %1\n\tvcvtb.f32.f16 %0, %0" : "=t"(out) : "r"(static_cast<std::uint32_t>(half)));
  return out;
#else
  return halfToFloatSoftware(half);
#endif
}

// Line order in the shared buffer.
enum Line : int {
  kIn0, kIn1, kIn2, kIn3,              // input diffusers
  kModAp0, kModAp1, kModAp2, kModAp3,  // modulated loop allpasses
  kAp0, kAp1, kAp2, kAp3,              // fixed loop allpasses
  kDel0, kDel1, kDel2, kDel3,          // stage delays (output taps)
  kNumLines
};

// Modulation reach of a modulated allpass, in tank samples each way
// (1.33 ms at a 48 kHz host rate).
inline constexpr std::uint32_t kMaxModSamples = 32;

// Nominal lengths in tank samples for a 16384-sample buffer (24 kHz tank):
// distinct primes, with stage totals of 3791, 3931, 3475 and 4067 so no two
// stages share a period. makeLayout() rescales them for other capacities.
inline constexpr std::array<std::uint32_t, kNumLines> kNominalLengths = {
    107, 163, 263, 383,     // 4.5-16 ms input diffusion
    617, 701, 787, 857,     // modulated allpasses
    1427, 1213, 1051, 1321, // fixed allpasses
    1747, 2017, 1637, 1889, // delays
};

constexpr bool isModulated(std::size_t line) { return line >= kModAp0 && line <= kModAp3; }

// A line's region holds ages 0..length. A modulated line also covers the
// LFO swing (it reads ages length-33 up to length+32) plus a guard sample.
constexpr std::uint32_t regionPadding(std::size_t line) {
  return 1u + (isModulated(line) ? kMaxModSamples + 2u : 0u);
}

struct Layout {
  std::array<std::uint32_t, kNumLines> base{};
  std::array<std::uint32_t, kNumLines> length{};
  std::uint32_t end = 0;
};

// Scale the nominal lengths to fill the buffer after padding. Flooring
// keeps the total within capacity.
constexpr Layout makeLayout(std::size_t capacity) {
  std::uint64_t nominalSum = 0;
  std::uint64_t padding = 0;
  for (std::size_t i = 0; i < kNumLines; ++i) {
    nominalSum += kNominalLengths[i];
    padding += regionPadding(i);
  }
  const std::uint64_t budget = capacity - padding;
  Layout layout{};
  std::uint32_t base = 0;
  for (std::size_t i = 0; i < kNumLines; ++i) {
    const auto length = static_cast<std::uint32_t>(kNominalLengths[i] * budget / nominalSum);
    layout.base[i] = base;
    layout.length[i] = length;
    base += length + regionPadding(i);
  }
  layout.end = base;
  return layout;
}

// Elliptic half-band polyphase allpass coefficients (transition band
// 0.4-0.6 of the tank Nyquist, 53 dB stopband). Path A runs coefficients 0
// and 2 on the later sample of each pair, path B coefficient 1 on the earlier.
inline constexpr float kHalfband0 = 0.12845635f;
inline constexpr float kHalfband1 = 0.42956674f;
inline constexpr float kHalfband2 = 0.79067550f;

}  // namespace dark_reverb_detail

template <std::size_t Capacity = 16384, DarkReverbStorage Storage = DarkReverbStorage::Half>
class DarkReverb {
  static_assert((Capacity & (Capacity - 1)) == 0, "DarkReverb capacity must be a power of two.");
  static_assert(Capacity >= 4096 && Capacity <= (std::size_t{1} << 20),
                "DarkReverb capacity must be 4096..1048576 tank samples.");

  using Sample = typename std::conditional<Storage == DarkReverbStorage::Half, std::uint16_t, float>::type;

 public:
  // Delay buffer size in bytes.
  static constexpr std::size_t kBufferBytes = Capacity * sizeof(Sample);

  void prepare(float sampleRate) {
    sampleRate_ = safeSampleRate(sampleRate);
    tankRate_ = 0.5f * sampleRate_;
    updateDecay();
    updateDamping();
    updateLowCut();
    updateModulation();
    coeffs_.dcTrap = onePoleCoefficient(1.0f);
    coeffs_.dcTrapGain = 1.0f - 0.5f * coeffs_.dcTrap;
    // 20 ms fade for the input gate so freezing does not click.
    coeffs_.gateRate = onePoleCoefficient(1.0f / (kTwoPi * 0.02f));
    reset();
  }

  void reset() {
    buffer_.fill(Sample{});  // +0.0 in both formats
    state_ = State{};
    state_.gate = coeffs_.gateTarget;
    pending_ = 0.0f;
    heldLeft_ = 0.0f;
    heldRight_ = 0.0f;
    phase_ = 0;
  }

  // Low-frequency T60 in seconds, 0.1..1000. Higher frequencies also lose
  // energy in the per-stage lowpass, so they die sooner (see setDampingHz).
  void setDecaySeconds(float seconds) {
    decaySeconds_ = clamp(seconds, 0.1f, 1000.0f);
    updateDecay();
  }

  // Cutoff of the one-pole lowpass in each of the four stages, 100 Hz up to
  // 0.45 * tankRate. Every trip around the ring (~0.64 s by default) passes
  // four of them, so content at this frequency loses ~12 dB per trip and
  // content an octave below loses ~4 dB. Lower is darker.
  void setDampingHz(float hz) {
    dampingHz_ = hz;
    updateDamping();
  }

  // One-pole high-pass on the tank input, 10..1000 Hz. It keeps rumble out
  // of long decays without changing how long bass that got in rings.
  void setLowCutHz(float hz) {
    lowCutHz_ = hz;
    updateLowCut();
  }

  // 0..1: allpass feedback in the input diffusers and the ring. Low values
  // leave audible discrete echoes; high values give a smooth wash.
  void setDiffusion(float amount) {
    const float a = clamp01(amount);
    coeffs_.inDiffusionA = 0.45f + 0.30f * a;   // 0.45..0.75
    coeffs_.inDiffusionB = 0.40f + 0.25f * a;   // 0.40..0.65
    coeffs_.loopDiffusionA = 0.35f + 0.35f * a; // 0.35..0.70 (modulated)
    coeffs_.loopDiffusionB = 0.30f + 0.25f * a; // 0.30..0.55
  }

  // 0..1 of the +/-32 tank-sample modulation reach (+/-1.33 ms at 48 kHz).
  // More depth smears the ring's modes further (smoother, more chorused
  // tails); less keeps pitch steadier. Unmodulated, a 60 s tail keeps its
  // spectral peaks for seconds (37 dB above the median, metallic); the
  // defaults (0.5, 0.5 Hz) bring that to ~14 dB (noise is ~10 dB) while each
  // modulated allpass bends pitch by only +/-3.6 cents. Full depth at 5 Hz
  // reaches about +/-70 cents.
  void setModDepth(float amount) {
    modDepth_ = clamp01(amount);
    // Depth does not change either oscillator's rotation rate. Avoid two
    // sin calls when a control-rate depth target moves.
    coeffs_.modDepth = modDepth_ * static_cast<float>(dark_reverb_detail::kMaxModSamples);
  }

  // 0.01..5 Hz. The second LFO runs at 0.618x so the stages drift apart.
  void setModRateHz(float hz) {
    modRateHz_ = hz;
    updateModulation();
  }

  // Freeze holds the tail: unity loop gain, damping bypassed, input faded
  // out over 20 ms. Unfreezing restores decay, damping and input. At the
  // default modulation a frozen tail loses about 1.2 dB per hour (mostly
  // highs); fast, deep modulation drains it sooner (full depth at 1.5 Hz:
  // ~6 dB per hour, at 5 Hz: ~12 dB per hour).
  void setFreeze(bool frozen) {
    frozen_ = frozen;
    coeffs_.gateTarget = frozen ? 0.0f : 1.0f;
    updateDecay();
    updateDamping();
  }

  void setMix(float mix) { mix_ = clamp01(mix); }

  // 0 = mono wet, 1 = natural, 2 = exaggerated.
  void setWidth(float width) {
    width_ = clamp(width, 0.0f, 2.0f);
    updateOutput();
  }

  float decaySeconds() const { return decaySeconds_; }
  bool frozen() const { return frozen_; }

  // Per-sample API. The tank runs on every second call. Output is identical
  // to the block API, so the two can be mixed freely.
  std::array<float, 2> process(float left, float right) {
    const float mono = left + right;
    float wetLeft;
    float wetRight;
    if (phase_ == 0) {
      pending_ = mono;
      wetLeft = heldLeft_;
      wetRight = heldRight_;
      phase_ = 1;
    } else {
      float wet[4];
      tick(state_, coeffs_, pending_, mono, wet);
      wetLeft = wet[0];
      wetRight = wet[1];
      heldLeft_ = wet[2];
      heldRight_ = wet[3];
      phase_ = 0;
    }
    return {lerp(left, wetLeft, mix_), lerp(right, wetRight, mix_)};
  }

  // Block API: in-place is fine (out may alias in). Any frame count works;
  // pairs are processed with state held in locals, and an odd leftover
  // sample goes through the per-sample path.
  void process(const float* inLeft, const float* inRight, float* outLeft, float* outRight,
               std::size_t frames) {
    std::size_t i = 0;
    if (phase_ != 0 && frames > 0) {
      const auto out = process(inLeft[0], inRight[0]);
      outLeft[0] = out[0];
      outRight[0] = out[1];
      i = 1;
    }
    if (frames - i >= 2) {
      // Three passes per chunk (decimate, tank, interpolate) instead of one
      // fused loop: each pass keeps only its own state live, so less of it
      // spills out of the M33's 32 FPU registers (M33 -O2: 542 instead of
      // 569 instructions and 116 instead of 161 memory ops per tank sample).
      // State lives in locals so buffer stores cannot alias it. Same
      // arithmetic, so the output is bit-identical to the per-sample path.
      State state = state_;
      const Coeffs coeffs = coeffs_;
      const float mix = mix_;
      float heldLeft = heldLeft_;
      float heldRight = heldRight_;
      constexpr std::size_t kChunk = 16;  // tank samples per pass
      // Decimated input is dead after each tank call; reuse its storage for
      // the left wet output. Saves 64 bytes of scratch on small audio stacks.
      float wetLeft[kChunk];
      float wetRight[kChunk];
      while (frames - i >= 2) {
        const std::size_t pairs = (frames - i) / 2 < kChunk ? (frames - i) / 2 : kChunk;
        for (std::size_t j = 0; j < pairs; ++j) {
          const std::size_t k = i + 2 * j;
          wetLeft[j] = inputStage(state, coeffs, inLeft[k] + inRight[k], inLeft[k + 1] + inRight[k + 1]);
        }
        for (std::size_t j = 0; j < pairs; ++j) {
          tankStage(state, coeffs, wetLeft[j], wetLeft[j], wetRight[j]);
        }
        for (std::size_t j = 0; j < pairs; ++j) {
          const std::size_t k = i + 2 * j;
          float left0;
          float left1;
          float right0;
          float right1;
          upsample(wetLeft[j], state.upLeftIn, state.upLeftA1, state.upLeftA2, state.upLeftB1, left0, left1);
          upsample(wetRight[j], state.upRightIn, state.upRightA1, state.upRightA2, state.upRightB1, right0, right1);
          const float dryLeft0 = inLeft[k];
          const float dryRight0 = inRight[k];
          const float dryLeft1 = inLeft[k + 1];
          const float dryRight1 = inRight[k + 1];
          outLeft[k] = lerp(dryLeft0, heldLeft, mix);
          outRight[k] = lerp(dryRight0, heldRight, mix);
          outLeft[k + 1] = lerp(dryLeft1, left0, mix);
          outRight[k + 1] = lerp(dryRight1, right0, mix);
          heldLeft = left1;
          heldRight = right1;
        }
        i += 2 * pairs;
      }
      state_ = state;
      heldLeft_ = heldLeft;
      heldRight_ = heldRight;
    }
    if (i < frames) {
      const auto out = process(inLeft[i], inRight[i]);
      outLeft[i] = out[0];
      outRight[i] = out[1];
    }
  }

 private:
  using Line = dark_reverb_detail::Line;
  static constexpr dark_reverb_detail::Layout kLayout = dark_reverb_detail::makeLayout(Capacity);
  static_assert(kLayout.end <= Capacity, "DarkReverb layout overflows its buffer.");
  static constexpr std::uint32_t kMask = static_cast<std::uint32_t>(Capacity - 1);

  // Output taps inside the stage delays, as ages in tank samples.
  static constexpr std::uint32_t kTapLeftA = kLayout.length[dark_reverb_detail::kDel2] * 61u / 100u;
  static constexpr std::uint32_t kTapLeftB = kLayout.length[dark_reverb_detail::kDel1] * 37u / 100u;
  static constexpr std::uint32_t kTapRightA = kLayout.length[dark_reverb_detail::kDel0] * 53u / 100u;
  static constexpr std::uint32_t kTapRightB = kLayout.length[dark_reverb_detail::kDel3] * 29u / 100u;

  // Undo the L+R sum and the half-band paths' combined gain of 2, so the tank
  // sees the mono average at unity.
  static constexpr float kInputGain = 0.25f;
  static constexpr float kOutputGain = 1.0f;
  // Clamp on the tank input. The ring is passive, so this bounds every stored
  // value far below the half-precision limit of 65504. NaN input is flushed
  // to -limit instead of poisoning the tail.
  static constexpr float kInputLimit = 4.0f;
  // Safety bound on the recirculating signal (+24 dBFS). A 1000 s decay fed
  // two minutes of -6 dBFS noise peaks near 8; this only stops a runaway.
  static constexpr float kRingLimit = 16.0f;

  struct State {
    // 2:1 half-band decimator. Path A: two sections (three memories shared
    // between them), path B: one section.
    float decA0 = 0.0f, decA1 = 0.0f, decA2 = 0.0f, decB0 = 0.0f, decB1 = 0.0f;
    // 1:2 half-band interpolators: last input plus section outputs.
    float upLeftIn = 0.0f, upLeftA1 = 0.0f, upLeftA2 = 0.0f, upLeftB1 = 0.0f;
    float upRightIn = 0.0f, upRightA1 = 0.0f, upRightA2 = 0.0f, upRightB1 = 0.0f;
    float lowCut = 0.0f;
    float gate = 1.0f;
    float damp0 = 0.0f, damp1 = 0.0f, damp2 = 0.0f, damp3 = 0.0f;
    float interp0 = 0.0f, interp1 = 0.0f, interp2 = 0.0f, interp3 = 0.0f;
    float dcTrap = 0.0f;
    float lfoASin = 0.0f, lfoACos = 1.0f, lfoBSin = 0.70710678f, lfoBCos = 0.70710678f;
    std::uint32_t pointer = 0;
  };

  struct Coeffs {
    float gain0 = 0.0f, gain1 = 0.0f, gain2 = 0.0f, gain3 = 0.0f;
    float damping = 1.0f;
    float lowCut = 0.0f;
    float dcTrap = 0.0f, dcTrapGain = 1.0f;
    float gateTarget = 1.0f;
    float gateRate = 1.0f;
    float inDiffusionA = 0.69f, inDiffusionB = 0.60f;
    float loopDiffusionA = 0.63f, loopDiffusionB = 0.50f;
    float modDepth = 0.0f;
    float lfoA = 0.0f, lfoB = 0.0f;
    float mid = 0.5f * kOutputGain, side = 0.5f * kOutputGain;
  };

  static RPDSP_DARK_REVERB_INLINE float load(Sample value) {
    if constexpr (Storage == DarkReverbStorage::Half) {
      return dark_reverb_detail::halfToFloat(value);
    } else {
      return value;
    }
  }

  static RPDSP_DARK_REVERB_INLINE Sample store(float value) {
    if constexpr (Storage == DarkReverbStorage::Half) {
      return dark_reverb_detail::floatToHalf(value);
    } else {
      return value;
    }
  }

  // Sample written `age` tank samples ago on line L (age 0 = this sample).
  template <int L>
  RPDSP_DARK_REVERB_INLINE float tap(std::uint32_t pointer, std::uint32_t age) const {
    return load(buffer_[(pointer + kLayout.base[L] + age) & kMask]);
  }

  template <int L>
  RPDSP_DARK_REVERB_INLINE float tapEnd(std::uint32_t pointer) const {
    return tap<L>(pointer, kLayout.length[L]);
  }

  template <int L>
  RPDSP_DARK_REVERB_INLINE void put(std::uint32_t pointer, float value) {
    buffer_[(pointer + kLayout.base[L]) & kMask] = store(value);
  }

  // Schroeder allpass: v = x + g * v[n-D], y = v[n-D] - g * v.
  template <int L>
  RPDSP_DARK_REVERB_INLINE float allpass(std::uint32_t pointer, float x, float g) {
    const float delayed = tapEnd<L>(pointer);
    const float v = x + g * delayed;
    put<L>(pointer, v);
    return delayed - g * v;
  }

  // Allpass whose delay swings by `offset` tank samples (|offset| <= 32).
  // The swing moves the ring's modes so long tails do not ring.
  //
  // The fractional read is a first-order allpass interpolator: unity gain at
  // every frequency, so modulation adds no loss to the loop. Linear
  // interpolation loses 0.4 dB at 3 kHz per pass, which shortens long decays
  // and drains freeze. Against an ideal modulated delay its error is -60 dB
  // at 1 kHz (linear: -44 dB).
  template <int L>
  RPDSP_DARK_REVERB_INLINE float modulatedAllpass(std::uint32_t pointer, float x, float offset,
                                                  float g, float& interp) {
    // Integer tap `whole` plus a fraction d in [0.5, 1.5); keeping d away
    // from 0 keeps the interpolator pole (at -a, |a| <= 1/3) well damped.
    // Age stays >= length - 33 > 0, so truncation is floor.
    const float age = static_cast<float>(kLayout.length[L]) + offset - 0.5f;
    const auto whole = static_cast<std::uint32_t>(age);
    const float t = age - static_cast<float>(whole) - 0.5f;  // d - 1
    // a = (1 - d) / (1 + d) = -t / (2 + t) as a cubic: no divide, and the
    // delay it realises stays within 0.01 samples of d.
    const float a = t * (-0.5f + t * (0.25f - 0.125f * t));
    const std::uint32_t index = pointer + kLayout.base[L] + whole;
    const float near = load(buffer_[index & kMask]);
    const float far = load(buffer_[(index + 1u) & kMask]);
    const float delayed = a * (near - interp) + far;
    interp = zapDenormal(delayed);
    const float v = x + g * delayed;
    put<L>(pointer, v);
    return delayed - g * v;
  }

  // 1:2 half-band interpolation of one tank sample into two host samples.
  static RPDSP_DARK_REVERB_INLINE void upsample(float x, float& in, float& a1, float& a2, float& b1,
                                                float& first, float& second) {
    using namespace dark_reverb_detail;
    const float t = kHalfband0 * (x - a1) + in;
    const float even = kHalfband2 * (t - a2) + a1;
    const float odd = kHalfband1 * (x - b1) + in;
    in = x;
    a1 = t;
    a2 = even;
    b1 = odd;
    first = even;
    second = odd;
  }

  // 2:1 decimation and input conditioning: two host-rate L+R sums (x0
  // earlier) in, one tank-rate sample out.
  static RPDSP_DARK_REVERB_INLINE float inputStage(State& s, const Coeffs& c, float x0, float x1) {
    using namespace dark_reverb_detail;
    const float a0 = kHalfband0 * (x1 - s.decA1) + s.decA0;
    const float a1 = kHalfband2 * (a0 - s.decA2) + s.decA1;
    const float b0 = kHalfband1 * (x0 - s.decB1) + s.decB0;
    s.decA0 = x1;
    s.decA1 = a0;
    s.decA2 = a1;
    s.decB0 = x0;
    s.decB1 = b0;
    float x = clamp((a1 + b0) * kInputGain, -kInputLimit, kInputLimit);
    // Low cut, then the freeze gate.
    s.lowCut = zapDenormal(s.lowCut + c.lowCut * (x - s.lowCut));
    x -= s.lowCut;
    s.gate = zapDenormal(s.gate + c.gateRate * (c.gateTarget - s.gate));
    return x * s.gate;
  }

  // One tank sample: diffusers, ring and taps. Writes the tank-rate wet pair.
  RPDSP_DARK_REVERB_INLINE void tankStage(State& s, const Coeffs& c, float x, float& wetLeft, float& wetRight) {
    using namespace dark_reverb_detail;
    const std::uint32_t p = s.pointer;
    x = allpass<kIn0>(p, x, c.inDiffusionA);
    x = allpass<kIn1>(p, x, c.inDiffusionA);
    x = allpass<kIn2>(p, x, c.inDiffusionB);
    x = allpass<kIn3>(p, x, c.inDiffusionB);

    // Magic-circle quadrature LFOs (Minsky rotation): amplitude stays
    // bounded without renormalisation.
    s.lfoASin += c.lfoA * s.lfoACos;
    s.lfoACos -= c.lfoA * s.lfoASin;
    s.lfoBSin += c.lfoB * s.lfoBCos;
    s.lfoBCos -= c.lfoB * s.lfoBSin;

    // Stage 0 (input enters here and at stage 2). Every trip around the ring
    // passes this point.
    float v = c.gain0 * tapEnd<kDel3>(p) + x;
    // 1 Hz subsonic trap. Delay modulation parametrically pumps pairs of
    // ring modes whose frequencies sum to the LFO rate, i.e. modes below
    // 5 Hz; with nothing damping them a frozen tail grows without bound (at
    // 5 Hz / full depth, +17 dB/min, all of it below 10 Hz). Normalised
    // one-pole high-pass: unity gain at Nyquist and |H| <= 1 everywhere, so
    // it costs a frozen 100 Hz tone only 0.04 dB/min.
    const float dc = v - s.dcTrap;
    s.dcTrap = zapDenormal(s.dcTrap + c.dcTrap * dc);
    // The clamp bounds the whole tank; normal use never reaches it.
    v = clamp(dc * c.dcTrapGain, -kRingLimit, kRingLimit);
    v = modulatedAllpass<kModAp0>(p, v, c.modDepth * s.lfoASin, c.loopDiffusionA, s.interp0);
    v = allpass<kAp0>(p, v, -c.loopDiffusionB);
    s.damp0 = zapDenormal(s.damp0 + c.damping * (v - s.damp0));
    put<kDel0>(p, s.damp0);

    // Stage 1.
    v = c.gain1 * tapEnd<kDel0>(p);
    v = modulatedAllpass<kModAp1>(p, v, c.modDepth * s.lfoBSin, c.loopDiffusionA, s.interp1);
    v = allpass<kAp1>(p, v, -c.loopDiffusionB);
    s.damp1 = zapDenormal(s.damp1 + c.damping * (v - s.damp1));
    put<kDel1>(p, s.damp1);

    // Stage 2.
    v = c.gain2 * tapEnd<kDel1>(p) + x;
    v = modulatedAllpass<kModAp2>(p, v, c.modDepth * s.lfoACos, c.loopDiffusionA, s.interp2);
    v = allpass<kAp2>(p, v, -c.loopDiffusionB);
    s.damp2 = zapDenormal(s.damp2 + c.damping * (v - s.damp2));
    put<kDel2>(p, s.damp2);

    // Stage 3.
    v = c.gain3 * tapEnd<kDel2>(p);
    v = modulatedAllpass<kModAp3>(p, v, c.modDepth * s.lfoBCos, c.loopDiffusionA, s.interp3);
    v = allpass<kAp3>(p, v, -c.loopDiffusionB);
    s.damp3 = zapDenormal(s.damp3 + c.damping * (v - s.damp3));
    put<kDel3>(p, s.damp3);

    // Stereo taps: each side hears its own injection stage directly plus two
    // taps from other stages.
    const float left = s.damp0 + tap<kDel2>(p, kTapLeftA) - tap<kDel1>(p, kTapLeftB);
    const float right = s.damp2 + tap<kDel0>(p, kTapRightA) - tap<kDel3>(p, kTapRightB);
    const float mid = (left + right) * c.mid;
    const float side = (left - right) * c.side;
    wetLeft = mid + side;
    wetRight = mid - side;
    s.pointer = (p - 1u) & kMask;
  }

  // One tank sample from two host-rate inputs; wet: {left0, right0, left1, right1}.
  RPDSP_DARK_REVERB_INLINE void tick(State& s, const Coeffs& c, float x0, float x1, float* wet) {
    float left;
    float right;
    tankStage(s, c, inputStage(s, c, x0, x1), left, right);
    upsample(left, s.upLeftIn, s.upLeftA1, s.upLeftA2, s.upLeftB1, wet[0], wet[2]);
    upsample(right, s.upRightIn, s.upRightA1, s.upRightA2, s.upRightB1, wet[1], wet[3]);
  }

  // One-pole smoothing coefficient for a cutoff at the tank rate.
  float onePoleCoefficient(float hz) const {
    return 1.0f - std::exp(-kTwoPi * hz / tankRate_);
  }

  void updateDecay() {
    using namespace dark_reverb_detail;
    // Per-stage gain from the stage's mean transit time (an allpass delays
    // by its length on average), so the whole ring meets the requested T60.
    const auto gainFor = [this](std::size_t stage) {
      if (frozen_) {
        return 1.0f;
      }
      const float samples = static_cast<float>(kLayout.length[kModAp0 + stage] +
                                                kLayout.length[kAp0 + stage] +
                                                kLayout.length[kDel0 + stage]);
      // 60 dB = ln(1000) nepers of amplitude.
      return std::exp(-6.90775528f * samples / (tankRate_ * decaySeconds_));
    };
    coeffs_.gain0 = gainFor(0);
    coeffs_.gain1 = gainFor(1);
    coeffs_.gain2 = gainFor(2);
    coeffs_.gain3 = gainFor(3);
  }

  void updateDamping() {
    const float hz = clamp(dampingHz_, 100.0f, 0.45f * tankRate_);
    coeffs_.damping = frozen_ ? 1.0f : onePoleCoefficient(hz);
  }

  void updateLowCut() {
    coeffs_.lowCut = onePoleCoefficient(clamp(lowCutHz_, 10.0f, 1000.0f));
  }

  void updateModulation() {
    using dark_reverb_detail::kMaxModSamples;
    coeffs_.modDepth = modDepth_ * static_cast<float>(kMaxModSamples);
    const float hz = clamp(modRateHz_, 0.01f, 5.0f);
    // Minsky rotation step for an exact frequency: 2 sin(pi f / fs).
    coeffs_.lfoA = 2.0f * std::sin(kPi * hz / tankRate_);
    coeffs_.lfoB = 2.0f * std::sin(kPi * hz * 0.618034f / tankRate_);
  }

  void updateOutput() {
    coeffs_.mid = 0.5f * kOutputGain;
    coeffs_.side = 0.5f * kOutputGain * width_;
  }

  float sampleRate_ = kDefaultSampleRate;
  float tankRate_ = 0.5f * kDefaultSampleRate;
  float decaySeconds_ = 20.0f;
  float dampingHz_ = 3000.0f;
  float lowCutHz_ = 40.0f;
  float modDepth_ = 0.5f;
  float modRateHz_ = 0.5f;
  float width_ = 1.0f;
  float mix_ = 0.35f;
  bool frozen_ = false;

  float pending_ = 0.0f;
  float heldLeft_ = 0.0f;
  float heldRight_ = 0.0f;
  int phase_ = 0;

  State state_{};
  Coeffs coeffs_{};
  std::array<Sample, Capacity> buffer_{};
};

}  // namespace rpdsp

#undef RPDSP_DARK_REVERB_INLINE
#undef RPDSP_DARK_REVERB_VCVTB
