#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#if defined(__has_include)
#if __has_include(<hardware/interp.h>)
#define RPDSP_HAS_HARDWARE_INTERP 1
#endif
#endif

#ifndef RPDSP_HAS_HARDWARE_INTERP
#define RPDSP_HAS_HARDWARE_INTERP 0
#endif

#if RPDSP_HAS_HARDWARE_INTERP
#include <hardware/interp.h>
#else
// Dummy structures and functions to compile on host
struct interp_hw_t {
  std::uint32_t accum[2];
  std::uint32_t base[3];
  std::uint32_t peek[3];
  std::uint32_t pop[3];
  std::uint32_t add_raw[2];
  std::uint32_t base01;
};

inline interp_hw_t dummyInterp0;
inline interp_hw_t dummyInterp1;

#ifndef interp0
#define interp0 (&dummyInterp0)
#endif
#ifndef interp1
#define interp1 (&dummyInterp1)
#endif

struct interp_config {
  std::uint32_t shift;
  std::uint32_t mask_lsb;
  std::uint32_t mask_msb;
  bool is_signed;
  bool cross_input;
  bool cross_result;
  bool blend;
  bool clamp;
  bool add_raw;
};

inline interp_config interp_default_config() {
  return interp_config{0, 0, 0, false, false, false, false, false, false};
}
inline void interp_config_set_shift(interp_config* c, std::uint32_t shift) { c->shift = shift; }
inline void interp_config_set_mask(interp_config* c, std::uint32_t lsb, std::uint32_t msb) {
  c->mask_lsb = lsb;
  c->mask_msb = msb;
}
inline void interp_config_set_signed(interp_config* c, bool is_signed) { c->is_signed = is_signed; }
inline void interp_config_set_cross_input(interp_config* c, bool cross_input) { c->cross_input = cross_input; }
inline void interp_config_set_cross_result(interp_config* c, bool cross_result) { c->cross_result = cross_result; }
inline void interp_config_set_blend(interp_config* c, bool blend) { c->blend = blend; }
inline void interp_config_set_clamp(interp_config* c, bool clamp) { c->clamp = clamp; }
inline void interp_config_set_add_raw(interp_config* c, bool add_raw) { c->add_raw = add_raw; }
inline void interp_set_config(interp_hw_t*, std::uint32_t, const interp_config*) {}
#endif

namespace rpdsp {

class HardwareInterpolatorPool {
 public:
  enum class Resource {
    Core0Interp0 = 0,
    Core0Interp1 = 1,
    Core1Interp0 = 2,
    Core1Interp1 = 3,
  };

  static bool claim(Resource resource) {
    std::uint32_t bit = 1u << static_cast<std::uint32_t>(resource);
    if (claimedMask_ & bit) {
      return false;
    }
    claimedMask_ |= bit;
    return true;
  }

  static void release(Resource resource) {
    std::uint32_t bit = 1u << static_cast<std::uint32_t>(resource);
    claimedMask_ &= ~bit;
  }

  static interp_hw_t* getHw(Resource resource) {
    switch (resource) {
      case Resource::Core0Interp0:
      case Resource::Core1Interp0:
        return interp0;
      case Resource::Core0Interp1:
      case Resource::Core1Interp1:
        return interp1;
    }
    return nullptr;
  }

 private:
  inline static std::uint32_t claimedMask_ = 0;
};

class HardwareWavefolder {
 public:
  HardwareWavefolder() = default;
  ~HardwareWavefolder() { deinit(); }

  HardwareWavefolder(const HardwareWavefolder&) = delete;
  HardwareWavefolder& operator=(const HardwareWavefolder&) = delete;

  int init(HardwareInterpolatorPool::Resource resource, std::uint8_t foldOrder) {
    if (foldOrder > 30) {
      return -1;
    }

    hw_ = HardwareInterpolatorPool::getHw(resource);
    if (!hw_) {
      return -2;
    }

    if (!HardwareInterpolatorPool::claim(resource)) {
      return -3;
    }

    resource_ = resource;
    foldOrder_ = foldOrder;
    numStages_ = 1;
    preGainQ16_ = 65536;

#if RPDSP_HAS_HARDWARE_INTERP
    lane0Cfg_ = interp_default_config();
    interp_config_set_shift(&lane0Cfg_, 0);
    interp_config_set_mask(&lane0Cfg_, 0, foldOrder);
    interp_config_set_signed(&lane0Cfg_, true);
    interp_set_config(hw_, 0, &lane0Cfg_);

    hw_->base[0] = 0;
#endif

    initialized_ = true;
    return 0;
  }

  void deinit() {
    if (initialized_) {
      HardwareInterpolatorPool::release(resource_);
      initialized_ = false;
      hw_ = nullptr;
    }
  }

  void setGain(float preGain) {
    preGainQ16_ = static_cast<std::int32_t>(preGain * 65536.0f);
  }

  void setStages(std::uint8_t numStages) {
    if (numStages < 1 || numStages > 8) {
      return;
    }
    numStages_ = numStages;
  }

  std::int32_t process(std::int32_t sample) const {
    std::int64_t gained = (static_cast<std::int64_t>(sample) * preGainQ16_) >> 16;
    std::int32_t val;
    if (gained > 2147483647LL) {
      val = 2147483647;
    } else if (gained < -2147483648LL) {
      val = -2147483648;
    } else {
      val = static_cast<std::int32_t>(gained);
    }

#if RPDSP_HAS_HARDWARE_INTERP
    for (std::uint8_t s = 0; s < numStages_; s++) {
      std::uint32_t abs_val = static_cast<std::uint32_t>(val < 0 ? -static_cast<std::int64_t>(val) : static_cast<std::int64_t>(val));
      hw_->accum[0] = abs_val;
      // peek[0] is the sign-extended masked value w in [-L, L) where
      // L = 1 << foldOrder. The triangle fold of |val| is |w|; without this
      // abs the hardware path produces a wrap (sawtooth), not a fold, and
      // diverges from the host model below.
      std::int32_t w = static_cast<std::int32_t>(hw_->peek[0]);
      std::int32_t tri = w < 0 ? -w : w;
      val = val < 0 ? -tri : tri;
    }
    return val;
#else
    // Software folding simulation on host
    const std::uint32_t foldLimit = 1u << foldOrder_;
    for (std::uint8_t s = 0; s < numStages_; s++) {
      std::uint32_t absVal = static_cast<std::uint32_t>(val < 0 ? -static_cast<std::int64_t>(val) : val);
      std::uint32_t doubleLimit = 2u * foldLimit;
      std::uint32_t mod = absVal % doubleLimit;
      std::int32_t tri = static_cast<std::int32_t>(foldLimit - (mod > foldLimit ? mod - foldLimit : foldLimit - mod));
      val = val < 0 ? -tri : tri;
    }
    return val;
#endif
  }

  void processBlock(const std::int32_t* src, std::int32_t* dst, size_t count) const {
    for (size_t i = 0; i < count; i++) {
      dst[i] = process(src[i]);
    }
  }

 private:
  interp_hw_t* hw_ = nullptr;
  HardwareInterpolatorPool::Resource resource_ = HardwareInterpolatorPool::Resource::Core0Interp0;
  interp_config lane0Cfg_;
  std::uint8_t foldOrder_ = 0;
  std::uint8_t numStages_ = 1;
  std::int32_t preGainQ16_ = 65536;
  bool initialized_ = false;
};

// Single-cycle linear crossfade using INTERP0 blend mode.
//
// Computes  out = a + ((b - a) * alpha) >> 8  for an 8-bit blend weight alpha
// (0 => all a, 255 => almost all b). On the RP2xxx this is one interpolator
// read with no branches or multiply on the CPU — the hardware datapath does the
// subtract, multiply and shift. This is the primitive behind the Cornell
// "audio cross-fade via blend mode" demo, where two signed audio sources are
// mixed by sweeping alpha.
class HardwareBlend {
 public:
  HardwareBlend() = default;
  ~HardwareBlend() { deinit(); }

  HardwareBlend(const HardwareBlend&) = delete;
  HardwareBlend& operator=(const HardwareBlend&) = delete;

  // Blend mode is only available on an interp0 lane.
  int init(HardwareInterpolatorPool::Resource resource) {
    if (resource != HardwareInterpolatorPool::Resource::Core0Interp0 &&
        resource != HardwareInterpolatorPool::Resource::Core1Interp0) {
      return -4; // Blend is only supported on interp0
    }

    hw_ = HardwareInterpolatorPool::getHw(resource);
    if (!hw_) {
      return -2;
    }
    if (!HardwareInterpolatorPool::claim(resource)) {
      return -3;
    }
    resource_ = resource;

#if RPDSP_HAS_HARDWARE_INTERP
    // Lane 0 carries the blend; lane 1 supplies the signed alpha weight.
    lane0Cfg_ = interp_default_config();
    interp_config_set_blend(&lane0Cfg_, true);
    interp_set_config(hw_, 0, &lane0Cfg_);

    lane1Cfg_ = interp_default_config();
    interp_config_set_signed(&lane1Cfg_, true);
    interp_set_config(hw_, 1, &lane1Cfg_);

    hw_->accum[1] = 0;
    hw_->base[0] = 0;
    hw_->base[1] = 0;
#endif

    initialized_ = true;
    return 0;
  }

  void deinit() {
    if (initialized_) {
      HardwareInterpolatorPool::release(resource_);
      initialized_ = false;
      hw_ = nullptr;
    }
  }

  // alphaQ8 in [0, 255]: 0 selects a, 255 selects (almost all) b.
  void setAlphaQ8(std::uint8_t alphaQ8) {
    alphaQ8_ = alphaQ8;
#if RPDSP_HAS_HARDWARE_INTERP
    hw_->accum[1] = alphaQ8;
#endif
  }

  // Crossfade with the alpha set by the most recent setAlphaQ8().
  std::int32_t process(std::int32_t a, std::int32_t b) const {
#if RPDSP_HAS_HARDWARE_INTERP
    hw_->base[0] = static_cast<std::uint32_t>(a);
    hw_->base[1] = static_cast<std::uint32_t>(b);
    return static_cast<std::int32_t>(hw_->peek[1]);
#else
    return a + (((b - a) * static_cast<std::int32_t>(alphaQ8_)) >> 8);
#endif
  }

  // Convenience: set alpha and crossfade in one call.
  std::int32_t blend(std::int32_t a, std::int32_t b, std::uint8_t alphaQ8) {
    setAlphaQ8(alphaQ8);
    return process(a, b);
  }

 private:
  interp_hw_t* hw_ = nullptr;
  HardwareInterpolatorPool::Resource resource_ = HardwareInterpolatorPool::Resource::Core0Interp0;
  std::uint8_t alphaQ8_ = 0;
  bool initialized_ = false;
#if RPDSP_HAS_HARDWARE_INTERP
  interp_config lane0Cfg_;
  interp_config lane1Cfg_;
#endif
};

// Single-cycle saturating clamp using INTERP1 clamp mode.
//
// Computes  out = min(max(x, lo), hi)  with no branches on the CPU. Clamp mode
// is only available on interp1. This is the primitive behind the Cornell
// "tone burst" demo, where an integrating envelope accumulator is clamped
// between zero and a peak amplitude before it scales the oscillator.
class HardwareClamp {
 public:
  HardwareClamp() = default;
  ~HardwareClamp() { deinit(); }

  HardwareClamp(const HardwareClamp&) = delete;
  HardwareClamp& operator=(const HardwareClamp&) = delete;

  int init(HardwareInterpolatorPool::Resource resource, std::int32_t lo, std::int32_t hi) {
    if (resource != HardwareInterpolatorPool::Resource::Core0Interp1 &&
        resource != HardwareInterpolatorPool::Resource::Core1Interp1) {
      return -4; // Clamp is only supported on interp1
    }

    hw_ = HardwareInterpolatorPool::getHw(resource);
    if (!hw_) {
      return -2;
    }
    if (!HardwareInterpolatorPool::claim(resource)) {
      return -3;
    }
    resource_ = resource;
    lo_ = lo;
    hi_ = hi;

#if RPDSP_HAS_HARDWARE_INTERP
    lane0Cfg_ = interp_default_config();
    interp_config_set_clamp(&lane0Cfg_, true);
    interp_config_set_signed(&lane0Cfg_, true);
    interp_set_config(hw_, 0, &lane0Cfg_);

    lane1Cfg_ = interp_default_config();
    interp_set_config(hw_, 1, &lane1Cfg_);

    hw_->base[0] = static_cast<std::uint32_t>(lo);
    hw_->base[1] = static_cast<std::uint32_t>(hi);
#endif

    initialized_ = true;
    return 0;
  }

  void deinit() {
    if (initialized_) {
      HardwareInterpolatorPool::release(resource_);
      initialized_ = false;
      hw_ = nullptr;
    }
  }

  void setRange(std::int32_t lo, std::int32_t hi) {
    lo_ = lo;
    hi_ = hi;
#if RPDSP_HAS_HARDWARE_INTERP
    hw_->base[0] = static_cast<std::uint32_t>(lo);
    hw_->base[1] = static_cast<std::uint32_t>(hi);
#endif
  }

  std::int32_t process(std::int32_t x) const {
#if RPDSP_HAS_HARDWARE_INTERP
    hw_->accum[0] = static_cast<std::uint32_t>(x);
    return static_cast<std::int32_t>(hw_->peek[0]);
#else
    return x < lo_ ? lo_ : (x > hi_ ? hi_ : x);
#endif
  }

 private:
  interp_hw_t* hw_ = nullptr;
  HardwareInterpolatorPool::Resource resource_ = HardwareInterpolatorPool::Resource::Core0Interp1;
  std::int32_t lo_ = 0;
  std::int32_t hi_ = 0;
  bool initialized_ = false;
#if RPDSP_HAS_HARDWARE_INTERP
  interp_config lane0Cfg_;
  interp_config lane1Cfg_;
#endif
};

class HardwareOscillator {
 public:
  HardwareOscillator() = default;

  HardwareOscillator(const HardwareOscillator&) = delete;
  HardwareOscillator& operator=(const HardwareOscillator&) = delete;

  static bool isPowerOfTwo(std::uint32_t value) {
    return value != 0u && (value & (value - 1u)) == 0u;
  }

  static std::uint8_t log2U32(std::uint32_t value) {
    std::uint8_t bits = 0u;
    while (value > 1u) {
      value >>= 1u;
      ++bits;
    }
    return bits;
  }

  static std::uint8_t tableBitCount(std::uint32_t tableLength) {
    if (!isPowerOfTwo(tableLength) || tableLength < 2u) {
      return 0u;
    }
    return log2U32(tableLength);
  }

  static int tuningWordFromFrequency(std::uint32_t tableLength,
                                     std::uint8_t fractionalBits,
                                     std::uint32_t sampleRateHz,
                                     std::uint32_t frequencyHz,
                                     std::uint32_t& phaseIncrementQnOut) {
    if (sampleRateHz == 0u) {
      return -1; // Invalid argument
    }
    if (tableLength < 2u || !isPowerOfTwo(tableLength)) {
      return -2; // Invalid table
    }
    std::uint8_t tableBits = tableBitCount(tableLength);
    if (fractionalBits < 8u || (tableBits + fractionalBits > 32u)) {
      return -3; // Invalid configuration
    }

    std::uint64_t numerator = static_cast<std::uint64_t>(frequencyHz) * static_cast<std::uint64_t>(tableLength);
    numerator <<= fractionalBits;
    phaseIncrementQnOut = static_cast<std::uint32_t>(numerator / static_cast<std::uint64_t>(sampleRateHz));
    return 0;
  }

  int init(interp_hw_t* interp,
           const std::int16_t* table,
           std::uint32_t tableLength,
           std::uint8_t fractionalBits,
           std::uint32_t initialPhaseQn,
           std::uint32_t phaseIncrementQn) {
    if (interp == nullptr || table == nullptr) {
      return -1; // Invalid argument
    }
    if (tableLength < 2u || !isPowerOfTwo(tableLength)) {
      return -2; // Invalid table
    }
    std::uint8_t tableBits = tableBitCount(tableLength);
    if (fractionalBits < 8u || (tableBits + fractionalBits > 32u)) {
      return -3; // Invalid configuration
    }

    interp_ = interp;
    table_ = table;
    tableLength_ = tableLength;
    tableMask_ = tableLength - 1u;
    tableBits_ = tableBits;
    fractionalBits_ = fractionalBits;
    phaseIncrementQn_ = phaseIncrementQn;

#if RPDSP_HAS_HARDWARE_INTERP
    if (interp_ != interp0) {
      return -4; // Blend is only supported on interp0
    }

    lane0Cfg_ = interp_default_config();
    interp_config_set_shift(&lane0Cfg_, fractionalBits_);
    interp_config_set_mask(&lane0Cfg_, 0u, tableBits_ - 1u);
    interp_config_set_blend(&lane0Cfg_, true);
    interp_set_config(interp_, 0, &lane0Cfg_);

    lane1Cfg_ = interp_default_config();
    interp_config_set_shift(&lane1Cfg_, fractionalBits_ - 8u);
    interp_config_set_cross_input(&lane1Cfg_, true);
    interp_config_set_signed(&lane1Cfg_, true);
    interp_set_config(interp_, 1, &lane1Cfg_);

    interp_->base[0] = 0u;
    interp_->base[1] = 0u;
    interp_->base[2] = 0u;
#endif

    setPhase(initialPhaseQn);
    return 0;
  }

  int setTable(const std::int16_t* table, std::uint32_t tableLength) {
    if (table == nullptr) return -1;
    if (tableLength < 2u || !isPowerOfTwo(tableLength)) return -2;

    table_ = table;
    tableLength_ = tableLength;
    tableMask_ = tableLength - 1u;
    tableBits_ = tableBitCount(tableLength);

#if RPDSP_HAS_HARDWARE_INTERP
    if (interp_ == interp0) {
      lane0Cfg_ = interp_default_config();
      interp_config_set_shift(&lane0Cfg_, fractionalBits_);
      interp_config_set_mask(&lane0Cfg_, 0u, tableBits_ - 1u);
      interp_config_set_blend(&lane0Cfg_, true);
      interp_set_config(interp_, 0, &lane0Cfg_);
    }
#endif

    return loadEndpoints();
  }

  int setPhase(std::uint32_t phaseQn) {
#if RPDSP_HAS_HARDWARE_INTERP
    interp_->accum[0] = phaseQn;
#else
    phase_ = phaseQn;
#endif
    return loadEndpoints();
  }

  int setPhaseIncrement(std::uint32_t phaseIncrementQn) {
    phaseIncrementQn_ = phaseIncrementQn;
    return 0;
  }

  std::uint32_t getPhase() const {
#if RPDSP_HAS_HARDWARE_INTERP
    return interp_->accum[0];
#else
    return phase_;
#endif
  }

  std::uint32_t getTableIndex() const {
#if RPDSP_HAS_HARDWARE_INTERP
    // add_raw[0] is the raw phase accumulator; the integer table index lives in
    // the bits above the fractional part, so shift the fraction out first.
    return (interp_->add_raw[0] >> fractionalBits_) & tableMask_;
#else
    return (phase_ >> fractionalBits_) & tableMask_;
#endif
  }

  std::uint8_t getFractionQ8() const {
#if RPDSP_HAS_HARDWARE_INTERP
    // Top 8 bits of the fractional part = the Q8 blend weight (alpha).
    return static_cast<std::uint8_t>((interp_->add_raw[0] >> (fractionalBits_ - 8)) & 0xffu);
#else
    std::uint32_t fractionMask = (1u << fractionalBits_) - 1;
    return static_cast<std::uint8_t>((phase_ & fractionMask) >> (fractionalBits_ - 8));
#endif
  }

  int peekSample(std::int32_t& sampleOut) {
    int status = loadEndpoints();
    if (status != 0) return status;

#if RPDSP_HAS_HARDWARE_INTERP
    sampleOut = static_cast<std::int32_t>(interp_->peek[1]);
#else
    std::uint32_t index = (phase_ >> fractionalBits_) & tableMask_;
    std::uint32_t nextIndex = (index + 1u) & tableMask_;
    std::int16_t sampleA = table_[index];
    std::int16_t sampleB = table_[nextIndex];
    std::uint32_t fractionMask = (1u << fractionalBits_) - 1;
    std::uint32_t fraction = phase_ & fractionMask;
    std::uint32_t alpha = fraction >> (fractionalBits_ - 8);
    sampleOut = sampleA + static_cast<std::int32_t>(alpha) * (sampleB - sampleA) / 256;
#endif
    return 0;
  }

  int nextSample(std::int32_t& sampleOut) {
    int status = peekSample(sampleOut);
    if (status != 0) return status;

#if RPDSP_HAS_HARDWARE_INTERP
    interp_->add_raw[0] = phaseIncrementQn_;
#else
    phase_ += phaseIncrementQn_;
#endif
    return 0;
  }

 private:
  int loadEndpoints() {
    if (table_ == nullptr) return -1;

#if RPDSP_HAS_HARDWARE_INTERP
    std::uint32_t index = (interp_->add_raw[0] >> fractionalBits_) & tableMask_;
    std::int16_t sample_a = table_[index];
    std::int16_t sample_b = table_[(index + 1u) & tableMask_];
    interp_->base01 = (static_cast<std::uint32_t>(static_cast<std::uint16_t>(sample_a))) |
                      (static_cast<std::uint32_t>(static_cast<std::uint16_t>(sample_b)) << 16);
#endif
    return 0;
  }

  interp_hw_t* interp_ = nullptr;
  const std::int16_t* table_ = nullptr;
  std::uint32_t tableLength_ = 0;
  std::uint32_t tableMask_ = 0;
  std::uint32_t phaseIncrementQn_ = 0;
  std::uint8_t tableBits_ = 0;
  std::uint8_t fractionalBits_ = 0;

#if RPDSP_HAS_HARDWARE_INTERP
  interp_config lane0Cfg_;
  interp_config lane1Cfg_;
#else
  std::uint32_t phase_ = 0;
#endif
};

// Morphing wavetable oscillator that time-shares the interp0 blend datapath.
//
// Unlike HardwareOscillator (which keeps its phase in the hardware
// accumulator), this class keeps the phase in software so that:
//   * several instances can share the single interp0 per core, and
//   * the blend datapath is free to be reused between samples for other
//     single-cycle blends (see blendQ8 / ringModQ8 / scaleQ8 below).
//
// Per sample it performs THREE hardware blends on the same interp0:
//   1. linear interpolation within table A at the current phase,
//   2. linear interpolation within table B at the same phase,
//   3. a blend between the two results weighted by the morph amount.
// Lane configs are re-asserted each call (two single-cycle stores), so
// instances with different table sizes / fractional bits coexist safely.
class HardwareMorphOscillator {
 public:
  HardwareMorphOscillator() = default;

  HardwareMorphOscillator(const HardwareMorphOscillator&) = delete;
  HardwareMorphOscillator& operator=(const HardwareMorphOscillator&) = delete;

  int init(interp_hw_t* interp,
           const std::int16_t* tableA,
           const std::int16_t* tableB,
           std::uint32_t tableLength,
           std::uint8_t fractionalBits,
           std::uint32_t initialPhaseQn,
           std::uint32_t phaseIncrementQn) {
    if (interp == nullptr || tableA == nullptr || tableB == nullptr) {
      return -1; // Invalid argument
    }
    if (tableLength < 2u || !HardwareOscillator::isPowerOfTwo(tableLength)) {
      return -2; // Invalid table
    }
    std::uint8_t tableBits = HardwareOscillator::tableBitCount(tableLength);
    if (fractionalBits < 8u || (tableBits + fractionalBits > 32u)) {
      return -3; // Invalid configuration
    }

#if RPDSP_HAS_HARDWARE_INTERP
    if (interp != interp0) {
      return -4; // Blend is only supported on interp0
    }
#endif

    interp_ = interp;
    tableA_ = tableA;
    tableB_ = tableB;
    tableLength_ = tableLength;
    tableMask_ = tableLength - 1u;
    tableBits_ = tableBits;
    fractionalBits_ = fractionalBits;
    phase_ = initialPhaseQn;
    phaseIncrementQn_ = phaseIncrementQn;

#if RPDSP_HAS_HARDWARE_INTERP
    lane0Cfg_ = interp_default_config();
    interp_config_set_shift(&lane0Cfg_, fractionalBits_);
    interp_config_set_mask(&lane0Cfg_, 0u, tableBits_ - 1u);
    interp_config_set_blend(&lane0Cfg_, true);

    lane1Cfg_ = interp_default_config();
    interp_config_set_shift(&lane1Cfg_, fractionalBits_ - 8u);
    interp_config_set_cross_input(&lane1Cfg_, true);
    interp_config_set_signed(&lane1Cfg_, true);
#endif

    return 0;
  }

  int setTables(const std::int16_t* tableA, const std::int16_t* tableB) {
    if (tableA == nullptr || tableB == nullptr) return -1;
    tableA_ = tableA;
    tableB_ = tableB;
    return 0;
  }

  // morphQ8: 0 => pure table A, 255 => (almost) pure table B.
  void setMorphQ8(std::uint8_t morphQ8) { morphQ8_ = morphQ8; }

  // morph in [0, 1] (values outside are clamped).
  void setMorph(float morph) {
    if (morph < 0.0f) morph = 0.0f;
    if (morph > 1.0f) morph = 1.0f;
    morphQ8_ = static_cast<std::uint8_t>(morph * 255.0f + 0.5f);
  }

  void setPhase(std::uint32_t phaseQn) { phase_ = phaseQn; }
  std::uint32_t getPhase() const { return phase_; }
  int setPhaseIncrement(std::uint32_t phaseIncrementQn) {
    phaseIncrementQn_ = phaseIncrementQn;
    return 0;
  }

  int nextSample(std::int32_t& sampleOut) {
    if (tableA_ == nullptr || tableB_ == nullptr) return -1;

    const std::uint32_t index = (phase_ >> fractionalBits_) & tableMask_;
    const std::uint32_t nextIndex = (index + 1u) & tableMask_;

#if RPDSP_HAS_HARDWARE_INTERP
    assertConfig();
    interp_->accum[0] = phase_;
    interp_->base01 = pack16(tableA_[index], tableA_[nextIndex]);
    const std::int32_t sA = static_cast<std::int32_t>(interp_->peek[1]);
    interp_->base01 = pack16(tableB_[index], tableB_[nextIndex]);
    const std::int32_t sB = static_cast<std::int32_t>(interp_->peek[1]);
    sampleOut = blendRaw(sA, sB, morphQ8_);
#else
    const std::uint32_t fractionMask = (1u << fractionalBits_) - 1u;
    const std::int32_t alpha =
        static_cast<std::int32_t>((phase_ & fractionMask) >> (fractionalBits_ - 8u));
    const std::int32_t sA = tableA_[index] +
        ((alpha * (tableA_[nextIndex] - tableA_[index])) >> 8);
    const std::int32_t sB = tableB_[index] +
        ((alpha * (tableB_[nextIndex] - tableB_[index])) >> 8);
    sampleOut = sA + (((sB - sA) * static_cast<std::int32_t>(morphQ8_)) >> 8);
#endif

    phase_ += phaseIncrementQn_;
    return 0;
  }

  // ---- Datapath reuse helpers -------------------------------------------
  // These borrow the already-configured blend lanes between oscillator
  // samples. Inputs are clamped to int16 range (the base01 register packs
  // two 16-bit values). Safe to call at any time because the oscillator
  // phase lives in software, not in the clobbered accumulator.

  // out = a + ((b - a) * alphaQ8) >> 8
  std::int32_t blendQ8(std::int32_t a, std::int32_t b, std::uint8_t alphaQ8) {
#if RPDSP_HAS_HARDWARE_INTERP
    assertConfig();
    return blendRaw(clamp16(a), clamp16(b), alphaQ8);
#else
    a = clamp16(a);
    b = clamp16(b);
    return a + (((b - a) * static_cast<std::int32_t>(alphaQ8)) >> 8);
#endif
  }

  // Four-quadrant ring modulation / bipolar VCA in one hardware blend:
  //   blend(-x, +x, modQ8)  =  x * (2*modQ8 - 256) / 256
  // modQ8 = 0 => -x (full inverted), 128 => silence, 255 => ~ +x.
  // For audio-rate ring mod, derive modQ8 from a signed 16-bit modulator:
  //   modQ8 = (mod16 + 32768) >> 8
  std::int32_t ringModQ8(std::int32_t x, std::uint8_t modQ8) {
    x = clamp16(x);
    return blendQ8(-x, x, modQ8);
  }

  // Convenience: ring-modulate a carrier by a signed 16-bit modulator.
  std::int32_t ringMod(std::int32_t carrier, std::int32_t mod16) {
    mod16 = clamp16(mod16);
    const std::uint8_t modQ8 =
        static_cast<std::uint8_t>((mod16 + 32768) >> 8);
    return ringModQ8(carrier, modQ8);
  }

  // Unipolar VCA: out = x * gainQ8 / 256 (blend from 0 to x).
  std::int32_t scaleQ8(std::int32_t x, std::uint8_t gainQ8) {
    return blendQ8(0, x, gainQ8);
  }

 private:
  static std::int32_t clamp16(std::int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32767) return -32767; // symmetric so -x is always representable
    return v;
  }

  static std::uint32_t pack16(std::int32_t a, std::int32_t b) {
    return (static_cast<std::uint32_t>(static_cast<std::uint16_t>(a))) |
           (static_cast<std::uint32_t>(static_cast<std::uint16_t>(b)) << 16);
  }

#if RPDSP_HAS_HARDWARE_INTERP
  void assertConfig() {
    interp_set_config(interp_, 0, &lane0Cfg_);
    interp_set_config(interp_, 1, &lane1Cfg_);
  }

  // Assumes assertConfig() has run. Places alphaQ8 where lane 1's
  // cross-input shift will extract it as the blend weight.
  std::int32_t blendRaw(std::int32_t a, std::int32_t b, std::uint8_t alphaQ8) {
    interp_->accum[0] =
        static_cast<std::uint32_t>(alphaQ8) << (fractionalBits_ - 8u);
    interp_->base01 = pack16(a, b);
    return static_cast<std::int32_t>(interp_->peek[1]);
  }
#endif

  interp_hw_t* interp_ = nullptr;
  const std::int16_t* tableA_ = nullptr;
  const std::int16_t* tableB_ = nullptr;
  std::uint32_t tableLength_ = 0;
  std::uint32_t tableMask_ = 0;
  std::uint32_t phase_ = 0;
  std::uint32_t phaseIncrementQn_ = 0;
  std::uint8_t tableBits_ = 0;
  std::uint8_t fractionalBits_ = 0;
  std::uint8_t morphQ8_ = 0;

#if RPDSP_HAS_HARDWARE_INTERP
  interp_config lane0Cfg_;
  interp_config lane1Cfg_;
#endif
};

// Wavefolder + saturating clamp time-multiplexed on a single interp1.
//
// Clamp mode only exists on interp1 and only on lane 0 — the same lane a
// mask-based wavefolder needs — so the two effects cannot coexist as
// separate statically-configured units. Instead this class claims interp1
// once and swaps the lane-0 control register between two precomputed
// configurations per call (a config write is a single-cycle store):
//
//   process(x) = clamp(fold(preGain * x), lo, hi)
//
// foldOnly() and clampOnly() are also exposed individually.
class HardwareFoldClamp {
 public:
  HardwareFoldClamp() = default;
  ~HardwareFoldClamp() { deinit(); }

  HardwareFoldClamp(const HardwareFoldClamp&) = delete;
  HardwareFoldClamp& operator=(const HardwareFoldClamp&) = delete;

  int init(HardwareInterpolatorPool::Resource resource,
           std::uint8_t foldOrder,
           std::int32_t lo,
           std::int32_t hi) {
    if (resource != HardwareInterpolatorPool::Resource::Core0Interp1 &&
        resource != HardwareInterpolatorPool::Resource::Core1Interp1) {
      return -4; // Clamp is only supported on interp1
    }
    if (foldOrder > 30) {
      return -1;
    }

    hw_ = HardwareInterpolatorPool::getHw(resource);
    if (!hw_) {
      return -2;
    }
    if (!HardwareInterpolatorPool::claim(resource)) {
      return -3;
    }
    resource_ = resource;
    foldOrder_ = foldOrder;
    numStages_ = 1;
    preGainQ16_ = 65536;
    lo_ = lo;
    hi_ = hi;

#if RPDSP_HAS_HARDWARE_INTERP
    foldCfg_ = interp_default_config();
    interp_config_set_shift(&foldCfg_, 0);
    interp_config_set_mask(&foldCfg_, 0, foldOrder_);
    interp_config_set_signed(&foldCfg_, true);

    clampCfg_ = interp_default_config();
    interp_config_set_clamp(&clampCfg_, true);
    interp_config_set_signed(&clampCfg_, true);
#endif

    initialized_ = true;
    return 0;
  }

  void deinit() {
    if (initialized_) {
      HardwareInterpolatorPool::release(resource_);
      initialized_ = false;
      hw_ = nullptr;
    }
  }

  void setGain(float preGain) {
    preGainQ16_ = static_cast<std::int32_t>(preGain * 65536.0f);
  }

  void setStages(std::uint8_t numStages) {
    if (numStages < 1 || numStages > 8) {
      return;
    }
    numStages_ = numStages;
  }

  int setFoldOrder(std::uint8_t foldOrder) {
    if (foldOrder > 30) return -1;
    foldOrder_ = foldOrder;
#if RPDSP_HAS_HARDWARE_INTERP
    interp_config_set_mask(&foldCfg_, 0, foldOrder_);
#endif
    return 0;
  }

  void setRange(std::int32_t lo, std::int32_t hi) {
    lo_ = lo;
    hi_ = hi;
  }

  std::int32_t foldOnly(std::int32_t sample) {
    std::int64_t gained = (static_cast<std::int64_t>(sample) * preGainQ16_) >> 16;
    std::int32_t val;
    if (gained > 2147483647LL) {
      val = 2147483647;
    } else if (gained < -2147483648LL) {
      val = -2147483648;
    } else {
      val = static_cast<std::int32_t>(gained);
    }

#if RPDSP_HAS_HARDWARE_INTERP
    interp_set_config(hw_, 0, &foldCfg_);
    hw_->base[0] = 0;
    for (std::uint8_t s = 0; s < numStages_; s++) {
      std::uint32_t abs_val = static_cast<std::uint32_t>(
          val < 0 ? -static_cast<std::int64_t>(val) : static_cast<std::int64_t>(val));
      hw_->accum[0] = abs_val;
      // Sign-extended masked value w in [-L, L); triangle fold = |w|.
      std::int32_t w = static_cast<std::int32_t>(hw_->peek[0]);
      std::int32_t tri = w < 0 ? -w : w;
      val = val < 0 ? -tri : tri;
    }
    return val;
#else
    const std::uint32_t foldLimit = 1u << foldOrder_;
    for (std::uint8_t s = 0; s < numStages_; s++) {
      std::uint32_t absVal = static_cast<std::uint32_t>(
          val < 0 ? -static_cast<std::int64_t>(val) : val);
      std::uint32_t doubleLimit = 2u * foldLimit;
      std::uint32_t mod = absVal % doubleLimit;
      std::int32_t tri = static_cast<std::int32_t>(
          foldLimit - (mod > foldLimit ? mod - foldLimit : foldLimit - mod));
      val = val < 0 ? -tri : tri;
    }
    return val;
#endif
  }

  std::int32_t clampOnly(std::int32_t x) {
#if RPDSP_HAS_HARDWARE_INTERP
    interp_set_config(hw_, 0, &clampCfg_);
    hw_->base[0] = static_cast<std::uint32_t>(lo_);
    hw_->base[1] = static_cast<std::uint32_t>(hi_);
    hw_->accum[0] = static_cast<std::uint32_t>(x);
    return static_cast<std::int32_t>(hw_->peek[0]);
#else
    return x < lo_ ? lo_ : (x > hi_ ? hi_ : x);
#endif
  }

  std::int32_t process(std::int32_t sample) {
    return clampOnly(foldOnly(sample));
  }

  void processBlock(const std::int32_t* src, std::int32_t* dst, size_t count) {
    for (size_t i = 0; i < count; i++) {
      dst[i] = process(src[i]);
    }
  }

 private:
  interp_hw_t* hw_ = nullptr;
  HardwareInterpolatorPool::Resource resource_ = HardwareInterpolatorPool::Resource::Core0Interp1;
  std::uint8_t foldOrder_ = 0;
  std::uint8_t numStages_ = 1;
  std::int32_t preGainQ16_ = 65536;
  std::int32_t lo_ = 0;
  std::int32_t hi_ = 0;
  bool initialized_ = false;
#if RPDSP_HAS_HARDWARE_INTERP
  interp_config foldCfg_;
  interp_config clampCfg_;
#endif
};

} // namespace rpdsp
// EOF
