#pragma once

#include <algorithm>

namespace rpdsp {

class ADSR {
public:
  // Integer sample counters make envelope timing deterministic across host and firmware builds.
  enum class Stage { kIdle, kAttack, kDecay, kSustain, kRelease };

  void prepare(float sampleRate) {
    sampleRate_ = sampleRate > 1.0f ? sampleRate : kDefaultSampleRate;
  }

  void reset() {
    value_ = 0.0f;
    releaseStart_ = 0.0f;
    releaseProgress_ = 0;
    stage_ = Stage::kIdle;
  }

  // Each stage also caches 1/length: process() then multiplies the progress
  // counter instead of dividing by the length every sample (a float divide is
  // ~14 cycles on Cortex-M33 against 1 for the multiply). Differs from the
  // division by at most 1 ulp; stage ends are still pinned exactly by the
  // integer counter compare.
  void setAttack(float seconds) {
    attackSamples_ = secondsToSamples(seconds);
    attackStep_ = 1.0f / static_cast<float>(attackSamples_);
  }

  void setDecay(float seconds) {
    decaySamples_ = secondsToSamples(seconds);
    decayStep_ = 1.0f / static_cast<float>(decaySamples_);
  }
  void setSustain(float level) { sustain_ = clamp01(level); }
  void setRelease(float seconds) {
    releaseSamples_ = secondsToSamples(seconds);
    releaseStep_ = 1.0f / static_cast<float>(releaseSamples_);
  }

  void set(float attackSeconds, float decaySeconds, float sustain,
           float releaseSeconds) {
    setAttack(attackSeconds);
    setDecay(decaySeconds);
    setSustain(sustain);
    setRelease(releaseSeconds);
  }

  void noteOn() {
    // Restart attack from the current state machine position; reset() is explicit when silence is desired.
    stage_ = Stage::kAttack;
    attackProgress_ = 0;
    decayProgress_ = 0;
  }

  void noteOff() {
    if (stage_ != Stage::kIdle) {
      // Capture the current value so release is click-free even when noteOff lands mid-attack.
      stage_ = Stage::kRelease;
      releaseStart_ = value_;
      releaseProgress_ = 0;
    }
  }

  float process() {
    switch (stage_) {
    case Stage::kIdle:
      value_ = 0.0f;
      break;
    case Stage::kAttack:
      ++attackProgress_;
      value_ = static_cast<float>(attackProgress_) * attackStep_;
      if (attackProgress_ >= attackSamples_) {
        value_ = 1.0f;
        stage_ = Stage::kDecay;
        decayProgress_ = 0;
      }
      break;
    case Stage::kDecay:
      ++decayProgress_;
      value_ = lerp(1.0f, sustain_,
                    static_cast<float>(decayProgress_) * decayStep_);
      if (decayProgress_ >= decaySamples_) {
        value_ = sustain_;
        stage_ = Stage::kSustain;
      }
      break;
    case Stage::kSustain:
      value_ = sustain_;
      break;
    case Stage::kRelease:
      ++releaseProgress_;
      value_ = lerp(releaseStart_, 0.0f,
                    static_cast<float>(releaseProgress_) * releaseStep_);
      if (releaseProgress_ >= releaseSamples_) {
        value_ = 0.0f;
        stage_ = Stage::kIdle;
      }
      break;
    }
    return clamp01(value_);
  }

  [[nodiscard]] bool isActive() const { return stage_ != Stage::kIdle; }
  [[nodiscard]] Stage stage() const { return stage_; }
  [[nodiscard]] float value() const { return value_; }

private:
  static constexpr float kDefaultSampleRate = 48000.0f;

  static float clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
  }

  static float lerp(float start, float end, float amount) {
    return start + (end - start) * amount;
  }

  int secondsToSamples(float seconds) const {
    // A zero-length stage still advances over one sample instead of dividing by zero.
    return std::max(1, static_cast<int>(seconds * sampleRate_ + 0.5f));
  }

  float sampleRate_ = kDefaultSampleRate;
  float sustain_ = 0.5f;
  float value_ = 0.0f;
  float releaseStart_ = 0.0f;
  int attackSamples_ = 480;
  int decaySamples_ = 4800;
  int releaseSamples_ = 9600;
  // Reciprocals of the stage lengths above; kept in sync by the setters.
  float attackStep_ = 1.0f / 480.0f;
  float decayStep_ = 1.0f / 4800.0f;
  float releaseStep_ = 1.0f / 9600.0f;
  int attackProgress_ = 0;
  int decayProgress_ = 0;
  int releaseProgress_ = 0;
  Stage stage_ = Stage::kIdle;
};

} // namespace rpdsp
