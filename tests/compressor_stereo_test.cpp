#include <rpdsp/dynamics.h>
#include <rpdsp/realtime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// Host checks for the linked stereo Compressor API. Run separately with normal
// IEEE flags (bit-exact comparisons) and with -O3 -ffast-math, which permits
// reassociation, so equality there uses a small numerical tolerance.

namespace {
void check(bool ok, const char* message) {
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

std::uint32_t bits(float value) {
  std::uint32_t result;
  std::memcpy(&result, &value, sizeof result);
  return result;
}

// Exponent-bits test so -ffinite-math-only cannot optimize the check away.
bool finite(float value) { return (bits(value) & 0x7F800000u) != 0x7F800000u; }

std::uint64_t hash = 14695981039346656037ull;
void digest(float value) {
  hash ^= bits(value);
  hash *= 1099511628211ull;
}

void equivalent(float a, float b, const char* message) {
#ifdef __FAST_MATH__
  check(std::fabs(a - b) <= 1.0e-5f * (1.0f + std::fabs(a)), message);
#else
  check(bits(a) == bits(b), message);
#endif
}

struct Settings {
  float thresholdDb, ratio, kneeDb, attackMs, releaseMs, makeupDb;
};

// The three Pico2Seq master-macro anchors (Warm, Glue, Punch), a hard knee and
// a ratio of 1 (no reduction), so both curve branches and the makeup-only path
// are exercised.
constexpr std::array<Settings, 5> kSettings = {{
    {-14.0f, 2.5f, 9.0f, 30.0f, 250.0f, 2.0f},
    {-10.0f, 1.8f, 6.0f, 15.0f, 150.0f, 1.5f},
    {-14.0f, 6.0f, 2.0f, 6.0f, 70.0f, 4.0f},
    {-20.0f, 4.0f, 0.0f, 1.0f, 40.0f, 0.0f},
    {-10.0f, 1.0f, 6.0f, 15.0f, 150.0f, 0.0f},
}};

void configure(rpdsp::Compressor& comp, const Settings& s, float sampleRate) {
  comp.prepare(sampleRate);
  comp.setThresholdDb(s.thresholdDb);
  comp.setRatio(s.ratio);
  comp.setKneeWidthDb(s.kneeDb);
  comp.setAttackRelease(s.attackMs, s.releaseMs);
  comp.setMakeupGainDb(s.makeupDb);
  comp.reset();
}

// Deterministic program: silence, loud/quiet pairs on either side, inverted
// polarity, noise and a decaying tail, so the louder channel swaps sides and
// the detector sees attacks, holds and releases.
constexpr std::size_t kFrames = 96000;
struct Program {
  std::vector<float> left = std::vector<float>(kFrames);
  std::vector<float> right = std::vector<float>(kFrames);
  Program() {
    rpdsp::XorShift32 noise(0x5eed1234u);
    struct Segment { float left, right; bool noisy; };
    constexpr Segment segments[] = {
        {0.0f, 0.0f, false}, {0.9f, 0.9f, false}, {0.9f, 0.05f, false}, {0.05f, 0.9f, false},
        {0.02f, 0.02f, false}, {0.6f, -0.6f, false}, {0.8f, 0.4f, true}, {0.0f, 0.0f, false},
        {0.5f, 0.5f, true}, {1.4f, 0.1f, false}, {0.001f, 0.001f, false}, {0.7f, 0.7f, false},
    };
    constexpr std::size_t count = sizeof(segments) / sizeof(segments[0]);
    const std::size_t length = kFrames / count;
    for (std::size_t i = 0; i < kFrames; ++i) {
      const Segment& segment = segments[std::min(i / length, count - 1)];
      const float phase = 6.283185307f * 220.0f * static_cast<float>(i) / 48000.0f;
      const float tone = std::sin(phase);
      const float grit = segment.noisy ? 0.5f * noise.nextBipolar() : 0.0f;
      left[i] = segment.left * (segment.noisy ? grit + 0.5f * tone : tone);
      right[i] = segment.right * (segment.noisy ? grit + 0.5f * tone : tone);
    }
  }
};

// 1. Equal channels: bit-identical to the mono transfer, on both outputs.
void equalChannelsMatchMono(const Program& program) {
  for (const Settings& s : kSettings) {
    for (const float sampleRate : {48000.0f, 44100.0f, 96000.0f}) {
      rpdsp::Compressor mono;
      rpdsp::Compressor stereo;
      configure(mono, s, sampleRate);
      configure(stereo, s, sampleRate);
      for (std::size_t i = 0; i < kFrames; ++i) {
        const float x = program.left[i];
        const float expected = mono.process(x);
        float left = x;
        float right = x;
        stereo.processStereo(left, right);
        equivalent(left, expected, "equal channels: left equals mono");
        equivalent(right, expected, "equal channels: right equals mono");
        digest(left);
      }
    }
  }
}

// Independent reimplementation from the public building blocks, advanced once
// per frame with the louder channel. The linked API must match it exactly.
void singleAdvancePerFrame(const Program& program) {
  for (const Settings& s : kSettings) {
    rpdsp::Compressor linked;
    configure(linked, s, 48000.0f);
    rpdsp::EnvelopeFollower detector;
    rpdsp::CompressorStaticCurve curve;
    rpdsp::GainReductionSmoother smoother;
    detector.prepare(48000.0f);
    smoother.prepare(48000.0f);
    detector.setAttackRelease(s.attackMs, s.releaseMs);
    smoother.setAttackRelease(s.attackMs, s.releaseMs);
    curve.setThresholdDb(s.thresholdDb);
    curve.setRatio(s.ratio);
    curve.setKneeWidthDb(s.kneeDb);
    detector.reset();
    smoother.reset();
    for (std::size_t i = 0; i < kFrames; ++i) {
      const float l = program.left[i];
      const float r = program.right[i];
      const float level = detector.process(std::max(std::fabs(l), std::fabs(r)));
      const float reduction = smoother.process(curve.gainReductionDb(rpdsp::gainToDb(level)));
      const float gain = rpdsp::dbToGain(reduction + s.makeupDb);
      float left = l;
      float right = r;
      linked.processStereo(left, right);
      equivalent(left, l * gain, "reference: left");
      equivalent(right, r * gain, "reference: right");
    }
  }
}

// The mono transfer is the formula the class shipped with (detector ->
// dB -> curve -> smoother -> makeup): refactoring it into a shared gain helper
// must not change one bit.
void monoMatchesPublicBlocks(const Program& program) {
  for (const Settings& s : kSettings) {
    rpdsp::Compressor mono;
    configure(mono, s, 48000.0f);
    rpdsp::EnvelopeFollower detector;
    rpdsp::CompressorStaticCurve curve;
    rpdsp::GainReductionSmoother smoother;
    detector.prepare(48000.0f);
    smoother.prepare(48000.0f);
    detector.setAttackRelease(s.attackMs, s.releaseMs);
    smoother.setAttackRelease(s.attackMs, s.releaseMs);
    curve.setThresholdDb(s.thresholdDb);
    curve.setRatio(s.ratio);
    curve.setKneeWidthDb(s.kneeDb);
    detector.reset();
    smoother.reset();
    for (std::size_t i = 0; i < kFrames; ++i) {
      const float x = program.right[i];
      const float level = detector.process(x);
      const float reduction = smoother.process(curve.gainReductionDb(rpdsp::gainToDb(level)));
      equivalent(mono.process(x), x * rpdsp::dbToGain(reduction + s.makeupDb), "mono matches the public building blocks");
    }
  }
}

// 2. One gain for both channels: the level ratio between channels is kept.
void sharedGainKeepsImage(const Program& program) {
  rpdsp::Compressor comp;
  configure(comp, kSettings[2], 48000.0f);
  std::size_t compared = 0;
  for (std::size_t i = 0; i < kFrames; ++i) {
    float left = program.left[i];
    float right = program.right[i];
    comp.processStereo(left, right);
    check(finite(left) && finite(right), "finite stereo output");
    if (std::fabs(program.left[i]) > 1.0e-3f && std::fabs(program.right[i]) > 1.0e-3f) {
      const float gainLeft = left / program.left[i];
      const float gainRight = right / program.right[i];
      check(std::fabs(gainLeft - gainRight) <= 1.0e-6f * std::fabs(gainLeft), "one gain for both channels");
      ++compared;
    }
    digest(left);
    digest(right);
  }
  check(compared > kFrames / 2, "shared-gain check covered the program");
}

// 3. Why the API exists: independent compressors lean the image, and driving
// one mono instance twice per frame advances its timing twice.
void linkedBeatsNaiveAlternatives() {
  constexpr std::size_t frames = 24000;
  rpdsp::Compressor linked;
  rpdsp::Compressor leftOnly;
  rpdsp::Compressor rightOnly;
  rpdsp::Compressor twice;
  const Settings& s = kSettings[0];
  for (rpdsp::Compressor* comp : {&linked, &leftOnly, &rightOnly, &twice}) configure(*comp, s, 48000.0f);
  double linkedWorst = 0.0;
  double independentWorst = 0.0;
  double advanceGap = 0.0;
  for (std::size_t i = 0; i < frames; ++i) {
    const float tone = std::sin(6.283185307f * 220.0f * static_cast<float>(i) / 48000.0f);
    const float inL = 0.9f * tone;    // hot side
    const float inR = 0.03f * tone;   // quiet side, far below threshold
    float l = inL;
    float r = inR;
    linked.processStereo(l, r);
    const float indL = leftOnly.process(inL);
    const float indR = rightOnly.process(inR);
    const float a = twice.process(inL);
    const float b = twice.process(inR);
    (void)b;
    if (i > 2400 && std::fabs(inL) > 0.05f) {
      const double inputRatioDb = 20.0 * std::log10(std::fabs(inR / inL));
      linkedWorst = std::max(linkedWorst, std::fabs(20.0 * std::log10(std::fabs(r / l)) - inputRatioDb));
      independentWorst = std::max(independentWorst, std::fabs(20.0 * std::log10(std::fabs(indR / indL)) - inputRatioDb));
      advanceGap = std::max(advanceGap, static_cast<double>(std::fabs(a - l)));
    }
  }
  check(linkedWorst < 1.0e-3, "linked compression keeps the L/R level ratio");
  check(independentWorst > 1.0, "independent compressors do shift the L/R level ratio");
  check(advanceGap > 0.01, "double-advancing a mono compressor differs from one linked step per frame");
}

// 4. The block form is a loop over frames: any partition gives one answer.
void blocksMatchFrames(const Program& program) {
  rpdsp::Compressor scalar;
  rpdsp::Compressor block;
  configure(scalar, kSettings[1], 48000.0f);
  configure(block, kSettings[1], 48000.0f);
  std::vector<float> l = program.left;
  std::vector<float> r = program.right;
  for (std::size_t i = 0; i < kFrames; ++i) scalar.processStereo(l[i], r[i]);

  std::vector<float> bl = program.left;
  std::vector<float> br = program.right;
  constexpr std::size_t sizes[] = {0, 1, 2, 3, 7, 64, 255, 256, 257, 1000};
  std::size_t at = 0;
  for (std::size_t call = 0; at < kFrames; ++call) {
    const std::size_t n = std::min(sizes[call % (sizeof(sizes) / sizeof(sizes[0]))], kFrames - at);
    block.processStereo(bl.data() + at, br.data() + at, n);
    at += n;
  }
  for (std::size_t i = 0; i < kFrames; ++i) {
    equivalent(bl[i], l[i], "block partition: left");
    equivalent(br[i], r[i], "block partition: right");
    digest(bl[i]);
  }
}

// 5. The same pointer for both channels is one mono buffer, not gain twice.
void aliasedBufferIsMono(const Program& program) {
  rpdsp::Compressor mono;
  rpdsp::Compressor aliased;
  configure(mono, kSettings[0], 48000.0f);
  configure(aliased, kSettings[0], 48000.0f);
  std::vector<float> buffer = program.left;
  aliased.processStereo(buffer.data(), buffer.data(), kFrames);
  for (std::size_t i = 0; i < kFrames; ++i) equivalent(buffer[i], mono.process(program.left[i]), "aliased channels are mono");
}

// 6. Silence, reset, sample rates and hostile magnitudes.
void edgeCases(const Program& program) {
  {
    rpdsp::Compressor comp;
    configure(comp, kSettings[0], 48000.0f);
    for (int i = 0; i < 4800; ++i) {
      float l = 0.0f;
      float r = 0.0f;
      comp.processStereo(l, r);
      check(l == 0.0f && r == 0.0f, "silence stays silent");
    }
  }
  {
    rpdsp::Compressor comp;
    configure(comp, kSettings[2], 48000.0f);
    std::vector<float> first(2 * kFrames / 4);
    for (std::size_t i = 0; i < first.size() / 2; ++i) {
      float l = program.left[i];
      float r = program.right[i];
      comp.processStereo(l, r);
      first[2 * i] = l;
      first[2 * i + 1] = r;
    }
    comp.reset();
    for (std::size_t i = 0; i < first.size() / 2; ++i) {
      float l = program.left[i];
      float r = program.right[i];
      comp.processStereo(l, r);
      check(bits(l) == bits(first[2 * i]) && bits(r) == bits(first[2 * i + 1]), "reset reproduces the first run exactly");
    }
  }
  for (const float sampleRate : {8000.0f, 22050.0f, 44100.0f, 96000.0f, 192000.0f}) {
    rpdsp::Compressor comp;
    configure(comp, kSettings[1], sampleRate);
    for (std::size_t i = 0; i < 4096; ++i) {
      float l = program.left[i * 3];
      float r = program.right[i * 3];
      comp.processStereo(l, r);
      check(finite(l) && finite(r), "finite output at every sample rate");
    }
  }
  {
    // Hostile magnitudes: denormals, tiny, unity and very large inputs.
    rpdsp::Compressor comp;
    configure(comp, kSettings[2], 48000.0f);
    constexpr float values[] = {1.0e-38f, -1.0e-30f, 1.0e-12f, 0.5f, -1.0f, 8.0f, 1.0e6f, -1.0e9f, 0.0f, 0.25f};
    for (int repeat = 0; repeat < 200; ++repeat) {
      for (const float v : values) {
        float l = v;
        float r = -0.5f * v;
        comp.processStereo(l, r);
        check(finite(l) && finite(r), "finite output for hostile magnitudes");
      }
    }
  }
}
}  // namespace

int main() {
  const Program program;
  equalChannelsMatchMono(program);
  monoMatchesPublicBlocks(program);
  singleAdvancePerFrame(program);
  sharedGainKeepsImage(program);
  linkedBeatsNaiveAlternatives();
  blocksMatchFrames(program);
  aliasedBufferIsMono(program);
  edgeCases(program);
  std::printf("PASS compressor stereo; deterministic output hash %016llx\n", static_cast<unsigned long long>(hash));
  return 0;
}
