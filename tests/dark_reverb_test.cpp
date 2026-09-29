#include <rpdsp/dark_reverb.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

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

std::uint64_t hash = 14695981039346656037ull;
std::FILE* trace = nullptr;
void digest(float value) {
  hash ^= bits(value);
  hash *= 1099511628211ull;
  if (trace) check(std::fwrite(&value, sizeof value, 1, trace) == 1, "write trace");
}

// Run separately with normal IEEE flags and -ffast-math. The latter permits
// reassociation, so scalar/block equality uses a small numerical tolerance.
void equivalent(float a, float b) {
#ifdef __FAST_MATH__
  check(std::fabs(a - b) <= 2.0e-4f * (1.0f + std::fabs(a)), "scalar/block agreement");
#else
  check(bits(a) == bits(b), "bit-identical scalar/block output");
#endif
}

template <class Reverb>
void controls(Reverb& reverb, unsigned step) {
  // Includes repeated targets, endpoints, clamping, freeze transitions and
  // simultaneous depth/rate changes. All setters run on the audio owner.
  static constexpr float decay[] = {20.0f, 0.1f, 60.0f, 1000.0f, -1.0f};
  static constexpr float damping[] = {3000.0f, 100.0f, 100000.0f};
  static constexpr float rates[] = {0.5f, 0.01f, 5.0f};
  reverb.setDecaySeconds(decay[(step / 3) % 5]);
  reverb.setDampingHz(damping[(step / 5) % 3]);
  reverb.setLowCutHz((step / 7) % 2 ? 40.0f : 1000.0f);
  reverb.setModDepth(static_cast<float>((step / 4) % 5) * 0.25f);
  reverb.setModRateHz(rates[(step / 9) % 3]);
  reverb.setFreeze((step / 11) % 3 == 1);
  reverb.setDiffusion(static_cast<float>((step / 6) % 5) * 0.25f);
  reverb.setWidth(static_cast<float>((step / 8) % 5) * 0.5f);
  reverb.setMix(static_cast<float>((step / 10) % 5) * 0.25f);
}

template <std::size_t Capacity, rpdsp::DarkReverbStorage Storage>
void exercise(float sampleRate) {
  using Reverb = rpdsp::DarkReverb<Capacity, Storage>;
  auto scalar = std::make_unique<Reverb>();
  auto block = std::make_unique<Reverb>();
  auto inplace = std::make_unique<Reverb>();
  for (auto* reverb : {scalar.get(), block.get(), inplace.get()}) {
    // Setter use before prepare must still take effect after prepare.
    controls(*reverb, 0);
    reverb->prepare(sampleRate);
  }
  constexpr std::size_t frames = 196613;
  std::vector<float> left(frames), right(frames), outL(frames), outR(frames);
  rpdsp::XorShift32 noise(0x173b54u);
  for (std::size_t i = 0; i < frames; ++i) {
    left[i] = i == 0 ? 1.0f : (i < frames / 3 ? 0.15f * noise.nextBipolar() : 0.0f);
    right[i] = i < frames / 4 ? 0.15f * noise.nextBipolar() : 0.0f;
  }
  auto inoutL = left;
  auto inoutR = right;
  constexpr std::size_t sizes[] = {0, 1, 31, 2, 33, 256, 7, 511, 64, 3};
  unsigned step = 0;
  for (std::size_t pos = 0; pos < frames; ++step) {
    const auto count = std::min(sizes[step % 10], frames - pos);
    for (auto* reverb : {scalar.get(), block.get(), inplace.get()}) controls(*reverb, step);
    block->process(left.data() + pos, right.data() + pos, outL.data() + pos, outR.data() + pos, count);
    inplace->process(inoutL.data() + pos, inoutR.data() + pos,
                     inoutL.data() + pos, inoutR.data() + pos, count);
    for (std::size_t k = pos; k < pos + count; ++k) {
      const auto expected = scalar->process(left[k], right[k]);
      equivalent(expected[0], outL[k]);
      equivalent(expected[1], outR[k]);
      check(bits(outL[k]) == bits(inoutL[k]) && bits(outR[k]) == bits(inoutR[k]), "in-place agreement");
      // Exponent bits keep this check effective under -ffinite-math-only.
      check((bits(outL[k]) & 0x7f800000u) != 0x7f800000u, "finite left output");
      check((bits(outR[k]) & 0x7f800000u) != 0x7f800000u, "finite right output");
      digest(outL[k]);
      digest(outR[k]);
    }
    pos += count;
    // Mix APIs on one object, including pending odd-sample state.
    if (pos < frames && step % 3 == 0) {
      const auto a = scalar->process(left[pos], right[pos]);
      const auto b = block->process(left[pos], right[pos]);
      const auto c = inplace->process(left[pos], right[pos]);
      equivalent(a[0], b[0]); equivalent(a[1], b[1]);
      equivalent(b[0], c[0]); equivalent(b[1], c[1]);
      digest(b[0]); digest(b[1]);
      ++pos;
    }
  }

  // Reprepare at another rate with unchanged targets must rebuild the
  // rate-dependent coefficients in prepare().
  block->prepare(sampleRate * 0.75f);
  scalar->prepare(sampleRate * 0.75f);
  for (int i = 0; i < 4097; ++i) {
    const float x = i == 0 ? 1.0f : 0.0f;
    const auto a = scalar->process(x, x);
    float b, c;
    block->process(&x, &x, &b, &c, 1);
    equivalent(a[0], b); equivalent(a[1], c);
    digest(b); digest(c);
  }
  block->reset();
  for (int i = 0; i < 1024; ++i) {
    const auto out = block->process(0.0f, 0.0f);
    check(out[0] == 0.0f && out[1] == 0.0f, "reset clears pending samples and tail");
  }
}

template <rpdsp::DarkReverbStorage Storage>
void tail() {
  auto reverb = std::make_unique<rpdsp::DarkReverb<16384, Storage>>();
  reverb->prepare(48000.0f);
  reverb->setMix(1.0f);
  reverb->setDecaySeconds(60.0f);
  double energy = 0.0;
  for (int i = 0; i < 48000 * 8; ++i) {
    const float x = i == 0 ? 1.0f : 0.0f;
    const auto out = reverb->process(x, x);
    if (i >= 48000 * 7) energy += out[0] * out[0] + out[1] * out[1];
    digest(out[0]); digest(out[1]);
  }
  check(energy > 1.0e-8 && energy < 100.0, "long tail persists and stays bounded");
}
} // namespace

int main(int argc, char** argv) {
  check(argc <= 2, "usage: dark_reverb_test [output-f32-trace]");
  if (argc == 2) {
    trace = std::fopen(argv[1], "wb");
    check(trace != nullptr, "open trace");
  }
  using rpdsp::DarkReverbStorage;
  for (float rate : {8000.0f, 44100.0f, 48000.0f, 96000.0f}) {
    exercise<4096, DarkReverbStorage::Half>(rate);
    exercise<16384, DarkReverbStorage::Half>(rate);
    exercise<65536, DarkReverbStorage::Half>(rate);
    exercise<4096, DarkReverbStorage::Float>(rate);
    exercise<16384, DarkReverbStorage::Float>(rate);
    exercise<65536, DarkReverbStorage::Float>(rate);
  }
  tail<DarkReverbStorage::Half>();
  tail<DarkReverbStorage::Float>();
  std::printf("PASS dark reverb; deterministic output hash %016llx\n", static_cast<unsigned long long>(hash));
  std::printf("object bytes: Half=%zu Float=%zu\n", sizeof(rpdsp::DarkReverb<>),
              sizeof(rpdsp::DarkReverb<16384, DarkReverbStorage::Float>));
  if (trace) check(std::fclose(trace) == 0, "close trace");
}
