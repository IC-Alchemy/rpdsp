// Host-only program. The Arduino build compiles every .cpp below the sketch's
// src/, which includes a checked-out src/rpdsp/tests, so the file is empty there.
#ifndef ARDUINO

// Host timing is useful for regression checks, not for estimating RP2350 CPU.
#include <rpdsp/dark_reverb.h>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>

using Clock = std::chrono::steady_clock;
volatile float sink = 0.0f;

template <class Reverb>
__attribute__((noinline)) void setDepth(Reverb& reverb, float depth) {
  reverb.setModDepth(depth);
}

template <class Reverb>
__attribute__((noinline)) void render(Reverb& reverb, const float* left, const float* right,
                                     float* outLeft, float* outRight) {
  reverb.process(left, right, outLeft, outRight, 256);
}

template <rpdsp::DarkReverbStorage Storage>
void benchmark(const char* label) {
  using Reverb = rpdsp::DarkReverb<16384, Storage>;
  auto reverb = std::make_unique<Reverb>();
  reverb->prepare(48000.0f);
  std::array<float, 256> left{}, right{}, outLeft{}, outRight{};
  rpdsp::XorShift32 noise;
  for (unsigned i = 0; i < 256; ++i) {
    left[i] = noise.nextBipolar() * 0.1f;
    right[i] = noise.nextBipolar() * 0.1f;
  }
  constexpr unsigned blocks = 6000;
  for (unsigned i = 0; i < 100; ++i)
    render(*reverb, left.data(), right.data(), outLeft.data(), outRight.data());
  const auto start = Clock::now();
  for (unsigned i = 0; i < blocks; ++i)
    render(*reverb, left.data(), right.data(), outLeft.data(), outRight.data());
  const auto end = Clock::now();
  sink = outLeft[17] + outRight[49];
  constexpr unsigned updates = 1000000;
  const auto controlStart = Clock::now();
  for (unsigned i = 0; i < updates; ++i)
    setDepth(*reverb, static_cast<float>(i & 255) * (1.0f / 255.0f));
  const auto controlEnd = Clock::now();
  render(*reverb, left.data(), right.data(), outLeft.data(), outRight.data());
  sink = outLeft[17] + outRight[49];
  std::printf("%s: %.2f ns/frame, %.2f ns/depth update\n", label,
      std::chrono::duration<double, std::nano>(end - start).count() / (blocks * 256.0),
      std::chrono::duration<double, std::nano>(controlEnd - controlStart).count() / updates);
}

int main() {
  benchmark<rpdsp::DarkReverbStorage::Half>("Half (software conversion on this host)");
  benchmark<rpdsp::DarkReverbStorage::Float>("Float");
}

#endif  // !ARDUINO
