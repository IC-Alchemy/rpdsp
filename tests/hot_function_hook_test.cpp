// Host-only program (see dark_reverb_test.cpp for the Arduino-build guard).
//
// RPDSP_HOT_FUNCTION must reach both DarkReverb::process() overloads for every
// storage type, so a firmware can place them in RAM. This test defines the hook as
// a 4 KiB alignment and checks that all six functions (two overloads, three
// instantiations) really start on a 4 KiB boundary, which is how an attribute that
// was not applied shows itself: by chance a function lands there once in 4096.
// Alignment is used because it is observable on any GNU-compatible host compiler;
// GCC 13.3 on x86-64 does not honor `section` on class-template members, while the
// ARM toolchain does. Firmware placement is therefore checked in the linked ELF
// (see README.md), not here. This test guards the wiring, and that the attribute
// leaves the output finite and unchanged.
#ifndef ARDUINO
#if defined(__GNUC__)

#define RPDSP_HOT_FUNCTION __attribute__((noinline, aligned(4096)))
#include <rpdsp/dark_reverb.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace {
void check(bool ok, const char* message) {
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

// Itanium ABI: a non-virtual member function pointer's first word is the address.
template <class Fn>
std::uintptr_t addressOf(Fn fn) {
  static_assert(sizeof(Fn) >= sizeof(std::uintptr_t), "member function pointer layout");
  std::uintptr_t address;
  std::memcpy(&address, &fn, sizeof address);
  return address;
}

std::uint32_t bits(float value) {
  std::uint32_t result;
  std::memcpy(&result, &value, sizeof result);
  return result;
}

template <class Reverb>
void checkWiringAndBehavior(const char* name) {
  using Block = void (Reverb::*)(const float*, const float*, float*, float*, std::size_t);
  using Scalar = std::array<float, 2> (Reverb::*)(float, float);
  const std::uintptr_t block = addressOf<Block>(&Reverb::process);
  const std::uintptr_t scalar = addressOf<Scalar>(&Reverb::process);
  std::printf("%s: block %% 4096 = %lu, per-sample %% 4096 = %lu\n", name,
              static_cast<unsigned long>(block % 4096), static_cast<unsigned long>(scalar % 4096));
  check(block % 4096 == 0, "block process() carries RPDSP_HOT_FUNCTION");
  check(scalar % 4096 == 0, "per-sample process() carries RPDSP_HOT_FUNCTION");

  // The attribute must not change behavior: the block and per-sample paths still
  // agree (bit for bit in IEEE builds) and the output is finite and audible.
  auto blockReverb = std::make_unique<Reverb>();
  auto scalarReverb = std::make_unique<Reverb>();
  for (auto* reverb : {blockReverb.get(), scalarReverb.get()}) {
    reverb->prepare(48000.0f);
    reverb->setMix(1.0f);
  }
  float in[256] = {1.0f, 0.5f, -0.25f};
  float outL[256] = {};
  float outR[256] = {};
  blockReverb->process(in, in, outL, outR, 256);
  bool audible = false;
  for (int i = 0; i < 256; ++i) {
    const auto frame = scalarReverb->process(in[i], in[i]);
    check((bits(outL[i]) & 0x7F800000u) != 0x7F800000u, "finite output");  // exponent bits: safe under -ffinite-math-only
#ifdef __FAST_MATH__
    check(std::fabs(outL[i] - frame[0]) <= 2.0e-4f * (1.0f + std::fabs(outL[i])), "block and per-sample agree");
#else
    check(bits(outL[i]) == bits(frame[0]) && bits(outR[i]) == bits(frame[1]), "block and per-sample agree");
#endif
    if (outL[i] != 0.0f || outR[i] != 0.0f) audible = true;
  }
  check(audible, "the tank produces output");
}
}  // namespace

int main() {
  checkWiringAndBehavior<rpdsp::DarkReverb<4096, rpdsp::DarkReverbStorage::Half>>("Half 4096");
  checkWiringAndBehavior<rpdsp::DarkReverb<16384, rpdsp::DarkReverbStorage::Half>>("Half 16384");
  checkWiringAndBehavior<rpdsp::DarkReverb<16384, rpdsp::DarkReverbStorage::Float>>("Float 16384");
  std::puts("hot function hook: ok");
  return 0;
}

#else
#include <cstdio>
int main() {
  std::puts("hot function hook: skipped (needs a GNU-compatible compiler)");
  return 0;
}
#endif
#endif  // ARDUINO
