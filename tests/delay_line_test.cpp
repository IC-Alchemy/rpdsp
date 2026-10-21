// Host-only program. The Arduino build compiles every .cpp below the sketch's
// src/, which includes a checked-out src/rpdsp/tests, so the file is empty there.
#ifndef ARDUINO

#include <rpdsp/delay_line.h>
#include <rpdsp/interpolation.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

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
// Binary trace of every cubic read, in the same shape the dark reverb test
// writes: native-endian float32 samples.
std::ofstream trace;

void digest(float value) {
  hash ^= bits(value);
  hash *= 1099511628211ull;
  if (trace.is_open()) {
    trace.write(reinterpret_cast<const char*>(&value), sizeof value);
    check(static_cast<bool>(trace), "write trace");
  }
}

constexpr size_t kCapacity = 16;
using Line = rpdsp::DelayLine<kCapacity>;

// The four samples DelayLine::readCubic() interpolates, plus the fraction it
// interpolates at. The ages are the documented tap selection: the delay is
// clamped to [0, Capacity - 1], the lower neighbour clamps to age 0 and the
// upper neighbours to age Capacity - 1.
struct Taps {
  float frac;
  float ym1;
  float y0;
  float y1;
  float y2;
};

Taps tapsFor(const Line& line, float delaySamples) {
  float clamped = delaySamples;
  if (clamped < 0.0f) clamped = 0.0f;
  const float maxDelay = static_cast<float>(kCapacity - 1);
  if (clamped > maxDelay) clamped = maxDelay;
  const auto whole = static_cast<size_t>(clamped);
  const size_t lower = whole > 0 ? whole - 1 : 0;
  const size_t upper = whole < kCapacity - 1 ? whole + 1 : kCapacity - 1;
  const size_t upper2 = upper < kCapacity - 1 ? upper + 1 : kCapacity - 1;
  return Taps{clamped - static_cast<float>(whole), line.read(lower), line.read(whole),
              line.read(upper), line.read(upper2)};
}

// Double-precision evaluation of the same third-order Lagrange polynomial,
// used as the reference the float result is compared against.
double lagrangeAt(double frac, double ym1, double y0, double y1, double y2) {
  const double c0 = (-frac * (frac - 1.0) * (frac - 2.0)) * (1.0 / 6.0);
  const double c1 = ((frac + 1.0) * (frac - 1.0) * (frac - 2.0)) * 0.5;
  const double c2 = (-(frac + 1.0) * frac * (frac - 2.0)) * 0.5;
  const double c3 = ((frac + 1.0) * frac * (frac - 1.0)) * (1.0 / 6.0);
  return ym1 * c0 + y0 * c1 + y1 * c2 + y2 * c3;
}

// The same four weights in double precision, one weight at a time. A delay
// line holding a single non-zero sample must return exactly the weight of the
// tap that sample sits on.
double weight(int tap, double frac) {
  switch (tap) {
    case 0: return (-frac * (frac - 1.0) * (frac - 2.0)) * (1.0 / 6.0);
    case 1: return ((frac + 1.0) * (frac - 1.0) * (frac - 2.0)) * 0.5;
    case 2: return (-(frac + 1.0) * frac * (frac - 2.0)) * 0.5;
    default: return ((frac + 1.0) * frac * (frac - 1.0)) * (1.0 / 6.0);
  }
}

// Age a (0 = most recently pushed sample) holds a^3 + 1. A cubic in age is
// reproduced exactly by third-order interpolation wherever the four taps are
// distinct and unclamped.
float cubicContent(int age) { return static_cast<float>(age * age * age + 1); }

Line makeCubicLine() {
  Line line;
  for (int age = static_cast<int>(kCapacity) - 1; age >= 0; --age) {
    line.push(cubicContent(age));
  }
  return line;
}

// A single non-zero sample at a known age; every other sample is zero.
Line makeImpulseLine(size_t age) {
  Line line;
  line.push(1.0f);
  for (size_t i = 0; i < age; ++i) line.push(0.0f);
  return line;
}

// The ring is filled completely and then advanced twice, so the write index
// sits at 2 and the older samples wrap across the end of the buffer.
Line makeWrapLine() {
  Line line;
  for (int i = 0; i < static_cast<int>(kCapacity); ++i) {
    line.push(static_cast<float>(i + 1));
  }
  line.push(100.0f);
  line.push(200.0f);
  return line;
}

// The helper and the DelayLine caller must return the same number.
void checkHelperMatchesCaller(const Line& line, float delaySamples) {
  const Taps taps = tapsFor(line, delaySamples);
  const float viaHelper =
      rpdsp::cubicLagrangeInterpolate(taps.frac, taps.ym1, taps.y0, taps.y1, taps.y2);
  const float viaCaller = line.readCubic(delaySamples);
#ifdef __FAST_MATH__
  check(std::fabs(viaHelper - viaCaller) <= 1e-6f * (1.0f + std::fabs(viaCaller)),
        "helper and caller agree");
#else
  check(bits(viaHelper) == bits(viaCaller), "helper and caller agree bit-for-bit");
#endif
}

// Integer delays return the tapped sample itself: the Lagrange weights are
// exactly 0, 1, 0, 0 there.
void integerPositions() {
  const Line line = makeCubicLine();
  for (size_t delay = 0; delay < kCapacity; ++delay) {
    const float value = line.readCubic(static_cast<float>(delay));
    check(bits(value) == bits(line.read(delay)), "integer delay returns the tapped sample");
    checkHelperMatchesCaller(line, static_cast<float>(delay));
  }
}

// Fractional delays of a cubic signal reproduce that cubic.
void fractionalPositions() {
  const Line line = makeCubicLine();
  for (int whole = 1; whole <= 13; ++whole) {
    for (float frac : {0.125f, 0.25f, 0.5f, 0.75f, 0.875f}) {
      const float delay = static_cast<float>(whole) + frac;
      const double expected = lagrangeAt(frac, cubicContent(whole - 1), cubicContent(whole),
                                         cubicContent(whole + 1), cubicContent(whole + 2));
      const float value = line.readCubic(delay);
      check(std::fabs(value - expected) <= 1e-4 * (1.0 + std::fabs(expected)),
            "fractional delay reproduces the cubic");
      checkHelperMatchesCaller(line, delay);
    }
  }
}

// A single non-zero sample picks out exactly one Lagrange weight.
void impulseWeights() {
  struct Case {
    size_t age;
    float whole;
    int tap;
  };
  const Case cases[] = {{5, 5.0f, 1}, {5, 6.0f, 0}, {7, 6.0f, 2}, {9, 7.0f, 3}};
  for (const Case& c : cases) {
    const Line line = makeImpulseLine(c.age);
    for (float frac : {0.25f, 0.5f, 0.75f}) {
      const float value = line.readCubic(c.whole + frac);
      const double expected = weight(c.tap, frac);
      check(std::fabs(value - expected) <= 1e-6, "impulse returns its Lagrange weight");
      checkHelperMatchesCaller(line, c.whole + frac);
    }
  }
}

// Delays outside the ring clamp to the endpoints.
void clampedEndpoints() {
  const Line line = makeCubicLine();
  check(bits(line.readCubic(-3.5f)) == bits(line.readCubic(0.0f)),
        "negative delay clamps to zero");
  check(bits(line.readCubic(-0.25f)) == bits(line.readCubic(0.0f)),
        "negative delay clamps to zero");
  check(bits(line.readCubic(1.0e6f)) == bits(line.readCubic(15.0f)),
        "delay beyond capacity clamps to the last sample");
  check(bits(line.readCubic(15.75f)) == bits(line.readCubic(15.0f)),
        "fractional delay beyond capacity clamps to the last sample");
  check(bits(line.readCubic(15.0f)) == bits(line.read(15)),
        "clamped top delay returns the oldest sample");
  checkHelperMatchesCaller(line, -3.5f);
  checkHelperMatchesCaller(line, 1.0e6f);
}

// Taps that wrap across the write index read the samples the ring holds.
void wrapAdjacentTaps() {
  const Line line = makeWrapLine();
  check(bits(line.read(0)) == bits(200.0f), "newest sample");
  check(bits(line.read(1)) == bits(100.0f), "second newest sample");
  check(bits(line.read(2)) == bits(16.0f), "oldest wrapped sample");
  check(bits(line.read(15)) == bits(3.0f), "oldest sample in the ring");

  // Delay 2 interpolates the taps at ages 1, 2, 3 and 4, which cross the wrap.
  for (float frac : {0.0f, 0.25f, 0.5f, 0.75f}) {
    const double expected = lagrangeAt(frac, 100.0, 16.0, 15.0, 14.0);
    const float value = line.readCubic(2.0f + frac);
    check(std::fabs(value - expected) <= 1e-4 * (1.0 + std::fabs(expected)),
          "wrap-adjacent taps interpolate the wrapped samples");
    checkHelperMatchesCaller(line, 2.0f + frac);
  }
}

// The reader itself: exact taps, linear interpolation, clamping and reset.
void readerCoverage() {
  Line line = makeCubicLine();
  for (size_t age = 0; age < kCapacity; ++age) {
    check(bits(line.read(age)) == bits(cubicContent(static_cast<int>(age))),
          "read returns the sample of that age");
  }
  check(bits(line.readLinear(0.0f)) == bits(line.read(0)), "linear read at an integer position");
  check(bits(line.readLinear(4.0f)) == bits(line.read(4)), "linear read at an integer position");
  check(bits(line.readLinear(-1.0f)) == bits(line.readLinear(0.0f)),
        "linear read clamps a negative delay");
  check(bits(line.readLinear(20.0f)) == bits(line.read(15)),
        "linear read clamps a delay beyond capacity");
  const float half = line.readLinear(4.5f);
  const double expectedHalf = 0.5 * (static_cast<double>(line.read(4)) + line.read(5));
  check(std::fabs(half - expectedHalf) <= 1e-4 * (1.0 + std::fabs(expectedHalf)),
        "linear read halfway between two samples");
  line.reset();
  check(bits(line.read(0)) == bits(0.0f), "reset clears the ring");
  check(bits(line.readCubic(0.0f)) == bits(0.0f), "reset clears the ring");
}

// The helper is a partition of unity: constant samples stay constant.
void partitionOfUnity() {
  for (float frac : {0.0f, 0.125f, 0.5f, 0.75f, 1.0f}) {
    const float value = rpdsp::cubicLagrangeInterpolate(frac, 1.0f, 1.0f, 1.0f, 1.0f);
    check(std::fabs(value - 1.0f) <= 1e-6f, "helper keeps constant samples constant");
  }
}

// Deterministic sweep written to the optional trace file.
void sweep(const Line& line) {
  for (float delay : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f, 1.5f, 2.25f, 2.5f, 3.125f, 3.75f,
                      4.5f, 5.0f, 6.5f, 7.25f, 9.5f, 12.75f, 14.5f, 15.0f, -1.0f, -0.5f,
                      16.0f, 100.0f}) {
    const float value = line.readCubic(delay);
    check(std::isfinite(value), "cubic read stays finite");
    digest(value);
    checkHelperMatchesCaller(line, delay);
  }
  for (int i = 0; i < 256; ++i) {
    const float delay = std::fmod(static_cast<float>(i) * 0.7919f, 16.0f);
    const float value = line.readCubic(delay);
    check(std::isfinite(value), "cubic read stays finite");
    digest(value);
    checkHelperMatchesCaller(line, delay);
  }
}

} // namespace

int main(int argc, char** argv) {
  check(argc <= 2, "usage: delay_line_test [output-f32-trace]");
  if (argc == 2) {
    trace.open(argv[1], std::ios::binary);
    check(trace.is_open(), "open trace");
  }

  integerPositions();
  fractionalPositions();
  impulseWeights();
  clampedEndpoints();
  wrapAdjacentTaps();
  readerCoverage();
  partitionOfUnity();

  const Line cubic = makeCubicLine();
  const Line wrap = makeWrapLine();
  sweep(cubic);
  sweep(wrap);

  std::printf("PASS delay line; deterministic output hash %016llx\n",
              static_cast<unsigned long long>(hash));
  if (trace.is_open()) {
    trace.close();
    check(!trace.fail(), "close trace");
  }
  return 0;
}

#endif  // !ARDUINO
