// Host-only program (see dark_reverb_test.cpp for the Arduino-build guard).
#ifndef ARDUINO

#include <rpdsp/DSPFunctions.h>
#include <rpdsp/oscillator.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {

void check(bool ok, const char* message) {
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

void diffuserRoundsToStartOfRing() {
  float buffer[8] = {0.5f};
  float state[] = {1.0f};
  // All four tiny negative positions round to 8 when the ring length is added.
  const float output = rpdsp::fx_diffuse(1.0f, buffer, 8, 1.0e-10f, 1.0f, state);
  check(std::fabs(output - 1.06f) < 1.0e-6f, "diffuser wraps rounded taps to slot zero");
}

void recipeWriteHeadsWrap() {
  constexpr int length = 7;
  float tape[length] = {}, diffuser[length] = {}, cloud[length] = {};
  float tapeState[4] = {}, diffuserState[1] = {}, cloudState[4] = {};
  std::uint32_t rng = 0;
  for (int i = 0; i < 3 * length; ++i) {
    const float next = static_cast<float>((i + 1) % length);
    const float input = i == 0 ? 1.0f : 0.0f;
    rpdsp::delay_tape(input, tape, length, 2.0f, 0.0f, 0.0f, tapeState);
    rpdsp::fx_diffuse(input, diffuser, length, 1.0f, 1.0f, diffuserState);
    rpdsp::gran_cloud(input, cloud, length, 2.0f, &rng, cloudState);
    check(tapeState[2] == next && diffuserState[0] == next && cloudState[0] == next,
          "recipe write heads wrap on a non-power-of-two buffer");
  }
}

void stationaryHardSyncOscillators() {
  // The binary-exact phase increment isolates the event scheduler from drift.
  constexpr float frequency = 1500.0f;
  rpdsp::BSplineSawOsc freeSaw;
  freeSaw.setFreq(frequency);
  freeSaw.setLeak(1.0f);
  rpdsp::HardSyncSaw sync;
  sync.setMasterFrequency(0.0f);
  sync.setSlaveFrequency(frequency);
  for (int i = 0; i < 1024; ++i) {
    check(std::fabs(sync.process() - freeSaw.process()) < 1.0e-6f,
          "stationary master leaves the slave free-running");
  }

  sync.setMasterFrequency(frequency);
  sync.setSlaveFrequency(0.0f);
  sync.reset(0.0f, 0.25f);
  check(sync.process() == -0.5f, "stationary slave holds its phase before sync");
  for (int i = 0; i < 64; ++i) sync.process();
  check(sync.process() == -1.0f, "active master can reset a stationary slave");
}

}  // namespace

int main() {
  diffuserRoundsToStartOfRing();
  recipeWriteHeadsWrap();
  stationaryHardSyncOscillators();
  std::puts("PASS DSP edge cases");
}

#endif
