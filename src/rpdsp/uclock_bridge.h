// uclock_bridge.h — bring uClock up and pump rpdsp::transport from its ISR
// ---------------------------------------------------------------------------
// The three lines of boilerplate that connect the repo's timebase to the repo's
// tick seam, in one place so every host does it identically:
//
//     rpdsp::uclock::begin(48000.0f);          // from the CONTROL core's setup
//     rpdsp::uclock::setTempo(120.0f);         // whenever a knob moves
//
// Call begin() from the control core (setup1() in the dual-core sketches), so
// uClock's hardware timer ISR lands there and never competes with the audio
// callback on Core 0.
//
// This is the ONLY header in rpdsp that includes uClock, and therefore the only
// one that is Arduino-only. Everything else — including transport.h, which the
// worlds use — stays portable and host-testable. Do not include this from a
// world; worlds read rpdsp::transport and know nothing about the clock source.
//
// See the root AGENTS.md ("uClock is the king of time") and
// libraries/uClock/README.md.

#pragma once

#include <Arduino.h>

#include <uClock.h>

#include "transport.h"

#include <cstdint>

namespace rpdsp {
namespace uclock {

// PPQN_384: matches Examples/Arp and ArpMoogFilter, and makes every division
// in transport.h — including triplets and dotted values — an exact integer
// number of ticks. See tests/test_transport.cpp.
inline constexpr std::uint32_t kDefaultPpqn = 384;

// uClock's tempo range. setTempo() clamps silently outside it, so callers that
// derive a tempo from something other than quarter notes should check.
inline constexpr float kMinTempo = 1.0f;
inline constexpr float kMaxTempo = 500.0f;

namespace detail {
// Called by uClock's hardware timer interrupt. This is the whole handler: one
// publish (three aligned stores behind a sequence counter) and return. Anything
// heavier here would both smear the clock (uClock reschedules from handler
// return, not from the tick) and race Core 0, which may be midway through a
// world's processFrame() reading the very state a trigger would clear.
//
// getShuffleLength() is the current step's groove offset in ticks. Publishing it
// alongside the tick means groove templates work everywhere downstream without
// being re-derived — and it costs one store, so the handler stays trivial.
inline void onOutputPpqn(std::uint32_t tick) {
  transport::publish(tick, micros(), uClock.getShuffleLength());
}

// Transport callbacks. External transport (MIDI Start/Stop via clockMe(), or a
// DIN-sync gate) lands here, so transport::State::running tracks the clock
// regardless of who started it. Two stores each; same ISR discipline.
inline void onClockStart() { transport::setRunning(true); }
inline void onClockStop() { transport::setRunning(false); }
}  // namespace detail

// Declared ahead of begin() so it can start the transport through the same path
// every other caller uses, rather than a second copy of uClock.start() plus
// transport::setRunning(true) that could drift out of step with it.
inline void start();
inline void stop();

/**
 * Start the clock. Call from the control core's setup (setup1()).
 *
 * @param sampleRate  audio sample rate, for transport's tick<->sample maths
 * @param bpm         initial tempo in quarter-note BPM
 * @param ppqn        output resolution; leave at kDefaultPpqn unless you have
 *                    a reason, and keep transport::configure() in step with it
 * @param startNow    begin rolling immediately (worlds are silent until then)
 */
inline void begin(float sampleRate,
                  float bpm = 120.0f,
                  std::uint32_t ppqn = kDefaultPpqn,
                  bool startNow = true) {
  transport::configure(ppqn, sampleRate);
  transport::reset();

  uClock.setOutputPPQN(
      static_cast<umodular::clock::uClockClass::PPQNResolution>(ppqn));
  uClock.setOnOutputPPQN(&detail::onOutputPpqn);
  // Keep transport::State::running honest even when something other than this
  // header moves the transport (external MIDI clock, a future DIN-sync input).
  uClock.setOnClockStart(&detail::onClockStart);
  uClock.setOnClockStop(&detail::onClockStop);
  uClock.init();
  uClock.setTempo(bpm);

  if (startNow) start();
}

/**
 * Tempo in quarter-note BPM, clamped into uClock's own range.
 *
 * uClock::setTempo() clamps silently outside 1..500, so a caller deriving a
 * tempo from something other than quarter notes gets a lie rather than an
 * error. Clamping here makes the value uClock holds and the value the UI shows
 * the same number. Check tempoInRange() first if the difference matters.
 */
inline void setTempo(float bpm) {
  const float clamped = bpm < kMinTempo ? kMinTempo
                                        : (bpm > kMaxTempo ? kMaxTempo : bpm);
  uClock.setTempo(clamped);
}

[[nodiscard]] inline float tempo() { return uClock.getTempo(); }

// Transport. Always go through these, never uClock directly: the tempo-synced
// worlds already shipping (AdsrVoice, PluckedGarden) go silent on
// transport::State::running == false, and calling uClock.start() on its own
// leaves that flag stale. The setOnClockStart/Stop callbacks wired in begin()
// make these idempotent rather than double-writing.
inline void start() {
  uClock.start();
  transport::setRunning(true);
}

inline void stop() {
  uClock.stop();
  transport::setRunning(false);
}

/** Stop the clock without rewinding it; resume with start(). */
inline void pause() {
  uClock.pause();
  transport::setRunning(false);
}

[[nodiscard]] inline bool running() { return transport::read().running; }

/** Toggle run/stop. The SEQ page's PREV+NEXT combo calls this. */
inline void toggleRunning() {
  if (running()) {
    stop();
  } else {
    start();
  }
}

/** True if the requested tempo is one uClock will actually deliver. */
[[nodiscard]] inline bool tempoInRange(float bpm) {
  return bpm >= kMinTempo && bpm <= kMaxTempo;
}

}  // namespace uclock
}  // namespace rpdsp
