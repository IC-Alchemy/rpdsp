// world.h — the one interface a Garden sketch implements
// ---------------------------------------------------------------------------
// A "world" is a self-contained instrument or effect rendered one stereo
// frame (or one block) at a time. The host — GardenEngine, or a bespoke
// dispatcher — owns all hardware: I2S, sliders, buttons, OLED, the master
// bus. A world owns only DSP.
//
// This is the canonical, library-level version of the per-example IWorld
// that used to be copy-pasted into every Examples/*/World.h. New sketches
// should derive from rpdsp::World and let GardenEngine host it.
//
// Threading contract (see Docs/realtime_rules.md):
//   Every method below is called from Core 0, inside the audio path, EXCEPT
//   prepare(), which is called once from Core 0 setup() before audio starts.
//   No method may block, allocate, or touch Serial. Parameters arrive
//   already smoothed — no synchronization is needed inside a world.
//
// Buffer-splitting contract (which host, which guarantee):
//   A host that places sequencer events at exact sample offsets renders each
//   buffer as several sub-blocks broken at those offsets. Under such a host
//   processBlock() and onButtonBTap() are called ONE OR MORE TIMES PER BUFFER,
//   on sub-block segments, at exact sample offsets — not once per buffer.
//   EncoderGardenEngine / GardenAudioCore (see GardenSequencer.h) is such a
//   host. The DspFn*Garden sketches keep their own single-block dispatchers and
//   still call each method exactly once per buffer.
//
//   A conforming world does not notice the difference: a sub-block is
//   indistinguishable from a full one, since numFrames is a parameter and no
//   state is keyed to buffer boundaries. Two consequences worth stating out
//   loud, because the next person will otherwise assume otherwise:
//     * Do not assume "once per buffer" for rate-limiting anything. A control
//       update that must happen once per buffer should be idempotent and
//       self-guarding (compare-and-early-return), not counted.
//     * Work done in onButtonBTap() now lands *inside* a buffer rather than
//       between buffers, so an expensive trigger action eats into that buffer's
//       deadline. Clearing arrays and reseeding an RNG is fine; anything heavier
//       will show up on the host's callback budget meter.

#pragma once

#include <cstddef>

namespace rpdsp {

class World {
 public:
  // Eight normalized [0,1] parameters — one per physical slider. Identical
  // across every Garden sketch (hardware contract: 8 sliders via CD4051).
  static constexpr std::size_t kNumParams = 8;

  virtual ~World() = default;

  /**
   * Called once at setup time with the audio sample rate, before any audio
   * is rendered. prepare() every rpdsp module you own here.
   */
  virtual void prepare(float sampleRate) = 0;

  /**
   * Called each time this world becomes the active one (initial boot and
   * every Button A world switch). Clear envelopes, delay lines, filter
   * state, phase accumulators, RNG — the world must start from silence,
   * not from whatever the previous visit left behind.
   */
  virtual void onEnter() = 0;

  /**
   * Momentary per-world action (reseed / mute / retrig / panic — whatever the
   * world defines). Fired once per Button B short-press, and once per sequencer
   * gate under a host that has one.
   *
   * Called from the audio path on Core 0, one or more times per audio buffer,
   * at exact sample offsets (see the buffer-splitting contract above). A manual
   * button tap arrives at offset 0; a sequenced gate arrives at its true offset;
   * a manual tap landing on a sequenced beat is one call, not two.
   *
   * Default no-op so worlds without a tap action simply omit the override.
   */
  virtual void onButtonBTap() {}

  /**
   * Render one stereo frame. p[] are the eight smoothed [0,1] parameters
   * for THIS frame. Write the frame into leftOut / rightOut; the host
   * applies the master bus (glue compressor + soft-clip) afterwards.
   *
   * Override either this or processBlock() — per-frame is the simple path.
   */
  virtual void processFrame(const float p[kNumParams],
                            float& leftOut, float& rightOut) = 0;

  /**
   * Render a whole block. Advanced path: override when the algorithm wants
   * block-level structure (SIMD-ish loops, hoisted coefficient updates).
   *
   * params holds numFrames consecutive frames, frame-major:
   * frame i's parameters are params[i * kNumParams + 0 .. + kNumParams-1],
   * each frame individually smoothed by the host. left/right each hold
   * numFrames samples to be written in full.
   *
   * The default implementation just calls processFrame() per frame, so
   * simple worlds never see this method.
   *
   * Under a sample-accurate host this is called once per sub-block rather than
   * once per buffer, with numFrames equal to the sub-block length and
   * left/right already offset into the buffer. See the contract at the top.
   */
  virtual void processBlock(const float* params,
                            float* left, float* right,
                            std::size_t numFrames) {
    for (std::size_t i = 0; i < numFrames; ++i) {
      processFrame(&params[i * kNumParams], left[i], right[i]);
    }
  }
};

}  // namespace rpdsp
