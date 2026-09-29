# DarkReverb after Pico2Seq's master delay

Plan based on `rpdsp` main at `3a2d89a011133ddda4a25d8c47051bc7cdb7ca16`
and `Pico2Seq` DeCluttered at `d0c61fd80f4b79c6de9616880373aad74cdb5871`.
This document proposes integration; it does not change Pico2Seq firmware.

## Storage decision

Start with `rpdsp::DarkReverb<16384, rpdsp::DarkReverbStorage::Float>` at
48 kHz, subject to a complete RAM and timing check. Keep the library's default
Half storage for callers that need its smaller buffer.

Both variants already calculate in `float`. The template selects **delay-buffer
storage**, not the arithmetic type. RP2350's Arm cores have hardware
single-precision floating point ([Raspberry Pi specification](https://www.raspberrypi.com/products/rp2350/)).

| Same 16,384-sample capacity | Half | Float |
| --- | ---: | ---: |
| Delay-buffer bytes | 32,768 | 65,536 |
| Object bytes, measured with host GCC 13.3 | 33,016 | 65,784 |
| Stored significand precision | 11 bits | 24 bits |
| Half/single conversions on buffer access | Required | None |
| Original author's M33 static instruction count per stereo frame | 269 | 232 |

The 37-instruction difference is about **13.8% fewer static instructions for
the reverb**, not a measured CPU saving and not 13.8% of the entire synth.
Those counts are inherited from the original reverb's comments/commit, not
reproduced in this change. Hardware cycles, XIP/SRAM placement, compiler
flags and live voices can change the result. A host comparison using software
Half conversion cannot predict the M33's hardware-conversion performance.

Float costs exactly 32 KiB more at this capacity and avoids repeated binary16
rounding in the feedback network. The strongest reason to keep Half is RAM:
`MasterDelay` alone owns 144,016 bytes of full-rate float storage and 65,536
bytes of synced int16 storage, before its state. The previous firmware's
reported global-RAM figure is not a current free-memory measurement and does
not account for every setup-time heap allocation, stack or allocator overhead.
Measure the complete running system before selecting Float permanently.

Do not halve Capacity just to make Float fit: that also changes delay lengths,
mode spacing and the sound. Compare both formats at the same Capacity first.
No fixed-point rewrite is proposed; it would need separate quiet-tail,
long-decay and freeze validation.

## 1. Reconcile the rpdsp dependency

Pico2Seq pins `src/rpdsp` to `25b3549ce446b11ac35c1eb6e3de73e28f0d4c9f`.
That revision contains the recipe coefficient APIs in `DSPFunctions.h` used by
`src/voice/engines/RecipeSources.h`. It diverges from `rpdsp/main`: the common
ancestor is `c8369de`, and main's reverb addition does not include that change.

Before updating the submodule, combine the reverb/optimization branch with
the `25b3549` recipe coefficient commit in rpdsp. The changes currently touch
different DSP headers, but compile and run the Pico2Seq recipe tests to verify
the combination. Pin Pico2Seq to that combined, reviewed commit. Never move
the submodule directly to current main and silently lose the coefficient APIs.

## 2. Add stereo processing to the master bus

Current code in `src/voice/VoiceManager.cpp` does:

`voice sum -> MasterDelay -> master gain -> mono Compressor`

`src/app/AudioEngine.cpp` then converts once to PCM16 and duplicates it into
the left and right I2S slots. The proposed order is:

`voice sum -> MasterDelay -> DarkReverb -> shared master gain -> linked stereo compressor -> PCM16 L/R`

This puts delay repeats into the reverb and keeps master volume and transport
mute downstream of both effects, matching the current mute behavior.

- Add `src/voice/MasterReverb.h` as a thin ownership/control adapter around
  the rpdsp class, and own one instance in `VoiceManager`. Prepare/reset before
  publishing `voicesReady`, at the same sample rate as MasterDelay. Never
  construct a 64 KiB reverb or clear its full buffer in the audio callback.
- Add `VoiceManager::processStereoBlock(float* left, float* right, uint32_t n)`.
  Split the existing master loop into delay, reverb, then gain/compression
  stages while retaining bounded chunking and the existing voice-span rules.
  Keep a documented mono wrapper for current tests/callers; at reverb mix zero
  it must reproduce the old mono bus. Do not advance DSP twice to get two channels.
- Render the delay's mono output once. The existing reverb block overload
  accepts the same pointer for both input channels:
  `reverb.process(postDelay, postDelay, wetLeft, wetRight, count)`.
  Its L+R normalization then receives the intended mono level. Keep the
  post-delay dry samples available for the external smoothed dry/wet blend.
- Add a linked stereo API to `rpdsp::Compressor`: detect
  `max(abs(left), abs(right))`, update one envelope and one gain smoother per
  frame, and apply that same gain to both channels. For equal L/R inputs it
  should reproduce the existing mono transfer. Two independent compressors
  can move the stereo image; calling the same mono compressor twice advances
  its timing twice per frame. Keep the compressor macro update cadence intact.
- In `AudioEngine.cpp`, use two static 256-float channel buffers and convert
  L/R separately with `AudioSamples::toPcm16`. I2S is already PCM16 stereo;
  the DMA format, pool size and sample rate can remain as configured.
- Keep all full-block scratch in members/static storage. Reuse the existing
  mono/voice scratch only after the voice mix is complete, with clear lifetime
  boundaries. Two full-size wet arrays cost 2 KiB if no existing storage is
  reused. Core 1 has a 2 KiB stack; the reverb's local chunk arrays and compiler
  spills still count, so inspect the final ARM call chain and interrupt margin.
- Keep emitted render/reverb/compressor hot functions in SRAM using the
  existing `PICO2SEQ_AUDIO_FUNC` convention, and inspect the linked ELF.
  Annotating a wrapper does not guarantee an out-of-line template callee is
  also placed in SRAM.

## 3. Controls and tail behavior

Use audio-owned applied DSP state, with lock-free atomic targets for independent
controls and an SPSC snapshot for coherent multi-parameter preset changes.
Core 0 must never call setters on a live reverb. Consume targets once per block
and only apply changed coefficient controls; `prepare()` is setup-only.

Suggested first defaults are **mix 0**, decay 20 s, damping 3 kHz, low cut
40 Hz, diffusion 0.8, modulation depth 0.5/rate 0.5 Hz, width 1, freeze off.
Mix zero preserves the existing sound on upgrade. Treat these as initial
settings for audition, not measured optimal settings.

The reverb's `setMix` is immediate. For click-free controls, render the wet path
with mix 1 and let `MasterReverb` blend it with the saved post-delay dry bus,
using one shared, per-sample smoothed mix value for L/R. Bound and smooth the
user controls; apply expensive coefficient changes at a bounded control rate.
The existing freeze input gate eases over 20 ms, but freeze also changes damping
and loop gains immediately: test and, if needed, ramp those transitions too.

Keep processing the tank when mix is zero so turning the effect back up reveals
the current tail. Do not add an energy-based tail skip in the first version:
Half tails can settle above exact zero and freeze can hold indefinitely.
Master/transport mute remains downstream and mutes the tail; preserve the
current behavior where effects can continue evolving while transport is muted.
Session load should fade out before any deliberate tank reset, then fade in.

Land DSP and tests before UI changes. Add a dedicated Reverb editor page rather
than reassigning the existing Utility faders (tempo/feedback, delay mix/time,
master volume/macro). The page should expose Mix, Decay, Damping and Freeze
first, with Low cut, Diffusion, Modulation and Width as secondary settings.
Implement routing/display through `ControlSurfaceLogic`, `AlchemyControlBridge`,
`UIState` and OLED code; verify its entry gesture against current bindings.

## 4. Persistence

`SettingsSnapshot` is locked at 24 bytes; the current v2 `ProjectSnapshot` is
12,400 bytes. Neither currently contains reverb settings. Do not insert fields
into either existing on-flash layout or put a float into its reserved word.

Introduce a new version with an appended, fixed-size effect-settings record,
update `SnapshotFormat` decoding/validation and `Session::captureSession` /
`applyAfterVoices`, and account for retained-RAM version/CRC handling too.
Upgrade v1/v2 files to the defaults above. Reject nonfinite/out-of-range effect
fields before publishing targets. Persist settings, not tank contents; restore
freeze off unless an explicit product decision defines freezing an empty tank.

## 5. Acceptance gates and commit order

1. **Combined rpdsp dependency + reverb regression tests.** Run the existing
   Pico2Seq recipe/DSP tests before and after the submodule update. This PR's
   host reverb tests are a prerequisite, not a Pico2Seq build result.
2. **Stereo compressor and master bus.** Add tests beside
   `test_master_bus.cpp`, `test_master_compressor.cpp` and `test_master_delay.cpp`:
   mix-zero legacy PCM, mono-input normalization, delay repeats reaching the
   reverb, distinct wet L/R, shared compressor gain, master ramps/mute,
   zero/odd/oversized blocks, resets and no allocations during rendering.
3. **Controls and persistence.** Test rapid target changes, queue ownership,
   freeze/unfreeze, v1/v2 migration, v3 round trip, invalid values and UI pickup.
   Check impulses, sustained bass/DC, high delay feedback, 0.1/60/1000 s decay,
   modulation endpoints and prolonged freeze; compare Half/Float spectra and
   tail levels as well as peak output. The ring clamp is not a final output limiter.
4. **Firmware build and RAM/stack audit.** Build with
   `scripts/build_pico2seq.ps1 -CpuMHz 225` (the current documented default),
   `-O3 -ffast-math`, keeping ELF/map and build options. Check actual linked code
   placement, global memory, setup-time heap usage, minimum remaining heap and
   stack high-water mark. Do not infer spare RAM from globals alone.
5. **Same-clock hardware A/B.** At 48 kHz/256 frames the deadline is
   5,333.33 us. Compare bypass, Half and Float with identical four-voice heavy
   presets, dense gates/slides, synced and unsynced delay, control motion and
   OLED/LED activity using `scripts/measure_audio_timing.ps1`. Require no
   underrun/TX-stall growth and no sustained over-budget renders; aim for at
   least 20% worst-case render headroom (about 4,267 us) as an engineering
   target. Include freeze and silent long-tail cases. Keep the 150 MHz
   comparison separate if desired; do not use a clock increase to claim a DSP
   speedup. Listening and real DMA/XIP behavior remain board-only checks.

If Float fails the RAM gate, retain Half at the same capacity and measure its
CPU cost. If CPU or memory still fails, use the results to choose the next
optimization; reducing capacity or changing interpolation requires a separate
sound-quality review.
