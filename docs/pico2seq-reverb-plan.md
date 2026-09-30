# DarkReverb after Pico2Seq's master delay

Status verified against GitHub on 2026-09-30 UTC. Functional baselines are
`rpdsp` main at `3fba788b234a2665d243a2410ee4c8df9c9780c1` and
`Pico2Seq` DeCluttered at `93bb7a1a7757c7dd776b203703bd21ff340301b9`.
Pico2Seq already pinned that rpdsp revision. The integration below was
implemented on 2026-09-30 in [rpdsp PR #5](https://github.com/IC-Alchemy/rpdsp/pull/5)
and [Pico2Seq PR #102](https://github.com/IC-Alchemy/Pico2Seq/pull/102); see
[Implementation record](#7-implementation-record-2026-09-30) for what was measured, what
changed from this plan, and the board checks that remain.

## Verified progress and starting point

- [x] [rpdsp PR #3](https://github.com/IC-Alchemy/rpdsp/pull/3) is merged as
  `a4852786c6aa4ddb3715797f8e00434a4ebecb9a`: scratch reuse, cheaper depth
  updates, reverb regression tests and benchmarks.
- [x] [rpdsp PR #4](https://github.com/IC-Alchemy/rpdsp/pull/4) is merged as
  `3fba788b234a2665d243a2410ee4c8df9c9780c1`: recipe coefficient APIs retained.
- [x] [Pico2Seq PR #101](https://github.com/IC-Alchemy/Pico2Seq/pull/101) is
  merged as `93bb7a1a7757c7dd776b203703bd21ff340301b9`: submodule advanced to
  the combined rpdsp revision.
- [x] Linked stereo compressor and post-delay master reverb (rpdsp PR #5, Pico2Seq PR #102).
- [x] Stereo PCM output, smoothed controls and Reverb editor.
- [x] Versioned persistence and old-project migration (format 3, v1/v2 upgrade).
- [x] Integration regressions and ARM firmware/RAM/stack inspection (software checks; see the
  record below for what is measured and what is an estimate).
- [ ] Physical Pico 2 timing, memory high-water checks and listening (**pending**: needs a board;
  commands are in Pico2Seq `docs/audio-performance.md`).

Start implementation on feature branches from the latest repository heads,
after reading applicable `AGENTS.md`, Pico2Seq's `CLAUDE.md` and
`docs/testing.md`. Recheck the heads and pin before editing; preserve later
changes and existing work. The missing-API prerequisite is complete. Do not
repeat the old cherry-pick just because `25b3549` is not a Git ancestor:
PR #4 brought its API/source changes onto main.

The [copyable agent prompt](pico2seq-reverb-agent-prompt.md) accompanies this plan.

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

**Decision (2026-09-30): Half is the shipping default.** With Float the counted
setup-time allocations exceed the linked heap by 1,724 bytes before allocator overhead;
Half leaves about 30 KB. See the record below and Pico2Seq's audit section.

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

## 1. Verify the existing dependency and establish test baselines

The dependency reconciliation and initial submodule update are finished.
Verify the APIs in the actual pinned `src/rpdsp/src/rpdsp/DSPFunctions.h`:

| Prepared oscillator API | Expected on the current pin |
| --- | --- |
| `PhaseDistortionCoefficients`, `make_osc_pdmorph_coefficients(shape)`, `osc_pdmorph(inc, coefficients, state)` | Present |
| `PrismCoefficients`, `make_osc_prism_coefficients(focus, spread)`, `osc_prism(inc, coefficients, state)` | Present |
| Original float-parameter overloads and `recipe_rate_at_sample_rate(...)` | Preserved |

Initialize the submodule and run the recipe/DSP tests before adding reverb
routing. Run the same checks after each relevant dependency change.
Further library work, such as the linked stereo compressor, belongs on an
rpdsp feature branch. Record its exact commit and update Pico2Seq's pin to
that compatible commit in the integration branch; preserve the coefficient
APIs and source behavior already used by the recipe voices.

Prior validation reported in merged PRs #4/#101, Linux GCC 13.3:

- Focused `[optimization],[recipes]`: **21 cases / 2,786,106 assertions passed**.
- Full host CTest: **34 failures out of 609**, with the identical failures at
  the old `25b3549` pin.
- The audio I2S stub target failed to compile because of designated-initializer
  order on that host; it was excluded from the above CTest run.
- ARM firmware and physical board timing/memory/listening were not checked.

These are the preceding PRs' reported results, not a new run for this plan
update. Capture a fresh baseline, enumerate existing failures and compare
failure identities after integration. Do not label the full suite clean or
treat every later failure as pre-existing. To validate changed stereo output,
make the relevant audio-stub target build on the chosen host (keeping any
portability fix focused and behavior-preserving), or provide an equivalent
testable PCM render path and explicitly report the uncovered driver target.

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

1. **Baseline and dependency verification.** The combined pin is already in
   place. Record fresh recipe/DSP and full-suite baselines, including known
   failures and the audio-stub build status. Run the standalone reverb tests
   with normal and fast-math flags. Existing rpdsp host results do not establish
   a Pico2Seq integration or firmware build result.
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

## 6. Integration-agent deliverables

Implement sections 2–4, then complete the software checks in section 5. Deliver
reviewable, logically grouped PRs in rpdsp and Pico2Seq as needed, with a precise
dependency pin and commit order. Update this checklist and the relevant
architecture/audio/control/persistence documentation with the resulting design.

The handoff report should include:

- Repository branches/commits/PRs, changed files and the exact rpdsp pin.
- Chosen Reverb-page entry gesture and controls; document any necessary deviation.
- Test commands and results, with baseline failures separated from new failures.
- Normal versus fast-math numerical comparisons and regression tolerances.
- Firmware build options and artifact paths; linked SRAM placement, static RAM,
  accounted setup-time allocations and estimated call-chain stack requirements.
- What was actually measured on hardware, and what remains pending.
- Exact commands and scenarios for the remaining board checks.

If no board or firmware toolchain is available, finish the available software
implementation and checks, retain reproducible build/capture commands, and
mark unavailable checks explicitly. Do not substitute host/emulator timing for
board CPU utilization or claim available RAM from global-memory totals alone.

## 7. Implementation record (2026-09-30)

Branches: rpdsp `claude/linked-stereo-compressor` (PR #5) and Pico2Seq
`ccr-9596f6b1-yf2jt1` (PR #102). Pico2Seq pins the rpdsp commit named in its PR.

**What was built**

- rpdsp: `Compressor::processStereo()` (linked detector on `max(|L|,|R|)`, one envelope and
  smoother advance per frame, one gain for both channels; equal L/R reproduces the mono
  compressor bit for bit), the `RPDSP_HOT_FUNCTION` placement hook on both
  `DarkReverb::process()` overloads, and Arduino-safe guards on the host-only test programs.
- Pico2Seq: `MasterReverb` (audio-owned adapter, lock-free targets plus an SPSC snapshot ring,
  eased coefficients, one shared per-sample mix, tank always running, freeze ramp),
  `VoiceManager::processStereoBlock()` with the order voices → delay → reverb → shared master gain →
  linked compressor, separate L/R PCM16 in `AudioEngine`, format-3 persistence with v1/v2
  upgrade, and the Reverb page.
- Reverb page gesture: hold Shift (button 8), hold button 6, press button 2. MAIN layer: faders 1–3 are
  Mix, Decay, Damping, button 1 toggles Freeze; TONE layer (button 2): Low cut, Diffusion, Mod depth,
  Width; Shift exits. Mod **rate** is stored but has no fader (four faders per layer).

**Deviations from this plan**

- Half, not Float, is the default (above); Float stays a build flag.
- `-DPICO2SEQ_REVERB_BYPASS=1` was added as a bench baseline for the CPU A/B, because mix zero
  keeps the tank running by design and so cannot serve as a no-reverb reference.
- The out-of-line `DarkReverb::process` overloads linked into flash even though their caller was in
  SRAM, so a placement hook was added to rpdsp rather than annotating the wrapper.
- Loading a project never resets the tank: the tail keeps evolving and the new settings ease in over
  about 30 ms, instead of the fade-out/reset/fade-in the plan suggested for a deliberate reset.
- The firmware reports `[DIAG MEM]` (heap, heap floor, both stacks) so the board checks below have
  something to read.

**Measured (host GCC 13.3, ARM GCC 16.1)** — see Pico2Seq `docs/testing.md` and
`docs/audio-performance.md` for the numbers and their limits:

- Full host suite: 692 tests, the same 33 failing tests with the same assertion text as the fresh
  baseline at `93bb7a1` (34 failures out of 609, one of them the audio target that did not compile);
  the audio target now builds and passes. Under `-O3 -ffast-math` the failing set is again identical to
  the baseline's. A second build with Float storage (`-DPICO2SEQ_REVERB_STORAGE=FLOAT`) gives the same result.
- Firmware at 225 MHz, `-O3 -ffast-math`, audio in SRAM: Half, Float and bypass all build. Linked heap
  capacity 400,352 B (Half) versus 409,744 B before the reverb. Hot reverb code is in SRAM after the
  hook. Frame sizes and an estimated Core 1 chain of about 1.1 KB of 2 KiB.

**Not measured** — everything that needs the board: CPU time and headroom of Half versus bypass,
the running heap, a stack high-water mark, XIP behavior and listening.
