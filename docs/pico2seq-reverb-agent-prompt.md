# Pico2Seq reverb integration agent prompt

Copy the text below into the integration agent's conversation.

Implement the DarkReverb integration in IC-Alchemy/Pico2Seq using this plan:
https://github.com/IC-Alchemy/rpdsp/blob/main/docs/pico2seq-reverb-plan.md

Read the complete plan, applicable AGENTS.md files, Pico2Seq/CLAUDE.md and docs/testing.md. Recheck current repository heads, initialize submodules and work on feature branches while preserving existing work. Complete the implementation and available validation, then deliver reviewable PRs.

The dependency prerequisite is already complete: rpdsp PR #3 (reverb optimization) and PR #4 (recipe coefficient APIs) are merged. Pico2Seq PR #101 is merged; verified DeCluttered commit 93bb7a1a7757c7dd776b203703bd21ff340301b9 pins rpdsp 3fba788b234a2665d243a2410ee4c8df9c9780c1. Do not repeat the old 25b3549 cherry-pick. Preserve the prepared phase-distortion and prism APIs when making further library changes.

Implement voices → MasterDelay → DarkReverb → shared master gain → linked stereo compressor → separate PCM16 L/R. Add MasterReverb, a stereo block API, and a compressor that updates one detector/gain envelope per frame and applies the same gain to both channels. Preserve legacy mono callers, mix-zero output, master ramps/mute, bounded rendering and the existing control cadence.

Start with DarkReverb<16384, DarkReverbStorage::Float> at 48 kHz, subject to the full RAM/stack audit; keep a Half candidate at the same capacity. Default mix to zero. Use audio-owned DSP state, lock-free control publication, smoothed dry/wet mixing and tested freeze transitions. Keep block scratch off Core 1's 2 KiB stack, avoid hot-path allocation or full-buffer clearing, and inspect actual linked hot-code placement in SRAM.

Implement a dedicated Reverb editor without disrupting current Utility controls, and versioned persistence with v1/v2 migration, validated values and freeze off on restore. Follow the plan's defaults and tail/reset behavior; keep the tank evolving at mix zero.

Capture fresh tests before editing. Previous PRs report 21 focused recipe/optimization cases passing, 34 pre-existing failures out of 609 CTest cases, and a Linux GCC 13.3 audio-I2S stub compile failure from designated-initializer order. Identify existing failures explicitly and make relevant stereo/PCM checks runnable. Add meaningful regression coverage for delay feeding reverb, stereo linking, bypass, irregular blocks, controls/freeze/mute, persistence and audio-path allocation.

Build firmware at the documented 225 MHz with -O3 -ffast-math when the toolchain is available; retain ELF/map/build options and audit RAM, setup allocations and stack use. Complete available software work if hardware is unavailable, and leave exact commands for same-clock Half/Float/bypass tests with heavy four-voice loads and the 5,333.33 us buffer deadline. Distinguish measured results, estimates and pending board checks.

Finish with PR links, exact compatible commit pins, test results versus baseline, build/artifact details, UI bindings and remaining hardware checks. Update the plan checklist and affected documentation.
