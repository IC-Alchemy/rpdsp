# Host checks

Two standalone C++17 programs cover the library additions used by Pico2Seq's
master bus: `dark_reverb_test.cpp` (below) and `compressor_stereo_test.cpp`.

## Linked stereo compressor

`compressor_stereo_test.cpp` checks `Compressor::processStereo()` with the
standard library only. Run from the repository root:

```sh
mkdir -p build
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tests/compressor_stereo_test.cpp -o build/compressor_stereo_test
./build/compressor_stereo_test
c++ -std=c++17 -O3 -ffast-math -Isrc tests/compressor_stereo_test.cpp -o build/compressor_stereo_fast
./build/compressor_stereo_fast
```

It covers equal channels reproducing the mono transfer bit-for-bit (five
settings including the three Pico2Seq macro anchors, at 44.1/48/96 kHz), a
reference built from the public envelope/curve/smoother blocks advanced once
per frame with the louder channel, one shared gain (the channel level ratio is
preserved), why two independent compressors or a doubled mono call are not
equivalent, block partitions of 0..1000 frames, aliased channel pointers,
silence, reset repeatability, sample rates from 8 to 192 kHz and hostile input
magnitudes. Normal IEEE builds compare bit-exactly; fast-math builds use a
1e-5 relative tolerance because reassociation can differ between loops. The
hash printed on success is a deterministic digest of a subset of the output
and is only comparable between identical compiler/flag configurations.

Mutation check used during review (not part of the program): making the
detector ignore the right channel, advancing the smoother twice per frame,
leaving the right channel unscaled, or removing the aliased-pointer guard each
makes this program fail.

# DarkReverb host checks

These standalone C++17 programs need only the standard library. Run from the
repository root:

```sh
mkdir -p build
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tests/dark_reverb_test.cpp -o build/dark_reverb_test
./build/dark_reverb_test
c++ -std=c++17 -O3 -ffast-math -Isrc tests/dark_reverb_test.cpp -o build/dark_reverb_fast
./build/dark_reverb_fast
c++ -std=c++17 -O2 -fstack-usage -Isrc tests/dark_reverb_benchmark.cpp -o build/dark_reverb_benchmark
./build/dark_reverb_benchmark
```

`dark_reverb_test` covers Half/Float, capacities 4096/16384/65536, sample rates
8/44.1/48/96 kHz, irregular and zero-length blocks, odd pending samples,
in-place processing, mixed scalar/block calls, all controls including repeated
targets and freeze, reset/reprepare and an eight-second tail smoke check.
Normal IEEE builds require bit-identical scalar/block output. Fast-math builds
permit a small tolerance because reassociation can differ between loops.
The finite-output check uses exponent bits so `-ffinite-math-only` cannot
optimize it away. This is not a T60, spectral or hours-long freeze certification.

To compare an optimization with an older revision, compile **the same test
source** against each revision's headers. Optionally pass a filename to write
the complete deterministic binary32 output trace:

```sh
git worktree add --detach ../rpdsp-before 3a2d89a011133ddda4a25d8c47051bc7cdb7ca16
c++ -std=c++17 -O2 -I../rpdsp-before/src tests/dark_reverb_test.cpp -o build/reverb_before
./build/reverb_before build/before.f32
./build/dark_reverb_test build/after.f32
cmp build/before.f32 build/after.f32
```

The trace contains 11,170,080 channel samples (native-endian float32), including
scalar probes and reprepare checks. Compare normal IEEE and fast-math builds
separately, with the same compiler, target and flags. A hash from another
compiler configuration is not a universal reference response.

The benchmark separates block rendering from depth-control updates. Run old
and new binaries alternately several times and use medians. On non-Arm hosts
Half uses software conversion, while M33 uses VCVTB; host Half/Float speed
ratios are **not RP2350 utilization estimates**. `-fstack-usage` reports each
compiled function's stack frame, not total call-chain or interrupt stack use.

For memory/undefined-behavior checks, build the test with
`-O1 -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer`.
If LeakSanitizer is unavailable in a restricted container, disable leak
detection explicitly and report that limitation separately from ASan/UBSan.

Firmware acceptance still needs an ARM build, linked SRAM/stack checks and
real hardware timing/listening. See the
[integration plan](../docs/pico2seq-reverb-plan.md).

## Optimization results, 2026-09-29

Measured here against `3a2d89a`, using x86-64 GCC 13.3.0 and this test source:

- `-O2 -Wall -Wextra -Werror`: pass; the complete before/after binary traces
  compare byte-for-byte equal across 11,170,080 channel samples.
- `-O3 -ffast-math`: pass; maximum before/after absolute difference
  `2.9802322387695312e-8`, residual RMS relative to baseline RMS `-155.94 dB`
  over the complete trace. This is a numerical regression result, not a
  hardware listening result or a precision guarantee for another compiler.
- AddressSanitizer, UndefinedBehaviorSanitizer and float-cast-overflow checks:
  pass at `-O1`. Leak detection was disabled because this container cannot
  provide LeakSanitizer's process/thread inspection.
- Object size is unchanged: Half 33,016 bytes; Float 65,784 bytes.
- Chunk arrays shrink from 192 to 128 bytes. GCC's host block-function stack
  frame falls from 544 to 480 bytes for Half and 528 to 480 for Float.
  ARM frame sizes and total Core 1 stack use have not been measured here.

Seven alternating host benchmark runs, medians, `-O2`:

| Operation | Before | After |
| --- | ---: | ---: |
| Half rendering, ns/stereo frame | 39.28 | 38.18 |
| Float rendering, ns/stereo frame | 26.51 | 26.31 |
| Half depth update, ns/call | 7.24 | 1.81 |
| Float depth update, ns/call | 7.49 | 1.98 |

The small render-time differences do not establish a robust speedup in this
shared host environment. The concrete changes are less scratch storage and
removing two `sin()` evaluations from depth changes. The broader unchanged-
parameter cache tried during development was discarded after fast-math
comparison showed a larger residual. The published patch keeps the smaller
change and leaves the Half default, tank topology, Capacity and public API intact.
