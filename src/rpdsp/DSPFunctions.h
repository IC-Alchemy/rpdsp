// DSPFunctions.h
// ---------------------------------------------------------------------------
// A collection of self-contained DSP "recipe" free functions: oscillators,
// filters, resonators, physical models, delays, modulators, dynamics and
// modulation sources. Each takes a caller-owned float* state array
// (zero-initialised) plus raw per-sample inputs; inc = freq/fs,
// w = 2*pi*f/fs. State advances by side effect on the state pointer.
//
// These are free functions, NOT prepare()/process() classes — they are the
// canonical example of the "caller-owned float* state" utility module noted
// in the "Modules that don't follow prepare()->process(float)" section of
// README.md.
//
// RNG idiom: lfo_randcubic / cv_wander / gran_cloud / lfo_hesitate take a
// caller-owned uint32_t* LCG. Zero is a valid seed because the recurrence
// includes a non-zero additive term. Use distinct seeds for distinct streams.
// All pointers must address the documented storage; use separate state per
// instance, zero-init before use, and reset buffer/state together when resizing.
// Inputs/controls must be finite. Audio is nominally +/-1; resonators and
// additive wet/dry mixes can exceed unity. Smooth controls outside these recipes.
//
// Denormal guards (zapDenormal) are applied to recursion/feedback state so
// long tails on silence don't trip host slow-path denormal handling (matches
// dynamics.h / filter.h / ladder.h). Clamping is specified per function;
// remaining soft ranges are caller preconditions. Coefficient factories run
// at setup/control rate; legacy timing defaults target 48 kHz. These recipes
// do not oversample: nonlinear shaping and modulation can still alias.

#pragma once

#include <cmath>
#include <cstdint>

#include "algorithm.h"
#include "realtime.h"

namespace rpdsp {

// Preserve a one-pole's time constant when moving from 48 kHz to another
// sample rate. Setup/control rate only (log/exp); endpoints remain exact.
inline float recipe_rate_at_sample_rate(float rateAt48k, float sampleRate) {
    const float a = clamp01(rateAt48k);
    const float fs = safeSampleRate(sampleRate);
    if (a == 0.0f || a == 1.0f || fs == 48000.0f) return a;
    return -expm1f(log1pf(-a) * (48000.0f / fs));
}

// One-pole update coefficients, all in [0,1]. Defaults retain the 48 kHz law.
struct FeedbackCompressorCoefficients {
    float detector = 0.0005f;
    float attack = 0.05f;
    float releaseFast = 0.004f;
    float releaseSlow = 0.001f;
};

inline FeedbackCompressorCoefficients make_comp_feedback_coefficients(float sampleRate) {
    return {recipe_rate_at_sample_rate(0.0005f, sampleRate),
            recipe_rate_at_sample_rate(0.05f, sampleRate),
            recipe_rate_at_sample_rate(0.004f, sampleRate),
            recipe_rate_at_sample_rate(0.001f, sampleRate)};
}

// Feedback-topology compressor: detector listens to its own output like
// vintage FET units, self-softening the knee. Release speeds up ~4x on
// transient-only material (program-dependent). thresh/amount clamped >=0.
// Zero-init state[2]: gain reduction, sustained output envelope.
inline float comp_feedback(float x, float thresh, float amount,
                           const FeedbackCompressorCoefficients& coeff, float* state) {
    thresh = fmaxf(thresh, 0.0f);
    amount = fmaxf(amount, 0.0f);
    float y = x * (1.0f - state[0]);                   // state[0] = gain reduction
    float a = fabsf(y);
    float over = fmaxf(a - thresh, 0.0f) * amount;
    float tgt = over / (1.0f + over);                  // saturating GR target
    state[1] += coeff.detector * (a - state[1]);
    float sus = fminf(state[1] / (thresh + 1e-9f), 1.0f);
    float rate = (tgt > state[0]) ? coeff.attack
                 : coeff.releaseFast + (coeff.releaseSlow - coeff.releaseFast) * sus;
    state[0] += rate * (tgt - state[0]);
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    return y;
}

inline float comp_feedback(float x, float thresh, float amount, float* state) {
    return comp_feedback(x, thresh, amount, FeedbackCompressorCoefficients{}, state);
}


// FILTERS
//
// Chamberlin SVF with soft-clipped resonance path only: resonance
// compresses musically instead of screaming, passband stays clean —
// mellow acid filter. w = 2*pi*fc/fs, clamped 0..0.89; res clamped 0..1.
// Zero-init state[2]: bandpass, lowpass integrators.
inline float filt_diodesvf(float x, float w, float res, float* state) {
    w = clamp(w, 0.0f, 0.89f);                          // hard precondition: w < 0.9
    res = clamp01(res);
    float lp = state[1] + w * state[0];
    float sat = fastTanh(state[0]);
    float hp = x - lp - (2.0f - 2.0f * res) * sat;
    state[0] += w * hp;
    state[1] = lp;
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    return lp;
}


// Chamberlin integration coefficients. Factory limits them to [0,0.89].
struct VowelFilterCoefficients { float first = 0.105f, second = 0.151f; };

// Setup/control rate: vowel clamped 0..4, including the exact 'u' endpoint.
// Scale the original 48 kHz tuning; the Chamberlin tuning is approximate.
inline VowelFilterCoefficients make_filt_vowel_coefficients(float vowel, float sampleRate = 48000.0f) {
    static const float w1[5] = {0.105f, 0.052f, 0.046f, 0.059f, 0.043f};
    static const float w2[5] = {0.151f, 0.209f, 0.223f, 0.105f, 0.092f};
    const float v = clamp(vowel, 0.0f, 4.0f);
    const int i = (v >= 4.0f) ? 3 : (int)v;
    const float f = v - (float)i;
    const float scale = 48000.0f / safeSampleRate(sampleRate);
    return {clamp((w1[i] + f * (w1[i + 1] - w1[i])) * scale, 0.0f, 0.89f),
            clamp((w2[i] + f * (w2[i + 1] - w2[i])) * scale, 0.0f, 0.89f)};
}

// Vowel formant filter: two bandpasses morphing through a-e-i-o-u.
// Zero-init state[4]: first BP/LP, second BP/LP integrators.
inline float filt_vowel(float x, const VowelFilterCoefficients& coeff, float* state) {
    const float a = coeff.first, b = coeff.second;
    float lp0 = state[1] + a * state[0];
    state[0] += a * (x - lp0 - 0.08f * state[0]);
    state[1] = lp0;
    float lp1 = state[3] + b * state[2];
    state[2] += b * (x - lp1 - 0.08f * state[2]);
    state[3] = lp1;
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    state[2] = zapDenormal(state[2]);
    state[3] = zapDenormal(state[3]);
    return state[0] + 0.7f * state[2];
}

inline float filt_vowel(float x, float vowel, float* state) {
    return filt_vowel(x, make_filt_vowel_coefficients(vowel), state);
}


///  Oscillators

// Phase-Distortion Morph Oscillator

// Phase-distortion morph oscillator: shape clamped 0..1, sine to bright
// sync-like spectra. The warped sine has slope corners and can alias.
// inc = signed freq/fs, clamped +/-0.5. Zero-init state[1]: phase.
inline float osc_pdmorph(float inc, float shape, float* state) {
    float p = wrap01(state[0] + clamp(inc, -0.5f, 0.5f));
    state[0] = p;
    shape = clamp01(shape);
    float k = 0.5f - shape * 0.49f;                    // knee: fast half / slow half
    float w = (p < k) ? p * (0.5f / k) : 0.5f + (p - k) * (0.5f / (1.0f - k));
    return sinNormalizedPhase(wrap01(w));
}


 // Feedback-FM Operator

// Feedback-FM operator with 2-sample averaged feedback to soften rapid
// feedback variation; high indices can still be chaotic and alias.
// mod/fbk are in cycles (try +/-1). inc clamped +/-0.5 cycles/sample.
// Zero-init state[3]: phase, last output, preceding output.
inline float osc_fbfm(float inc, float fbk, float mod, float* state) {
    float p = wrap01(state[0] + clamp(inc, -0.5f, 0.5f));
    state[0] = p;
    float ph = p + mod + fbk * 0.5f * (state[1] + state[2]);
    float y = sinNormalizedPhase(wrap01(ph));
    state[2] = state[1]; state[1] = y;
    return y;
}


// Chaotic Drift Oscillator

// Pitched chaotic oscillator: sine core, detuned each cycle by a logistic
// map, with turbulent shimmer. chaos clamped 0..1 (3.6..4.0 map regime).
// inc = signed freq/fs, clamped +/-0.5. Zero-init state[2]: phase, map value.
inline float osc_chaosdrift(float inc, float chaos, float* state) {
    inc = clamp(inc, -0.5f, 0.5f);
    chaos = clamp01(chaos);
    float p = state[0] + inc * (1.0f + 0.06f * chaos * (state[1] - 0.5f));
    if (p >= 1.0f || p < 0.0f) {                      // either direction crosses a cycle
        float z = state[1] < 1e-6f ? 0.618f : state[1];
        state[1] = (3.6f + 0.4f * chaos) * z * (1.0f - z) * 0.99f;
    }
    state[0] = wrap01(p);
    return sinNormalizedPhase(state[0]);
}


 //Morphing Tri/Square Oscillator

// Morphing oscillator: skew bends triangle to saw, drive squares it up —
// two continuous morph axes from one core. Output renormalized so drive
// doesn't pump level. Alias rises with drive; keep drive < 4 up high.
// inc clamped +/-0.5 cycles/sample, skew clamped +/-1, drive clamped >=0.
// Zero-init state[1]: phase.
inline float osc_morphtsq(float inc, float skew, float drive, float* state) {
    float p = wrap01(state[0] + clamp(inc, -0.5f, 0.5f)); state[0] = p;
    skew = clamp(skew, -1.0f, 1.0f);
    float k = 0.5f + 0.49f * skew;
    float tri = (p < k) ? p / k : (1.0f - p) / (1.0f - k);
    float d = 1.0f + fmaxf(drive, 0.0f);
    return fastTanh((2.0f * tri - 1.0f) * d) / fastTanh(d);
}


// Through-Zero FM Sine

// Through-zero linear FM sine with pitch servo: a slow DC servo removes
// the average detune that heavy asymmetric FM causes, so the note stays
// centered while the timbre goes wild. fm in +/- units of inc; inc clamped
// +/-0.5. Servo coefficient 0.0002 per sample (48 kHz default timing).
// Zero-init state[2]: phase, FM DC estimate.
inline float osc_tzfm(float inc, float fm, float* state) {
    inc = clamp(inc, -0.5f, 0.5f);
    float raw = inc * (1.0f + fm);
    state[1] += 0.0002f * (raw - state[1] - inc);      // tracks DC of the FM
    state[1] = zapDenormal(state[1]);
    float p = wrap01(state[0] + raw - state[1]);
    state[0] = p;
    return sinNormalizedPhase(p);
}


/// DSF Oscillator


// DSF oscillator (closed-form summation): a partial series from 3 sines.
// bright clamped 0..0.94 = spectral rolloff; non-integer ratio gives
// inharmonic spectra. ratio is signed partial spacing (try 0..16).
// The infinite series is not band-limited. inc clamped +/-0.5 cycles/sample.
// Zero-init state[2]: fundamental phase, partial-spacing phase.
inline float osc_dsf(float inc, float ratio, float bright, float* state) {
    inc = clamp(inc, -0.5f, 0.5f);
    state[0] = wrap01(state[0] + inc);
    state[1] = wrap01(state[1] + inc * ratio);
    float a = clamp(bright, 0.0f, 0.94f);
    float num = sinNormalizedPhase(state[0]) - a * sinNormalizedPhase(wrap01(state[0] - state[1]));
    float den = 1.0f + a * a - 2.0f * a * sinNormalizedPhase(wrap01(state[1] + 0.25f));
    return num * (1.0f - a) / den;
}


/// Formant Grain Oscillator (FOF-lite)


// Formant grain oscillator (FOF-lite): sine burst with exp decay + fast
// attack, retriggered every fundamental period — pitch and formant are
// independent knobs; instant vowel/brass tones. inc = f0/fs and
// finc = formant/fs, both signed and clamped +/-0.5. decay clamped 0..1
// (try 0.995..0.9995 at 48 kHz); fast attack retention is 0.8 per sample.
// Zero-init state[4]: fundamental phase, carrier phase, decay, attack tail.
inline float osc_formant(float inc, float finc, float decay, float* state) {
    inc = clamp(inc, -0.5f, 0.5f);
    finc = clamp(finc, -0.5f, 0.5f);
    decay = clamp01(decay);
    float p = state[0] + inc;
    float rt = (float)(p >= 1.0f || p < 0.0f);
    state[0] = wrap01(p);
    state[1] = wrap01(state[1] * (1.0f - rt) + finc);  // bounded even when f0 is zero
    state[2] = state[2] * decay * (1.0f - rt) + rt;    // exp decay window
    state[3] = state[3] * 0.8f * (1.0f - rt) + rt;     // fast attack ramp
    state[2] = zapDenormal(state[2]);
    state[3] = zapDenormal(state[3]);
    return sinNormalizedPhase(state[1]) * state[2] * (1.0f - state[3]);
}


///Reversing Hard Sync


// Reversing hard sync: at master wrap the slave *reverses direction*
// instead of resetting. Reversals are sample-quantized, and slope changes
// still alias. minc = signed master freq/fs, clamped +/-0.5; ratio is the
// signed slave/master ratio (try 0..16).
// Zero-init state[3]: master phase, slave phase, direction (0 starts forward).
inline float osc_revsync(float minc, float ratio, float* state) {
    minc = clamp(minc, -0.5f, 0.5f);
    float dir = (state[2] == 0.0f) ? 1.0f : state[2];
    float mp = state[0] + minc;
    if (mp >= 1.0f || mp < 0.0f) dir = -dir;
    state[0] = wrap01(mp); state[2] = dir;
    float sp = state[1] + minc * ratio * dir;
    sp = wrap01(sp);
    state[1] = sp;
    return sinNormalizedPhase(sp);
}



// Spectral Prism Oscillator

// Six-partial oscillator with a movable spectral focus: focus 0..1 moves
// from the fundamental to harmonic 6; spread 0..1 widens the squared tent
// around that focus. Narrow focus isolates a partial, wide focus blends
// neighboring harmonics. inc = signed f0/fs, clamped to +/-0.5.
// Each partial fades over 0.45..0.5 cycles/sample before being omitted;
// fast control modulation can still alias. Output nominally +/-1.
// Zero-init state[1]: fundamental phase. Advance before rendering, as above.
inline float osc_prism(float inc, float focus, float spread, float* state) {
    inc = clamp(inc, -0.5f, 0.5f);
    const float center = 1.0f + 5.0f * clamp01(focus);
    const float width = 1.0f + 5.0f * clamp01(spread);
    state[0] = wrap01(state[0] + inc);
    float out = 0.0f, norm = 0.0f;
    for (int h = 1; h <= 6; ++h) {
        const float tent = fmaxf(0.0f, 1.0f - fabsf((float)h - center) / width);
        const float weight = tent * tent;
        const float fade = clamp01((0.5f - fabsf(inc) * (float)h) * 20.0f);
        norm += weight;                              // retain the Nyquist fade
        if (weight * fade > 0.0f) {
            out += weight * fade * sinNormalizedPhase(wrap01(state[0] * (float)h));
        }
    }
    return out / norm;                               // norm >= 0.5 for these bounds
}


// Resonators & Physical Modeling

// Braided Modal Resonator

// Two rotating modes exchange energy through an orthogonal scattering
// junction: an impulse blooms into beating, split resonances. g1/g2 are
// tan(pi*f/fs), clamped to 0..1 (isolated modes up to fs/4); calculate at
// control rate. couple is a signed half-angle tangent, clamped +/-0.1;
// zero decouples the modes. Coupling changes the combined modal pitches.
// loss 0.00001..1 removes that fraction of amplitude per sample; for T60
// use loss = 1 - exp(log(0.001)/(seconds*fs)), within the supported range.
// The rotations preserve energy before loss, even with changing controls.
// Zero-init state[4]: mode A real/imaginary, mode B real/imaginary.
// Feed small impulses/noise; sustained resonant input can exceed unity.
inline float res_braid(float x, float g1, float g2, float loss, float couple, float* state) {
    g1 = clamp(g1, 0.0f, 1.0f);
    g2 = clamp(g2, 0.0f, 1.0f);
    couple = clamp(couple, -0.1f, 0.1f);
    const float keep = 1.0f - clamp(loss, 0.00001f, 1.0f);
    const float inv = 1.0f / (1.0f + couple * couple);
    const float c = (1.0f - couple * couple) * inv;
    const float s = 2.0f * couple * inv;
    const float ar = state[0] + x, ai = state[1];
    const float br = state[2], bi = state[3];
    const float re[2] = {c * ar - s * br, s * ar + c * br};
    const float im[2] = {c * ai - s * bi, s * ai + c * bi};
    const float g[2] = {g1, g2};
    for (int i = 0; i < 2; ++i) {
        const float d = 1.0f / (1.0f + g[i] * g[i]);
        const float rc = (1.0f - g[i] * g[i]) * d;
        const float rs = 2.0f * g[i] * d;
        state[2 * i] = zapDenormal(keep * (rc * re[i] - rs * im[i]));
        state[2 * i + 1] = zapDenormal(keep * (rs * re[i] + rc * im[i]));
    }
    return 0.70710678f * (state[0] + state[2]);
}

// Tension Modal Resonator


// Modal string resonator with tension nonlinearity: pitch sharpens as it
// rings louder, like a plucked string. Feed impulses/noise. ZDF SVF core.
// w = 2*pi*f0/fs (clamped 0..1.19), damp 0.001..0.05, stretch 0..0.3.
// Zero-init state[3]: BP/LP integrators, amplitude follower (rate 0.002/sample).
inline float res_tension(float x, float w, float damp, float stretch, float* state) {
    w = clamp(w, 0.0f, 1.19f);                          // hard precondition: w < 1.2
    float g = 0.5f * w * (1.0f + stretch * state[2]);
    float bp = (state[0] + g * (x - state[1])) / (1.0f + g * (g + damp));
    float lp = state[1] + g * bp;
    state[0] = 2.0f * bp - state[0];
    state[1] = 2.0f * lp - state[1];
    state[2] += 0.002f * (fabsf(bp) - state[2]);
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    state[2] = zapDenormal(state[2]);
    return bp;
}


// Guitar Feedback Simulator


// Guitar-amp feedback simulator: input envelope slowly opens a
// regeneration path around a high-Q bandpass "string" at w, which blooms
// just past unity and tanh-limits. w = 2*pi*f/fs, 0..1.2; bloom >=0.
// Zero-init state[4]: BP/LP integrators, input envelope (rate 0.001/sample),
// limited feedback. Older callers allocating 5 floats remain compatible.
inline float gtr_feedback(float x, float w, float bloom, float* state) {
    state[2] += 0.001f * (fabsf(x) - state[2]);        // slow onset
    float regen = fminf(state[2] * bloom * 20.0f, 1.02f);
    float g = 0.5f * w;
    float v = 0.2f * x + regen * state[3];
    float bp = (state[0] + g * (v - state[1])) / (1.0f + g * (g + 0.02f));
    float lp = state[1] + g * bp;
    state[0] = 2.0f * bp - state[0];
    state[1] = 2.0f * lp - state[1];
    state[3] = fastTanh(bp);
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    state[2] = zapDenormal(state[2]);
    state[3] = zapDenormal(state[3]);
    return x + state[3];
}


// ---

// Delays & Time FX

// BBD-Style Delay


// BBD-style delay: buffer written at a virtual clock rate, so time
// changes glide in pitch like a real bucket brigade; input one-pole
// darkens with slower clocks like cascaded stage loss.
// clock clamped 0.05..1.0 (delay approximately n/clock samples); |fb| < 1.
// n must be 2..2^24; invalid n returns silence without accessing buf/state.
// Zero-init buf[n] + state[3]: write position, input LP, previous wet output.
inline float delay_bbd(float x, float* buf, int n, float clock, float fb, float* state) {
    if (n < 2 || n > 16777216) return 0.0f;
    clock = clamp(clock, 0.05f, 1.0f);
    state[1] += fminf(clock, 1.0f) * 0.8f * (x + fb * state[2] - state[1]);
    float wp = state[0] + clock;
    wp -= (float)n * (float)(wp >= (float)n);
    int i0 = (int)wp;
    buf[i0] = state[1];
    int r0 = i0 + 1; r0 -= n * (r0 >= n);              // oldest sample
    int r1 = r0 + 1; r1 -= n * (r1 >= n);
    float fr = wp - (float)i0;
    float out = buf[r0] + fr * (buf[r1] - buf[r0]);
    state[0] = wp; state[2] = out;
    state[1] = zapDenormal(state[1]);
    state[2] = zapDenormal(state[2]);
    return out;
}


// Tape Delay


// Normalized LFO increments and a one-pole update coefficient in [0,1].
struct TapeDelayCoefficients {
    float wowInc = 0.8f / 48000.0f;
    float flutterInc = 6.3f / 48000.0f;
    float oxide = 0.35f;
};

inline TapeDelayCoefficients make_delay_tape_coefficients(float sampleRate) {
    const float fs = safeSampleRate(sampleRate);
    return {0.8f / fs, 6.3f / fs, recipe_rate_at_sample_rate(0.35f, fs)};
}

// Tape delay: wow (0.8 Hz) + flutter (6.3 Hz) modulate the read head;
// regeneration path gets oxide-style LP + soft sat. dly clamped 1..n-2
// samples; wow clamped 0..min(dly-1, n-2-dly) so BOTH excursions fit.
// fb typically +/-0.95; larger values saturate the regeneration path.
// n must be 4..2^24; invalid n returns silence without accessing buf/state.
// Zero-init buf[n] + state[4]: wow/flutter phases, write index, oxide LP.
inline float delay_tape(float x, float* buf, int n, float dly, float wow, float fb,
                        const TapeDelayCoefficients& coeff, float* state) {
    if (n < 4 || n > 16777216) return 0.0f;
    dly = clamp(dly, 1.0f, (float)(n - 2));
    wow = clamp(wow, 0.0f, fminf(dly - 1.0f, (float)(n - 2) - dly));
    state[0] = wrap01(state[0] + coeff.wowInc);
    state[1] = wrap01(state[1] + coeff.flutterInc);
    float t0 = 2.0f * state[0]; if (t0 > 1.0f) t0 -= 2.0f;
    float t1 = 2.0f * state[1]; if (t1 > 1.0f) t1 -= 2.0f;
    float mod = wow * (2.8f * t0 * (1.0f - fabsf(t0)) + 1.2f * t1 * (1.0f - fabsf(t1)));
    int wp = (int)state[2];
    float rp = (float)wp - clamp(dly + mod, 1.0f, (float)(n - 2));
    rp += (float)n * (float)(rp < 0.0f);
    if (rp >= (float)n) rp = 0.0f;                  // tiny negative rp can round to n
    int r0 = (int)rp, r1 = r0 + 1; r1 -= n * (r1 >= n);
    float fr = rp - (float)r0;
    float out = buf[r0] + fr * (buf[r1] - buf[r0]);
    state[3] += coeff.oxide * (out - state[3]);
    buf[wp] = x + fastTanh(state[3] * fb);
    state[2] = (float)((wp + 1) % n);
    state[3] = zapDenormal(state[3]);
    return out;
}

inline float delay_tape(float x, float* buf, int n, float dly, float wow, float fb, float* state) {
    return delay_tape(x, buf, n, dly, wow, fb, TapeDelayCoefficients{}, state);
}

// Allpass Swarm


// Regenerative allpass swarm: 3 detuned allpasses in a feedback loop held
// at the edge of oscillation by an energy governor — metallic bloom that
// rings and swells. This is a soft governor, not a hard output limiter.
// color 0..1, regen 0..1.2. Zero-init state[5]: three allpass memories,
// loop output, mean-square energy (follower rate 0.001/sample).
inline float fx_swarm(float x, float color, float regen, float* state) {
    float fb = regen / (1.0f + state[4] * state[4]);   // governor
    float v = x + fb * state[3];
    float a0 = 0.30f + 0.55f * color;
    const float det[3] = { 1.0f, 0.83f, 0.67f };
    for (int i = 0; i < 3; ++i) {
        float a = a0 * det[i];
        float t = v - a * state[i];
        v = state[i] + a * t;
        state[i] = t;
    }
    state[3] = v;
    state[4] += 0.001f * (v * v - state[4]);           // energy sensor
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    state[2] = zapDenormal(state[2]);
    state[3] = zapDenormal(state[3]);
    state[4] = zapDenormal(state[4]);
    return v;
}


// Prime-Tap Diffuser


// Prime-tap diffuser: 4 sign-alternating taps at prime offsets smear
// transients into instant ambience — a reverb impression for 4 buffer
// reads. size clamped 0..min(1,(n-2)/1980); mix 0..1 (additive wet gain).
// n must be 3..2^24; invalid n returns dry input without accessing storage.
// Zero-init buf[n] + state[1]: write index.
inline float fx_diffuse(float x, float* buf, int n, float size, float mix, float* state) {
    if (n < 3 || n > 16777216) return x;
    // Clamp size so the largest prime tap (1979*size + 1) fits inside the
    // ring after a single wrap: hard precondition n > 1980*size + 2.
    const float sizeMax = fminf(1.0f, (float)(n - 2) * (1.0f / 1980.0f));
    size = clamp(size, 0.0f, sizeMax);
    int wp = (int)state[0];
    buf[wp] = x;
    state[0] = (float)((wp + 1) % n);
    static const float off[4] = {241.0f, 563.0f, 1181.0f, 1979.0f};
    static const float gn[4]  = {0.45f, -0.38f, 0.31f, -0.26f};
    float wet = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float rp = (float)wp - off[i] * size - 1.0f;
        rp += (float)n * (float)(rp < 0.0f);
        wet += gn[i] * buf[(int)rp];
    }
    return x + mix * wet;
}


// Grain Cloud Taps


// Grain-cloud tap: 3 slowly wandering read taps over a ring buffer —
// doppler from the tap motion smears any input into a texture cloud with
// zero grain scheduling. spread clamped 0..n-4 samples.
// n must be 4..2^24; invalid n returns silence without accessing storage/RNG.
// Zero-init buf[n] + state[4]: write index, three tap lags (rate 0.0003/sample).
inline float gran_cloud(float x, float* buf, int n, float spread, uint32_t* rng, float* state) {
    if (n < 4 || n > 16777216) return 0.0f;
    // Hard precondition: spread < n-4 so each wandering tap wraps to a
    // valid index after a single wrap.
    spread = clamp(spread, 0.0f, fmaxf(0.0f, (float)(n - 4)));
    int wp = (int)state[0];
    buf[wp] = x;
    state[0] = (float)((wp + 1) % n);
    float out = 0.0f;
    for (int i = 0; i < 3; ++i) {
        uint32_t r = *rng * 1664525u + 1013904223u; *rng = r;
        float tgt = (float)(r >> 8) * (1.0f / 16777216.0f) * spread + 2.0f;
        state[1 + i] += 0.0003f * (tgt - state[1 + i]);
        float rp = (float)wp - state[1 + i];
        rp += (float)n * (float)(rp < 0.0f);
        if (rp >= (float)n) rp = 0.0f;              // tiny startup lag can round to n
        int r0 = (int)rp, r1 = r0 + 1; r1 -= n * (r1 >= n);
        float fr = rp - (float)r0;
        out += buf[r0] + fr * (buf[r1] - buf[r0]);
    }
    return out * 0.577f;                               // ~1/sqrt(3)
}



// Frequency Shifter (SSB)


// Unit carrier rotation. Use the factory at control rate, not per sample.
struct FrequencyShiftCoefficients { float cosine = 1.0f, sine = 0.0f; };

inline FrequencyShiftCoefficients make_fx_freqshift_coefficients(float inc) {
    const float angle = kTwoPi * clamp(inc, -0.5f, 0.5f);
    return {cosf(angle), sinf(angle)};
}

// Frequency shifter: 8-section IIR Hilbert pair + quadrature carrier.
// Positive rotation shifts down; negative shifts up (legacy convention).
// Hilbert rejection
// is approximate and deteriorates near DC/Nyquist; shifted audio can alias.
// Zero-init state[35]: eight [x1,x2,y1,y2] sections, I delay, carrier cos/sin.
// coeff must be a unit rotation, normally generated by the factory above.
inline float fx_freqshift(float x, const FrequencyShiftCoefficients& coeff, float* state) {
    static const float aa[4] = {0.6923878f, 0.93606543f, 0.98822952f, 0.99874885f};
    static const float ab[4] = {0.40219212f, 0.85617109f, 0.97229095f, 0.99528848f};
    float i = x, q = x;
    float* s = state;                                  // 4 floats/section: x1,x2,y1,y2
    for (int k = 0; k < 4; ++k, s += 4) {
        float a2 = aa[k] * aa[k];
        float y = a2 * (i + s[3]) - s[1];
        s[1] = s[0]; s[0] = i; s[3] = s[2]; s[2] = y;
        i = y;
    }
    for (int k = 0; k < 4; ++k, s += 4) {
        float a2 = ab[k] * ab[k];
        float y = a2 * (q + s[3]) - s[1];
        s[1] = s[0]; s[0] = q; s[3] = s[2]; s[2] = y;
        q = y;
    }
    float id = state[32]; state[32] = i;               // 1-sample delay on I path
    float c = state[33], sn = state[34];
    if (c * c + sn * sn < 0.25f) { c = 1.0f; sn = 0.0f; }   // zero-init revive
    float c2 = coeff.cosine * c - coeff.sine * sn;
    float s2 = coeff.sine * c + coeff.cosine * sn;
    float g = 1.5f - 0.5f * (c2 * c2 + s2 * s2);       // cheap renorm
    state[33] = c2 * g; state[34] = s2 * g;
    for (int k = 0; k < 35; ++k) state[k] = zapDenormal(state[k]);
    return id * state[33] - q * state[34];
}

// Compatible audio-rate modulation entry point: inc = signed shift Hz/fs,
// clamped +/-0.5. Shared polynomial sines avoid per-sample libm calls.
// Use the coefficient overload for fixed/control-rate shifts and best tuning.
inline float fx_freqshift(float x, float inc, float* state) {
    const float phase = wrap01(clamp(inc, -0.5f, 0.5f));
    const FrequencyShiftCoefficients coeff{
        sinNormalizedPhase(wrap01(phase + 0.25f)), sinNormalizedPhase(phase)};
    return fx_freqshift(x, coeff, state);
}

// Diode Ring Mod


// Passive-style diode ring mod: both inputs pass through diode deadzones,
// giving the raspy carrier-bleed character of transformer ring mods
// instead of clean multiplication. bias 0..1 opens the deadzone.
inline float ringmod_diode(float x, float carrier, float bias) {
    float d = 0.2f - bias * 0.15f;
    float p = fmaxf(0.5f * x + carrier - d, 0.0f);
    float q = fmaxf(carrier - 0.5f * x - d, 0.0f);
    return 2.0f * (p * p - q * q);
}


// Memory Wavefolder

// Triangle folds around a moving bias derived from recent input history.
// Rising and falling portions follow different curves, giving a reed-like
// rasp whose folds shift with articulation. drive 1..8; memory 0..1 sets
// the moving bias depth; rate 0..1 is its one-pole coefficient (0 freezes).
// For a time constant tau seconds, rate = 1 - exp(-1/(tau*fs)).
// Subtract the folded bias so silence returns to zero; normalize for its
// available headroom. memory=0, drive=1 passes nominal +/-1 input through.
// Output +/-1, naive folding aliases: oversample for bright/high notes.
// Zero-init state[1]: signed input follower. Finite audio inputs expected.
inline float fx_memoryfold(float x, float drive, float memory, float rate, float* state) {
    drive = clamp(drive, 1.0f, 8.0f);
    state[0] = zapDenormal(state[0] + clamp01(rate) * (x - state[0]));
    const float bias = clamp01(memory) * clamp(state[0], -1.0f, 1.0f);
    const float p = wrap01(0.25f * (drive * x + bias + 1.0f));
    const float folded = 1.0f - fabsf(4.0f * p - 2.0f);
    return (folded - bias) / (1.0f + fabsf(bias));
}


// Analog Octave-Down


// Analog-style octave-down: flip-flop toggled on rising zero crossings
// ring-mods the input itself, so the sub tracks envelope automatically
// (naive flip-flop octavers output a fixed-level square). mix 0..1 adds sub.
// Zero-init state[3]: flip-flop, previous input, sub LP (rate 0.15/sample).
inline float pitch_octdown(float x, float mix, float* state) {
    if (state[1] <= 0.0f && x > 0.0f) state[0] = 1.0f - state[0];
    state[1] = x;
    float sub = x * (state[0] > 0.5f ? 1.0f : -1.0f);
    state[2] += 0.15f * (sub - state[2]);              // mellow the sub
    state[2] = zapDenormal(state[2]);
    return x + mix * state[2];
}



// Analog RC ADSR


// RC-style ADSR: attack charges toward 130% then clamps at full scale,
// giving the charging curve of an analog envelope. Rates and sustain
// clamped 0..1 (bigger rate = faster; 0 holds, 1 completes in one sample).
// Gate-off immediately releases, including during attack. Gate rises above
// 0.5 retrigger from the current level; force state[2]=1 while gated to retrigger.
// Zero-init state[3]: level, previous gate, attack flag.
inline float adsr_analog(float gate, float atk, float dec, float sus, float rel, float* state) {
    atk = clamp01(atk); dec = clamp01(dec);
    sus = clamp01(sus); rel = clamp01(rel);
    float lvl = state[0];
    float on = (float)(gate > 0.5f);
    float rose = on * (1.0f - state[1]);
    state[1] = on;
    float atkp = on > 0.5f ? fmaxf(state[2], rose) : 0.0f;
    if (atkp > 0.5f) {
        lvl += atk * (1.3f - lvl);                     // overshoot target
        if (lvl >= 1.0f) { lvl = 1.0f; atkp = 0.0f; }
    } else {
        lvl += (on > 0.5f ? dec : rel) * (on * sus - lvl);
    }
    state[0] = lvl; state[2] = atkp;
    state[0] = zapDenormal(state[0]);
    return state[0];
}


// Looping AD Envelope


// Looping AD envelope: exponential attack aims past full scale (punchy),
// exponential decay; loop > 0.5 retriggers at the floor for LFO-like
// bursts. Rates are one-pole coefficients, clamped 0..1.
// Zero-init state[3]: level, attack flag, previous trigger.
inline float env_loopad(float trig, float atk, float dec, float loop, float* state) {
    atk = clamp01(atk); dec = clamp01(dec);
    if (trig > 0.5f && state[2] < 0.5f) state[1] = 1.0f;
    state[2] = (float)(trig > 0.5f);
    if (state[1] > 0.5f) {
        state[0] += atk * (1.1f - state[0]);
        if (state[0] >= 1.0f) { state[0] = 1.0f; state[1] = 0.0f; }
    } else {
        state[0] -= dec * state[0];
        state[1] = (loop > 0.5f && state[0] < 0.002f) ? 1.0f : state[1];
    }
    state[0] = zapDenormal(state[0]);
    return state[0];
}



// C1-Smooth Random LFO


// C1-smooth random LFO: cubic Hermite segments with random targets AND
// random tangents — wanders like breath; no S&H corners, no filter lag,
// exact segment timing. rate = segment freq/fs, clamped 0..1.
// Random tangents can overshoot +/-1. Zero-init state[5]: phase,
// start/target values, start/target tangents. First segment is silent.
inline float lfo_randcubic(float rate, uint32_t* rng, float* state) {
    rate = clamp01(rate);
    float p = state[0] + rate;
    if (p >= 1.0f) {
        p -= 1.0f;
        uint32_t r = *rng * 1664525u + 1013904223u;
        uint32_t r2 = r * 1664525u + 1013904223u;
        *rng = r2;
        state[1] = state[2]; state[3] = state[4];
        state[2] = (float)(int32_t)r  * 4.6566129e-10f;
        state[4] = (float)(int32_t)r2 * 9.3132257e-10f;   // tangents 2x range
    }
    state[0] = p;
    float p2 = p * p, p3 = p2 * p;
    return state[1] * (2.0f * p3 - 3.0f * p2 + 1.0f) + state[2] * (3.0f * p2 - 2.0f * p3)
         + state[3] * (p3 - 2.0f * p2 + p) + state[4] * (p3 - p2);
}


// Hesitating Random LFO

// A clocked random gesture: rest at the previous target, then glide to a
// correlated new one with zero first/second derivatives at both ends.
// linger 0..0.95 sets the resting fraction of each segment. memory -0.99
// ..0.99: positive clusters targets, negative favors alternating sides,
// zero gives independent targets. Output stays +/-1 without overshoot.
// rate = segment Hz / call rate, clamped 0..1; 0 pauses the phase.
// Smooth changes to rate/linger/memory externally if clicks matter.
// Zero-init state[4]: phase, start, target, initialized flag. Caller-owned
// LCG rng may start at zero; distinct seeds give independent instances.
inline float lfo_hesitate(float rate, float linger, float memory, uint32_t* rng, float* state) {
    const float hold = clamp(linger, 0.0f, 0.95f);
    memory = clamp(memory, -0.99f, 0.99f);
    float p = state[0] + clamp01(rate);
    if (state[3] == 0.0f || p >= 1.0f) {
        if (p >= 1.0f) p -= 1.0f;
        const uint32_t r = *rng * 1664525u + 1013904223u;
        *rng = r;
        const float u = (float)(r >> 8) * (1.0f / 8388608.0f) - 1.0f;
        state[1] = state[2];
        state[2] = memory * state[1] + (1.0f - fabsf(memory)) * u;
        state[3] = 1.0f;
    }
    state[0] = p;
    const float t = clamp01((p - hold) / (1.0f - hold));
    const float ease = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
    return state[1] + ease * (state[2] - state[1]);
}


// Lorenz Chaos Source


// Lorenz chaos source with AGC: output stays ~+/-1 at any rate setting
// instead of the raw attractor's wild scale. Self-starts from zero state.
// rate 0.0001 (slow CV) .. 0.01 (audio growl), explicit Euler integration.
// Zero-init state[4]: attractor x/y/z, amplitude follower (rate 0.001/sample).
// Output is limited to +/-1.5; its level is approximate, especially at startup.
inline float chaos_lorenz(float rate, float* state) {
    float x = state[0] + (float)(state[0] == 0.0f) * 0.01f;
    float y = state[1], z = state[2];
    state[0] = x + rate * 10.0f * (y - x);
    state[1] = y + rate * (x * (28.0f - z) - y);
    state[2] = z + rate * (x * y - 2.6667f * z);
    state[3] += 0.001f * (fabsf(state[0]) - state[3]); // level tracker
    return fmaxf(-1.5f, fminf(1.5f, state[0] / (1.25f * state[3] + 1e-4f)));
}


// Wandering CV


// Wandering CV: random walk with spring-back to center plus rare jump
// events — more musical than S&H or pure drift. Call at control rate.
// LCG is zero-init safe. spring ~0.001, step ~0.01..0.1 per call.
// Zero-init state[1]: current CV. Output clamped +/-1.
inline float cv_wander(float spring, float step, uint32_t* rng, float* state) {
    uint32_t r = *rng * 1664525u + 1013904223u;
    *rng = r;
    float u = (float)(int32_t)r * 4.6566129e-10f;      // -1..1
    float jump = (float)((r >> 8 & 1023u) == 0u);      // ~0.1% chance
    state[0] += step * u * (1.0f + 8.0f * jump) - spring * state[0];
    state[0] = fmaxf(-1.0f, fminf(1.0f, state[0]));
    return state[0];
}




// Hybrid Slew Limiter


// Hybrid slew limiter: linear rate near target (analog glide feel) plus a
// small proportional term on the excess, so huge jumps still land in
// bounded time. rate = non-negative units/sample; zero still allows the
// 0.02/sample proportional catch-up. Zero-init state[1]: current value.
inline float smooth_catchup(float target, float rate, float* state) {
    float d = target - state[0];
    float lin = fmaxf(-rate, fminf(rate, d));
    state[0] += lin + 0.02f * (d - lin);
    return state[0];
}

}  // namespace rpdsp
