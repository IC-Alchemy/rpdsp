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
// libraries/rpdsp/README.md.
//
// RNG idiom: lfo_randcubic / cv_wander / gran_cloud take a uint32_t* that is
// a caller-owned, non-zero-seeded LCG (signature unchanged). Seed it once
// before first use; a zero seed stays zero forever.
//
// Denormal guards (zapDenormal) are applied to recursion/feedback state so
// long tails on silence don't trip host slow-path denormal handling (matches
// dynamics.h / filter.h / ladder.h). Only hard preconditions are clamped
// (cutoff frequency w, spectral rolloff bright, delay spread/size); soft
// ranges documented per function are trusted.

#pragma once

#include <cmath>
#include <cstdint>

#include "algorithm.h"
#include "realtime.h"

namespace rpdsp {

// Feedback-topology compressor: detector listens to its own output like
// vintage FET units, self-softening the knee. Release speeds up ~4x on
// transient-only material (program-dependent). Zero-init state[2].
inline float comp_feedback(float x, float thresh, float amount, float* state) {
    float y = x * (1.0f - state[0]);                   // state[0] = gain reduction
    float a = fabsf(y);
    float over = fmaxf(a - thresh, 0.0f) * amount;
    float tgt = over / (1.0f + over);                  // saturating GR target
    state[1] += 0.0005f * (a - state[1]);              // sustain detector
    float sus = fminf(state[1] / (thresh + 1e-9f), 1.0f);
    float rate = (tgt > state[0]) ? 0.05f : 0.004f - 0.003f * sus;
    state[0] += rate * (tgt - state[0]);
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    return y;
}



// FILTERS
//
// Chamberlin SVF with soft-clipped resonance path only: resonance
// compresses musically instead of screaming, passband stays clean —
// mellow acid filter. w = 2*pi*fc/fs (< 0.9), res 0..1. Zero-init state[2].
inline float filt_diodesvf(float x, float w, float res, float* state) {
    w = clamp(w, 0.0f, 0.89f);                          // hard precondition: w < 0.9
    float lp = state[1] + w * state[0];
    float bp = fmaxf(-3.0f, fminf(3.0f, state[0]));
    float sat = bp * (27.0f + bp * bp) / (27.0f + 9.0f * bp * bp);
    float hp = x - lp - (2.0f - 2.0f * res) * sat;
    state[0] += w * hp;
    state[1] = lp;
    state[0] = zapDenormal(state[0]);
    state[1] = zapDenormal(state[1]);
    return lp;
}


// Vowel formant filter: two bandpasses morphing continuously through
// a-e-i-o-u. vowel 0..4. 48 kHz coefficients. Zero-init state[4].
inline float filt_vowel(float x, float vowel, float* state) {
    static const float w1[5] = {0.105f, 0.052f, 0.046f, 0.059f, 0.043f};
    static const float w2[5] = {0.151f, 0.209f, 0.223f, 0.105f, 0.092f};
    float v = fmaxf(0.0f, fminf(3.999f, vowel));
    int i = (int)v; float f = v - (float)i;
    float a = w1[i] + f * (w1[i + 1] - w1[i]);
    float b = w2[i] + f * (w2[i + 1] - w2[i]);
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



///  Oscillators

// Phase-Distortion Morph Oscillator

// Phase-distortion morph oscillator: shape 0 = pure sine, 1 = bright
// sync-like spectrum, continuous in between. Low alias since output is
// always one warped sine cycle. inc = freq/fs. Zero-init state[1].
inline float osc_pdmorph(float inc, float shape, float* state) {
    float p = state[0] + inc;
    p -= (float)(int)p;
    state[0] = p;
    float k = 0.5f - shape * 0.49f;                    // knee: fast half / slow half
    float w = (p < k) ? p * (0.5f / k) : 0.5f + (p - k) * (0.5f / (1.0f - k));
    float t = 2.0f * w; if (t > 1.0f) t -= 2.0f;       // cheap sin(pi*t)
    float y = 4.0f * t * (1.0f - fabsf(t));
    return y * (0.775f + 0.225f * fabsf(y));
}


 // Feedback-FM Operator

// Feedback-FM operator with 2-sample averaged feedback (kills self-FM
// chaos noise at high indices). mod = external phase mod input.
// inc = freq/fs. Zero-init state[3].
inline float osc_fbfm(float inc, float fbk, float mod, float* state) {
    float p = state[0] + inc;
    p -= (float)(int)p;
    state[0] = p;
    float ph = p + mod + fbk * 0.5f * (state[1] + state[2]);
    ph -= floorf(ph);
    float t = 2.0f * ph; if (t > 1.0f) t -= 2.0f;
    float y = 4.0f * t * (1.0f - fabsf(t));
    y *= 0.775f + 0.225f * fabsf(y);
    state[2] = state[1]; state[1] = y;
    return y;
}


// Chaotic Drift Oscillator

// Pitched chaotic oscillator: sine core, detuned each cycle by a logistic
// map — stable pitch center with turbulent shimmer. chaos 0..1
// (3.6..4.0 map regime). inc = freq/fs. Zero-init state[2].
inline float osc_chaosdrift(float inc, float chaos, float* state) {
    float p = state[0] + inc * (1.0f + 0.06f * chaos * (state[1] - 0.5f));
    if (p >= 1.0f) {                                   // once per cycle: advance map
        p -= 1.0f;
        float z = state[1] < 1e-6f ? 0.618f : state[1];
        state[1] = (3.6f + 0.4f * chaos) * z * (1.0f - z) * 0.99f;
    }
    state[0] = p;
    float t = 2.0f * p; if (t > 1.0f) t -= 2.0f;
    float y = 4.0f * t * (1.0f - fabsf(t));
    return y * (0.775f + 0.225f * fabsf(y));
}


 //Morphing Tri/Square Oscillator

// Morphing oscillator: skew bends triangle to saw, drive squares it up —
// two continuous morph axes from one core. Output renormalized so drive
// doesn't pump level. Alias rises with drive; keep drive < 4 up high.
// inc = freq/fs. Zero-init state[1].
inline float osc_morphtsq(float inc, float skew, float drive, float* state) {
    float p = state[0] + inc; p -= (float)(int)p; state[0] = p;
    float k = 0.5f + 0.49f * skew;
    float tri = (p < k) ? p / k : (1.0f - p) / (1.0f - k);
    float d = 1.0f + drive;
    float v = fmaxf(-3.0f, fminf(3.0f, (2.0f * tri - 1.0f) * d));
    float dc = fminf(d, 3.0f);
    float norm = dc * (27.0f + dc * dc) / (27.0f + 9.0f * dc * dc);
    return (v * (27.0f + v * v) / (27.0f + 9.0f * v * v)) / norm;
}


// Through-Zero FM Sine

// Through-zero linear FM sine with pitch servo: a slow DC servo removes
// the average detune that heavy asymmetric FM causes, so the note stays
// centered while the timbre goes wild. fm in +/- units of inc. Zero-init state[2].
inline float osc_tzfm(float inc, float fm, float* state) {
    float raw = inc * (1.0f + fm);
    state[1] += 0.0002f * (raw - state[1] - inc);      // tracks DC of the FM
    float p = state[0] + raw - state[1];
    p -= floorf(p);                                    // handles negative rates
    state[0] = p;
    float t = 2.0f * p; if (t > 1.0f) t -= 2.0f;
    float y = 4.0f * t * (1.0f - fabsf(t));
    return y * (0.775f + 0.225f * fabsf(y));
}


/// DSF Oscillator


// DSF oscillator (closed-form summation): a full partial series from 3
// cheap sines. bright 0..0.9 = spectral rolloff; non-integer ratio gives
// clangorous inharmonic spectra a saw can't. inc = f0/fs. Zero-init state[2].
inline float osc_dsf(float inc, float ratio, float bright, float* state) {
    auto s = [](float p) { p -= floorf(p); float t = 2.0f * p; if (t > 1.0f) t -= 2.0f;
        float y = 4.0f * t * (1.0f - fabsf(t)); return y * (0.775f + 0.225f * fabsf(y)); };
    state[0] += inc;         state[0] -= (float)(int)state[0];
    state[1] += inc * ratio; state[1] -= (float)(int)state[1];
    float a = fminf(bright, 0.94f);
    float num = s(state[0]) - a * s(state[0] - state[1]);
    float den = 1.0f + a * a - 2.0f * a * s(state[1] + 0.25f);
    return num * (1.0f - a) / den;
}


/// Formant Grain Oscillator (FOF-lite)


// Formant grain oscillator (FOF-lite): sine burst with exp decay + fast
// attack, retriggered every fundamental period — pitch and formant are
// independent knobs; instant vowel/brass tones. inc = f0/fs,
// finc = formant/fs, decay ~0.995..0.9995. Zero-init state[4].
inline float osc_formant(float inc, float finc, float decay, float* state) {
    float p = state[0] + inc;
    float rt = (float)(p >= 1.0f);
    state[0] = p - rt;
    state[1] = state[1] * (1.0f - rt) + finc;          // grain carrier phase
    state[2] = state[2] * decay * (1.0f - rt) + rt;    // exp decay window
    state[3] = state[3] * 0.8f * (1.0f - rt) + rt;     // fast attack ramp
    float q = state[1] - floorf(state[1]);
    float t = 2.0f * q; if (t > 1.0f) t -= 2.0f;
    float y = 4.0f * t * (1.0f - fabsf(t));
    state[2] = zapDenormal(state[2]);
    state[3] = zapDenormal(state[3]);
    return y * (0.775f + 0.225f * fabsf(y)) * state[2] * (1.0f - state[3]);
}


///Reversing Hard Sync


// Reversing hard sync: at master wrap the slave *reverses direction*
// instead of resetting — waveform stays continuous, so you get sync
// timbre with a fraction of the alias. minc = master freq/fs.
// Zero-init state[3].
inline float osc_revsync(float minc, float ratio, float* state) {
    float dir = (state[2] == 0.0f) ? 1.0f : state[2];
    float mp = state[0] + minc;
    if (mp >= 1.0f) { mp -= 1.0f; dir = -dir; }
    state[0] = mp; state[2] = dir;
    float sp = state[1] + minc * ratio * dir;
    sp -= floorf(sp);
    state[1] = sp;
    float t = 2.0f * sp; if (t > 1.0f) t -= 2.0f;
    float y = 4.0f * t * (1.0f - fabsf(t));
    return y * (0.775f + 0.225f * fabsf(y));
}



// Resonators & Physical Modeling

// Tension Modal Resonator


// Modal string resonator with tension nonlinearity: pitch sharpens as it
// rings louder, like a plucked string. Feed impulses/noise. ZDF SVF core.
// w = 2*pi*f0/fs (< 1.2), damp ~0.001..0.05, stretch ~0..0.3. Zero-init state[3].
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
// just past unity and tanh-limits — hold a note and it sings. Zero-init state[5].
inline float gtr_feedback(float x, float w, float bloom, float* state) {
    state[2] += 0.001f * (fabsf(x) - state[2]);        // slow onset
    float regen = fminf(state[2] * bloom * 20.0f, 1.02f);
    float g = 0.5f * w;
    float v = 0.2f * x + regen * state[3];
    float bp = (state[0] + g * (v - state[1])) / (1.0f + g * (g + 0.02f));
    float lp = state[1] + g * bp;
    state[0] = 2.0f * bp - state[0];
    state[1] = 2.0f * lp - state[1];
    float c = fmaxf(-3.0f, fminf(3.0f, bp));
    state[3] = c * (27.0f + c * c) / (27.0f + 9.0f * c * c);   // limiter
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
// clock 0.05..1.0 (delay = n/clock samples), zero-init buf + state[3].
inline float delay_bbd(float x, float* buf, int n, float clock, float fb, float* state) {
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
    state[2] = zapDenormal(state[2]);
    return out;
}


// Tape Delay


// Tape delay: wow (0.8 Hz) + flutter (6.3 Hz) modulate the read head;
// regeneration path gets oxide-style LP + soft sat. dly in samples,
// keep wow+3 < dly < n-2. 48 kHz LFO rates. Zero-init buf + state[4].
inline float delay_tape(float x, float* buf, int n, float dly, float wow, float fb, float* state) {
    state[0] += 1.667e-5f;  state[0] -= (float)(int)state[0];
    state[1] += 1.3125e-4f; state[1] -= (float)(int)state[1];
    float t0 = 2.0f * state[0]; if (t0 > 1.0f) t0 -= 2.0f;
    float t1 = 2.0f * state[1]; if (t1 > 1.0f) t1 -= 2.0f;
    float mod = wow * (2.8f * t0 * (1.0f - fabsf(t0)) + 1.2f * t1 * (1.0f - fabsf(t1)));
    int wp = (int)state[2];
    float rp = (float)wp - dly - mod;
    rp += (float)n * (float)(rp < 0.0f);
    int r0 = (int)rp, r1 = r0 + 1; r1 -= n * (r1 >= n);
    float fr = rp - (float)r0;
    float out = buf[r0] + fr * (buf[r1] - buf[r0]);
    state[3] += 0.35f * (out - state[3]);              // oxide rolloff
    float s = fmaxf(-3.0f, fminf(3.0f, state[3] * fb));
    buf[wp] = x + s * (27.0f + s * s) / (27.0f + 9.0f * s * s);
    state[2] = (float)((wp + 1) % n);
    state[3] = zapDenormal(state[3]);
    return out;
}


// Allpass Swarm


// Regenerative allpass swarm: 3 detuned allpasses in a feedback loop held
// at the edge of oscillation by an energy governor — metallic bloom that
// rings and swells but can't blow up. color 0..1, regen 0..1.2. Zero-init state[5].
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
// reads. size 0..1 (needs n > 1980*size + 2). Zero-init buf + state[1].
inline float fx_diffuse(float x, float* buf, int n, float size, float mix, float* state) {
    // Clamp size so the largest prime tap (1979*size + 1) fits inside the
    // ring after a single wrap: hard precondition n > 1980*size + 2.
    const float sizeMax = fmaxf(0.0f, (float)(n - 2) * (1.0f / 1980.0f));
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
// zero grain scheduling. spread < n-4 samples. Zero-init buf + state[4].
inline float gran_cloud(float x, float* buf, int n, float spread, uint32_t* rng, float* state) {
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
        int r0 = (int)rp, r1 = r0 + 1; r1 -= n * (r1 >= n);
        float fr = rp - (float)r0;
        out += buf[r0] + fr * (buf[r1] - buf[r0]);
    }
    return out * 0.577f;                               // ~1/sqrt(3)
}



// Frequency Shifter (SSB)


// Frequency shifter (single-sideband): 8-section IIR Hilbert pair +
// self-correcting quadrature carrier. Shifts every partial by inc*fs Hz —
// inharmonic bells and ghosts, unlike pitch shifting. Negative inc flips
// sideband. state[35] zero-init.
inline float fx_freqshift(float x, float inc, float* state) {
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
    float th = 6.2831853f * inc;
    float c2 = c - th * sn, s2 = sn + th * c;
    float g = 1.5f - 0.5f * (c2 * c2 + s2 * s2);       // cheap renorm
    state[33] = c2 * g; state[34] = s2 * g;
    for (int k = 0; k < 35; ++k) state[k] = zapDenormal(state[k]);
    return id * state[33] - q * state[34];
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


// Analog Octave-Down


// Analog-style octave-down: flip-flop toggled on rising zero crossings
// ring-mods the input itself, so the sub tracks envelope automatically
// (naive flip-flop octavers output a fixed-level square). Zero-init state[3].
inline float pitch_octdown(float x, float mix, float* state) {
    if (state[1] <= 0.0f && x > 0.0f) state[0] = 1.0f - state[0];
    state[1] = x;
    float sub = x * (state[0] > 0.5f ? 1.0f : -1.0f);
    state[2] += 0.15f * (sub - state[2]);              // mellow the sub
    return x + mix * state[2];
}



// Analog RC ADSR


// RC-style ADSR: attack charges toward 130% then clamps at full scale,
// giving the punchy convex attack of analog envelopes. Rates are
// one-pole coeffs (bigger = faster). Zero-init state[3].
inline float adsr_analog(float gate, float atk, float dec, float sus, float rel, float* state) {
    float lvl = state[0];
    float on = (float)(gate > 0.5f);
    float rose = on * (1.0f - state[1]);
    state[1] = on;
    float atkp = fmaxf(state[2], rose);                // retrigger attack phase
    if (atkp > 0.5f) {
        lvl += atk * (1.3f - lvl);                     // overshoot target
        if (lvl >= 1.0f) { lvl = 1.0f; atkp = 0.0f; }
    } else {
        lvl += (on > 0.5f ? dec : rel) * (on * sus - lvl);
    }
    state[0] = lvl; state[2] = atkp;
    state[0] = zapDenormal(state[0]);
    return lvl;
}


// Looping AD Envelope


// Looping AD envelope: exponential attack aims past full scale (punchy),
// exponential decay; loop > 0.5 retriggers at the floor for LFO-like
// bursts. Rates are one-pole coeffs. Zero-init state[3].
inline float env_loopad(float trig, float atk, float dec, float loop, float* state) {
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
// exact segment timing. rate = segment freq/fs. Zero-init state[5].
inline float lfo_randcubic(float rate, uint32_t* rng, float* state) {
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


// Lorenz Chaos Source


// Lorenz chaos source with AGC: output stays ~+/-1 at any rate setting
// instead of the raw attractor's wild scale. Self-starts from zero state.
// rate 0.0001 (slow CV) .. 0.01 (audio growl). Zero-init state[4].
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
// LCG is zero-init safe. spring ~0.001, step ~0.01..0.1.
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
// bounded time. rate = units/sample. Zero-init state[1].
inline float smooth_catchup(float target, float rate, float* state) {
    float d = target - state[0];
    float lin = fmaxf(-rate, fminf(rate, d));
    state[0] += lin + 0.02f * (d - lin);
    return state[0];
}

}  // namespace rpdsp
