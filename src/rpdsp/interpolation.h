#pragma once

// Force the inline expansion of the shared interpolation helper. The helper
// replaces an expression that used to live directly in DelayLine::readCubic(),
// and the expanded form is what keeps the generated code, and therefore the
// results, identical to the previous implementation. It also matters under
// -ffast-math, where an out-of-line call lets the compiler pick a different
// evaluation order for the four products and their sum.
#if defined(__clang__) || defined(__GNUC__)
#define RPDSP_INTERPOLATION_INLINE inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define RPDSP_INTERPOLATION_INLINE __forceinline
#else
#define RPDSP_INTERPOLATION_INLINE inline
#endif

namespace rpdsp {

// Stateless third-order Lagrange (cubic) interpolation helper.
//
// This is a shared, stateless interpolation primitive: it owns no storage, no
// state and allocates nothing. Callers keep their own sample storage and their
// own tap ordering, and pass the four ordered samples around the fractional
// position.
//
// The coefficients and the summation order are copied verbatim from
// DelayLine::readCubic() so that results stay bit-identical to the previous
// inlined expression.
//
// Callers that replace a local copy of this expression must keep the call
// structure they had. A -ffast-math build (the Pico2Seq firmware profile uses
// it) reassociates the four products and their sum differently depending on
// where the expression sits in the caller's code: moving it up by one level
// changed Pico2Seq's master delay output by up to 9.6e-05. The helper is
// force-inlined so it cannot become an out-of-line call, but the surrounding
// shape is still the caller's responsibility.
RPDSP_INTERPOLATION_INLINE float cubicLagrangeInterpolate(float frac, float ym1, float y0,
                                                          float y1, float y2) {
  const float c0 = (-frac * (frac - 1.0f) * (frac - 2.0f)) * (1.0f / 6.0f);
  const float c1 = ((frac + 1.0f) * (frac - 1.0f) * (frac - 2.0f)) * 0.5f;
  const float c2 = (-(frac + 1.0f) * frac * (frac - 2.0f)) * 0.5f;
  const float c3 = ((frac + 1.0f) * frac * (frac - 1.0f)) * (1.0f / 6.0f);

  return (ym1 * c0) + (y0 * c1) + (y1 * c2) + (y2 * c3);
}

}  // namespace rpdsp
