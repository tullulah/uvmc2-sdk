/*
 * vpyease.c — see vpyease.h. Q14 throughout: 16384 is 1.0.
 */
#include "vpyease.h"

int vpy_sin_q14(int a);          /* vpy.c */

#define ONE 16384

static int32_t clamp_t(int32_t t) { return t < 0 ? 0 : (t > ONE ? ONE : t); }
static int32_t mul(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 14); }

int32_t vpy_ease_linear(int32_t t)   { return clamp_t(t); }
int32_t vpy_ease_in_quad(int32_t t)  { t = clamp_t(t); return mul(t, t); }
int32_t vpy_ease_out_quad(int32_t t) { t = clamp_t(t); const int32_t u = ONE - t; return ONE - mul(u, u); }
int32_t vpy_ease_in_out_quad(int32_t t)
{
    t = clamp_t(t);
    if (t < ONE / 2) return 2 * mul(t, t);
    const int32_t u = ONE - t;
    return ONE - 2 * mul(u, u);
}
int32_t vpy_ease_in_cubic(int32_t t)  { t = clamp_t(t); return mul(mul(t, t), t); }
int32_t vpy_ease_out_cubic(int32_t t) { t = clamp_t(t); const int32_t u = ONE - t; return ONE - mul(mul(u, u), u); }
int32_t vpy_ease_in_out_cubic(int32_t t)
{
    t = clamp_t(t);
    if (t < ONE / 2) return 4 * mul(mul(t, t), t);
    const int32_t u = ONE - t;
    return ONE - 4 * mul(mul(u, u), u);
}
int32_t vpy_ease_smoothstep(int32_t t)
{
    t = clamp_t(t);
    return mul(mul(t, t), 3 * ONE - 2 * t);
}

/* 1 + c3 (t-1)³ + c1 (t-1)², c1 = 1.70158 (the usual overshoot of ~10%), c3 = c1 + 1 */
int32_t vpy_ease_out_back(int32_t t)
{
    t = clamp_t(t);
    const int32_t c1 = 27879, c3 = c1 + ONE;             /* 1.70158 and 2.70158, Q14 */
    const int32_t u = t - ONE;
    return ONE + mul(c3, mul(mul(u, u), u)) + mul(c1, mul(u, u));
}

/* the classic piecewise parabolas (n1 = 7.5625, d1 = 2.75) */
int32_t vpy_ease_out_bounce(int32_t t)
{
    t = clamp_t(t);
    if (t == ONE) return ONE;     /* the constants round to 16377 there; the end is the end */
    const int32_t n1 = 123904;                            /* 7.5625 */
    const int32_t d1i = 5958;                             /* 1/2.75 */
    if (t < d1i)            return mul(n1, mul(t, t));
    if (t < 2 * d1i)        { const int32_t u = t - 8937;  return mul(n1, mul(u, u)) + 12288; }   /* 1.5/2.75, +0.75 */
    if (t < 2 * d1i + d1i / 2) { const int32_t u = t - 13405; return mul(n1, mul(u, u)) + 15360; } /* 2.25/2.75, +0.9375 */
    const int32_t u = t - 15639;                          /* 2.625/2.75 */
    return mul(n1, mul(u, u)) + 16128;                    /* +0.984375 */
}

/* 2^(-10t) sin((10t - 0.75) 2pi/3) + 1, with 2^(-10t) as integer halvings */
int32_t vpy_ease_out_elastic(int32_t t)
{
    t = clamp_t(t);
    if (t == 0) return 0;
    if (t == ONE) return ONE;
    /* 2^(-10t): 10t in Q14 is the exponent; whole part by shifts, the rest by a
     * linear step between powers (good to a few percent, enough for a spring) */
    const int32_t e = 10 * t;                             /* Q14 */
    const int whole = e >> 14, frac = e & (ONE - 1);
    int32_t decay = ONE >> (whole > 14 ? 14 : whole);
    decay -= (int32_t)(((int64_t)decay * frac) >> 15);   /* × (1 - frac/2): 2^-f ≈ 1 - f/2 */
    /* angle: (10t - 0.75) / 3 turns, in 4096ths of a turn */
    const int64_t ang = ((int64_t)(e - 12288) * 4096) / (3 * ONE);
    return mul(decay, vpy_sin_q14((int)ang)) + ONE;
}

int32_t vpy_lerp_ease(int32_t from, int32_t to, int32_t t_q14, vpy_ease_fn curve)
{
    const int32_t k = curve ? curve(clamp_t(t_q14)) : clamp_t(t_q14);
    return from + (int32_t)(((int64_t)(to - from) * k) >> 14);
}

int32_t vpy_tween(int32_t from, int32_t to, int frame, int frames, vpy_ease_fn curve)
{
    if (frames <= 0 || frame >= frames) return to;
    if (frame <= 0) return from;
    return vpy_lerp_ease(from, to, (int32_t)(((int64_t)frame << 14) / frames), curve);
}
