/*
 * vpyease.h — easing curves and tweens, in integers.
 *
 * A curve takes t in Q14 (0 = the start, 16384 = the end) and returns how far
 * along the movement is, also Q14. Every curve gives exactly 0 at 0 and exactly
 * 16384 at 16384, so a tween lands where it was told to. "back" and "elastic"
 * overshoot past 16384 on the way, on purpose.
 *
 *     x = vpy_tween(from, to, frame, frames, vpy_ease_out_cubic);
 *
 * Integer only, deterministic; elastic uses vpy_sin_q14 from vpy.c.
 */
#ifndef VPYEASE_H
#define VPYEASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t (*vpy_ease_fn)(int32_t t_q14);

int32_t vpy_ease_linear(int32_t t);
int32_t vpy_ease_in_quad(int32_t t);
int32_t vpy_ease_out_quad(int32_t t);
int32_t vpy_ease_in_out_quad(int32_t t);
int32_t vpy_ease_in_cubic(int32_t t);
int32_t vpy_ease_out_cubic(int32_t t);
int32_t vpy_ease_in_out_cubic(int32_t t);
int32_t vpy_ease_smoothstep(int32_t t);    /* 3t² - 2t³ */
int32_t vpy_ease_out_back(int32_t t);      /* past the end by ~10%, then back */
int32_t vpy_ease_out_bounce(int32_t t);    /* a ball dropped onto the end: 3 bounces */
int32_t vpy_ease_out_elastic(int32_t t);   /* a spring settling on the end */

/* from + (to - from) × curve(t). t is clamped to 0..16384. */
int32_t vpy_lerp_ease(int32_t from, int32_t to, int32_t t_q14, vpy_ease_fn curve);
/* The same over `frames` steps: frame 0 is `from`, frame `frames` and after is `to`. */
int32_t vpy_tween(int32_t from, int32_t to, int frame, int frames, vpy_ease_fn curve);

#ifdef __cplusplus
}
#endif

#endif /* VPYEASE_H */
