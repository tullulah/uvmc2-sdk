/* motion_check — vpyease and vpycam, as tests with known answers.
 *
 *   cc -O2 -w -Iinclude tools/motion_check.c vpyease.c vpycam.c -lm -o /tmp/motion_check && /tmp/motion_check
 *
 * vpycam_look_at needs vpy3d and is not exercised here; vpy3d's look_at has its
 * own users. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpyease.h"
#include "vpycam.h"

int vpy_sin_q14(int a) { return (int)lround(16384.0 * sin(a * 2 * M_PI / 4096)); }
int vpy3d_look_at(int32_t ex, int32_t ey, int32_t ez, int32_t tx, int32_t ty, int32_t tz,
                  int32_t ux, int32_t uy, int32_t uz)
{ (void)ex; (void)ey; (void)ez; (void)tx; (void)ty; (void)tz; (void)ux; (void)uy; (void)uz; return 1; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

int main(void)
{
    /* every curve: exactly 0 at 0 and exactly 1 at 1 */
    static const struct { const char *name; vpy_ease_fn f; int mono; } C[] = {
        { "linear", vpy_ease_linear, 1 }, { "in_quad", vpy_ease_in_quad, 1 },
        { "out_quad", vpy_ease_out_quad, 1 }, { "in_out_quad", vpy_ease_in_out_quad, 1 },
        { "in_cubic", vpy_ease_in_cubic, 1 }, { "out_cubic", vpy_ease_out_cubic, 1 },
        { "in_out_cubic", vpy_ease_in_out_cubic, 1 }, { "smoothstep", vpy_ease_smoothstep, 1 },
        { "out_back", vpy_ease_out_back, 0 }, { "out_bounce", vpy_ease_out_bounce, 0 },
        { "out_elastic", vpy_ease_out_elastic, 0 },
    };
    for (unsigned i = 0; i < sizeof C / sizeof C[0]; i++) {
        int mono = 1, prev = -1;
        for (int t = 0; t <= 16384; t += 64) { const int v = C[i].f(t); if (v < prev) mono = 0; prev = v; }
        CHECK(C[i].f(0) == 0 && C[i].f(16384) == 16384 && (!C[i].mono || mono),
              "%-13s 0 -> %d, 1 -> %d%s", C[i].name, C[i].f(0), C[i].f(16384), C[i].mono ? ", never goes back" : "");
    }
    int peak = 0; for (int t = 0; t <= 16384; t += 16) if (vpy_ease_out_back(t) > peak) peak = vpy_ease_out_back(t);
    CHECK(peak > 17500 && peak < 18500, "out_back overshoots by ~10%%: peak %.3f", peak / 16384.0);
    int bounces = 0, prev = vpy_ease_out_bounce(0), dir = 1;
    for (int t = 16; t <= 16384; t += 16) { const int v = vpy_ease_out_bounce(t); if (dir > 0 && v < prev) { bounces++; dir = -1; } if (dir < 0 && v > prev) dir = 1; prev = v; }
    CHECK(bounces == 3 && vpy_ease_out_bounce(5958) > 16000, "out_bounce: hits the end at t=0.364, then bounces %d times", bounces);
    CHECK(vpy_ease_in_out_quad(8192) == 8192 && vpy_ease_smoothstep(8192) == 8192, "in_out and smoothstep pass the middle at the middle");
    CHECK(vpy_tween(100, 900, 0, 30, vpy_ease_out_cubic) == 100 && vpy_tween(100, 900, 30, 30, vpy_ease_out_cubic) == 900 &&
          vpy_tween(100, 900, 99, 30, vpy_ease_out_cubic) == 900, "tween: frame 0 is from, frame N and after is to");

    /* the camera: a dead zone, then following, leading, without creeping */
    vpycam_reset(0, 0, 0);
    vpycam_follow_config(200, 100, 0, 10, 4);
    int32_t x, y, z;
    vpycam_follow(150, 50, 0, 0, 0, 0); vpycam_focus(&x, &y, &z);
    CHECK(x == 0 && y == 0, "target inside the dead zone: focus stays (%d,%d)", x, y);
    for (int i = 0; i < 60; i++) vpycam_follow(1000, 0, 0, 0, 0, 0);
    vpycam_focus(&x, &y, &z);
    CHECK(x == 800, "target at 1000: focus settles at the zone's edge, 800 (%d) — no creeping short of it", x);
    vpycam_reset(0, 0, 0); vpycam_follow_config(0, 0, 0, 10, 1);
    vpycam_follow(0, 0, 0, 30, 0, 0); vpycam_focus(&x, &y, &z);
    CHECK(x == 300, "leading: a target moving 30 a frame, 10 frames of lead, focus at %d (300)", x);

    /* shake: decays to nothing, the same twice */
    int32_t first[40]; int ok = 1, maxa = 0;
    for (int run = 0; run < 2; run++) {
        vpycam_reset(0, 0, 0); vpycam_seed(5); vpycam_shake(40, 20);
        for (int i = 0; i < 25; i++) {
            vpycam_step(); int32_t dx, dy, dz; vpycam_shake_offset(&dx, &dy, &dz);
            if (run == 0) { first[i] = dx; if (abs(dx) > maxa) maxa = abs(dx); }
            else if (first[i] != dx) ok = 0;
            if (i >= 20 && (dx | dy | dz)) ok = 0;
        }
    }
    CHECK(ok && maxa > 10 && maxa <= 40, "shake of 40 for 20 frames: up to %d, nothing after, the same twice", maxa);

    /* hit-stop: exactly the frames asked */
    vpycam_reset(0, 0, 0); vpycam_hitstop(4); vpycam_hitstop(2);
    int stopped = 0; for (int i = 0; i < 10; i++) { stopped += vpycam_stopped(); vpycam_step(); }
    CHECK(stopped == 4, "hit-stop 4 (and a shorter one ignored): stopped %d frames", stopped);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
