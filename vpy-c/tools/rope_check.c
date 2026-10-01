/* rope_check — vpyrope against physics with known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/rope_check.c vpyrope.c vpy3d.c vpy.c \
 *      -lm -o /tmp/rope_check && /tmp/rope_check
 *
 * Exit status is the number of failures. <math.h> only for reference values.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpyrope.h"

/* the drawing side is linked but not exercised here */
uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

int main(void)
{
    int32_t x, y, z;

    /* 1. a rope laid out sideways, pinned at one end, ends up hanging straight down */
    vpyrope_reset(); vpyrope_set_gravity(0, -9800, 0);
    int r = vpyrope_new(0, 2000, 0, 1000, 2000, 0, 10);
    vpyrope_pin(r, 0, 0, 2000, 0);
    for (int i = 0; i < 1000; i++) { vpyrope_pin(r, 0, 0, 2000, 0); vpyrope_step(); }
    vpyrope_point(r, 10, &x, &y, &z);
    CHECK(abs(x) < 15 && abs(y - 1000) < 30, "a 1000-unit rope pinned at y=2000 hangs to (%d, %d), expected (0, 1000)", x, y);
    CHECK(vpyrope_stats()->stretch < 10, "and its links stay their length (worst %d over)", vpyrope_stats()->stretch);

    /* 2. a pendulum: one stiff link swings with T = 2 pi sqrt(L / g) */
    vpyrope_reset(); vpyrope_set_gravity(0, -9800, 0); vpyrope_set_damping(256);
    const int L = 1000;
    r = vpyrope_new(0, 0, 0, 150, -(int32_t)sqrt(L * L - 150 * 150), 0, 1);   /* a small swing */
    int crossings = 0, first = -1, last = -1; int32_t px = 150;
    for (int i = 0; i < 2000; i++) {
        vpyrope_pin(r, 0, 0, 0, 0); vpyrope_step();
        vpyrope_point(r, 1, &x, &y, &z);
        if ((px > 0) != (x > 0)) { crossings++; if (first < 0) first = i; last = i; }
        px = x;
    }
    const double T = 2.0 * (last - first) / (crossings - 1) / 50.0, Tth = 2 * M_PI * sqrt(L / 9800.0);
    CHECK(fabs(T - Tth) < 0.03 * Tth, "pendulum of 1000: period %.3f s, theory %.3f s", T, Tth);

    /* 3. dropped on a floor it lies on it, never through it */
    vpyrope_reset(); vpyrope_set_gravity(0, -9800, 0); vpyrope_set_floor(1, 0);
    r = vpyrope_new(-500, 800, 0, 500, 800, 0, 20);
    for (int i = 0; i < 300; i++) vpyrope_step();
    int on = 1; for (int i = 0; i <= 20; i++) { vpyrope_point(r, i, &x, &y, &z); if (y < 0 || y > 5) on = 0; }
    CHECK(on, "a rope dropped on a floor lies on it (every point between 0 and 5)");

    /* 4. a moving pin drags the rope after it */
    vpyrope_reset(); vpyrope_set_gravity(0, -9800, 0);
    r = vpyrope_new(0, 0, 0, 0, -500, 0, 5);
    for (int i = 0; i < 200; i++) { vpyrope_pin(r, 0, i * 5, 0, 0); vpyrope_step(); }
    vpyrope_point(r, 5, &x, &y, &z);
    CHECK(x > 700, "pin moved to x=995: the far end followed to x=%d", x);

    /* 5. tables full are counted; same calls, same result */
    vpyrope_reset();
    int made = 0; for (int i = 0; i < VPYROPE_MAX_ROPES + 2; i++) made += vpyrope_new(0, 0, 0, 100, 0, 0, 4) >= 0;
    CHECK(made == VPYROPE_MAX_ROPES && vpyrope_stats()->refused == 2, "%d ropes made of %d asked, %u refused", made, VPYROPE_MAX_ROPES + 2, vpyrope_stats()->refused);
    int32_t a[2];
    for (int run = 0; run < 2; run++) {
        vpyrope_reset(); vpyrope_set_gravity(0, -9800, 0);
        r = vpyrope_new(0, 0, 0, 700, 300, 100, 12);
        for (int i = 0; i < 150; i++) { vpyrope_pin(r, 0, 0, 0, 0); vpyrope_step(); }
        vpyrope_point(r, 12, &x, &y, &z); a[run] = x * 31 + y * 7 + z;
    }
    CHECK(a[0] == a[1], "deterministic: the same swing twice");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
