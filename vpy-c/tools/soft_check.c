/* soft_check — vpysoft against behaviours with known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/soft_check.c vpysoft.c vpy3d.c vpy.c \
 *      -lm -o /tmp/soft_check && /tmp/soft_check
 *
 * Exit status is the number of failures. <math.h> only for reference values.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy3d.h"
#include "vpysoft.h"

/* the drawing side is linked; strokes are counted through vpysoft_stats */
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

/* a blob's rim area and whether it turns one way all round (convex) */
static double blob_area(int b, int n, int *convex)
{
    double a = 0; int pos = 0, neg = 0;
    for (int i = 0; i < n; i++) {
        int32_t x0, y0, z0, x1, y1, z1, x2, y2, z2;
        vpysoft_point(b, 1 + i, &x0, &y0, &z0);
        vpysoft_point(b, 1 + (i + 1) % n, &x1, &y1, &z1);
        vpysoft_point(b, 1 + (i + 2) % n, &x2, &y2, &z2);
        a += (double)x0 * y1 - (double)x1 * y0;
        const double cr = (double)(x1 - x0) * (y2 - y1) - (double)(y1 - y0) * (x2 - x1);
        if (cr > 0) pos++; else if (cr < 0) neg++;
    }
    if (convex) *convex = (pos == 0 || neg == 0) || (pos <= 2 || neg <= 2);
    return fabs(a) / 2;
}

/* the period of the bottom of a 2×2 patch hanging off its pinned top, bouncing */
static double period_of(int stiff)
{
    vpysoft_reset(); vpysoft_set_iterations(1); vpysoft_set_damping(256);
    int b = vpysoft_cloth(0, 0, 0, 2, 2, 100, stiff);
    vpysoft_pin(b, 0, 0, 0, 0); vpysoft_pin(b, 1, 100, 0, 0);
    /* pulled down 30 and let go */
    vpysoft_pin(b, 2, 0, -130, 0); vpysoft_pin(b, 3, 100, -130, 0);
    vpysoft_step(); vpysoft_unpin(b, 2); vpysoft_unpin(b, 3);
    int32_t x, y, z, prev = -130; int cross = 0, first = -1, last = -1;
    for (int i = 0; i < 2000; i++) {
        vpysoft_step(); vpysoft_point(b, 2, &x, &y, &z);
        if ((prev < -100) != (y < -100)) { cross++; if (first < 0) first = i; last = i; }
        prev = y;
    }
    return cross > 2 ? 2.0 * (last - first) / (cross - 1) : 1e9;
}

int main(void)
{
    int32_t x, y, z;

    /* 1. a cloth pinned at its two top corners hangs below them, sags in the
     *    middle, and its springs hold their length */
    vpysoft_reset(); vpysoft_set_gravity(0, -9800, 0);
    const int W = 9, H = 6, C = 50;
    int c = vpysoft_cloth(0, 1000, 0, W, H, C, 256);
    int maxstretch = 0;
    for (int i = 0; i < 300; i++) {
        vpysoft_pin(c, 0, 0, 1000, 0); vpysoft_pin(c, W - 1, (W - 1) * C, 1000, 0);
        vpysoft_step();
        if (vpysoft_stats()->stretch > maxstretch && i > 50) maxstretch = vpysoft_stats()->stretch;
    }
    int above = 0; int32_t mid_top, corner_bottom, mid_bottom;
    for (int i = 0; i < W * H; i++) { vpysoft_point(c, i, &x, &y, &z); if (y > 1000) above++; }
    vpysoft_point(c, W / 2, &x, &mid_top, &z);
    vpysoft_point(c, (H - 1) * W, &x, &corner_bottom, &z);
    vpysoft_point(c, (H - 1) * W + W / 2, &x, &mid_bottom, &z);
    CHECK(above == 0 && mid_top < 1000 && mid_bottom < mid_top && corner_bottom < 1000 - (H - 1) * C + C,
          "a cloth pinned at its top corners hangs below them: top middle y=%d (the taut top row barely sags), bottom middle %d, bottom corner %d", mid_top, mid_bottom, corner_bottom);
    CHECK(maxstretch <= C / 5, "and once it has settled its springs stretch at most %d units (of %d)", maxstretch, C);

    /* 2. a blob dropped on the floor comes to rest, keeps its area and stays round */
    double area[2]; int conv[2]; int32_t restmove[2];
    for (int pr = 0; pr < 2; pr++) {
        vpysoft_reset(); vpysoft_set_gravity(0, -9800, 0); vpysoft_set_floor(1, 0, 128);
        int bl = vpysoft_blob(0, 600, 0, 16, 200, 200, pr ? 128 : 0);
        const double a0 = M_PI * 200 * 200 * sin(2 * M_PI / 16) * 16 / (2 * M_PI);   /* the 16-gon */
        int32_t y_then = 0, y_now = 0;
        for (int i = 0; i < 400; i++) { vpysoft_step(); if (i == 389) vpysoft_point(bl, 0, &x, &y_then, &z); }
        vpysoft_point(bl, 0, &x, &y_now, &z);
        restmove[pr] = abs(y_now - y_then);
        int below = 0;
        for (int i = 0; i <= 16; i++) { vpysoft_point(bl, i, &x, &y, &z); if (y < 0) below++; }
        area[pr] = blob_area(bl, 16, &conv[pr]) / a0;
        if (pr) CHECK(below == 0 && restmove[pr] <= 2, "a blob dropped on the floor rests on it (centre moved %d in the last 10 steps, %d points under the floor)", restmove[pr], below);
    }
    CHECK(fabs(area[1] - 1) < 0.10 && conv[1], "with pressure it keeps its area: %.0f%% of what it was made with, and stays convex", area[1] * 100);
    CHECK(area[0] < area[1], "without pressure it squashes more: %.0f%% of its area", area[0] * 100);

    /* 3. a spring: stiffer bounces faster */
    const double soft = period_of(32), stiff = period_of(128);
    CHECK(stiff < soft && stiff < 1e8, "a stiffer spring bounces faster: period %.1f steps at stiffness 128, %.1f at 32", stiff, soft);

    /* 4. a mesh: a cube dropped as a jelly lands and does not collapse */
    static vpy_mesh cube; int v[8];
    vpy3d_mesh_begin(&cube);
    for (int k = 0; k < 8; k++) v[k] = vpy3d_vertex((k & 1) ? 100 : -100, (k & 2) ? 100 : -100, (k & 4) ? 100 : -100);
    vpy3d_quad(v[0], v[4], v[6], v[2]); vpy3d_quad(v[1], v[3], v[7], v[5]);
    vpy3d_quad(v[0], v[1], v[5], v[4]); vpy3d_quad(v[2], v[6], v[7], v[3]);
    vpy3d_quad(v[0], v[2], v[3], v[1]); vpy3d_quad(v[4], v[5], v[7], v[6]);
    vpy3d_mesh_end(VPY3D_HARD_45);
    vpysoft_reset(); vpysoft_set_gravity(0, -9800, 0); vpysoft_set_floor(1, 0, 128);
    int m = vpysoft_mesh(&cube, 0, 400, 0, 256);
    for (int i = 0; i < 300; i++) vpysoft_step();
    int32_t lo = 1 << 30, hi = -(1 << 30);
    for (int i = 0; i < vpysoft_points(m); i++) { vpysoft_point(m, i, &x, &y, &z); if (y < lo) lo = y; if (y > hi) hi = y; }
    CHECK(m >= 0 && vpysoft_points(m) == 9 && lo >= 0 && hi - lo > 150, "a cube mesh as a jelly: %d points (8 + its centre), %u springs, lands at y=%d and stands %d tall (200)", vpysoft_points(m), vpysoft_stats()->springs, lo, hi - lo);

    /* 5. the stroke budget, and what is left out is counted */
    vpysoft_reset(); vpysoft_set_budget(50);
    int big = vpysoft_cloth(-100, 100, 0, 10, 10, 20, 256);      /* 180 structural springs */
    vpysoft_draw2d(big, 80);
    CHECK(vpysoft_stats()->drawn == 50 && vpysoft_stats()->shed == 130, "a 10×10 cloth with a budget of 50: %u drawn, %u shed (180 structural springs; shear is never drawn)", vpysoft_stats()->drawn, vpysoft_stats()->shed);

    /* 6. refusals are counted; the tables are packed */
    vpysoft_reset();
    int bad = (vpysoft_blob(0, 0, 0, 2, 100, 200, 128) < 0) + (vpysoft_cloth(0, 0, 0, 1, 5, 10, 256) < 0);
    int made = 0; for (int i = 0; i < VPYSOFT_MAX_BODIES + 2; i++) made += vpysoft_blob(0, 0, 0, 8, 50, 200, 128) >= 0;
    CHECK(bad == 2 && made == VPYSOFT_MAX_BODIES && vpysoft_stats()->refused == 4, "refused: a 2-point blob, a 1-wide cloth, and 2 bodies past the table (%u counted)", vpysoft_stats()->refused);
    vpysoft_reset();
    int hog = vpysoft_cloth(0, 0, 0, 12, 12, 10, 256);                  /* 144 points, 506 springs */
    int none = vpysoft_cloth(0, 0, 0, 12, 12, 10, 256);                 /* 144 more do not fit 256 */
    vpysoft_free(hog);
    int again = vpysoft_cloth(0, 0, 0, 12, 12, 10, 256);
    CHECK(hog >= 0 && none < 0 && again >= 0, "a second 12×12 cloth does not fit the point table and is refused; freeing the first makes room");

    /* 7. the same calls, the same result */
    int32_t h[2];
    for (int run = 0; run < 2; run++) {
        vpysoft_reset(); vpysoft_set_gravity(0, -9800, 0); vpysoft_set_floor(1, 0, 128);
        int bl = vpysoft_blob(30, 700, 0, 12, 150, 180, 100);
        vpysoft_push(bl, 800, 0, 0);
        for (int i = 0; i < 200; i++) vpysoft_step();
        int32_t acc = 0;
        for (int i = 0; i <= 12; i++) { vpysoft_point(bl, i, &x, &y, &z); acc = acc * 31 + x * 7 + y; }
        h[run] = acc;
    }
    CHECK(h[0] == h[1], "deterministic: the same blob, pushed and dropped twice, lands the same");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
