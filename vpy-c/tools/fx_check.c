/* fx_check — what vpyfx promises, as tests with known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/fx_check.c vpyfx.c vpy3d.c vpy.c \
 *      -lm -o /tmp/fx_check && /tmp/fx_check
 *
 * The strokes are caught where libvpy hands them to the hardware
 * (v_directDraw32), so what is checked is what would be drawn. Exit status is
 * the number of failures. <math.h> is used HERE only, for reference values.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy.h"
#include "vpy3d.h"
#include "vpyfx.h"

/* the PiTrex contract, caught */
uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
typedef struct { int32_t x0, y0, x1, y1; int br; } seg_t;
static seg_t S[4096]; static int NS;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e)
{ if (NS < 4096) S[NS++] = (seg_t){ a, b, c, d, e }; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* draw in 2D (x, y as device units) and flush into S */
static void frame(void) { NS = 0; vpyfx_draw2d(); vpy_flush(); }

int main(void)
{
    /* 1. a spark under gravity: semi-implicit Euler, like vpyphys */
    vpyfx_reset(); vpyfx_set_gravity(0, -9800, 0);
    vpyfx_spark(0, 10000, 0, 0, 0, 0, 1000, 100);
    for (int i = 0; i < 25; i++) vpyfx_step();
    frame();
    const double yd = 10000 - 9800.0 / 2500 * 25 * 26 / 2;
    CHECK(NS == 1 && fabs(S[0].y0 - yd) <= 2, "spark falls: y=%d, expected %.0f", NS ? S[0].y0 : -1, yd);
    CHECK(NS == 1 && S[0].y1 > S[0].y0, "its streak trails behind it (above, falling): %d -> %d", S[0].y0, S[0].y1);

    /* 2. a spark at rest still draws a stroke with length */
    vpyfx_reset(); vpyfx_spark(100, 100, 0, 0, 0, 0, 50, 100); vpyfx_step(); frame();
    CHECK(NS == 1 && abs(S[0].x1 - S[0].x0) + abs(S[0].y1 - S[0].y0) >= 3, "a still spark is a short stroke, not a dot");

    /* 3. the same seed, the same burst */
    int32_t first[64]; int nfirst = 0, same = 1;
    for (int run = 0; run < 2; run++) {
        vpyfx_reset(); vpyfx_seed(1234); vpyfx_set_gravity(0, -9800, 0);
        vpyfx_burst(0, 0, 0, 0, 0, 0, 20, 3000, 100, 120);
        for (int i = 0; i < 10; i++) vpyfx_step();
        frame();
        for (int i = 0; i < NS && i < 64; i++) {
            if (run == 0) first[nfirst++] = S[i].x0 * 7 + S[i].y0;
            else if (first[i] != S[i].x0 * 7 + S[i].y0) same = 0;
        }
    }
    CHECK(same && nfirst == 20, "deterministic: a seeded burst of 20 lands the same twice");

    /* 4. the budget draws so many and counts the rest */
    vpyfx_reset(); vpyfx_set_budget(100);
    vpyfx_burst(0, 0, 0, 0, 0, 0, 150, 1000, 100, 120);
    vpyfx_step(); frame();
    CHECK(vpyfx_stats()->drawn == 100 && vpyfx_stats()->shed == 50 && NS == 100,
          "budget 100 of 150 alive: drew %u, shed %u", vpyfx_stats()->drawn, vpyfx_stats()->shed);

    /* 5. the pool full: new pieces replace the oldest, counted */
    vpyfx_reset();
    vpyfx_burst(0, 0, 0, 0, 0, 0, VPYFX_MAX + 10, 1000, 100, 120);
    CHECK(vpyfx_stats()->recycled == 10, "pool of %d, %d asked: recycled %u", VPYFX_MAX, VPYFX_MAX + 10, vpyfx_stats()->recycled);

    /* 6. fading: dimmer as it ages, gone at the end of its life */
    vpyfx_reset(); vpyfx_spark(0, 0, 0, 0, 0, 0, 50, 120);
    vpyfx_step(); frame(); const int b0 = NS ? S[0].br : 0;
    for (int i = 0; i < 25; i++) vpyfx_step(); frame(); const int b1 = NS ? S[0].br : 0;
    for (int i = 0; i < 30; i++) vpyfx_step(); frame();
    CHECK(b0 > b1 && b1 > 0 && NS == 0, "fades: brightness %d, then %d, then gone", b0, b1);

    /* 7. a cube shatters into its twelve edges, flying out, keeping their length */
    vpy_mesh cube; int v[8];
    vpy3d_mesh_begin(&cube);
    for (int k = 0; k < 8; k++) v[k] = vpy3d_vertex((k & 1) ? 100 : -100, (k & 2) ? 100 : -100, (k & 4) ? 100 : -100);
    vpy3d_quad(v[0], v[4], v[6], v[2]); vpy3d_quad(v[1], v[3], v[7], v[5]);
    vpy3d_quad(v[0], v[1], v[5], v[4]); vpy3d_quad(v[2], v[6], v[7], v[3]);
    vpy3d_quad(v[0], v[2], v[3], v[1]); vpy3d_quad(v[4], v[5], v[7], v[6]);
    vpy3d_mesh_end(VPY3D_HARD_45);
    vpyfx_reset(); vpyfx_seed(7);
    const vpy_xf at = vpy3d_translate(0, 1000, 0);
    const int n = vpyfx_shatter(&cube, &at, 0, 0, 0, 0, 1000, 0, 2000, 2048, 200, 120);
    for (int i = 0; i < 20; i++) vpyfx_step();
    frame();
    int lenok = 1, out = 1;
    for (int i = 0; i < NS; i++) {
        const double l = hypot(S[i].x1 - S[i].x0, S[i].y1 - S[i].y0);
        if (l > 200 * 1.02) lenok = 0;          /* projected onto x,y it can only be shorter */
        const double cx = (S[i].x0 + S[i].x1) / 2.0, cy = (S[i].y0 + S[i].y1) / 2.0 - 1000;
        if (hypot(cx, cy) < 100) out = 0;
    }
    CHECK(n == 12 && NS == 12, "a cube shatters into %d sticks (12 edges)", n);
    CHECK(lenok, "spinning sticks keep their length (no end drawn longer than 200)");
    CHECK(out, "every piece flew away from the blow");

    /* 8. on a floor, the pieces end up lying on it */
    vpyfx_reset(); vpyfx_seed(9); vpyfx_set_gravity(0, -9800, 0); vpyfx_set_floor(1, 0, 96);
    vpyfx_shatter(&cube, &at, 0, 0, 0, 0, 1000, 0, 1500, 2048, 1000, 120);
    for (int i = 0; i < 400; i++) vpyfx_step();
    frame();
    int lying = NS > 0;
    for (int i = 0; i < NS; i++) if (S[i].y0 < -2 || S[i].y1 < -2 || (S[i].y0 > 60 && S[i].y1 > 60)) lying = 0;
    int moved = 0; seg_t snap[16]; const int ns = NS;
    for (int i = 0; i < NS && i < 16; i++) snap[i] = S[i];
    for (int i = 0; i < 50; i++) vpyfx_step();
    frame();
    for (int i = 0; i < ns && i < 16; i++) if (abs(snap[i].x0 - S[i].x0) > 2 || abs(snap[i].y0 - S[i].y0) > 2) moved = 1;
    CHECK(lying, "after 8 s every piece is down on the floor, none below it");
    CHECK(!moved, "and they have stopped moving");

    /* 9. a ring grows at its speed, in its plane, as N strokes */
    vpyfx_reset();
    vpyfx_ring(0, 0, 0, 0, 0, 1, 100, 1000, 16, 100, 120);   /* in the x-y plane */
    for (int i = 0; i < 10; i++) vpyfx_step();
    frame();
    double rmin = 1e9, rmax = 0;
    for (int i = 0; i < NS; i++) { const double r = hypot(S[i].x0, S[i].y0); if (r < rmin) rmin = r; if (r > rmax) rmax = r; }
    CHECK(NS == 16 && fabs(rmin - 300) <= 3 && fabs(rmax - 300) <= 3,
          "ring: 16 strokes, radius %.0f..%.0f after 10 steps (100 + 1000 u/s x 0.2 s = 300)", rmin, rmax);
    vpyfx_set_budget(10); frame();
    CHECK(NS == 0 && vpyfx_stats()->shed == 1, "a ring the budget cannot fit whole is left out and counted");

    /* 10. a line stays put and fades; under gravity, too */
    vpyfx_reset(); vpyfx_set_gravity(0, -9800, 0);
    vpyfx_line(0, 500, 0, 300, 500, 0, 50, 120);
    for (int i = 0; i < 20; i++) vpyfx_step();
    frame();
    CHECK(NS == 1 && S[0].y0 == 500 && S[0].y1 == 500 && S[0].br < 120, "a line does not fall, and fades (br %d)", NS ? S[0].br : -1);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
