/* ent_check — vpyent against what it promises.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/ent_check.c vpyent.c vpy3d.c vpyphys.c \
 *      vpyimpact.c vpyfx.c vpycam.c vpy.c -lm -o /tmp/ent_check && /tmp/ent_check
 *
 * Strokes are counted where libvpy hands them to the hardware, so what is checked is
 * what would be drawn. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"
#include "vpyphys.h"
#include "vpyimpact.h"
#include "vpycam.h"
#include "vpyent.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int NS;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; NS++; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

static void box(vpy_mesh *m, int h, int centres)
{
    int v[8];
    vpy3d_mesh_begin(m);
    for (int k = 0; k < 8; k++) v[k] = vpy3d_vertex((k & 1) ? h : -h, (k & 2) ? h : -h, (k & 4) ? h : -h);
    static const int F[6][4] = { {0,4,6,2}, {1,3,7,5}, {0,1,5,4}, {2,6,7,3}, {0,2,3,1}, {4,5,7,6} };
    static const int C[6][3] = { {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1} };
    for (int f = 0; f < 6; f++) {
        if (!centres) { vpy3d_quad(v[F[f][0]], v[F[f][1]], v[F[f][2]], v[F[f][3]]); continue; }
        const int c = vpy3d_vertex(C[f][0] * h, C[f][1] * h, C[f][2] * h);
        for (int i = 0; i < 4; i++) vpy3d_tri(c, v[F[f][i]], v[F[f][(i + 1) % 4]]);
    }
    vpy3d_mesh_end(15826);
}

/* one frame of the scene: reset the occluder, draw, flush; the strokes it made */
static int frame(void) { vpy3d_occl_reset(); NS = 0; vpyent_draw(); vpy_flush(); return NS; }
static int place_ent(const vpy_mesh *m, int32_t x, int32_t y, int32_t z)
{
    const int e = vpyent_create(m, VPYP_NONE);
    vpy_xf at = vpy3d_translate(x, y, z); vpyent_set_place(e, &at);
    return e;
}

int main(void)
{
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    vpy_mesh cube, big, crate;
    box(&cube, 200, 0); box(&big, 400, 0); box(&crate, 200, 1);

    /* 1. near to far, whatever the creation order: a box behind a bigger one is hidden */
    vpyent_reset();
    int back = place_ent(&cube, 0, 0, 800);
    const int s_back = frame();
    vpyent_destroy(back);
    int front = place_ent(&big, 0, 0, -500);
    const int s_front = frame();
    back = place_ent(&cube, 0, 0, 800);                    /* created AFTER the front one */
    const int s_both = frame();
    vpyent_reset();
    back = place_ent(&cube, 0, 0, 800); front = place_ent(&big, 0, 0, -500);   /* and before */
    const int s_both2 = frame();
    CHECK(s_back > 0 && s_both == s_front && s_both2 == s_front,
          "a box behind a bigger one is hidden whichever was created first: %d + %d alone, %d and %d together",
          s_front, s_back, s_both, s_both2);
    CHECK(vpyent_stats()->drawn == 2 && vpyent_stats()->occl_missing == 0, "both drawn (%u), every occluder taken", vpyent_stats()->drawn);
    (void)front;

    /* 2. a mesh of more than 8 vertices and no shape hides nothing, and says so; a BOX does */
    vpyent_reset();
    int c1 = place_ent(&crate, 0, 0, -500); place_ent(&cube, 0, 0, 800);
    const int s_leaky = frame();
    const uint32_t missing = vpyent_stats()->occl_missing;
    vpyent_set_occluder(c1, VPYENT_OCC_BOX, 200, 200, 200);
    const int s_tight = frame();
    CHECK(missing == 1 && vpyent_stats()->occl_missing == 0 && s_tight < s_leaky,
          "a 14-vertex crate with no shape: counted as missing (%u) and see-through (%d strokes); as a BOX it hides (%d)", missing, s_leaky, s_tight);

    /* 3. dents and marks are the entity's own */
    vpyent_reset();
    int a = place_ent(&crate, -500, 0, 0), b = place_ent(&crate, 500, 0, 0);
    vpyent_set_occluder(a, VPYENT_OCC_NONE, 0, 0, 0); vpyent_set_occluder(b, VPYENT_OCC_NONE, 0, 0, 0);
    const int s_same = frame();
    vpyent_dent(a, -500, 0, -200, 0, 0, 1, 80, 200);
    const int s_dent = frame();
    vpyent_mark(b, 500, 0, -200, 0, 0, -16384);
    const int s_mark = frame();
    CHECK(s_dent > s_same, "a dent in one crate adds its folds: %d strokes, %d before", s_dent, s_same);
    CHECK(s_mark == s_dent + 6, "a mark on the other adds its ring: %d strokes (%d + 6)", s_mark, s_dent);
    vpyent_set_mesh(a, &crate);
    CHECK(frame() == s_same + 6, "giving the dented one its shared mesh back undoes the dent only");

    /* 4. destroy: the body goes with it and the slot is free again */
    vpyent_reset(); vpyp_reset();
    int body = vpyp_add_box(0, 500, 0, 200, 200, 200, 1);
    int e = vpyent_create(&cube, body);
    CHECK(vpyent_of_body(body) == e, "the entity of body %d is %d", body, e);
    vpyent_destroy(e);
    CHECK(!vpyent_alive(e) && !vpyp_alive(body) && vpyent_create(&cube, VPYP_NONE) == e,
          "destroyed: entity gone, its body removed, the slot taken by the next create");

    /* 5. a full table refuses and counts */
    vpyent_reset();
    int made = 0; for (int i = 0; i < VPYENT_MAX + 3; i++) made += vpyent_create(&cube, VPYP_NONE) >= 0;
    CHECK(made == VPYENT_MAX && vpyent_stats()->refused == 3, "%d made of %d asked, %u refused", made, VPYENT_MAX + 3, vpyent_stats()->refused);

    /* 6. the step: physics falls, the landing sounds, a hit-stop holds time */
    int32_t y_run[2];
    for (int run = 0; run < 2; run++) {
        vpyent_reset(); vpyp_reset(); vpyimpact_reset(); vpycam_reset(0, 0, 0);
        vpyp_set_gravity(0, -9800, 0); vpyp_set_floor(1, 0, 40, 150);
        body = vpyp_add_box(0, 1200, 0, 130, 130, 130, 2);
        e = vpyent_create(&cube, body); vpyent_set_material(e, VPYI_WOOD);
        vpyent_set_occluder(e, VPYENT_OCC_BOX, 130, 130, 130);
        int shadow_high = -1, shadow_low = -1;
        for (int i = 0; i < 120; i++) {
            vpyent_step(VPYI_NONE);
            if (i == 2)   shadow_high = vpyent_draw_shadows(300, -1000, 200, 0, 60, 35);
            if (i == 119) shadow_low  = vpyent_draw_shadows(300, -1000, 200, 0, 60, 35);
        }
        int32_t x, y, z; vpyp_position(body, &x, &y, &z); y_run[run] = y;
        if (run == 0) {
            CHECK(y < 140 && vpyimpact_stats()->played >= 1, "a crate falls to the floor (y=%d) and its landing sounds (%u hits)", y, vpyimpact_stats()->played);
            CHECK(shadow_high == 1 && shadow_low == 0, "a shadow while it falls (%d), none once it rests (%d)", shadow_high, shadow_low);
            vpycam_hitstop(3);
            int held = 0; for (int i = 0; i < 3; i++) held += vpyent_step(VPYI_NONE) == 0;
            int32_t y2; vpyp_position(body, &x, &y2, &z);
            CHECK(held == 3 && y2 == y, "a hit-stop of 3 holds time for 3 steps (%d)", held);
        }
    }
    CHECK(y_run[0] == y_run[1], "deterministic: the same fall twice (y=%d, %d)", y_run[0], y_run[1]);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
