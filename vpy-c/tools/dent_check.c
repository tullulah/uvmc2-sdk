/* dent_check — vpy3d_mesh_copy / vpy3d_mesh_dent, as tests with known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/dent_check.c vpy3d.c vpy.c \
 *      -o /tmp/dent_check && /tmp/dent_check
 *
 * Strokes are caught where libvpy hands them to the hardware, so what is checked
 * is what would be drawn. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

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

/* a box of half size h, plain or with a vertex in the middle of every face */
static void box(vpy_mesh *m, int h, int centres, int hard)
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
    vpy3d_mesh_end(hard);
}

static int strokes(const vpy_mesh *m, const vpy_xf *at)
{
    NS = 0; vpy3d_draw_mesh(m, at, 100); vpy_flush(); return NS;
}

int main(void)
{
    vpy3d_look_at(300, 400, -1500, 0, 0, 0, 0, 1, 0);
    const vpy_xf at = vpy3d_translate(0, 0, 0);
    vpy_mesh plain, centred, mine, other;
    box(&plain, 200, 0, 15826);           /* cos 15 deg */
    box(&centred, 200, 1, 15826);

    const int sp = strokes(&plain, &at), sc = strokes(&centred, &at);
    CHECK(sp == sc, "a box with face centres draws like a plain box: %d strokes vs %d", sc, sp);

    CHECK(vpy3d_mesh_copy(&mine, &centred), "a copy of its own");
    const int s0 = strokes(&mine, &at);
    vpy3d_mesh_dent(&mine, 0, 0, -200, 0, 0, 1, 80, 150);   /* the face towards the camera, pushed in */
    const int s1 = strokes(&mine, &at);
    CHECK(s1 > s0, "dented: the fold shows, %d strokes against %d", s1, s0);
    CHECK(strokes(&centred, &at) == sc, "and the shared mesh is untouched (%d)", strokes(&centred, &at));

    const vpy3d_stats_t *st = vpy3d_stats();
    const int used = st->verts;
    CHECK(vpy3d_mesh_copy(&mine, &centred) && strokes(&mine, &at) == s0, "copying again resets the dent");
    CHECK(vpy3d_stats()->verts == used, "and uses no more pool (vertices %d, still %d)", used, vpy3d_stats()->verts);
    CHECK(vpy3d_mesh_copy(&other, &centred) && vpy3d_stats()->verts > used, "a second object takes its own pool space");

    /* a dent off the face does nothing */
    vpy3d_mesh_copy(&mine, &centred);
    vpy3d_mesh_dent(&mine, 0, 0, -2000, 0, 0, 1, 80, 150);
    CHECK(strokes(&mine, &at) == s0, "a dent with nothing in reach changes nothing");

    /* world to model: undo a rotation and a translation */
    vpy_xf r = vpy3d_rot_y(1024);                          /* a quarter turn */
    r.t[0] = 500; r.t[1] = 100; r.t[2] = -300;
    int32_t mx, my, mz;
    /* model (200, 0, 0) under a quarter turn about y lands at world (0,0,-200) + t */
    vpy3d_world_to_model(&r, 500, 100, -500, &mx, &my, &mz);
    CHECK(abs(mx - 200) <= 1 && abs(my) <= 1 && abs(mz) <= 1, "world to model: (%d, %d, %d), expected (200, 0, 0)", mx, my, mz);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
