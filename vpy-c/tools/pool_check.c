/* pool_check — vpy3d's shared pools: what happens when they are full, and giving
 * them back (vpy3d_pool_mark / vpy3d_pool_release).
 *
 *   cc -O2 -w -DVPY3D_POOL_V=40 -DVPY3D_POOL_F=30 -DVPY3D_POOL_FV=120 -DVPY3D_POOL_E=60 \
 *      -Iinclude -I../pitrex-sim/include tools/pool_check.c vpy3d.c vpy.c \
 *      -o /tmp/pool_check && /tmp/pool_check
 *
 * The pools are made small on purpose: five 8-vertex boxes fit, the sixth does not.
 * Strokes are caught where libvpy hands them to the hardware, so "draws the same"
 * is checked on what would be drawn. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int NS;
static int32_t SUM;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)e; NS++; SUM += a * 3 + b * 5 + c * 7 + d * 11; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* a box built the way a game writes one: no check on what vpy3d_vertex returns */
static int box(vpy_mesh *m, int h)
{
    int v[8];
    vpy3d_mesh_begin(m);
    for (int k = 0; k < 8; k++) v[k] = vpy3d_vertex((k & 1) ? h : -h, (k & 2) ? h : -h, (k & 4) ? h : -h);
    vpy3d_quad(v[0], v[4], v[6], v[2]); vpy3d_quad(v[1], v[3], v[7], v[5]);
    vpy3d_quad(v[0], v[1], v[5], v[4]); vpy3d_quad(v[2], v[6], v[7], v[3]);
    vpy3d_quad(v[0], v[2], v[3], v[1]); vpy3d_quad(v[4], v[5], v[7], v[6]);
    return vpy3d_mesh_end(VPY3D_HARD_45);
}

/* what drawing it sends: the stroke count and a sum over the coordinates */
static void drawn(const vpy_mesh *m, int *n, int32_t *sum)
{
    const vpy_xf at = vpy3d_translate(0, 0, 4000);
    vpy_flush();
    NS = 0; SUM = 0;
    vpy3d_draw_mesh(m, &at, 100);
    vpy_flush();
    *n = NS; *sum = SUM;
}

int main(void)
{
    vpy3d_look_at(1500, 1200, 0, 0, 0, 4000, 0, 1, 0);
    const vpy3d_stats_t *st = vpy3d_stats();

    printf("FULL POOLS: the build that does not fit fails safe\n");
    static vpy_mesh b[7];
    int built = 0;
    for (int i = 0; i < 5; i++) built += box(&b[i], 300 + i * 50);
    CHECK(built == 5 && st->overflow == 0, "five boxes fit (%d built, overflow %u)", built, (unsigned)st->overflow);
    int n0; int32_t s0; drawn(&b[0], &n0, &s0);
    CHECK(n0 > 0, "the first draws (%d strokes)", n0);
    const int r = box(&b[5], 500);                 /* 40 vertices are taken: none left */
    CHECK(r == 0, "the sixth reports failure (mesh_end %d)", r);
    CHECK(st->overflow > 0, "and overflow counts it (%u)", (unsigned)st->overflow);
    CHECK(b[5].nv == 0 && b[5].nf == 0 && b[5].ne == 0, "it is left empty (nv %d nf %d ne %d)", b[5].nv, b[5].nf, b[5].ne);
    int n5; int32_t s5; drawn(&b[5], &n5, &s5);
    CHECK(n5 == 0, "and draws nothing (%d strokes), without reading out of the pools", n5);
    int n0b; int32_t s0b; drawn(&b[0], &n0b, &s0b);
    CHECK(n0b == n0 && s0b == s0, "the first still draws the same");
    vpy_mesh e;
    CHECK(vpy3d_mesh_begin(&e) == 0, "mesh_begin on full pools returns 0, as the header says");
    vpy3d_mesh_end(VPY3D_HARD_45);

    printf("A FACE THAT NAMES NO VERTEX is refused\n");
    {
        /* a fresh run of the pools is not possible in one process: release to empty */
        const vpy3d_pool_mark_t zero = { 0, 0, 0, 0 };
        CHECK(vpy3d_pool_release(zero), "release to empty");
        const uint32_t before = st->overflow;
        vpy_mesh m;
        vpy3d_mesh_begin(&m);
        const int a = vpy3d_vertex(0, 0, 0), bb = vpy3d_vertex(100, 0, 0), c = vpy3d_vertex(0, 100, 0);
        CHECK(vpy3d_tri(a, bb, -1) < 0, "a -1 index (a vertex that did not fit) is refused");
        CHECK(vpy3d_tri(a, bb, 7) < 0, "an index past the mesh's vertices is refused");
        CHECK(vpy3d_tri(a, bb, c) >= 0, "three good ones are taken");
        CHECK(vpy3d_mesh_end(VPY3D_HARD_45) == 0 && st->overflow == before + 2, "the build reports it (overflow +%u)", (unsigned)(st->overflow - before));
    }

    printf("MARK AND RELEASE: a level's meshes given back, what lasts untouched\n");
    {
        const vpy3d_pool_mark_t zero = { 0, 0, 0, 0 };
        vpy3d_pool_release(zero);
        const uint32_t ov = st->overflow;
        static vpy_mesh keep1, keep2, lvl[3];
        box(&keep1, 400); box(&keep2, 250);
        int k1n; int32_t k1s; drawn(&keep1, &k1n, &k1s);
        const vpy3d_pool_mark_t mk = vpy3d_pool_mark();
        CHECK(mk.v == 16, "the mark is after the two that last (v %u)", mk.v);
        /* three levels, one after the other, each filling what is left */
        int all = 1;
        for (int l = 0; l < 3; l++) {
            CHECK(vpy3d_pool_release(mk), "level %d: released to the mark", l + 1);
            for (int i = 0; i < 3; i++) all &= box(&lvl[i], 200 + l * 100 + i * 30);
            CHECK(lvl[0].v0 == mk.v, "level %d's first mesh starts at the mark (v0 %u)", l + 1, lvl[0].v0);
        }
        CHECK(all && st->overflow == ov, "three levels of three boxes each in pools that hold five boxes (overflow +%u)", (unsigned)(st->overflow - ov));
        int k1nb; int32_t k1sb; drawn(&keep1, &k1nb, &k1sb);
        CHECK(k1nb == k1n && k1sb == k1s, "what lasts draws the same after three releases");
        int ln; int32_t ls; drawn(&lvl[2], &ln, &ls);
        CHECK(ln > 0, "the last level's mesh draws (%d strokes)", ln);
        /* refusals */
        vpy3d_pool_release(mk);                        /* room for a build to be under way */
        vpy_mesh m; vpy3d_mesh_begin(&m); vpy3d_vertex(0, 0, 0);
        CHECK(!vpy3d_pool_release(mk), "refused in the middle of a build");
        vpy3d_mesh_end(VPY3D_HARD_45);
        vpy3d_pool_release(mk);
        const vpy3d_pool_mark_t later = { (uint16_t)(mk.v + 8), mk.f, mk.fv, mk.e };
        CHECK(!vpy3d_pool_release(later), "refused for a mark past where the pools are");
        CHECK(st->verts >= 40, "the high-water mark is not lowered (verts %u)", st->verts);
    }

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
