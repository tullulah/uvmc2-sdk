/* anim_check — two-bone IK and mesh morphing, against geometry.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/anim_check.c vpyik.c vpy3d.c vpy.c \
 *      -lm -o /tmp/anim_check && /tmp/anim_check
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy.h"
#include "vpy3d.h"
#include "vpyik.h"

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
static double dist(const int32_t a[3], const int32_t b[3])
{ return sqrt((double)(a[0]-b[0])*(a[0]-b[0]) + (double)(a[1]-b[1])*(a[1]-b[1]) + (double)(a[2]-b[2])*(a[2]-b[2])); }

static void box(vpy_mesh *m, int hx, int hy)
{
    int v[8];
    vpy3d_mesh_begin(m);
    for (int k = 0; k < 8; k++) v[k] = vpy3d_vertex((k & 1) ? hx : -hx, (k & 2) ? hy : -hy, (k & 4) ? 100 : -100);
    vpy3d_quad(v[0], v[4], v[6], v[2]); vpy3d_quad(v[1], v[3], v[7], v[5]);
    vpy3d_quad(v[0], v[1], v[5], v[4]); vpy3d_quad(v[2], v[6], v[7], v[3]);
    vpy3d_quad(v[0], v[2], v[3], v[1]); vpy3d_quad(v[4], v[5], v[7], v[6]);
    vpy3d_mesh_end(VPY3D_HARD_45);
}

int main(void)
{
    /* IK: reachable — bones keep their lengths, the hand is on the target, the elbow bends to the pole */
    const int32_t root[3] = { 0, 0, 0 }, target[3] = { 600, 200, 0 }, pole[3] = { 300, 800, 0 };
    int32_t j[3], e[3];
    int r = vpyik_two_bone(root, target, 400, 400, pole, j, e);
    CHECK(r == 1 && dist(e, target) <= 2, "reachable: the hand on the target (off by %.1f)", dist(e, target));
    CHECK(fabs(dist(root, j) - 400) <= 2 && fabs(dist(j, e) - 400) <= 2, "both bones keep their 400 (%.1f, %.1f)", dist(root, j), dist(j, e));
    CHECK(j[1] > 200, "the elbow bends towards the pole, up (y %d)", j[1]);
    const int32_t pole2[3] = { 300, -800, 0 };
    vpyik_two_bone(root, target, 400, 400, pole2, j, e);
    CHECK(j[1] < 0, "a pole below: the elbow bends down (y %d)", j[1]);
    /* out of reach: stretched straight towards it */
    const int32_t far[3] = { 2000, 0, 0 };
    r = vpyik_two_bone(root, far, 400, 400, pole, j, e);
    CHECK(r == 0 && abs(e[0] - 800) <= 2 && abs(e[1]) <= 2 && abs(j[0] - 400) <= 2 && abs(j[1]) <= 2,
          "out of reach: straight towards it, elbow at (%d,%d), hand at (%d,%d)", j[0], j[1], e[0], e[1]);

    /* morph: a box becoming a taller box */
    vpy3d_look_at(300, 400, -1500, 0, 0, 0, 0, 1, 0);
    const vpy_xf at = vpy3d_translate(0, 0, 0);
    vpy_mesh a, b, m, other;
    box(&a, 100, 100); box(&b, 100, 300);
    CHECK(vpy3d_mesh_copy(&m, &a), "a copy to morph");
    vpy3d_mesh_blend(&m, &a, &b, 0);     NS = 0; vpy3d_draw_mesh(&m, &at, 100); vpy_flush(); const int s0 = NS;
    NS = 0; vpy3d_draw_mesh(&a, &at, 100); vpy_flush(); const int sa = NS;
    CHECK(s0 == sa, "t = 0 draws like the first shape (%d strokes)", s0);
    int32_t ea[3], eb[3];
    vpy3d_mesh_blend(&m, &a, &b, 8192);
    /* the top corners should be half way: y = 200 */
    int ok = 0;
    for (int e2 = 0; e2 < vpy3d_mesh_edge_count(&m); e2++) { vpy3d_mesh_edge(&m, e2, ea, eb); if (ea[1] == 200 || eb[1] == 200) ok = 1; }
    CHECK(ok, "t = 0.5: the top half way between the two, at y = 200");
    box(&other, 100, 100);
    vpy_mesh tri; int v0, v1, v2;
    vpy3d_mesh_begin(&tri); v0 = vpy3d_vertex(0,0,0); v1 = vpy3d_vertex(100,0,0); v2 = vpy3d_vertex(0,100,0); vpy3d_tri(v0, v1, v2); vpy3d_mesh_end(VPY3D_HARD_45);
    CHECK(vpy3d_mesh_blend(&m, &a, &tri, 8192) == 0, "shapes that do not match: refused");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
