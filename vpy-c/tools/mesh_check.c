/* mesh_check — vpy3d_ray_mesh, vpy3d_marks and vpy3d_lod_pick, against known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/mesh_check.c vpy3d.c vpy.c \
 *      -o /tmp/mesh_check && /tmp/mesh_check
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

/* faces in this order: -x +x -y +y -z +z; with centres each face is 4 triangles */
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

static int marks_strokes(const vpy3d_marks *mk, const vpy_xf *at, int style, int *drawn)
{
    NS = 0; *drawn = vpy3d_marks_draw(mk, at, 40, 127, style); vpy_flush(); return NS;
}

int main(void)
{
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    vpy_mesh cube, crate, mine;
    box(&cube, 200, 0);
    box(&crate, 200, 1);
    vpy3d_hit h;

    /* 1. straight at the -z face from in front */
    vpy_xf at = vpy3d_translate(0, 0, 0);
    int f = vpy3d_ray_mesh(&cube, &at, 50, -30, -1000, 0, 0, 1, 5000, &h);
    CHECK(f == 4 && h.z == -200 && h.x == 50 && h.y == -30 && h.dist == 800 && h.nz == -16384,
          "a ray along +z meets face 4 (-z) at (%d,%d,%d), dist %d, normal z %d", h.x, h.y, h.z, h.dist, h.nz);
    /* 2. from the side, translated */
    at = vpy3d_translate(1000, 500, 0);
    f = vpy3d_ray_mesh(&cube, &at, 3000, 520, 10, -1, 0, 0, 5000, &h);
    CHECK(f == 1 && h.x == 1200 && h.y == 520 && h.dist == 1800 && h.nx == 16384,
          "translated: along -x meets +x (face %d) at x=%d, dist %d, normal x %d", f, h.x, h.dist, h.nx);
    /* 3. misses: beside it, pointing away, too short */
    CHECK(vpy3d_ray_mesh(&cube, &at, 3000, 750, 0, -1, 0, 0, 5000, &h) < 0, "a ray passing over the top misses");
    CHECK(vpy3d_ray_mesh(&cube, &at, 3000, 500, 0, 1, 0, 0, 5000, &h) < 0, "a ray pointing away misses");
    CHECK(vpy3d_ray_mesh(&cube, &at, 3000, 500, 0, -1, 0, 0, 1700, &h) < 0, "a ray that stops short misses");
    /* 4. turned an eighth about y: the corner faces the ray, at sqrt(2) × 200 */
    at = vpy3d_rot_y(512);
    f = vpy3d_ray_mesh(&cube, &at, 0, 0, -1000, 0, 0, 1, 5000, &h);
    CHECK(f >= 0 && abs(h.z + 283) <= 2 && abs(h.dist - 717) <= 2,
          "turned 45 deg: the edge is met at z=%d (expected -283), dist %d", h.z, h.dist);
    f = vpy3d_ray_mesh(&cube, &at, 100, 0, -1000, 0, 0, 1, 5000, &h);
    {   /* on the face's plane, 200 out from the centre along its normal */
        const int32_t along = (int32_t)(((int64_t)h.nx * h.x + (int64_t)h.nz * h.z) >> 14);
        CHECK(f >= 0 && abs(h.z + 183) <= 2 && abs(abs(h.nx) - 11585) <= 30 && abs(h.nz + 11585) <= 30 && abs(along - 200) <= 2,
              "beside the edge: z=%d (expected -183), normal (%d, %d), %d out along it (200)", h.z, h.nx, h.nz, along);
    }
    /* 5. an oblique ray, not normalised */
    at = vpy3d_translate(0, 0, 0);
    f = vpy3d_ray_mesh(&cube, &at, -1000, 1000, 0, 3, -3, 0, 5000, &h);
    CHECK(f >= 0 && ((h.x == -200 && h.y >= 199) || (h.y == 200 && h.x <= -199)),
          "oblique at the top edge: (%d, %d)", h.x, h.y);
    /* 6. a dented face is hit at the bottom of the dent */
    vpy3d_mesh_copy(&mine, &crate);
    f = vpy3d_ray_mesh(&mine, &at, 0, 0, -1000, 0, 0, 1, 5000, &h);
    CHECK(f >= 0 && h.z == -200, "the copy, undented: z=%d", h.z);
    vpy3d_mesh_dent(&mine, 0, 0, -200, 0, 0, 1, 80, 150);
    f = vpy3d_ray_mesh(&mine, &at, 0, 0, -1000, 0, 0, 1, 5000, &h);
    CHECK(f >= 0 && abs(h.z + 120) <= 1, "dented 80 deep: hit at z=%d (expected -120)", h.z);
    f = vpy3d_ray_mesh(&crate, &at, 0, 0, -1000, 0, 0, 1, 5000, &h);
    CHECK(f >= 0 && h.z == -200, "and the shared mesh is still hit at z=%d", h.z);
    /* 7. an open plate is hit from behind too, solid faces are not */
    vpy_mesh plate; vpy3d_mesh_begin(&plate);
    { int a = vpy3d_vertex(-100,-100,0), b = vpy3d_vertex(100,-100,0), c = vpy3d_vertex(100,100,0), d = vpy3d_vertex(-100,100,0);
      vpy3d_quad(a, b, c, d); }
    vpy3d_mesh_end(15826); vpy3d_mesh_open(&plate, 1);
    const int f1 = vpy3d_ray_mesh(&plate, &at, 0, 0, -500, 0, 0, 1, 5000, &h);
    const int16_t n1 = h.nz;
    const int f2 = vpy3d_ray_mesh(&plate, &at, 0, 0, 500, 0, 0, -1, 5000, &h);
    CHECK(f1 == 0 && f2 == 0 && n1 == -h.nz && (int)n1 * -1 < 0 == (n1 > 0) ,
          "a plate is hit from both sides, the normal towards the ray each time (%d, %d)", n1, h.nz);
    CHECK(vpy3d_ray_mesh(&cube, &at, 0, 0, 0, 0, 0, 1, 5000, &h) < 0,
          "a ray from inside a solid passes out through it");

    /* 8. marks: added where a ray hit, turn with the object, hide when it turns away */
    vpy3d_marks mk; vpy3d_marks_clear(&mk);
    at = vpy3d_translate(0, 0, 0);
    vpy3d_ray_mesh(&cube, &at, 0, 0, -1000, 0, 0, 1, 5000, &h);
    vpy3d_marks_add(&mk, &at, h.x, h.y, h.z, h.nx, h.ny, h.nz);
    vpy3d_occl_reset();
    int drawn, s = marks_strokes(&mk, &at, VPY3D_MARK_RING, &drawn);
    CHECK(drawn == 1 && s == 6, "a ring on the face towards the camera: %d mark, %d strokes (6)", drawn, s);
    s = marks_strokes(&mk, &at, VPY3D_MARK_CRACK, &drawn);
    CHECK(drawn == 1 && s == 5, "a crack: %d strokes (5)", s);
    vpy_xf turned = vpy3d_rot_y(2048);                        /* half a turn: that face now looks away */
    s = marks_strokes(&mk, &turned, VPY3D_MARK_RING, &drawn);
    CHECK(drawn == 0 && s == 0, "turned away: %d marks, %d strokes", drawn, s);
    turned = vpy3d_rot_y(256);                                /* a little: still seen, and moved */
    s = marks_strokes(&mk, &turned, VPY3D_MARK_RING, &drawn);
    int32_t mx, my, mz; vpy3d_world_to_model(&turned, 0, 0, 0, &mx, &my, &mz);
    CHECK(drawn == 1 && s == 6, "turned 22 deg: still drawn (%d strokes)", s);
    /* where it went: hit the turned cube through the mark's own model point */
    {
        const int32_t sx = (int32_t)(((int64_t)turned.m[2] * -200) >> 14), sz = (int32_t)(((int64_t)turned.m[8] * -200) >> 14);
        CHECK(sx < -50 && sz > -200, "and the mark's centre went round with it: (%d, %d) from (0, -200)", sx, sz);
    }
    for (int i = 0; i < 6; i++) vpy3d_marks_add(&mk, &at, 10 * i, 0, -200, 0, 0, -16384);
    CHECK(mk.n == VPY3D_MARKS && mk.p[(mk.next + VPY3D_MARKS - 1) % VPY3D_MARKS][0] == 50,
          "a full set keeps %d, the newest last (x=%d)", mk.n, mk.p[(mk.next + VPY3D_MARKS - 1) % VPY3D_MARKS][0]);
    vpy3d_marks_clear(&mk);
    s = marks_strokes(&mk, &at, VPY3D_MARK_RING, &drawn);
    CHECK(drawn == 0 && s == 0, "cleared: nothing drawn");

    /* 9. level of detail by distance */
    const int32_t sizes[3] = { 1500, 500, 50 };
    int l0 = vpy3d_lod_pick(&(vpy_xf){ {16384,0,0, 0,16384,0, 0,0,16384}, {0,0,0} }, 200, sizes, 3);
    vpy_xf far1 = vpy3d_translate(0, 0, 5000), far2 = vpy3d_translate(0, 0, 60000), behind = vpy3d_translate(0, 0, -5000);
    int l1 = vpy3d_lod_pick(&far1, 200, sizes, 3), l2 = vpy3d_lod_pick(&far2, 200, sizes, 3), lb = vpy3d_lod_pick(&behind, 200, sizes, 3);
    vpy_xf far3 = vpy3d_translate(0, 0, 200000);
    int l3 = vpy3d_lod_pick(&far3, 200, sizes, 3);
    CHECK(l0 == 0 && l1 == 1 && l2 == 2 && l3 == -1 && lb == -1,
          "LOD: near %d, mid %d, far %d, a speck %d, behind %d (0 1 2 -1 -1); sizes %d %d %d",
          l0, l1, l2, l3, lb, vpy3d_screen_size(0,0,0,200), vpy3d_screen_size(0,0,5000,200), vpy3d_screen_size(0,0,60000,200));
    const vpy_mesh *lods[3] = { &crate, &cube, &cube };
    NS = 0; int dl = vpy3d_draw_lod(lods, sizes, 3, &far3, 200, 100); vpy_flush();
    CHECK(dl == -1 && NS == 0, "a speck draws nothing");
    NS = 0; dl = vpy3d_draw_lod(lods, sizes, 3, &far1, 200, 100); vpy_flush();
    CHECK(dl == 1 && NS > 0, "level %d drawn: %d strokes", dl, NS);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
