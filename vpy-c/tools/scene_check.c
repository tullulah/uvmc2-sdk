/* scene_check — a .vscene drawn by vpy3d_draw_scene, against the same meshes
 * placed by hand with vpy3d_draw_mesh.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/scene_check.c vpy3d.c vpy.c \
 *      -o /tmp/scene_check && /tmp/scene_check
 *
 * The images are written byte by byte in the layout vsceneres.rs (scene) and
 * vmeshres.rs (meshes) document. Strokes are caught where libvpy hands them to
 * the hardware. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
#define MAXS 4096
static int32_t S[MAXS][5]; static int NS;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e)
{ if (NS < MAXS) { S[NS][0] = a; S[NS][1] = b; S[NS][2] = c; S[NS][3] = d; S[NS][4] = e; } NS++; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* ---- image writers ---- */
static unsigned char bufs[16][2048];
static unsigned char *img;
static int at, nimg;
static void u8(int v) { img[at++] = (unsigned char)v; }
static void u16(int v) { u8(v & 255); u8((v >> 8) & 255); }
static void i32(int32_t v) { u16(v & 0xffff); u16((v >> 16) & 0xffff); }
static unsigned char *begin(void) { img = bufs[nimg++ % 16]; at = 0; memset(img, 0, sizeof bufs[0]); return img; }
static void pt(int32_t x, int32_t y, int32_t z) { i32(x); i32(y); i32(z); }

/* a scenery-only mesh: `n` horizontal lines 400 wide, 200 apart, at z 0 */
static const unsigned char *lines_mesh(int n)
{
    unsigned char *m = begin();
    u8('V'); u8('M'); u8('S'); u8('H'); u8(2); u8(0); u16(11585); u16(0); u16(0); u16(0); u16(0);
    u16(n);
    for (int k = 0; k < n; k++) { u8(2); u8(100); u8(0); pt(-200, k * 200, 0); pt(200, k * 200, 0); }
    u16(0);
    return m;
}
/* a scenery-only mesh: one wall occluder 4000 square at z 0 */
static const unsigned char *wall_mesh(void)
{
    unsigned char *m = begin();
    u8('V'); u8('M'); u8('S'); u8('H'); u8(2); u8(0); u16(11585); u16(0); u16(0); u16(0); u16(0);
    u16(0);
    u16(1); u8(4); pt(-2000, -2000, 0); pt(2000, -2000, 0); pt(2000, 2000, 0); pt(-2000, 2000, 0);
    return m;
}

static unsigned char *scene;
static void scene_head(int nmesh, int nitems, int32_t full, int32_t gone)
{
    scene = begin();
    u8('V'); u8('S'); u8('C'); u8('N'); u8(1); u8(nmesh); u16(nitems); i32(full); i32(gone);
}
/* flags: 1 repeat, 2 no fog, 4 window, 8 group negated */
static void item(int flags, int group, int pct, int axis, int32_t ax, int32_t ay, int32_t az,
                 int32_t fx, int32_t fy, int32_t fz, int32_t step, int count, int32_t wmin, int32_t wmax,
                 int nmesh, const int *idx)
{
    u8(flags); u8(group); u8(pct); u8(axis);
    i32(ax); i32(ay); i32(az); i32(fx); i32(fy); i32(fz);
    i32(step); u8(count); i32(wmin); i32(wmax);
    u8(nmesh); for (int k = 0; k < nmesh; k++) u8(idx[k]);
}

/* strokes of one draw, kept for comparing */
static int32_t A[MAXS][5]; static int NA;
static int run_scene(const vpy_scene *s, const vpy_xf *place, int32_t px, int32_t py, int32_t pz, uint32_t g)
{
    int32_t pos[3] = { px, py, pz };
    vpy3d_occl_reset(); NS = 0;
    vpy3d_draw_scene(s, place, pos, g, 100);
    vpy_flush();
    NA = NS < MAXS ? NS : MAXS; memcpy(A, S, sizeof(S[0]) * NA);
    return NS;
}
static int same_as_scene(void) { return NS == NA && !memcmp(A, S, sizeof(S[0]) * NA); }

#define Q16 65536

int main(void)
{
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    const unsigned char *one = lines_mesh(1), *two = lines_mesh(2), *wall = wall_mesh();
    const unsigned char *meshes[2] = { one, two };
    vpy_scene s;
    vpy_mesh m_one, m_two;                       /* the same pieces, for drawing by hand */
    vpy3d_load_mesh(&m_one, one);
    vpy3d_load_mesh(&m_two, two);
    const int i0 = 0, i1 = 1, i01[2] = { 0, 1 };

    /* 1. a repeat coming at you: copies at at + n*step, the first within a step past at */
    scene_head(1, 1, 0, 0);
    item(1, 255, 100, 2, 0, 0, 1000, 0, 0, -Q16, 1000, 3, 0, 0, 1, &i0);
    CHECK(vpy3d_load_scene(&s, scene, meshes), "a one-item scene loads (error %d)", vpy3d_error());
    int n = run_scene(&s, 0, 0, 0, 1500, 0);
    vpy3d_occl_reset(); NS = 0;
    for (int z = 500; z <= 2500; z += 1000) { vpy_xf t = vpy3d_translate(0, 0, z); vpy3d_draw_mesh(&m_one, &t, 100); }
    vpy_flush();
    CHECK(n == 3 && same_as_scene(), "travelled 1500: copies at z 500, 1500, 2500 (%d strokes, same as by hand: %d)", n, same_as_scene());

    /* 2. two meshes taking turns, numbered along the whole run (also behind the origin) */
    scene_head(2, 1, 0, 0);
    item(1, 255, 100, 2, 0, 0, 1000, 0, 0, -Q16, 1000, 3, 0, 0, 2, i01);
    vpy3d_load_scene(&s, scene, meshes);
    n = run_scene(&s, 0, 0, 0, 1500, 0);
    CHECK(n == 2 + 1 + 2, "copies 1, 2, 3 use two, one, two lines: %d strokes", n);
    n = run_scene(&s, 0, 0, 0, 2500, 0);
    CHECK(n == 1 + 2 + 1, "one step on, copies 2, 3, 4: one, two, one: %d", n);
    n = run_scene(&s, 0, 0, 0, -1500, 0);
    CHECK(n == 1 + 2 + 1, "behind the origin, copies -2, -1, 0: one, two, one: %d", n);

    /* 3. a window on z */
    scene_head(1, 1, 0, 0);
    item(1 | 4, 255, 100, 2, 0, 0, 1000, 0, 0, -Q16, 1000, 3, 0, 1999, 1, &i0);
    vpy3d_load_scene(&s, scene, meshes);
    n = run_scene(&s, 0, 0, 0, 0, 0);
    CHECK(n == 1, "window max 1999: only the copy at 1000 (%d)", n);

    /* 4. groups: item 0 in group 3, item 1 in "!3" */
    scene_head(2, 2, 0, 0);
    item(0, 3, 100, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, &i0);
    item(8, 3, 100, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, &i1);
    vpy3d_load_scene(&s, scene, meshes);
    n = run_scene(&s, 0, 0, 0, 0, 1u << 3);
    CHECK(n == 1, "group 3 on: the group's item only (%d)", n);
    n = run_scene(&s, 0, 0, 0, 0, 0);
    CHECK(n == 2, "group 3 off: the \"!\" item only (%d)", n);

    /* 5. follow by a fraction, then place, against the same by hand */
    scene_head(1, 1, 0, 0);
    item(0, 255, 50, 2, 100, 0, 500, Q16 / 2, 0, 0, 0, 1, 0, 0, 1, &i0);
    const unsigned char *m2[1] = { two };
    CHECK(vpy3d_load_scene(&s, scene, m2), "a scene of the two-line mesh loads");
    vpy_xf place = vpy3d_translate(0, 300, 200);
    n = run_scene(&s, &place, 1000, 0, 0, 0);
    vpy3d_occl_reset(); NS = 0;
    { vpy_xf t = vpy3d_translate(600, 300, 700); vpy3d_draw_mesh(&m_two, &t, 50); }
    vpy_flush();
    CHECK(n == 2 && same_as_scene(), "follow 0.5 of x 1000, placed: as at (600, 300, 700) at br 50 (%d)", n);

    /* 6. an item's occluders hide the items after it, not the ones before */
    scene_head(2, 3, 0, 0);
    item(0, 255, 100, 2, 0, 0, 2000, 0, 0, 0, 0, 1, 0, 0, 1, &i0);   /* behind the wall, before it */
    item(0, 255, 100, 2, 0, 0, 1000, 0, 0, 0, 0, 1, 0, 0, 1, &i1);   /* the wall: table entry 1 */
    item(0, 255, 100, 2, 0, 0, 3000, 0, 0, 0, 0, 1, 0, 0, 1, &i0);   /* behind it, after it */
    const unsigned char *mw[2] = { one, wall };
    CHECK(vpy3d_load_scene(&s, scene, mw), "a scene with a wall loads");
    n = run_scene(&s, 0, 0, 0, 0, 0);
    CHECK(n == 1, "the line listed before the wall is drawn, the one after is hidden (%d)", n);

    /* 7. the scene's fog is its own; the game's fog is back afterwards */
    scene_head(1, 2, 1000, 2000);
    item(0, 255, 100, 2, 0, 0, 5000, 0, 0, 0, 0, 1, 0, 0, 1, &i0);   /* past gone: faded out */
    item(2, 255, 100, 2, 0, 0, 5000, 0, 0, 0, 0, 1, 0, 0, 1, &i0);   /* fog: false */
    vpy3d_load_scene(&s, scene, meshes);
    vpy3d_set_scenery_fog(100000, 200000);
    n = run_scene(&s, 0, 0, 0, 0, 0);
    CHECK(n == 1, "the fogged copy is gone, the fog:false one is drawn (%d)", n);
    vpy3d_occl_reset(); NS = 0;
    { vpy_xf t = vpy3d_translate(0, 0, 5000); vpy3d_draw_mesh(&m_one, &t, 100); }
    vpy_flush();
    CHECK(NS == 1 && S[0][4] == 100, "after it, the game's own fog again: drawn at full (br %d)", NS ? S[0][4] : -1);
    vpy3d_set_scenery_fog(0, 0);

    /* 7b. fog by the whole copy: one fade from its origin's depth, as vpy3d_fog */
    scene_head(1, 1, 1000, 6000);
    item(16, 255, 100, 2, 0, 0, 500, 0, 0, 0, 0, 1, 0, 0, 1, &i0);    /* origin depth 3500 */
    const unsigned char *m1[1] = { two };
    vpy3d_load_scene(&s, scene, m1);
    n = run_scene(&s, 0, 0, 0, 0, 0);
    int want = vpy3d_fog(100, 0, 0, 500, 1000, 6000);
    CHECK(n == 2 && A[0][4] == want && A[1][4] == want, "both lines at the copy's one fade, %d (got %d, %d)", want, A[0][4], A[1][4]);
    scene_head(1, 1, 1000, 6000);
    item(16, 255, 100, 2, 0, 0, 2950, 0, 0, 0, 0, 1, 0, 0, 1, &i0);   /* origin depth 5950: br 1 */
    vpy3d_load_scene(&s, scene, m1);
    n = run_scene(&s, 0, 0, 0, 0, 0);
    CHECK(n == 0, "a copy faded below 4 is skipped whole (%d)", n);

    /* 7c. a piece with faces: built once into a shared slot, reused on a reload */
    {
        static unsigned char solids[VPY3D_SCENE_SOLIDS + 1][64];
        const unsigned char *sm[VPY3D_SCENE_SOLIDS + 1];
        for (int j = 0; j <= VPY3D_SCENE_SOLIDS; j++) {     /* a square plate, each its own image */
            img = solids[j]; at = 0;
            u8('V'); u8('M'); u8('S'); u8('H'); u8(1); u8(1); u16(11585); u16(4); u16(1); u16(4); u16(0);
            for (int k = 0; k < 4; k++) { u16((k & 1) ? 300 : -300); u16((k & 2) ? 300 : -300); u16(0); }
            u8(4); u16(0); u16(1); u16(3); u16(2);
            sm[j] = solids[j];
        }
        scene_head(1, 1, 0, 0);
        item(0, 255, 100, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, &i0);
        int ok = 1;
        for (int r = 0; r < 20 && ok; r++) ok = vpy3d_load_scene(&s, scene, sm);
        CHECK(ok, "a scene with a solid piece loads 20 times over (one slot, reused)");
        n = run_scene(&s, 0, 0, 0, 0, 0);
        vpy3d_occl_reset(); NS = 0;
        { vpy_mesh plate; vpy3d_load_mesh(&plate, sm[0]); vpy_xf t = vpy3d_identity(); vpy3d_draw_mesh(&plate, &t, 100); }
        vpy_flush();
        CHECK(n == 4 && same_as_scene(), "and draws as the plate by hand (%d strokes)", n);
        int loaded = 1;
        for (int j = 1; j <= VPY3D_SCENE_SOLIDS; j++) {
            ok = vpy3d_load_scene(&s, scene, &sm[j]);
            if (ok) loaded++;
        }
        CHECK(loaded == VPY3D_SCENE_SOLIDS && vpy3d_error() == VPY3D_ERR_POOL,
              "%d different solids fill the %d slots, the next is refused (error %d)", loaded, VPY3D_SCENE_SOLIDS, vpy3d_error());
    }

    /* 8. bad images are refused */
    scene_head(1, 1, 0, 0);
    item(0, 255, 100, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, &i1);       /* index 1 of a 1-mesh table */
    CHECK(!vpy3d_load_scene(&s, scene, meshes) && vpy3d_error() == VPY3D_ERR_DATA, "a mesh index past the table");
    scene[4] = 2;
    CHECK(!vpy3d_load_scene(&s, scene, meshes) && vpy3d_error() == VPY3D_ERR_VERSION, "version 2 is refused");
    scene[0] = 'X';
    CHECK(!vpy3d_load_scene(&s, scene, meshes) && vpy3d_error() == VPY3D_ERR_MAGIC, "not a scene");
    scene_head(1, 1, 0, 0);
    item(1, 255, 100, 2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 1, &i0);       /* a repeat with step 0 */
    CHECK(!vpy3d_load_scene(&s, scene, meshes), "a repeat with step 0");
    scene_head(1, 1, 0, 0);
    item(0, 255, 100, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, &i0);
    const unsigned char *bad[1] = { scene };                             /* not a mesh */
    CHECK(!vpy3d_load_scene(&s, scene, bad) && vpy3d_error() == VPY3D_ERR_MAGIC, "a mesh that does not load");

    printf("%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails;
}
