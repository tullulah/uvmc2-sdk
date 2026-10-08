/* scenery_check — a version-2 .vmesh's lines and occluders, drawn by vpy3d_draw_mesh.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/scenery_check.c vpy3d.c vpy.c \
 *      -o /tmp/scenery_check && /tmp/scenery_check
 *
 * The images are written here byte by byte in the layout vmeshres.rs documents, so
 * the loader is checked against the format, not against itself. Strokes are caught
 * where libvpy hands them to the hardware. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int NS, MAXBR;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e)
{ (void)a; (void)b; (void)c; (void)d; NS++; if (e > MAXBR) MAXBR = e; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* ---- an image writer, in the layout of vmeshres.rs ---- */
/* Each image gets its own buffer: a mesh reads its scenery in place, so reusing one
 * buffer would change a mesh already loaded from it. */
static unsigned char bufs[12][4096];
static unsigned char *img = bufs[0];
static int at, nimg;
static void u8(int v) { img[at++] = (unsigned char)v; }
static void u16(int v) { u8(v & 255); u8((v >> 8) & 255); }
static void i32(int32_t v) { u16(v & 0xffff); u16((v >> 16) & 0xffff); }
static void header(int version, int nv, int nf, int nidx)
{
    img = bufs[nimg++ % 12]; at = 0; memset(img, 0, sizeof bufs[0]);
    u8('V'); u8('M'); u8('S'); u8('H'); u8(version); u8(0); u16(11585);
    u16(nv); u16(nf); u16(nidx); u16(0);
}
static void pt(int32_t x, int32_t y, int32_t z) { i32(x); i32(y); i32(z); }
/* a square wall of half-size h at depth z, as an occluder (4 corners) */
static void wall(int32_t h, int32_t z) { u8(4); pt(-h, -h, z); pt(h, -h, z); pt(h, h, z); pt(-h, h, z); }

static int strokes(void) { NS = 0; MAXBR = 0; vpy_flush(); return NS; }

int main(void)
{
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    vpy_xf here = vpy3d_translate(0, 0, 0);
    vpy_mesh a, b, c;

    /* 1. scenery only: one open line of 2 segments, one closed triangle (3 segments) */
    header(2, 0, 0, 0);
    u16(2);
    u8(3); u8(100); u8(0); pt(-500, 0, 0); pt(0, 300, 0); pt(500, 0, 0);
    u8(3); u8(100); u8(1); pt(-200, -400, 0); pt(200, -400, 0); pt(0, -100, 0);
    u16(0);
    CHECK(vpy3d_load_mesh(&a, img), "a scenery-only image loads (error %d)", vpy3d_error());
    CHECK(a.nv == 0 && a.nf == 0 && a.scenery, "it has no faces and keeps its scenery");
    vpy3d_occl_reset();
    vpy3d_draw_mesh(&a, &here, 100);
    int n = strokes();
    CHECK(n == 5, "2 + 3 segments drawn: %d strokes", n);

    /* 2. a line's percent of br */
    header(2, 0, 0, 0);
    u16(1); u8(2); u8(50); u8(0); pt(-500, 0, 0); pt(500, 0, 0);
    u16(0);
    vpy3d_load_mesh(&b, img);
    vpy3d_occl_reset();
    vpy3d_draw_mesh(&b, &here, 100);
    n = strokes();
    CHECK(n == 1 && MAXBR == 50, "50%% of br 100 draws at %d (%d stroke)", MAXBR, n);

    /* 3. a mesh's own occluder hides what is drawn AFTER it, never its own lines */
    header(2, 0, 0, 0);
    u16(1); u8(2); u8(100); u8(0); pt(-500, 0, 1000); pt(500, 0, 1000);   /* behind its own wall */
    u16(1); wall(800, 0);
    vpy3d_load_mesh(&c, img);
    vpy3d_occl_reset();
    vpy3d_draw_mesh(&c, &here, 100);
    n = strokes();
    CHECK(n == 1, "its own line behind its own occluder is still drawn (%d)", n);
    CHECK(vpy3d_occl_count() == 1, "and the occluder went into the table (%d)", vpy3d_occl_count());
    vpy_xf behind = vpy3d_translate(0, 0, 2000);
    vpy3d_draw_mesh(&b, &behind, 100);
    n = strokes();
    CHECK(n == 0, "a line drawn after it, behind the wall, is hidden (%d strokes)", n);
    vpy_xf beside = vpy3d_translate(3000, 0, 2000);
    vpy3d_draw_mesh(&b, &beside, 100);
    n = strokes();
    CHECK(n == 1, "one beside the wall is not (%d)", n);

    /* 4. scenery fog: by the depth of each segment's middle */
    vpy3d_occl_reset();
    vpy3d_set_scenery_fog(1000, 5000);
    vpy3d_draw_mesh(&b, &here, 100);                 /* camera z 3000: half way */
    n = strokes();
    CHECK(n == 1 && MAXBR >= 24 && MAXBR <= 26, "half way into the fog: br %d (its 50%% of 100, halved: ~25)", MAXBR);
    vpy_xf far = vpy3d_translate(0, 0, 4000);         /* camera z 7000: gone */
    vpy3d_draw_mesh(&b, &far, 100);
    n = strokes();
    CHECK(n == 0, "past gone_at nothing is drawn (%d)", n);
    header(2, 0, 0, 0);
    u16(1); u8(2); u8(50); u8(2); pt(-500, 0, 0); pt(500, 0, 0);     /* flags bit1: no fog */
    u16(0);
    vpy3d_load_mesh(&a, img);
    vpy3d_draw_mesh(&a, &far, 100);
    n = strokes();
    CHECK(n == 1 && MAXBR == 50, "a line marked fog:false ignores it (br %d, %d stroke)", MAXBR, n);
    vpy3d_set_scenery_fog(0, 0);

    /* 5. a copy keeps the scenery; version 1 still loads; version 3 does not */
    vpy_mesh cube, mine;
    header(1, 4, 1, 4);
    for (int k = 0; k < 4; k++) { u16((k & 1) ? 100 : -100); u16((k & 2) ? 100 : -100); u16(0); }
    u8(4); u16(0); u16(2); u16(3); u16(1);
    CHECK(vpy3d_load_mesh(&cube, img) && !cube.scenery, "a version-1 image loads, with no scenery");
    CHECK(vpy3d_mesh_copy(&mine, &c) && mine.scenery == c.scenery, "a copy keeps the scenery");
    img[4] = 3;
    CHECK(!vpy3d_load_mesh(&cube, img) && vpy3d_error() == VPY3D_ERR_VERSION, "version 3 is refused");

    /* 6. a bad scenery table is refused at load, not at draw */
    header(2, 0, 0, 0);
    u16(1); u8(1); u8(100); u8(0); pt(0, 0, 0);
    u16(0);
    CHECK(!vpy3d_load_mesh(&a, img) && vpy3d_error() == VPY3D_ERR_DATA, "a one-point line is refused");

    /* 7. scenery only needs no pool space: it loads with the pools full */
    {
        vpy_mesh filler; int ok = 1;
        for (int k = 0; k < 10000 && ok; k++) {        /* fill the vertex pool */
            ok = vpy3d_mesh_begin(&filler);
            for (int v = 0; v < 8 && ok; v++) ok = vpy3d_vertex(v, v, v) >= 0;
            vpy3d_mesh_end(VPY3D_HARD_45);
        }
        header(2, 0, 0, 0);
        u16(1); u8(2); u8(100); u8(0); pt(-500, 0, 0); pt(500, 0, 0);
        u16(0);
        CHECK(vpy3d_load_mesh(&a, img) && a.scenery, "with the pools full, scenery still loads (error %d)", vpy3d_error());
    }

    printf("%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails;
}
