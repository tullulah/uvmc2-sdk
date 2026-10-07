/* rot_check — vpy_draw_vector_rot puts a sprite's points where the rotation says:
 * a quarter turn (32 in VPy's 128-step unit) sends +X to +Y, half a turn to -X,
 * angle 0 is exactly vpy_draw_vector, and the mirror is applied before turning.
 * Reads the strokes back from the stroke buffer, in device units (VPy x 127).
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/rot_check.c vpy.c -o /tmp/rot_check && /tmp/rot_check
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"

/* The PiTrex contract, as nothing: only the stroke buffer is looked at. */
uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
void v_init(void) {}
uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {}
void v_setRefresh(int hz) { (void)hz; }
void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void vectrexinit(int m) { (void)m; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* An "L": from the origin 20 units along +X, then 10 up. One path, intensity 100. */
static const unsigned char L_vec[] = {
    1, 0,                       /* path_count */
    100, 0, 0, 0, 0,            /* intensity, y0, x0, pad, pad */
    0xFF, 0, 20,                /* dy=0,  dx=20 */
    0xFF, 10, 0,                /* dy=10, dx=0  */
    0x02,
};

/* Two paths, the second a cubic bezier, for the angle-0 identity. */
static const unsigned char B_vec[] = {
    2, 0,
    90, (unsigned char)-10, (unsigned char)-30, 0, 0,
    0xFF, 20, 25, 0xFF, (unsigned char)-5, 30, 0x02,
    60, 5, 5, 0, 0,
    0xFE, 5, 5, 20, 40, 40, (unsigned char)-20, 50, 5, 0x02,
};

typedef struct { int32_t x0, y0, x1, y1; int br; } seg_t;

static int take(seg_t *out, int max)
{
    int n = vpy_pending_strokes();
    for (int i = 0; i < n && i < max; i++)
        vpy_peek_stroke(i, &out[i].x0, &out[i].y0, &out[i].x1, &out[i].y1, &out[i].br);
    vpy_flush();
    return n;
}

#define U(v) ((v) * 127)   /* VPy units -> device units */

static int seg_is(const seg_t *s, int x0, int y0, int x1, int y1)
{
    return s->x0 == x0 && s->y0 == y0 && s->x1 == x1 && s->y1 == y1;
}

static void expect_L(const char *what, int x, int y, int angle, int mirror,
                     int ax, int ay, int bx, int by)  /* the two corners after the origin, local */
{
    seg_t s[8];
    vpy_draw_vector_rot_ex(L_vec, x, y, angle, mirror, 0);
    int n = take(s, 8);
    CHECK(n == 2 && seg_is(&s[0], U(x), U(y), U(x + ax), U(y + ay))
                 && seg_is(&s[1], U(x + ax), U(y + ay), U(x + bx), U(y + by)),
          "%-28s (%d,%d)->(%d,%d)->(%d,%d)  got (%d,%d)->(%d,%d)->(%d,%d)", what,
          U(x), U(y), U(x + ax), U(y + ay), U(x + bx), U(y + by),
          s[0].x0, s[0].y0, s[0].x1, s[0].y1, s[1].x1, s[1].y1);
}

int main(void)
{
    printf("vpy_draw_vector_rot: L sprite drawn at (10,-5)\n");
    expect_L("angle 0",                10, -5,   0, 0,  20,  0,  20,  10);
    expect_L("angle 32 = 90 deg",      10, -5,  32, 0,   0, 20, -10,  20);
    expect_L("angle 64 = 180 deg",     10, -5,  64, 0, -20,  0, -20, -10);
    expect_L("angle 96 = 270 deg",     10, -5,  96, 0,   0, -20, 10, -20);
    expect_L("angle 160 = 32 (mod 128)", 10, -5, 160, 0,  0, 20, -10,  20);
    expect_L("angle -96 = 32",         10, -5, -96, 0,   0, 20, -10,  20);
    expect_L("mirror, then 90 deg",    10, -5,  32, 1,   0, -20, -10, -20);
    expect_L("flip Y, then 90 deg",    10, -5,  32, 2,   0, 20,  10,  20);
    expect_L("flip X+Y, then 90 deg",  10, -5,  32, 3,   0, -20, 10, -20);

    /* 45 degrees: (20,0) -> (14.142, 14.142) units = 1796.0 device units. */
    {
        seg_t s[4];
        vpy_draw_vector_rot(L_vec, 0, 0, 16);
        take(s, 4);
        CHECK(s[0].x1 == 1796 && s[0].y1 == 1796, "angle 16 = 45 deg            (1796,1796) got (%d,%d)",
              s[0].x1, s[0].y1);
    }

    /* Per-path intensity kept; override replaces it. */
    {
        seg_t s[4];
        vpy_draw_vector_rot(L_vec, 0, 0, 32);
        take(s, 4);
        CHECK(s[0].br == 100 && s[1].br == 100, "path intensity kept           100 got %d", s[0].br);
        vpy_draw_vector_rot_ex(L_vec, 0, 0, 32, 0, 40);
        take(s, 4);
        CHECK(s[0].br == 40 && s[1].br == 40, "intensity override            40 got %d", s[0].br);
    }

    /* Angle 0 draws exactly what vpy_draw_vector draws, beziers included. */
    {
        seg_t a[64], b[64];
        vpy_draw_vector(B_vec, 7, -3);
        int na = take(a, 64);
        vpy_draw_vector_rot(B_vec, 7, -3, 0);
        int nb = take(b, 64);
        int same = na == nb && na > 0;
        for (int i = 0; same && i < na; i++)
            same = seg_is(&a[i], b[i].x0, b[i].y0, b[i].x1, b[i].y1) && a[i].br == b[i].br;
        CHECK(same, "angle 0 == vpy_draw_vector    %d strokes vs %d", na, nb);
    }

    printf(fails ? "\n%d FAILED\n" : "\nall ok\n", fails);
    return fails ? 1 : 0;
}
