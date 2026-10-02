/* text3d_check — vpy3d_text and vpy3d_text_billboard against what they promise.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/text3d_check.c vpy3d.c vpy.c \
 *      -o /tmp/text3d_check && /tmp/text3d_check
 *
 * Strokes are caught where libvpy hands them to the hardware. Exit = failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int NS; static int32_t X0 = 1 << 30, X1 = -(1 << 30), Y0 = 1 << 30, Y1 = -(1 << 30);
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e)
{
    (void)e; NS++;
    if (a < X0) X0 = a; if (c < X0) X0 = c; if (a > X1) X1 = a; if (c > X1) X1 = c;
    if (b < Y0) Y0 = b; if (d < Y0) Y0 = d; if (b > Y1) Y1 = b; if (d > Y1) Y1 = d;
}
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)
static void clear(void) { NS = 0; X0 = Y0 = 1 << 30; X1 = Y1 = -(1 << 30); }
static int font_strokes(const char *s)
{
    int n = 0;
    for (; *s; s++) { const signed char *g = vpy_font_glyph((unsigned char)*s); do { if (g[0]) n++; g += 3; } while ((int)g[0] <= 0); }
    return n;
}

int main(void)
{
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);

    /* 1. a sign facing the camera: one stroke per font stroke, and centred where asked */
    vpy_xf at = vpy3d_translate(0, 0, 0);
    clear(); int n = vpy3d_text("HELLO", &at, 200, 100, VPY3D_TEXT_CENTRE); vpy_flush();
    CHECK(n == font_strokes("HELLO") && NS == n, "HELLO: %d strokes sent, %d drawn, the font has %d", n, NS, font_strokes("HELLO"));
    CHECK(abs((X0 + X1) / 2) < (X1 - X0) / 20, "centred: x from %d to %d", X0, X1);
    const int32_t w200 = X1 - X0, h200 = Y1 - Y0;

    /* 2. twice the height is twice the size on screen */
    clear(); vpy3d_text("HELLO", &at, 400, 100, VPY3D_TEXT_CENTRE); vpy_flush();
    CHECK(abs((X1 - X0) - 2 * w200) <= w200 / 20 && abs((Y1 - Y0) - 2 * h200) <= h200 / 20,
          "height 400 draws %d x %d, height 200 %d x %d", X1 - X0, Y1 - Y0, w200, h200);

    /* 3. turned half round it reads backwards — and FRONT hides it */
    vpy_xf back = vpy3d_rot_y(2048); back.t[0] = 0; back.t[1] = 0; back.t[2] = 0;
    clear(); n = vpy3d_text("HELLO", &back, 200, 100, VPY3D_TEXT_FRONT); vpy_flush();
    CHECK(n == 0 && NS == 0, "seen from behind with TEXT_FRONT: nothing drawn");
    clear(); n = vpy3d_text("HELLO", &at, 200, 100, VPY3D_TEXT_FRONT); vpy_flush();
    CHECK(n > 0, "and from the front it is (%d strokes)", n);

    /* 4. on the floor, seen from above at an angle: shorter than it is wide on screen */
    vpy3d_look_at(0, 2000, -2000, 0, 0, 0, 0, 1, 0);
    vpy_xf floor_at = vpy3d_rot_x(1024); floor_at.t[0] = 0; floor_at.t[1] = 0; floor_at.t[2] = 0;
    clear(); vpy3d_text("HELLO", &floor_at, 200, 100, VPY3D_TEXT_CENTRE); vpy_flush();
    const int32_t fw = X1 - X0, fh = Y1 - Y0;
    CHECK(fh * w200 < fw * h200 * 9 / 10, "painted on the floor it is foreshortened: %d x %d against %d x %d facing", fw, fh, w200, h200);

    /* 5. a billboard reads the same from any side */
    int32_t bw[2], bh[2];
    for (int k = 0; k < 2; k++) {
        if (k) vpy3d_look_at(3000, 0, 0, 0, 0, 0, 0, 1, 0); else vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
        clear(); vpy3d_text_billboard("HELLO", 0, 0, 0, 200, 100, VPY3D_TEXT_CENTRE); vpy_flush();
        bw[k] = X1 - X0; bh[k] = Y1 - Y0;
    }
    CHECK(abs(bw[0] - bw[1]) <= bw[0] / 50 && abs(bh[0] - bh[1]) <= bh[0] / 50 && bw[0] > bh[0],
          "billboard: %d x %d from the front, %d x %d from the side", bw[0], bh[0], bw[1], bh[1]);

    /* 6. behind a solid, OCCLUDE cuts it */
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    vpy3d_occl_reset();
    const int32_t wall[4][3] = { { -2000, -2000, -500 }, { 2000, -2000, -500 }, { 2000, 2000, -500 }, { -2000, 2000, -500 } };
    vpy3d_occl_add(wall, 4);
    clear(); vpy3d_text("HELLO", &at, 200, 100, VPY3D_TEXT_OCCLUDE | VPY3D_TEXT_CENTRE); vpy_flush();
    CHECK(NS == 0, "behind a wall that covers it, with OCCLUDE: %d strokes", NS);
    vpy3d_occl_reset();

    /* 7. stereo: the convergence plane has no parallax, nearer is crossed, further uncrossed */
    {
        vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
        int32_t c[3], sx[2][3], sy;
        const int32_t z[3] = { 0, -1500, 3000 };          /* at the convergence plane, nearer, further */
        for (int e = 0; e < 2; e++) {
            vpy3d_set_stereo(e ? 1 : -1, 60, 3000);
            for (int k = 0; k < 3; k++) { vpy3d_to_camera(0, 0, z[k], c); vpy3d_project(c, &sx[e][k], &sy); }
        }
        vpy3d_set_stereo(0, 0, 0);
        int32_t mono; vpy3d_to_camera(0, 0, 0, c); vpy3d_project(c, &mono, &sy);
        CHECK(abs(sx[0][0] - sx[1][0]) <= 1 && sx[0][1] > sx[1][1] && sx[0][2] < sx[1][2] && mono == 0,
              "stereo: on the convergence plane L %d R %d; nearer L %d > R %d (out of the screen); further L %d < R %d; mono back at %d",
              sx[0][0], sx[1][0], sx[0][1], sx[1][1], sx[0][2], sx[1][2], mono);
    }

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
