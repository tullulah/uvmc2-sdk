/* shape_check — vpy3d against a console's calibrated screen shape.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/shape_check.c vpy3d.c vpy.c \
 *      -o /tmp/shape_check && /tmp/shape_check
 *
 * This file DEFINES the console's shape (uvm2_screen_*), as uvm2_config.c does in a .um2;
 * aspect_check.c is the build without it, where nothing may change. Exit = failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"

volatile int32_t uvm2_screen_aspect_q8 = 300, uvm2_screen_win_x = 18000, uvm2_screen_win_y = 20500;

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

static void project(int32_t x, int32_t y, int32_t z, int32_t *sx, int32_t *sy)
{
    int32_t c[3]; vpy3d_to_camera(x, y, z, c); vpy3d_project(c, sx, sy);
}

int main(void)
{
    int32_t sx, sy;
    /* 1. the console's aspect is taken by itself: a world square draws 300/256 wider */
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    project(1000, 1000, 0, &sx, &sy);
    CHECK(abs(sx * 256 - sy * 300) <= 300, "console aspect 300/256: a square's corner at (%d, %d), ratio %.3f", sx, sy, (double)sx / sy);

    /* 2. a game that sets its own wins */
    vpy3d_set_aspect(1, 1);
    vpy3d_look_at(0, 0, -3000, 0, 0, 0, 0, 1, 0);
    project(1000, 1000, 0, &sx, &sy);
    CHECK(sx == sy, "the game's vpy3d_set_aspect(1,1) wins over the console's: (%d, %d)", sx, sy);

    /* 3. the window only on request, and then the console's */
    const int h0 = vpy3d_h_half_angle(), v0 = vpy3d_v_half_angle();
    const int used = vpy3d_use_console_window();
    const int h1 = vpy3d_h_half_angle(), v1 = vpy3d_v_half_angle();
    CHECK(h0 == v0 && used && h1 > h0 && v1 > h1, "the window: square %d/%d by default; the console's 18000 x 20500 when asked, %d/%d", h0, v0, h1, v1);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
