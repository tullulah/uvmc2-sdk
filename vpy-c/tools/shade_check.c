/* shade_check — vpy3d_fog, vpy3d_shadow and vpy3d_screen_size, against geometry.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/shade_check.c vpy3d.c vpy.c \
 *      -o /tmp/shade_check && /tmp/shade_check
 *
 * Exit status is the number of failures.
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

int main(void)
{
    vpy3d_look_at(0, 0, 0, 0, 0, 1000, 0, 1, 0);          /* looking down +z */
    CHECK(vpy3d_fog(100, 0, 0, 500, 1000, 3000) == 100, "fog: full brightness nearer than full_until");
    CHECK(vpy3d_fog(100, 0, 0, 2000, 1000, 3000) == 50, "fog: half way between, half (%d)", vpy3d_fog(100, 0, 0, 2000, 1000, 3000));
    CHECK(vpy3d_fog(100, 0, 0, 3500, 1000, 3000) == 0, "fog: nothing past gone_at");

    CHECK(vpy3d_screen_size(0, 0, 2000, 100) == 1400 && vpy3d_screen_size(0, 0, 4000, 100) == 700,
          "screen size halves as the distance doubles: %d, %d", vpy3d_screen_size(0, 0, 2000, 100), vpy3d_screen_size(0, 0, 4000, 100));
    CHECK(vpy3d_screen_size(0, 0, 100, 100) == 0, "behind the near plane: 0");

    /* a box 100 up, light straight down: its shadow is its own square, 4 sides */
    vpy3d_look_at(0, 3000, -3000, 0, 0, 0, 0, 1, 0);
    int32_t c[8][3];
    for (int k = 0; k < 8; k++) { c[k][0] = (k & 1) ? 100 : -100; c[k][1] = (k & 2) ? 300 : 100; c[k][2] = (k & 4) ? 100 : -100; }
    NS = 0; int n = vpy3d_shadow((const int32_t (*)[3])c, 8, 0, -1, 0, 0, 60); vpy_flush();
    CHECK(n == 4 && NS == 4, "light straight down: a square shadow, %d sides, %d strokes", n, NS);
    /* light slanting along x only: the two squares slide apart along x, so the
     * shadow is a longer rectangle; slanting in x AND z, a hexagon */
    NS = 0; n = vpy3d_shadow((const int32_t (*)[3])c, 8, 1, -1, 0, 0, 60); vpy_flush();
    CHECK(n == 4, "light slanting in x: a stretched rectangle (%d sides)", n);
    NS = 0; n = vpy3d_shadow((const int32_t (*)[3])c, 8, 1, -1, 1, 0, 60); vpy_flush();
    CHECK(n == 6, "light slanting in x and z: a hexagon (%d sides)", n);
    CHECK(vpy3d_shadow((const int32_t (*)[3])c, 8, 0, 1, 0, 0, 60) == 0, "light from below: no shadow");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
