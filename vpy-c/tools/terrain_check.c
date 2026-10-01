/* terrain_check — vpy3d_terrain's floating horizon, against what it must hide.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/terrain_check.c vpy3d.c vpy.c \
 *      -lm -o /tmp/terrain_check && /tmp/terrain_check [picture.svg]
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy.h"
#include "vpy3d.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
typedef struct { int32_t x0, y0, x1, y1; } seg_t;
static seg_t S[4096]; static int NS;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)e; if (NS < 4096) S[NS++] = (seg_t){ a, b, c, d }; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

#define C 24
#define R 16
static int16_t H[R * C];

/* narrow enough that every row is on screen: a segment the screen clips away is
 * not the horizon's doing */
static int draw(void) { NS = 0; vpy3d_terrain(H, C, R, -1150, 1000, 100, 100); vpy_flush(); return NS; }

int main(int argc, char **argv)
{
    vpy3d_look_at(0, 900, -1500, 0, 0, 2500, 0, 1, 0);       /* above the land, looking across it */

    for (int i = 0; i < R * C; i++) H[i] = 0;
    const int flat = draw();
    CHECK(flat == R * (C - 1), "flat land: every segment of every row shows (%d of %d)", flat, R * (C - 1));

    /* a ridge across row 4: what is behind it, lower, is hidden */
    for (int c = 0; c < C; c++) H[4 * C + c] = 700;
    const int ridge = draw();
    CHECK(ridge < flat * 3 / 4, "a ridge across the 5th row hides the land behind it: %d strokes against %d", ridge, flat);

    /* rolling hills: no stroke may lie below what was already drawn in its columns */
    for (int r = 0; r < R; r++) for (int c = 0; c < C; c++)
        H[r * C + c] = (int16_t)(300 * sin(c * 0.5) * cos(r * 0.4) + 200 * sin(r * 0.9 + c * 0.2));
    draw();
    /* build the horizon from the strokes in the order drawn, and check each against the one before */
    static double hz[2000]; for (int i = 0; i < 2000; i++) hz[i] = -1e18;
    int below = 0;
    for (int s = 0; s < NS; s++) {
        const int x0 = S[s].x0 < S[s].x1 ? S[s].x0 : S[s].x1, x1 = S[s].x0 < S[s].x1 ? S[s].x1 : S[s].x0;
        for (int k = 0; k <= 16; k++) {
            const double t = k / 16.0, x = S[s].x0 + t * (S[s].x1 - S[s].x0), y = S[s].y0 + t * (S[s].y1 - S[s].y0);
            const int col = (int)((x + 16000) / 16);
            if (col >= 0 && col < 2000 && y < hz[col] - 200) below++;
        }
        for (int x = x0; x <= x1; x += 16) {
            const double t = S[s].x1 == S[s].x0 ? 0 : (double)(x - S[s].x0) / (S[s].x1 - S[s].x0);
            const double y = S[s].y0 + t * (S[s].y1 - S[s].y0);
            const int col = (x + 16000) / 16;
            if (col >= 0 && col < 2000 && y > hz[col]) hz[col] = y;
        }
    }
    CHECK(below == 0, "rolling hills, %d strokes: nothing drawn below what was drawn before it (%d points under)", NS, below);

    if (argc > 1) {
        FILE *f = fopen(argv[1], "w");
        fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' viewBox='-16500 -16500 33000 33000' width='600' height='600' style='background:#000'><g transform='scale(1,-1)'>");
        for (int s = 0; s < NS; s++) fprintf(f, "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='#8f8' stroke-width='80'/>", S[s].x0, S[s].y0, S[s].x1, S[s].y1);
        fprintf(f, "</g></svg>"); fclose(f);
    }
    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
