/* fill_check — the serpentine raster (vpy_draw_filled_polygon) on a few shapes.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/fill_check.c vpy.c \
 *      -o /tmp/fill_check && /tmp/fill_check /tmp/fill.svg
 *
 * Per shape: the strokes, how many start where the one before ended (the path is
 * continuous there, so the beam does not move), and whether every stroke stays inside
 * the shape's box. With an argument, an SVG of all of them to look at. Exit = failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "vpy.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static FILE *svg;
static int failures;

static void shape(const char *name, const int *xy, int n)
{
    int minx = 9999, maxx = -9999, miny = 9999, maxy = -9999;
    for (int i = 0; i < n; i++) {
        if (xy[2*i] < minx) minx = xy[2*i]; if (xy[2*i] > maxx) maxx = xy[2*i];
        if (xy[2*i+1] < miny) miny = xy[2*i+1]; if (xy[2*i+1] > maxy) maxy = xy[2*i+1];
    }
    vpy_draw_filled_polygon(xy, n, 100);
    int k = vpy_pending_strokes(), chained = 0, out = 0;
    int32_t px1 = 0, py1 = 0;
    const int32_t m = 127;   /* a VPy unit of slack: the joins follow the edge */
    for (int i = 0; i < k; i++) {
        int32_t x0, y0, x1, y1; int br;
        vpy_peek_stroke(i, &x0, &y0, &x1, &y1, &br);
        if (i && x0 == px1 && y0 == py1) chained++;
        if (x0 < minx * 127 - m || x0 > maxx * 127 + m || x1 < minx * 127 - m || x1 > maxx * 127 + m ||
            y0 < miny * 127 - m || y0 > maxy * 127 + m || y1 < miny * 127 - m || y1 > maxy * 127 + m) out++;
        if (svg) fprintf(svg, "<line x1='%d' y1='%d' x2='%d' y2='%d' stroke='#0f0' stroke-width='30'/>\n",
                         x0, -y0, x1, -y1);
        px1 = x1; py1 = y1;
    }
    printf("%-10s %4d strokes, %4d chained (%d%%), %d outside the box\n",
           name, k, chained, k ? 100 * chained / k : 0, out);
    if (k == 0 || out) failures++;
    vpy_flush();
}

int main(int argc, char **argv)
{
    vpy_init();
    if (argc > 1) {
        svg = fopen(argv[1], "w");
        fprintf(svg, "<svg xmlns='http://www.w3.org/2000/svg' viewBox='-16500 -16500 33000 33000' style='background:#000'>\n");
    }
    const int rect[]  = { -110, 60, -40, 60, -40, 110, -110, 110 };
    const int tri[]   = { 0, 60, 70, 110, 100, 50 };
    const int tee[]   = { -110, 30, -30, 30, -30, 10, -60, 10, -60, -60, -80, -60, -80, 10, -110, 10 };
    const int star[]  = { 60, 30, 72, -5, 110, -5, 80, -28, 92, -65, 60, -42, 28, -65, 40, -28, 10, -5, 48, -5 };
    const int notch[] = { -20, -70, 30, -70, 30, -110, 10, -110, 5, -90, 0, -110, -20, -110 };
    shape("rect", rect, 4);
    shape("triangle", tri, 3);
    shape("T", tee, 8);
    shape("star", star, 10);
    shape("notch", notch, 7);
    vpy_fill_style(300, 0);                     /* sparse and flat, to see the path */
    const int box[] = { 40, -110, 110, -110, 110, -75, 40, -75 };
    shape("flat 300", box, 4);
    vpy_fill_style(0, 0);

    /* the raster, every row and one in two: 4 rows of 3 runs (127, 64, 127) each */
    {
        static const unsigned char img[] = { 9, 4,
            0, 3, 3, 127, 3, 64, 3, 127,   0, 3, 3, 127, 3, 64, 3, 127,
            0, 3, 3, 127, 3, 64, 3, 127,   0, 3, 3, 127, 3, 64, 3, 127 };
        int all, half, back_ok = 1;
        vpy_draw_raster(img, -100, -20, 100, 100, 0, 127);
        all = vpy_pending_strokes(); vpy_flush();
        vpy_fill_rows(2);
        vpy_draw_raster(img, -100, -20, 100, 100, 0, 127);
        half = vpy_pending_strokes();
        {   /* the two rows kept go opposite ways */
            int32_t a0, b0, a1, b1, c0, d0, c1, d1; int br;
            vpy_peek_stroke(0, &a0, &b0, &a1, &b1, &br);
            vpy_peek_stroke(3, &c0, &d0, &c1, &d1, &br);
            back_ok = (a1 > a0) && (c1 < c0);
        }
        vpy_flush();
        vpy_fill_rows(1);
        printf("raster     %d strokes every row, %d one in two, serpentine %s\n", all, half, back_ok ? "ok" : "WRONG");
        if (all != 12 || half != 6 || !back_ok) failures++;
    }
    if (svg) { fprintf(svg, "</svg>\n"); fclose(svg); }
    printf("%s\n", failures ? "FAIL" : "ok");
    return failures;
}
