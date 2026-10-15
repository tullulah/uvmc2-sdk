/* vs_logo.h — the Vectrex Studio "VS" logo, from examples/vectrex_studio_intro/assets/
 * vectors/logo.vec (the RP2350 cartridge's boot intro), for the UVMC2 upload screens: the SD
 * bridge's and the debug monitor's. Turned about Y as the intro's DRAW_VECTOR_3D turns it,
 * 128 steps a turn. Generated; centred on its box. */
#ifndef VS_LOGO_H
#define VS_LOGO_H
#include <stdint.h>

/* Paths: point count, closed, then x, y, z per point. */
#define VS_LOGO_PATHS 4
static const int8_t vs_logo[] = {
    13, 0,
    -23, 50, 0, -23, 32, 0, -33, 32, 0, -33, -5, 0, -21, -5, 0, -21, -16, 0, 3, -16, 0, 3, -7, 0, 13, -7, 0, 13, 32, 0, 3, 32, 0, 3, 50, 0, -23, 50, 0,
    5, 0,
    -15, 44, 0, -15, 32, 0, -9, 32, 0, -9, 44, 0, -15, 44, 0,
    13, 0,
    24, -49, 0, 24, -29, 0, 34, -29, 0, 34, 7, 0, 24, 7, 0, 24, 17, 0, -2, 17, 0, -2, 7, 0, -12, 7, 0, -12, -29, 0, -2, -29, 0, -2, -49, 0, 24, -49, 0,
    5, 0,
    18, -41, 0, 18, -32, 0, 10, -32, 0, 10, -41, 0, 18, -41, 0,
};
/* sin(2*pi*a/128) * 127, a = 0..127 */
static const int8_t vs_sin[128] = {
    0, 6, 12, 19, 25, 31, 37, 43, 49, 54, 60, 65, 71, 76, 81, 85, 90, 94, 98, 102, 106, 109, 112, 115, 117, 120, 122, 123, 125, 126, 126, 127, 127, 127, 126, 126, 125, 123, 122, 120, 117, 115, 112, 109, 106, 102, 98, 94, 90, 85, 81, 76, 71, 65, 60, 54, 49, 43, 37, 31, 25, 19, 12, 6, 0, -6, -12, -19, -25, -31, -37, -43, -49, -54, -60, -65, -71, -76, -81, -85, -90, -94, -98, -102, -106, -109, -112, -115, -117, -120, -122, -123, -125, -126, -126, -127, -127, -127, -126, -126, -125, -123, -122, -120, -117, -115, -112, -109, -106, -102, -98, -94, -90, -85, -81, -76, -71, -65, -60, -54, -49, -43, -37, -31, -25, -19, -12, -6
};

/* Draw the logo turned by `angle` (0..127) about its vertical axis, centred at (x, y), scaled
 * by scale/16; the caller's move_abs / delta draw it (a delta is at most 127 a side). */
static inline void vs_logo_draw(int angle, int x, int y, int scale, int intensity,
                                void (*reset)(void), void (*bright)(int), void (*move_abs)(int, int), void (*delta)(int, int))
{
    const int s = vs_sin[angle & 127], c = vs_sin[(angle + 32) & 127];
    const int8_t *p = vs_logo;
    for (int k = 0; k < VS_LOGO_PATHS; k++) {
        const int n = *p++, closed = *p++;
        int px = 0, py = 0;
        reset();
        bright(intensity);
        for (int i = 0; i <= n; i++) {
            const int8_t *q = p + 3 * (i == n ? 0 : i);
            if (i == n && !closed) break;
            const int rx = (q[0] * c + q[2] * s) / 127;
            const int sx = x + rx * scale / 16, sy = y + q[1] * scale / 16;
            if (i == 0) move_abs(sx, sy);
            else {
                int dx = sx - px, dy = sy - py;
                while (dx || dy) {
                    const int ax = dx > 100 ? 100 : dx < -100 ? -100 : dx, ay = dy > 100 ? 100 : dy < -100 ? -100 : dy;
                    delta(ax, ay);
                    dx -= ax; dy -= ay;
                }
            }
            px = sx; py = sy;
        }
        p += 3 * n;
    }
    reset();
}
#endif
