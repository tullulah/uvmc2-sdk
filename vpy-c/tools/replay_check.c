/* replay_check — a recording played back IS the game: drive vpyphys with
 * random input, record it, play it back from the same seed, and the bodies
 * end up exactly where they did.
 *
 *   cc -O2 -w -Iinclude tools/replay_check.c vpyreplay.c vpyphys.c -lm -o /tmp/replay_check && /tmp/replay_check
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpyreplay.h"
#include "vpyphys.h"

int vpy_sin_q14(int a) { return (int)lround(16384.0 * sin(a * 2 * M_PI / 4096)); }
int vpy_cos_q14(int a) { return (int)lround(16384.0 * cos(a * 2 * M_PI / 4096)); }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* a tiny "game": the stick pushes a ball, button 1 drops a box, and the seed
 * decides where boxes land */
static uint32_t rng;
static uint32_t next(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static void game_start(uint32_t seed) { rng = seed; vpyp_reset(); vpyp_set_gravity(0, -9800, 0); vpyp_set_floor(1, 0, 60, 150); vpyp_add_sphere(0, 100, 0, 100, 1); }
static void game_frame(uint8_t b, int8_t jx, int8_t jy)
{
    vpyp_apply_impulse(0, jx * 4, 0, jy * 4);
    if (b & 1) vpyp_add_box((int32_t)(next() % 1000) - 500, 800, (int32_t)(next() % 1000) - 500, 80, 80, 80, 1);
    vpyp_step();
}
static int32_t fingerprint(void)
{
    int32_t f = 0, x, y, z;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) if (vpyp_alive(i)) { vpyp_position(i, &x, &y, &z); f = f * 31 + x * 7 + y * 3 + z; }
    return f;
}

int main(void)
{
    static vpyreplay_frame buf[600];
    uint32_t live = 12345;
    vpyreplay_record(buf, 600, 777);
    game_start(vpyreplay_seed());
    for (int i = 0; i < 500; i++) {
        live ^= live << 13; live ^= live >> 17; live ^= live << 5;
        uint8_t b = (live % 23) == 0; int8_t jx = (int8_t)((live >> 8) % 255 - 127), jy = (int8_t)((live >> 16) % 255 - 127);
        vpyreplay_input(&b, &jx, &jy);
        game_frame(b, jx, jy);
    }
    const int32_t played = fingerprint(); const int n = vpyreplay_frames();
    vpyreplay_stop();

    vpyreplay_play(buf, n, 777);
    game_start(vpyreplay_seed());
    for (int i = 0; i < 500; i++) {
        uint8_t b = 0xFF; int8_t jx = 99, jy = 99;           /* the live input is ignored */
        vpyreplay_input(&b, &jx, &jy);
        game_frame(b, jx, jy);
    }
    CHECK(n == 500 && fingerprint() == played, "500 frames recorded and played back: every body where it was");
    uint8_t b = 1; int8_t jx = 5, jy = 5; vpyreplay_input(&b, &jx, &jy);
    CHECK(vpyreplay_done() && b == 0 && jx == 0, "past the end: done, and nobody touches anything");

    vpyreplay_record(buf, 10, 1);
    for (int i = 0; i < 15; i++) { uint8_t bb = 0; int8_t x = 0, y = 0; vpyreplay_input(&bb, &x, &y); }
    CHECK(vpyreplay_frames() == 10 && vpyreplay_lost() == 5, "a full recording keeps 10 and counts the 5 it lost");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
