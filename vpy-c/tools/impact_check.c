/* impact_check — vpyimpact against what it promises.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/impact_check.c vpyimpact.c vpyphys.c vpy.c \
 *      -lm -o /tmp/impact_check && /tmp/impact_check
 *
 * The PSG writes are captured from libvpy's own SFX player, so what is checked
 * is what reaches the chip. Exit status is the number of failures.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy.h"
#include "vpyphys.h"
#include "vpyimpact.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }
static uint8_t psg[16];
void v_writePSG(uint8_t r, uint8_t v) { psg[r & 15] = v; }
void v_setSoundAY(uint8_t r, uint8_t v) { v_writePSG(r, v); }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* play the current SFX to the end, one frame per update; the volumes it wrote */
static int run(int *vols, int max)
{
    int n = 0;
    for (int f = 0; f < 64; f++) {
        psg[10] = 0xEE;
        vpy_sfx_update(); vpyimpact_step();
        if (psg[10] != 0xEE && n < max) vols[n++] = psg[10];
    }
    return n;
}

int main(void)
{
    int vols[64], n;

    /* 1. the range: silent below, louder with the impulse, full from `loud` */
    vpyimpact_reset();
    CHECK(!vpyimpact_hit(800, VPYI_WOOD) && vpyimpact_stats()->quiet == 1, "an impulse under the range is silent and counted as quiet");
    vpyimpact_hit(2000, VPYI_WOOD); int v_soft = vpyimpact_stats()->last_volume; run(vols, 64);
    vpyimpact_hit(8000, VPYI_WOOD); int v_mid = vpyimpact_stats()->last_volume; run(vols, 64);
    vpyimpact_hit(50000, VPYI_WOOD); int v_full = vpyimpact_stats()->last_volume;
    CHECK(v_soft < v_mid && v_mid < v_full && v_full == 15 && v_soft >= 4,
          "volume follows the impulse: 2000 -> %d, 8000 -> %d, 50000 -> %d (15)", v_soft, v_mid, v_full);

    /* 2. the stream reaches the PSG: falling volume, rising period, noise only at first, C muted at the end */
    n = run(vols, 64);
    int falls = 1; for (int i = 1; i < n; i++) if (vols[i] > vols[i - 1]) falls = 0;
    CHECK(n >= 5 && vols[0] == 15 && falls && vols[n - 1] == 0, "a full wood knock: %d frames of volume, starting at %d, never rising, ending muted (%d)", n, vols[0], vols[n - 1]);
    vpyimpact_hit(50000, VPYI_WOOD);
    int len; const uint8_t *st = vpyimpact_last_stream(&len);
    int first_per = st[4 + 2 + 2*2 + 1 + 0] , ok_format = st[0]=='S' && st[len - 1] == 0 && st[len - 2] == 0;
    (void)first_per;
    CHECK(ok_format, "the stream has the SFX player's shape: a 4-byte header and an end marker (%d bytes)", len);
    uint16_t per_first = 0, per_last = 0; int noise_frames = 0, frame = 0;
    for (int f = 0; f < 64; f++) {
        vpy_sfx_update(); vpyimpact_step();
        const uint16_t per = (uint16_t)(psg[4] | ((psg[5] & 15) << 8));
        if (f == 0) per_first = per;
        if (psg[10]) { per_last = per; frame++; if (!(psg[7] & 0x20)) noise_frames++; }
    }
    CHECK(per_last > per_first && noise_frames >= 1 && noise_frames < frame,
          "wood: the pitch falls (period %u -> %u) and the noise is only the first %d of %d frames", per_first, per_last, noise_frames, frame);

    /* 3. metal rings longer than wood, soft is shortest and has no noise */
    vpyimpact_hit(50000, VPYI_METAL); int nm = run(vols, 64);
    vpyimpact_hit(50000, VPYI_WOOD);  int nw = run(vols, 64);
    vpyimpact_hit(50000, VPYI_SOFT);  int nsoft = 0, soft_noise = 0;
    for (int f = 0; f < 64; f++) { vpy_sfx_update(); vpyimpact_step(); if (psg[10]) { nsoft++; if (!(psg[7] & 0x20)) soft_noise++; } }
    CHECK(nm > nw && nw > nsoft && soft_noise == 0, "metal %d frames > wood %d > soft %d, soft without noise", nm, nw, nsoft);

    /* 4. one channel: a quieter hit while a loud one rings is skipped; once it has died down it plays */
    vpyimpact_reset();
    vpyimpact_hit(50000, VPYI_METAL);
    vpy_sfx_update(); vpyimpact_step();
    const int skip = !vpyimpact_hit(2500, VPYI_WOOD) && vpyimpact_stats()->skipped == 1;
    for (int f = 0; f < 40; f++) { vpy_sfx_update(); vpyimpact_step(); }
    CHECK(skip && vpyimpact_hit(2500, VPYI_WOOD), "a soft knock under a ringing crash is skipped (counted), and heard once the crash has died");

    /* 5. the music keeps its channels: the hit touches only the mixer's C bits */
    /* a one-event track that turns tones A and B on (mixer 0x3C), then waits */
    vpyimpact_reset();
    static const uint8_t TRACK[] = { 0,0,0,0, 0,0,0,0,  0, 1, 7, 0x3C,  200, 1, 8, 0,  0, 0 };
    vpy_play_music(TRACK);
    vpy_music_update(); vpy_music_update();          /* the first update only primes */
    const uint8_t music_mix = psg[7];
    vpyimpact_hit(50000, VPYI_WOOD); vpy_sfx_update();
    CHECK(music_mix == 0x3C && (psg[7] & 0xDB) == (0x3C & 0xDB) && !(psg[7] & 0x04),
          "the hit takes only channel C: mixer 0x%02x after the music's 0x%02x (A and B kept, tone C on)", psg[7], music_mix);
    vpy_stop_music();

    /* 6. from the physics: a crate dropped from higher sounds louder, and resting is silent */
    int vol_drop[2];
    for (int h = 0; h < 2; h++) {
        vpyp_reset(); vpyp_set_gravity(0, -9800, 0); vpyp_set_floor(1, 0, 40, 150);
        vpyimpact_reset();
        static uint8_t mat[VPYP_MAX_BODIES];
        int id = vpyp_add_box(0, h ? 1600 : 400, 0, 130, 130, 130, 2); mat[id] = VPYI_WOOD;
        int loudest = 0; uint32_t after_rest = 0;
        for (int i = 0; i < 200; i++) {
            vpyp_step(); vpyimpact_step(); vpy_sfx_update();
            if (vpyimpact_contacts(mat, VPYI_NONE) && vpyimpact_stats()->last_volume > loudest) loudest = vpyimpact_stats()->last_volume;
            if (i == 150) after_rest = vpyimpact_stats()->played;
        }
        vol_drop[h] = loudest;
        CHECK(vpyimpact_stats()->played == after_rest, "drop from %d: once it rests it is silent (%u hits, none after 3 s)", h ? 1600 : 400, vpyimpact_stats()->played);
    }
    CHECK(vol_drop[1] > vol_drop[0], "a crate dropped from 1600 lands louder (%d) than from 400 (%d)", vol_drop[1], vol_drop[0]);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
