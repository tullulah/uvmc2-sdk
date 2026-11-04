/* uvm2_intro.c — THE VECTREX STUDIO CARD, before a game's OPTIONS screen.
 *
 * The kuroishi cartridge's boot card (hardware/uvm2/kuroishi/src/boot.c) made part of the SDK:
 * the logo turns twice while it fades in, holds face on over the wordmark with STUDIO and
 * GAMES under it, and fades out, to the Vectrex Studio jingle on the jack. Every game with
 * UVM2_SETTING_OPTIONS shows it; uvm2_options_boot() calls it first.
 *
 * THE JINGLE IS INSIDE THE IMAGE: uvm2_intro_data.h, made by tools/gen_intro.py, as 16 kHz IMA
 * ADPCM (~34 KB; the reasons are in the generator). It is doubled to the jack's 32 kHz here by
 * putting the midpoint between each two samples.
 *
 * THE CARD LASTS AS LONG AS THE JINGLE: the hold is what the song leaves between the spin and
 * the fade, so a new recording moves the card with it. Any button skips to the fade, and the
 * sound fades with the picture.
 *
 * THE JACK IS LEFT STOPPED, not running: the ring loops whatever it holds, and between this and
 * the game there is the OPTIONS screen and the game's start-up, which write nothing to it. The
 * game's own uvm2_jack_init() starts it again (uvm2_jack.c: a second init restarts the first).
 * On a cartridge without a jack the card is the same, silent.
 *
 * IT IS NOT OBLIGATORY: `make uvm2 UVM2_INTRO=0` (or UVM2_INTRO = 0 in the game's Makefile)
 * defines UVM2_NO_INTRO, which compiles all of this out -- the ~36 KB with it -- and leaves an
 * empty uvm2_intro(). uvm2.mk documents it.
 */
#include <stdint.h>
#include "uvm2_config.h"

#ifdef UVM2_NO_INTRO
void uvm2_intro(void) { }
#else
#include "uvm2_draw.h"
#include "uvm2_text.h"
#include "uvm2_jack.h"
#include "uvm2_intro_data.h"
#include "pico/time.h"

extern volatile uint8_t uvm2_cached_buttons;   /* core 1's copy, ACTIVE LOW */

/* THE CARD'S TIMING, in ticks of TICK_HZ: kuroishi's, which counted 50 Hz frames. Two turns in
 * SPIN while it fades in over FADE_IN, then the hold, then FADE_OUT.
 *
 * Drawn at TICK_HZ too: kuroishi's rate, and on a CRT the rate is part of the brightness. The
 * list fits: MEASURED in the UVM2 emulator (tools/correr_uvm2.mjs, "el juego dice") at 161
 * vectors and 11702 bus cycles, against the 30000 of a 50 Hz period.
 *
 * BUT IT KEEPS TIME BY THE RP2350's MICROSECOND TIMER, not by counting frames: a frame that
 * runs long (a slow SD, a console that measures its E slow) would stretch a frame count and
 * leave the picture behind the jingle; the clock keeps the two ending together. */
#define TICK_HZ     50
#define TICK_US    (1000000 / TICK_HZ)
#define SPIN       128
#define FADE_IN     40
#define FADE_OUT    25
#define BRIGHT      80     /* the card's top level, in percent of each path's own */
/* The jack's ring runs AHEAD of the DAC by up to 1600 samples (uvm2_jack.c): 50 ms, two and
 * a half ticks. Silence written that many ticks before the end has reached the DAC by it. */
#define QUIET       3
_Static_assert(QUIET * UVM2_JACK_RATE / TICK_HZ >= 1600, "QUIET must cover the jack's queue");

/* Where the pieces sit (kuroishi's): the logo high, the wordmark low, STUDIO in the gap the
 * wordmark leaves between its two rules, GAMES under it.
 *
 * STUDIO IS CENTRED ON THE RULES, which sit at y = -18 in the wordmark's own drawing
 * (vs_wordmark.vec). kuroishi gave a y for VPy's font, whose y is a baseline 8 above the
 * letters; this font's y is their top (uvm2_text.c), so the same number put the word above
 * the rules, measured in the emulator. GAMES keeps kuroishi's 22 under it. */
#define LOGO_Y      34
#define MARK_Y    (-40)
#define RULE_Y    (MARK_Y - 18)
#define WORDS       2      /* uvm2_print_text scale */
#define ADVANCE   ((7 * WORDS) >> 1)   /* uvm2_text.c's advance per letter */
#define LETTER_H  ((6 * WORDS) >> 1)   /* and its letter height */
#define STUDIO_Y  (RULE_Y + LETTER_H / 2)
#define GAMES_Y   (STUDIO_Y - 22)

/* THE TURN'S PERSPECTIVE: a point `z` towards the viewer is drawn D/(D - z) larger, so the
 * near edge leads instead of the logo squashing flat. */
#define D          320

/* ── the jingle ─────────────────────────────────────────────────────────────────────────── */

static const int16_t STEP[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55,
    60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500,
    20350, 22385, 24623, 27086, 29794, 32767 };
static const int8_t INDEX[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

_Static_assert(VS_JINGLE_RATE * 2 == UVM2_JACK_RATE, "the jingle is doubled to the jack's rate");

static struct { uint32_t pos; int pred, idx, prev; } s_dec;

static int adpcm_next(void)
{
    if (s_dec.pos >= VS_JINGLE_SAMPLES) return 0;
    const uint8_t byte = VS_JINGLE[s_dec.pos >> 1];
    const int code = (s_dec.pos & 1) ? byte >> 4 : byte & 15;
    s_dec.pos++;
    const int step = STEP[s_dec.idx];
    int diff = step >> 3;
    if (code & 4) diff += step;
    if (code & 2) diff += step >> 1;
    if (code & 1) diff += step >> 2;
    int p = (code & 8) ? s_dec.pred - diff : s_dec.pred + diff;
    if (p > 32767) p = 32767;
    if (p < -32768) p = -32768;
    s_dec.pred = p;
    int i = s_dec.idx + INDEX[code & 7];
    s_dec.idx = i < 0 ? 0 : i > 88 ? 88 : i;
    return p;
}

/* What the jack can take now, at `gain_q8` (256 = as recorded). */
static void feed(int gain_q8)
{
    int16_t buf[64];
    int space = uvm2_jack_space() & ~1;
    while (space > 0) {
        const int n = space > 64 ? 64 : space;
        for (int k = 0; k < n; k += 2) {
            const int cur = adpcm_next();
            buf[k]     = (int16_t)((((s_dec.prev + cur) >> 1) * gain_q8) >> 8);
            buf[k + 1] = (int16_t)((cur * gain_q8) >> 8);
            s_dec.prev = cur;
        }
        uvm2_jack_write(buf, n);
        space -= n;
    }
}

/* ── the card ───────────────────────────────────────────────────────────────────────────── */

/* One path's points joined, each point through `xf` (the logo's turn) or as they are. */
static void path(const uvm2_intro_path *p, int dy, int lv, int c, int s, int turn)
{
    uvm2_draw_intensity(p->bright * lv / 100);
    int px = 0, py = 0;
    for (int i = 0; i < p->n; i++) {
        int x = p->pt[i][0], y = p->pt[i][1];
        if (turn) {
            const int xr = (x * c) >> 14, z = (x * s) >> 14;
            x = xr * D / (D - z);
            y = y * D / (D - z);
        }
        y += dy;
        if (i == 0) uvm2_draw_move_abs(x, y);
        else        uvm2_draw_delta(x - px, y - py);
        px = x; py = y;
    }
    if (p->closed && p->n > 2) uvm2_draw_delta(p->pt[0][0] - px, p->pt[0][1] + dy - py);
}

static void words(int y, const char *t, int lv)
{
    int n = 0;
    while (t[n]) n++;
    uvm2_print_text(-(n * ADVANCE) / 2, y, t, WORDS, lv);
}

static void card(int lv, int a)
{
    const int s = VS_SIN[a & (VS_SIN_STEPS - 1)];
    const int c = VS_SIN[(a + VS_SIN_STEPS / 4) & (VS_SIN_STEPS - 1)];
    for (int p = 0; p < VS_LOGO_N; p++)     path(&VS_LOGO[p], LOGO_Y, lv, c, s, 1);
    for (int p = 0; p < VS_WORDMARK_N; p++) path(&VS_WORDMARK[p], MARK_Y, lv, 0, 0, 0);
    words(STUDIO_Y, "STUDIO", lv);
    words(GAMES_Y, "GAMES", lv);
}

void uvm2_intro(void)
{
    const int total = (int)((VS_JINGLE_SAMPLES * TICK_HZ + VS_JINGLE_RATE - 1) / VS_JINGLE_RATE);
    const int hold = total > SPIN + FADE_OUT ? total - SPIN - FADE_OUT : 0;
    const int fade_at = SPIN + hold, end = fade_at + FADE_OUT;

    uvm2_draw_rotate((int)uvm2_setting_rotate);
    uvm2_set_refresh(TICK_HZ);
    const int jack = uvm2_jack_init();
    s_dec.pos = 0; s_dec.pred = 0; s_dec.idx = 0; s_dec.prev = 0;
    /* EDGES START AS "PRESSED": the button that launched the game is still down. */
    uint8_t before = 0xFF;

    uint32_t t0 = time_us_32();
    for (;;) {
        const uint32_t us = time_us_32() - t0;
        const int t = (int)(us / TICK_US);
        if (t >= end) break;
        const int lv = t < FADE_IN  ? BRIGHT * t / FADE_IN
                     : t > fade_at  ? BRIGHT * (end - t) / FADE_OUT
                     : BRIGHT;
        /* The sound reaches silence QUIET ticks before the picture, so that what is still
         * queued in the ring when the jack is stopped is silence. */
        if (jack) {
            const int left = end - QUIET - t;
            feed(t <= fade_at ? 256 : left <= 0 ? 0 : 256 * left / (FADE_OUT - QUIET));
        }
        uvm2_frame_begin();
        /* two turns over SPIN ticks, then face on; by the microsecond, not by the tick, so a
         * fast frame turns it a little and not by steps */
        if (lv > 0) card(lv, t < SPIN ? (int)((uint64_t)us * 2 * VS_SIN_STEPS / (SPIN * TICK_US)) : 0);
        uvm2_frame_end();

        const uint8_t b = (uint8_t)~uvm2_cached_buttons;      /* 1 = pressed */
        if ((b & ~before) && t < fade_at) t0 -= (uint32_t)(fade_at - t) * TICK_US;   /* to the fade */
        before = b;
    }
    if (jack) uvm2_jack_set(0);
}

#endif /* UVM2_NO_INTRO */
