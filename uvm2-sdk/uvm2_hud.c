/* uvm2_hud.c — THE DIAGNOSTICS HUD: what stats.py reads over SWD, on the tube, over any game.
 *
 * WHY. A UVMC2 on the bench usually has no probe on it, and the questions are always the same
 * four — is the list overflowing, how fast is it running, is the clamp right, how deep are the
 * stacks. This answers them without a probe and without changing the game: hold the combo and
 * the figures appear in the top-left corner of every frame; hold it again and they go.
 *
 * WHAT IT SHOWS, two lines in the top-left corner:
 *
 *     F50 C12840 D0 Z0          fps (previous frame, from us_frame_last), the bus cycles the
 *                               GAME's drawing took this frame, dropped and ramps_clamped
 *                               (previous frame) — and "!!" at the end when either is not 0
 *     S2624 812 N3614 V322 M150 deepest core 0 / core 1 stack in bytes ("!" after them if
 *                               either overflowed), the game's commands and lit vectors
 *                               this frame, and the core clock in MHz
 *
 * C, N and V are read at the moment the HUD starts drawing, so they are exactly the game's and
 * never include the HUD's own share. D and Z must both be 0 — CLAUDE.md: "if `dropped` is
 * non-zero, nothing on screen is evidence of anything".
 *
 * THE COMBO: BUTTONS 1 AND 4, AND ONLY THOSE, HELD FOR TWO SECONDS. Toggles once per hold.
 *   - not 2+3: that is the calibration combo, held while launching (uvm2_wizard.c), and the
 *     hold can run on into the game;
 *   - not all four: that is the debug cartridge BIOS's exit to the menu — so 2 and 3 must be
 *     UP for this to count, and holding all four never toggles the HUD;
 *   - two seconds, because games do press 1 and 4 together (fire and something else); a
 *     deliberate hold of both, with nothing else, for that long is rare in play.
 * The buttons come from uvm2_cached_buttons, which core 1 refreshes between frames: reading
 * the cache costs nothing and touches no bus (CLAUDE.md invariants 3 and 7).
 * `uvm2_hud` is also a plain volatile: poke it over SWD (tools/swd_var.py) to switch it.
 *
 * ITS COST, MEASURED, BOUNDED AND COUNTED. It is text in the list like any other, and text is
 * expensive on this beam. Measured on the host (tools/uvm2_hud_test.c, 2026-10-03):
 *     five lines, one re-zero per glyph (uvm2_print_text)   2877 commands, 25139 bus cycles
 *     the same, chained (uvm2_print_text_chained)           2249 commands, 15277 bus cycles
 *     these two lines, chained                             ~1050 commands,  ~7100 bus cycles
 * — the text size barely changes it: the cost is per stroke, not per unit of length. So: two
 * lines, chained, about a quarter of a 30000-cycle frame. A game near its budget WILL drop
 * below 50 Hz with the HUD on; that is the instrument perturbing the measurement (07), which
 * is why C, N and V are the game's alone and the HUD's own cost is in uvm2_hud_stats. Chained
 * text is drawn the way a game draws and so depends on the zero calibration; on a badly
 * calibrated console the HUD bends exactly like the game's text does.
 * It is skipped (and counted in `skipped`) when the list has less room left than the HUD can
 * need, so it is never the reason a game's list overflows.
 *
 * Not in a BIOS build (UVM2_BIOS): the debug cartridge has a probe and its own menu, and its
 * RAM code has a size past which its input stops working (see its build.rs).
 */
#include "uvm2_bus.h"
#include "uvm2_draw.h"
#include "uvm2_text.h"

extern volatile uint8_t uvm2_cached_buttons;      /* uvm2_core1.c; active LOW */
extern uint32_t uvm2_cycles_per_e_q8;             /* uvm2_bus.c, measured at boot */

volatile uint8_t uvm2_hud;                         /* 0 off, 1 on */
uvm2_hud_stats_t uvm2_hud_stats;

#define HUD_COMBO       0x09u     /* buttons 1 and 4 (bits 0 and 3)... */
#define HUD_COMBO_MASK  0x0Fu     /* ...with 2 and 3 up */
#define HUD_HOLD        100u      /* frames: two seconds at 50 Hz */
#define HUD_X          (-125)
#define HUD_Y           120
#define HUD_STEP        16        /* between lines, at the stock text size */
#define HUD_SCALE       3         /* uvm2_print_text's stock size (x1.5); 1 is unreadable */
#define HUD_BRIGHT      100
/* The most the HUD can add, with every number at its longest: measured ~1050 commands for two
 * typical lines (above); this leaves room for longer numbers and both alarms. */
#define HUD_ROOM        2600u     /* two more lines when the game adds its own */

static char *put_s(char *p, const char *s) { while (*s) *p++ = *s++; return p; }
static char *put_u(char *p, uint32_t v)
{
    char d[11]; int k = 0;
    do { d[k++] = (char)('0' + v % 10u); v /= 10u; } while (v && k < 10);
    while (k) *p++ = d[--k];
    return p;
}

/* THE LINES AS TEXT TOO, so what the tube shows can be read back — by the host test, and
 * over SWD (`uvm2_hud_text`) when the photo is unreadable. */
#define HUD_LINES 4
char uvm2_hud_text[HUD_LINES][32];

/* AVERAGES, NOT THE LAST FRAME. A figure that changes every frame cannot be read off a tube
 * (reported on the UVMC2, 2026-10-05: "so fast it is impossible to read a number"). So the
 * HUD accumulates for `uvm2_hud_window_us` of real frame time (us_frame_last summed, no extra
 * clock) and then shows, until the next window closes:
 *   F  frames in the window / its length — the real fps, not 1/one frame
 *   C, N, V  per-frame means
 *   D, Z     the WORST frame in the window, so one bad frame is not averaged away
 * 0 refreshes every frame (the host test does that). A game's own lines (uvm2_hud_game)
 * refresh on the same beat. */
volatile uint32_t uvm2_hud_window_us = 1000000u;
static struct { uint32_t frames, us; uint64_t cyc, cmds, vec; uint32_t dropped, clamped; } s_acc;
static struct { uint32_t fps_x10, cyc, cmds, vec, dropped, clamped; int valid; } s_show;

/* A game's own lines under the SDK's two: called once per window with how many frames it
 * covered and their total length in microseconds, so the game can turn its accumulators into
 * per-frame means. It writes up to two lines (empty = not drawn); they stay up until the next
 * call. Weak: a game that does not define it shows two lines, as before. */
__attribute__((weak)) void uvm2_hud_game(uint32_t frames, uint32_t us, char lines[2][32])
{
    (void)frames; (void)us; lines[0][0] = lines[1][0] = 0;
}
static char s_game[2][32];

static void line(int row, const char *txt)
{
    char *d = uvm2_hud_text[row];
    int k = 0;
    while (txt[k] && k < 31) { d[k] = txt[k]; k++; }
    d[k] = 0;
    uvm2_print_text_chained(HUD_X, HUD_Y - row * HUD_STEP, txt, HUD_SCALE, HUD_BRIGHT);
}

/* The combo: count the frames 1+4 (and only those) are held; toggle once when the count
 * reaches HUD_HOLD, and not again until they are released. */
static void combo(void)
{
    static uint32_t held;
    const uint8_t pressed = (uint8_t)~uvm2_cached_buttons & HUD_COMBO_MASK;
    if (pressed != HUD_COMBO) { held = 0; return; }
    if (held < HUD_HOLD && ++held == HUD_HOLD) {
        uvm2_hud = (uint8_t)!uvm2_hud;
        uvm2_hud_stats.toggles++;
    }
}

void uvm2_hud_frame(void)
{
    combo();
    if (!uvm2_hud) {
        uvm2_hud_stats.cmds = uvm2_hud_stats.cycles = uvm2_hud_stats.vectors = 0;
        s_acc.frames = 0; s_show.valid = 0;           /* next time on, start a fresh window */
        return;
    }
    if (uvm2_list_room() < HUD_ROOM) { uvm2_hud_stats.skipped++; return; }

    /* the game's own figures for THIS frame, taken before the HUD adds anything */
    const uint32_t c0 = uvm2_list_commands(), y0 = uvm2_list_cycles(), v0 = uvm2_stats.vectors;
    char t[32], *p;

    s_acc.frames++;
    s_acc.us   += uvm2_stats.us_frame_last;
    s_acc.cyc  += y0; s_acc.cmds += c0; s_acc.vec += v0;
    if (uvm2_stats.dropped > s_acc.dropped)       s_acc.dropped = uvm2_stats.dropped;
    if (uvm2_stats.ramps_clamped > s_acc.clamped) s_acc.clamped = uvm2_stats.ramps_clamped;
    if (!s_show.valid || s_acc.us >= uvm2_hud_window_us) {
        const uint32_t n = s_acc.frames;
        s_show.fps_x10 = s_acc.us ? (uint32_t)(((uint64_t)n * 10000000u + s_acc.us / 2u) / s_acc.us) : 0u;
        s_show.cyc  = (uint32_t)(s_acc.cyc  / n);
        s_show.cmds = (uint32_t)(s_acc.cmds / n);
        s_show.vec  = (uint32_t)(s_acc.vec  / n);
        s_show.dropped = s_acc.dropped; s_show.clamped = s_acc.clamped;
        s_show.valid = 1;
        uvm2_hud_game(n, s_acc.us, s_game);
        s_acc.frames = 0; s_acc.us = 0; s_acc.cyc = s_acc.cmds = s_acc.vec = 0;
        s_acc.dropped = s_acc.clamped = 0;
    }

    p = put_s(t, "F");
    if (s_show.fps_x10) { p = put_u(p, s_show.fps_x10 / 10u); p = put_s(p, "."); p = put_u(p, s_show.fps_x10 % 10u); }
    else p = put_s(p, "--");
    p = put_s(p, " C"); p = put_u(p, s_show.cyc);
    p = put_s(p, " D"); p = put_u(p, s_show.dropped);
    p = put_s(p, " Z"); p = put_u(p, s_show.clamped);
    if (s_show.dropped || s_show.clamped) p = put_s(p, " !!");
    *p = 0;
    line(0, t);

    p = put_s(t, "S"); p = put_u(p, uvm2_stats.stack0_peak);
    p = put_s(p, " "); p = put_u(p, uvm2_stats.stack1_peak);
    if (uvm2_stats.stack_overflow) p = put_s(p, "!");
    p = put_s(p, " N"); p = put_u(p, s_show.cmds);
    p = put_s(p, " V"); p = put_u(p, s_show.vec);
    /* M: the core clock in MHz, as measured at boot against E (uvm2_measure_e: CPU cycles per
     * 1.5 MHz period, Q8). The SDK never sets the clock — a .um2 runs at whatever the
     * cartridge's firmware left — so this is the only way to know it without a probe. */
    p = put_s(p, " M");
    if (uvm2_cycles_per_e_q8) p = put_u(p, (uvm2_cycles_per_e_q8 * 3u + 256u) / 512u);
    else p = put_s(p, "--");
    *p = 0;
    line(1, t);
    for (int g = 0; g < 2; g++) {
        if (s_game[g][0]) line(2 + g, s_game[g]);
        else uvm2_hud_text[2 + g][0] = 0;
    }

    uvm2_hud_stats.cmds    = uvm2_list_commands() - c0;
    uvm2_hud_stats.cycles  = uvm2_list_cycles() - y0;
    uvm2_hud_stats.vectors = uvm2_stats.vectors - v0;
    uvm2_hud_stats.drawn++;
}
