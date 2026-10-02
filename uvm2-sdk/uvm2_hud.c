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
 *     S2624 812 N3614 V322      deepest core 0 / core 1 stack in bytes ("!" after them if
 *                               either overflowed), the game's commands and lit vectors
 *                               this frame
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
#define HUD_ROOM        1600u

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
#define HUD_LINES 2
char uvm2_hud_text[HUD_LINES][32];

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
    if (!uvm2_hud) { uvm2_hud_stats.cmds = uvm2_hud_stats.cycles = uvm2_hud_stats.vectors = 0; return; }
    if (uvm2_list_room() < HUD_ROOM) { uvm2_hud_stats.skipped++; return; }

    /* the game's own figures for THIS frame, taken before the HUD adds anything */
    const uint32_t c0 = uvm2_list_commands(), y0 = uvm2_list_cycles(), v0 = uvm2_stats.vectors;
    char t[32], *p;

    p = put_s(t, "F");
    if (uvm2_stats.us_frame_last) p = put_u(p, (1000000u + uvm2_stats.us_frame_last / 2u) / uvm2_stats.us_frame_last);
    else p = put_s(p, "--");
    p = put_s(p, " C"); p = put_u(p, y0);
    p = put_s(p, " D"); p = put_u(p, uvm2_stats.dropped);
    p = put_s(p, " Z"); p = put_u(p, uvm2_stats.ramps_clamped);
    if (uvm2_stats.dropped || uvm2_stats.ramps_clamped) p = put_s(p, " !!");
    *p = 0;
    line(0, t);

    p = put_s(t, "S"); p = put_u(p, uvm2_stats.stack0_peak);
    p = put_s(p, " "); p = put_u(p, uvm2_stats.stack1_peak);
    if (uvm2_stats.stack_overflow) p = put_s(p, "!");
    p = put_s(p, " N"); p = put_u(p, c0);
    p = put_s(p, " V"); p = put_u(p, v0);
    *p = 0;
    line(1, t);

    uvm2_hud_stats.cmds    = uvm2_list_commands() - c0;
    uvm2_hud_stats.cycles  = uvm2_list_cycles() - y0;
    uvm2_hud_stats.vectors = uvm2_stats.vectors - v0;
    uvm2_hud_stats.drawn++;
}
