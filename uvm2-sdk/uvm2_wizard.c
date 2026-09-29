/* uvm2_wizard.c — THE CALIBRATION SCREEN.
 *
 * See uvm2_config.h for the reasoning behind the four parameters. This is only the interface.
 *
 * THE PATTERN MEASURES, IT DOES NOT DECORATE. TWO squares of the SAME size are drawn one above
 * the other: the top one with 4 long strokes, the bottom one with 40 short ones. Since they measure
 * the same, any difference between them comes from the NUMBER of strokes and not from their
 * length, and that separates the error's two terms:
 *
 *     the 40 one opens and the 4 one does not  -> it is the FIXED per-stroke term (t1_tail_q8)
 *     both open in proportion                  -> it is the SCALE                 (scale)
 *
 * A closed polygon that opens accumulates the fixed loss N times; a scale error would make it
 * smaller but it would still be closed. That is why the reference cartridge has one screen for
 * vectors and another for text: they are these same two terms.
 *
 * WITH ZERO SELECTED THE PATTERN IS TEXT INSTEAD: see zero_pattern.
 *
 * CONTROLLER: up/down picks a parameter, left/right moves it, button 4 saves and exits.
 * HOW TO GET HERE: hold buttons 2 and 3 while launching the game with 4 (uvm2_config_boot_combo).
 */
#include "uvm2_config.h"
#include "uvm2_draw.h"
#include "uvm2_text.h"
#include "uvm2_bus.h"

void uvm2_core1_start(void);
void uvm2_core1_stop(void);

/* INPUT IS READ FROM THE CACHE, NOT FROM THE BUS.
 *
 * `uvm2_read_buttons()` and `uvm2_read_axes()` talk to the PSG over the Vectrex bus — and CORE
 * 1 is using that bus to replay the list. Calling them from core 0, as this did when it was
 * written, puts both cores on the same bus: measured on the console, the wizard ran at 1 fps
 * with `us_exec = 1,125,454 us` (1.1 seconds executing a list of 17,111 cycles, i.e. 11 ms of
 * work). The emulator does NOT reproduce it: there it ran at 50 Hz, because it does not model
 * bus contention.
 *
 * Core 1 already reads them once per frame and leaves them here; games read from here. The
 * buttons arrive RAW, active low (0 = pressed). */
extern volatile uint8_t  uvm2_cached_buttons;
extern volatile uint32_t uvm2_cached_axes;

#define SIDE      44      /* the square's side, in device units */
#define SEP       56      /* distance between the two squares' centres */
/* uvm2_print_text's `scale` is in HALF units and its stock value is 3 (x1.5). I once used 1
 * — a third of normal — and on the console it was unreadable. 4 is x2. */
#define TEXT      3      /* the wizard's letter size */
#define STEP      19     /* distance between lines */
#define VISIBLE   5      /* how many fit at once in the screen's +-127 */

/* THE WHEEL: an octagon and its eight spokes. It replaced, on 2026-09-28, a figure copied
 * from another game's calibration screen — eight crooked segments that on the console read as
 * a broken drawing whether or not anything was wrong, which is useless for judging by eye.
 *
 * A wheel has one right answer that anybody can see:
 *
 *     the octagon closes                        -> long strokes arrive where they were aimed
 *     the spokes meet in ONE point              -> every jump back to the centre lands there
 *     each spoke ends on its corner             -> jumps and strokes agree about distance
 *     the diagonals are at 45 degrees and the
 *     opposite spokes form straight lines       -> X and Y move alike, positive and negative
 *
 * The corners are (34, 0) and (24, 24) and their mirrors: 24 * sqrt(2) = 33.9, so the eight
 * corners sit on one circle to within 0.1 unit and the octagon is regular on the integer grid. */
#define WHEEL_R   34      /* the corners on the axes */
#define WHEEL_D   24      /* the diagonal corners: WHEEL_R / sqrt(2), rounded */
static const signed char WHEEL[8][2] = {
    {  WHEEL_R,        0 }, {  WHEEL_D,  WHEEL_D }, {        0,  WHEEL_R }, { -WHEEL_D,  WHEEL_D },
    { -WHEEL_R,        0 }, { -WHEEL_D, -WHEEL_D }, {        0, -WHEEL_R }, {  WHEEL_D, -WHEEL_D },
};

static void wheel(int cx, int cy)
{
    /* The rim: one chained polygon, so its closing is the test of the long strokes. */
    uvm2_draw_move_abs(cx + WHEEL[0][0], cy + WHEEL[0][1]);
    for (int k = 1; k <= 8; k++)
        uvm2_draw_delta(WHEEL[k & 7][0] - WHEEL[(k - 1) & 7][0], WHEEL[k & 7][1] - WHEEL[(k - 1) & 7][1]);
    /* The spokes: each one a JUMP back to the centre and a stroke out, so where they meet is the
     * test of the jumps. */
    for (int k = 0; k < 8; k++) {
        uvm2_draw_move_abs(cx, cy);
        uvm2_draw_delta(WHEEL[k][0], WHEEL[k][1]);
    }
}

/* THE ZERO PATTERN: SEVERAL LINES OF TEXT, DRAWN THE WAY A GAME DRAWS THEM.
 *
 * Shown while ZERO is the selected field. A zero reference that is wrong for this console adds
 * the same velocity to every ramp, and nothing makes that more obvious than rows of short
 * strokes run without a re-zero: each row leans into a diagonal and the glyphs slant. That is
 * what a tester's console showed with the AAE ports on 2026-09-28 (and the VecFever writes 0x07
 * where the SDK's default is 0x23 — see UVM2_ZERO_OFFSET).
 *
 * The two long strokes are the reference: one ramp each, so they barely carry the offset.
 * Adjust ZERO until every row runs parallel to the top line and every column stands parallel
 * to the left one. */
static const char *const ZERO_ROWS[] = {
    "HIGH SCORES",
    "1 DBC 0025350",
    "2 WAN 0019420",
    "3 HAN 0017880",
    "4 GAR 0012750",
    "5 MLH 0010030",
};
#define ZERO_LEFT   (-112)   /* the rows' left edge */
#define ZERO_TOP    118      /* the first row's top */
#define ZERO_ROW    15       /* distance between rows */

static void zero_pattern(int bright)
{
    const int rows = (int)(sizeof ZERO_ROWS / sizeof ZERO_ROWS[0]);
    const int bottom = ZERO_TOP - rows * ZERO_ROW;
    uvm2_draw_move_abs(ZERO_LEFT - 6, ZERO_TOP + 4);           /* the top line */
    uvm2_draw_delta(160, 0);
    uvm2_draw_move_abs(ZERO_LEFT - 6, ZERO_TOP + 4);           /* the left line */
    uvm2_draw_delta(0, bottom - ZERO_TOP - 4);
    for (int r = 0; r < rows; r++)
        uvm2_print_text_chained(ZERO_LEFT, ZERO_TOP - r * ZERO_ROW, ZERO_ROWS[r], TEXT, bright);
}

/* A square drawn with `n` strokes per side, centred on (cx, cy). With n = 1 it is the 4 long
 * strokes; with n = 10, the 40 short ones. The total travel is THE SAME. */
static void square(int cx, int cy, int n)
{
    static const int dx[4] = { 1, 0, -1, 0 };
    static const int dy[4] = { 0, 1,  0, -1 };
    const int side = SIDE;
    uvm2_draw_move_abs(cx - side / 2, cy - side / 2);
    for (int l = 0; l < 4; l++) {
        /* The exact split, so n strokes measure the same as one: the ideal position is
         * accumulated and the difference emitted, instead of repeating side/n and losing the
         * remainder n times. */
        int done = 0;
        for (int i = 1; i <= n; i++) {
            int ideal = side * i / n;
            uvm2_draw_delta(dx[l] * (ideal - done), dy[l] * (ideal - done));
            done = ideal;
        }
    }
}

/* `labels`, WHEN A NUMBER IS NOT AN ANSWER.
 *
 * A switch shown as a digit is a switch nobody can use: "AUDIO 0" does not say whether zero
 * is the jack or the console, and the player has no manual on the sofa. With `labels` set, the
 * value indexes it and the word is drawn instead — the field still behaves like a number
 * underneath, so nothing else in here changes.
 *
 * Left NULL by every field that really is a number (a compound literal with fewer
 * initialisers zero-fills the rest, so the four console fields below say nothing about it).
 * It must hold max-min+1 entries. */
struct field { const char *name; int32_t *value; int32_t min, max, step;
               const char *const *labels; };

static const char *const LBL_ONOFF[] = { "OFF", "ON" };
/* JACK first because 0 is the jack: a config file written before this setting existed brings
 * a zero, and on the UVMC2 the jack is what it was already doing. See uvm2_config.h. */
static const char *const LBL_AUDIO[] = { "JACK", "CONSOLE" };

/* THE GAME CAN SUPPLY THE FIGURE.
 *
 * The SDK's pattern (the reference figure and the two squares) separates the error's two terms
 * well, but calibrating against it is not the same as calibrating against WHAT LOOKS WRONG.
 * The arbiter is the drawing that bothers you, not a laboratory figure.
 *
 * With `figure` null the usual pattern is drawn. With a function, that one is drawn IN ITS
 * PLACE and the rest of the screen — the values, the controller, the saving — is identical.
 * The game passes it already centred and at its own scale: the wizard knows nothing about
 * it. */
int uvm2_config_wizard_with(void (*figure)(void))
{
    struct uvm2_config c;
    uvm2_config_current(&c);

    /* THE PARAMETERS, TAKEN FROM A REFERENCE GAME AND NOT DEDUCED BY ME.
     *
     * Vectorblade's `calibration.asm` has THREE screens and all three adjust ONE thing: a byte
     * primed into the ZERO REFERENCE (`calibrateString` does `ORB=$82` — mux on channel 1 — and
     * `ORA = calibrationValueString`), which is exactly what our zero block emits with
     * `uvm2_zero_offset`.
     *
     * Its factory values: `calibrationValue16 = $23` (35) and `calibrationValue50 = $56` (86).
     * **Ours was at 7**, thirty units below where it starts to matter — which is why moving it
     * from 7 to 5 on the console did nothing.
     *
     * It has THREE because the value depends on the SCALE of what is drawn: one for the boss
     * (long strokes), another for text. It is the same division others describe ("separate ones
     * for vectors and text"), and not two terms of an error model as I had assumed.
     *
     * `scale` and `t1_tail_q8` stay because they are REAL drawing knobs, but at the end: they
     * are not what calibrates a console.
     *
     * THE CONSOLE'S ALWAYS; THE GAME'S, ONLY THOSE THE GAME DECLARES AS ITS OWN
     * (uvm2_config_game). A switch that means nothing in this game is not shown: Donkey Kong is
     * vertical and has no business seeing a ROTATE, and the menu is hidden per game and not for
     * everyone. */
    /* Four of the console's plus every one the game declares. It was 8, which is EXACTLY
     * full now that AUDIO exists — and a fifth game setting would have written past the end
     * with nothing to say so. */
    struct field fields[12];
    int n = 0;
    fields[n++] = (struct field){ "ZERO",   &c.zero,       0, 255, 1 };
    fields[n++] = (struct field){ "BRIGHT", &c.bright,     0, 127, 1 };
    fields[n++] = (struct field){ "SCALE",  &c.scale,   80, 400, 1 };
    /* TAIL's ceiling is TWICE THE MEASURED VALUE, not 512. The default is 640 (2.5 E cycles,
     * T1_EXTRA_Q8 in ramp.rs, measured 2026-09-15), and with a ceiling of 512 the first touch
     * of the stick clamped it to 512 — the calibration moved without anyone asking, and it
     * could never be put back. Reported on the console 2026-09-28: "tail no sube de 512". */
    fields[n++] = (struct field){ "TAIL",   &c.t1_tail_q8, -512, 1280, 8 };
    {
        const unsigned mine = uvm2_config_game_settings();
        /* ROTATE: the screen is vertical and quite a few arcade machines are horizontal. It
         * shows instantly on the figure itself, which is exactly what a setting like this
         * needs. */
        if (mine & UVM2_SETTING_ROTATE) fields[n++] = (struct field){ "ROTATE", &c.rotate,     0, 1, 1, LBL_ONOFF };
        if (mine & UVM2_SETTING_MENU)   fields[n++] = (struct field){ "MENU",   &c.start_menu, 0, 1, 1, LBL_ONOFF };
        if (mine & UVM2_SETTING_HZ)     fields[n++] = (struct field){ "HZ",     &c.hz,         0, 60, 10 };
        /* WHERE THE SOUND COMES OUT. The setting has been stored and loaded since the jack
         * existed, and the field table is what was missing — so a game could declare it and
         * the player still had no way to change it. The SDK does not route anything here: it
         * moves the value, and the game that declared it reads uvm2_setting_audio and decides
         * what that means. */
        if (mine & UVM2_SETTING_AUDIO)  fields[n++] = (struct field){ "AUDIO",  &c.audio,      0, 1, 1, LBL_AUDIO };
    }
    int sel = 0, saved = 0;
    /* THE STICK'S REST POSITION IS NOT ZERO, AND ASSUMING IT WAS BROKE THIS SCREEN.
     *
     * Reported on the console 2026-09-29: "push down, and pressing down again does not move
     * until I change the value". The re-arm below needed the axis inside +-25 for three
     * frames, and this console's Y does not rest that close to zero — so after one move it
     * never re-armed. Pushing left or right appeared to fix it because moving X brings a
     * diagonally-held Y back towards the middle, which let the settle finish.
     *
     * So the centre is MEASURED instead of assumed, over the first eight frames. The wizard
     * is opened by a button combo and never by the stick, so at entry the stick is at rest
     * by construction — and eight frames is 0.16 s, which nobody can push a stick inside.
     * Clamped to +-40 so that if somebody IS holding it, a wrong centre stays small.
     *
     * Both axes, not just Y. An uncentred X is worse than an awkward menu: left/right is the
     * ADJUST, so a resting offset past the threshold walks the selected calibration value on
     * its own, with nobody touching anything. */
    int cx = 0, cy = 0, c_acc_x = 0, c_acc_y = 0, c_n = 0;
    /* Locals, not statics. They were `static` inside the loop, which persists across wizard
     * sessions: the second opening in one power-up started with whatever the first left. */
    int armed = 1, settled = 0;
    uint32_t tick = 0;
    /* THE BUTTONS ARE ACTIVE LOW (PSG reg 14 raw: 0 = pressed), so they are inverted here ONCE
     * and the rest of the code reasons with 1 = pressed. Without inverting, the edge detects
     * the RELEASE and at rest every bit is 1. */
    uint8_t before = (uint8_t)~uvm2_cached_buttons;

    for (;;) {
        uvm2_config_apply(&c);          /* the effect is visible WHILE it is moved */
        uvm2_frame_begin();
        uvm2_draw_intensity(c.bright);

        if (fields[sel].value == &c.zero) {
            zero_pattern(c.bright);       /* the zero's own pattern, whatever the game passed */
        } else if (figure) {
            figure();                     /* the game's, see above */
        } else {
            /* The wheel, and beside it the two squares — 4 strokes against 40 — which
             * separate the fixed per-stroke term from the scale. */
            wheel(-50, 38);
            square( 70, 60,  1);
            square( 70, 10, 10);
        }

        /* A WINDOW, NOT THE WHOLE LIST, like dkong's menu: with the game settings it is up to
         * seven lines, and at 26 units per step from -18 the list ran off the bottom of the
         * screen — you saw four and a half. VISIBLE of them are shown with the selected one
         * centred, and the letter size drops to something that fits. */
        int first = sel - VISIBLE / 2;
        if (first > n - VISIBLE) first = n - VISIBLE;
        if (first < 0) first = 0;
        for (int w = 0; w < VISIBLE && first + w < n; w++) {
            const int i = first + w;
            char line[24];
            int p = 0;
            line[p++] = (i == sel) ? '>' : ' ';
            for (const char *s = fields[i].name; *s; s++) line[p++] = *s;
            line[p++] = ' ';
            /* The value, by hand: the SDK has no printf and pulling it in for this would cost
             * 20 KB of flash for four numbers. A field with `labels` draws the word instead;
             * the range check is the labels' bound, so a value out of range falls back to the
             * digits rather than reading past the table. */
            int32_t v = *fields[i].value;
            if (fields[i].labels && v >= fields[i].min && v <= fields[i].max) {
                for (const char *s = fields[i].labels[v - fields[i].min]; *s; s++) line[p++] = *s;
            } else {
                if (v < 0) { line[p++] = '-'; v = -v; }
                char d[8]; int k = 0;
                do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 7);
                while (k) line[p++] = d[--k];
            }
            line[p] = 0;
            uvm2_print_text(-112, -22 - w * STEP, line, TEXT, c.bright);
        }
        uvm2_frame_end();

        /* THE CONTROLLER, ON EDGES. Without this one tap moves the value thirty times: the
         * loop runs at 50 Hz and a finger takes longer. */
        uint8_t b = (uint8_t)~uvm2_cached_buttons;
        uint8_t pressed = (uint8_t)(b & ~before);
        before = b;
        /* (J1X << 24) | (J1Y << 16) | (J2X << 8) | J2Y — controller 1 is in the HIGH bytes.
         * Reading the low ones reads controller 2, which is what this did when it was
         * written. */
        uint32_t axes = uvm2_cached_axes;
        int jx = (int8_t)(axes >> 24), jy = (int8_t)(axes >> 16);

        /* The first eight frames only measure; the stick does nothing yet. */
        if (c_n < 8) {
            c_acc_x += jx; c_acc_y += jy;
            if (++c_n == 8) {
                cx = c_acc_x / 8; cy = c_acc_y / 8;
                if (cx >  40) cx =  40;  if (cx < -40) cx = -40;
                if (cy >  40) cy =  40;  if (cy < -40) cy = -40;
            }
            goto after_input;
        }
        jx -= cx;
        jy -= cy;

        /* UP/DOWN SELECTS: ONE STEP PER EXCURSION, WITH HYSTERESIS AND SETTLING.
         *
         * Reported on this screen: "push down and bring the stick back to centre, and it goes
         * back up". The stick is a SPRING: on release it crosses the centre and overshoots to
         * the other side for a few frames. What used to be here re-armed as soon as the axis
         * entered +-40, so that bounce counted as a second excursion — in the opposite
         * direction.
         *
         * The rule is the one already validated in dkong's menu, with the SAME complaint and
         * the same words: it fires on crossing +-60 and does not re-arm until the axis has
         * spent three consecutive frames within +-25. The spring's bounce lasts less than that
         * and falls entirely inside the unarmed period. */
        {
            int step = 0;
            /* 60 fires and 35 re-arms: 25 units of hysteresis, which the spring's bounce
             * stays inside, and far enough from zero that a measured centre a few units out
             * still settles. It was 25, chosen when the centre was assumed to be 0. */
            if (jy > -35 && jy < 35) {
                if (settled < 3) settled++;
                if (settled >= 3) armed = 1;
            } else {
                settled = 0;
                if (armed) {
                    if (jy >= 60)      { armed = 0; step = -1; }   /* up */
                    else if (jy <= -60){ armed = 0; step = +1; }   /* down */
                }
            }
            if (step) sel = (sel + n + step) % n;
        }

        /* LEFT/RIGHT ADJUSTS, CONTINUOUSLY WHILE HELD — like the reference game, which moves
         * +-1 every two frames (`Vec_Loop_Count+1 & 1`). On edges it was useless: the zero
         * reference's useful range runs from 0 to 255, and at one step per press it takes a
         * hundred taps to reach where it starts to show. */
        tick++;
        if ((jx > 40 || jx < -40) && (tick & 1u) == 0) {
            int32_t *v = fields[sel].value;
            *v += (jx > 0 ? fields[sel].step : -fields[sel].step);
            if (*v < fields[sel].min) *v = fields[sel].min;
            if (*v > fields[sel].max) *v = fields[sel].max;
        }

after_input:
        if (pressed & 0x08) {           /* button 4: save and exit */
            /* CORE 1 IS NOT STOPPED HERE, AND THIS COMMENT USED TO LIE.
             *
             * It said "stop core 1 before touching the flash", and that was true when
             * `uvm2_config_save` wrote to flash. Not any more: that path is disabled
             * (`save_to_flash_DO_NOT_USE`, and the reason is there) and it now writes to the
             * SD. The comment stayed and so did the call.
             *
             * And it is not harmless: `uvm2_frame_end` has core 0's ONLY wait —
             * `while (uvm2_frame_done - (s_frame_no-1) < 0)` — and the one that advances that
             * counter is core 1. Reset it and core 0 stays there for ever. It hung dkong on the
             * console on 2026-09-14; over SWD, pc pinned in uvm2_draw.c in three samples, and
             * the screen NOT black because core 1 kept repeating the last list. */
            saved = uvm2_config_save();
            break;
        }
    }
    return saved;
}

int uvm2_config_wizard(void) { return uvm2_config_wizard_with(0); }

/* HOLD BUTTONS 2 AND 3 WHILE LAUNCHING THE GAME, AND THE WIZARD OPENS FIRST.
 *
 * WHY 2+3 AND NOT 1+4, WHICH IT WAS FOR A DAY: in the cartridge's own menu button 4 launches
 * the game and button 1 goes back, so holding 1 while pressing 4 never started anything. The
 * menu uses neither 2 nor 3, so they can be held through the launch with a thumb while the
 * other presses 4 as usual.
 *
 * Until this existed no game opened the wizard, so a console whose zero differs from the
 * default had no way to fix it short of editing config/uvm2.cfg on a PC.
 *
 * Core 1 refreshes the button cache every frame period even with no list published, so this
 * waits until it has done so a few times — a cache still at its power-on value would read as
 * "not held" and the check would be one that cannot fire. `uvm2_boot_combo` records the
 * outcome for SWD: -1 never checked, 0 not held, 1 held and the wizard ran, 2 core 1 never
 * refreshed the cache (the check could not be made). */
volatile int32_t uvm2_boot_combo = -1;

void uvm2_config_boot_combo(void)
{
    /* FIRST, AND WHETHER OR NOT THE COMBO IS HELD. The game's settings have to be declared
     * before the wizard can show them, and loaded before the first sound plays — and this is
     * the one place that runs before the game does on both start-up paths. Empty unless the
     * game defines it. */
    uvm2_game_settings();

    /* uvm2_stats is not volatile, and core 1 is the one advancing this: read it as volatile
     * or the loop may never see it move. */
    volatile const uint32_t *idle = &uvm2_stats.idle_frames;
    const uint32_t start = *idle;
    /* Bounded by a spin count, not a clock, so it cannot hang if core 1 never runs. At the
     * 20 ms idle period, 3 refreshes are ~60 ms. */
    for (uint32_t spin = 0; *idle - start < 3u; spin++) {
        if (spin > 50000000u) { uvm2_boot_combo = 2; return; }
    }
    const uint8_t held = (uint8_t)~uvm2_cached_buttons;       /* active low: 1 = pressed */
    if ((held & 0x06u) != 0x06u) { uvm2_boot_combo = 0; return; }   /* buttons 2 (bit 1) and 3 (bit 2) */
    uvm2_boot_combo = 1;
    uvm2_config_wizard();
}
