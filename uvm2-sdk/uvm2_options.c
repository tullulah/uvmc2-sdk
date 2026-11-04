/* uvm2_options.c — THE STANDARD OPTIONS SCREEN every game can have.
 *
 * Before this, each game that wanted options built its own: dkong has a start menu with
 * REFRESH, START IN MENU and AUDIO; mhavoc a "HOLD 4 FOR SETTINGS" window into the calibration
 * screen; most ports nothing, and the game settings that did exist (ROTATE, AUDIO, HZ) were
 * only reachable at the bottom of the CALIBRATION screen, among ZERO, TAIL and ASPECT.
 *
 * This is dkong's menu made generic. A game opts in with UVM2_SETTING_OPTIONS in its
 * uvm2_config_game() (from uvm2_game_settings, so it is loaded before this runs):
 *
 *     void uvm2_game_settings(void)
 *     {
 *         uvm2_config_title("ALPHA ONE");
 *         uvm2_config_game("ALPHAONE", UVM2_SETTING_OPTIONS | UVM2_SETTING_ROTATE
 *                                    | UVM2_SETTING_AUDIO);
 *         uvm2_config_load();
 *     }
 *
 * The lines, top to bottom; each only if the game has it:
 *
 *     PLAY
 *     SCREEN VERTICAL / HORIZONTAL     UVM2_SETTING_ROTATE (horizontal arcade machines):
 *                                      how the CONSOLE stands; HORIZONTAL turns the drawing
 *     SOUND JACK / VECTREX             UVM2_SETTING_AUDIO  (games with sound on the jack)
 *     REFRESH <game's> / 50 / 60 / MAX always (the game's own rate first: UVM2_HZ)
 *     BRIGHTNESS 100                   always, percent (a gain on every intensity, uvm2_draw.c)
 *     CONTROLS                         always: which console button does each of the game's
 *                                      four (controls_menu, saved as `buttons`)
 *     CALIBRATE                        always (the SDK's calibration screen)
 *     START IN MENU / START IN GAME    always
 *
 * Stick up/down picks a line, left/right changes it, button 1 changes it too (and on PLAY,
 * plays), button 4 plays from anywhere.
 *
 * WHEN IT OPENS (uvm2_options_boot): with START IN MENU, every launch; with START IN GAME,
 * only when button 4 is HELD for a second at launch -- otherwise turning it off would be for
 * good. Held, not pressed: a launch from a menu by button 4 lasts a few frames with the button
 * still down, and dkong learnt that every launch then looked like a request (dkong menu.c).
 *
 * SOUND AND SCREEN ARE DECIDED HERE, BEFORE THE GAME RUNS, and that is why this sits before
 * the game's main and not behind a pause combo: a port reads uvm2_setting_audio once at init
 * (mh_jack_init, dk_board_select...), so changing it under a running game would need every
 * port to support switching live.
 */
#include <stdint.h>
#include "uvm2_config.h"
#include "uvm2_draw.h"
#include "uvm2_text.h"
#include "uvm2_bus.h"

extern volatile uint8_t  uvm2_cached_buttons;   /* core 1's copy, ACTIVE LOW */
extern volatile uint32_t uvm2_cached_axes;

#define TEXT        3      /* letter size, the calibration screen's */
#define ADVANCE    10      /* (7 * TEXT) >> 1: uvm2_text.c's advance per letter */
#define ROW        18      /* distance between lines */
#define LEFT      (-72)    /* the options' left edge */
#define TOP        62      /* the first option */
#define INK        90      /* the text's own intensity, before the brightness gain */
#define HOLD_FRAMES 50     /* button 4 held this long at launch opens the screen: one second */
#define BRIGHT_STEP 10
#define BRIGHT_LO   50
#define BRIGHT_HI  200

enum { OPT_PLAY, OPT_SCREEN, OPT_SOUND, OPT_REFRESH, OPT_BRIGHT, OPT_CONTROLS, OPT_CALIBRATE,
       OPT_START };

/* ── THE STICK, AS A MENU READS IT ──────────────────────────────────────────────────────
 *
 * Its rest is MEASURED over the first eight frames, as the calibration screen does
 * (uvm2_wizard.c): this console's Y does not rest at zero, and assuming it did left the menu
 * unable to re-arm after one step. And one step per push: the stick has to come back to the
 * centre, and stay there three frames, before it moves anything again. */
struct nav { int cx, cy, acc_x, acc_y, acc_n, armed_x, armed_y, calm; };

/* One frame of it: 0 while it is still measuring the rest, else 1 with the steps taken. */
static int nav_read(struct nav *v, int *step_x, int *step_y)
{
    const uint32_t axes = uvm2_cached_axes;
    int jx = (int8_t)(axes >> 24), jy = (int8_t)(axes >> 16);
    *step_x = *step_y = 0;
    if (v->acc_n < 8) {
        v->acc_x += jx; v->acc_y += jy;
        if (++v->acc_n == 8) {
            v->cx = v->acc_x / 8; v->cy = v->acc_y / 8;
            if (v->cx > 40) v->cx = 40;  if (v->cx < -40) v->cx = -40;
            if (v->cy > 40) v->cy = 40;  if (v->cy < -40) v->cy = -40;
        }
        return 0;
    }
    jx -= v->cx; jy -= v->cy;
    if (jx > -35 && jx < 35 && jy > -35 && jy < 35) {
        if (v->calm < 3) v->calm++;
        if (v->calm >= 3) v->armed_x = v->armed_y = 1;
    } else v->calm = 0;
    if (v->armed_y && (jy >= 60 || jy <= -60)) { v->armed_y = 0; *step_y = jy >= 60 ? -1 : +1; }
    if (v->armed_x && (jx >= 60 || jx <= -60)) { v->armed_x = 0; *step_x = jx >= 60 ? +1 : -1; }
    return 1;
}

static void text_centred(int y, const char *s)
{
    int n = 0;
    while (s[n]) n++;
    uvm2_print_text(-(n * ADVANCE) / 2, y, s, TEXT, INK);
}

/* "BRIGHTNESS 120" without printf: this runs in every game image. Percent, but with no
 * sign: the SDK font has no strokes for '%' (uvm2_font.h, length 0). */
static const char *bright_label(void)
{
    static char line[20];
    const char *t = "BRIGHTNESS ";
    int p = 0, v = (int)uvm2_setting_bright, k = 0;
    char d[4];
    while (*t) line[p++] = *t++;
    do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 3);
    while (k) line[p++] = d[--k];
    line[p] = 0;
    return line;
}

/* "REFRESH 38 HZ" / "REFRESH MAX", for whatever rate is set. */
static const char *refresh_label(void)
{
    static char line[20];
    const char *t = "REFRESH ";
    int p = 0, v = (int)uvm2_setting_hz, k = 0;
    char d[4];
    while (*t) line[p++] = *t++;
    if (v == 0) { t = "MAX"; while (*t) line[p++] = *t++; line[p] = 0; return line; }
    do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 3);
    while (k) line[p++] = d[--k];
    t = " HZ";
    while (*t) line[p++] = *t++;
    line[p] = 0;
    return line;
}

/* THE REFRESH CHOICES: the game's own first (Armor Attack's 38), then 50 and 60 if they are
 * not that, then flat out. Cycled in either direction. */
static int refresh_step(int dir)
{
    int opts[4], n = 0, at = 0;
    const int native = uvm2_config_native_hz();
    opts[n++] = native;
    if (native != 50) opts[n++] = 50;
    if (native != 60) opts[n++] = 60;
    opts[n++] = 0;
    for (int i = 0; i < n; i++) if (opts[i] == uvm2_setting_hz) at = i;
    return opts[(at + n + dir) % n];
}

static const char *label(int opt)
{
    switch (opt) {
    case OPT_PLAY:      return "PLAY";
    /* HOW THE CONSOLE STANDS, not how the arcade's monitor did. VERTICAL is the Vectrex
     * upright, the drawing as the game makes it; HORIZONTAL is the console on its side, the
     * drawing turned 90 degrees to fill it. Named the other way round at first (after the
     * cabinet) and on the console "HORIZONTAL" was the default and changed nothing. */
    case OPT_SCREEN:    return uvm2_setting_rotate ? "SCREEN HORIZONTAL" : "SCREEN VERTICAL";
    case OPT_SOUND:     return uvm2_setting_audio ? "SOUND VECTREX" : "SOUND JACK";
    case OPT_REFRESH:   return refresh_label();
    case OPT_BRIGHT:    return bright_label();
    case OPT_CONTROLS:  return "CONTROLS";
    case OPT_CALIBRATE: return "CALIBRATE";
    default:            return uvm2_setting_menu ? "START IN MENU" : "START IN GAME";
    }
}

/* One step of a line's value, dir = +1 / -1. Returns 1 if something changed. */
static int change(int opt, int dir)
{
    switch (opt) {
    case OPT_SCREEN:  uvm2_setting_rotate = !uvm2_setting_rotate; return 1;
    case OPT_SOUND:   uvm2_setting_audio  = !uvm2_setting_audio;  return 1;
    case OPT_START:   uvm2_setting_menu   = !uvm2_setting_menu;   return 1;
    case OPT_REFRESH:
        uvm2_setting_hz = refresh_step(dir);
        return 1;
    case OPT_BRIGHT: {
        int32_t v = uvm2_setting_bright + dir * BRIGHT_STEP;
        if (v < BRIGHT_LO) v = BRIGHT_LO;
        if (v > BRIGHT_HI) v = BRIGHT_HI;
        if (v == uvm2_setting_bright) return 0;
        uvm2_setting_bright = v;
        return 1;
    }
    default: return 0;
    }
}

/* ── CONTROLS: THE GAME'S FOUR BUTTONS, AND WHICH ONE OF THE CONSOLE'S DOES EACH ──────────
 *
 *     FIRE         BUTTON 2
 *     TURRET L     BUTTON 1      the game's names (uvm2_config_buttons), then the
 *     ...                        physical button that does it
 *     DEFAULTS
 *     BACK
 *
 * Stick up/down picks a line; PRESSING A BUTTON on a game line gives that button to it, and
 * the line that had it gets this line's old one -- a swap, so the four stay four. Any button
 * on DEFAULTS puts them back as wired, on BACK leaves. Every button does the same thing here
 * because every button is what is being assigned. Returns 1 if the map changed. */
#define C_NAME  (-110)     /* the game's name for the button */
#define C_PHYS    30       /* "BUTTON n" */
static int controls_menu(struct nav *nv)
{
    const int n = 6;                                  /* 4 buttons, DEFAULTS, BACK */
    const int start = uvm2_setting_buttons;
    int sel = 0;
    uint8_t before = 0xFF;                            /* the press that opened this */
    char phys[4][9];
    for (;;) {
        int32_t m = uvm2_setting_buttons;
        for (int i = 3; i >= 0; i--) {
            const char *t = "BUTTON ";
            int p = 0;
            while (*t) phys[i][p++] = *t++;
            phys[i][p++] = (char)('0' + m % 10);
            phys[i][p] = 0;
            m /= 10;
        }
        uvm2_draw_rotate((int)uvm2_setting_rotate);
        uvm2_frame_begin();
        uvm2_draw_intensity(INK);
        text_centred(TOP + 40, "CONTROLS");
        for (int i = 0; i < n; i++) {
            const int y = TOP - i * ROW;
            if (i == sel) uvm2_print_text(C_NAME - 16, y, ">", TEXT, INK);
            if (i < 4) {
                uvm2_print_text(C_NAME, y, uvm2_config_button_name(i), TEXT, INK);
                uvm2_print_text(C_PHYS, y, phys[i], TEXT, INK);
            } else uvm2_print_text(C_NAME, y, i == 4 ? "DEFAULTS" : "BACK", TEXT, INK);
        }
        text_centred(TOP - n * ROW - 14, sel < 4 ? "PRESS ITS BUTTON" : "ANY BUTTON");
        uvm2_frame_end();

        const uint8_t b = (uint8_t)(~uvm2_cached_buttons & 0x0F);
        const uint8_t pressed = (uint8_t)(b & ~before);
        before = b;
        int sx, sy;
        if (!nav_read(nv, &sx, &sy)) continue;
        if (sy) sel = (sel + n + sy) % n;
        if (!pressed) continue;
        if (sel == 5) break;
        if (sel == 4) { uvm2_buttons_set(1234); continue; }
        /* The lowest button pressed is the one given; whoever had it takes this line's. */
        int phys_b = 0;
        while (!(pressed & (1u << phys_b))) phys_b++;
        int digits[4];
        int32_t m2 = uvm2_setting_buttons;
        for (int i = 3; i >= 0; i--) { digits[i] = (int)(m2 % 10); m2 /= 10; }
        for (int i = 0; i < 4; i++)
            if (digits[i] == phys_b + 1) digits[i] = digits[sel];
        digits[sel] = phys_b + 1;
        uvm2_buttons_set(digits[0] * 1000 + digits[1] * 100 + digits[2] * 10 + digits[3]);
    }
    return uvm2_setting_buttons != start;
}

int uvm2_options_menu(void)
{
    const unsigned mine = uvm2_config_game_settings();
    int opts[10], n = 0;
    opts[n++] = OPT_PLAY;
    if (mine & UVM2_SETTING_ROTATE) opts[n++] = OPT_SCREEN;
    if (mine & UVM2_SETTING_AUDIO)  opts[n++] = OPT_SOUND;
    opts[n++] = OPT_REFRESH;
    opts[n++] = OPT_BRIGHT;
    if (mine & UVM2_SETTING_BUTTONS) opts[n++] = OPT_CONTROLS;
    opts[n++] = OPT_CALIBRATE;
    opts[n++] = OPT_START;

    /* THE SCREEN FOLLOWS ITS OWN SCREEN LINE, live: whoever has the console on its side and
     * picks HORIZONTAL reads the rest of this menu the right way up, and sees at once what
     * the line does. */
    /* FREE-RUNNING, NOT 50 Hz. Seven lines of the SDK font are ~1000 strokes: MEASURED in
     * the UVM2 emulator at 34141 bus cycles a frame against the 30000 of a 50 Hz period, and
     * held to 50 Hz the beam drew a different part of the list each frame (the title cut to
     * "ALPH", PLAY missing). Free, the frame takes what it takes: ~23 ms, steady. */
    uvm2_set_refresh(0u);

    int sel = 0, dirty = 0, save_failed = 0;
    struct nav nv = { 0 };
    /* EDGES START AS "PRESSED": a button already down when the screen appears -- button 4
     * from the launch -- has to be released before it counts. */
    uint8_t before = 0xFF;

    for (;;) {
        uvm2_draw_rotate((int)uvm2_setting_rotate);
        uvm2_frame_begin();
        uvm2_draw_intensity(INK);
        text_centred(TOP + 40, uvm2_config_title_get());
        for (int i = 0; i < n; i++) {
            const int y = TOP - i * ROW;
            if (i == sel) uvm2_print_text(LEFT - 16, y, ">", TEXT, INK);
            uvm2_print_text(LEFT, y, label(opts[i]), TEXT, INK);
        }
        text_centred(TOP - n * ROW - 14, save_failed ? "COULD NOT SAVE" : "BUTTON 4 PLAYS");
        uvm2_frame_end();
        if (save_failed) save_failed--;

        const uint8_t b = (uint8_t)~uvm2_cached_buttons;      /* 1 = pressed */
        const uint8_t pressed = (uint8_t)(b & ~before);
        before = b;
        int step_x, step_y;
        if (!nav_read(&nv, &step_x, &step_y)) continue;
        if (step_y) sel = (sel + n + step_y) % n;

        const int opt = opts[sel];
        int go = (pressed & 0x08) != 0;
        if (step_x && change(opt, step_x)) dirty = 1;
        if (pressed & 0x01) {
            if (opt == OPT_PLAY) go = 1;
            else if (opt == OPT_CONTROLS) {
                if (controls_menu(&nv)) dirty = 1;
                before = 0xFF;
            }
            else if (opt == OPT_CALIBRATE) {
                uvm2_config_wizard();             /* saves on its own */
                uvm2_set_refresh(0u);
                before = 0xFF;
            } else if (change(opt, +1)) dirty = 1;
        }
        if (go) break;
        /* Saved as it changes, as dkong does: a console switched off on this screen keeps
         * what it shows. A failure is SAID, not swallowed -- on our own cartridge a save that
         * did nothing went unnoticed until it was read back over SWD. */
        if (dirty) { dirty = 0; if (!uvm2_config_save()) save_failed = 150; }
    }

    uvm2_draw_rotate((int)uvm2_setting_rotate);
    uvm2_set_refresh((unsigned)uvm2_setting_hz);
    return 1;
}

void uvm2_options_boot(void)
{
    if (!(uvm2_config_game_settings() & UVM2_SETTING_OPTIONS)) return;
    uvm2_intro();               /* the Vectrex Studio card, every launch (uvm2_intro.c) */
    if (!uvm2_setting_menu) {
        /* START IN GAME: only a HELD button 4 opens it. Each wait is a real frame (one
         * command), or the count would run out in microseconds: an empty frame returns at
         * once and core 1 never refreshes the buttons (dkong menu.c). */
        int held = 0;
        while (held < HOLD_FRAMES && ((uint8_t)~uvm2_cached_buttons & 0x08)) {
            uvm2_frame_begin();
            uvm2_draw_intensity(INK);
            text_centred(0, "OPTIONS");
            uvm2_frame_end();
            held++;
        }
        if (held < HOLD_FRAMES) {
            uvm2_set_refresh((unsigned)uvm2_setting_hz);
            return;
        }
    }
    uvm2_options_menu();
}
