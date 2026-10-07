/*
 * vpy.h — the VPy builtin runtime, in C.
 *
 * Reimplements the procedural VPy builtins (draw / input / math / text) ON TOP
 * of the minimal PiTrex SDK contract (vectrexInterface.h: v_directDraw32,
 * v_WaitRecal, v_readButtons, v_readJoystick1Analog, v_millis, v_setSoundAY).
 * Because it uses ONLY that contract, the same C code runs identically on real
 * PiTrex hardware and in the Vectrex Studio emulator panel (WASM + host shim).
 *
 * Coordinates use the same logical VPy space as the .vpy language (roughly
 * -127..127, +Y up); vpy scales by x127 to PiTrex units, exactly like the VPy
 * pitrex codegen. Brightness is 0..127.
 */
#ifndef VPY_H
#define VPY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- lifecycle ---- */
void vpy_init(void);                 /* vectrexinit + v_init + v_setRefresh(50) */
void vpy_frame_begin(void);          /* vpy_wait_recal + refresh input snapshot */
void vpy_run(void (*setup)(void), void (*loop)(void)); /* init; setup(); loop forever */

/* ---- the stroke buffer ---------------------------------------------------
 * Every drawing call in libvpy accumulates strokes; NOTHING reaches
 * v_directDraw32 until the frame is flushed. This is what lets a frame be
 * costed and shed before it turns into bus commands (one stroke is ~13
 * commands on the UVM2, and they cannot be taken back once emitted).
 *
 * THE ONE RULE: something has to flush, once per frame. Use vpy_run(), or
 * vpy_frame_begin(), or vpy_wait_recal() as your frame boundary. A game that
 * calls v_WaitRecal() directly will accumulate and never draw — that shows up
 * as vpy_draw_stats()->frames stuck at 0 while ->dropped climbs, which is the
 * signal to look for if the screen is black.
 *
 * Priority decides what survives a full buffer: a stroke that cannot fit
 * evicts the lowest-priority stroke below it, or is dropped if everything
 * present is at least as important. Draw the HUD at VPY_PRI_KEEP and distant
 * detail at VPY_PRI_LOW and the picture degrades in the right order.
 *
 * THIS IS NOT THE SDK DROPPING GEOMETRY. The SDK's drawing path has no knobs — what it is
 * asked for is what it draws (the starter kit's invariant 4). Shedding happens HERE, in the
 * game library, before anything is asked of the SDK, only when the buffer is full, and never
 * silently: every stroke shed or dropped is counted in vpy_draw_stats(). If `shed` or
 * `dropped` is non-zero, the frame on screen is not the frame the game built. */
#define VPY_PRI_LOW    0     /* shed first: decals, distant scenery */
#define VPY_PRI_NORMAL 128   /* the default */
#define VPY_PRI_KEEP   255   /* never shed: the player, the HUD */

typedef struct {
    uint32_t frames;    /* flushes so far. STUCK AT 0 = nothing is flushing */
    uint32_t strokes;   /* strokes in the frame last flushed */
    uint32_t peak;      /* the most ever flushed in one frame */
    uint32_t shed;      /* strokes evicted to make room for more important ones */
    uint32_t dropped;   /* strokes lost: the buffer was full of equal-or-better */
    uint32_t clamped;   /* coordinates pulled back to the legal PiTrex range */
} vpy_draw_stats_t;

/* A stroke straight into the buffer, in PiTrex deflection units (VPy unit x
 * 127). For code that computes its own geometry and would lose it to the
 * +-127 logical grid — a 3D projection, a camera pan. */
void vpy_draw_line_dev(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int br);

void vpy_wait_recal(void);           /* flush, then v_WaitRecal: the frame boundary */
void vpy_flush(void);                /* send the accumulated frame, empty the buffer */
void vpy_set_priority(int p);        /* shed priority of the strokes drawn next */
int  vpy_get_priority(void);
int  vpy_pending_strokes(void);      /* accumulated but not yet flushed */
/* Read back stroke i of the frame being built (0 if there is no such stroke).
 * For telemetry and capture: sees the frame without disturbing the beam. */
int  vpy_peek_stroke(int i, int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1, int *br);
const vpy_draw_stats_t *vpy_draw_stats(void);

/* ---- drawing (VPy space, brightness 0..127) ---- */
void vpy_set_intensity(int b);
void vpy_move(int x, int y);
void vpy_draw_line(int x0, int y0, int x1, int y1, int b);
void vpy_draw_circle(int cx, int cy, int r, int b);
void vpy_draw_rect(int x, int y, int w, int h, int b);           /* x,y = lower-left */
void vpy_draw_filled_rect(int x, int y, int w, int h, int b);
void vpy_draw_polygon(const int *xy, int n, int b);              /* n vertices, xy[2n] */
void vpy_draw_ellipse(int cx, int cy, int rx, int ry, int b);    /* 16-segment ellipse */

/* ---- compiled vector sprites (.vec) --------------------------------------
 * `data` is a vector path stream compiled from a .vec by
 *   `vpy_cli compile-asset <file>.vec --format c --out <name>.h`
 * (same path/segment geometry compiler as the ARM/PiTrex backend). The sprite
 * is drawn at VPy position (x,y) using the asset's own per-path intensity.
 * Bezier segments are tessellated into lines (the host contract only exposes
 * v_directDraw32). draw_vector_ex adds horizontal mirror + an optional
 * intensity override (pass intensity<=0 to keep each path's own intensity). */
void vpy_draw_vector(const unsigned char *data, int x, int y);
void vpy_draw_vector_ex(const unsigned char *data, int x, int y, int mirror, int intensity);

/* Draw a compiled .vec sprite at (x,y) rotated by `angle` about its origin.
 * `angle` uses VPy's angle unit, the one vpy_sin/vpy_cos and DRAW_VECTOR_3D take:
 * 0..127 is one full turn, counter-clockwise (+Y up), so 32 = 90 degrees,
 * 64 = 180 and 96 = 270; any int is taken modulo 128. The origin is the point of
 * the sprite that vpy_draw_vector places at (x,y): the centre of the .vec's bounding
 * box, as `compile-asset` writes it. Every point is rotated (sin/cos from the Q14
 * table, exact at the quarter turns) and placed with sub-unit precision; each path
 * keeps its own intensity. Same data as vpy_draw_vector (`{NAME}_vec`). */
void vpy_draw_vector_rot(const unsigned char *data, int x, int y, int angle);
/* vpy_draw_vector_rot with a mirror and an intensity override.
 * `mirror` flips the sprite in sprite space BEFORE rotating it: bit 0 left-right,
 * bit 1 upside down (0 none, 1 X, 2 Y, 3 both, as VPy's DRAW_VECTOR_EX);
 * `intensity` > 0 replaces every path's intensity (<= 0 keeps the asset's own),
 * as in vpy_draw_vector_ex. */
void vpy_draw_vector_rot_ex(const unsigned char *data, int x, int y, int angle,
                            int mirror, int intensity);

/* ---- compiled animations (.vanim) ----------------------------------------
 * `anim` is a position-independent anim descriptor compiled from a .vanim by
 *   `vpy_cli compile-asset <file>.vanim --format c --out <name>.h`
 * and `sprites` its companion frame-sprite pointer table (`{NAME}_anim_sprites`),
 * each entry pointing at a compiled frame `.vec` array. vpy_draw_anim ticks the
 * animation (advancing frames by each frame's duration, looping or freezing per
 * the asset) and draws the current frame at VPy position (x,y). Bit-exact with
 * the VPy pitrex `pitrex_draw_anim`: a SINGLE global cursor is shared by all
 * calls (one animation ticked per frame). */
void vpy_draw_anim(const unsigned char *anim, const unsigned char *const *sprites,
                   int x, int y, int mirror);

/* ---- text ---- */
void vpy_set_text_size(int s);       /* glyph scale (VPy units per grid unit ~ s) */
void vpy_print_text(int x, int y, const char *s);
/* The vector font's glyph for a character: [pattern, dy, dx] signed-byte triples,
 * pattern != 0 a stroke and 0 a move, deltas in units of 1/15 of the font's grid,
 * ending before the next positive pattern byte (see vpy.c). For drawing the same
 * letters elsewhere — vpy3d_text puts them on a plane in the world. */
const signed char *vpy_font_glyph(int c);
/* The next character of a string as a font code, advancing past it: UTF-8 Latin-1
 * letters as their code (accents folded away, the openers \u00A1 \u00BF kept), so
 * Spanish text can be written as it is. vpy_print_text and vpy3d_text use it. */
int vpy_font_next(const char **s);
void vpy_print_number(int x, int y, long n);

/* ---- input ----
 * All read the SDK input globals (currentJoy1X/Y, currentButtonState) directly;
 * they are refreshed each frame by vpy_frame_begin() (vpy_run path) or by the
 * VPy game loop. vpy_update_buttons() forces an immediate re-read mid-frame. */
int vpy_j1_x(void);                  /* -127..127 */
int vpy_j1_y(void);                  /* -127..127 (+ = up) */
int vpy_j1_button(int n);            /* n = 1..4 -> 0/1 (bit n-1 of button state) */
void vpy_update_buttons(void);       /* v_readButtons + v_readJoystick1Analog */

/* ---- math ---- */
int vpy_abs(int v);
int vpy_min(int a, int b);
int vpy_max(int a, int b);
int vpy_clamp(int v, int lo, int hi);
int vpy_sin(int a);                  /* a: 0..127 = full circle; returns -127..127 */
int vpy_cos(int a);
/* Q14 trig: 4096 steps per full turn, returns -16384..16384 (16384 = 1.0).
 * A SEPARATE name, not a replacement: vpy_sin/vpy_cos keep their 128-step
 * meaning on every target, so no call changes what it returns depending on
 * where it was compiled. Use these wherever 128 steps would show as a
 * staircase — a rotating camera, a slow orbit, any 3D transform. */
/* The unit, named. Callers were writing 4096 and 16384 out by hand, which is how
 * an angle unit and the code that consumes it drift apart. #ifndef because
 * vpy_trig_q14.h defines the same two for the table generator. */
#ifndef VPY_Q14_TURN
#define VPY_Q14_TURN 4096    /* steps per full turn */
#endif
#ifndef VPY_Q14_ONE
#define VPY_Q14_ONE  16384   /* Q14 scale: this is 1.0 */
#endif
int vpy_sin_q14(int a);
int vpy_cos_q14(int a);
int vpy_sqrt(int v);
int vpy_atan2(int y, int x);         /* returns 0..127 (full circle) */
int vpy_rand(void);
int vpy_rand_range(int lo, int hi);
void vpy_seed(unsigned s);

/* ---- sound (basic tone on PSG channel A) ---- */
void vpy_beep(int on);
void vpy_tone(int period, int volume); /* period 12-bit, volume 0..15; 0 vol = off */

/* ---- compiled music / SFX playback ---------------------------------------
 * `data` is a PSG event stream compiled from a .vmus/.vsfx by
 *   `vpy_cli compile-asset <file> --format c --out <name>.h`
 * (same notes->PSG compiler as the ARM/PiTrex backend). A simple 50 Hz
 * sequencer walks the stream, writing PSG registers via v_setSoundAY each frame.
 * vpy_run() drives vpy_music_update()/vpy_sfx_update() automatically. */
void vpy_play_music(const unsigned char *data);  /* start/replace music playback */
void vpy_music_update(void);                      /* advance one frame (auto-called) */
void vpy_stop_music(void);                         /* mute channels + stop */
void vpy_play_sfx(const unsigned char *data);     /* trigger a one-shot SFX (channel C) */
void vpy_sfx_update(void);                         /* advance one frame (auto-called) */

/* ---- instruments (.vinstr) -----------------------------------------------
 * `instr` is the 16-byte instrument block compiled from a .vinstr by
 *   `vpy_cli compile-asset <file>.vinstr --format c --out <name>.h`  ({NAME}_instr)
 * An instrument is a timbre without a pitch: vpy_play_note gives it one. It sets
 * the tone period of PSG channel `channel` (0-2 = A-C) for MIDI note `note`
 * (clamped to 24-107), its volume, and enables that channel's tone. The note
 * lasts the instrument's duration_frames (0 = held until the next note on the
 * channel) and steps through its arpeggio. noise and pitch sweep are not played,
 * as in the VPy runtime. vpy_run() drives vpy_note_update() automatically. */
void vpy_play_note(const unsigned char *instr, int channel, int note);
void vpy_note_update(void);                        /* advance one frame (auto-called) */

/* ---- vector recordings (.vrec) -------------------------------------------
 * `rec` is the recording table compiled from a .vrec by
 *   `vpy_cli compile-asset <file>.vrec --format c --out <name>.h`  ({NAME}_vrec)
 * Draws frame `frame % frame_count` at VPy position (x,y), `scale` 128 = 100%,
 * each chain at its recorded intensity. Every segment of the frame is drawn,
 * so keep recordings small. The frame is usually vpy_sample_pos(fps) (the
 * soundtrack's clock) or a counter of your own. */
void vpy_draw_recording(const unsigned char *rec, int x, int y, int scale, int frame);
int  vpy_recording_frames(const unsigned char *rec);   /* frame_count (0 if empty) */

/* ---- digitised samples (.vsmp) -------------------------------------------
 * `smp` is the 4-bit sample compiled from a .vsmp by
 *   `vpy_cli compile-asset <file>.vsmp --format c --out <name>.h`  ({NAME}_smp)
 * vpy_play_sample starts it once on sample voice 0 (re-calling restarts it);
 * vpy_sample_pos(fps) is how far that voice has played, as a frame number at
 * `fps` — 0 when nothing is playing. Pass it to vpy_draw_recording and the
 * picture follows the audio instead of drifting from it.
 *
 * Where it sounds (the same as VPy's PLAY_SAMPLE / SAMPLE_POS):
 *   simulator         Web Audio, the cursor from the audio clock
 *   rp2350 cartridge  the BIOS sample player (PSG volume DAC); needs a BIOS with
 *                     table version >= 2, silent on an older one
 *   uvm2 (.um2)       silent, and vpy_sample_pos stays 0: a .um2 cannot hold a
 *                     sample in its image (UVM2_TARGET.md). Ports ship theirs in
 *                     an SD bundle and play them with v_playSample(index, …).
 *   no sample player  (the real PiTrex SDK) silent; vpy_sample_pos counts the
 *                     frames presented since its first call, converted to `fps`,
 *                     so a recording still plays at its own speed. */
void vpy_play_sample(const unsigned char *smp);
int  vpy_sample_pos(int fps);

/* ---- compiled levels (.vplay) --------------------------------------------
 * `level` is a level byte image compiled from a .vplay by
 *   `vpy_cli compile-asset <file>.vplay --format c --out <name>.h`
 * (same header/object layout as the ARM/PiTrex level backend). Objects
 * reference sprites by INDEX; `sprites` is the companion pointer table the
 * generated header emits alongside the image (`{NAME}_level_sprites`), each
 * entry pointing at a compiled `.vec` array. Both are passed to LOAD_LEVEL:
 *   LOAD_LEVEL(WORLD_level, WORLD_level_sprites);
 * SHOW_LEVEL() draws every visible object (BG + GP + FG) offset by the camera
 * and culled to the screen, calling vpy_draw_vector_ex per object — mirroring
 * pitrex_show_level. UPDATE_LEVEL() advances gameplay-object physics (velocity
 * + gravity) exactly like pitrex_update_level. Camera is in VPy units. */
void vpy_load_level(const unsigned char *level, const unsigned char *const *sprites);
void vpy_show_level(void);                          /* draw all layers with camera offset */
void vpy_update_level(void);                        /* advance GP object physics one frame */
void vpy_set_camera_x(int x);
void vpy_set_camera_y(int y);
int  vpy_get_camera_x(void);
int  vpy_get_camera_y(void);
/* Level scalar accessors (bridged: the VPy pitrex codegen calls these). */
int  vpy_get_scroll_limit_left(void);
int  vpy_get_scroll_limit_right(void);
int  vpy_get_scroll_limit_top(void);
int  vpy_get_scroll_limit_bottom(void);
int  vpy_get_level_floor_y(void);
int  vpy_level_collision_y(int px, int py, int hh);
int  vpy_level_collision_x(int px, int py, int hw, int hy);

/* Enemy runtime (bridged: the VPy pitrex codegen calls these). `img` is a
 * position-independent `_NAME_ENEMIES_C` image; `sprites[i]` a `_{SPRITE}_VEC`
 * image. Full AI (patrol/area/wander) + static & anim draw, all render-verified
 * bit-exact vs the inline pitrex enemy runtime. */
void vpy_spawn_enemies(const unsigned char *img, const unsigned char *const *sprites);
void vpy_update_enemies(void);
void vpy_draw_enemies(void);
void vpy_kill_enemy(int idx);
/* Enemy pool accessors (read/write the same s_enemies pool). */
int  vpy_get_enemy_active(int idx);
int  vpy_get_enemy_x(int idx);
int  vpy_get_enemy_y(int idx);
int  vpy_get_enemy_state(int idx);
void vpy_set_enemy_x(int idx, int x);
void vpy_set_enemy_y(int idx, int y);
void vpy_set_enemy_state(int idx, int st);
void vpy_set_enemy_dir(int idx, int dir);
void vpy_enemy_fire_event(int idx, int hash);
/* Runtime-hashing wrapper matching the VPy `ENEMY_FIRE_EVENT(idx, "name")`
 * builtin: computes the SAME FNV-1a u8 hash the codegen bakes in and dispatches
 * it through the enemy's state machine. */
void vpy_enemy_fire_event_str(int idx, const char *event);

/* Optional VPy-style uppercase aliases so migrated .vpy reads naturally. */
#ifdef VPY_SHORT_NAMES
#define SET_INTENSITY   vpy_set_intensity
#define MOVE            vpy_move
#define DRAW_LINE       vpy_draw_line
#define DRAW_CIRCLE     vpy_draw_circle
#define DRAW_RECT       vpy_draw_rect
#define DRAW_FILLED_RECT vpy_draw_filled_rect
#define DRAW_POLYGON    vpy_draw_polygon
#define DRAW_ELLIPSE    vpy_draw_ellipse
#define DRAW_VECTOR     vpy_draw_vector
#define DRAW_VECTOR_EX  vpy_draw_vector_ex
#define DRAW_VECTOR_ROT vpy_draw_vector_rot
#define DRAW_VECTOR_ROT_EX vpy_draw_vector_rot_ex
#define DRAW_ANIM       vpy_draw_anim
#define WAIT_RECAL      vpy_wait_recal
#define FLUSH           vpy_flush
#define SET_PRIORITY    vpy_set_priority
#define SIN_Q14         vpy_sin_q14
#define COS_Q14         vpy_cos_q14
#define SET_TEXT_SIZE   vpy_set_text_size
#define PRINT_TEXT      vpy_print_text
#define PRINT_NUMBER    vpy_print_number
#define J1_X            vpy_j1_x
#define J1_Y            vpy_j1_y
#define UPDATE_BUTTONS  vpy_update_buttons
#define PLAY_MUSIC      vpy_play_music
#define STOP_MUSIC      vpy_stop_music
#define PLAY_SFX        vpy_play_sfx
#define PLAY_NOTE       vpy_play_note
#define DRAW_RECORDING  vpy_draw_recording
#define PLAY_SAMPLE     vpy_play_sample
#define SAMPLE_POS      vpy_sample_pos
#define LOAD_LEVEL      vpy_load_level
#define SHOW_LEVEL      vpy_show_level
#define UPDATE_LEVEL    vpy_update_level
#define SET_CAMERA_X    vpy_set_camera_x
#define SET_CAMERA_Y    vpy_set_camera_y
#define GET_CAMERA_X    vpy_get_camera_x
#define GET_CAMERA_Y    vpy_get_camera_y
/* Level accessors */
#define GET_LEVEL_FLOOR_Y      vpy_get_level_floor_y
#define GET_SCROLL_LIMIT_LEFT   vpy_get_scroll_limit_left
#define GET_SCROLL_LIMIT_RIGHT  vpy_get_scroll_limit_right
#define GET_SCROLL_LIMIT_TOP    vpy_get_scroll_limit_top
#define GET_SCROLL_LIMIT_BOTTOM vpy_get_scroll_limit_bottom
#define LEVEL_COLLISION_X       vpy_level_collision_x
#define LEVEL_COLLISION_Y       vpy_level_collision_y
/* Enemies. SPAWN_ENEMIES(data, sprites) takes the compiled `_NAME_ENEMIES_C`
 * image + its `_NAME_ENEMY_SPRITES` table (VPy resolves those from the level
 * name; C passes the symbols explicitly). */
#define SPAWN_ENEMIES   vpy_spawn_enemies
#define UPDATE_ENEMIES  vpy_update_enemies
#define DRAW_ENEMIES    vpy_draw_enemies
#define KILL_ENEMY      vpy_kill_enemy
#define GET_ENEMY_ACTIVE vpy_get_enemy_active
#define GET_ENEMY_X     vpy_get_enemy_x
#define GET_ENEMY_Y     vpy_get_enemy_y
#define GET_ENEMY_STATE vpy_get_enemy_state
#define SET_ENEMY_X     vpy_set_enemy_x
#define SET_ENEMY_Y     vpy_set_enemy_y
#define SET_ENEMY_STATE vpy_set_enemy_state
#define SET_ENEMY_DIR   vpy_set_enemy_dir
/* ENEMY_FIRE_EVENT(idx, "name") — runtime-hashed exactly like the VPy builtin. */
#define ENEMY_FIRE_EVENT vpy_enemy_fire_event_str
/* Digital buttons: VPy exposes J1_BUTTON_1..4 (); C maps each to vpy_j1_button(n). */
#define J1_BUTTON_1()   vpy_j1_button(1)
#define J1_BUTTON_2()   vpy_j1_button(2)
#define J1_BUTTON_3()   vpy_j1_button(3)
#define J1_BUTTON_4()   vpy_j1_button(4)
/* Debug print is a no-op on the minimal C runtime (no host console). */
#define DEBUG_PRINT_LABELED(label, val) ((void)(val))
#endif

#ifdef __cplusplus
}
#endif

#endif /* VPY_H */
