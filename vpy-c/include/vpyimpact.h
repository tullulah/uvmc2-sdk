/*
 * vpyimpact.h — the sound of things hitting each other, made from the hit.
 *
 * NO SAMPLES. Each impact is SYNTHESISED on the PSG at the moment it happens: a
 * short envelope of noise and a tone whose pitch falls, as an effect stream
 * played through libvpy's own SFX player (vpy_play_sfx, channel C) — so it
 * shares the PSG with music exactly as any other effect does, and it costs a
 * few hundred bytes of buffer and no ROM.
 *
 * HOW HARD IT SOUNDS IS HOW HARD IT HIT. The volume comes from the contact's
 * impulse (vpyphys reports it, mass × units/s): below `quiet` nothing sounds —
 * a body resting on another reports a little every step and must stay silent —
 * and from `loud` up it is full volume. A heavy crate falling far is louder than
 * a light one falling a little, because that is what the impulse says.
 *
 * WHAT IT SOUNDS LIKE IS WHAT IT IS MADE OF. A material per body:
 *   VPYI_WOOD   a dull knock: a burst of noise and a low tone that drops
 *   VPYI_SOFT   a short low bump, no noise: a ball, a cushion
 *   VPYI_METAL  a brighter tone that rings for longer
 * When two things meet, the one that rings more (metal > wood > soft) decides.
 *
 * ONE CHANNEL, SO HITS TAKE TURNS. A new hit replaces the one playing only if
 * it is at least as loud as what that one still has left; otherwise it is
 * skipped and COUNTED (vpyimpact_stats()->skipped). A pile settling does not
 * machine-gun the channel, and the loudest crash of a step is the one heard.
 *
 * WHERE IT HAPPENED. With a listener set (vpyimpact_set_listener, usually the
 * camera), a hit is quieter the further away it is — full up to `near`, silent
 * from `far` — on every cartridge, speaker included.
 *
 * STEREO, ON A CARTRIDGE WITH A DAC. Give it a PCM sink (the UVMC2's jack:
 * uvm2_jack_write_lr) and the same voices are also rendered as 16-bit stereo,
 * panned to the side they happened on, up to VPYI_PCM_VOICES at once; and LOOPS —
 * continuous sources like an engine — whose pitch follows the Doppler shift as
 * they come and go. The game calls vpyimpact_pcm(n) with how many samples the
 * DAC wants (uvm2_jack_space()).
 *
 * Integer only, deterministic. Call vpyimpact_step() once per frame (it ages
 * what is playing; vpy_run already advances the SFX player itself).
 */
#ifndef VPYIMPACT_H
#define VPYIMPACT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { VPYI_SOFT = 0, VPYI_WOOD = 1, VPYI_METAL = 2, VPYI_NONE = 255 };

#ifndef VPYI_PCM_VOICES
#define VPYI_PCM_VOICES 4      /* hits sounding at once on the PCM path */
#endif
#ifndef VPYI_PCM_LOOPS
#define VPYI_PCM_LOOPS  4      /* continuous sources */
#endif

/* Impulses below `quiet` are silent; `loud` and above are full volume.
 * Defaults 1500 and 20000 (mass × units/s, as vpyphys reports). */
void vpyimpact_set_range(int32_t quiet, int32_t loud);

/* One hit. Returns 1 if it sounds, 0 if too quiet or quieter than what plays. */
int  vpyimpact_hit(int32_t impulse, int material);
/* The same, at a place: quieter with distance from the listener, and panned. */
int  vpyimpact_hit_at(int32_t impulse, int material, int32_t x, int32_t y, int32_t z);

/* The listener: where it is, its right-hand direction in x/z (any length), and the
 * distance falloff — full volume up to `near`, nothing from `far`. far = 0: off. */
void vpyimpact_set_listener(int32_t x, int32_t y, int32_t z, int32_t right_x, int32_t right_z,
                            int32_t near_dist, int32_t far_dist);

/* THE PCM PATH. `sink` gets left and right 16-bit samples at `rate`; psg_too = 1
 * keeps the speaker sounding as well, 0 sends hits to the sink only. NULL turns
 * it off. vpyimpact_pcm(n) renders n samples into the sink: call it every frame
 * with what the DAC can take. */
typedef void (*vpyimpact_sink)(const int16_t *l, const int16_t *r, int n);
void vpyimpact_set_pcm(vpyimpact_sink sink, int rate, int psg_too);
void vpyimpact_pcm(int n);
/* A continuous source in `slot` (0..VPYI_PCM_LOOPS-1): where it is and how it moves
 * (units/s), its pitch in Hz and volume 0..15 (0 stops it). Set it every frame; the
 * pitch heard is Doppler-shifted. PCM path only. Returns 1 if it sounds. */
int  vpyimpact_loop(int slot, int32_t x, int32_t y, int32_t z, int32_t vx, int32_t vy, int32_t vz,
                    int hz, int volume);
/* the speed of sound for Doppler, world units/s; default 343000 (mm) */
void vpyimpact_set_sound_speed(int32_t units_per_s);

/* The loudest contact of the last vpyp_step(), sounded with the materials in
 * `material_of[body id]` (VPYP_MAX_BODIES of them; VPYI_NONE = silent body) and
 * `floor_material` for the floor. Returns 1 if something sounded. */
int  vpyimpact_contacts(const uint8_t *material_of, int floor_material);

/* Once per frame: ages the sound playing, so the next hit knows how much of it
 * is left. */
void vpyimpact_step(void);

typedef struct {
    uint32_t played;            /* hits that sounded */
    uint32_t skipped;           /* hits quieter than what was playing */
    uint32_t quiet;             /* hits under the range: silent by design */
    int32_t  last_impulse;      /* the last hit sounded, and its volume 1..15 */
    int      last_volume;
    uint32_t far;               /* hits too far from the listener to be heard */
    int      last_pan;          /* -127 left .. +127 right, of the last hit */
    uint32_t pcm_samples;       /* rendered into the sink */
    uint32_t pcm_stolen;        /* PCM voices cut short for a new hit */
    uint32_t pcm_clipped;       /* samples the mix had to clamp */
    int32_t  last_loop_hz;      /* the Doppler-shifted pitch of the last loop set */
} vpyimpact_stats_t;
const vpyimpact_stats_t *vpyimpact_stats(void);
void vpyimpact_reset(void);

/* The effect stream the last hit produced (for the host check): the SFX
 * format vpy_play_sfx reads — a 4-byte header, then [delay, n, (reg, val) × n]
 * per frame, ending in [delay, 0]. */
const uint8_t *vpyimpact_last_stream(int *len);

#ifdef __cplusplus
}
#endif

#endif /* VPYIMPACT_H */
