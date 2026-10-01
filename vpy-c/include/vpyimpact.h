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

/* Impulses below `quiet` are silent; `loud` and above are full volume.
 * Defaults 1500 and 20000 (mass × units/s, as vpyphys reports). */
void vpyimpact_set_range(int32_t quiet, int32_t loud);

/* One hit. Returns 1 if it sounds, 0 if too quiet or quieter than what plays. */
int  vpyimpact_hit(int32_t impulse, int material);

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
