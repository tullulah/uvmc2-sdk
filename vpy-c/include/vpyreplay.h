/*
 * vpyreplay.h — record the player's input, play it back frame for frame.
 *
 * Everything in libvpy that moves is deterministic: the same input and the same
 * seed give the same frame, bit for bit — physics, effects, ropes, camera. So a
 * replay is only the input, three bytes a frame, and a seed: an attract mode,
 * a ghost to race, a bug report that can be watched again.
 *
 *     vpyreplay_record(buf, 3000, seed);         // or vpyreplay_play(buf, n, seed)
 *     ... each frame, BEFORE the game reads its input:
 *     vpyreplay_input(&buttons, &jx, &jy);       // records them, or replaces them
 *
 * The seed is the game's to use: seed every generator from vpyreplay_seed()
 * when a recording or a playback starts. A recording that outgrows its buffer
 * stops recording and counts what it lost — the replay would not be the game.
 */
#ifndef VPYREPLAY_H
#define VPYREPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { uint8_t buttons; int8_t jx, jy; } vpyreplay_frame;

enum { VPYREPLAY_OFF = 0, VPYREPLAY_RECORDING = 1, VPYREPLAY_PLAYING = 2 };

void     vpyreplay_record(vpyreplay_frame *buf, int capacity, uint32_t seed);
void     vpyreplay_play(const vpyreplay_frame *buf, int frames, uint32_t seed);
void     vpyreplay_stop(void);
int      vpyreplay_mode(void);
uint32_t vpyreplay_seed(void);
/* once a frame: recording, it stores these; playing, it replaces them. Off, it
 * leaves them alone. */
void     vpyreplay_input(uint8_t *buttons, int8_t *jx, int8_t *jy);
int      vpyreplay_frames(void);     /* recorded, or played, so far */
int      vpyreplay_done(void);       /* a playback that has run out */
uint32_t vpyreplay_lost(void);       /* frames a full recording could not keep */

#ifdef __cplusplus
}
#endif

#endif /* VPYREPLAY_H */
