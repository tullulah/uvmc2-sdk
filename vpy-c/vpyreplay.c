/*
 * vpyreplay.c — see vpyreplay.h.
 */
#include "vpyreplay.h"

static vpyreplay_frame       *s_rec;
static const vpyreplay_frame *s_play;
static int      s_cap, s_n, s_pos, s_mode, s_done;
static uint32_t s_seed, s_lost;

void vpyreplay_record(vpyreplay_frame *buf, int capacity, uint32_t seed)
{
    s_rec = buf; s_play = 0; s_cap = capacity > 0 ? capacity : 0;
    s_n = 0; s_pos = 0; s_seed = seed; s_lost = 0; s_done = 0;
    s_mode = buf ? VPYREPLAY_RECORDING : VPYREPLAY_OFF;
}
void vpyreplay_play(const vpyreplay_frame *buf, int frames, uint32_t seed)
{
    s_play = buf; s_rec = 0; s_n = frames > 0 ? frames : 0;
    s_pos = 0; s_seed = seed; s_done = 0;
    s_mode = buf ? VPYREPLAY_PLAYING : VPYREPLAY_OFF;
}
void     vpyreplay_stop(void)  { s_mode = VPYREPLAY_OFF; }
int      vpyreplay_mode(void)  { return s_mode; }
uint32_t vpyreplay_seed(void)  { return s_seed; }
int      vpyreplay_frames(void){ return s_mode == VPYREPLAY_PLAYING ? s_pos : s_n; }
int      vpyreplay_done(void)  { return s_done; }
uint32_t vpyreplay_lost(void)  { return s_lost; }

void vpyreplay_input(uint8_t *buttons, int8_t *jx, int8_t *jy)
{
    if (s_mode == VPYREPLAY_RECORDING) {
        if (s_n < s_cap) {
            s_rec[s_n].buttons = *buttons; s_rec[s_n].jx = *jx; s_rec[s_n].jy = *jy;
            s_n++;
        } else {
            s_lost++;                    /* the replay would not be the game: say so */
        }
    } else if (s_mode == VPYREPLAY_PLAYING) {
        if (s_pos < s_n) {
            *buttons = s_play[s_pos].buttons; *jx = s_play[s_pos].jx; *jy = s_play[s_pos].jy;
            s_pos++;
        } else {
            *buttons = 0; *jx = 0; *jy = 0;   /* past the end: nobody is touching anything */
            s_done = 1;
        }
    }
}
