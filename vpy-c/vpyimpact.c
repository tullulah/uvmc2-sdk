/*
 * vpyimpact.c — see vpyimpact.h.
 *
 * THE PSG. Channel C: tone period in registers 4 (low) and 5 (high nibble),
 * volume in 10 (0..15, bit 4 off: no hardware envelope), the noise period in 6,
 * and in the mixer (7) bit 2 off = tone C on, bit 5 off = noise C on. The SFX
 * player merges only those two mixer bits over the music's, so a hit never
 * silences the music's channels. On a Vectrex the PSG runs at 1.5 MHz, so a tone
 * period P sounds at 1.5e6 / (16 P) Hz: 780 is ~120 Hz, 140 is ~670 Hz.
 */
#include "vpyimpact.h"
#include "vpy.h"
#include "vpyphys.h"

/* A MATERIAL'S VOICE. Starting values from the PSG arithmetic above (a knock
 * near 120 Hz, a ring near 670), NOT yet tuned by ear on a console: a sound
 * design, not a measurement, so change them freely.
 *   tone0 → tone1   the tone's period at the first frame and the last (rising
 *                   period = falling pitch, which is what a knock does)
 *   noise           noise period (0 = no noise) and for how many frames
 *   frames          the longest it lasts, at full volume
 *   decay           the volume kept from one frame to the next, Q8 */
typedef struct { uint16_t tone0, tone1; uint8_t noise, noise_frames, frames, decay; } voice_t;
static const voice_t VOICE[3] = {
    /* SOFT  */ { 470,  700, 0,  0,  6, 170 },
    /* WOOD  */ { 780, 1100, 8,  3, 10, 190 },
    /* METAL */ { 140,  150, 2,  1, 24, 230 },
};

static int32_t s_quiet = 1500, s_loud = 20000;
static uint8_t s_buf[4 + 32 * 12 + 2];      /* header + up to 32 frames of 5 writes + end */
static int     s_len;
/* WHAT IS PLAYING: its volume at the start, its decay, and how many frames ago it
 * started — so how loud it still is can be worked out without asking the player. */
static int     s_v0, s_decay, s_age = 1000, s_frames;
static vpyimpact_stats_t s_stats;

void vpyimpact_set_range(int32_t quiet, int32_t loud)
{
    if (quiet < 0) quiet = 0;
    if (loud <= quiet) loud = quiet + 1;
    s_quiet = quiet; s_loud = loud;
}

void vpyimpact_reset(void)
{
    vpyimpact_stats_t z = { 0 };
    s_stats = z;
    s_age = 1000; s_v0 = 0;
}

const vpyimpact_stats_t *vpyimpact_stats(void) { return &s_stats; }
const uint8_t *vpyimpact_last_stream(int *len) { if (len) *len = s_len; return s_buf; }

/* volume (0..15) of a sound that started at v0 with this decay, `age` frames on */
static int volume_at(int v0, int decay, int age, int frames)
{
    if (age >= frames) return 0;
    int v = v0 << 8;                                 /* Q8 */
    for (int k = 0; k < age; k++) v = (v * decay) >> 8;
    return v >> 8;
}

void vpyimpact_step(void) { if (s_age < 1000) s_age++; }

/* the impulse onto 1..15. Square-root shaped: the ear hears loudness roughly as
 * the log of power, and a linear map left every middling knock nearly silent. */
static int volume_of(int32_t impulse)
{
    if (impulse >= s_loud) return 15;
    const int64_t f = ((int64_t)(impulse - s_quiet) << 16) / (s_loud - s_quiet);   /* Q16, 0..1 */
    int64_t r = 0, bit = (int64_t)1 << 30, n = f << 16;                             /* sqrt in Q16 */
    while (bit > n) bit >>= 2;
    while (bit) { if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; } else r >>= 1; bit >>= 2; }
    const int v = 4 + (int)((r * 11) >> 16);         /* 4..15: the quietest hit is still heard */
    return v > 15 ? 15 : v;
}

static void put(int *p, uint8_t reg, uint8_t val) { s_buf[(*p)++] = reg; s_buf[(*p)++] = val; }

int vpyimpact_hit(int32_t impulse, int material)
{
    if (material < VPYI_SOFT || material > VPYI_METAL) return 0;
    if (impulse < s_quiet) { s_stats.quiet++; return 0; }
    const int v0 = volume_of(impulse);
    if (v0 < volume_at(s_v0, s_decay, s_age, s_frames)) { s_stats.skipped++; return 0; }

    const voice_t *vc = &VOICE[material];
    /* A softer hit is shorter as well as quieter: the frames scale with the volume. */
    int frames = vc->frames * v0 / 15;
    if (frames < 2) frames = 2;
    int p = 0;
    s_buf[p++] = 'S'; s_buf[p++] = 'F'; s_buf[p++] = 'X'; s_buf[p++] = 0;   /* the header the player skips */
    int v = v0 << 8;
    for (int k = 0; k < frames; k++) {
        const int vol = v >> 8;
        if (vol <= 0) break;
        const int per = vc->tone0 + (vc->tone1 - vc->tone0) * k / (frames > 1 ? frames - 1 : 1);
        const int noisy = vc->noise && k < vc->noise_frames;
        s_buf[p++] = 0;                              /* delay: the next frame */
        const int nat = p; s_buf[p++] = 0;           /* how many writes: filled below */
        int n = 0;
        if (k == 0 || (noisy != (vc->noise && k - 1 < vc->noise_frames))) {
            /* the mixer: tone C always on, noise C while the noise lasts */
            put(&p, 7, (uint8_t)(0x3F & ~0x04 & ~(noisy ? 0x20 : 0x00))); n++;
            if (noisy) { put(&p, 6, vc->noise); n++; }
        }
        put(&p, 4, (uint8_t)(per & 0xFF)); n++;
        put(&p, 5, (uint8_t)((per >> 8) & 0x0F)); n++;
        put(&p, 10, (uint8_t)vol); n++;
        s_buf[nat] = (uint8_t)n;
        v = (v * vc->decay) >> 8;
    }
    s_buf[p++] = 0; s_buf[p++] = 0;                  /* end: the player mutes channel C */
    s_len = p;

    s_v0 = v0; s_decay = vc->decay; s_frames = frames; s_age = 0;
    s_stats.played++; s_stats.last_impulse = impulse; s_stats.last_volume = v0;
    vpy_play_sfx(s_buf);
    return 1;
}

int vpyimpact_contacts(const uint8_t *material_of, int floor_material)
{
    int32_t best = -1; int mat = VPYI_NONE;
    for (int i = 0; i < vpyp_contact_count(); i++) {
        const vpyp_contact *c = vpyp_contact_get(i);
        const int ma = c->a >= 0 ? material_of[c->a] : floor_material;
        const int mb = c->b >= 0 ? material_of[c->b] : floor_material;
        /* the one that rings more decides; a silent body only sounds what it hits */
        int m = ma == VPYI_NONE ? mb : (mb == VPYI_NONE ? ma : (ma > mb ? ma : mb));
        if (m == VPYI_NONE) continue;
        if (c->impulse > best) { best = c->impulse; mat = m; }
    }
    if (best < 0) return 0;
    return vpyimpact_hit(best, mat);
}
