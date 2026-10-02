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
static vpyimpact_sink   s_sink;             /* the PCM output (the UVMC2's jack), or none */
static int              s_rate = 32000, s_psg_too = 1;

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

/* ── where it happened ──────────────────────────────────────────────────────
 * The listener (usually the camera) and how sound falls off with distance. With
 * no listener set (far = 0) nothing is attenuated and everything is centred. */
static int32_t s_lx, s_ly, s_lz, s_right_x = 16384, s_right_z = 0, s_near, s_far;

void vpyimpact_set_listener(int32_t x, int32_t y, int32_t z, int32_t right_x, int32_t right_z,
                            int32_t near_dist, int32_t far_dist)
{
    s_lx = x; s_ly = y; s_lz = z;
    /* the right-hand direction, made unit (Q14), so pan is the sine of the bearing */
    int64_t l2 = (int64_t)right_x * right_x + (int64_t)right_z * right_z, r = 0, bit = (int64_t)1 << 62;
    while (bit > l2) bit >>= 2;
    while (bit) { if (l2 >= r + bit) { l2 -= r + bit; r = (r >> 1) + bit; } else r >>= 1; bit >>= 2; }
    if (r) { s_right_x = (int32_t)((int64_t)right_x * 16384 / r); s_right_z = (int32_t)((int64_t)right_z * 16384 / r); }
    s_near = near_dist < 0 ? 0 : near_dist;
    s_far = far_dist > s_near ? far_dist : 0;
}

static int64_t isqrt64(int64_t n)
{
    if (n <= 0) return 0;
    int64_t r = 0, bit = (int64_t)1 << 62;
    while (bit > n) bit >>= 2;
    while (bit) { if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; } else r >>= 1; bit >>= 2; }
    return r;
}

/* how much of a sound at (x,y,z) reaches the listener, Q8 (256 = all), and where it
 * sits, -127 (hard left) .. +127 (hard right) */
static int place(int32_t x, int32_t y, int32_t z, int *pan)
{
    *pan = 0;
    if (!s_far) return 256;
    const int64_t dx = (int64_t)x - s_lx, dy = (int64_t)y - s_ly, dz = (int64_t)z - s_lz;
    const int64_t d = isqrt64(dx * dx + dy * dy + dz * dz);
    if (d > 0) {
        int64_t p = (dx * s_right_x + dz * s_right_z) / d * 127 / 16384;
        *pan = (int)(p > 127 ? 127 : (p < -127 ? -127 : p));
    }
    if (d <= s_near) return 256;
    if (d >= s_far) return 0;
    return (int)(256 - (d - s_near) * 256 / (s_far - s_near));
}

static void pcm_start(int material, int v0, int frames, int pan);

int vpyimpact_hit(int32_t impulse, int material)
{
    return vpyimpact_hit_at(impulse, material, s_lx, s_ly, s_lz);
}

int vpyimpact_hit_at(int32_t impulse, int material, int32_t x, int32_t y, int32_t z)
{
    if (material < VPYI_SOFT || material > VPYI_METAL) return 0;
    if (impulse < s_quiet) { s_stats.quiet++; return 0; }
    int pan;
    const int near = place(x, y, z, &pan);
    if (near == 0) { s_stats.far++; return 0; }
    /* the distance takes volume off the impulse's: a crash across the room is
     * quieter than the same crash at your feet, but not silent until `far` */
    int v0 = (volume_of(impulse) * near + 128) >> 8;
    if (v0 < 1) { s_stats.far++; return 0; }
    s_stats.last_pan = pan;
    const voice_t *pv = &VOICE[material];
    int pframes = pv->frames * v0 / 15;
    if (pframes < 2) pframes = 2;
    if (s_sink) pcm_start(material, v0, pframes, pan);     /* the jack takes up to four at once */
    if (s_sink && !s_psg_too) { s_stats.played++; s_stats.last_impulse = impulse; s_stats.last_volume = v0; return 1; }
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
    int32_t best = -1, bx = 0, by = 0, bz = 0; int mat = VPYI_NONE;
    for (int i = 0; i < vpyp_contact_count(); i++) {
        const vpyp_contact *c = vpyp_contact_get(i);
        const int ma = c->a >= 0 ? material_of[c->a] : floor_material;
        const int mb = c->b >= 0 ? material_of[c->b] : floor_material;
        /* the one that rings more decides; a silent body only sounds what it hits */
        int m = ma == VPYI_NONE ? mb : (mb == VPYI_NONE ? ma : (ma > mb ? ma : mb));
        if (m == VPYI_NONE) continue;
        if (c->impulse > best) { best = c->impulse; mat = m; bx = c->x; by = c->y; bz = c->z; }
    }
    if (best < 0) return 0;
    return vpyimpact_hit_at(best, mat, bx, by, bz);
}

/* ── THE PCM PATH: stereo, several at once, and sources that move ─────────────
 *
 * The PSG is one mono channel for every effect. A cartridge with its own DAC (the
 * UVMC2's jack, 16-bit stereo) can do better, and this renders the SAME voices as
 * PCM: the tone a square wave at the PSG period's frequency, the noise a 17-bit
 * LFSR stepped at the PSG noise rate, the volume through the AY's logarithmic
 * levels — so the jack and the speaker sound like the same object. Up to
 * VPYI_PCM_VOICES hits at once, each panned where it happened.
 *
 * LOOPS are continuous sources — an engine, a hum — set every frame with where they
 * are and how they move. Their pitch follows the Doppler shift against the listener:
 * f' = f · c / (c + v_r), v_r the speed AWAY from the listener and c the speed of
 * sound in world units (343 000 mm/s by default; a game can exaggerate it). */
#define PSG_HZ  1500000                     /* the Vectrex's PSG clock */
/* the AY-3-8910's 16 volume levels, logarithmic, as fractions of full scale (Q16) —
 * the chip's own DAC curve, so a level here is as loud as on the speaker */
static const uint16_t AY_LEVEL[16] = { 0, 836, 1212, 1773, 2619, 3875, 5397, 8058,
                                       10397, 15161, 22074, 31350, 40620, 49230, 58290, 65535 };
#define VOICE_AMP 7000                      /* full scale per voice: 4 voices + 4 loops stay under 32767 mostly; the mix clamps */

typedef struct {
    uint8_t  on, material, noise_on;
    int      vol_q8, frames, frame, decay, per0, per1, noise_frames;
    uint32_t phase, inc, noise_acc, noise_inc, lfsr;
    int      gl, gr;                        /* pan gains, Q8 */
    int      samples_left_in_frame;
} pcm_voice_t;
static pcm_voice_t s_pv[VPYI_PCM_VOICES];

typedef struct { uint8_t on; int vol, gl, gr; uint32_t phase, inc; } pcm_loop_t;
static pcm_loop_t s_loop[VPYI_PCM_LOOPS];
static int32_t s_sound_speed = 343000;

void vpyimpact_set_pcm(vpyimpact_sink sink, int rate, int psg_too)
{
    s_sink = sink;
    s_rate = rate > 0 ? rate : 32000;
    s_psg_too = psg_too ? 1 : 0;
    for (int i = 0; i < VPYI_PCM_VOICES; i++) s_pv[i].on = 0;
    for (int i = 0; i < VPYI_PCM_LOOPS; i++) s_loop[i].on = 0;
}
void vpyimpact_set_sound_speed(int32_t units_per_s) { if (units_per_s > 0) s_sound_speed = units_per_s; }

static uint32_t inc_of_hz(uint32_t hz) { return (uint32_t)(((uint64_t)hz << 32) / (uint32_t)s_rate); }
static uint32_t inc_of_period(int per) { return per > 0 ? inc_of_hz((uint32_t)(PSG_HZ / (16 * per))) : 0; }
static void gains(int pan, int *gl, int *gr)
{
    /* linear pan, the centre at full in both: a centred hit is as loud as on the speaker */
    *gl = pan > 0 ? 256 - pan * 2 : 256;
    *gr = pan < 0 ? 256 + pan * 2 : 256;
    if (*gl < 0) *gl = 0;
    if (*gr < 0) *gr = 0;
}

static void pcm_start(int material, int v0, int frames, int pan)
{
    /* a free voice, or else the quietest */
    int k = 0;
    for (int i = 0; i < VPYI_PCM_VOICES; i++) {
        if (!s_pv[i].on) { k = i; break; }
        if (s_pv[i].vol_q8 < s_pv[k].vol_q8) k = i;
    }
    if (s_pv[k].on) s_stats.pcm_stolen++;
    pcm_voice_t *v = &s_pv[k];
    const voice_t *vc = &VOICE[material];
    v->on = 1; v->material = (uint8_t)material;
    v->vol_q8 = v0 << 8; v->frames = frames; v->frame = 0; v->decay = vc->decay;
    v->per0 = vc->tone0; v->per1 = vc->tone1; v->noise_frames = vc->noise ? vc->noise_frames : 0;
    v->phase = 0; v->inc = inc_of_period(vc->tone0);
    v->noise_inc = vc->noise ? inc_of_hz((uint32_t)(PSG_HZ / (16 * vc->noise))) : 0;
    v->noise_acc = 0; v->lfsr = 1;
    gains(pan, &v->gl, &v->gr);
    v->samples_left_in_frame = s_rate / 50;
}

int vpyimpact_loop(int slot, int32_t x, int32_t y, int32_t z, int32_t vx, int32_t vy, int32_t vz,
                   int hz, int volume)
{
    if (slot < 0 || slot >= VPYI_PCM_LOOPS) return 0;
    pcm_loop_t *L = &s_loop[slot];
    if (volume <= 0 || hz <= 0) { L->on = 0; return 0; }
    int pan;
    const int near = place(x, y, z, &pan);
    /* DOPPLER: the speed away from the listener along the line between them */
    int64_t f = hz;
    if (s_far) {
        const int64_t dx = (int64_t)x - s_lx, dy = (int64_t)y - s_ly, dz = (int64_t)z - s_lz;
        const int64_t d = isqrt64(dx * dx + dy * dy + dz * dz);
        if (d > 0) {
            int64_t vr = (dx * vx + dy * vy + dz * vz) / d;
            if (vr < -s_sound_speed / 2) vr = -s_sound_speed / 2;   /* no sonic booms */
            f = (int64_t)hz * s_sound_speed / (s_sound_speed + vr);
        }
    }
    L->on = 1;
    L->vol = (volume > 15 ? 15 : volume) * near / 256;
    gains(pan, &L->gl, &L->gr);
    L->inc = inc_of_hz((uint32_t)(f > 0 ? f : 1));
    s_stats.last_loop_hz = (int32_t)f;
    return 1;
}

void vpyimpact_pcm(int n)
{
    if (!s_sink || n <= 0) return;
    static int16_t l[256], r[256];
    while (n > 0) {
        const int chunk = n > 256 ? 256 : n;
        for (int i = 0; i < chunk; i++) {
            int32_t al = 0, ar = 0;
            for (int k = 0; k < VPYI_PCM_VOICES; k++) {
                pcm_voice_t *v = &s_pv[k];
                if (!v->on) continue;
                const int noisy = v->frame < v->noise_frames;
                int bit = (v->phase >> 31) & 1;                       /* the square wave */
                if (noisy) {
                    v->noise_acc += v->noise_inc;
                    if (v->noise_acc < v->noise_inc)                  /* wrapped: step the LFSR */
                        v->lfsr = (v->lfsr >> 1) ^ ((uint32_t)(-(int32_t)(v->lfsr & 1u)) & 0x12000u);
                    bit &= v->lfsr & 1;                               /* the PSG ANDs tone and noise */
                }
                v->phase += v->inc;
                const int lvl = v->vol_q8 >> 8;
                const int32_t a = (bit ? 1 : -1) * (int32_t)(((int64_t)AY_LEVEL[lvl] * VOICE_AMP) >> 16);
                al += a * v->gl >> 8; ar += a * v->gr >> 8;
                if (--v->samples_left_in_frame <= 0) {                /* the envelope moves once a frame, as on the PSG */
                    v->samples_left_in_frame = s_rate / 50;
                    if (++v->frame >= v->frames || (v->vol_q8 = (v->vol_q8 * v->decay) >> 8) < 256) { v->on = 0; continue; }
                    v->inc = inc_of_period(v->per0 + (v->per1 - v->per0) * v->frame / (v->frames > 1 ? v->frames - 1 : 1));
                }
            }
            for (int k = 0; k < VPYI_PCM_LOOPS; k++) {
                pcm_loop_t *L = &s_loop[k];
                if (!L->on) continue;
                const int32_t a = ((L->phase >> 31) & 1 ? 1 : -1) * (int32_t)(((int64_t)AY_LEVEL[L->vol] * VOICE_AMP) >> 16);
                L->phase += L->inc;
                al += a * L->gl >> 8; ar += a * L->gr >> 8;
            }
            if (al > 32767 || al < -32768 || ar > 32767 || ar < -32768) s_stats.pcm_clipped++;
            l[i] = (int16_t)(al > 32767 ? 32767 : (al < -32768 ? -32768 : al));
            r[i] = (int16_t)(ar > 32767 ? 32767 : (ar < -32768 ? -32768 : ar));
        }
        s_sink(l, r, chunk);
        s_stats.pcm_samples += (uint32_t)chunk;
        n -= chunk;
    }
}
