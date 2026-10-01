/*
 * vpycam.c — see vpycam.h.
 */
#include "vpycam.h"
#include "vpy3d.h"

static int32_t  s_f[3];                       /* the focus */
static int32_t  s_dead[3] = { 0, 0, 0 };
static int      s_lead = 0, s_smooth = 1;
static int32_t  s_shake_amt, s_shake_left, s_shake_len;
static int32_t  s_off[3];                     /* this frame's shake */
static int      s_stop;
static uint32_t s_rng = 0x9E3779B9u;

static uint32_t rnd(void) { uint32_t x = s_rng; x ^= x << 13; x ^= x >> 17; x ^= x << 5; return s_rng = x; }

void vpycam_reset(int32_t fx, int32_t fy, int32_t fz)
{
    s_f[0] = fx; s_f[1] = fy; s_f[2] = fz;
    s_shake_amt = s_shake_left = s_shake_len = 0;
    s_off[0] = s_off[1] = s_off[2] = 0;
    s_stop = 0;
}
void vpycam_seed(uint32_t seed) { s_rng = seed ? seed : 0x9E3779B9u; }

void vpycam_follow_config(int32_t dx, int32_t dy, int32_t dz, int lead, int smooth)
{
    s_dead[0] = dx < 0 ? 0 : dx; s_dead[1] = dy < 0 ? 0 : dy; s_dead[2] = dz < 0 ? 0 : dz;
    s_lead = lead < 0 ? 0 : lead;
    s_smooth = smooth < 1 ? 1 : smooth;
}

void vpycam_follow(int32_t tx, int32_t ty, int32_t tz, int32_t vx, int32_t vy, int32_t vz)
{
    const int32_t want[3] = { tx + vx * s_lead, ty + vy * s_lead, tz + vz * s_lead };
    for (int k = 0; k < 3; k++) {
        /* inside the dead zone the focus stays; outside, it goes for the zone's edge */
        int32_t goal = s_f[k];
        if (want[k] > s_f[k] + s_dead[k]) goal = want[k] - s_dead[k];
        if (want[k] < s_f[k] - s_dead[k]) goal = want[k] + s_dead[k];
        const int32_t gap = goal - s_f[k];
        int32_t step = gap / s_smooth;
        if (step == 0 && gap != 0) step = gap;        /* the last few units: snap, do not creep */
        s_f[k] += step;
    }
}
void vpycam_focus(int32_t *x, int32_t *y, int32_t *z) { *x = s_f[0]; *y = s_f[1]; *z = s_f[2]; }

void vpycam_shake(int32_t amount, int frames)
{
    if (frames <= 0 || amount <= 0) return;
    /* the larger of what is shaking now and what is asked */
    const int32_t now = s_shake_len ? (int32_t)((int64_t)s_shake_amt * s_shake_left / s_shake_len) : 0;
    if (amount >= now) { s_shake_amt = amount; s_shake_len = frames; s_shake_left = frames; }
}
void vpycam_shake_offset(int32_t *dx, int32_t *dy, int32_t *dz) { *dx = s_off[0]; *dy = s_off[1]; *dz = s_off[2]; }

void vpycam_hitstop(int frames) { if (frames > s_stop) s_stop = frames; }
int  vpycam_stopped(void) { return s_stop > 0; }

void vpycam_step(void)
{
    if (s_stop > 0) s_stop--;
    if (s_shake_left > 0) {
        const int32_t a = (int32_t)((int64_t)s_shake_amt * s_shake_left / s_shake_len);   /* decays linearly */
        for (int k = 0; k < 3; k++) s_off[k] = a ? (int32_t)(rnd() % (uint32_t)(2 * a + 1)) - a : 0;
        s_shake_left--;
    } else {
        s_off[0] = s_off[1] = s_off[2] = 0;
    }
}

int vpycam_look_at(int32_t ox, int32_t oy, int32_t oz)
{
    const int32_t fx = s_f[0] + s_off[0], fy = s_f[1] + s_off[1], fz = s_f[2] + s_off[2];
    return vpy3d_look_at(fx + ox, fy + oy, fz + oz, fx, fy, fz, 0, 1, 0);
}
