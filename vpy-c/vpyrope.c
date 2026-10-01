/*
 * vpyrope.c — see vpyrope.h. Positions Q8 world units; Verlet: the velocity is
 * the difference between where a point is and where it was.
 */
#include "vpyrope.h"
#include "vpy.h"
#include "vpy3d.h"

#define Q   8
#define ONE (1 << Q)

typedef struct { int32_t p[3], o[3]; uint8_t pinned; int32_t pin[3]; } pt_t;
typedef struct { uint8_t used; int first, n; int32_t rest; } rope_t;   /* rest: link length, Q8 */

static pt_t   s_p[VPYROPE_MAX_POINTS];
static rope_t s_r[VPYROPE_MAX_ROPES];
static int    s_rate = 50, s_iter = 8, s_damp = 254;
static int32_t s_g_api[3], s_g[3];
static int    s_floor_on; static int32_t s_floor_y;
static vpyrope_stats_t s_stats;

static int64_t isqrt64(int64_t n)
{
    if (n <= 0) return 0;
    int64_t r = 0, bit = (int64_t)1 << 62;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return r;
}
static void redo_gravity(void)
{
    const int64_t d = (int64_t)s_rate * s_rate;
    for (int k = 0; k < 3; k++) {
        const int64_t n = (int64_t)s_g_api[k] * ONE;
        s_g[k] = (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
    }
}

void vpyrope_reset(void)
{
    for (int i = 0; i < VPYROPE_MAX_ROPES; i++) s_r[i].used = 0;
    s_rate = 50; s_iter = 8; s_damp = 254;
    s_g_api[0] = s_g_api[1] = s_g_api[2] = 0; redo_gravity();
    s_floor_on = 0;
    vpyrope_stats_t z = {0}; s_stats = z;
}
void vpyrope_set_rate(int r) { if (r > 0) { s_rate = r; redo_gravity(); } }
void vpyrope_set_gravity(int32_t gx, int32_t gy, int32_t gz) { s_g_api[0] = gx; s_g_api[1] = gy; s_g_api[2] = gz; redo_gravity(); }
void vpyrope_set_floor(int on, int32_t y) { s_floor_on = on ? 1 : 0; s_floor_y = y * ONE; }
void vpyrope_set_iterations(int n) { s_iter = n < 1 ? 1 : (n > 64 ? 64 : n); }
void vpyrope_set_damping(int q8) { s_damp = q8 < 0 ? 0 : (q8 > ONE ? ONE : q8); }
const vpyrope_stats_t *vpyrope_stats(void) { return &s_stats; }

/* the points are kept packed: a new rope takes the first run of free slots */
static int free_run(int n)
{
    for (int start = 0; start + n <= VPYROPE_MAX_POINTS; start++) {
        int ok = 1;
        for (int r = 0; r < VPYROPE_MAX_ROPES && ok; r++)
            if (s_r[r].used && start < s_r[r].first + s_r[r].n && s_r[r].first < start + n) ok = 0;
        if (ok) return start;
    }
    return -1;
}

int vpyrope_new(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz, int links)
{
    if (links < 1) return -1;
    int r = 0;
    while (r < VPYROPE_MAX_ROPES && s_r[r].used) r++;
    const int start = r < VPYROPE_MAX_ROPES ? free_run(links + 1) : -1;
    if (start < 0) { s_stats.refused++; return -1; }
    const int64_t a[3] = { (int64_t)ax * ONE, (int64_t)ay * ONE, (int64_t)az * ONE };
    const int64_t b[3] = { (int64_t)bx * ONE, (int64_t)by * ONE, (int64_t)bz * ONE };
    for (int i = 0; i <= links; i++) {
        pt_t *p = &s_p[start + i];
        for (int k = 0; k < 3; k++) { p->p[k] = (int32_t)(a[k] + (b[k] - a[k]) * i / links); p->o[k] = p->p[k]; }
        p->pinned = 0;
    }
    const int64_t d[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    s_r[r].used = 1; s_r[r].first = start; s_r[r].n = links + 1;
    s_r[r].rest = (int32_t)(isqrt64(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) / links);
    return r;
}
void vpyrope_free(int rope) { if (rope >= 0 && rope < VPYROPE_MAX_ROPES) s_r[rope].used = 0; }

static pt_t *pt(int rope, int i)
{
    if (rope < 0 || rope >= VPYROPE_MAX_ROPES || !s_r[rope].used || i < 0 || i >= s_r[rope].n) return 0;
    return &s_p[s_r[rope].first + i];
}
void vpyrope_pin(int rope, int i, int32_t x, int32_t y, int32_t z)
{
    pt_t *p = pt(rope, i); if (!p) return;
    p->pinned = 1; p->pin[0] = x * ONE; p->pin[1] = y * ONE; p->pin[2] = z * ONE;
}
void vpyrope_unpin(int rope, int i) { pt_t *p = pt(rope, i); if (p) p->pinned = 0; }
int  vpyrope_points(int rope) { return (rope >= 0 && rope < VPYROPE_MAX_ROPES && s_r[rope].used) ? s_r[rope].n : 0; }
void vpyrope_point(int rope, int i, int32_t *x, int32_t *y, int32_t *z)
{
    const pt_t *p = pt(rope, i);
    if (!p) { *x = *y = *z = 0; return; }
    *x = p->p[0] >> Q; *y = p->p[1] >> Q; *z = p->p[2] >> Q;
}

void vpyrope_step(void)
{
    uint32_t ropes = 0, points = 0;
    int32_t worst = 0;
    for (int r = 0; r < VPYROPE_MAX_ROPES; r++) {
        rope_t *R = &s_r[r];
        if (!R->used) continue;
        ropes++; points += (uint32_t)R->n;
        pt_t *P = &s_p[R->first];
        /* move: Verlet, damped, with gravity; a pinned point goes where it is held */
        for (int i = 0; i < R->n; i++) {
            pt_t *p = &P[i];
            if (p->pinned) { for (int k = 0; k < 3; k++) { p->o[k] = p->p[k]; p->p[k] = p->pin[k]; } continue; }
            for (int k = 0; k < 3; k++) {
                const int32_t v = (int32_t)(((int64_t)(p->p[k] - p->o[k]) * s_damp) >> Q);
                p->o[k] = p->p[k];
                p->p[k] += v + s_g[k];
            }
        }
        /* links back to length, a few times; a pinned end does not give */
        for (int it = 0; it < s_iter; it++) {
            for (int i = 0; i + 1 < R->n; i++) {
                pt_t *a = &P[i], *b = &P[i + 1];
                int64_t d[3], l2 = 0;
                for (int k = 0; k < 3; k++) { d[k] = (int64_t)b->p[k] - a->p[k]; l2 += d[k] * d[k]; }
                const int64_t l = isqrt64(l2);
                if (l == 0) continue;
                const int64_t diff = l - R->rest;                 /* too long > 0 */
                const int wa = a->pinned ? 0 : 1, wb = b->pinned ? 0 : 1;
                if (!(wa + wb)) continue;
                for (int k = 0; k < 3; k++) {
                    const int64_t m = d[k] * diff / l;
                    if (wa) a->p[k] += (int32_t)(m * wa / (wa + wb));
                    if (wb) b->p[k] -= (int32_t)(m * wb / (wa + wb));
                }
            }
            if (s_floor_on)
                for (int i = 0; i < R->n; i++) if (!P[i].pinned && P[i].p[1] < s_floor_y) P[i].p[1] = s_floor_y;
        }
        for (int i = 0; i + 1 < R->n; i++) {
            int64_t l2 = 0;
            for (int k = 0; k < 3; k++) { const int64_t d = (int64_t)P[i + 1].p[k] - P[i].p[k]; l2 += d * d; }
            const int32_t over = (int32_t)((isqrt64(l2) - R->rest) >> Q);
            if (over > worst) worst = over;
        }
    }
    s_stats.ropes = ropes; s_stats.points = points; s_stats.stretch = worst;
}

static void draw_mode(int rope, int br, int mode)
{
    if (rope < 0 || rope >= VPYROPE_MAX_ROPES || !s_r[rope].used) return;
    const pt_t *P = &s_p[s_r[rope].first];
    for (int i = 0; i + 1 < s_r[rope].n; i++) {
        const int32_t ax = P[i].p[0] >> Q, ay = P[i].p[1] >> Q, az = P[i].p[2] >> Q;
        const int32_t bx = P[i + 1].p[0] >> Q, by = P[i + 1].p[1] >> Q, bz = P[i + 1].p[2] >> Q;
        if (mode == 2) vpy_draw_line_dev(ax, ay, bx, by, br);
        else if (mode == 1) vpy3d_occl_line(ax, ay, az, bx, by, bz, br);
        else vpy3d_line_world(ax, ay, az, bx, by, bz, br);
    }
}
void vpyrope_draw(int rope, int br, int occlude) { draw_mode(rope, br, occlude ? 1 : 0); }
void vpyrope_draw2d(int rope, int br) { draw_mode(rope, br, 2); }
