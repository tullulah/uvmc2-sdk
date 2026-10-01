/*
 * vpyphys.c — see vpyphys.h. Integer only, deterministic, like the rest of libvpy.
 *
 * One step, in the order it runs (each substep of it):
 *   1. gravity into the velocities of every awake moving body
 *   2. contacts found for every pair whose masks meet, and against the floor
 *   3. a sleeping body touched hard by an awake one wakes; one touched by
 *      nothing at all wakes too — whatever held it up has gone
 *   4. VPYP_ITERATIONS passes of impulses over the contacts
 *   5. positions advanced by the velocities
 *   6. overlap pushed apart, the heavier body moving less
 *   7. bodies that have been still long enough, and are supported, sleep
 *
 * UNITS INSIDE: positions and sizes Q8 world units; velocities Q8 world units
 * per SUBSTEP; normals Q14; inverse mass Q16 (65536 / mass, 0 = immovable);
 * impulses accumulate in "mass × Q8 velocity" and are converted back to
 * mass × units/s only when reported.
 */
#include "vpyphys.h"

#define Q        8
#define ONE      (1 << Q)
#define N1       16384            /* Q14 one */

#define VPYP_ITERATIONS 6         /* impulse passes: what makes a stack hold */
#define SLOP     (ONE / 4)        /* overlap left alone, Q8: a quarter unit */
/* THE CONTACT SKIN, AND WHY IT GROWS WITH SPEED. Contacts are found BEFORE the
 * shapes touch, as far ahead as they can travel in one substep plus SKIN, and
 * the solver lets the gap close but no further (a speculative contact).
 *
 * Measured without it: a ball dropped on the floor reached a steady bounce and
 * never stopped. Falling 10 units a step it jumped over a fixed skin, sank 6
 * units into the floor, and pushing it back out handed it the energy of those 6
 * units every bounce. And a body resting exactly on the floor counted as
 * touching only every other step, so it trembled and never slept. */
#define SKIN     (2 * ONE)
#define PUSH_NUM 4                /* overlap removed per substep: 4/5 */
#define PUSH_DEN 5
#define SLEEP_STEPS 25            /* half a second still at 50 Hz */
#define SLEEP_SPEED 20            /* units/s: below this a body counts as still */
#define WAKE_SPEED  60            /* units/s: an awake body hitting a sleeper this hard wakes it */

enum { SHAPE_SPHERE = 0, SHAPE_BOX = 1 };

typedef struct {
    uint8_t  alive, shape, asleep, mask;
    int32_t  p[3];                /* Q8 */
    int32_t  v[3];                /* Q8 per substep */
    int32_t  h[3];                /* Q8 half extents; a sphere's radius is h[0] */
    int32_t  inv_m;               /* Q16 */
    int16_t  rest, fric;          /* Q8 */
    uint16_t still;               /* substeps spent below SLEEP_SPEED */
    uint16_t touching;            /* contacts this substep */
} body_t;

typedef struct {
    int16_t a, b;                 /* b may be VPYP_FLOOR */
    int32_t n[3];                 /* Q14, from a to b */
    int32_t depth;                /* Q8 */
    int32_t pt[3];                /* Q8 */
    int32_t target;               /* Q8/substep: the separation speed asked for */
    int64_t jn, jt;               /* accumulated normal and friction impulse */
    int16_t rest, fric;
} contact_t;

static body_t       s_b[VPYP_MAX_BODIES];
static contact_t    s_c[VPYP_MAX_CONTACTS];
static int          s_nc;
/* WARM STARTING: last substep's normal impulse for each pair. A contact that
 * persists starts from what it needed last time, so the solver only corrects
 * the change. Without it a stack of five boxes sank 25 units per box and never
 * slept: six passes from zero cannot carry the weight of five boxes down to the
 * floor, and the leftover sinking is what the overlap push then fought. */
typedef struct { int16_t a, b; int64_t jn; } warm_t;
static warm_t       s_w[VPYP_MAX_CONTACTS];
static int          s_nw;
static vpyp_contact s_rep[VPYP_MAX_CONTACTS];   /* what a step reports */
static int          s_nrep;
static vpyp_stats_t s_stats;

static int     s_rate = 50, s_sub = 1;
static int32_t s_g_api[3];       /* units/s² as asked, so a rate change can redo it */
static int32_t s_g[3];           /* Q8 per substep² */
static int     s_floor_on;
static int32_t s_floor_y;        /* Q8 */
static int16_t s_floor_rest = 64, s_floor_fric = 128;

/* ── small integer maths ───────────────────────────────────────────────────── */
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
static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int64_t iabs64(int64_t v) { return v < 0 ? -v : v; }

/* units/s -> Q8 per substep, rounded to nearest so a sign does not bias it */
static int32_t speed_in(int32_t u)
{
    const int64_t d = (int64_t)s_rate * s_sub;
    const int64_t n = (int64_t)u * ONE;
    return (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
}
static int32_t speed_out(int32_t q)
{
    const int64_t n = (int64_t)q * s_rate * s_sub;
    return (int32_t)((n + (n >= 0 ? ONE / 2 : -ONE / 2)) / ONE);
}
static void redo_gravity(void)
{
    const int64_t d = (int64_t)s_rate * s_sub * s_rate * s_sub;
    for (int k = 0; k < 3; k++) {
        const int64_t n = (int64_t)s_g_api[k] * ONE;
        s_g[k] = (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
    }
}

/* ── the world ─────────────────────────────────────────────────────────────── */
void vpyp_reset(void)
{
    for (int i = 0; i < VPYP_MAX_BODIES; i++) s_b[i].alive = 0;
    s_nc = 0; s_nrep = 0; s_nw = 0;
    s_rate = 50; s_sub = 1;
    s_g_api[0] = s_g_api[1] = s_g_api[2] = 0; redo_gravity();
    s_floor_on = 0;
    vpyp_stats_t z = {0}; s_stats = z;
}

/* A change of rate or substeps changes what one internal velocity unit MEANS,
 * so every velocity is carried across rather than silently rescaled. */
static void retime(int rate, int sub)
{
    int32_t keep[VPYP_MAX_BODIES][3];
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) for (int k = 0; k < 3; k++) keep[i][k] = speed_out(s_b[i].v[k]);
    s_rate = rate; s_sub = sub;
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) for (int k = 0; k < 3; k++) s_b[i].v[k] = speed_in(keep[i][k]);
    redo_gravity();
}
void vpyp_set_rate(int r)     { if (r > 0) retime(r, s_sub); }
void vpyp_set_substeps(int n) { if (n > 0) retime(s_rate, n); }

void vpyp_set_gravity(int32_t gx, int32_t gy, int32_t gz)
{
    s_g_api[0] = gx; s_g_api[1] = gy; s_g_api[2] = gz;
    redo_gravity();
    for (int i = 0; i < VPYP_MAX_BODIES; i++) { s_b[i].asleep = 0; s_b[i].still = 0; }
}

void vpyp_set_floor(int on, int32_t y, int rest_q8, int fric_q8)
{
    s_floor_on = on ? 1 : 0;
    s_floor_y = y * ONE;
    s_floor_rest = (int16_t)clampi(rest_q8, 0, ONE);
    s_floor_fric = (int16_t)clampi(fric_q8, 0, 4 * ONE);
}

/* ── bodies ────────────────────────────────────────────────────────────────── */
static body_t *get(int id)
{
    return (id >= 0 && id < VPYP_MAX_BODIES && s_b[id].alive) ? &s_b[id] : 0;
}

static int add(int shape, int32_t x, int32_t y, int32_t z,
               int32_t hx, int32_t hy, int32_t hz, int32_t mass)
{
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        if (s_b[i].alive) continue;
        body_t *b = &s_b[i];
        b->alive = 1; b->shape = (uint8_t)shape; b->asleep = 0; b->mask = 1;
        b->p[0] = x * ONE; b->p[1] = y * ONE; b->p[2] = z * ONE;
        b->v[0] = b->v[1] = b->v[2] = 0;
        b->h[0] = hx * ONE; b->h[1] = hy * ONE; b->h[2] = hz * ONE;
        b->inv_m = mass > 0 ? (int32_t)(65536 / mass) : 0;
        if (mass > 0 && b->inv_m == 0) b->inv_m = 1;     /* heavier than 65536: as heavy as it gets */
        b->rest = 64; b->fric = 128;
        b->still = 0; b->touching = 0;
        return i;
    }
    s_stats.refused++;
    return VPYP_NONE;
}

int vpyp_add_sphere(int32_t x, int32_t y, int32_t z, int32_t r, int32_t mass)
{
    return add(SHAPE_SPHERE, x, y, z, r, r, r, mass);
}
int vpyp_add_box(int32_t x, int32_t y, int32_t z,
                 int32_t hx, int32_t hy, int32_t hz, int32_t mass)
{
    return add(SHAPE_BOX, x, y, z, hx, hy, hz, mass);
}
void vpyp_remove(int id)
{
    body_t *b = get(id); if (!b) return;
    b->alive = 0;
    /* its warm impulses must not be handed to whatever takes the slot next */
    for (int i = 0; i < s_nw; i++) if (s_w[i].a == id || s_w[i].b == id) s_w[i].a = s_w[i].b = VPYP_NONE;
}
int  vpyp_alive(int id)  { return get(id) != 0; }

void vpyp_set_material(int id, int rest_q8, int fric_q8)
{
    body_t *b = get(id); if (!b) return;
    b->rest = (int16_t)clampi(rest_q8, 0, ONE);
    b->fric = (int16_t)clampi(fric_q8, 0, 4 * ONE);
}
void vpyp_set_mask(int id, uint8_t m) { body_t *b = get(id); if (b) b->mask = m; }

static void wake(body_t *b) { b->asleep = 0; b->still = 0; }

void vpyp_set_position(int id, int32_t x, int32_t y, int32_t z)
{
    body_t *b = get(id); if (!b) return;
    b->p[0] = x * ONE; b->p[1] = y * ONE; b->p[2] = z * ONE; wake(b);
}
void vpyp_set_velocity(int id, int32_t vx, int32_t vy, int32_t vz)
{
    body_t *b = get(id); if (!b || !b->inv_m) return;
    b->v[0] = speed_in(vx); b->v[1] = speed_in(vy); b->v[2] = speed_in(vz); wake(b);
}
void vpyp_apply_impulse(int id, int32_t ix, int32_t iy, int32_t iz)
{
    body_t *b = get(id); if (!b || !b->inv_m) return;
    const int32_t in[3] = { ix, iy, iz };
    for (int k = 0; k < 3; k++)
        b->v[k] += (int32_t)(((int64_t)speed_in(in[k]) * b->inv_m) >> 16);
    wake(b);
}
void vpyp_position(int id, int32_t *x, int32_t *y, int32_t *z)
{
    body_t *b = get(id);
    if (!b) { *x = *y = *z = 0; return; }
    *x = b->p[0] >> Q; *y = b->p[1] >> Q; *z = b->p[2] >> Q;   /* floor toward -inf: consistent */
}
void vpyp_velocity(int id, int32_t *vx, int32_t *vy, int32_t *vz)
{
    body_t *b = get(id);
    if (!b) { *vx = *vy = *vz = 0; return; }
    *vx = speed_out(b->v[0]); *vy = speed_out(b->v[1]); *vz = speed_out(b->v[2]);
}
int vpyp_sleeping(int id) { body_t *b = get(id); return b ? b->asleep : 0; }

/* ── finding contacts ──────────────────────────────────────────────────────── */
/* Each fills the normal (Q14, from a to b), the depth and a point; 0 = apart. */
static int sphere_sphere(const body_t *a, const body_t *b, contact_t *c, int64_t mg)
{
    int64_t d[3], d2 = 0;
    for (int k = 0; k < 3; k++) { d[k] = (int64_t)b->p[k] - a->p[k]; d2 += d[k] * d[k]; }
    const int64_t rs = (int64_t)a->h[0] + b->h[0];
    if (d2 >= (rs + mg) * (rs + mg)) return 0;
    const int64_t dist = isqrt64(d2);
    if (dist == 0) { c->n[0] = 0; c->n[1] = N1; c->n[2] = 0; }
    else for (int k = 0; k < 3; k++) c->n[k] = (int32_t)(d[k] * N1 / dist);
    c->depth = (int32_t)(rs - dist);
    for (int k = 0; k < 3; k++) c->pt[k] = a->p[k] + (int32_t)(((int64_t)c->n[k] * a->h[0]) >> 14);
    return 1;
}

/* sphere s against box x; the normal comes out pointing from the SPHERE to the box */
static int sphere_box(const body_t *s, const body_t *x, contact_t *c, int64_t mg)
{
    int32_t q[3];
    int64_t d[3], d2 = 0;
    for (int k = 0; k < 3; k++) {
        q[k] = clampi(s->p[k], x->p[k] - x->h[k], x->p[k] + x->h[k]);
        d[k] = (int64_t)s->p[k] - q[k];                       /* box -> sphere */
        d2 += d[k] * d[k];
    }
    const int64_t r = s->h[0];
    if (d2 == 0) {
        /* THE CENTRE IS INSIDE THE BOX: out through the nearest face. */
        int best = 0; int64_t bd = -1; int sgn = 1;
        for (int k = 0; k < 3; k++) {
            const int64_t up = (int64_t)x->p[k] + x->h[k] - s->p[k];
            const int64_t dn = (int64_t)s->p[k] - (x->p[k] - x->h[k]);
            if (bd < 0 || up < bd) { bd = up; best = k; sgn = 1; }
            if (dn < bd)           { bd = dn; best = k; sgn = -1; }
        }
        for (int k = 0; k < 3; k++) c->n[k] = 0;
        c->n[best] = -sgn * N1;                               /* sphere -> box */
        c->depth = (int32_t)(r + bd);
        for (int k = 0; k < 3; k++) c->pt[k] = s->p[k];
        return 1;
    }
    if (d2 >= (r + mg) * (r + mg)) return 0;
    const int64_t dist = isqrt64(d2);
    for (int k = 0; k < 3; k++) c->n[k] = (int32_t)(-d[k] * N1 / dist);
    c->depth = (int32_t)(r - dist);
    for (int k = 0; k < 3; k++) c->pt[k] = q[k];
    return 1;
}

static int box_box(const body_t *a, const body_t *b, contact_t *c, int64_t mg)
{
    int best = -1; int64_t bo = 0; int sgn = 1;
    for (int k = 0; k < 3; k++) {
        const int64_t d = (int64_t)b->p[k] - a->p[k];
        const int64_t o = (int64_t)a->h[k] + b->h[k] - iabs64(d);
        if (o <= -mg) return 0;
        if (best < 0 || o < bo) { bo = o; best = k; sgn = d >= 0 ? 1 : -1; }
    }
    for (int k = 0; k < 3; k++) {
        c->n[k] = 0;
        /* the middle of the overlap on every axis */
        const int32_t lo = a->p[k] - a->h[k] > b->p[k] - b->h[k] ? a->p[k] - a->h[k] : b->p[k] - b->h[k];
        const int32_t hi = a->p[k] + a->h[k] < b->p[k] + b->h[k] ? a->p[k] + a->h[k] : b->p[k] + b->h[k];
        c->pt[k] = lo + (hi - lo) / 2;
    }
    c->n[best] = sgn * N1;
    c->depth = (int32_t)bo;
    return 1;
}

static int body_floor(const body_t *a, contact_t *c, int64_t mg)
{
    const int32_t bottom = a->p[1] - a->h[1];                 /* a sphere's h[1] is its radius */
    if ((int64_t)bottom >= (int64_t)s_floor_y + mg) return 0;
    c->n[0] = 0; c->n[1] = -N1; c->n[2] = 0;                  /* body -> floor: down */
    c->depth = s_floor_y - bottom;
    c->pt[0] = a->p[0]; c->pt[1] = s_floor_y; c->pt[2] = a->p[2];
    return 1;
}

/* How far a body can travel in one substep, as an upper bound (L1). */
static int64_t reach(const body_t *b)
{
    if (b->asleep) return 0;
    return iabs64(b->v[0]) + iabs64(b->v[1]) + iabs64(b->v[2]);
}

static int collide(int ia, int ib, contact_t *c)
{
    const body_t *a = &s_b[ia], *b = &s_b[ib];
    const int64_t mg = SKIN + reach(a) + reach(b);
    if (a->shape == SHAPE_SPHERE && b->shape == SHAPE_SPHERE) return sphere_sphere(a, b, c, mg);
    if (a->shape == SHAPE_BOX    && b->shape == SHAPE_BOX)    return box_box(a, b, c, mg);
    if (a->shape == SHAPE_SPHERE) return sphere_box(a, b, c, mg);
    /* box a, sphere b: find it the other way round and turn the normal back */
    if (!sphere_box(b, a, c, mg)) return 0;
    for (int k = 0; k < 3; k++) c->n[k] = -c->n[k];
    return 1;
}

/* ── solving ──────────────────────────────────────────────────────────────── */
/* A body's say in a contact: nothing if it cannot move or is asleep. */
static int32_t inv_of(int id)
{
    if (id < 0) return 0;
    const body_t *b = &s_b[id];
    return b->asleep ? 0 : b->inv_m;
}
static const int32_t ZERO3[3] = { 0, 0, 0 };
static const int32_t *vel_of(int id) { return id < 0 ? ZERO3 : s_b[id].v; }

static int64_t vrel_n(const contact_t *c, int64_t vr[3])
{
    const int32_t *va = vel_of(c->a), *vb = vel_of(c->b);
    int64_t vn = 0;
    for (int k = 0; k < 3; k++) { vr[k] = (int64_t)vb[k] - va[k]; vn += vr[k] * c->n[k]; }
    return vn >> 14;
}

static void push(int id, const int64_t dv[3], int sign)
{
    if (id < 0) return;
    body_t *b = &s_b[id];
    if (b->asleep || !b->inv_m) return;
    for (int k = 0; k < 3; k++) b->v[k] += (int32_t)(sign * ((dv[k] * b->inv_m) >> 16));
}

static void solve(contact_t *c)
{
    const int32_t ima = inv_of(c->a), imb = inv_of(c->b);
    const int64_t im = (int64_t)ima + imb;
    if (im == 0) return;

    /* NORMAL: drive the approach speed to the target, never pulling (acc >= 0) */
    int64_t vr[3];
    const int64_t vn = vrel_n(c, vr);
    int64_t dj = ((int64_t)c->target - vn) * 65536 / im;
    int64_t acc = c->jn + dj;
    if (acc < 0) acc = 0;
    dj = acc - c->jn; c->jn = acc;
    if (dj) {
        int64_t imp[3];
        for (int k = 0; k < 3; k++) imp[k] = (c->n[k] * dj) >> 14;
        push(c->a, imp, -1); push(c->b, imp, +1);
    }

    /* FRICTION: oppose the sliding, at most mu times what presses them together */
    const int64_t vn2 = vrel_n(c, vr);
    int64_t vt[3], vt2 = 0;
    for (int k = 0; k < 3; k++) { vt[k] = vr[k] - ((c->n[k] * vn2) >> 14); vt2 += vt[k] * vt[k]; }
    const int64_t vtl = isqrt64(vt2);
    if (vtl == 0) return;
    const int64_t cap = (c->jn * c->fric >> Q) - c->jt;
    if (cap <= 0) return;
    int64_t jt = vtl * 65536 / im;
    if (jt > cap) jt = cap;
    c->jt += jt;
    int64_t imp[3];
    for (int k = 0; k < 3; k++) imp[k] = vt[k] * jt / vtl;   /* along the sliding */
    push(c->a, imp, +1); push(c->b, imp, -1);
}

static void separate(const contact_t *c)
{
    const int32_t ima = inv_of(c->a), imb = inv_of(c->b);
    const int64_t im = (int64_t)ima + imb;
    if (im == 0 || c->depth <= SLOP) return;
    const int64_t corr = (int64_t)(c->depth - SLOP) * PUSH_NUM / PUSH_DEN;
    for (int k = 0; k < 3; k++) {
        const int64_t m = (c->n[k] * corr) >> 14;
        if (c->a >= 0 && ima) s_b[c->a].p[k] -= (int32_t)(m * ima / im);
        if (c->b >= 0 && imb) s_b[c->b].p[k] += (int32_t)(m * imb / im);
    }
}

static int mix_rest(int a, int b) { return a > b ? a : b; }
static int mix_fric(int a, int b) { return (int)isqrt64((int64_t)a * b); }

static void add_contact(int a, int b, contact_t *c)
{
    if (s_nc >= VPYP_MAX_CONTACTS) { s_stats.contacts_dropped++; return; }
    c->a = (int16_t)a; c->b = (int16_t)b;
    c->jn = 0; c->jt = 0;
    for (int i = 0; i < s_nw; i++)
        if (s_w[i].a == a && s_w[i].b == b) { c->jn = s_w[i].jn; break; }
    const body_t *A = &s_b[a];
    if (b == VPYP_FLOOR) { c->rest = (int16_t)mix_rest(A->rest, s_floor_rest); c->fric = (int16_t)mix_fric(A->fric, s_floor_fric); }
    else { c->rest = (int16_t)mix_rest(A->rest, s_b[b].rest); c->fric = (int16_t)mix_fric(A->fric, s_b[b].fric); }
    s_c[s_nc++] = *c;
    s_b[a].touching++;
    if (b >= 0) s_b[b].touching++;
}

/* Merge one substep's contacts into what the step reports: one entry per pair,
 * the hardest hit of the step, so a crash inside a substep is not lost behind
 * the resting contact that follows it. */
static void report(void)
{
    const int64_t d = ONE;
    for (int i = 0; i < s_nc; i++) {
        const contact_t *c = &s_c[i];
        const int32_t imp = (int32_t)(c->jn * s_rate * s_sub / d);
        int j = 0;
        while (j < s_nrep && !(s_rep[j].a == c->a && s_rep[j].b == c->b)) j++;
        if (j == s_nrep) {
            if (s_nrep >= VPYP_MAX_CONTACTS) { s_stats.contacts_dropped++; continue; }
            s_nrep++; s_rep[j].impulse = 0;
        }
        vpyp_contact *r = &s_rep[j];
        r->a = c->a; r->b = c->b;
        r->x = c->pt[0] >> Q; r->y = c->pt[1] >> Q; r->z = c->pt[2] >> Q;
        r->nx = (int16_t)c->n[0]; r->ny = (int16_t)c->n[1]; r->nz = (int16_t)c->n[2];
        r->depth = c->depth >> Q;
        if (imp > r->impulse) r->impulse = imp;
    }
}

static void substep(void)
{
    /* 1. gravity */
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || b->asleep) continue;
        for (int k = 0; k < 3; k++) b->v[k] += s_g[k];
    }

    /* 2. contacts. Sleeping pairs are FOUND (so a sleeper knows it is held up)
     *    but not solved, which inv_of() sees to. */
    s_nc = 0;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) s_b[i].touching = 0;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        const body_t *a = &s_b[i];
        if (!a->alive) continue;
        if (s_floor_on && a->inv_m) {
            contact_t c;
            if (body_floor(a, &c, SKIN + reach(a))) add_contact(i, VPYP_FLOOR, &c);
        }
        for (int j = i + 1; j < VPYP_MAX_BODIES; j++) {
            const body_t *b = &s_b[j];
            if (!b->alive || !(a->mask & b->mask)) continue;
            if (!a->inv_m && !b->inv_m) continue;             /* two statics */
            s_stats.pairs++;
            contact_t c;
            if (collide(i, j, &c)) add_contact(i, j, &c);
        }
    }
    s_stats.contacts += (uint32_t)s_nc;

    /* what one substep of gravity adds, plus a margin: an approach slower than
     * this is a body RESTING on another, not hitting it. Below it nothing
     * bounces and nothing wakes — measured: with a flat 60 units/s, the weight
     * of a box resting on a sleeping one (196 units/s of gravity per step at
     * 50 Hz) woke it every time, and a stack never slept. */
    int64_t gl2 = 0;
    for (int k = 0; k < 3; k++) gl2 += (int64_t)s_g[k] * s_g[k];
    const int64_t rest_v = 3 * isqrt64(gl2) + speed_in(SLEEP_SPEED);

    /* 3. waking */
    const int64_t wake_v = rest_v > speed_in(WAKE_SPEED) ? rest_v : speed_in(WAKE_SPEED);
    for (int i = 0; i < s_nc; i++) {
        contact_t *c = &s_c[i];
        if (c->b < 0) continue;
        body_t *A = &s_b[c->a], *B = &s_b[c->b];
        if (A->asleep == B->asleep) continue;
        int64_t vr[3];
        const uint8_t sa = A->asleep, sb = B->asleep;
        A->asleep = 0; B->asleep = 0;                          /* measure as if both moved */
        const int64_t vn = vrel_n(c, vr);
        A->asleep = sa; B->asleep = sb;
        if (-vn > wake_v) { wake(A); wake(B); }
    }
    /* A SLEEPER STAYS ASLEEP ONLY WHILE SOMETHING HOLDS IT UP: a contact on its
     * gravity side with the floor, a static body or another sleeper. "Touching
     * anything" is not enough — pull the bottom box out of a sleeping stack and
     * every other box still touches the one above it, and the stack would float.
     * With no gravity there is nothing to fall, and nothing wakes on this rule. */
    if (s_g[0] | s_g[1] | s_g[2]) {
        uint8_t held[VPYP_MAX_BODIES] = {0};
        for (int i = 0; i < s_nc; i++) {
            const contact_t *c = &s_c[i];
            const int64_t ng = (int64_t)c->n[0] * s_g[0] + (int64_t)c->n[1] * s_g[1] + (int64_t)c->n[2] * s_g[2];
            /* n points from a to b: b is below a when n goes WITH gravity */
            const int below = ng > 0 ? c->b : (ng < 0 ? c->a : VPYP_NONE);
            const int above = ng > 0 ? c->a : (ng < 0 ? c->b : VPYP_NONE);
            if (above < 0) continue;
            const int firm = below == VPYP_FLOOR ||
                             (below >= 0 && (!s_b[below].inv_m || s_b[below].asleep));
            if (firm) held[above] = 1;
        }
        for (int i = 0; i < VPYP_MAX_BODIES; i++) {
            body_t *b = &s_b[i];
            if (b->alive && b->asleep && !held[i]) wake(b);
        }
    }

    /* the separation speed each contact asks for: a bounce only off a real
     * approach, so a resting body does not tremble on its own gravity */
    for (int i = 0; i < s_nc; i++) {
        contact_t *c = &s_c[i];
        int64_t vr[3];
        const int64_t vn = vrel_n(c, vr);
        /* depth < 0 is a gap inside the skin: the bodies may close it this
         * substep and no more. A bounce only when the approach is real AND
         * would close the gap now. */
        if (vn < -rest_v && vn < c->depth) c->target = (int32_t)(-vn * c->rest >> Q);
        else c->target = c->depth < 0 ? c->depth : 0;
    }

    /* 4. impulses: last substep's first, then the passes */
    for (int i = 0; i < s_nc; i++) {
        contact_t *c = &s_c[i];
        if (!c->jn) continue;
        if (!inv_of(c->a) && !inv_of(c->b)) { c->jn = 0; continue; }
        int64_t imp[3];
        for (int k = 0; k < 3; k++) imp[k] = (c->n[k] * c->jn) >> 14;
        push(c->a, imp, -1); push(c->b, imp, +1);
    }
    for (int it = 0; it < VPYP_ITERATIONS; it++)
        for (int i = 0; i < s_nc; i++) solve(&s_c[i]);
    s_nw = 0;
    for (int i = 0; i < s_nc; i++) {
        s_w[s_nw].a = s_c[i].a; s_w[s_nw].b = s_c[i].b; s_w[s_nw].jn = s_c[i].jn; s_nw++;
    }

    /* 5. positions */
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || b->asleep) continue;
        for (int k = 0; k < 3; k++) b->p[k] += b->v[k];
    }

    /* 6. overlap */
    for (int i = 0; i < s_nc; i++) separate(&s_c[i]);

    /* 7. sleep */
    const int64_t sv = speed_in(SLEEP_SPEED);
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || b->asleep) continue;
        int64_t v2 = 0;
        for (int k = 0; k < 3; k++) v2 += (int64_t)b->v[k] * b->v[k];
        if (v2 < sv * sv && b->touching) {
            if (++b->still >= SLEEP_STEPS * s_sub) { b->asleep = 1; b->v[0] = b->v[1] = b->v[2] = 0; }
        } else {
            b->still = 0;
        }
    }

    report();
}

void vpyp_step(void)
{
    s_stats.pairs = 0; s_stats.contacts = 0;
    s_nrep = 0;
    for (int s = 0; s < s_sub; s++) substep();
    uint32_t n = 0, awake = 0;
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) { n++; if (s_b[i].inv_m && !s_b[i].asleep) awake++; }
    s_stats.bodies = n; s_stats.awake = awake;
}

int vpyp_contact_count(void) { return s_nrep; }
const vpyp_contact *vpyp_contact_get(int i) { return (i >= 0 && i < s_nrep) ? &s_rep[i] : 0; }
const vpyp_stats_t *vpyp_stats(void) { return &s_stats; }

/* ── rays ─────────────────────────────────────────────────────────────────── */
/* All in Q8 world units along a Q14 unit direction; t comes out in Q8. */
static int ray_sphere(const int64_t o[3], const int32_t u[3], const body_t *b, int64_t *t, int32_t n[3])
{
    int64_t m[3], bdot = 0, mm = 0;
    for (int k = 0; k < 3; k++) { m[k] = o[k] - b->p[k]; bdot += m[k] * u[k]; mm += m[k] * m[k]; }
    bdot >>= 14;
    const int64_t r = b->h[0];
    const int64_t c = mm - r * r;
    if (c > 0 && bdot > 0) return 0;                          /* outside and pointing away */
    const int64_t disc = bdot * bdot - c;
    if (disc < 0) return 0;
    int64_t tt = -bdot - isqrt64(disc);
    if (tt < 0) tt = 0;                                       /* starting inside */
    for (int k = 0; k < 3; k++) {
        const int64_t hp = o[k] + ((u[k] * tt) >> 14) - b->p[k];
        n[k] = r ? (int32_t)(hp * N1 / r) : 0;
    }
    *t = tt;
    return 1;
}

static int ray_box(const int64_t o[3], const int32_t u[3], const body_t *b, int64_t *t, int32_t n[3])
{
    int64_t t0 = 0, t1 = (int64_t)1 << 50;
    int axis = -1, sgn = 0;
    for (int k = 0; k < 3; k++) {
        const int64_t lo = (int64_t)b->p[k] - b->h[k], hi = (int64_t)b->p[k] + b->h[k];
        if (u[k] == 0) { if (o[k] < lo || o[k] > hi) return 0; continue; }
        int64_t ta = (lo - o[k]) * N1 / u[k], tb = (hi - o[k]) * N1 / u[k];
        int s = -1;                                           /* entering through the low face */
        if (ta > tb) { const int64_t x = ta; ta = tb; tb = x; s = 1; }
        if (ta > t0) { t0 = ta; axis = k; sgn = s; }
        if (tb < t1) t1 = tb;
        if (t0 > t1) return 0;
    }
    n[0] = n[1] = n[2] = 0;
    if (axis >= 0) n[axis] = sgn * N1;
    *t = t0;
    return 1;
}

int vpyp_raycast(int32_t ox, int32_t oy, int32_t oz,
                 int32_t dx, int32_t dy, int32_t dz,
                 int32_t max_dist, uint8_t mask, vpyp_hit *out)
{
    const int64_t dl = isqrt64((int64_t)dx * dx + (int64_t)dy * dy + (int64_t)dz * dz);
    if (dl == 0) return VPYP_NONE;
    const int32_t u[3] = { (int32_t)((int64_t)dx * N1 / dl), (int32_t)((int64_t)dy * N1 / dl),
                           (int32_t)((int64_t)dz * N1 / dl) };
    const int64_t o[3] = { (int64_t)ox * ONE, (int64_t)oy * ONE, (int64_t)oz * ONE };
    int64_t best = (int64_t)max_dist * ONE;
    int id = VPYP_NONE;
    int32_t bn[3] = { 0, 0, 0 };

    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        const body_t *b = &s_b[i];
        if (!b->alive || !(b->mask & mask)) continue;
        int64_t t; int32_t n[3];
        const int hit = b->shape == SHAPE_SPHERE ? ray_sphere(o, u, b, &t, n) : ray_box(o, u, b, &t, n);
        if (hit && t <= best) { best = t; id = i; bn[0] = n[0]; bn[1] = n[1]; bn[2] = n[2]; }
    }
    if (s_floor_on && u[1] < 0 && o[1] > s_floor_y) {
        const int64_t t = (o[1] - s_floor_y) * N1 / -u[1];
        if (t <= best) { best = t; id = VPYP_FLOOR; bn[0] = 0; bn[1] = N1; bn[2] = 0; }
    }
    if (id == VPYP_NONE) return VPYP_NONE;
    if (out) {
        out->id = id;
        out->x = (int32_t)((o[0] + ((u[0] * best) >> 14)) >> Q);
        out->y = (int32_t)((o[1] + ((u[1] * best) >> 14)) >> Q);
        out->z = (int32_t)((o[2] + ((u[2] * best) >> 14)) >> Q);
        out->nx = (int16_t)bn[0]; out->ny = (int16_t)bn[1]; out->nz = (int16_t)bn[2];
        out->dist = (int32_t)(best >> Q);
    }
    return id;
}
