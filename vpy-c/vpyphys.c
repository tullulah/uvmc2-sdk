/*
 * vpyphys.c — see vpyphys.h. Integer only, deterministic, like the rest of libvpy.
 *
 * One step, in the order it runs (each substep of it):
 *   1. gravity into the velocities of every awake moving body
 *   2. contacts found for every pair whose masks meet, and against the floor —
 *      each with the POINT where it acts, so a hit off-centre turns the body
 *   3. a sleeping body hit hard by an awake one wakes; one that nothing holds
 *      up any more wakes too
 *   4. last substep's impulses re-applied (warm starting), then
 *      VPYP_ITERATIONS passes of impulses over the contacts
 *   5. positions and orientations advanced; spin damped a little
 *   6. overlap pushed apart, the heavier body moving less
 *   7. bodies that have been still long enough, and are supported, sleep
 *
 * UNITS INSIDE
 *   position, size      Q8 world units
 *   velocity            Q8 world units per SUBSTEP
 *   angular velocity    radians per substep, Q20
 *   orientation         unit quaternion, Q30 (w, x, y, z)
 *   rotation matrix     Q14, rows (the same layout as vpy_xf.m)
 *   normals             Q14
 *   inverse mass        Q16 (65536 / mass; 0 = immovable)
 *   inverse inertia     Q32, 1 / (mass × units²); 0 = does not turn
 *   impulse             mass × Q8 velocity, converted only when reported
 *
 * INERTIA IS A SCALAR: the mean of the three principal moments. Exact for a
 * sphere and a cube; a long box turns a little too easily about its long axis
 * and a little too hard across it. A full tensor costs a matrix product per
 * contact per pass, and on a 50 Hz tube nobody will see the difference.
 */
#include "vpyphys.h"

int vpy_sin_q14(int a);          /* vpy.c: every game that has libvpy links it */
int vpy_cos_q14(int a);

#define Q        8
#define ONE      (1 << Q)
#define N1       16384            /* Q14 one */
#define QQ       30               /* quaternion scale */
#define QONE     ((int64_t)1 << QQ)
#define WQ       20               /* angular velocity scale */

#define VPYP_ITERATIONS 8         /* impulse passes: what makes a stack hold */
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
#define SLEEP_SPIN  205           /* 4096ths of a turn per second: ~18 deg/s */
#define WAKE_SPEED  60            /* units/s: an awake body hitting a sleeper this hard wakes it */
/* SPIN DAMPING: 1/128 of the spin per substep. Without any, a ball rolling on a
 * flat floor rolls for ever — friction only couples rolling to sliding, it
 * takes nothing away — and so it never sleeps. This is rolling resistance. */
#define SPIN_DAMP_SHIFT 7
#define MAX_PAIR_CONTACTS 4       /* a box on a box needs four corners, no more */

enum { SHAPE_SPHERE = 0, SHAPE_BOX = 1 };

typedef struct {
    uint8_t  alive, shape, asleep, mask;
    int32_t  p[3];                /* Q8 */
    int32_t  v[3];                /* Q8 per substep */
    int32_t  w[3];                /* rad per substep, Q20 */
    int32_t  q[4];                /* Q30: w, x, y, z */
    int32_t  r[9];                /* Q14 rotation, derived from q each substep */
    int32_t  h[3];                /* Q8 half extents; a sphere's radius is h[0] */
    int32_t  inv_m;               /* Q16 */
    int64_t  inv_i;               /* Q32 per (mass × units²) */
    int64_t  inv_i_free;          /* what inv_i is when rotation is not locked */
    int16_t  rest, fric;          /* Q8 */
    uint16_t still;               /* substeps spent below SLEEP_SPEED */
    uint16_t touching;            /* contacts this substep */
} body_t;

/* KEPT SMALL ON PURPOSE: there are VPYP_MAX_CONTACTS of these. Directions are
 * Q14 and fit 16 bits; the point is not stored (it is the centre plus ra). */
typedef struct {
    int16_t a, b;                 /* b may be VPYP_FLOOR */
    int16_t feat;                 /* which corner: keeps warm starting per point */
    int16_t npair;                /* how many contacts this pair has: shares the push */
    int16_t n[3];                 /* Q14, from a to b */
    int16_t t1[3], t2[3];         /* two tangents, Q14: friction lives in this plane */
    int16_t rest, fric;
    int32_t depth;                /* Q8 */
    int32_t ra[3], rb[3];         /* Q8, from each centre to the point */
    int32_t target;               /* Q8/substep: the separation speed asked for */
    int32_t kn, kt1, kt2;         /* inverse masses along n and the tangents, Q16 */
    int64_t jn, jt1, jt2;         /* accumulated normal and friction impulses */
} contact_t;

static body_t       s_b[VPYP_MAX_BODIES];
static contact_t    s_c[VPYP_MAX_CONTACTS];
static int          s_nc;
/* WARM STARTING: last substep's normal impulse for each contact point. A
 * contact that persists starts from what it needed last time, so the solver
 * only corrects the change. Without it a stack of five boxes sank 25 units per
 * box and never slept: passes from zero cannot carry the weight of five boxes
 * down to the floor, and the leftover sinking is what the overlap push fought. */
typedef struct { int16_t a, b, feat; int64_t jn; } warm_t;
static warm_t       s_w[VPYP_MAX_CONTACTS];
static int          s_nw;
/* What a step reports: one entry per touching PAIR, so half the contact table
 * is plenty (a box pair alone is four contacts and one report). */
#define MAX_REPORTS (VPYP_MAX_CONTACTS / 2)
static vpyp_contact s_rep[MAX_REPORTS];
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

static void cross(const int64_t a[3], const int64_t b[3], int64_t o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
/* rotation (Q14) times a vector, and its transpose */
static void rot(const int32_t m[9], const int64_t v[3], int64_t o[3])
{
    for (int i = 0; i < 3; i++) o[i] = (m[i*3] * v[0] + m[i*3+1] * v[1] + m[i*3+2] * v[2]) >> 14;
}
static void rot_t(const int32_t m[9], const int64_t v[3], int64_t o[3])
{
    for (int i = 0; i < 3; i++) o[i] = (m[i] * v[0] + m[3+i] * v[1] + m[6+i] * v[2]) >> 14;
}

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
/* 4096ths of a turn per second <-> radians per substep Q20.
 * 2*pi * 2^20 / 4096 = 1608.495... */
static int32_t spin_in(int32_t a)
{
    const int64_t d = (int64_t)s_rate * s_sub * 1000;
    const int64_t n = (int64_t)a * 1608495;
    return (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
}
static int32_t spin_out(int32_t w)
{
    const int64_t n = (int64_t)w * s_rate * s_sub * 1000;
    return (int32_t)((n + (n >= 0 ? 804247 : -804247)) / 1608495);
}
static void redo_gravity(void)
{
    const int64_t d = (int64_t)s_rate * s_sub * s_rate * s_sub;
    for (int k = 0; k < 3; k++) {
        const int64_t n = (int64_t)s_g_api[k] * ONE;
        s_g[k] = (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
    }
}

/* ── orientation ───────────────────────────────────────────────────────────── */
static void quat_normalise(int32_t q[4])
{
    int64_t l2 = 0;
    for (int k = 0; k < 4; k++) l2 += ((int64_t)q[k] * q[k]) >> 30;   /* Q30 */
    const int64_t l = isqrt64(l2 << 30);                              /* Q30 */
    if (l == 0) { q[0] = (int32_t)QONE; q[1] = q[2] = q[3] = 0; return; }
    for (int k = 0; k < 4; k++) q[k] = (int32_t)(((int64_t)q[k] << 30) / l);
}

static void quat_to_matrix(const int32_t q[4], int32_t m[9])
{
    const int64_t w = q[0], x = q[1], y = q[2], z = q[3];
    /* products are Q60; the matrix is Q14: >> 46, and the 2× folds into >> 45 */
    m[0] = (int32_t)(N1 - (((y * y + z * z) >> 30) * N1 >> 29));
    m[1] = (int32_t)(((x * y - w * z) >> 30) * N1 >> 29);
    m[2] = (int32_t)(((x * z + w * y) >> 30) * N1 >> 29);
    m[3] = (int32_t)(((x * y + w * z) >> 30) * N1 >> 29);
    m[4] = (int32_t)(N1 - (((x * x + z * z) >> 30) * N1 >> 29));
    m[5] = (int32_t)(((y * z - w * x) >> 30) * N1 >> 29);
    m[6] = (int32_t)(((x * z - w * y) >> 30) * N1 >> 29);
    m[7] = (int32_t)(((y * z + w * x) >> 30) * N1 >> 29);
    m[8] = (int32_t)(N1 - (((x * x + y * y) >> 30) * N1 >> 29));
}

/* q += 0.5 * (0, w) * q, with w in rad per substep Q20 */
static void quat_integrate(int32_t q[4], const int32_t w[3])
{
    const int64_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const int64_t wx = w[0], wy = w[1], wz = w[2];
    const int sh = WQ + 1;                               /* Q20 rate, and the half */
    q[0] += (int32_t)((-wx * qx - wy * qy - wz * qz) >> sh);
    q[1] += (int32_t)(( wx * qw + wy * qz - wz * qy) >> sh);
    q[2] += (int32_t)(( wy * qw + wz * qx - wx * qz) >> sh);
    q[3] += (int32_t)(( wz * qw + wx * qy - wy * qx) >> sh);
    quat_normalise(q);
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
    static int32_t keep[VPYP_MAX_BODIES][6];
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) for (int k = 0; k < 3; k++) {
            keep[i][k] = speed_out(s_b[i].v[k]); keep[i][3 + k] = spin_out(s_b[i].w[k]);
        }
    s_rate = rate; s_sub = sub;
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) for (int k = 0; k < 3; k++) {
            s_b[i].v[k] = speed_in(keep[i][k]); s_b[i].w[k] = spin_in(keep[i][3 + k]);
        }
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

/* 2^32 / I, with I the mean principal moment in mass × units² */
static int64_t inverse_inertia(int shape, int32_t mass, int32_t hx, int32_t hy, int32_t hz)
{
    if (mass <= 0) return 0;
    int64_t i;
    if (shape == SHAPE_SPHERE) i = (int64_t)2 * mass * hx * hx / 5;
    else i = (int64_t)2 * mass * ((int64_t)hx * hx + (int64_t)hy * hy + (int64_t)hz * hz) / 9;
    if (i <= 0) i = 1;
    const int64_t inv = (((int64_t)1 << 32) + i / 2) / i;
    return inv > 0 ? inv : 1;
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
        b->w[0] = b->w[1] = b->w[2] = 0;
        b->q[0] = (int32_t)QONE; b->q[1] = b->q[2] = b->q[3] = 0;
        quat_to_matrix(b->q, b->r);
        b->h[0] = hx * ONE; b->h[1] = hy * ONE; b->h[2] = hz * ONE;
        if (mass > 4096) mass = 4096;   /* the impulse arithmetic is sized for this */
        b->inv_m = mass > 0 ? (int32_t)(65536 / mass) : 0;
        b->inv_i_free = inverse_inertia(shape, mass, hx, hy, hz);
        b->inv_i = b->inv_i_free;
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
int vpyp_alive(int id) { return get(id) != 0; }

void vpyp_set_material(int id, int rest_q8, int fric_q8)
{
    body_t *b = get(id); if (!b) return;
    b->rest = (int16_t)clampi(rest_q8, 0, ONE);
    b->fric = (int16_t)clampi(fric_q8, 0, 4 * ONE);
}
void vpyp_set_mask(int id, uint8_t m) { body_t *b = get(id); if (b) b->mask = m; }
void vpyp_lock_rotation(int id, int on)
{
    body_t *b = get(id); if (!b) return;
    b->inv_i = on ? 0 : b->inv_i_free;
    if (on) b->w[0] = b->w[1] = b->w[2] = 0;
}

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
void vpyp_set_spin(int id, int32_t wx, int32_t wy, int32_t wz)
{
    body_t *b = get(id); if (!b || !b->inv_i) return;
    b->w[0] = spin_in(wx); b->w[1] = spin_in(wy); b->w[2] = spin_in(wz); wake(b);
}
void vpyp_set_rotation(int id, int32_t ax, int32_t ay, int32_t az, int angle)
{
    body_t *b = get(id); if (!b) return;
    const int64_t l = isqrt64((int64_t)ax * ax + (int64_t)ay * ay + (int64_t)az * az);
    if (l == 0) return;
    const int64_t s = vpy_sin_q14(angle / 2), c = vpy_cos_q14(angle / 2);   /* Q14 */
    b->q[0] = (int32_t)(c << 16);
    b->q[1] = (int32_t)((s << 16) * ax / l);
    b->q[2] = (int32_t)((s << 16) * ay / l);
    b->q[3] = (int32_t)((s << 16) * az / l);
    quat_normalise(b->q);
    quat_to_matrix(b->q, b->r);
    wake(b);
}

/* r × (impulse) into the spin, Q20; two shifts so nothing overflows (see UNITS) */
static void spin_kick(body_t *b, const int64_t r[3], const int64_t imp[3], int sign)
{
    if (!b->inv_i) return;
    int64_t t[3];
    cross(r, imp, t);
    for (int k = 0; k < 3; k++) b->w[k] += (int32_t)(sign * ((((t[k] >> 14) * b->inv_i) >> 14)));
}

void vpyp_apply_impulse(int id, int32_t ix, int32_t iy, int32_t iz)
{
    body_t *b = get(id); if (!b || !b->inv_m) return;
    const int32_t in[3] = { ix, iy, iz };
    for (int k = 0; k < 3; k++)
        b->v[k] += (int32_t)(((int64_t)speed_in(in[k]) * b->inv_m) >> 16);
    wake(b);
}
void vpyp_apply_impulse_at(int id, int32_t ix, int32_t iy, int32_t iz,
                           int32_t px, int32_t py, int32_t pz)
{
    body_t *b = get(id); if (!b || !b->inv_m) return;
    vpyp_apply_impulse(id, ix, iy, iz);
    /* the impulse in internal units is speed_in(i) × mass, i.e. speed_in(i) × 65536 / inv_m */
    const int64_t imp[3] = { (int64_t)speed_in(ix) * 65536 / b->inv_m,
                             (int64_t)speed_in(iy) * 65536 / b->inv_m,
                             (int64_t)speed_in(iz) * 65536 / b->inv_m };
    const int64_t r[3] = { (int64_t)px * ONE - b->p[0], (int64_t)py * ONE - b->p[1], (int64_t)pz * ONE - b->p[2] };
    spin_kick(b, r, imp, +1);
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
void vpyp_spin(int id, int32_t *wx, int32_t *wy, int32_t *wz)
{
    body_t *b = get(id);
    if (!b) { *wx = *wy = *wz = 0; return; }
    *wx = spin_out(b->w[0]); *wy = spin_out(b->w[1]); *wz = spin_out(b->w[2]);
}
void vpyp_rotation(int id, int32_t m[9])
{
    body_t *b = get(id);
    for (int k = 0; k < 9; k++) m[k] = b ? b->r[k] : ((k % 4) ? 0 : N1);
}
int vpyp_sleeping(int id) { body_t *b = get(id); return b ? b->asleep : 0; }

/* ── finding contacts ──────────────────────────────────────────────────────── */
/* A candidate before it is a contact: normal from a to b, depth, point. */
typedef struct { int32_t n[3]; int32_t depth; int32_t pt[3]; int16_t feat; } cand_t;

static int sphere_sphere(const body_t *a, const body_t *b, cand_t *c, int64_t mg)
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
    c->feat = 0;
    return 1;
}

/* sphere s against box x (rotated); the normal points from the SPHERE to the box */
static int sphere_box(const body_t *s, const body_t *x, cand_t *c, int64_t mg)
{
    int64_t rel[3], loc[3], ql[3], qw[3];
    for (int k = 0; k < 3; k++) rel[k] = (int64_t)s->p[k] - x->p[k];
    rot_t(x->r, rel, loc);                                    /* into the box's frame */
    for (int k = 0; k < 3; k++) ql[k] = clampi((int32_t)loc[k], -x->h[k], x->h[k]);
    int64_t d[3], d2 = 0;
    for (int k = 0; k < 3; k++) { d[k] = loc[k] - ql[k]; d2 += d[k] * d[k]; }   /* box -> sphere, local */
    const int64_t r = s->h[0];
    int64_t nl[3];
    if (d2 == 0) {
        /* THE CENTRE IS INSIDE THE BOX: out through the nearest face. */
        int best = 0; int64_t bd = -1; int sgn = 1;
        for (int k = 0; k < 3; k++) {
            const int64_t up = x->h[k] - loc[k], dn = loc[k] + x->h[k];
            if (bd < 0 || up < bd) { bd = up; best = k; sgn = 1; }
            if (dn < bd)           { bd = dn; best = k; sgn = -1; }
        }
        nl[0] = nl[1] = nl[2] = 0; nl[best] = -sgn * N1;      /* sphere -> box */
        c->depth = (int32_t)(r + bd);
        int64_t nw[3]; rot(x->r, nl, nw);
        for (int k = 0; k < 3; k++) { c->n[k] = (int32_t)nw[k]; c->pt[k] = s->p[k]; }
        c->feat = 0;
        return 1;
    }
    if (d2 >= (r + mg) * (r + mg)) return 0;
    const int64_t dist = isqrt64(d2);
    for (int k = 0; k < 3; k++) nl[k] = -d[k] * N1 / dist;
    int64_t nw[3]; rot(x->r, nl, nw);
    rot(x->r, ql, qw);
    for (int k = 0; k < 3; k++) { c->n[k] = (int32_t)nw[k]; c->pt[k] = x->p[k] + (int32_t)qw[k]; }
    c->depth = (int32_t)(r - dist);
    c->feat = 0;
    return 1;
}

/* the eight corners of a box in the world, Q8; corner k: bit 0 +x, bit 1 +y, bit 2 +z */
static void corners(const body_t *b, int64_t out[8][3])
{
    for (int k = 0; k < 8; k++) {
        const int64_t l[3] = { (k & 1) ? b->h[0] : -b->h[0], (k & 2) ? b->h[1] : -b->h[1],
                               (k & 4) ? b->h[2] : -b->h[2] };
        int64_t w[3]; rot(b->r, l, w);
        for (int j = 0; j < 3; j++) out[k][j] = b->p[j] + w[j];
    }
}

/* how far a box reaches along a unit axis (Q14): the sum of its half sizes'
 * projections */
static int64_t box_radius(const body_t *b, const int64_t ax[3])
{
    int64_t s = 0;
    for (int k = 0; k < 3; k++) {
        const int64_t d = (b->r[k] * ax[0] + b->r[3+k] * ax[1] + b->r[6+k] * ax[2]) >> 14;   /* column k · ax */
        s += (iabs64(d) * b->h[k]) >> 14;
    }
    return s;
}

/* FROM ALL THE CANDIDATE POINTS, THE FOUR THAT SPAN THE SUPPORT.
 *
 * Measured: keeping the four DEEPEST tipped a perfectly stacked column of boxes
 * over. Two aligned boxes offer each corner twice (one from each box), all at
 * the same depth, and four "deepest" could all come from one side — a support
 * on one edge, which is a turning moment that is not there. So: drop points
 * that coincide, and of what is left keep the extremes across the face — the
 * corners of the contact polygon — which is what holds a box flat. `u` and `v`
 * are two directions across the face, Q14. */
#define MAX_CAND 16
static int spread(cand_t *list, int n, const int64_t u[3], const int64_t v[3], cand_t *out)
{
    cand_t uniq[MAX_CAND]; int m = 0;
    for (int i = 0; i < n; i++) {
        int dup = 0;
        for (int j = 0; j < m && !dup; j++) {
            int64_t d = 0;
            for (int k = 0; k < 3; k++) d += iabs64((int64_t)list[i].pt[k] - uniq[j].pt[k]);
            if (d < ONE) { dup = 1; if (list[i].depth > uniq[j].depth) uniq[j] = list[i]; }
        }
        if (!dup) uniq[m++] = list[i];
    }
    if (m <= MAX_PAIR_CONTACTS) { for (int i = 0; i < m; i++) out[i] = uniq[i]; return m; }
    /* the extremes along u+v, u-v and their opposites */
    int pick[4] = { 0, 0, 0, 0 }; int64_t ext[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < m; i++) {
        int64_t pu = 0, pv = 0;
        for (int k = 0; k < 3; k++) { pu += (int64_t)uniq[i].pt[k] * u[k]; pv += (int64_t)uniq[i].pt[k] * v[k]; }
        pu >>= 14; pv >>= 14;
        const int64_t key[4] = { pu + pv, -(pu + pv), pu - pv, -(pu - pv) };
        for (int e = 0; e < 4; e++) if (i == 0 || key[e] > ext[e]) { ext[e] = key[e]; pick[e] = i; }
    }
    int nout = 0;
    for (int e = 0; e < 4; e++) {
        int seen = 0;
        for (int j = 0; j < e; j++) if (pick[j] == pick[e]) seen = 1;
        if (!seen) out[nout++] = uniq[pick[e]];
    }
    return nout;
}

/* BOX AGAINST BOX, by the six face axes (separating-axis test) and corners.
 *
 * The axis of least overlap names a REFERENCE face; the contact points are the
 * other box's corners that lie under that face and within its extent, plus the
 * reference box's own corners that lie inside the other box — together that is
 * the contact polygon of two faces resting on each other, which is what makes
 * a box sit flat on another instead of on a point.
 *
 * NOT CHECKED: the nine edge-against-edge axes. Two boxes meeting edge to edge
 * at a skew can pass a little into each other before a face takes over. */
static int box_box(const body_t *a, const body_t *b, cand_t *out, int64_t mg)
{
    int64_t d[3];
    for (int k = 0; k < 3; k++) d[k] = (int64_t)b->p[k] - a->p[k];
    int ref = -1, refk = 0; int64_t best = 0; int64_t bax[3] = {0, 0, 0};
    for (int f = 0; f < 6; f++) {
        const body_t *o = f < 3 ? a : b;
        const int k = f % 3;
        const int64_t ax[3] = { o->r[k], o->r[3+k], o->r[6+k] };      /* column k */
        const int64_t dist = iabs64((d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]) >> 14);
        const int64_t ov = box_radius(a, ax) + box_radius(b, ax) - dist;
        if (ov <= -mg) return 0;                                       /* separated */
        /* a's faces win ties, so a resting pair does not flip its reference */
        if (ref < 0 || ov < best - (f >= 3 ? 16 : 0)) {
            best = ov; ref = f < 3 ? 0 : 1; refk = k;
            bax[0] = ax[0]; bax[1] = ax[1]; bax[2] = ax[2];
        }
    }
    const body_t *R = ref ? b : a, *I = ref ? a : b;
    /* the reference normal, pointing from R towards I */
    const int64_t s = (d[0] * bax[0] + d[1] * bax[1] + d[2] * bax[2]) * (ref ? -1 : 1);
    int64_t n[3];
    for (int k = 0; k < 3; k++) n[k] = s >= 0 ? bax[k] : -bax[k];
    /* the normal of the contact goes from a to b */
    int32_t nab[3];
    for (int k = 0; k < 3; k++) nab[k] = (int32_t)(ref ? -n[k] : n[k]);

    cand_t list[MAX_CAND]; int nl = 0;
    const int u = (refk + 1) % 3, v = (refk + 2) % 3;                  /* the face's own axes */
    int64_t ci[8][3]; corners(I, ci);
    for (int c = 0; c < 8; c++) {
        int64_t rel[3], loc[3];
        for (int k = 0; k < 3; k++) rel[k] = ci[c][k] - R->p[k];
        rot_t(R->r, rel, loc);
        const int64_t along = (rel[0] * n[0] + rel[1] * n[1] + rel[2] * n[2]) >> 14;
        const int64_t depth = R->h[refk] - along;
        if (depth <= -mg) continue;
        if (iabs64(loc[u]) > R->h[u] + mg || iabs64(loc[v]) > R->h[v] + mg) continue;
        cand_t cd;
        for (int k = 0; k < 3; k++) { cd.n[k] = nab[k]; cd.pt[k] = (int32_t)ci[c][k]; }
        cd.depth = (int32_t)depth; cd.feat = (int16_t)(c + (ref ? 8 : 0));
        if (nl < MAX_CAND) list[nl++] = cd;
    }
    int64_t cr[8][3]; corners(R, cr);
    for (int c = 0; c < 8; c++) {
        int64_t rel[3], loc[3];
        for (int k = 0; k < 3; k++) rel[k] = cr[c][k] - I->p[k];
        rot_t(I->r, rel, loc);
        if (iabs64(loc[0]) > I->h[0] || iabs64(loc[1]) > I->h[1] || iabs64(loc[2]) > I->h[2]) continue;
        cand_t cd;
        for (int k = 0; k < 3; k++) { cd.n[k] = nab[k]; cd.pt[k] = (int32_t)cr[c][k]; }
        cd.depth = (int32_t)best; cd.feat = (int16_t)(16 + c + (ref ? 8 : 0));
        if (nl < MAX_CAND) list[nl++] = cd;
    }
    const int64_t au[3] = { R->r[u], R->r[3+u], R->r[6+u] }, av[3] = { R->r[v], R->r[3+v], R->r[6+v] };
    return spread(list, nl, au, av, out);
}

static int body_floor(const body_t *a, cand_t *out, int64_t mg)
{
    if (a->shape == SHAPE_SPHERE) {
        const int32_t bottom = a->p[1] - a->h[0];
        if ((int64_t)bottom >= (int64_t)s_floor_y + mg) return 0;
        out->n[0] = 0; out->n[1] = -N1; out->n[2] = 0;          /* body -> floor: down */
        out->depth = s_floor_y - bottom;
        out->pt[0] = a->p[0]; out->pt[1] = s_floor_y; out->pt[2] = a->p[2];
        out->feat = 0;
        return 1;
    }
    cand_t list[MAX_CAND]; int nl = 0;
    int64_t c8[8][3]; corners(a, c8);
    for (int c = 0; c < 8; c++) {
        if (c8[c][1] >= (int64_t)s_floor_y + mg) continue;
        cand_t cd;
        cd.n[0] = 0; cd.n[1] = -N1; cd.n[2] = 0;
        cd.depth = (int32_t)(s_floor_y - c8[c][1]);
        cd.pt[0] = (int32_t)c8[c][0]; cd.pt[1] = (int32_t)c8[c][1]; cd.pt[2] = (int32_t)c8[c][2];
        cd.feat = (int16_t)c;
        list[nl++] = cd;
    }
    const int64_t au[3] = { N1, 0, 0 }, av[3] = { 0, 0, N1 };
    return spread(list, nl, au, av, out);
}

/* How far a body can travel in one substep, as an upper bound: its speed plus
 * its spin times its size. */
static int64_t reach(const body_t *b)
{
    if (b->asleep) return 0;
    const int64_t lin = iabs64(b->v[0]) + iabs64(b->v[1]) + iabs64(b->v[2]);
    const int64_t spin = iabs64(b->w[0]) + iabs64(b->w[1]) + iabs64(b->w[2]);
    const int64_t size = (int64_t)b->h[0] + b->h[1] + b->h[2];
    return lin + ((spin * size) >> WQ);
}

static int collide(int ia, int ib, cand_t *out)
{
    const body_t *a = &s_b[ia], *b = &s_b[ib];
    const int64_t mg = SKIN + reach(a) + reach(b);
    if (a->shape == SHAPE_SPHERE && b->shape == SHAPE_SPHERE) return sphere_sphere(a, b, out, mg);
    if (a->shape == SHAPE_BOX    && b->shape == SHAPE_BOX)    return box_box(a, b, out, mg);
    if (a->shape == SHAPE_SPHERE) return sphere_box(a, b, out, mg);
    /* box a, sphere b: find it the other way round and turn the normal back */
    if (!sphere_box(b, a, out, mg)) return 0;
    for (int k = 0; k < 3; k++) out->n[k] = -out->n[k];
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
static int64_t invi_of(int id)
{
    if (id < 0) return 0;
    const body_t *b = &s_b[id];
    return b->asleep ? 0 : b->inv_i;
}

/* velocity of the point r (from the centre) of body id, Q8 per substep */
static void point_vel(int id, const int32_t r[3], int64_t out[3])
{
    if (id < 0) { out[0] = out[1] = out[2] = 0; return; }
    const body_t *b = &s_b[id];
    const int64_t w[3] = { b->w[0], b->w[1], b->w[2] }, rr[3] = { r[0], r[1], r[2] };
    int64_t wr[3]; cross(w, rr, wr);
    for (int k = 0; k < 3; k++) out[k] = b->v[k] + (wr[k] >> WQ);
}

static int64_t vrel(const contact_t *c, int64_t vr[3])
{
    int64_t va[3], vb[3];
    point_vel(c->a, c->ra, va); point_vel(c->b, c->rb, vb);
    int64_t vn = 0;
    for (int k = 0; k < 3; k++) { vr[k] = vb[k] - va[k]; vn += vr[k] * c->n[k]; }
    return vn >> 14;
}

/* The inverse of the mass a contact "feels" along a unit direction (Q14):
 * both linear masses plus what each body's turning adds, |r × dir|² / I. Q16. */
static int64_t k_along(const contact_t *c, const int64_t dir[3])
{
    int64_t k = (int64_t)inv_of(c->a) + inv_of(c->b);
    const int ids[2] = { c->a, c->b };
    const int32_t *rs[2] = { c->ra, c->rb };
    for (int s = 0; s < 2; s++) {
        const int64_t ii = invi_of(ids[s]);
        if (!ii) continue;
        const int64_t r[3] = { rs[s][0], rs[s][1], rs[s][2] };
        int64_t rx[3]; cross(r, dir, rx);
        int64_t l2 = 0;
        for (int j = 0; j < 3; j++) { const int64_t t = rx[j] >> 14; l2 += t * t; }   /* Q16 units² */
        k += ((l2 >> 16) * ii) >> 16;
    }
    return k;
}

/* an impulse (mass × Q8 velocity, a vector) on a body at r, with a sign */
static void push(int id, const int32_t r[3], const int64_t imp[3], int sign)
{
    if (id < 0) return;
    body_t *b = &s_b[id];
    if (b->asleep || !b->inv_m) return;
    for (int k = 0; k < 3; k++) b->v[k] += (int32_t)(sign * ((imp[k] * b->inv_m) >> 16));
    if (b->inv_i) {
        const int64_t rr[3] = { r[0], r[1], r[2] };
        spin_kick(b, rr, imp, sign);
    }
}

static void solve(contact_t *c)
{
    if (c->kn == 0) return;

    /* NORMAL: drive the approach speed to the target, never pulling (acc >= 0) */
    int64_t vr[3];
    const int64_t vn = vrel(c, vr);
    int64_t dj = ((int64_t)c->target - vn) * 65536 / c->kn;
    int64_t acc = c->jn + dj;
    if (acc < 0) acc = 0;
    dj = acc - c->jn; c->jn = acc;
    if (dj) {
        int64_t imp[3];
        for (int k = 0; k < 3; k++) imp[k] = (c->n[k] * dj) >> 14;
        push(c->a, c->ra, imp, -1); push(c->b, c->rb, imp, +1);
    }

    /* FRICTION, ACCUMULATED IN TWO FIXED TANGENTS AND KEPT INSIDE mu × NORMAL.
     *
     * The total may go DOWN as well as up from one pass to the next. Measured:
     * with a single "how much more" that only ever grew, a box sliding on four
     * corners stopped 30% short of v²/(2µg) — the weight moves between the
     * corners from pass to pass, and friction already given to a corner that
     * ends up lightly loaded was never taken back. */
    if (!c->kt1 || !c->kt2) return;
    vrel(c, vr);
    int64_t v1 = 0, v2 = 0;
    for (int k = 0; k < 3; k++) { v1 += vr[k] * c->t1[k]; v2 += vr[k] * c->t2[k]; }
    v1 >>= 14; v2 >>= 14;
    int64_t n1 = c->jt1 - v1 * 65536 / c->kt1;     /* what would stop the sliding */
    int64_t n2 = c->jt2 - v2 * 65536 / c->kt2;
    const int64_t cap = c->jn * c->fric >> Q;
    const int64_t len = isqrt64(n1 * n1 + n2 * n2);
    if (len > cap) { n1 = len ? n1 * cap / len : 0; n2 = len ? n2 * cap / len : 0; }
    const int64_t d1 = n1 - c->jt1, d2 = n2 - c->jt2;
    c->jt1 = n1; c->jt2 = n2;
    if (d1 | d2) {
        int64_t imp[3];
        for (int k = 0; k < 3; k++) imp[k] = (c->t1[k] * d1 + c->t2[k] * d2) >> 14;
        push(c->a, c->ra, imp, -1); push(c->b, c->rb, imp, +1);
    }
}

static void separate(const contact_t *c)
{
    const int32_t ima = inv_of(c->a), imb = inv_of(c->b);
    const int64_t im = (int64_t)ima + imb;
    if (im == 0 || c->depth <= SLOP) return;
    /* SHARED BETWEEN THE PAIR'S POINTS. A box on four corners otherwise moves out
     * four times as far as it overlapped — measured: a stack of five launched its
     * bottom box upwards and fell over the moment boxes could turn. */
    const int64_t corr = (int64_t)(c->depth - SLOP) * PUSH_NUM / (PUSH_DEN * (c->npair > 0 ? c->npair : 1));
    for (int k = 0; k < 3; k++) {
        const int64_t m = (c->n[k] * corr) >> 14;
        if (c->a >= 0 && ima) s_b[c->a].p[k] -= (int32_t)(m * ima / im);
        if (c->b >= 0 && imb) s_b[c->b].p[k] += (int32_t)(m * imb / im);
    }
}

static int mix_rest(int a, int b) { return a > b ? a : b; }
static int mix_fric(int a, int b) { return (int)isqrt64((int64_t)a * b); }

static void add_contact(int a, int b, const cand_t *cd)
{
    if (s_nc >= VPYP_MAX_CONTACTS) { s_stats.contacts_dropped++; return; }
    contact_t *c = &s_c[s_nc];
    c->a = (int16_t)a; c->b = (int16_t)b; c->feat = cd->feat;
    for (int k = 0; k < 3; k++) {
        c->n[k] = (int16_t)cd->n[k];
        c->ra[k] = cd->pt[k] - s_b[a].p[k];
        c->rb[k] = b >= 0 ? cd->pt[k] - s_b[b].p[k] : 0;
    }
    c->depth = cd->depth;
    c->jn = 0; c->jt1 = 0; c->jt2 = 0;
    for (int i = 0; i < s_nw; i++)
        if (s_w[i].a == a && s_w[i].b == b && s_w[i].feat == cd->feat) { c->jn = s_w[i].jn; break; }
    const body_t *A = &s_b[a];
    if (b == VPYP_FLOOR) { c->rest = (int16_t)mix_rest(A->rest, s_floor_rest); c->fric = (int16_t)mix_fric(A->fric, s_floor_fric); }
    else { c->rest = (int16_t)mix_rest(A->rest, s_b[b].rest); c->fric = (int16_t)mix_fric(A->fric, s_b[b].fric); }
    s_nc++;
    s_b[a].touching++;
    if (b >= 0) s_b[b].touching++;
}

/* Merge one substep's contacts into what the step reports: one entry per pair,
 * the hardest point of the hardest substep, so a crash inside a substep is not
 * lost behind the resting contact that follows it. */
static void report(void)
{
    for (int i = 0; i < s_nc; i++) {
        const contact_t *c = &s_c[i];
        const int32_t imp = (int32_t)(c->jn * s_rate * s_sub / ONE);
        int j = 0;
        while (j < s_nrep && !(s_rep[j].a == c->a && s_rep[j].b == c->b)) j++;
        if (j == s_nrep) {
            if (s_nrep >= MAX_REPORTS) { s_stats.contacts_dropped++; continue; }
            s_nrep++; s_rep[j].impulse = -1;
        }
        vpyp_contact *r = &s_rep[j];
        if (imp <= r->impulse) continue;
        r->a = c->a; r->b = c->b;
        const int32_t *pa = s_b[c->a].p;
        r->x = (pa[0] + c->ra[0]) >> Q; r->y = (pa[1] + c->ra[1]) >> Q; r->z = (pa[2] + c->ra[2]) >> Q;
        r->nx = (int16_t)c->n[0]; r->ny = (int16_t)c->n[1]; r->nz = (int16_t)c->n[2];
        r->depth = c->depth >> Q;
        r->impulse = imp;
    }
}

static void substep(void)
{
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) quat_to_matrix(s_b[i].q, s_b[i].r);

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
        cand_t cd[MAX_PAIR_CONTACTS];
        if (s_floor_on && a->inv_m) {
            const int n = body_floor(a, cd, SKIN + reach(a));
            const int first = s_nc;
            for (int k = 0; k < n; k++) add_contact(i, VPYP_FLOOR, &cd[k]);
            for (int k = first; k < s_nc; k++) s_c[k].npair = (int16_t)(s_nc - first);
        }
        for (int j = i + 1; j < VPYP_MAX_BODIES; j++) {
            const body_t *b = &s_b[j];
            if (!b->alive || !(a->mask & b->mask)) continue;
            if (!a->inv_m && !b->inv_m) continue;             /* two statics */
            s_stats.pairs++;
            const int n = collide(i, j, cd);
            const int first = s_nc;
            for (int k = 0; k < n; k++) add_contact(i, j, &cd[k]);
            for (int k = first; k < s_nc; k++) s_c[k].npair = (int16_t)(s_nc - first);
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
        const int64_t vn = vrel(c, vr);
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

    /* the separation speed each contact asks for, and the mass it feels */
    for (int i = 0; i < s_nc; i++) {
        contact_t *c = &s_c[i];
        int64_t vr[3];
        const int64_t vn = vrel(c, vr);
        /* depth < 0 is a gap inside the skin: the bodies may close it this
         * substep and no more. A bounce only when the approach is real AND
         * would close the gap now. */
        if (vn < -rest_v && vn < c->depth) c->target = (int32_t)(-vn * c->rest >> Q);
        else c->target = c->depth < 0 ? c->depth : 0;
        const int64_t n[3] = { c->n[0], c->n[1], c->n[2] };
        c->kn = (int32_t)k_along(c, n);
        /* a tangent from whichever world axis is least along n, then n × it */
        const int ax = iabs64(n[0]) < iabs64(n[1]) ? (iabs64(n[0]) < iabs64(n[2]) ? 0 : 2)
                                                    : (iabs64(n[1]) < iabs64(n[2]) ? 1 : 2);
        int64_t e[3] = { 0, 0, 0 }; e[ax] = N1;
        int64_t t1[3], t2[3];
        cross(n, e, t1);
        int64_t l = isqrt64(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
        for (int k = 0; k < 3; k++) t1[k] = l ? t1[k] * N1 / l : 0;
        cross(n, t1, t2);
        for (int k = 0; k < 3; k++) t2[k] >>= 14;
        for (int k = 0; k < 3; k++) { c->t1[k] = (int16_t)t1[k]; c->t2[k] = (int16_t)t2[k]; }
        c->kt1 = (int32_t)k_along(c, t1);
        c->kt2 = (int32_t)k_along(c, t2);
    }

    /* 4. impulses: last substep's first, then the passes */
    for (int i = 0; i < s_nc; i++) {
        contact_t *c = &s_c[i];
        if (!c->jn) continue;
        if (!c->kn) { c->jn = 0; continue; }
        int64_t imp[3];
        for (int k = 0; k < 3; k++) imp[k] = (c->n[k] * c->jn) >> 14;
        push(c->a, c->ra, imp, -1); push(c->b, c->rb, imp, +1);
    }
    /* the passes alternate direction: always solving in the same order leans a
     * stack the same way every step, and the lean adds up. Measured on a column
     * of five boxes: 26 units of drift at the top in one direction; 6 with the
     * passes alternating. */
    for (int it = 0; it < VPYP_ITERATIONS; it++) {
        if (it & 1) for (int i = s_nc - 1; i >= 0; i--) solve(&s_c[i]);
        else        for (int i = 0; i < s_nc; i++)      solve(&s_c[i]);
    }
    s_nw = 0;
    for (int i = 0; i < s_nc; i++) {
        s_w[s_nw].a = s_c[i].a; s_w[s_nw].b = s_c[i].b; s_w[s_nw].feat = s_c[i].feat;
        s_w[s_nw].jn = s_c[i].jn; s_nw++;
    }

    /* 5. positions and orientations */
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || b->asleep) continue;
        for (int k = 0; k < 3; k++) b->p[k] += b->v[k];
        if (b->inv_i && (b->w[0] | b->w[1] | b->w[2])) {
            quat_integrate(b->q, b->w);
            for (int k = 0; k < 3; k++) b->w[k] -= b->w[k] >> SPIN_DAMP_SHIFT;
        }
    }

    /* 6. overlap */
    for (int i = 0; i < s_nc; i++) separate(&s_c[i]);

    /* 7. sleep */
    const int64_t sv = speed_in(SLEEP_SPEED), sw = spin_in(SLEEP_SPIN);
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || b->asleep) continue;
        int64_t v2 = 0, w2 = 0;
        for (int k = 0; k < 3; k++) { v2 += (int64_t)b->v[k] * b->v[k]; w2 += (int64_t)b->w[k] * b->w[k]; }
        if (v2 < sv * sv && w2 < sw * sw && b->touching) {
            if (++b->still >= SLEEP_STEPS * s_sub) {
                b->asleep = 1;
                b->v[0] = b->v[1] = b->v[2] = 0;
                b->w[0] = b->w[1] = b->w[2] = 0;
            }
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
    for (int i = 0; i < VPYP_MAX_BODIES; i++)
        if (s_b[i].alive) quat_to_matrix(s_b[i].q, s_b[i].r);
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

/* the slab test in the box's own frame, so a turned box is hit where it is */
static int ray_box(const int64_t o[3], const int32_t u[3], const body_t *b, int64_t *t, int32_t n[3])
{
    int64_t rel[3], ol[3], ul[3];
    const int64_t uw[3] = { u[0], u[1], u[2] };
    for (int k = 0; k < 3; k++) rel[k] = o[k] - b->p[k];
    rot_t(b->r, rel, ol);
    rot_t(b->r, uw, ul);
    int64_t t0 = 0, t1 = (int64_t)1 << 50;
    int axis = -1, sgn = 0;
    for (int k = 0; k < 3; k++) {
        const int64_t lo = -(int64_t)b->h[k], hi = b->h[k];
        if (ul[k] == 0) { if (ol[k] < lo || ol[k] > hi) return 0; continue; }
        int64_t ta = (lo - ol[k]) * N1 / ul[k], tb = (hi - ol[k]) * N1 / ul[k];
        int s = -1;                                           /* entering through the low face */
        if (ta > tb) { const int64_t x = ta; ta = tb; tb = x; s = 1; }
        if (ta > t0) { t0 = ta; axis = k; sgn = s; }
        if (tb < t1) t1 = tb;
        if (t0 > t1) return 0;
    }
    int64_t nl[3] = { 0, 0, 0 }, nw[3];
    if (axis >= 0) nl[axis] = sgn * N1;
    rot(b->r, nl, nw);
    n[0] = (int32_t)nw[0]; n[1] = (int32_t)nw[1]; n[2] = (int32_t)nw[2];
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
