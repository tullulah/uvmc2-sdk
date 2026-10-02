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

enum { SHAPE_SPHERE = 0, SHAPE_BOX = 1, SHAPE_HULL = 2 };

typedef struct {
    uint8_t  alive, shape, asleep, mask;
    uint8_t  hull;                /* SHAPE_HULL: which registered shape */
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

/* ── convex hulls ──────────────────────────────────────────────────────────── */
/* A hull SHAPE is registered once and shared by every body made from it. It is
 * kept in the body's own frame: vertices Q8, face planes (n Q14 outward,
 * n·x <= d inside, d Q8), each face's polygon as vertex indices, the edges, and
 * the edge DIRECTIONS with parallel ones merged — a cube has twelve edges but
 * three directions, and it is the directions the separating-axis test crosses. */
/* how far a face's corner may sit off the face's plane, Q8: two units, for
 * meshes whose corners were rounded to whole units */
#define HULL_FLAT (2 * ONE)
typedef struct {
    uint8_t used, nv, nf, ne, nd, nidx;
    int32_t v[VPYP_HULL_MAX_VERTS][3];
    int16_t fn[VPYP_HULL_MAX_FACES][3];
    int32_t fd[VPYP_HULL_MAX_FACES];
    uint8_t fstart[VPYP_HULL_MAX_FACES], fcount[VPYP_HULL_MAX_FACES];
    uint8_t fidx[VPYP_HULL_MAX_INDICES];
    uint8_t e[VPYP_HULL_MAX_EDGES][2];
    int16_t dir[VPYP_HULL_MAX_EDGES][3];
    int32_t ext[3];               /* how far it reaches along each local axis, Q8 */
    int64_t r2;                   /* mean squared distance of the vertices, units² */
} hull_t;
static hull_t s_hull[VPYP_MAX_HULLS];
static int    s_hull_error;

/* joints: see the joints section */
#define JOINT_BETA_DEN 5          /* a fifth of the error back per substep */
typedef struct {
    uint8_t used, hinge;
    int16_t a, b;                 /* b = VPYP_NONE: the world */
    int32_t la[3], lb[3];         /* the point in each body's frame, Q8 (lb: world, for the world) */
    int16_t xa[3], xb[3];         /* the hinge axis in each body's frame, Q14 */
    int32_t ra[3], rb[3];         /* this substep: from each centre to the point, world Q8 */
    int32_t tgt[3];               /* Q8/substep along each world axis */
    int64_t adj[9], det;          /* the point's 3x3 effective mass, inverted: adj / det */
    int     ksh;                  /* ...of the matrix scaled down by this many bits */
    int16_t t1[3], t2[3];         /* across the axis, Q14 */
    int32_t ta1, ta2;             /* relative spin asked for along t1, t2, Q20 */
    int64_t ka;                   /* inverse inertia felt, Q32 */
    int64_t jl[3], ja[3];         /* accumulated linear and angular impulses, world */
} joint_t;
static joint_t s_j[VPYP_MAX_JOINTS];
static uint8_t s_njoint[VPYP_MAX_BODIES];   /* joints on each body: pairs with none skip the search */

int vpyp_hull_error(void) { return s_hull_error; }

static int hull_refuse(int why) { s_hull_error = why; s_stats.shapes_refused++; return VPYP_NONE; }

/* the direction from a to b as a Q14 unit vector; 0 if they coincide */
static int unit_q14(const int64_t d[3], int16_t out[3])
{
    const int64_t l = isqrt64(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (l == 0) return 0;
    for (int k = 0; k < 3; k++) out[k] = (int16_t)(d[k] * N1 / l);
    return 1;
}

int vpyp_hull_shape(const int16_t *xyz, int nverts, const uint8_t *faces)
{
    int slot = 0;
    while (slot < VPYP_MAX_HULLS && s_hull[slot].used) slot++;
    if (slot == VPYP_MAX_HULLS) return hull_refuse(VPYP_HULL_TABLE_FULL);
    if (nverts < 4 || nverts > VPYP_HULL_MAX_VERTS) return hull_refuse(VPYP_HULL_TOO_BIG);
    hull_t *H = &s_hull[slot];
    H->nv = (uint8_t)nverts; H->nf = 0; H->ne = 0; H->nd = 0; H->nidx = 0;
    H->ext[0] = H->ext[1] = H->ext[2] = 0; H->r2 = 0;
    for (int i = 0; i < nverts; i++) {
        for (int k = 0; k < 3; k++) {
            H->v[i][k] = xyz[i * 3 + k] * ONE;
            const int32_t a = H->v[i][k] < 0 ? -H->v[i][k] : H->v[i][k];
            if (a > H->ext[k]) H->ext[k] = a;
        }
        H->r2 += (int64_t)xyz[i*3] * xyz[i*3] + (int64_t)xyz[i*3+1] * xyz[i*3+1] + (int64_t)xyz[i*3+2] * xyz[i*3+2];
    }
    H->r2 /= nverts;
    /* the faces: count, then that many vertex indices; a count of 0 ends the list */
    for (const uint8_t *f = faces; *f; f += 1 + *f) {
        const int n = *f;
        if (n < 3) return hull_refuse(VPYP_HULL_BAD_FACE);
        if (H->nf >= VPYP_HULL_MAX_FACES || H->nidx + n > VPYP_HULL_MAX_INDICES) return hull_refuse(VPYP_HULL_TOO_BIG);
        for (int j = 0; j < n; j++) if (f[1 + j] >= nverts) return hull_refuse(VPYP_HULL_BAD_FACE);
        /* the normal by Newell's sum: every edge of the polygon has its say, so
         * a face whose corners are not quite coplanar still gets one normal */
        int64_t nn[3] = { 0, 0, 0 }, cen[3] = { 0, 0, 0 };
        for (int j = 0; j < n; j++) {
            const int16_t *p = &xyz[f[1 + j] * 3], *q = &xyz[f[1 + (j + 1) % n] * 3];
            nn[0] += (int64_t)(p[1] - q[1]) * (p[2] + q[2]);
            nn[1] += (int64_t)(p[2] - q[2]) * (p[0] + q[0]);
            nn[2] += (int64_t)(p[0] - q[0]) * (p[1] + q[1]);
            for (int k = 0; k < 3; k++) cen[k] += p[k];
        }
        int16_t un[3];
        if (!unit_q14(nn, un)) return hull_refuse(VPYP_HULL_BAD_FACE);
        /* OUTWARD whichever way the face was wound: away from the vertices' mean */
        int64_t mean[3] = { 0, 0, 0 };
        for (int i = 0; i < nverts; i++) for (int k = 0; k < 3; k++) mean[k] += xyz[i * 3 + k];
        int64_t side = 0;
        for (int k = 0; k < 3; k++) side += (cen[k] * nverts - mean[k] * n) * un[k];
        if (side < 0) for (int k = 0; k < 3; k++) un[k] = (int16_t)-un[k];
        const int fi = H->nf++;
        for (int k = 0; k < 3; k++) H->fn[fi][k] = un[k];
        /* the plane through the face's furthest corner, so every corner is inside */
        int64_t d = INT64_MIN;
        for (int j = 0; j < n; j++) {
            const int32_t *p = H->v[f[1 + j]];
            const int64_t pd = ((int64_t)un[0] * p[0] + (int64_t)un[1] * p[1] + (int64_t)un[2] * p[2]) >> 14;
            if (pd > d) d = pd;
        }
        H->fd[fi] = (int32_t)d;
        /* FLAT: every corner of the face on its plane. Without this a corner
         * pushed into the solid passed as convex — each plane goes through its
         * face's furthest corner, so a sunken one is always "behind" it. */
        for (int j = 0; j < n; j++) {
            const int32_t *p = H->v[f[1 + j]];
            const int64_t pd = ((int64_t)un[0] * p[0] + (int64_t)un[1] * p[1] + (int64_t)un[2] * p[2]) >> 14;
            if (d - pd > HULL_FLAT) return hull_refuse(VPYP_HULL_NOT_FLAT);
        }
        H->fstart[fi] = H->nidx; H->fcount[fi] = (uint8_t)n;
        for (int j = 0; j < n; j++) H->fidx[H->nidx++] = f[1 + j];
        /* its edges, each once */
        for (int j = 0; j < n; j++) {
            const uint8_t a = f[1 + j], b = f[1 + (j + 1) % n];
            int seen = 0;
            for (int e = 0; e < H->ne && !seen; e++)
                if ((H->e[e][0] == a && H->e[e][1] == b) || (H->e[e][0] == b && H->e[e][1] == a)) seen = 1;
            if (seen) continue;
            if (H->ne >= VPYP_HULL_MAX_EDGES) return hull_refuse(VPYP_HULL_TOO_BIG);
            H->e[H->ne][0] = a; H->e[H->ne][1] = b; H->ne++;
            const int64_t dd[3] = { xyz[b*3] - xyz[a*3], xyz[b*3+1] - xyz[a*3+1], xyz[b*3+2] - xyz[a*3+2] };
            int16_t ud[3];
            if (!unit_q14(dd, ud)) return hull_refuse(VPYP_HULL_BAD_FACE);
            int par = 0;
            for (int q = 0; q < H->nd && !par; q++) {
                const int64_t dot = ((int64_t)ud[0] * H->dir[q][0] + (int64_t)ud[1] * H->dir[q][1] + (int64_t)ud[2] * H->dir[q][2]) >> 14;
                if (iabs64(dot) > N1 - N1 / 8192) par = 1;     /* within ~1 degree */
            }
            if (!par) { for (int k = 0; k < 3; k++) H->dir[H->nd][k] = ud[k]; H->nd++; }
        }
    }
    if (H->nf < 4) return hull_refuse(VPYP_HULL_BAD_FACE);
    /* CONVEX, AND THE ORIGIN INSIDE: every vertex on or behind every face (to a
     * unit), and the body's centre — the point it turns about — in the solid */
    for (int fi = 0; fi < H->nf; fi++) {
        if (H->fd[fi] < 0) return hull_refuse(VPYP_HULL_ORIGIN_OUTSIDE);
        for (int i = 0; i < nverts; i++) {
            const int64_t pd = ((int64_t)H->fn[fi][0] * H->v[i][0] + (int64_t)H->fn[fi][1] * H->v[i][1] + (int64_t)H->fn[fi][2] * H->v[i][2]) >> 14;
            if (pd > H->fd[fi] + ONE) return hull_refuse(VPYP_HULL_NOT_CONVEX);
        }
    }
    H->used = 1;
    s_hull_error = VPYP_HULL_OK;
    return slot;
}

/* ── the world ─────────────────────────────────────────────────────────────── */
void vpyp_reset(void)
{
    for (int i = 0; i < VPYP_MAX_BODIES; i++) s_b[i].alive = 0;
    for (int i = 0; i < VPYP_MAX_HULLS; i++) s_hull[i].used = 0;
    for (int i = 0; i < VPYP_MAX_JOINTS; i++) s_j[i].used = 0;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) s_njoint[i] = 0;
    s_hull_error = VPYP_HULL_OK;
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
static int64_t inverse_inertia(int shape, int32_t mass, int32_t hx, int32_t hy, int32_t hz, int64_t r2)
{
    if (mass <= 0) return 0;
    int64_t i;
    if (shape == SHAPE_SPHERE) i = (int64_t)2 * mass * hx * hx / 5;
    else if (shape == SHAPE_HULL) i = (int64_t)2 * mass * r2 / 9;   /* see vpyp_add_hull */
    else i = (int64_t)2 * mass * ((int64_t)hx * hx + (int64_t)hy * hy + (int64_t)hz * hz) / 9;
    if (i <= 0) i = 1;
    const int64_t inv = (((int64_t)1 << 32) + i / 2) / i;
    return inv > 0 ? inv : 1;
}

static int add(int shape, int32_t x, int32_t y, int32_t z,
               int32_t hx, int32_t hy, int32_t hz, int32_t mass, int64_t r2)
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
        b->inv_i_free = inverse_inertia(shape, mass, hx, hy, hz, r2);
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
    return add(SHAPE_SPHERE, x, y, z, r, r, r, mass, 0);
}
int vpyp_add_box(int32_t x, int32_t y, int32_t z,
                 int32_t hx, int32_t hy, int32_t hz, int32_t mass)
{
    return add(SHAPE_BOX, x, y, z, hx, hy, hz, mass, 0);
}
/* THE INERTIA OF A HULL is the box formula's, 2/9 × mass × the mean squared
 * distance of the corners from the centre — the same number a box gets from
 * its eight corners, so a hull made as a cube turns exactly like the box. For
 * other shapes it is an estimate from the corners, like the scalar inertia
 * itself. */
int vpyp_add_hull(int32_t x, int32_t y, int32_t z, int shape, int32_t mass)
{
    if (shape < 0 || shape >= VPYP_MAX_HULLS || !s_hull[shape].used) { s_stats.refused++; return VPYP_NONE; }
    const hull_t *H = &s_hull[shape];
    const int id = add(SHAPE_HULL, x, y, z, H->ext[0] >> Q, H->ext[1] >> Q, H->ext[2] >> Q, mass, H->r2);
    if (id >= 0) s_b[id].hull = (uint8_t)shape;
    return id;
}
static void wake(body_t *b);
void vpyp_remove(int id)
{
    body_t *b = get(id); if (!b) return;
    /* ITS JOINTS GO WITH IT, and what they held wakes */
    for (int j = 0; j < VPYP_MAX_JOINTS; j++)
        if (s_j[j].used && (s_j[j].a == id || s_j[j].b == id)) vpyp_joint_remove(j);
    b->alive = 0;
    /* EVERYTHING THAT TOUCHED IT WAKES, and the solver decides what still
     * stands. Measured: shoot the middle crate out of the bottom row of a
     * sleeping pyramid and the two crates resting half on it stayed where they
     * were, asleep over a gap — each still had SOME support, which is all the
     * "held up" rule asks. Then its warm impulses must not be handed to
     * whatever takes the slot next. */
    for (int i = 0; i < s_nw; i++) {
        if (s_w[i].a != id && s_w[i].b != id) continue;
        const int other = s_w[i].a == id ? s_w[i].b : s_w[i].a;
        if (other >= 0 && s_b[other].alive) wake(&s_b[other]);
        s_w[i].a = s_w[i].b = VPYP_NONE;
    }
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
    /* THE IMPULSE IS ALREADY mass × units/s: in internal units it is speed_in(i),
     * and NOT that times the mass again. Measured: it was, and a body of mass 4
     * kicked at its edge spun four times too fast — mass 1, which every test
     * used until the joints came, hid it. */
    const int64_t imp[3] = { speed_in(ix), speed_in(iy), speed_in(iz) };
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
#define MAX_CAND 32
/* THE CANDIDATE LISTS ARE STATIC, NOT ON THE STACK. A list is 32 × 32 bytes, and box against
 * box held three of them at once — box_box's, spread's, and body_floor's inlined into the
 * step: measured with -fstack-usage, substep 1752 + box_box 1808 + spread 1168 bytes, ~4.7 KB
 * on one call path. A .um2's core 0 has 4 KB of stack (SCRATCH_Y) with core 1's — the one
 * replaying the list — right below it, so physics_demo crashed the UVMC2 the day MAX_CAND
 * went from 16 to 32, while it ran on the debug cartridge, whose game core has another stack.
 * The physics is not re-entrant, and each function keeps its own list, so static is safe. */
static int spread(cand_t *list, int n, const int64_t u[3], const int64_t v[3], cand_t *out)
{
    static cand_t uniq[MAX_CAND]; int m = 0;
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

/* The point where two edges come closest: edge 1 through c1 along unit d1
 * (Q14), half length h1, and edge 2 likewise; the midpoint of the two closest
 * points, Q8, into `mid`. Lines, then clamped to the edges' lengths. */
static void edge_edge_point(const int64_t c1[3], const int64_t d1[3], int64_t h1,
                            const int64_t c2[3], const int64_t d2[3], int64_t h2, int64_t mid[3])
{
    int64_t r[3];
    for (int k = 0; k < 3; k++) r[k] = c1[k] - c2[k];
    const int64_t b = (d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]) >> 14;     /* Q14 */
    const int64_t c = (d1[0] * r[0] + d1[1] * r[1] + d1[2] * r[2]) >> 14;        /* Q8 */
    const int64_t f = (d2[0] * r[0] + d2[1] * r[1] + d2[2] * r[2]) >> 14;
    const int64_t den = (int64_t)N1 * N1 - b * b;                                /* Q28 */
    int64_t s = den > 0 ? (b * f - c * N1) * N1 / den : 0;                       /* Q8 */
    s = s < -h1 ? -h1 : (s > h1 ? h1 : s);
    int64_t t = ((b * s) >> 14) + f;
    t = t < -h2 ? -h2 : (t > h2 ? h2 : t);
    for (int k = 0; k < 3; k++)
        mid[k] = ((c1[k] + ((d1[k] * s) >> 14)) + (c2[k] + ((d2[k] * t) >> 14))) / 2;
}

/* AN EDGE AXIS HAS TO BEAT THE BEST FACE BY THIS MUCH (Q8) to be used. Two boxes
 * resting face to face also have edge axes — the cross of two parallel-ish face
 * axes is another face axis — with the same overlap, and a pair that flipped
 * between a face (four points) and an edge (one) would rock. */
#define EDGE_BIAS ONE
/* Below this length (Q14) the cross of two edges is no direction at all: the
 * edges are parallel to within ~1 degree, and a face axis covers the pair. */
#define EDGE_PARALLEL (N1 / 64)

/* BOX AGAINST BOX, by the separating-axis test: six face axes and the nine
 * edge-against-edge ones.
 *
 * When a face axis has the least overlap it names a REFERENCE face; the contact
 * points are the other box's corners that lie under that face and within its
 * extent, plus the reference box's own corners that lie inside the other box —
 * together that is the contact polygon of two faces resting on each other,
 * which is what makes a box sit flat on another instead of on a point.
 *
 * When an EDGE axis wins, the boxes meet edge across edge, and there is one
 * contact: where the two edges come closest. Measured with the face axes alone: a
 * box dropped crosswise onto another's ridge, both turned 45 degrees, found no
 * corner under any face and fell straight through it (tools/phys_check.c, 13);
 * with the edge axes it rests on the ridge within a unit. */
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
    /* the nine edge axes: a's edge i across b's edge j */
    int ei = -1, ej = -1; int64_t ebest = 0, eax[3] = { 0, 0, 0 };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        const int64_t ea[3] = { a->r[i], a->r[3+i], a->r[6+i] }, eb[3] = { b->r[j], b->r[3+j], b->r[6+j] };
        int64_t cx[3]; cross(ea, eb, cx);                                      /* Q28 */
        const int64_t l = isqrt64(((cx[0] >> 2) * (cx[0] >> 2) + (cx[1] >> 2) * (cx[1] >> 2) + (cx[2] >> 2) * (cx[2] >> 2))) << 2;
        if (l < (int64_t)EDGE_PARALLEL * N1) continue;
        int64_t ax[3];
        for (int k = 0; k < 3; k++) ax[k] = cx[k] * N1 / l;                    /* Q14 unit */
        const int64_t dist = iabs64((d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]) >> 14);
        const int64_t ov = box_radius(a, ax) + box_radius(b, ax) - dist;
        if (ov <= -mg) return 0;                                               /* separated */
        if (ei < 0 || ov < ebest) { ebest = ov; ei = i; ej = j; eax[0] = ax[0]; eax[1] = ax[1]; eax[2] = ax[2]; }
    }
    if (ei >= 0 && ebest < best - EDGE_BIAS) {
        /* the normal from a to b */
        const int64_t sd = d[0] * eax[0] + d[1] * eax[1] + d[2] * eax[2];
        int64_t n[3];
        for (int k = 0; k < 3; k++) n[k] = sd >= 0 ? eax[k] : -eax[k];
        /* each box's edge that reaches furthest towards the other: from the
         * centre, out along every other axis on the side the normal says */
        int64_t ca[3], cb[3];
        for (int k = 0; k < 3; k++) { ca[k] = a->p[k]; cb[k] = b->p[k]; }
        for (int m = 0; m < 3; m++) {
            if (m != ei) {
                const int64_t col[3] = { a->r[m], a->r[3+m], a->r[6+m] };
                const int64_t sg = (col[0] * n[0] + col[1] * n[1] + col[2] * n[2]) >= 0 ? 1 : -1;
                for (int k = 0; k < 3; k++) ca[k] += (sg * col[k] * a->h[m]) >> 14;
            }
            if (m != ej) {
                const int64_t col[3] = { b->r[m], b->r[3+m], b->r[6+m] };
                const int64_t sg = (col[0] * n[0] + col[1] * n[1] + col[2] * n[2]) >= 0 ? -1 : 1;
                for (int k = 0; k < 3; k++) cb[k] += (sg * col[k] * b->h[m]) >> 14;
            }
        }
        const int64_t da[3] = { a->r[ei], a->r[3+ei], a->r[6+ei] }, db[3] = { b->r[ej], b->r[3+ej], b->r[6+ej] };
        int64_t mid[3];
        edge_edge_point(ca, da, a->h[ei], cb, db, b->h[ej], mid);
        for (int k = 0; k < 3; k++) { out[0].n[k] = (int32_t)n[k]; out[0].pt[k] = (int32_t)mid[k]; }
        out[0].depth = (int32_t)ebest;
        out[0].feat = (int16_t)(48 + ei * 3 + ej);
        return 1;
    }
    const body_t *R = ref ? b : a, *I = ref ? a : b;
    /* the reference normal, pointing from R towards I */
    const int64_t s = (d[0] * bax[0] + d[1] * bax[1] + d[2] * bax[2]) * (ref ? -1 : 1);
    int64_t n[3];
    for (int k = 0; k < 3; k++) n[k] = s >= 0 ? bax[k] : -bax[k];
    /* the normal of the contact goes from a to b */
    int32_t nab[3];
    for (int k = 0; k < 3; k++) nab[k] = (int32_t)(ref ? -n[k] : n[k]);

    static cand_t list[MAX_CAND]; int nl = 0;
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

/* A CONVEX BODY SEEN FROM THE WORLD: a hull, or a box put in the same form, so
 * one separating-axis test serves hull against hull and hull against box. */
#define PV VPYP_HULL_MAX_VERTS
#define PF VPYP_HULL_MAX_FACES
typedef struct {
    int nv, nf, ne, nd;
    int64_t v[PV][3];             /* Q8 world */
    int64_t fn[PF][3];            /* Q14 world, outward */
    int64_t fd[PF];               /* Q8: n·x <= d inside */
    const uint8_t *fstart, *fcount, *fidx;
    const uint8_t (*e)[2];
    int64_t dir[VPYP_HULL_MAX_EDGES][3];   /* Q14 world */
} poly_t;

/* a box's faces as polygons over its corners (bit 0 +x, bit 1 +y, bit 2 +z),
 * each wound round its face; the winding does not matter to what uses it */
static const uint8_t BOX_FIDX[24] = { 0,2,6,4,  1,3,7,5,  0,1,5,4,  2,3,7,6,  0,1,3,2,  4,5,7,6 };
static const uint8_t BOX_FSTART[6] = { 0, 4, 8, 12, 16, 20 }, BOX_FCOUNT[6] = { 4, 4, 4, 4, 4, 4 };
static const uint8_t BOX_E[12][2] = { {0,1},{2,3},{4,5},{6,7}, {0,2},{1,3},{4,6},{5,7}, {0,4},{1,5},{2,6},{3,7} };

static void poly_of(const body_t *b, poly_t *P)
{
    if (b->shape == SHAPE_BOX) {
        P->nv = 8; P->nf = 6; P->ne = 12; P->nd = 3;
        corners(b, P->v);
        for (int f = 0; f < 6; f++) {
            const int k = f >> 1, sg = (f & 1) ? 1 : -1;
            for (int j = 0; j < 3; j++) P->fn[f][j] = sg * b->r[j * 3 + k];
            P->fd[f] = ((P->fn[f][0] * b->p[0] + P->fn[f][1] * b->p[1] + P->fn[f][2] * b->p[2]) >> 14) + b->h[k];
        }
        for (int k = 0; k < 3; k++) for (int j = 0; j < 3; j++) P->dir[k][j] = b->r[j * 3 + k];
        P->fstart = BOX_FSTART; P->fcount = BOX_FCOUNT; P->fidx = BOX_FIDX; P->e = BOX_E;
        return;
    }
    const hull_t *H = &s_hull[b->hull];
    P->nv = H->nv; P->nf = H->nf; P->ne = H->ne; P->nd = H->nd;
    for (int i = 0; i < H->nv; i++) {
        const int64_t l[3] = { H->v[i][0], H->v[i][1], H->v[i][2] };
        int64_t w[3]; rot(b->r, l, w);
        for (int k = 0; k < 3; k++) P->v[i][k] = b->p[k] + w[k];
    }
    for (int f = 0; f < H->nf; f++) {
        const int64_t l[3] = { H->fn[f][0], H->fn[f][1], H->fn[f][2] };
        rot(b->r, l, P->fn[f]);
        P->fd[f] = H->fd[f] + ((P->fn[f][0] * b->p[0] + P->fn[f][1] * b->p[1] + P->fn[f][2] * b->p[2]) >> 14);
    }
    for (int d = 0; d < H->nd; d++) {
        const int64_t l[3] = { H->dir[d][0], H->dir[d][1], H->dir[d][2] };
        rot(b->r, l, P->dir[d]);
    }
    P->fstart = H->fstart; P->fcount = H->fcount; P->fidx = H->fidx; P->e = H->e;
}

static int64_t dot14(const int64_t a[3], const int64_t b[3]) { return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) >> 14; }
static void poly_span(const poly_t *P, const int64_t ax[3], int64_t *lo, int64_t *hi)
{
    *lo = *hi = dot14(P->v[0], ax);
    for (int i = 1; i < P->nv; i++) { const int64_t t = dot14(P->v[i], ax); if (t < *lo) *lo = t; if (t > *hi) *hi = t; }
}
/* inside a face's prism: on the inner side of every side plane through its edges, to mg */
static int in_face(const poly_t *P, int f, const int64_t x[3], int64_t mg)
{
    const int n = P->fcount[f]; const uint8_t *ix = &P->fidx[P->fstart[f]];
    int64_t cen[3] = { 0, 0, 0 };
    for (int j = 0; j < n; j++) for (int k = 0; k < 3; k++) cen[k] += P->v[ix[j]][k];
    for (int k = 0; k < 3; k++) cen[k] /= n;
    for (int j = 0; j < n; j++) {
        const int64_t *p = P->v[ix[j]], *q = P->v[ix[(j + 1) % n]];
        const int64_t ed[3] = { q[0] - p[0], q[1] - p[1], q[2] - p[2] };
        int64_t sn[3]; cross(ed, P->fn[f], sn);                          /* Q8·Q14 */
        int64_t sl = isqrt64((sn[0] >> 8) * (sn[0] >> 8) + (sn[1] >> 8) * (sn[1] >> 8) + (sn[2] >> 8) * (sn[2] >> 8));
        if (sl == 0) continue;
        int64_t u[3];
        for (int k = 0; k < 3; k++) u[k] = (sn[k] >> 8) * N1 / sl;         /* Q14 unit */
        const int64_t rc[3] = { cen[0] - p[0], cen[1] - p[1], cen[2] - p[2] };
        if (dot14(rc, u) > 0) for (int k = 0; k < 3; k++) u[k] = -u[k];    /* outward of the face */
        const int64_t rx[3] = { x[0] - p[0], x[1] - p[1], x[2] - p[2] };
        if (dot14(rx, u) > mg) return 0;
    }
    return 1;
}
static int inside_poly(const poly_t *P, const int64_t x[3])
{
    for (int f = 0; f < P->nf; f++) if (dot14(P->fn[f], x) > P->fd[f]) return 0;
    return 1;
}

/* CONVEX AGAINST CONVEX: the box test above, for any two convex bodies — every
 * face of both, and every pair of edge directions. The contact is built the
 * same way: a reference face and what of the other body lies under it, or one
 * point where two edges cross. Costs (faces + faces + dirs × dirs) projections of
 * every vertex: a 12-vertex hull against a box is ~1500 multiplies a substep. */
static poly_t s_pa, s_pb;
static int poly_poly(const body_t *a, const body_t *b, cand_t *out, int64_t mg)
{
    poly_t *A = &s_pa, *B = &s_pb;
    poly_of(a, A); poly_of(b, B);
    int64_t d[3];
    for (int k = 0; k < 3; k++) d[k] = (int64_t)b->p[k] - a->p[k];
    int ref = -1, rf = 0; int64_t best = 0;
    for (int s = 0; s < 2; s++) {
        const poly_t *R = s ? B : A, *I = s ? A : B;
        for (int f = 0; f < R->nf; f++) {
            int64_t lo, hi; poly_span(I, R->fn[f], &lo, &hi);
            const int64_t ov = R->fd[f] - lo;
            if (ov <= -mg) return 0;
            if (ref < 0 || ov < best - (s ? 16 : 0)) { best = ov; ref = s; rf = f; }
        }
    }
    int ei = -1, ej = -1; int64_t ebest = 0, eax[3] = { 0, 0, 0 };
    for (int i = 0; i < A->nd; i++) for (int j = 0; j < B->nd; j++) {
        int64_t cx[3]; cross(A->dir[i], B->dir[j], cx);
        const int64_t l = isqrt64(((cx[0] >> 2) * (cx[0] >> 2) + (cx[1] >> 2) * (cx[1] >> 2) + (cx[2] >> 2) * (cx[2] >> 2))) << 2;
        if (l < (int64_t)EDGE_PARALLEL * N1) continue;
        int64_t ax[3];
        for (int k = 0; k < 3; k++) ax[k] = cx[k] * N1 / l;
        if (dot14(d, ax) < 0) for (int k = 0; k < 3; k++) ax[k] = -ax[k];   /* from a to b */
        int64_t alo, ahi, blo, bhi; poly_span(A, ax, &alo, &ahi); poly_span(B, ax, &blo, &bhi);
        const int64_t ov = ahi - blo;
        if (ov <= -mg) return 0;
        if (ei < 0 || ov < ebest) { ebest = ov; ei = i; ej = j; for (int k = 0; k < 3; k++) eax[k] = ax[k]; }
    }
    if (ei >= 0 && ebest < best - EDGE_BIAS) {
        /* the edge of each body along those directions that reaches furthest
         * towards the other */
        int ea = -1, eb = -1; int64_t ka = 0, kb = 0;
        for (int e = 0; e < A->ne; e++) {
            const int64_t *p = A->v[A->e[e][0]], *q = A->v[A->e[e][1]];
            const int64_t ed[3] = { q[0] - p[0], q[1] - p[1], q[2] - p[2] };
            int64_t cx[3]; cross(ed, A->dir[ei], cx);
            if (iabs64(cx[0]) + iabs64(cx[1]) + iabs64(cx[2]) > (iabs64(ed[0]) + iabs64(ed[1]) + iabs64(ed[2])) * (N1 / 64)) continue;
            const int64_t mid[3] = { p[0] + q[0], p[1] + q[1], p[2] + q[2] };
            const int64_t key = dot14(mid, eax);
            if (ea < 0 || key > ka) { ka = key; ea = e; }
        }
        for (int e = 0; e < B->ne; e++) {
            const int64_t *p = B->v[B->e[e][0]], *q = B->v[B->e[e][1]];
            const int64_t ed[3] = { q[0] - p[0], q[1] - p[1], q[2] - p[2] };
            int64_t cx[3]; cross(ed, B->dir[ej], cx);
            if (iabs64(cx[0]) + iabs64(cx[1]) + iabs64(cx[2]) > (iabs64(ed[0]) + iabs64(ed[1]) + iabs64(ed[2])) * (N1 / 64)) continue;
            const int64_t mid[3] = { p[0] + q[0], p[1] + q[1], p[2] + q[2] };
            const int64_t key = -dot14(mid, eax);
            if (eb < 0 || key > kb) { kb = key; eb = e; }
        }
        if (ea >= 0 && eb >= 0) {
            int64_t ca[3], cb[3], da[3], db[3];
            const int64_t *pa = A->v[A->e[ea][0]], *qa = A->v[A->e[ea][1]];
            const int64_t *pb = B->v[B->e[eb][0]], *qb = B->v[B->e[eb][1]];
            int64_t la2 = 0, lb2 = 0;
            for (int k = 0; k < 3; k++) {
                ca[k] = (pa[k] + qa[k]) / 2; cb[k] = (pb[k] + qb[k]) / 2;
                da[k] = qa[k] - pa[k]; db[k] = qb[k] - pb[k];
                la2 += da[k] * da[k]; lb2 += db[k] * db[k];
            }
            const int64_t la = isqrt64(la2), lb = isqrt64(lb2);
            for (int k = 0; k < 3; k++) { da[k] = la ? da[k] * N1 / la : 0; db[k] = lb ? db[k] * N1 / lb : 0; }
            int64_t mid[3];
            edge_edge_point(ca, da, la / 2, cb, db, lb / 2, mid);
            for (int k = 0; k < 3; k++) { out[0].n[k] = (int32_t)eax[k]; out[0].pt[k] = (int32_t)mid[k]; }
            out[0].depth = (int32_t)ebest;
            out[0].feat = (int16_t)(512 + ea * 64 + eb);
            return 1;
        }
    }
    const poly_t *R = ref ? B : A, *I = ref ? A : B;
    const int64_t *n = R->fn[rf];                          /* out of R, towards I */
    int32_t nab[3];
    for (int k = 0; k < 3; k++) nab[k] = (int32_t)(ref ? -n[k] : n[k]);
    static cand_t list[MAX_CAND]; int nl = 0;
    for (int i = 0; i < I->nv && nl < MAX_CAND; i++) {
        const int64_t depth = R->fd[rf] - dot14(I->v[i], n);
        if (depth <= -mg || !in_face(R, rf, I->v[i], mg)) continue;
        cand_t cd;
        for (int k = 0; k < 3; k++) { cd.n[k] = nab[k]; cd.pt[k] = (int32_t)I->v[i][k]; }
        cd.depth = (int32_t)depth; cd.feat = (int16_t)(i + (ref ? 64 : 0));
        list[nl++] = cd;
    }
    const uint8_t *ix = &R->fidx[R->fstart[rf]];
    for (int j = 0; j < R->fcount[rf] && nl < MAX_CAND; j++) {
        if (!inside_poly(I, R->v[ix[j]])) continue;
        cand_t cd;
        for (int k = 0; k < 3; k++) { cd.n[k] = nab[k]; cd.pt[k] = (int32_t)R->v[ix[j]][k]; }
        cd.depth = (int32_t)best; cd.feat = (int16_t)(128 + ix[j] + (ref ? 64 : 0));
        list[nl++] = cd;
    }
    /* two directions across the reference face */
    const int64_t *p0 = R->v[ix[0]], *p1 = R->v[ix[1]];
    const int64_t e0[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    const int64_t el = isqrt64(e0[0] * e0[0] + e0[1] * e0[1] + e0[2] * e0[2]);
    int64_t au[3], av[3];
    for (int k = 0; k < 3; k++) au[k] = el ? e0[k] * N1 / el : 0;
    cross(n, au, av);
    for (int k = 0; k < 3; k++) av[k] >>= 14;
    return spread(list, nl, au, av, out);
}

/* SPHERE AGAINST HULL, exactly: the nearest point of the hull's surface to the
 * centre is on a face (its projection lies inside it), an edge or a corner —
 * each looked at — unless the centre is inside, when it leaves through the
 * face it is closest to. The normal points from the sphere to the hull. */
static int sphere_hull(const body_t *s, const body_t *h, cand_t *c, int64_t mg)
{
    poly_t *P = &s_pa; poly_of(h, P);
    const int64_t x[3] = { s->p[0], s->p[1], s->p[2] };
    const int64_t r = s->h[0];
    int bf = 0; int64_t bs = INT64_MIN;
    for (int f = 0; f < P->nf; f++) { const int64_t sd = dot14(P->fn[f], x) - P->fd[f]; if (sd > bs) { bs = sd; bf = f; } }
    if (bs <= 0) {
        /* THE CENTRE IS INSIDE: out through the nearest face */
        for (int k = 0; k < 3; k++) { c->n[k] = (int32_t)-P->fn[bf][k]; c->pt[k] = s->p[k]; }
        c->depth = (int32_t)(r - bs);
        c->feat = 0;
        return 1;
    }
    if (bs >= r + mg) return 0;                            /* beyond a face plane: apart */
    int64_t best[3]; int64_t bd2 = -1;
    for (int f = 0; f < P->nf; f++) {
        const int64_t sd = dot14(P->fn[f], x) - P->fd[f];
        if (sd <= 0) continue;
        int64_t q[3];
        for (int k = 0; k < 3; k++) q[k] = x[k] - ((P->fn[f][k] * sd) >> 14);
        if (!in_face(P, f, q, 0)) continue;
        const int64_t d2 = sd * sd;
        if (bd2 < 0 || d2 < bd2) { bd2 = d2; for (int k = 0; k < 3; k++) best[k] = q[k]; }
    }
    for (int e = 0; e < P->ne; e++) {
        const int64_t *p = P->v[P->e[e][0]], *q = P->v[P->e[e][1]];
        int64_t ed[3], rx[3], l2 = 0, t = 0;
        for (int k = 0; k < 3; k++) { ed[k] = q[k] - p[k]; rx[k] = x[k] - p[k]; l2 += ed[k] * ed[k]; t += ed[k] * rx[k]; }
        int64_t cp[3];
        if (t <= 0 || l2 == 0) for (int k = 0; k < 3; k++) cp[k] = p[k];
        else if (t >= l2)      for (int k = 0; k < 3; k++) cp[k] = q[k];
        else                   for (int k = 0; k < 3; k++) cp[k] = p[k] + ed[k] * t / l2;
        int64_t d2 = 0;
        for (int k = 0; k < 3; k++) d2 += (x[k] - cp[k]) * (x[k] - cp[k]);
        if (bd2 < 0 || d2 < bd2) { bd2 = d2; for (int k = 0; k < 3; k++) best[k] = cp[k]; }
    }
    if (bd2 < 0 || bd2 >= (r + mg) * (r + mg)) return 0;
    const int64_t dist = isqrt64(bd2);
    if (dist == 0) return 0;
    for (int k = 0; k < 3; k++) { c->n[k] = (int32_t)((best[k] - x[k]) * N1 / dist); c->pt[k] = (int32_t)best[k]; }
    c->depth = (int32_t)(r - dist);
    c->feat = 0;
    return 1;
}

/* a ray against a convex hull: the latest entry through a face plane, before the
 * earliest exit (Cyrus–Beck) */
static int ray_hull(const int64_t o[3], const int32_t u[3], const body_t *b, int64_t *t, int32_t n[3])
{
    poly_t *P = &s_pa; poly_of(b, P);
    const int64_t uu[3] = { u[0], u[1], u[2] };
    int64_t t0 = 0, t1 = (int64_t)1 << 50; int ef = -1;
    for (int f = 0; f < P->nf; f++) {
        const int64_t den = dot14(P->fn[f], uu);              /* Q14 */
        const int64_t num = P->fd[f] - dot14(P->fn[f], o);    /* Q8: > 0 inside */
        if (den == 0) { if (num < 0) return 0; continue; }
        const int64_t tt = num * N1 / den;
        if (den < 0) { if (tt > t0) { t0 = tt; ef = f; } }
        else if (tt < t1) t1 = tt;
        if (t0 > t1) return 0;
    }
    for (int k = 0; k < 3; k++) n[k] = ef >= 0 ? (int32_t)P->fn[ef][k] : 0;
    *t = t0;
    return 1;
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
    static cand_t list[MAX_CAND]; int nl = 0;
    poly_t *P = &s_pa; poly_of(a, P);
    int64_t (*c8)[3] = P->v;
    for (int c = 0; c < P->nv && nl < MAX_CAND; c++) {
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
    if (a->shape != SHAPE_SPHERE && b->shape != SHAPE_SPHERE) return poly_poly(a, b, out, mg);
    if (a->shape == SHAPE_SPHERE) return b->shape == SHAPE_BOX ? sphere_box(a, b, out, mg) : sphere_hull(a, b, out, mg);
    /* box or hull a, sphere b: find it the other way round and turn the normal back */
    if (!(a->shape == SHAPE_BOX ? sphere_box(b, a, out, mg) : sphere_hull(b, a, out, mg))) return 0;
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

/* ── joints ─────────────────────────────────────────────────────────────────
 * A JOINT holds a point of one body to a point of another (or to a fixed point
 * of the world): a BALL joint. A HINGE also keeps an axis of each body lined
 * up, so the pair turns about that axis only — a door, a wheel, a flail.
 *
 * Solved with the contacts, by the same sequential impulses: the point's
 * relative velocity driven to what closes the gap, one world axis at a time,
 * and for a hinge the relative spin across the axis driven to what lines the
 * axes up. A fifth of the error is asked back per substep (Baumgarte); the
 * impulses are kept and re-applied next substep like the contacts' (warm
 * starting) — without that a hanging chain stretches under its own weight.
 * There is no position projection: what is left of the error is the joint's
 * stretch, measured every step (vpyp_stats()->joint_stretch) rather than
 * assumed small. */

/* THE POINT IS SOLVED IN ONE GO, NOT AXIS BY AXIS. What a joint's point "weighs"
 * depends on the direction: along the arm it is the body's mass, across it the
 * body's turning too — 250 times lighter for a ball on a 500-unit arm. Measured
 * with the three world axes solved in turn, like a contact: a pendulum's joint
 * gave 9 units, a door's 41. So the 3x3 matrix
 *     K = (1/ma + 1/mb) I + (1/Ia)(|ra|² I - ra raᵀ) + (1/Ib)(|rb|² I - rb rbᵀ)
 * is inverted (as adjugate over determinant), scaled down first so the
 * determinant fits 64 bits. Units as k_along: Q16. */
static void joint_mass(joint_t *J)
{
    int64_t K[9] = { 0 };
    const int64_t im = (int64_t)inv_of(J->a) + inv_of(J->b);
    K[0] = K[4] = K[8] = im;
    const int ids[2] = { J->a, J->b };
    const int32_t *rs[2] = { J->ra, J->rb };
    for (int s = 0; s < 2; s++) {
        const int64_t ii = invi_of(ids[s]);
        if (!ii) continue;
        const int64_t r[3] = { rs[s][0], rs[s][1], rs[s][2] };
        const int64_t r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];               /* Q16 units² */
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            const int64_t m = (i == j ? r2 : 0) - r[i] * r[j];
            K[i * 3 + j] += ((m >> 16) * ii) >> 16;
        }
    }
    int64_t big = 0;
    for (int i = 0; i < 9; i++) if (iabs64(K[i]) > big) big = iabs64(K[i]);
    int sh = 0;
    while ((big >> sh) >= (1 << 15)) sh++;
    for (int i = 0; i < 9; i++) K[i] >>= sh;
    J->adj[0] = K[4] * K[8] - K[5] * K[7]; J->adj[1] = K[2] * K[7] - K[1] * K[8]; J->adj[2] = K[1] * K[5] - K[2] * K[4];
    J->adj[3] = K[5] * K[6] - K[3] * K[8]; J->adj[4] = K[0] * K[8] - K[2] * K[6]; J->adj[5] = K[2] * K[3] - K[0] * K[5];
    J->adj[6] = K[3] * K[7] - K[4] * K[6]; J->adj[7] = K[1] * K[6] - K[0] * K[7]; J->adj[8] = K[0] * K[4] - K[1] * K[3];
    J->det = K[0] * J->adj[0] + K[1] * J->adj[3] + K[2] * J->adj[6];
    J->ksh = sh;
}

/* an angular impulse (the units of r × impulse) into a body's spin */
static void ang_kick(int id, const int64_t L[3], int sign)
{
    if (id < 0) return;
    body_t *b = &s_b[id];
    if (b->asleep || !b->inv_i) return;
    for (int k = 0; k < 3; k++) b->w[k] += (int32_t)(sign * (((L[k] >> 14) * b->inv_i) >> 14));
}

static int joint_new(int a, int b, int32_t px, int32_t py, int32_t pz, int32_t ax, int32_t ay, int32_t az, int hinge)
{
    body_t *A = get(a), *B = b == VPYP_NONE ? 0 : get(b);
    if (!A || (b != VPYP_NONE && !B) || a == b || (!A->inv_m && (!B || !B->inv_m))) { s_stats.joints_refused++; return VPYP_NONE; }
    int j = 0;
    while (j < VPYP_MAX_JOINTS && s_j[j].used) j++;
    if (j == VPYP_MAX_JOINTS) { s_stats.joints_refused++; return VPYP_NONE; }
    int16_t axis[3] = { 0, 0, 0 };
    if (hinge) {
        int64_t d[3] = { ax, ay, az };
        while (iabs64(d[0]) < (1 << 20) && iabs64(d[1]) < (1 << 20) && iabs64(d[2]) < (1 << 20)) {
            if (!(d[0] | d[1] | d[2])) { s_stats.joints_refused++; return VPYP_NONE; }
            for (int k = 0; k < 3; k++) d[k] *= 2;
        }
        unit_q14(d, axis);
    }
    joint_t *J = &s_j[j];
    J->used = 1; J->hinge = (uint8_t)hinge; J->a = (int16_t)a; J->b = (int16_t)b;
    const int64_t pw[3] = { (int64_t)px * ONE, (int64_t)py * ONE, (int64_t)pz * ONE };
    const int64_t aw[3] = { axis[0], axis[1], axis[2] };
    int64_t rel[3], loc[3];
    for (int k = 0; k < 3; k++) rel[k] = pw[k] - A->p[k];
    rot_t(A->r, rel, loc);
    for (int k = 0; k < 3; k++) J->la[k] = (int32_t)loc[k];
    rot_t(A->r, aw, loc);
    for (int k = 0; k < 3; k++) J->xa[k] = (int16_t)loc[k];
    if (B) {
        for (int k = 0; k < 3; k++) rel[k] = pw[k] - B->p[k];
        rot_t(B->r, rel, loc);
        for (int k = 0; k < 3; k++) J->lb[k] = (int32_t)loc[k];
        rot_t(B->r, aw, loc);
        for (int k = 0; k < 3; k++) J->xb[k] = (int16_t)loc[k];
    } else {
        for (int k = 0; k < 3; k++) { J->lb[k] = (int32_t)pw[k]; J->xb[k] = axis[k]; }
    }
    for (int k = 0; k < 3; k++) { J->jl[k] = 0; J->ja[k] = 0; }
    s_njoint[a]++; if (B) s_njoint[b]++;
    wake(A); if (B) wake(B);
    return j;
}
int vpyp_ball_joint(int a, int b, int32_t px, int32_t py, int32_t pz)
{
    return joint_new(a, b, px, py, pz, 0, 0, 0, 0);
}
int vpyp_hinge(int a, int b, int32_t px, int32_t py, int32_t pz, int32_t ax, int32_t ay, int32_t az)
{
    return joint_new(a, b, px, py, pz, ax, ay, az, 1);
}
void vpyp_joint_remove(int j)
{
    if (j < 0 || j >= VPYP_MAX_JOINTS || !s_j[j].used) return;
    joint_t *J = &s_j[j];
    J->used = 0;
    if (s_b[J->a].alive) { s_njoint[J->a]--; wake(&s_b[J->a]); }
    if (J->b >= 0 && s_b[J->b].alive) { s_njoint[J->b]--; wake(&s_b[J->b]); }
}
int vpyp_joint_alive(int j) { return j >= 0 && j < VPYP_MAX_JOINTS && s_j[j].used; }

static int joined(int a, int b)
{
    if (!s_njoint[a] || !s_njoint[b]) return 0;
    for (int j = 0; j < VPYP_MAX_JOINTS; j++)
        if (s_j[j].used && ((s_j[j].a == a && s_j[j].b == b) || (s_j[j].a == b && s_j[j].b == a))) return 1;
    return 0;
}

/* where each joint is this substep, what it asks for, and what it weighs */
static void joints_prepare(void)
{
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
        joint_t *J = &s_j[j];
        if (!J->used) continue;
        const body_t *A = &s_b[J->a], *B = J->b >= 0 ? &s_b[J->b] : 0;
        const int64_t la[3] = { J->la[0], J->la[1], J->la[2] };
        int64_t ra[3]; rot(A->r, la, ra);
        int64_t pb[3], rb[3] = { 0, 0, 0 };
        if (B) {
            const int64_t lb[3] = { J->lb[0], J->lb[1], J->lb[2] };
            rot(B->r, lb, rb);
            for (int k = 0; k < 3; k++) pb[k] = B->p[k] + rb[k];
        } else for (int k = 0; k < 3; k++) pb[k] = J->lb[k];
        for (int k = 0; k < 3; k++) {
            J->ra[k] = (int32_t)ra[k]; J->rb[k] = (int32_t)rb[k];
            const int64_t err = pb[k] - (A->p[k] + ra[k]);                     /* b's point from a's */
            J->tgt[k] = (int32_t)(-err / JOINT_BETA_DEN);
        }
        joint_mass(J);
        s_b[J->a].touching++; if (B) s_b[J->b].touching++;
        if (!J->hinge) continue;
        const int64_t xa[3] = { J->xa[0], J->xa[1], J->xa[2] }, xb[3] = { J->xb[0], J->xb[1], J->xb[2] };
        int64_t wa[3], wb[3];
        rot(A->r, xa, wa);
        if (B) rot(B->r, xb, wb); else for (int k = 0; k < 3; k++) wb[k] = xb[k];
        int64_t c[3]; cross(wa, wb, c);
        for (int k = 0; k < 3; k++) c[k] >>= 14;                               /* ~ the angle off, Q14 rad */
        /* two directions across a's axis */
        const int ax = iabs64(wa[0]) < iabs64(wa[1]) ? (iabs64(wa[0]) < iabs64(wa[2]) ? 0 : 2)
                                                      : (iabs64(wa[1]) < iabs64(wa[2]) ? 1 : 2);
        int64_t e[3] = { 0, 0, 0 }; e[ax] = N1;
        int64_t t1[3], t2[3];
        cross(wa, e, t1);
        const int64_t l = isqrt64(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
        for (int k = 0; k < 3; k++) t1[k] = l ? t1[k] * N1 / l : 0;
        cross(wa, t1, t2);
        for (int k = 0; k < 3; k++) { t2[k] >>= 14; J->t1[k] = (int16_t)t1[k]; J->t2[k] = (int16_t)t2[k]; }
        /* b turned off a's axis by c: turning b back about -c lines them up */
        J->ta1 = (int32_t)(-((dot14(c, t1)) << (WQ - 14)) / JOINT_BETA_DEN);
        J->ta2 = (int32_t)(-((dot14(c, t2)) << (WQ - 14)) / JOINT_BETA_DEN);
        J->ka = invi_of(J->a) + invi_of(J->b);
    }
}

static void joints_warm(void)
{
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
        joint_t *J = &s_j[j];
        if (!J->used) continue;
        push(J->a, J->ra, J->jl, -1); push(J->b, J->rb, J->jl, +1);
        ang_kick(J->a, J->ja, -1); ang_kick(J->b, J->ja, +1);
    }
}

static void joint_solve(joint_t *J)
{
    if (J->det > 0) {
        int64_t va[3], vb[3], e[3];
        point_vel(J->a, J->ra, va); point_vel(J->b, J->rb, vb);
        for (int k = 0; k < 3; k++) e[k] = (int64_t)J->tgt[k] - (vb[k] - va[k]);   /* Q8/substep */
        /* impulse = K⁻¹ e = adj e / (det << ksh), times 65536 for K's Q16 */
        int64_t imp[3];
        for (int i = 0; i < 3; i++) {
            const int64_t num = J->adj[i * 3] * e[0] + J->adj[i * 3 + 1] * e[1] + J->adj[i * 3 + 2] * e[2];
            imp[i] = (num >> J->ksh) * 65536 / J->det;
            J->jl[i] += imp[i];
        }
        push(J->a, J->ra, imp, -1); push(J->b, J->rb, imp, +1);
    }
    if (!J->hinge || !J->ka) return;
    for (int s = 0; s < 2; s++) {
        const int16_t *t = s ? J->t2 : J->t1;
        const int64_t tt[3] = { t[0], t[1], t[2] };
        int64_t wr[3];
        for (int k = 0; k < 3; k++)
            wr[k] = (J->b >= 0 && !s_b[J->b].asleep ? s_b[J->b].w[k] : 0) - (s_b[J->a].asleep ? 0 : s_b[J->a].w[k]);
        const int64_t dw = (s ? J->ta2 : J->ta1) - dot14(wr, tt);             /* Q20 */
        const int64_t L = dw * ((int64_t)1 << 28) / J->ka;
        int64_t Lv[3];
        for (int k = 0; k < 3; k++) { Lv[k] = (tt[k] * L) >> 14; J->ja[k] += Lv[k]; }
        ang_kick(J->a, Lv, -1); ang_kick(J->b, Lv, +1);
    }
}

/* THE POSITIONS PUT BACK, after they have moved. Velocities alone left the
 * joints apart — measured: a door kicked into a fast swing stood 10 units off
 * its hinge, a swinging chain of five 45 off — because a body that turns moves
 * its point along an arc while the step moves it along a line. So the error
 * left is closed here as well: the same 3x3 mass, the same split between the
 * two bodies, as a nudge of position AND of orientation, so a turning body is
 * turned back onto its hinge rather than slid. A few passes, since one joint's
 * correction moves the next one's body in a chain. */
#define JOINT_PROJECT_PASSES 3
static void joints_project(void)
{
    for (int pass = 0; pass < JOINT_PROJECT_PASSES; pass++)
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
        joint_t *J = &s_j[j];
        if (!J->used) continue;
        body_t *A = &s_b[J->a], *B = J->b >= 0 ? &s_b[J->b] : 0;
        quat_to_matrix(A->q, A->r);
        if (B) quat_to_matrix(B->q, B->r);
        const int64_t la[3] = { J->la[0], J->la[1], J->la[2] };
        int64_t ra[3], rb[3] = { 0, 0, 0 }, pb[3];
        rot(A->r, la, ra);
        if (B) { const int64_t lb[3] = { J->lb[0], J->lb[1], J->lb[2] }; rot(B->r, lb, rb);
                 for (int k = 0; k < 3; k++) pb[k] = B->p[k] + rb[k]; }
        else for (int k = 0; k < 3; k++) pb[k] = J->lb[k];
        int64_t err[3], e2 = 0;
        for (int k = 0; k < 3; k++) { err[k] = pb[k] - (A->p[k] + ra[k]); e2 += err[k] * err[k]; J->ra[k] = (int32_t)ra[k]; J->rb[k] = (int32_t)rb[k]; }
        if (e2 <= (int64_t)SLOP * SLOP) continue;
        joint_mass(J);
        if (J->det <= 0) continue;
        int64_t P[3];
        for (int i = 0; i < 3; i++) {
            const int64_t num = J->adj[i * 3] * -err[0] + J->adj[i * 3 + 1] * -err[1] + J->adj[i * 3 + 2] * -err[2];
            P[i] = (num >> J->ksh) * 65536 / J->det * PUSH_NUM / PUSH_DEN;
        }
        body_t *bs[2] = { A, B }; const int64_t *rs[2] = { ra, rb }; const int sg[2] = { -1, 1 };
        for (int s = 0; s < 2; s++) {
            body_t *b = bs[s];
            if (!b || b->asleep || !b->inv_m) continue;
            for (int k = 0; k < 3; k++) b->p[k] += (int32_t)(sg[s] * ((P[k] * b->inv_m) >> 16));
            if (!b->inv_i) continue;
            int64_t t[3]; cross(rs[s], P, t);
            int32_t dth[3];
            for (int k = 0; k < 3; k++) dth[k] = (int32_t)(sg[s] * (((t[k] >> 14) * b->inv_i) >> 14));
            quat_integrate(b->q, dth);
        }
    }
}

/* how far apart the worst joint's two points are, units, after the step */
static int32_t joints_stretch(void)
{
    int64_t worst = 0;
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
        const joint_t *J = &s_j[j];
        if (!J->used) continue;
        const body_t *A = &s_b[J->a], *B = J->b >= 0 ? &s_b[J->b] : 0;
        const int64_t la[3] = { J->la[0], J->la[1], J->la[2] };
        int64_t ra[3]; rot(A->r, la, ra);
        int64_t pb[3];
        if (B) { const int64_t lb[3] = { J->lb[0], J->lb[1], J->lb[2] }; int64_t rb[3]; rot(B->r, lb, rb);
                 for (int k = 0; k < 3; k++) pb[k] = B->p[k] + rb[k]; }
        else for (int k = 0; k < 3; k++) pb[k] = J->lb[k];
        int64_t d2 = 0;
        for (int k = 0; k < 3; k++) { const int64_t d = pb[k] - (A->p[k] + ra[k]); d2 += d * d; }
        const int64_t d = isqrt64(d2);
        if (d > worst) worst = d;
    }
    return (int32_t)(worst >> Q);
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
            if (joined(i, j)) continue;                      /* a hinge's two halves overlap by design */
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
    /* two bodies joined move together: one awake wakes the other */
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
        const joint_t *J = &s_j[j];
        if (!J->used || J->b < 0) continue;
        body_t *A = &s_b[J->a], *B = &s_b[J->b];
        if (A->asleep != B->asleep && A->inv_m && B->inv_m) { wake(A); wake(B); }
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
        /* a joint to the world or to something that does not move holds a body up */
        for (int j = 0; j < VPYP_MAX_JOINTS; j++) {
            const joint_t *J = &s_j[j];
            if (!J->used) continue;
            if (J->b < 0 || !s_b[J->b].inv_m || s_b[J->b].asleep) held[J->a] = 1;
            if (J->b >= 0 && (!s_b[J->a].inv_m || s_b[J->a].asleep)) held[J->b] = 1;
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

    joints_prepare();

    /* 4. impulses: last substep's first, then the passes */
    joints_warm();
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
        for (int j = 0; j < VPYP_MAX_JOINTS; j++) if (s_j[j].used) joint_solve(&s_j[j]);
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

    joints_project();

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
    uint32_t nj = 0;
    for (int j = 0; j < VPYP_MAX_JOINTS; j++) nj += s_j[j].used;
    s_stats.joints = nj;
    s_stats.joint_stretch = joints_stretch();
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

/* ── a blast ──────────────────────────────────────────────────────────────── */
int vpyp_blast(int32_t cx, int32_t cy, int32_t cz, int32_t radius, int32_t speed, uint8_t mask)
{
    if (radius <= 0) return 0;
    const int64_t c[3] = { (int64_t)cx * ONE, (int64_t)cy * ONE, (int64_t)cz * ONE };
    const int64_t R = (int64_t)radius * ONE;
    int n = 0;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        body_t *b = &s_b[i];
        if (!b->alive || !b->inv_m || !(b->mask & mask)) continue;
        int64_t d[3], d2 = 0;
        for (int k = 0; k < 3; k++) { d[k] = b->p[k] - c[k]; d2 += d[k] * d[k]; }
        const int64_t dist = isqrt64(d2);
        if (dist >= R) continue;
        if (dist == 0) { d[0] = 0; d[1] = ONE; d[2] = 0; }
        const int64_t dl = dist ? dist : ONE;
        /* the same change of SPEED for every body at a given distance, falling
         * to nothing at the radius — heavy or light, a blast throws them alike */
        const int64_t dv = (int64_t)speed_in(speed) * (R - dist) / R;          /* Q8/substep */
        /* applied on the side facing the blast, so it turns as well as moves */
        const int64_t size = b->shape == SHAPE_SPHERE ? b->h[0]
                           : (b->h[0] < b->h[1] ? (b->h[0] < b->h[2] ? b->h[0] : b->h[2])
                                                : (b->h[1] < b->h[2] ? b->h[1] : b->h[2]));
        int64_t imp[3], r[3];
        for (int k = 0; k < 3; k++) {
            imp[k] = (d[k] * dv / dl) * 65536 / b->inv_m;                      /* mass × Q8 vel */
            r[k] = -d[k] * size / dl;
        }
        for (int k = 0; k < 3; k++) b->v[k] += (int32_t)((imp[k] * b->inv_m) >> 16);
        spin_kick(b, r, imp, +1);
        wake(b);
        n++;
    }
    return n;
}

int vpyp_raycast(int32_t ox, int32_t oy, int32_t oz,
                 int32_t dx, int32_t dy, int32_t dz,
                 int32_t max_dist, uint8_t mask, vpyp_hit *out)
{
    /* SCALED UP BEFORE IT IS NORMALISED: the square root of a short direction is
     * a poor integer — (3,-3,0) has length 4 to isqrt, and the distances came
     * out 6% long. At 2^20 the root is good to a millionth. */
    int64_t dd[3] = { dx, dy, dz };
    while (iabs64(dd[0]) < (1 << 20) && iabs64(dd[1]) < (1 << 20) && iabs64(dd[2]) < (1 << 20)) {
        if (!(dd[0] | dd[1] | dd[2])) return VPYP_NONE;
        for (int k = 0; k < 3; k++) dd[k] *= 2;
    }
    const int64_t dl = isqrt64(dd[0] * dd[0] + dd[1] * dd[1] + dd[2] * dd[2]);
    if (dl == 0) return VPYP_NONE;
    const int32_t u[3] = { (int32_t)(dd[0] * N1 / dl), (int32_t)(dd[1] * N1 / dl), (int32_t)(dd[2] * N1 / dl) };
    const int64_t o[3] = { (int64_t)ox * ONE, (int64_t)oy * ONE, (int64_t)oz * ONE };
    int64_t best = (int64_t)max_dist * ONE;
    int id = VPYP_NONE;
    int32_t bn[3] = { 0, 0, 0 };

    for (int i = 0; i < VPYP_MAX_BODIES; i++) {
        const body_t *b = &s_b[i];
        if (!b->alive || !(b->mask & mask)) continue;
        int64_t t; int32_t n[3];
        const int hit = b->shape == SHAPE_SPHERE ? ray_sphere(o, u, b, &t, n)
                      : b->shape == SHAPE_BOX    ? ray_box(o, u, b, &t, n) : ray_hull(o, u, b, &t, n);
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
