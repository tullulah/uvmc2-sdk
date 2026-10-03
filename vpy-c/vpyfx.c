/*
 * vpyfx.c — see vpyfx.h. Integer only, deterministic, like the rest of libvpy.
 *
 * UNITS INSIDE: positions Q8 world units; velocities Q8 per step; a stick's
 * half-vector Q8; spin radians per step Q20; brightness 0..127.
 */
#include "vpyfx.h"

#define Q    8
#define ONE  (1 << Q)
#define WQ   20
#define N1   16384

/* A spark's streak is this many steps of its own travel, and never shorter than
 * MIN_STREAK: a zero-length stroke is a dot the beam may not light at all. */
#define STREAK_STEPS 2
#define MIN_STREAK   (3 * ONE)
#define MIN_BR       6            /* fainter than this is not worth a stroke */
#define SCRAPE_NUM   3            /* what a bounce leaves of the sliding and the spin */
#define SCRAPE_DEN   4

enum { K_SPARK = 0, K_STICK = 1, K_RING = 2, K_LINE = 3, K_GATHER = 4 };

/* A ring keeps its centre in p, its plane's normal (Q14) in h, its radius (Q8)
 * in hl, its growth per step (Q8) in v[0] and its number of segments in w[0].
 * A line keeps a stick's ends and never moves. */
/* A GATHER piece (vpyfx_assemble) flies from where it starts to its slot on an edge:
 * it keeps its start in v (the centre) and w (the half), its slot in t and th, and
 * counts life down to its arrival; then it stays, at full brightness, until its group
 * is released. `delay` holds a piece still before it starts (both a disintegration's
 * wave and an assembly's stagger); `group` ties an assembly's pieces together. */
typedef struct {
    uint8_t  alive, kind, group, arrived;
    int16_t  br0;
    int16_t  delay;               /* steps before it starts to move */
    int32_t  life, life0;
    int32_t  p[3];                /* Q8: a spark's point, a stick's centre */
    int32_t  v[3];                /* Q8 per step */
    int32_t  h[3];                /* Q8: half of a stick, centre to one end */
    int32_t  hl;                  /* |h|, kept as it turns */
    int32_t  w[3];                /* Q20 rad per step */
    int32_t  t[3], th[3];         /* GATHER: the slot's centre and half, Q8 */
} piece_t;

static piece_t       s_p[VPYFX_MAX];
static vpyfx_stats_t s_stats;
static uint32_t      s_rng = 0x2545F491u;
static int           s_rate = 50, s_budget = 160;
static int32_t       s_g_api[3], s_g[3];
static int           s_floor_on;
static int32_t       s_floor_y;
static int           s_bounce = 96;

/* ── small things ─────────────────────────────────────────────────────────── */
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
static uint32_t rnd(void)                     /* xorshift32 */
{
    uint32_t x = s_rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return s_rng = x;
}
static int32_t rnd_range(int32_t lo, int32_t hi)    /* lo..hi inclusive */
{
    if (hi <= lo) return lo;
    return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1));
}
/* a random direction, Q14 */
static void rnd_dir(int32_t d[3])
{
    for (int tries = 0; tries < 8; tries++) {
        int64_t v[3], l2 = 0;
        for (int k = 0; k < 3; k++) { v[k] = rnd_range(-N1, N1); l2 += v[k] * v[k]; }
        if (l2 > (int64_t)N1 * N1 || l2 < (int64_t)N1 * N1 / 64) continue;   /* inside the ball */
        const int64_t l = isqrt64(l2);
        for (int k = 0; k < 3; k++) d[k] = (int32_t)(v[k] * N1 / l);
        return;
    }
    d[0] = 0; d[1] = N1; d[2] = 0;
}
static int32_t speed_in(int32_t u)
{
    const int64_t n = (int64_t)u * ONE;
    return (int32_t)((n + (n >= 0 ? s_rate / 2 : -s_rate / 2)) / s_rate);
}
static int32_t spin_in(int32_t a)        /* 4096ths of a turn / s -> rad / step Q20 */
{
    const int64_t d = (int64_t)s_rate * 1000;
    const int64_t n = (int64_t)a * 1608495;
    return (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
}
static void redo_gravity(void)
{
    const int64_t d = (int64_t)s_rate * s_rate;
    for (int k = 0; k < 3; k++) {
        const int64_t n = (int64_t)s_g_api[k] * ONE;
        s_g[k] = (int32_t)((n + (n >= 0 ? d / 2 : -d / 2)) / d);
    }
}

/* ── the system ───────────────────────────────────────────────────────────── */
void vpyfx_reset(void)
{
    for (int i = 0; i < VPYFX_MAX; i++) s_p[i].alive = 0;
    s_rate = 50; s_budget = 160;
    s_g_api[0] = s_g_api[1] = s_g_api[2] = 0; redo_gravity();
    s_floor_on = 0;
    vpyfx_stats_t z = {0}; s_stats = z;
}
void vpyfx_seed(uint32_t seed) { s_rng = seed ? seed : 0x2545F491u; }
void vpyfx_set_rate(int r) { if (r > 0) { s_rate = r; redo_gravity(); } }
void vpyfx_set_gravity(int32_t gx, int32_t gy, int32_t gz)
{
    s_g_api[0] = gx; s_g_api[1] = gy; s_g_api[2] = gz; redo_gravity();
}
void vpyfx_set_floor(int on, int32_t y, int bounce_q8)
{
    s_floor_on = on ? 1 : 0; s_floor_y = y * ONE;
    s_bounce = bounce_q8 < 0 ? 0 : (bounce_q8 > ONE ? ONE : bounce_q8);
}
void vpyfx_set_budget(int n) { s_budget = n < 0 ? 0 : n; }
const vpyfx_stats_t *vpyfx_stats(void) { return &s_stats; }

/* A free slot, or — the pool full — the piece nearest its end, counted. */
static piece_t *slot(void)
{
    int best = -1; int32_t least = 0x7fffffff;
    for (int i = 0; i < VPYFX_MAX; i++) {
        if (!s_p[i].alive) return &s_p[i];
        if (s_p[i].life < least) { least = s_p[i].life; best = i; }
    }
    s_stats.recycled++;
    return &s_p[best];
}

static piece_t *new_piece(int life, int br)
{
    if (life <= 0 || br <= 0) return 0;
    piece_t *p = slot();
    p->alive = 1; p->kind = K_SPARK; p->group = 0; p->arrived = 0; p->delay = 0;
    p->life = p->life0 = life;
    p->br0 = (int16_t)(br > 127 ? 127 : br);
    p->h[0] = p->h[1] = p->h[2] = 0; p->hl = 0;
    p->w[0] = p->w[1] = p->w[2] = 0;
    return p;
}

/* ── making pieces ─────────────────────────────────────────────────────────── */
int vpyfx_spark(int32_t x, int32_t y, int32_t z, int32_t vx, int32_t vy, int32_t vz, int life, int br)
{
    piece_t *p = new_piece(life, br);
    if (!p) return 0;
    p->p[0] = x * ONE; p->p[1] = y * ONE; p->p[2] = z * ONE;
    p->v[0] = speed_in(vx); p->v[1] = speed_in(vy); p->v[2] = speed_in(vz);
    return 1;
}

int vpyfx_burst(int32_t x, int32_t y, int32_t z, int32_t vx, int32_t vy, int32_t vz,
                int count, int32_t speed, int life, int br)
{
    int n = 0;
    for (int i = 0; i < count; i++) {
        int32_t d[3]; rnd_dir(d);
        const int32_t s = rnd_range(speed / 3, speed);              /* not all at the same speed */
        const int l = rnd_range(life * 3 / 4, life * 5 / 4);
        n += vpyfx_spark(x, y, z, vx + (int32_t)((int64_t)d[0] * s / N1),
                                  vy + (int32_t)((int64_t)d[1] * s / N1),
                                  vz + (int32_t)((int64_t)d[2] * s / N1), l, br);
    }
    return n;
}

static piece_t *stick_q8(const int64_t a[3], const int64_t b[3], const int32_t v[3], int32_t spin, int life, int br)
{
    piece_t *p = new_piece(life, br);
    if (!p) return 0;
    p->kind = K_STICK;
    int64_t l2 = 0;
    for (int k = 0; k < 3; k++) {
        p->p[k] = (int32_t)((a[k] + b[k]) / 2);
        p->h[k] = (int32_t)((b[k] - a[k]) / 2);
        l2 += (int64_t)p->h[k] * p->h[k];
        p->v[k] = v[k];
    }
    p->hl = (int32_t)isqrt64(l2);
    if (spin) {
        int32_t d[3]; rnd_dir(d);
        const int32_t w = spin_in(rnd_range(spin / 3, spin));
        for (int k = 0; k < 3; k++) p->w[k] = (int32_t)((int64_t)d[k] * w / N1);
    }
    return p;
}

int vpyfx_stick(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz,
                int32_t vx, int32_t vy, int32_t vz, int32_t spin, int life, int br)
{
    const int64_t a[3] = { (int64_t)ax * ONE, (int64_t)ay * ONE, (int64_t)az * ONE };
    const int64_t b[3] = { (int64_t)bx * ONE, (int64_t)by * ONE, (int64_t)bz * ONE };
    const int32_t v[3] = { speed_in(vx), speed_in(vy), speed_in(vz) };
    return stick_q8(a, b, v, spin, life, br) != 0;
}

int vpyfx_shatter(const vpy_mesh *m, const vpy_xf *place,
                  int32_t vx, int32_t vy, int32_t vz,
                  int32_t cx, int32_t cy, int32_t cz,
                  int32_t speed, int32_t spin, int life, int br)
{
    const int ne = vpy3d_mesh_edge_count(m);
    const int64_t c[3] = { (int64_t)cx * ONE, (int64_t)cy * ONE, (int64_t)cz * ONE };
    int n = 0;
    for (int e = 0; e < ne; e++) {
        int32_t ma[3], mb[3];
        if (!vpy3d_mesh_edge(m, e, ma, mb)) continue;
        int64_t wa[3], wb[3], mid[3];
        for (int r = 0; r < 3; r++) {      /* model -> world, as vpy3d places a mesh */
            wa[r] = ((((int64_t)place->m[r*3] * ma[0] + (int64_t)place->m[r*3+1] * ma[1] +
                       (int64_t)place->m[r*3+2] * ma[2]) >> 14) + place->t[r]) * ONE;
            wb[r] = ((((int64_t)place->m[r*3] * mb[0] + (int64_t)place->m[r*3+1] * mb[1] +
                       (int64_t)place->m[r*3+2] * mb[2]) >> 14) + place->t[r]) * ONE;
            mid[r] = (wa[r] + wb[r]) / 2;
        }
        /* away from the blow, at a speed that varies piece to piece, plus a
         * little scatter so pieces in a line do not stay in a line */
        int64_t d[3], l2 = 0;
        for (int k = 0; k < 3; k++) { d[k] = mid[k] - c[k]; l2 += d[k] * d[k]; }
        const int64_t l = isqrt64(l2);
        int32_t jit[3]; rnd_dir(jit);
        const int32_t s = rnd_range(speed / 2, speed);
        const int32_t base[3] = { vx, vy, vz };
        int32_t v[3];
        for (int k = 0; k < 3; k++) {
            const int64_t out = l ? d[k] * s / l : 0;
            v[k] = speed_in(base[k] + (int32_t)out + (int32_t)((int64_t)jit[k] * (s / 4) / N1));
        }
        n += stick_q8(wa, wb, v, spin, rnd_range(life * 3 / 4, life * 5 / 4), br) != 0;
    }
    return n;
}

/* An edge of `m` placed by `place`, in world units Q8, split into n equal pieces: piece i
 * runs from a + (b-a)·i/n to a + (b-a)·(i+1)/n. Returns 0 for an edge that is not there. */
static int edge_piece(const vpy_mesh *m, const vpy_xf *place, int e, int i, int n, int64_t a[3], int64_t b[3])
{
    int32_t ma[3], mb[3];
    if (!vpy3d_mesh_edge(m, e, ma, mb)) return 0;
    for (int r = 0; r < 3; r++) {
        const int64_t wa = ((((int64_t)place->m[r*3] * ma[0] + (int64_t)place->m[r*3+1] * ma[1] +
                              (int64_t)place->m[r*3+2] * ma[2]) >> 14) + place->t[r]) * ONE;
        const int64_t wb = ((((int64_t)place->m[r*3] * mb[0] + (int64_t)place->m[r*3+1] * mb[1] +
                              (int64_t)place->m[r*3+2] * mb[2]) >> 14) + place->t[r]) * ONE;
        a[r] = wa + (wb - wa) * i / n;
        b[r] = wa + (wb - wa) * (i + 1) / n;
    }
    return 1;
}

int vpyfx_disintegrate(const vpy_mesh *m, const vpy_xf *place, int32_t cx, int32_t cy, int32_t cz,
                       int per_edge, int32_t speed, int32_t spin, int32_t wave, int life, int br)
{
    if (per_edge < 1) per_edge = 1;
    const int ne = vpy3d_mesh_edge_count(m);
    const int64_t c[3] = { (int64_t)cx * ONE, (int64_t)cy * ONE, (int64_t)cz * ONE };
    /* the wave's speed in Q8 per step: a piece this far from the blow waits this long */
    const int64_t wave_q8 = wave > 0 ? (int64_t)speed_in(wave) : 0;
    int n = 0;
    for (int e = 0; e < ne; e++)
        for (int i = 0; i < per_edge; i++) {
            int64_t a[3], b[3];
            if (!edge_piece(m, place, e, i, per_edge, a, b)) continue;
            int64_t d[3], l2 = 0;
            for (int k = 0; k < 3; k++) { d[k] = (a[k] + b[k]) / 2 - c[k]; l2 += d[k] * d[k]; }
            const int64_t l = isqrt64(l2);
            int32_t jit[3]; rnd_dir(jit);
            const int32_t sp = rnd_range(speed / 2, speed);
            int32_t v[3];
            for (int k = 0; k < 3; k++)
                v[k] = speed_in((int32_t)(l ? d[k] * sp / l : 0) + (int32_t)((int64_t)jit[k] * (sp / 3) / N1));
            piece_t *p = stick_q8(a, b, v, spin, rnd_range(life * 3 / 4, life * 5 / 4), br);
            if (!p) continue;
            if (wave_q8 > 0) {
                const int64_t dl = l / wave_q8;
                p->delay = (int16_t)(dl > 32000 ? 32000 : dl);
            }
            n++;
        }
    return n;
}

static uint8_t s_group;
int vpyfx_assemble(const vpy_mesh *m, const vpy_xf *place, int per_edge, int32_t scatter,
                   int frames, int stagger, int br)
{
    if (per_edge < 1) per_edge = 1;
    if (frames < 1) frames = 1;
    if (++s_group == 0) s_group = 1;                 /* 0 is "no group" */
    const int ne = vpy3d_mesh_edge_count(m);
    int made = 0;
    for (int e = 0; e < ne; e++)
        for (int i = 0; i < per_edge; i++) {
            int64_t a[3], b[3];
            if (!edge_piece(m, place, e, i, per_edge, a, b)) continue;
            piece_t *p = new_piece(frames, br);
            if (!p) continue;
            p->kind = K_GATHER; p->group = s_group;
            int64_t l2 = 0;
            for (int k = 0; k < 3; k++) {
                p->t[k] = (int32_t)((a[k] + b[k]) / 2);
                p->th[k] = (int32_t)((b[k] - a[k]) / 2);
                l2 += (int64_t)p->th[k] * p->th[k];
            }
            /* it starts out somewhere round its slot, turned any way, the same length */
            int32_t d[3], r[3]; rnd_dir(d); rnd_dir(r);
            const int64_t far = (int64_t)rnd_range(scatter / 2, scatter) * ONE, hl = isqrt64(l2);
            for (int k = 0; k < 3; k++) {
                p->v[k] = p->t[k] + (int32_t)(d[k] * far / N1);
                p->w[k] = (int32_t)(r[k] * hl / N1);
                p->p[k] = p->v[k]; p->h[k] = p->w[k];
            }
            p->hl = (int32_t)hl;
            /* SEWN EDGE BY EDGE: an edge's pieces leave together, the edges one after
             * another across `stagger` steps, with a little spread inside an edge */
            p->delay = (int16_t)(ne > 1 ? (int64_t)stagger * e / ne : 0) + (int16_t)rnd_range(0, stagger / (ne + 1) + 1);
            made++;
        }
    return made ? s_group : 0;
}

int vpyfx_assembled(int group)
{
    int any = 0;
    for (int i = 0; i < VPYFX_MAX; i++) {
        const piece_t *p = &s_p[i];
        if (!p->alive || p->kind != K_GATHER || p->group != group) continue;
        any = 1;
        if (!p->arrived) return 0;
    }
    return any;
}

void vpyfx_release(int group)
{
    for (int i = 0; i < VPYFX_MAX; i++)
        if (s_p[i].alive && s_p[i].kind == K_GATHER && s_p[i].group == group) s_p[i].alive = 0;
}

int vpyfx_line(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz, int life, int br)
{
    const int64_t a[3] = { (int64_t)ax * ONE, (int64_t)ay * ONE, (int64_t)az * ONE };
    const int64_t b[3] = { (int64_t)bx * ONE, (int64_t)by * ONE, (int64_t)bz * ONE };
    const int32_t v[3] = { 0, 0, 0 };
    piece_t *p = stick_q8(a, b, v, 0, life, br);
    if (!p) return 0;
    p->kind = K_LINE;
    return 1;
}

int vpyfx_ring(int32_t cx, int32_t cy, int32_t cz, int32_t nx, int32_t ny, int32_t nz,
               int32_t r0, int32_t speed, int segments, int life, int br)
{
    const int64_t l = isqrt64((int64_t)nx * nx + (int64_t)ny * ny + (int64_t)nz * nz);
    if (l == 0 || segments < 3) return 0;
    piece_t *p = new_piece(life, br);
    if (!p) return 0;
    p->kind = K_RING;
    p->p[0] = cx * ONE; p->p[1] = cy * ONE; p->p[2] = cz * ONE;
    p->h[0] = (int32_t)(nx * (int64_t)N1 / l); p->h[1] = (int32_t)(ny * (int64_t)N1 / l); p->h[2] = (int32_t)(nz * (int64_t)N1 / l);
    p->hl = r0 * ONE;
    p->v[0] = speed_in(speed); p->v[1] = p->v[2] = 0;
    p->w[0] = segments > 32 ? 32 : segments;
    return 1;
}

/* ── every frame ──────────────────────────────────────────────────────────── */
void vpyfx_step(void)
{
    uint32_t alive = 0;
    for (int i = 0; i < VPYFX_MAX; i++) {
        piece_t *p = &s_p[i];
        if (!p->alive) continue;
        alive++;
        if (p->delay > 0) { p->delay--; continue; }     /* still waiting: neither moves nor ages */
        if (p->kind == K_GATHER) {
            if (p->arrived) continue;
            if (--p->life <= 0) {                        /* there: exactly on its slot */
                for (int k = 0; k < 3; k++) { p->p[k] = p->t[k]; p->h[k] = p->th[k]; }
                p->arrived = 1;
                continue;
            }
            /* EASE OUT: 1 - (1 - u)^3, so it comes in fast and settles on its slot */
            const int64_t u = (int64_t)(p->life0 - p->life) * N1 / p->life0, r = N1 - u;
            const int64_t e = N1 - ((r * r >> 14) * r >> 14);
            for (int k = 0; k < 3; k++) {
                p->p[k] = (int32_t)(p->v[k] + (((int64_t)p->t[k] - p->v[k]) * e >> 14));
                p->h[k] = (int32_t)(p->w[k] + (((int64_t)p->th[k] - p->w[k]) * e >> 14));
            }
            continue;
        }
        if (--p->life <= 0) { p->alive = 0; alive--; continue; }
        if (p->kind == K_RING) { p->hl += p->v[0]; continue; }
        if (p->kind == K_LINE) continue;
        for (int k = 0; k < 3; k++) { p->v[k] += s_g[k]; p->p[k] += p->v[k]; }
        if (p->kind == K_STICK && (p->w[0] | p->w[1] | p->w[2])) {
            /* turn the half-vector by w × h, and keep its length */
            const int64_t w[3] = { p->w[0], p->w[1], p->w[2] }, h[3] = { p->h[0], p->h[1], p->h[2] };
            int64_t nh[3], l2 = 0;
            nh[0] = h[0] + ((w[1] * h[2] - w[2] * h[1]) >> WQ);
            nh[1] = h[1] + ((w[2] * h[0] - w[0] * h[2]) >> WQ);
            nh[2] = h[2] + ((w[0] * h[1] - w[1] * h[0]) >> WQ);
            for (int k = 0; k < 3; k++) l2 += nh[k] * nh[k];
            const int64_t l = isqrt64(l2);
            for (int k = 0; k < 3; k++) p->h[k] = l ? (int32_t)(nh[k] * p->hl / l) : (int32_t)h[k];
        }
        if (s_floor_on) {
            const int32_t reach = p->h[1] < 0 ? -p->h[1] : p->h[1];   /* a stick's lower end */
            const int32_t low = p->p[1] - reach;
            if (low < s_floor_y) {
                p->p[1] += s_floor_y - low;
                if (p->v[1] < 0) {
                    p->v[1] = (int32_t)((-(int64_t)p->v[1] * s_bounce) >> Q);
                    /* a bounce smaller than one step of gravity is lying still */
                    if (p->v[1] <= (s_g[1] < 0 ? -s_g[1] : s_g[1]) * 2) p->v[1] = 0;
                    p->v[0] = p->v[0] * SCRAPE_NUM / SCRAPE_DEN;
                    p->v[2] = p->v[2] * SCRAPE_NUM / SCRAPE_DEN;
                    for (int k = 0; k < 3; k++) p->w[k] = p->w[k] * SCRAPE_NUM / SCRAPE_DEN;
                }
            }
        }
    }
    s_stats.alive = alive;
}

static int brightness(const piece_t *p)
{
    /* waiting or gathering: whole — it is still, or about to be, part of the object */
    if (p->delay > 0 || p->kind == K_GATHER) return p->br0;
    return (int)((int64_t)p->br0 * p->life / p->life0);
}

/* the two ends of a piece, Q8 */
static void ends(const piece_t *p, int64_t a[3], int64_t b[3])
{
    if (p->kind == K_STICK || p->kind == K_LINE || p->kind == K_GATHER) {
        for (int k = 0; k < 3; k++) { a[k] = (int64_t)p->p[k] - p->h[k]; b[k] = (int64_t)p->p[k] + p->h[k]; }
        return;
    }
    int64_t s[3], l2 = 0;
    for (int k = 0; k < 3; k++) { s[k] = -(int64_t)p->v[k] * STREAK_STEPS; l2 += s[k] * s[k]; }
    if (l2 < (int64_t)MIN_STREAK * MIN_STREAK) {
        const int64_t l = isqrt64(l2);
        if (l == 0) { s[0] = MIN_STREAK; s[1] = s[2] = 0; }
        else for (int k = 0; k < 3; k++) s[k] = s[k] * MIN_STREAK / l;
    }
    for (int k = 0; k < 3; k++) { a[k] = p->p[k]; b[k] = p->p[k] + s[k]; }
}

static void draw_all(int mode)          /* 0 = 3D, 1 = 3D occluded, 2 = 2D */
{
    const int keep = vpy_get_priority();
    vpy_set_priority(VPY_PRI_LOW);
    uint32_t drawn = 0, shed = 0;
    for (int i = 0; i < VPYFX_MAX; i++) {
        const piece_t *p = &s_p[i];
        if (!p->alive) continue;
        const int br = brightness(p);
        if (br < MIN_BR) continue;
        if (p->kind == K_RING) {
            const int segs = p->w[0];
            if ((int)drawn + segs > s_budget) { shed++; continue; }
            /* two directions across the plane: from whichever axis is least
             * along the normal, then the normal crossed with that */
            const int64_t n[3] = { p->h[0], p->h[1], p->h[2] };
            const int ax = (n[0] < 0 ? -n[0] : n[0]) < (n[1] < 0 ? -n[1] : n[1])
                         ? ((n[0] < 0 ? -n[0] : n[0]) < (n[2] < 0 ? -n[2] : n[2]) ? 0 : 2)
                         : ((n[1] < 0 ? -n[1] : n[1]) < (n[2] < 0 ? -n[2] : n[2]) ? 1 : 2);
            int64_t e[3] = { 0, 0, 0 }; e[ax] = N1;
            int64_t u[3] = { n[1] * e[2] - n[2] * e[1], n[2] * e[0] - n[0] * e[2], n[0] * e[1] - n[1] * e[0] };
            const int64_t ul = isqrt64(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
            for (int k = 0; k < 3; k++) u[k] = ul ? u[k] * N1 / ul : 0;
            const int64_t w[3] = { (n[1] * u[2] - n[2] * u[1]) >> 14, (n[2] * u[0] - n[0] * u[2]) >> 14,
                                   (n[0] * u[1] - n[1] * u[0]) >> 14 };
            int32_t px = 0, py = 0, pz = 0;
            for (int sgi = 0; sgi <= segs; sgi++) {
                const int ang = sgi * VPY_Q14_TURN / segs;
                const int64_t c = vpy_cos_q14(ang), sn = vpy_sin_q14(ang);
                int32_t q[3];
                for (int k = 0; k < 3; k++)
                    q[k] = (int32_t)((p->p[k] + ((((u[k] * c + w[k] * sn) >> 14) * p->hl) >> 14)) >> Q);
                if (sgi) {
                    if (mode == 2) vpy_draw_line_dev(px, py, q[0], q[1], br);
                    else if (mode == 1) vpy3d_occl_line(px, py, pz, q[0], q[1], q[2], br);
                    else vpy3d_line_world(px, py, pz, q[0], q[1], q[2], br);
                    drawn++;
                }
                px = q[0]; py = q[1]; pz = q[2];
            }
            continue;
        }
        if ((int)drawn >= s_budget) { shed++; continue; }
        int64_t a[3], b[3];
        ends(p, a, b);
        const int32_t ax = (int32_t)(a[0] >> Q), ay = (int32_t)(a[1] >> Q), az = (int32_t)(a[2] >> Q);
        const int32_t bx = (int32_t)(b[0] >> Q), by = (int32_t)(b[1] >> Q), bz = (int32_t)(b[2] >> Q);
        if (mode == 2) vpy_draw_line_dev(ax, ay, bx, by, br);
        else if (mode == 1) vpy3d_occl_line(ax, ay, az, bx, by, bz, br);
        else vpy3d_line_world(ax, ay, az, bx, by, bz, br);
        drawn++;
    }
    vpy_set_priority(keep);
    s_stats.drawn = drawn; s_stats.shed = shed;
}

void vpyfx_draw(int occlude) { draw_all(occlude ? 1 : 0); }
void vpyfx_draw2d(void)      { draw_all(2); }
