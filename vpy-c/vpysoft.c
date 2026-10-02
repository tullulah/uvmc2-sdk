/*
 * vpysoft.c — see vpysoft.h. Positions Q8 world units; Verlet, like vpyrope.c:
 * the velocity is the difference between where a point is and where it was.
 */
#include "vpysoft.h"
#include "vpy.h"
#include "vpy3d.h"

#define Q   8
#define ONE (1 << Q)

typedef struct { int32_t p[3], o[3], pin[3]; uint8_t pinned; } pt_t;
/* a, b: indices into s_p. rest Q8. stiff Q8 (256 = back to length every pass). */
typedef struct { uint16_t a, b; int32_t rest; uint16_t stiff; uint8_t visible; } spring_t;
enum { K_FREE = 0, K_CLOTH, K_BLOB, K_MESH };
typedef struct {
    uint8_t kind;
    int     p0, np;               /* its points: s_p[p0 .. p0+np) */
    int     s0, ns;               /* its springs */
    int64_t area0;                /* a blob's own area, Q16 units² */
    int     pressure;             /* Q8 */
} body_t;

static pt_t     s_p[VPYSOFT_MAX_POINTS];
static spring_t s_s[VPYSOFT_MAX_SPRINGS];
static body_t   s_b[VPYSOFT_MAX_BODIES];
static int      s_rate = 50, s_iter = 8, s_damp = 254, s_budget = 160;
static int32_t  s_g_api[3], s_g[3];
static int      s_floor_on, s_floor_fric = 128;
static int32_t  s_floor_y;
static vpysoft_stats_t s_stats;

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
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

void vpysoft_reset(void)
{
    for (int i = 0; i < VPYSOFT_MAX_BODIES; i++) s_b[i].kind = K_FREE;
    s_rate = 50; s_iter = 8; s_damp = 254; s_budget = 160;
    s_g_api[0] = s_g_api[1] = s_g_api[2] = 0; redo_gravity();
    s_floor_on = 0; s_floor_fric = 128;
    vpysoft_stats_t z = {0}; s_stats = z;
}
void vpysoft_set_rate(int r) { if (r > 0) { s_rate = r; redo_gravity(); } }
void vpysoft_set_gravity(int32_t gx, int32_t gy, int32_t gz) { s_g_api[0] = gx; s_g_api[1] = gy; s_g_api[2] = gz; redo_gravity(); }
void vpysoft_set_floor(int on, int32_t y, int fric_q8) { s_floor_on = on ? 1 : 0; s_floor_y = y * ONE; s_floor_fric = clampi(fric_q8, 0, ONE); }
void vpysoft_set_iterations(int n) { s_iter = clampi(n, 1, 64); }
void vpysoft_set_damping(int q8) { s_damp = clampi(q8, 0, ONE); }
void vpysoft_set_budget(int n) { s_budget = n < 0 ? 0 : n; }
const vpysoft_stats_t *vpysoft_stats(void) { return &s_stats; }

/* THE TABLES ARE KEPT PACKED: a new body takes the first run of free slots in
 * each, so a body's points and springs are each one contiguous range and the
 * step walks plain arrays. `which` 0 = points, 1 = springs. */
static int free_run(int which, int n)
{
    const int max = which ? VPYSOFT_MAX_SPRINGS : VPYSOFT_MAX_POINTS;
    for (int start = 0; start + n <= max; start++) {
        int ok = 1;
        for (int b = 0; b < VPYSOFT_MAX_BODIES && ok; b++) {
            if (s_b[b].kind == K_FREE) continue;
            const int f = which ? s_b[b].s0 : s_b[b].p0, c = which ? s_b[b].ns : s_b[b].np;
            if (start < f + c && f < start + n) { ok = 0; start = f + c - 1; }
        }
        if (ok) return start;
    }
    return -1;
}
/* A body with room for np points and ns springs, or -1 (counted). */
static int new_body(int kind, int np, int ns)
{
    if (np < 2 || ns < 1 || np > VPYSOFT_MAX_POINTS || ns > VPYSOFT_MAX_SPRINGS) { s_stats.refused++; return -1; }
    int b = 0;
    while (b < VPYSOFT_MAX_BODIES && s_b[b].kind != K_FREE) b++;
    if (b == VPYSOFT_MAX_BODIES) { s_stats.refused++; return -1; }
    const int p0 = free_run(0, np), s0 = free_run(1, ns);
    if (p0 < 0 || s0 < 0) { s_stats.refused++; return -1; }
    body_t *B = &s_b[b];
    B->kind = (uint8_t)kind; B->p0 = p0; B->np = np; B->s0 = s0; B->ns = 0;
    B->area0 = 0; B->pressure = 0;
    return b;
}
static void set_point(int i, int64_t x, int64_t y, int64_t z)
{
    pt_t *p = &s_p[i];
    p->p[0] = p->o[0] = (int32_t)x; p->p[1] = p->o[1] = (int32_t)y; p->p[2] = p->o[2] = (int32_t)z;
    p->pinned = 0;
}
/* a spring between two of body b's points (local indices), at its current length */
static void add_spring(int b, int ia, int ib, int stiff, int visible)
{
    body_t *B = &s_b[b];
    spring_t *s = &s_s[B->s0 + B->ns++];
    s->a = (uint16_t)(B->p0 + ia); s->b = (uint16_t)(B->p0 + ib);
    s->stiff = (uint16_t)clampi(stiff, 1, ONE); s->visible = (uint8_t)visible;
    int64_t d2 = 0;
    for (int k = 0; k < 3; k++) { const int64_t d = (int64_t)s_p[s->b].p[k] - s_p[s->a].p[k]; d2 += d * d; }
    s->rest = (int32_t)isqrt64(d2);
}

int vpysoft_cloth(int32_t x, int32_t y, int32_t z, int w, int h, int32_t cell, int stiff)
{
    if (w < 2 || h < 2 || cell <= 0) { s_stats.refused++; return -1; }
    const int ns = (w - 1) * h + w * (h - 1) + 2 * (w - 1) * (h - 1);
    const int b = new_body(K_CLOTH, w * h, ns);
    if (b < 0) return -1;
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++)
            set_point(s_b[b].p0 + r * w + c, ((int64_t)x + (int64_t)c * cell) * ONE,
                      ((int64_t)y - (int64_t)r * cell) * ONE, (int64_t)z * ONE);
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++) {
            const int i = r * w + c;
            if (c + 1 < w) add_spring(b, i, i + 1, stiff, 1);
            if (r + 1 < h) add_spring(b, i, i + w, stiff, 1);
            /* SHEAR, HIDDEN: without the diagonals a grid of squares folds into
             * rhombi under its own weight; drawn, they would cross every cell */
            if (c + 1 < w && r + 1 < h) { add_spring(b, i, i + w + 1, stiff / 2, 0); add_spring(b, i + 1, i + w, stiff / 2, 0); }
        }
    return b;
}

/* a blob's area in the x-y plane, Q16, by the shoelace formula over its rim */
static int64_t rim_area(const body_t *B)
{
    const int n = B->np - 1;
    int64_t a2 = 0;
    for (int i = 0; i < n; i++) {
        const pt_t *p = &s_p[B->p0 + 1 + i], *q = &s_p[B->p0 + 1 + (i + 1) % n];
        a2 += (int64_t)p->p[0] * q->p[1] - (int64_t)q->p[0] * p->p[1];
    }
    return a2 / 2;
}

int vpysoft_blob(int32_t x, int32_t y, int32_t z, int n, int32_t radius, int stiff, int pressure)
{
    if (n < 3 || radius <= 0) { s_stats.refused++; return -1; }
    const int b = new_body(K_BLOB, n + 1, 2 * n);
    if (b < 0) return -1;
    body_t *B = &s_b[b];
    set_point(B->p0, (int64_t)x * ONE, (int64_t)y * ONE, (int64_t)z * ONE);
    for (int i = 0; i < n; i++) {
        const int a = i * VPY_Q14_TURN / n;                         /* counter-clockwise */
        set_point(B->p0 + 1 + i, ((int64_t)x * ONE) + ((int64_t)radius * ONE * vpy_cos_q14(a)) / VPY_Q14_ONE,
                  ((int64_t)y * ONE) + ((int64_t)radius * ONE * vpy_sin_q14(a)) / VPY_Q14_ONE, (int64_t)z * ONE);
    }
    for (int i = 0; i < n; i++) add_spring(b, 1 + i, 1 + (i + 1) % n, stiff, 1);
    /* SPOKES, SOFT AND HIDDEN: they keep the centre in the middle so the blob
     * has a place to draw a face from, but at a quarter of the rim's stiffness
     * they let it squash — the area term is what holds the volume */
    for (int i = 0; i < n; i++) add_spring(b, 0, 1 + i, stiff / 4 > 0 ? stiff / 4 : 1, 0);
    B->area0 = rim_area(B);
    B->pressure = clampi(pressure, 0, ONE);
    return b;
}

int vpysoft_mesh(const vpy_mesh *m, int32_t x, int32_t y, int32_t z, int stiff)
{
    const int ne = vpy3d_mesh_edge_count(m);
    if (ne <= 0) { s_stats.refused++; return -1; }
    /* THE VERTICES, FROM THE EDGES' ENDS: vpy3d gives edges as two points, so two
     * ends at the same place are the same vertex. Counted first, so the body is
     * made the size it needs or refused whole. */
    static int32_t v[VPYSOFT_MAX_POINTS][3];
    int nv = 0;
    for (int e = 0; e < ne; e++) {
        int32_t ab[2][3];
        vpy3d_mesh_edge(m, e, ab[0], ab[1]);
        for (int s = 0; s < 2; s++) {
            int f = 0;
            while (f < nv && !(v[f][0] == ab[s][0] && v[f][1] == ab[s][1] && v[f][2] == ab[s][2])) f++;
            if (f == nv) {
                if (nv == VPYSOFT_MAX_POINTS) { s_stats.refused++; return -1; }
                v[nv][0] = ab[s][0]; v[nv][1] = ab[s][1]; v[nv][2] = ab[s][2]; nv++;
            }
        }
    }
    /* PLUS A HIDDEN CENTRE, TIED TO EVERY VERTEX. Edges alone do not hold a solid's
     * shape: measured, a cube of its 12 edges dropped on the floor folded flat, 0
     * units tall — a wire box has no stiffness against shear. Spokes to a centre
     * point, at half the edges' stiffness, give it back without drawing anything. */
    if (nv + 1 > VPYSOFT_MAX_POINTS) { s_stats.refused++; return -1; }
    const int b = new_body(K_MESH, nv + 1, ne + nv);
    if (b < 0) return -1;
    int64_t cen[3] = { 0, 0, 0 };
    for (int i = 0; i < nv; i++) {
        set_point(s_b[b].p0 + i, ((int64_t)x + v[i][0]) * ONE, ((int64_t)y + v[i][1]) * ONE, ((int64_t)z + v[i][2]) * ONE);
        for (int k = 0; k < 3; k++) cen[k] += v[i][k];
    }
    set_point(s_b[b].p0 + nv, ((int64_t)x + cen[0] / nv) * ONE, ((int64_t)y + cen[1] / nv) * ONE, ((int64_t)z + cen[2] / nv) * ONE);
    for (int e = 0; e < ne; e++) {
        int32_t ab[2][3]; int idx[2] = { 0, 0 };
        vpy3d_mesh_edge(m, e, ab[0], ab[1]);
        for (int s = 0; s < 2; s++)
            while (!(v[idx[s]][0] == ab[s][0] && v[idx[s]][1] == ab[s][1] && v[idx[s]][2] == ab[s][2])) idx[s]++;
        add_spring(b, idx[0], idx[1], stiff, 1);
    }
    for (int i = 0; i < nv; i++) add_spring(b, nv, i, stiff / 2 > 0 ? stiff / 2 : 1, 0);
    return b;
}

void vpysoft_free(int b) { if (b >= 0 && b < VPYSOFT_MAX_BODIES) s_b[b].kind = K_FREE; }

static pt_t *pt(int b, int i)
{
    if (b < 0 || b >= VPYSOFT_MAX_BODIES || s_b[b].kind == K_FREE || i < 0 || i >= s_b[b].np) return 0;
    return &s_p[s_b[b].p0 + i];
}
int  vpysoft_points(int b) { return (b >= 0 && b < VPYSOFT_MAX_BODIES && s_b[b].kind != K_FREE) ? s_b[b].np : 0; }
void vpysoft_point(int b, int i, int32_t *x, int32_t *y, int32_t *z)
{
    const pt_t *p = pt(b, i);
    if (!p) { *x = *y = *z = 0; return; }
    *x = p->p[0] >> Q; *y = p->p[1] >> Q; *z = p->p[2] >> Q;
}
void vpysoft_pin(int b, int i, int32_t x, int32_t y, int32_t z)
{
    pt_t *p = pt(b, i); if (!p) return;
    p->pinned = 1; p->pin[0] = x * ONE; p->pin[1] = y * ONE; p->pin[2] = z * ONE;
}
void vpysoft_unpin(int b, int i) { pt_t *p = pt(b, i); if (p) p->pinned = 0; }
void vpysoft_push(int b, int32_t vx, int32_t vy, int32_t vz)
{
    if (b < 0 || b >= VPYSOFT_MAX_BODIES || s_b[b].kind == K_FREE) return;
    const int32_t d[3] = { (int32_t)((int64_t)vx * ONE / s_rate), (int32_t)((int64_t)vy * ONE / s_rate),
                           (int32_t)((int64_t)vz * ONE / s_rate) };
    for (int i = 0; i < s_b[b].np; i++) {
        pt_t *p = &s_p[s_b[b].p0 + i];
        if (!p->pinned) for (int k = 0; k < 3; k++) p->o[k] -= d[k];   /* Verlet: a velocity is p - o */
    }
}

/* A BLOB'S PRESSURE: the area error put back, `pressure` of it per pass, by
 * moving every rim point along the area's gradient. Moving rim point i by δ
 * changes the area by δ · g_i, with g_i = ½ (y[i+1] - y[i-1], x[i-1] - x[i+1]);
 * so δ_i = g_i × ΔA / Σ|g|² puts back exactly ΔA to first order, whichever way
 * the rim is wound. */
static void keep_area(const body_t *B)
{
    const int n = B->np - 1;
    const int64_t dA = B->area0 - rim_area(B);
    if (dA == 0) return;
    int64_t sum = 0;
    for (int i = 0; i < n; i++) {
        const pt_t *nx = &s_p[B->p0 + 1 + (i + 1) % n], *pv = &s_p[B->p0 + 1 + (i + n - 1) % n];
        const int64_t gx = ((int64_t)nx->p[1] - pv->p[1]) / 2, gy = ((int64_t)pv->p[0] - nx->p[0]) / 2;
        sum += gx * gx + gy * gy;
    }
    if (sum == 0) return;
    for (int i = 0; i < n; i++) {
        pt_t *p = &s_p[B->p0 + 1 + i];
        if (p->pinned) continue;
        const pt_t *nx = &s_p[B->p0 + 1 + (i + 1) % n], *pv = &s_p[B->p0 + 1 + (i + n - 1) % n];
        const int64_t gx = ((int64_t)nx->p[1] - pv->p[1]) / 2, gy = ((int64_t)pv->p[0] - nx->p[0]) / 2;
        p->p[0] += (int32_t)(gx * (dA * B->pressure / ONE) / sum);
        p->p[1] += (int32_t)(gy * (dA * B->pressure / ONE) / sum);
    }
}

void vpysoft_step(void)
{
    uint32_t nb = 0, np = 0, ns = 0;
    int64_t worst = 0, worst_area = 0;
    for (int b = 0; b < VPYSOFT_MAX_BODIES; b++) {
        body_t *B = &s_b[b];
        if (B->kind == K_FREE) continue;
        nb++; np += (uint32_t)B->np; ns += (uint32_t)B->ns;
        /* move: Verlet, damped, with gravity; a pinned point goes where it is held */
        for (int i = 0; i < B->np; i++) {
            pt_t *p = &s_p[B->p0 + i];
            if (p->pinned) { for (int k = 0; k < 3; k++) { p->o[k] = p->p[k]; p->p[k] = p->pin[k]; } continue; }
            for (int k = 0; k < 3; k++) {
                const int32_t v = (int32_t)(((int64_t)(p->p[k] - p->o[k]) * s_damp) >> Q);
                p->o[k] = p->p[k];
                p->p[k] += v + s_g[k];
            }
        }
        /* springs back towards length, a few times; a pinned end does not give */
        for (int it = 0; it < s_iter; it++) {
            for (int j = 0; j < B->ns; j++) {
                const spring_t *s = &s_s[B->s0 + j];
                pt_t *a = &s_p[s->a], *c = &s_p[s->b];
                int64_t d[3], l2 = 0;
                for (int k = 0; k < 3; k++) { d[k] = (int64_t)c->p[k] - a->p[k]; l2 += d[k] * d[k]; }
                const int64_t l = isqrt64(l2);
                if (l == 0) continue;
                const int64_t diff = (l - s->rest) * s->stiff / ONE;           /* too long > 0 */
                const int wa = a->pinned ? 0 : 1, wc = c->pinned ? 0 : 1;
                if (!(wa + wc)) continue;
                for (int k = 0; k < 3; k++) {
                    const int64_t m = d[k] * diff / l;
                    if (wa) a->p[k] += (int32_t)(m * wa / (wa + wc));
                    if (wc) c->p[k] -= (int32_t)(m * wc / (wa + wc));
                }
            }
            if (B->kind == K_BLOB && B->pressure) keep_area(B);
            if (s_floor_on)
                for (int i = 0; i < B->np; i++) {
                    pt_t *p = &s_p[B->p0 + i];
                    if (p->pinned || p->p[1] >= s_floor_y) continue;
                    p->p[1] = s_floor_y;
                    /* friction: a share of the sideways step taken back */
                    p->o[0] += (int32_t)(((int64_t)(p->p[0] - p->o[0]) * s_floor_fric) >> Q);
                    p->o[2] += (int32_t)(((int64_t)(p->p[2] - p->o[2]) * s_floor_fric) >> Q);
                }
        }
        for (int j = 0; j < B->ns; j++) {
            const spring_t *s = &s_s[B->s0 + j];
            int64_t l2 = 0;
            for (int k = 0; k < 3; k++) { const int64_t d = (int64_t)s_p[s->b].p[k] - s_p[s->a].p[k]; l2 += d * d; }
            const int64_t over = isqrt64(l2) - s->rest;
            if (over > worst) worst = over;
        }
        if (B->kind == K_BLOB && B->area0) {
            int64_t e = rim_area(B) - B->area0; if (e < 0) e = -e;
            e = e * ONE / (B->area0 < 0 ? -B->area0 : B->area0);
            if (e > worst_area) worst_area = e;
        }
    }
    s_stats.bodies = nb; s_stats.points = np; s_stats.springs = ns;
    s_stats.stretch = (int32_t)(worst >> Q);
    s_stats.area_error_q8 = (int32_t)worst_area;
}

/* every visible spring one stroke, at the lowest priority, within the budget */
static void draw_mode(int b, int br, int mode)
{
    if (b < 0 || b >= VPYSOFT_MAX_BODIES || s_b[b].kind == K_FREE) return;
    const int keep = vpy_get_priority();
    vpy_set_priority(VPY_PRI_LOW);
    uint32_t drawn = 0, shed = 0;
    for (int j = 0; j < s_b[b].ns; j++) {
        const spring_t *s = &s_s[s_b[b].s0 + j];
        if (!s->visible) continue;
        if ((int)drawn >= s_budget) { shed++; continue; }
        const pt_t *a = &s_p[s->a], *c = &s_p[s->b];
        const int32_t ax = a->p[0] >> Q, ay = a->p[1] >> Q, az = a->p[2] >> Q;
        const int32_t cx = c->p[0] >> Q, cy = c->p[1] >> Q, cz = c->p[2] >> Q;
        if (mode == 2) vpy_draw_line_dev(ax, ay, cx, cy, br);
        else if (mode == 1) vpy3d_occl_line(ax, ay, az, cx, cy, cz, br);
        else vpy3d_line_world(ax, ay, az, cx, cy, cz, br);
        drawn++;
    }
    vpy_set_priority(keep);
    s_stats.drawn = drawn; s_stats.shed = shed;
}
void vpysoft_draw(int b, int br, int occlude) { draw_mode(b, br, occlude ? 1 : 0); }
void vpysoft_draw2d(int b, int br) { draw_mode(b, br, 2); }
