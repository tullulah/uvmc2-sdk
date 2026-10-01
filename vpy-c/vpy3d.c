/*
 * vpy3d.c — see vpy3d.h. Integer only, like the rest of libvpy.
 *
 * The three parts, in the order they run:
 *   BUILD (once)  vertices and faces in, normals + an edge table with
 *                 adjacency and straight-run links out.
 *   DRAW (frame)  transform the vertices, test every face against the camera,
 *                 decide each edge, merge straight runs, push strokes.
 *   The strokes go to libvpy's buffer, so 3D shares the frame, the priorities
 *   and the flush with the rest of the game.
 */
#include "vpy3d.h"

/* ── pools, shared by every mesh ─────────────────────────────────────────── */
typedef struct {
    uint16_t a, b;      /* its two vertices, mesh-local */
    int16_t  f0, f1;    /* the faces either side, mesh-local; f1 < 0 = boundary */
    int16_t  ca, cb;    /* edge continuing the straight run past a / past b */
    uint8_t  hard;      /* the faces meet sharply: keep it when one is visible */
} vpy3d_edge;

static int16_t     PV[VPY3D_POOL_V][3];
static int16_t     PN[VPY3D_POOL_F][3];     /* face normals, Q14 unit vectors */
static uint16_t    PFs[VPY3D_POOL_F];       /* where the face's indices start */
static uint8_t     PFn[VPY3D_POOL_F];       /* how many it has */
static uint16_t    PFV[VPY3D_POOL_FV];
static vpy3d_edge  PE[VPY3D_POOL_E];
static int nPV, nPF, nPFV, nPE;

static vpy_mesh   *B;                        /* the mesh being built */
static vpy3d_stats_t s_stats;

/* per-draw scratch */
static int32_t TC[VPY3D_MESH_MAXV][3];       /* vertices in camera space */
static uint8_t FV[VPY3D_MESH_MAXF];          /* 1 = face turned toward us */
static uint8_t ED[VPY3D_POOL_E];             /* 0 skip, 1 draw, 2 already drawn */
static uint8_t ES[VPY3D_POOL_E];             /* 1 = silhouette, 0 = interior */
/* the runs of one mesh, as vertex pairs, before they are put in an order */
static uint16_t RA[VPY3D_POOL_E], RB[VPY3D_POOL_E];
static uint8_t  RU[VPY3D_POOL_E];            /* already emitted */
static int      s_chain = 1;                 /* follow connectivity when emitting */
static int      s_wire;                      /* draw hidden edges too: see the header */
static int      s_mesh_occl;                 /* meshes go through the occluder: see the header */

/* ── camera and lens ─────────────────────────────────────────────────────── */
static vpy_xf  s_cam;
static int     s_cam_set;
static int32_t s_focal = 28000;   /* deflection units per unit of x/z (~58 deg) */
static int32_t s_near  = 600;     /* world units */
/* Half the visible window, per axis, in deflection units. The default is still
 * the 15500 square every vpy3d game has been composed against. The glass is
 * bigger and portrait — about +-18000 x +-20500 on the one console photographed
 * (2026-10-01, examples/geometry_card) — but one console does not set a default:
 * a game that wants the tall window says so with vpy3d_set_clip_xy, and a second
 * console's photograph is what moves these two numbers. */
static int32_t s_clip_x = 15500;
static int32_t s_clip_y = 15500;

/* ── small integer maths ─────────────────────────────────────────────────── */
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

/* Scale (x,y,z) to a Q14 unit vector. 0 if it has no length to speak of. */
static int norm_q14(int64_t x, int64_t y, int64_t z, int32_t *out)
{
    int64_t len = isqrt64(x * x + y * y + z * z);
    if (len == 0) return 0;
    out[0] = (int32_t)((x * VPY3D_ONE) / len);
    out[1] = (int32_t)((y * VPY3D_ONE) / len);
    out[2] = (int32_t)((z * VPY3D_ONE) / len);
    return 1;
}

static void cross64(const int64_t *a, const int64_t *b, int64_t *o)
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

/* ── transforms ──────────────────────────────────────────────────────────── */
vpy_xf vpy3d_identity(void)
{
    vpy_xf r;
    r.m[0] = VPY3D_ONE; r.m[1] = 0;          r.m[2] = 0;
    r.m[3] = 0;         r.m[4] = VPY3D_ONE;  r.m[5] = 0;
    r.m[6] = 0;         r.m[7] = 0;          r.m[8] = VPY3D_ONE;
    r.t[0] = r.t[1] = r.t[2] = 0;
    return r;
}

vpy_xf vpy3d_mul(const vpy_xf *a, const vpy_xf *b)   /* a applied after b */
{
    vpy_xf r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int64_t s = 0;
            for (int k = 0; k < 3; k++)
                s += (int64_t)a->m[i * 3 + k] * b->m[k * 3 + j];
            r.m[i * 3 + j] = (int32_t)(s >> 14);
        }
        int64_t t = 0;
        for (int k = 0; k < 3; k++) t += (int64_t)a->m[i * 3 + k] * b->t[k];
        r.t[i] = (int32_t)(t >> 14) + a->t[i];
    }
    return r;
}

vpy_xf vpy3d_rot_x(int ang)
{
    int32_t c = vpy_cos_q14(ang), s = vpy_sin_q14(ang);
    vpy_xf r = vpy3d_identity();
    r.m[4] = c; r.m[5] = -s;
    r.m[7] = s; r.m[8] =  c;
    return r;
}
vpy_xf vpy3d_rot_y(int ang)
{
    int32_t c = vpy_cos_q14(ang), s = vpy_sin_q14(ang);
    vpy_xf r = vpy3d_identity();
    r.m[0] =  c; r.m[2] = s;
    r.m[6] = -s; r.m[8] = c;
    return r;
}
vpy_xf vpy3d_rot_z(int ang)
{
    int32_t c = vpy_cos_q14(ang), s = vpy_sin_q14(ang);
    vpy_xf r = vpy3d_identity();
    r.m[0] = c; r.m[1] = -s;
    r.m[3] = s; r.m[4] =  c;
    return r;
}
vpy_xf vpy3d_translate(int32_t x, int32_t y, int32_t z)
{
    vpy_xf r = vpy3d_identity();
    r.t[0] = x; r.t[1] = y; r.t[2] = z;
    return r;
}

/* ── the camera ──────────────────────────────────────────────────────────── */
void vpy3d_set_camera(const vpy_xf *c) { s_cam = *c; s_cam_set = 1; }
const vpy_xf *vpy3d_camera(void)
{
    if (!s_cam_set) { s_cam = vpy3d_identity(); s_cam_set = 1; }
    return &s_cam;
}
void vpy3d_eye(int32_t *out)
{
    const vpy_xf *c = vpy3d_camera();
    /* R is orthonormal, so its inverse is its transpose: eye = -R^T * t. */
    for (int i = 0; i < 3; i++) {
        int64_t s = (int64_t)c->m[0 * 3 + i] * c->t[0]
                  + (int64_t)c->m[1 * 3 + i] * c->t[1]
                  + (int64_t)c->m[2 * 3 + i] * c->t[2];
        out[i] = (int32_t)(-(s >> 14));
    }
}

void vpy3d_set_focal(int32_t f) { if (f > 0) s_focal = f; }
void vpy3d_set_near(int32_t n)  { if (n > 0) s_near  = n; }
void vpy3d_set_clip(int32_t h)  { if (h > 0) s_clip_x = s_clip_y = h; }
void vpy3d_set_clip_xy(int32_t hx, int32_t hy)
{
    if (hx > 0) s_clip_x = hx;
    if (hy > 0) s_clip_y = hy;
}

int vpy3d_look_at(int32_t ex, int32_t ey, int32_t ez,
                  int32_t tx, int32_t ty, int32_t tz,
                  int32_t ux, int32_t uy, int32_t uz)
{
    /* Camera basis: f forward (+z into the screen), r right (+x), u up (+y).
     * The rotation that takes world into camera space has those as its ROWS,
     * because it is the inverse of the rotation that would orient the camera —
     * and the inverse of an orthonormal matrix is its transpose. */
    int32_t f[3], r[3], u[3];
    int64_t fv[3] = { (int64_t)tx - ex, (int64_t)ty - ey, (int64_t)tz - ez };
    if (!norm_q14(fv[0], fv[1], fv[2], f)) return 0;

    int64_t upv[3] = { ux, uy, uz }, rv[3];
    int64_t fq[3] = { f[0], f[1], f[2] };
    cross64(upv, fq, rv);                       /* right = up x forward */
    if (!norm_q14(rv[0], rv[1], rv[2], r)) return 0;   /* looking straight up */

    int64_t rq[3] = { r[0], r[1], r[2] }, uv[3];
    cross64(fq, rq, uv);                        /* true up = forward x right */
    if (!norm_q14(uv[0], uv[1], uv[2], u)) return 0;

    vpy_xf c;
    c.m[0] = r[0]; c.m[1] = r[1]; c.m[2] = r[2];
    c.m[3] = u[0]; c.m[4] = u[1]; c.m[5] = u[2];
    c.m[6] = f[0]; c.m[7] = f[1]; c.m[8] = f[2];
    /* translation = -R * eye */
    for (int i = 0; i < 3; i++) {
        int64_t s = (int64_t)c.m[i * 3 + 0] * ex
                  + (int64_t)c.m[i * 3 + 1] * ey
                  + (int64_t)c.m[i * 3 + 2] * ez;
        c.t[i] = (int32_t)(-(s >> 14));
    }
    vpy3d_set_camera(&c);
    return 1;
}

void vpy3d_to_camera(int32_t wx, int32_t wy, int32_t wz, int32_t *out)
{
    const vpy_xf *c = vpy3d_camera();
    for (int i = 0; i < 3; i++) {
        int64_t s = (int64_t)c->m[i * 3 + 0] * wx
                  + (int64_t)c->m[i * 3 + 1] * wy
                  + (int64_t)c->m[i * 3 + 2] * wz;
        out[i] = (int32_t)(s >> 14) + c->t[i];
    }
}

/* ── projection and clipping ─────────────────────────────────────────────── */
static int64_t lim(int64_t v)
{
    const int64_t L = (int64_t)1 << 30;
    return v > L ? L : (v < -L ? -L : v);
}

/* ONE UNIT IS ONE UNIT ON BOTH AXES. The glass is portrait; the UNITS are not.
 *
 * MEASURED 2026-10-01, one console, by photograph (examples/geometry_card in the
 * starter kit: device units straight to v_directDraw32, no vpy3d in between). A
 * 16000 x 16000 square came out 0.92 as wide as it was tall, and the circle
 * inscribed in it round to the eye. So a y unit covers about the same glass as
 * an x unit — the residual ~10% is that console's size pots or the camera, and
 * one console is not evidence for a default.
 *
 * WHAT THAT REPLACED. From 2026-09-30 to 2026-10-01 this file multiplied x by
 * 4/3, on the argument that R401/R408 (horizontal and vertical SIZE) stretch
 * each axis to its own edge of a 4:3 tube, so a y unit would be 1.33 x units.
 * It was derived from the service manual and never measured, and it disagreed
 * with the PiTrex contract (vectrexInterface.h: the same units on both axes, a
 * portrait WINDOW), with the BIOS (one scale factor for x and y, round circles)
 * and with every emulator. The photograph settled it: the portrait shape of the
 * tube lives in the WINDOW — see the per-axis clip below — not in the scale.
 *
 * The ratio stays a knob, because a console whose size pots are off is real and
 * a game may want to undo it. Not because the default is in doubt. */
/* Overridable at build time ONLY so a host harness can compare pictures:
 * `cc -DVPY3D_ASPECT_NUM=4 -DVPY3D_ASPECT_DEN=3 ...` reruns the 4:3 picture. */
#ifndef VPY3D_ASPECT_NUM
#define VPY3D_ASPECT_NUM 1
#endif
#ifndef VPY3D_ASPECT_DEN
#define VPY3D_ASPECT_DEN 1
#endif
static int32_t s_asp_num = VPY3D_ASPECT_NUM, s_asp_den = VPY3D_ASPECT_DEN;

void vpy3d_set_aspect(int32_t num, int32_t den)
{
    if (num > 0 && den > 0) { s_asp_num = num; s_asp_den = den; }
}
void vpy3d_aspect(int32_t *num, int32_t *den) { *num = s_asp_num; *den = s_asp_den; }

static inline int64_t proj_x(int64_t x, int64_t z)
{
    return lim(x * s_focal * s_asp_num / (z * s_asp_den));
}
static inline int64_t proj_y(int64_t y, int64_t z) { return lim(y * s_focal / z); }

int vpy3d_project(const int32_t *p, int32_t *sx, int32_t *sy)
{
    if (p[2] < s_near) return 0;
    *sx = (int32_t)proj_x(p[0], p[2]);
    *sy = (int32_t)proj_y(p[1], p[2]);
    return 1;
}

/* How wide the picture actually is, in the Q14 trig's units — so a game that
 * needs to know what is on screen (a light cone, a ray fan, a cull) reads it
 * instead of writing 58 degrees down a second time and drifting from it.
 *
 * tan(half) = clip / (focal * num/den), with each axis's own clip. Found by bisection over the same sine
 * table the renderer uses, once, so the answer cannot disagree with the picture. */
static int half_angle(int32_t clip, int32_t num, int32_t den)
{
    int lo = 0, hi = VPY_Q14_TURN / 4;       /* a quarter turn is the limit */
    for (int i = 0; i < 14; i++) {
        const int mid = (lo + hi) / 2;
        /* sin/cos >= clip*den / (focal*num)  ->  the angle is at least mid */
        const int64_t l = (int64_t)vpy_sin_q14(mid) * (int64_t)s_focal * num;
        const int64_t r = (int64_t)vpy_cos_q14(mid) * (int64_t)clip * den;
        if (l < r) lo = mid; else hi = mid;
    }
    return lo;
}
int vpy3d_h_half_angle(void) { return half_angle(s_clip_x, s_asp_num, s_asp_den); }
int vpy3d_v_half_angle(void) { return half_angle(s_clip_y, 1, 1); }

static int outcode(int64_t x, int64_t y)
{
    return (x < -s_clip_x) | ((x > s_clip_x) << 1) | ((y < -s_clip_y) << 2) | ((y > s_clip_y) << 3);
}

/* Cohen-Sutherland against the visible square, then into libvpy's buffer. */
static void line_screen(int64_t x0, int64_t y0, int64_t x1, int64_t y1, int br)
{
    int c0 = outcode(x0, y0), c1 = outcode(x1, y1);
    for (int guard = 0; guard < 8; guard++) {
        if (!(c0 | c1)) {
            vpy_draw_line_dev((int32_t)x0, (int32_t)y0, (int32_t)x1, (int32_t)y1, br);
            s_stats.strokes++;
            return;
        }
        if (c0 & c1) return;                 /* wholly outside the same edge */
        int c = c0 ? c0 : c1;
        int64_t x, y, dx = x1 - x0, dy = y1 - y0;
        if      (c & 8) { y =  s_clip_y; x = x0 + (dy ? dx * (y - y0) / dy : 0); }
        else if (c & 4) { y = -s_clip_y; x = x0 + (dy ? dx * (y - y0) / dy : 0); }
        else if (c & 2) { x =  s_clip_x; y = y0 + (dx ? dy * (x - x0) / dx : 0); }
        else            { x = -s_clip_x; y = y0 + (dx ? dy * (x - x0) / dx : 0); }
        if (c == c0) { x0 = x; y0 = y; c0 = outcode(x0, y0); }
        else         { x1 = x; y1 = y; c1 = outcode(x1, y1); }
    }
}

void vpy3d_line_cam(const int32_t *a, const int32_t *b, int br)
{
    int64_t x0 = a[0], y0 = a[1], z0 = a[2];
    int64_t x1 = b[0], y1 = b[1], z1 = b[2];
    if (z0 < s_near && z1 < s_near) return;          /* all of it is behind us */
    if (z0 < s_near) {                                /* cut it at the near plane */
        int64_t k = ((s_near - z0) << 16) / (z1 - z0);
        x0 += ((x1 - x0) * k) >> 16;
        y0 += ((y1 - y0) * k) >> 16;
        z0 = s_near;
    }
    if (z1 < s_near) {
        int64_t k = ((s_near - z1) << 16) / (z0 - z1);
        x1 += ((x0 - x1) * k) >> 16;
        y1 += ((y0 - y1) * k) >> 16;
        z1 = s_near;
    }
    line_screen(proj_x(x0, z0), proj_y(y0, z0),
                proj_x(x1, z1), proj_y(y1, z1), br);
}

void vpy3d_line_world(int32_t ax, int32_t ay, int32_t az,
                      int32_t bx, int32_t by, int32_t bz, int br)
{
    int32_t a[3], b[3];
    vpy3d_to_camera(ax, ay, az, a);
    vpy3d_to_camera(bx, by, bz, b);
    vpy3d_line_cam(a, b, br);
}

/* ── building a mesh ─────────────────────────────────────────────────────── */
int vpy3d_mesh_begin(vpy_mesh *m)
{
    B = m;
    m->v0 = (uint16_t)nPV; m->nv = 0;
    m->f0 = (uint16_t)nPF; m->nf = 0;
    m->e0 = (uint16_t)nPE; m->ne = 0;
    m->open = 0;
    return 1;
}

int vpy3d_vertex(int x, int y, int z)
{
    if (!B || nPV >= VPY3D_POOL_V || B->nv >= VPY3D_MESH_MAXV) {
        s_stats.overflow++; return -1;
    }
    PV[nPV][0] = (int16_t)x; PV[nPV][1] = (int16_t)y; PV[nPV][2] = (int16_t)z;
    nPV++;
    return B->nv++;
}

int vpy3d_face(const int *idx, int n)
{
    if (!B || n < 3 || nPF >= VPY3D_POOL_F || B->nf >= VPY3D_MESH_MAXF
           || nPFV + n > VPY3D_POOL_FV) {
        s_stats.overflow++; return -1;
    }
    PFs[nPF] = (uint16_t)nPFV;
    PFn[nPF] = (uint8_t)n;
    for (int i = 0; i < n; i++) PFV[nPFV++] = (uint16_t)idx[i];
    nPF++;
    return B->nf++;
}
int vpy3d_quad(int a, int b, int c, int d) { int i[4] = {a,b,c,d}; return vpy3d_face(i, 4); }
int vpy3d_tri (int a, int b, int c)        { int i[3] = {a,b,c};   return vpy3d_face(i, 3); }

void vpy3d_mesh_open(vpy_mesh *m, int open) { m->open = (uint8_t)(open ? 1 : 0); }

static const int16_t *MV(const vpy_mesh *m, int i) { return PV[m->v0 + i]; }

/* Read-only access to a built mesh's edges, in MODEL space: what breaking it
 * into pieces (vpyfx_shatter) needs. Every edge, not only the visible ones. */
int vpy3d_mesh_edge_count(const vpy_mesh *m) { return m ? m->ne : 0; }
int vpy3d_mesh_edge(const vpy_mesh *m, int e, int32_t a[3], int32_t b[3])
{
    if (!m || e < 0 || e >= m->ne) return 0;
    const vpy3d_edge *E = &PE[m->e0 + e];
    const int16_t *pa = MV(m, E->a), *pb = MV(m, E->b);
    for (int k = 0; k < 3; k++) { a[k] = pa[k]; b[k] = pb[k]; }
    return 1;
}

/* Find the edge (a,b) inside the mesh, or add it. */
static int edge_of(vpy_mesh *m, int a, int b, int face)
{
    int lo = a < b ? a : b, hi = a < b ? b : a;
    for (int e = 0; e < m->ne; e++) {
        vpy3d_edge *E = &PE[m->e0 + e];
        if (E->a == lo && E->b == hi) {
            if (E->f1 < 0) E->f1 = (int16_t)face;
            return e;
        }
    }
    if (nPE >= VPY3D_POOL_E) { s_stats.overflow++; return -1; }
    vpy3d_edge *E = &PE[nPE++];
    E->a = (uint16_t)lo; E->b = (uint16_t)hi;
    E->f0 = (int16_t)face; E->f1 = -1;
    E->ca = E->cb = -1; E->hard = 0;
    return m->ne++;
}

/* NORMALS, CREASES AND STRAIGHT RUNS, from the mesh's vertices as they are now.
 * mesh_end runs it once; vpy3d_mesh_dent runs it again after moving vertices,
 * because a dent changes all three: a flat face folds into a crease, and two
 * edges that were one straight line stop being one — merging them into a single
 * stroke would then draw a line that is not there. */
static void mesh_geometry(vpy_mesh *m)
{
    /* --- face normals, by Newell's method ---------------------------------
     * Newell rather than a cross product of the first three vertices: it uses
     * every vertex, so it survives a quad that is slightly non-planar and a
     * face whose first three corners are nearly in line. Wind anticlockwise
     * seen from outside and it points outward. */
    int64_t cx = 0, cy = 0, cz = 0;
    for (int i = 0; i < m->nv; i++) {
        const int16_t *v = MV(m, i);
        cx += v[0]; cy += v[1]; cz += v[2];
    }
    if (m->nv) { cx /= m->nv; cy /= m->nv; cz /= m->nv; }

    for (int f = 0; f < m->nf; f++) {
        int fi = m->f0 + f;
        int n = PFn[fi];
        const uint16_t *iv = &PFV[PFs[fi]];
        int64_t nx = 0, ny = 0, nz = 0, gx = 0, gy = 0, gz = 0;
        for (int i = 0; i < n; i++) {
            const int16_t *p = MV(m, iv[i]);
            const int16_t *q = MV(m, iv[(i + 1) % n]);
            nx += (int64_t)(p[1] - q[1]) * (p[2] + q[2]);
            ny += (int64_t)(p[2] - q[2]) * (p[0] + q[0]);
            nz += (int64_t)(p[0] - q[0]) * (p[1] + q[1]);
            gx += p[0]; gy += p[1]; gz += p[2];
        }
        gx /= n; gy /= n; gz /= n;
        int32_t u[3];
        if (!norm_q14(nx, ny, nz, u)) { u[0] = 0; u[1] = 0; u[2] = VPY3D_ONE; }
        /* A normal pointing back at the middle of the mesh is inside-out. This
         * rescues a face wound the wrong way; it cannot rescue one whose plane
         * runs through the centroid, so winding still matters. */
        int64_t outward = (int64_t)u[0] * (gx - cx)
                        + (int64_t)u[1] * (gy - cy)
                        + (int64_t)u[2] * (gz - cz);
        if (outward < 0) { u[0] = -u[0]; u[1] = -u[1]; u[2] = -u[2]; }
        PN[fi][0] = (int16_t)u[0]; PN[fi][1] = (int16_t)u[1]; PN[fi][2] = (int16_t)u[2];
    }

    /* --- which edges are creases ------------------------------------------ */
    for (int e = 0; e < m->ne; e++) {
        vpy3d_edge *E = &PE[m->e0 + e];
        if (E->f1 < 0) { E->hard = 1; continue; }        /* a rim is always a line */
        const int16_t *p = PN[m->f0 + E->f0], *q = PN[m->f0 + E->f1];
        int32_t dot = ((int32_t)p[0] * q[0] + (int32_t)p[1] * q[1]
                     + (int32_t)p[2] * q[2]) >> 14;
        E->hard = (uint8_t)(dot < m->hard_cos);
    }

    /* --- straight-run links ------------------------------------------------
     * Two edges meeting at a vertex continue one straight line when their
     * directions away from that vertex are exactly opposite. Exact integer
     * test, no tolerance: these are lattice points. If more than one candidate
     * meets there the run is ambiguous and stops, which is the safe answer. */
    for (int e = 0; e < m->ne; e++) {
        vpy3d_edge *E = &PE[m->e0 + e];
        for (int side = 0; side < 2; side++) {
            int v     = side ? E->b : E->a;
            int other = side ? E->a : E->b;
            const int16_t *pv = MV(m, v), *po = MV(m, other);
            int64_t d1[3] = { po[0]-pv[0], po[1]-pv[1], po[2]-pv[2] };
            int found = -1, count = 0;
            for (int g = 0; g < m->ne; g++) {
                if (g == e) continue;
                vpy3d_edge *G = &PE[m->e0 + g];
                int w;
                if      (G->a == v) w = G->b;
                else if (G->b == v) w = G->a;
                else continue;
                const int16_t *pw = MV(m, w);
                int64_t d2[3] = { pw[0]-pv[0], pw[1]-pv[1], pw[2]-pv[2] };
                int64_t cr[3]; cross64(d1, d2, cr);
                int64_t dp = d1[0]*d2[0] + d1[1]*d2[1] + d1[2]*d2[2];
                if (cr[0] == 0 && cr[1] == 0 && cr[2] == 0 && dp < 0) {
                    found = g; count++;
                }
            }
            int16_t link = (count == 1) ? (int16_t)found : (int16_t)-1;
            if (side) E->cb = link; else E->ca = link;
        }
    }

}

int vpy3d_mesh_end(int hard_cos_q14)
{
    if (!B) return 0;
    vpy_mesh *m = B;
    B = 0;
    uint32_t bad = s_stats.overflow;
    m->hard_cos = (int16_t)hard_cos_q14;

    /* --- the edge table, with the face either side ------------------------ */
    for (int f = 0; f < m->nf; f++) {
        int fi = m->f0 + f;
        int n = PFn[fi];
        const uint16_t *iv = &PFV[PFs[fi]];
        for (int i = 0; i < n; i++)
            if (edge_of(m, iv[i], iv[(i + 1) % n], f) < 0) break;
    }

    mesh_geometry(m);

    if (nPV > s_stats.verts)    s_stats.verts    = (uint16_t)nPV;
    if (nPF > s_stats.faces)    s_stats.faces    = (uint16_t)nPF;
    if (nPFV > s_stats.face_idx) s_stats.face_idx = (uint16_t)nPFV;
    if (nPE > s_stats.edges)    s_stats.edges    = (uint16_t)nPE;
    return s_stats.overflow == bad;
}

/* ── a mesh of its own, and dents in it ─────────────────────────────────── */
int vpy3d_mesh_copy(vpy_mesh *dst, const vpy_mesh *src)
{
    if (!dst || !src) return 0;
    const int same = dst->nv == src->nv && dst->nf == src->nf && dst->ne == src->ne
                  && dst->v0 != src->v0 && dst->nv;
    if (!same) {
        int fv = 0;
        for (int f = 0; f < src->nf; f++) fv += PFn[src->f0 + f];
        if (nPV + src->nv > VPY3D_POOL_V || nPF + src->nf > VPY3D_POOL_F ||
            nPFV + fv > VPY3D_POOL_FV || nPE + src->ne > VPY3D_POOL_E) {
            s_stats.overflow++;
            return 0;
        }
        dst->v0 = (uint16_t)nPV; nPV += src->nv;
        dst->f0 = (uint16_t)nPF; nPF += src->nf;
        dst->e0 = (uint16_t)nPE; nPE += src->ne;
        dst->nv = src->nv; dst->nf = src->nf; dst->ne = src->ne;
        for (int f = 0; f < src->nf; f++) {                    /* the faces' vertex lists */
            const int sf = src->f0 + f, df = dst->f0 + f;
            PFs[df] = (uint16_t)nPFV; PFn[df] = PFn[sf];
            for (int i = 0; i < PFn[sf]; i++) PFV[nPFV++] = PFV[PFs[sf] + i];
        }
        if (nPV > s_stats.verts) s_stats.verts = (uint16_t)nPV;
        if (nPF > s_stats.faces) s_stats.faces = (uint16_t)nPF;
        if (nPFV > s_stats.face_idx) s_stats.face_idx = (uint16_t)nPFV;
        if (nPE > s_stats.edges) s_stats.edges = (uint16_t)nPE;
    }
    /* reset (or fill) everything that depends on the shape */
    for (int i = 0; i < src->nv; i++)
        for (int k = 0; k < 3; k++) PV[dst->v0 + i][k] = PV[src->v0 + i][k];
    for (int f = 0; f < src->nf; f++)
        for (int k = 0; k < 3; k++) PN[dst->f0 + f][k] = PN[src->f0 + f][k];
    for (int e = 0; e < src->ne; e++) PE[dst->e0 + e] = PE[src->e0 + e];
    dst->open = src->open;
    dst->hard_cos = src->hard_cos;
    return 1;
}

void vpy3d_mesh_dent(vpy_mesh *m, int32_t px, int32_t py, int32_t pz,
                     int32_t dx, int32_t dy, int32_t dz, int32_t depth, int32_t radius)
{
    if (!m || radius <= 0 || depth == 0) return;
    const int64_t dl = isqrt64((int64_t)dx * dx + (int64_t)dy * dy + (int64_t)dz * dz);
    if (dl == 0) return;
    int moved = 0;
    for (int i = 0; i < m->nv; i++) {
        int16_t *v = PV[m->v0 + i];
        const int64_t ex = v[0] - px, ey = v[1] - py, ez = v[2] - pz;
        const int64_t d = isqrt64(ex * ex + ey * ey + ez * ez);
        if (d >= radius) continue;
        const int64_t push = (int64_t)depth * (radius - d) / radius;    /* falls off to the rim */
        const int64_t mv[3] = { dx * push / dl, dy * push / dl, dz * push / dl };
        for (int k = 0; k < 3; k++) {
            int64_t c = v[k] + mv[k];
            if (c > 32767) c = 32767;
            if (c < -32768) c = -32768;
            v[k] = (int16_t)c;
        }
        moved = 1;
    }
    if (moved) mesh_geometry(m);
}

int vpy3d_mesh_blend(vpy_mesh *dst, const vpy_mesh *a, const vpy_mesh *b, int32_t t_q14)
{
    if (!dst || !a || !b || a->nv != b->nv || a->nf != b->nf || dst->nv != a->nv || dst->v0 == a->v0) return 0;
    if (t_q14 < 0) t_q14 = 0;
    if (t_q14 > VPY3D_ONE) t_q14 = VPY3D_ONE;
    for (int i = 0; i < a->nv; i++)
        for (int k = 0; k < 3; k++) {
            const int32_t pa = PV[a->v0 + i][k], pb = PV[b->v0 + i][k];
            PV[dst->v0 + i][k] = (int16_t)(pa + (((int64_t)(pb - pa) * t_q14) >> 14));
        }
    mesh_geometry(dst);
    return 1;
}

void vpy3d_world_to_model(const vpy_xf *place, int32_t wx, int32_t wy, int32_t wz,
                          int32_t *mx, int32_t *my, int32_t *mz)
{
    /* the rotation is orthonormal, so its inverse is its transpose */
    const int64_t r[3] = { (int64_t)wx - place->t[0], (int64_t)wy - place->t[1], (int64_t)wz - place->t[2] };
    *mx = (int32_t)((place->m[0] * r[0] + place->m[3] * r[1] + place->m[6] * r[2]) >> 14);
    *my = (int32_t)((place->m[1] * r[0] + place->m[4] * r[1] + place->m[7] * r[2]) >> 14);
    *mz = (int32_t)((place->m[2] * r[0] + place->m[5] * r[1] + place->m[8] * r[2]) >> 14);
}

/* ── drawing a mesh ──────────────────────────────────────────────────────── */
void vpy3d_draw_mesh(const vpy_mesh *m, const vpy_xf *place, int br)
{
    if (m->nv > VPY3D_MESH_MAXV || m->nf > VPY3D_MESH_MAXF) { s_stats.overflow++; return; }
    const vpy_xf *cam = vpy3d_camera();
    vpy_xf mv = vpy3d_mul(cam, place);         /* model -> camera, in one go */

    for (int i = 0; i < m->nv; i++) {
        const int16_t *p = MV(m, i);
        for (int r = 0; r < 3; r++) {
            int32_t s = (mv.m[r*3+0] * p[0] + mv.m[r*3+1] * p[1] + mv.m[r*3+2] * p[2]) >> 14;
            TC[i][r] = s + mv.t[r];
        }
    }

    /* A face is turned toward us when its normal leans back at the camera —
     * the camera sits at the origin, so the vector to any of its vertices IS
     * the view direction and the sign of the dot product decides. */
    for (int f = 0; f < m->nf; f++) {
        const int16_t *n = PN[m->f0 + f];
        int64_t nx = (int64_t)(mv.m[0]*n[0] + mv.m[1]*n[1] + mv.m[2]*n[2]) >> 14;
        int64_t ny = (int64_t)(mv.m[3]*n[0] + mv.m[4]*n[1] + mv.m[5]*n[2]) >> 14;
        int64_t nz = (int64_t)(mv.m[6]*n[0] + mv.m[7]*n[1] + mv.m[8]*n[2]) >> 14;
        const int32_t *v = TC[PFV[PFs[m->f0 + f]]];
        FV[f] = (uint8_t)((nx*v[0] + ny*v[1] + nz*v[2]) < 0);
    }

    for (int e = 0; e < m->ne; e++) {
        const vpy3d_edge *E = &PE[m->e0 + e];
        int draw;
        if (E->f1 < 0)         draw = m->open ? 1 : FV[E->f0];   /* a rim */
        else if (E->hard)      draw = FV[E->f0] | FV[E->f1];     /* a crease */
        else                   draw = FV[E->f0] != FV[E->f1];    /* a silhouette */
        ES[e] = (uint8_t)(E->f1 < 0 || FV[E->f0] != FV[E->f1]);
        if (s_wire) draw = 1;             /* a cage keeps what the solid hides */
        ED[e] = (uint8_t)draw;
        if (!draw) s_stats.culled++;
    }

    /* Emit, folding straight runs into one stroke. A straight line in 3D
     * projects to a straight line in 2D, so this is exact, not an approximation
     * — and a stroke costs the beam the same whatever its length. */
    int ns = 0;
    for (int e = 0; e < m->ne; e++) {
        if (ED[e] != 1) continue;
        int c = e, v = PE[m->e0 + e].a;
        for (int g = 0; g < 64; g++) {               /* walk back to the start */
            const vpy3d_edge *C = &PE[m->e0 + c];
            int nb = (C->a == v) ? C->ca : C->cb;
            if (nb < 0 || ED[nb] != 1) break;
            const vpy3d_edge *N = &PE[m->e0 + nb];
            v = (N->a == v) ? N->b : N->a;
            c = nb;
        }
        int start = v;
        for (int g = 0; g < 64; g++) {               /* ...then on to the end */
            const vpy3d_edge *C = &PE[m->e0 + c];
            ED[c] = 2;
            if (g) s_stats.merged++;
            v = (C->a == v) ? C->b : C->a;
            int nb = (C->a == v) ? C->ca : C->cb;
            if (nb < 0 || ED[nb] != 1) break;
            c = nb;
        }
        if (ns < VPY3D_POOL_E) { RA[ns] = (uint16_t)start; RB[ns] = (uint16_t)v; RU[ns] = 0; ns++; }
    }

    /* One of the mesh's strokes, in camera space. Through the occluder when the
     * game asked for it, so a mesh drawn after a silhouette is cut by it like any
     * line; otherwise exactly what it always was. With occlusion on and no
     * occluder added this is still vpy3d_line_cam plus one compare. */
#define MESH_LINE(p, q) (s_mesh_occl ? vpy3d_occl_line_cam((p), (q), br) \
                                     : vpy3d_line_cam((p), (q), br))

    /* Put them in an order the BEAM likes. A stroke that starts where the last
     * one ended pays no blanked jump (~48 cycles), and — the part that is not
     * just cost — no re-centre either: the SDK re-zeroes the integrators before
     * any jump over uvm2_zero_jump, and on a small object nearly every jump is
     * over it. Emitting in edge-index order measured 1 of 9 strokes chained and
     * 78% of jumps forcing a re-zero, and because WHICH edges survive changes as
     * the object turns, that pattern changed every frame and the figure shook.
     *
     * This is not the nearest-neighbour reordering that is known to backfire: it
     * follows the mesh's own connectivity, which is exact — two runs meet or they
     * do not — so it cannot invent a join that is not there. */
    if (!s_chain) {
        for (int i = 0; i < ns; i++) MESH_LINE(TC[RA[i]], TC[RB[i]]);
        return;
    }
    int cur = -1;
    for (int done = 0; done < ns; done++) {
        int pick = -1, flip = 0;
        if (cur >= 0) {
            for (int i = 0; i < ns; i++) {
                if (RU[i]) continue;
                if (RA[i] == cur) { pick = i; flip = 0; break; }
                if (RB[i] == cur) { pick = i; flip = 1; break; }
            }
        }
        if (pick < 0) {                       /* nothing meets the beam: lift it */
            for (int i = 0; i < ns; i++) if (!RU[i]) { pick = i; flip = 0; break; }
            if (pick < 0) break;
        } else {
            s_stats.chained++;
        }
        RU[pick] = 1;
        int a = flip ? RB[pick] : RA[pick];
        int b = flip ? RA[pick] : RB[pick];
        /* A run that loses its start to an occluder no longer begins where the
         * beam is, so `chained` counts the joins the mesh offers, not the ones
         * that survived the cut. */
        MESH_LINE(TC[a], TC[b]);
        cur = b;
    }
#undef MESH_LINE
}

/* ── loading a compiled .vmesh ───────────────────────────────────────────
 * The layout is in vmeshres.rs; it is little-endian and byte-packed, so it is
 * read a byte at a time rather than cast over — the image may sit at any
 * alignment in ROM and a Cortex-M would not forgive a misaligned i16 load
 * through a struct pointer. */
static vpy3d_error_t s_err;
vpy3d_error_t vpy3d_error(void) { return s_err; }

static uint16_t rd16(const unsigned char *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  rds16(const unsigned char *p) { return (int16_t)rd16(p); }

int vpy3d_load_mesh(vpy_mesh *m, const unsigned char *d)
{
    s_err = VPY3D_OK;
    if (!d || d[0] != 'V' || d[1] != 'M' || d[2] != 'S' || d[3] != 'H') {
        s_err = VPY3D_ERR_MAGIC; return 0;
    }
    if (d[4] != 1) { s_err = VPY3D_ERR_VERSION; return 0; }

    int      open      = d[5] & 1;
    int      hard_cos  = rds16(d + 6);
    unsigned nv        = rd16(d + 8);
    unsigned nf        = rd16(d + 10);
    unsigned nidx      = rd16(d + 12);

    uint32_t before = s_stats.overflow;
    vpy3d_mesh_begin(m);

    const unsigned char *v = d + 16;
    for (unsigned i = 0; i < nv; i++, v += 6)
        if (vpy3d_vertex(rds16(v), rds16(v + 2), rds16(v + 4)) < 0) {
            s_err = VPY3D_ERR_POOL; B = 0; return 0;
        }

    const unsigned char *f = v;
    unsigned seen = 0;
    int idx[VPY3D_MESH_MAXV];
    for (unsigned i = 0; i < nf; i++) {
        unsigned n = *f++;
        if (n < 3 || n > VPY3D_MESH_MAXV || seen + n > nidx) {
            s_err = VPY3D_ERR_DATA; B = 0; return 0;
        }
        for (unsigned k = 0; k < n; k++, f += 2) {
            unsigned vi = rd16(f);
            if (vi >= nv) { s_err = VPY3D_ERR_DATA; B = 0; return 0; }
            idx[k] = (int)vi;
        }
        seen += n;
        if (vpy3d_face(idx, (int)n) < 0) { s_err = VPY3D_ERR_POOL; B = 0; return 0; }
    }
    if (seen != nidx) { s_err = VPY3D_ERR_DATA; B = 0; return 0; }

    if (!vpy3d_mesh_end(hard_cos)) {
        s_err = (s_stats.overflow != before) ? VPY3D_ERR_POOL : VPY3D_ERR_DATA;
        return 0;
    }
    vpy3d_mesh_open(m, open);
    return 1;
}

void vpy3d_set_wire(int on) { s_wire = on ? 1 : 0; }
int  vpy3d_get_wire(void)   { return s_wire; }

void vpy3d_set_mesh_occlusion(int on) { s_mesh_occl = on ? 1 : 0; }
int  vpy3d_get_mesh_occlusion(void)   { return s_mesh_occl; }
void vpy3d_set_chaining(int on) { s_chain = on ? 1 : 0; }
int  vpy3d_get_chaining(void)    { return s_chain; }

/* ── counters ────────────────────────────────────────────────────────────── */
const vpy3d_stats_t *vpy3d_stats(void) { return &s_stats; }
void vpy3d_reset_counts(void)
{
    s_stats.strokes = 0; s_stats.culled = 0; s_stats.merged = 0;
    s_stats.occl_cut = 0; s_stats.occl_refused = 0; s_stats.occl_full = 0;
}

/* ── one solid hiding another ─────────────────────────────────────────────────
 *
 * Everything above removes hidden lines WITHIN a mesh: it knows which of a
 * cube's own faces are turned away and drops their edges. It knows nothing
 * about a second cube in front of the first, and on a display with no depth
 * buffer that means the second cube is not there at all — you see straight
 * through it. Every game with more than one solid on screen has this, and none
 * of them will find it by reading the code: it looks like "the picture is
 * dirty".
 *
 * So: a screen-space occluder. A convex solid's silhouette is the convex hull
 * of its projected corners, and a line behind it is that line minus the part
 * inside the hull. Both cheap, both exact for convex occluders, and neither
 * needs anything this module does not already have.
 *
 * WITH NO OCCLUDERS ADDED vpy3d_occl_line IS vpy3d_line_world and costs one
 * compare, which is why every stroke in a game can go through it.
 *
 * It came from kuroishi (2026-09-24), where the symptom was "the waves look
 * transparent while they move", and it lived in that game until hakaba needed
 * exactly the same thing. A second copy is how two versions of one algorithm
 * start to disagree; this is the third place it would have been written out.
 */

/* THE TABLE, and it is a budget rather than a count: past it nothing more hides
 * anything, which is the picture we had before this code existed and never a
 * wrong one. Add the nearest solids first — they hide the most. */
#define OCC_MAX 64
/* The silhouette of a box is a hexagon. The hull drops collinear points (the
 * `<= 0` in occ_hull_of), so eight corners give at most eight vertices; ten is
 * slack, and occ_hull_of refuses rather than overrun it. */
#define OCC_MAXV 10

typedef struct {
    int32_t x[OCC_MAXV], y[OCC_MAXV];
    int     n;
    int32_t bx0, by0, bx1, by1;     /* screen bounds: the cheap reject */
    int32_t znear;                  /* camera z of its nearest corner */
} occ_hull;

static occ_hull s_occ[OCC_MAX];
static int      s_nocc;

void vpy3d_occl_reset(void) { s_nocc = 0; }
int  vpy3d_occl_count(void) { return s_nocc; }

static int64_t occ_cross(int32_t ox, int32_t oy, int32_t ax, int32_t ay,
                         int32_t bx, int32_t by)
{
    return (int64_t)(ax - ox) * (by - oy) - (int64_t)(ay - oy) * (bx - ox);
}

/* Andrew's monotone chain, on at most eight points. Anticlockwise, which is
 * what the clipper below assumes when it takes an edge's outward normal. */
static int occ_hull_of(occ_hull *h, int32_t *px, int32_t *py, int n)
{
    /* sort by x then y — insertion, because n is eight */
    for (int i = 1; i < n; i++) {
        const int32_t x = px[i], y = py[i];
        int j = i - 1;
        while (j >= 0 && (px[j] > x || (px[j] == x && py[j] > y))) {
            px[j + 1] = px[j]; py[j + 1] = py[j]; j--;
        }
        px[j + 1] = x; py[j + 1] = y;
    }
    int32_t hx[OCC_MAXV * 2], hy[OCC_MAXV * 2];
    int k = 0;
    for (int i = 0; i < n; i++) {                       /* lower */
        while (k >= 2 && occ_cross(hx[k-2], hy[k-2], hx[k-1], hy[k-1], px[i], py[i]) <= 0) k--;
        if (k >= OCC_MAXV * 2) return 0;
        hx[k] = px[i]; hy[k] = py[i]; k++;
    }
    for (int i = n - 2, t = k + 1; i >= 0; i--) {       /* upper */
        while (k >= t && occ_cross(hx[k-2], hy[k-2], hx[k-1], hy[k-1], px[i], py[i]) <= 0) k--;
        if (k >= OCC_MAXV * 2) return 0;
        hx[k] = px[i]; hy[k] = py[i]; k++;
    }
    k--;                                                 /* the first point twice */
    if (k < 3 || k > OCC_MAXV) return 0;                 /* degenerate, or too many */
    h->n = k;
    h->bx0 = h->bx1 = hx[0]; h->by0 = h->by1 = hy[0];
    for (int i = 0; i < k; i++) {
        h->x[i] = hx[i]; h->y[i] = hy[i];
        if (hx[i] < h->bx0) h->bx0 = hx[i];
        if (hx[i] > h->bx1) h->bx1 = hx[i];
        if (hy[i] < h->by0) h->by0 = hy[i];
        if (hy[i] > h->by1) h->by1 = hy[i];
    }
    return 1;
}

int vpy3d_occl_add(const int32_t (*corners)[3], int n)
{
    if (s_nocc >= OCC_MAX) { s_stats.occl_full++; return 0; }
    if (n < 3 || n > 8) { s_stats.occl_refused++; return 0; }
    int32_t px[8], py[8];
    int32_t znear = 0x7fffffff;
    for (int i = 0; i < n; i++) {
        int32_t cam[3];
        vpy3d_to_camera(corners[i][0], corners[i][1], corners[i][2], cam);
        if (cam[2] < znear) znear = cam[2];
        /* A corner behind the near plane makes the hull meaningless — the
         * silhouette of a solid the camera is inside of is not a polygon. Take
         * the whole occluder out rather than clip it: it is one frame of the
         * picture we already had, never a wrong one. */
        if (!vpy3d_project(cam, &px[i], &py[i])) { s_stats.occl_refused++; return 0; }
    }
    if (!occ_hull_of(&s_occ[s_nocc], px, py, n)) { s_stats.occl_refused++; return 0; }
    s_occ[s_nocc].znear = znear;
    s_nocc++;
    return 1;
}

int vpy3d_occl_add_mesh(const vpy_mesh *m, const vpy_xf *place)
{
    /* The mesh's own vertices, placed, as the corner set. Only the HULL of them
     * matters, so a mesh of more than eight vertices cannot go through here —
     * and that is the honest limit rather than a silent approximation: give it
     * the eight corners of the box you mean. */
    if (!m || m->nv < 3 || m->nv > 8) { s_stats.occl_refused++; return 0; }
    int32_t c[8][3];
    for (int i = 0; i < m->nv; i++) {
        const int16_t *p = MV(m, i);                     /* model space */
        for (int r = 0; r < 3; r++)
            c[i][r] = ((place->m[r*3+0] * p[0] + place->m[r*3+1] * p[1] +
                        place->m[r*3+2] * p[2]) >> 14) + place->t[r];
    }
    return vpy3d_occl_add(c, m->nv);
}

/* Q16 along the segment. */
#define OCC_ONE_T 65536
/* A piece shorter than this is not a line, it is a dot: the beam pays a blanked
 * jump to reach it and then barely moves. Slivers left along an occluder's own
 * edge are exactly that, and they are the least trustworthy part of the answer
 * anyway — the hull is the silhouette to within a pixel, not to within nothing. */
#define OCC_MIN_T (OCC_ONE_T / 48)

/* Where the segment is INSIDE the hull, as [ta, tb] in Q16. Cyrus-Beck: clip
 * the parameter against each edge's half plane and what survives is the inside.
 * 0 if it never enters. */
static int occ_inside_span(const occ_hull *h, int64_t px, int64_t py,
                           int64_t dx, int64_t dy, int32_t *ta, int32_t *tb)
{
    int64_t t0 = 0, t1 = OCC_ONE_T;
    for (int i = 0; i < h->n; i++) {
        const int j = (i + 1 == h->n) ? 0 : i + 1;
        /* anticlockwise, so the outward normal of A->B is (dy, -dx) */
        const int64_t nx = h->y[j] - h->y[i], ny = -(int64_t)(h->x[j] - h->x[i]);
        const int64_t den = nx * dx + ny * dy;
        const int64_t num = nx * (px - h->x[i]) + ny * (py - h->y[i]);
        if (den == 0) { if (num > 0) return 0; continue; }   /* parallel, outside */
        const int64_t t = (-num * OCC_ONE_T) / den;
        if (den < 0) { if (t > t0) t0 = t; }                 /* coming in */
        else         { if (t < t1) t1 = t; }                 /* going out */
        if (t0 >= t1) return 0;
    }
    *ta = (int32_t)t0; *tb = (int32_t)t1;
    return 1;
}

/* The camera-space point at SCREEN fraction t.
 *
 * NOT the camera-space point at fraction t: a projection is not linear, and
 * lerping the camera points by the screen t would put the cut in the wrong
 * place — most wrongly on the longest strokes, which are the ones that most
 * need cutting. The perspective-correct inverse is
 *   u = t * z0 / (z1 + t * (z0 - z1))
 * and handing camera points back to vpy3d_line_cam is what keeps the near
 * plane, the screen clip and the stroke count in ONE place. */
static void occ_at_t(const int32_t *a, const int32_t *b, int32_t t, int32_t *out)
{
    const int64_t z0 = a[2], z1 = b[2];
    const int64_t den = z1 * OCC_ONE_T + (int64_t)t * (z0 - z1);
    const int64_t u = den ? ((int64_t)t * z0 * OCC_ONE_T) / den : t;
    for (int i = 0; i < 3; i++)
        out[i] = (int32_t)(a[i] + (((int64_t)(b[i] - a[i]) * u) >> 16));
}

void vpy3d_occl_line(int32_t ax, int32_t ay, int32_t az,
                     int32_t bx, int32_t by, int32_t bz, int br)
{
    int32_t a[3], b[3];
    vpy3d_to_camera(ax, ay, az, a);
    vpy3d_to_camera(bx, by, bz, b);
    vpy3d_occl_line_cam(a, b, br);
}

void vpy3d_occl_line_cam(const int32_t *ca, const int32_t *cb, int br)
{
    if (s_nocc == 0) { vpy3d_line_cam(ca, cb, br); return; } /* nothing to hide behind */
    /* Copies: the near-plane cut below moves the ends, and the caller's points
     * (a mesh's transformed vertices among them) are shared with other edges. */
    int32_t a[3] = { ca[0], ca[1], ca[2] }, b[3] = { cb[0], cb[1], cb[2] };

    /* CUT AT THE NEAR PLANE FIRST, with vpy3d_line_cam's own arithmetic, so the
     * pieces handed back to it are the ones it would have drawn anyway. Without
     * this a line with an end behind the camera could not be projected, and it
     * was drawn whole straight through every occluder — a ground line starting
     * under the camera is the common case, and the commonest thing to hide. */
    if (a[2] < s_near && b[2] < s_near) return;      /* all of it is behind us */
    if (a[2] < s_near) {
        const int64_t k = ((int64_t)(s_near - a[2]) << 16) / ((int64_t)b[2] - a[2]);
        a[0] += (int32_t)((((int64_t)b[0] - a[0]) * k) >> 16);
        a[1] += (int32_t)((((int64_t)b[1] - a[1]) * k) >> 16);
        a[2] = s_near;
    }
    if (b[2] < s_near) {
        const int64_t k = ((int64_t)(s_near - b[2]) << 16) / ((int64_t)a[2] - b[2]);
        b[0] += (int32_t)((((int64_t)a[0] - b[0]) * k) >> 16);
        b[1] += (int32_t)((((int64_t)a[1] - b[1]) * k) >> 16);
        b[2] = s_near;
    }
    int32_t s0x = 0, s0y = 0, s1x = 0, s1y = 0;
    /* both ends are at or past the near plane now, so both project; if one ever
     * did not, the line is drawn whole rather than cut with garbage */
    if (!vpy3d_project(a, &s0x, &s0y) || !vpy3d_project(b, &s1x, &s1y)) { vpy3d_line_cam(a, b, br); return; }
    const int64_t dx = (int64_t)s1x - s0x, dy = (int64_t)s1y - s0y;

    /* Every span the line spends inside an occluder, then the gaps between
     * them. Collected first and sorted, because two occluders can overlap and
     * cutting them one at a time would re-draw what the other already hid. */
    int32_t sa[OCC_MAX], sb[OCC_MAX];
    int ns = 0;
    for (int i = 0; i < s_nocc; i++) {
        const occ_hull *h = &s_occ[i];
        /* IN FRONT OF THE WHOLE SOLID: it cannot be behind it. Not a depth
         * test — there is none here, and there is no need for one while the
         * caller draws near to far — but the one case a silhouette gets
         * catastrophically wrong is a thing standing in front of it, so the
         * cheapest guard against being used out of order lives here. */
        if (a[2] < h->znear && b[2] < h->znear) continue;
        if ((s0x < h->bx0 && s1x < h->bx0) || (s0x > h->bx1 && s1x > h->bx1) ||
            (s0y < h->by0 && s1y < h->by0) || (s0y > h->by1 && s1y > h->by1)) continue;
        int32_t t0, t1;
        if (!occ_inside_span(h, s0x, s0y, dx, dy, &t0, &t1)) continue;
        if (t0 < 0) t0 = 0;
        if (t1 > OCC_ONE_T) t1 = OCC_ONE_T;
        if (t1 <= t0) continue;
        int k = ns++;
        while (k > 0 && sa[k - 1] > t0) { sa[k] = sa[k-1]; sb[k] = sb[k-1]; k--; }
        sa[k] = t0; sb[k] = t1;
    }
    if (ns == 0) { vpy3d_line_cam(a, b, br); return; }
    s_stats.occl_cut++;

    int32_t cur = 0;
    for (int i = 0; i < ns && cur < OCC_ONE_T; i++) {
        if (sa[i] - cur > OCC_MIN_T) {
            int32_t p[3], q[3];
            occ_at_t(a, b, cur, p); occ_at_t(a, b, sa[i], q);
            vpy3d_line_cam(p, q, br);
        }
        if (sb[i] > cur) cur = sb[i];
    }
    if (OCC_ONE_T - cur > OCC_MIN_T) {
        int32_t p[3];
        occ_at_t(a, b, cur, p);
        vpy3d_line_cam(p, b, br);
    }
}

/* ── shading the world: depth, shadows, size ─────────────────────────────── */
int vpy3d_fog(int br, int32_t x, int32_t y, int32_t z, int32_t full_until, int32_t gone_at)
{
    int32_t c[3];
    vpy3d_to_camera(x, y, z, c);
    const int32_t d = c[2];                      /* depth: what perspective divides by */
    if (d <= full_until) return br;
    if (d >= gone_at || gone_at <= full_until) return 0;
    return (int)((int64_t)br * (gone_at - d) / (gone_at - full_until));
}

int vpy3d_shadow(const int32_t (*corners)[3], int n, int32_t lx, int32_t ly, int32_t lz,
                 int32_t floor_y, int br)
{
    if (n < 3 || n > 8 || ly >= 0) return 0;     /* the light has to come down */
    int32_t px[8], pz[8];
    for (int i = 0; i < n; i++) {
        /* slide the corner down the light ray to the floor */
        const int64_t h = (int64_t)corners[i][1] - floor_y;
        if (h < 0) return 0;                     /* a corner below the floor: no shadow */
        px[i] = (int32_t)(corners[i][0] + (int64_t)lx * h / -ly);
        pz[i] = (int32_t)(corners[i][2] + (int64_t)lz * h / -ly);
    }
    occ_hull hull;
    if (!occ_hull_of(&hull, px, pz, n)) return 0;
    /* a hair above the floor, so the floor's own lines do not fight it */
    const int32_t y = floor_y + 2;
    for (int i = 0; i < hull.n; i++) {
        const int j = (i + 1 == hull.n) ? 0 : i + 1;
        vpy3d_occl_line(hull.x[i], y, hull.y[i], hull.x[j], y, hull.y[j], br);
    }
    return hull.n;
}

int32_t vpy3d_screen_size(int32_t x, int32_t y, int32_t z, int32_t radius)
{
    int32_t c[3];
    vpy3d_to_camera(x, y, z, c);
    if (c[2] < s_near) return 0;
    return (int32_t)((int64_t)radius * s_focal / c[2]);
}

/* ── terrain, by the floating horizon ────────────────────────────────────── */
#ifndef VPY3D_HORIZON_COLS
#define VPY3D_HORIZON_COLS 256     /* columns across the screen the horizon is kept in */
#endif
static int32_t s_hz[VPY3D_HORIZON_COLS];
/* THE ROW RAISES THE HORIZON WHEN IT IS DONE, not segment by segment. Raising it
 * as each segment went made a row hide bits of ITSELF where its segments meet —
 * rounding at the shared column — and the lines came out with small gaps. A row
 * hides what is behind it, never itself. */
static int32_t s_row[VPY3D_HORIZON_COLS];

static int hz_col(int64_t x)        /* screen x -> horizon column, clamped */
{
    const int64_t c = (x + s_clip_x) * (VPY3D_HORIZON_COLS - 1) / (2 * (int64_t)s_clip_x);
    return c < 0 ? 0 : (c >= VPY3D_HORIZON_COLS ? VPY3D_HORIZON_COLS - 1 : (int)c);
}

/* One segment of a row, on screen: the parts above the horizon are drawn, then
 * the horizon rises to it. Returns strokes drawn. */
static int hz_segment(int64_t x0, int64_t y0, int64_t x1, int64_t y1, int br)
{
    if (x1 < x0) { int64_t t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
    const int c0 = hz_col(x0), c1 = hz_col(x1);
    int strokes = 0, run = -1;
    int64_t rx = 0, ry = 0;
    for (int c = c0; c <= c1 + 1; c++) {
        int vis = 0; int64_t x = 0, y = 0;
        if (c <= c1) {
            /* this column's x, and the segment's y there */
            x = c == c0 ? x0 : (c == c1 ? x1 : -(int64_t)s_clip_x + (int64_t)c * 2 * s_clip_x / (VPY3D_HORIZON_COLS - 1));
            y = x1 == x0 ? (y0 > y1 ? y0 : y1) : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
            vis = y >= s_hz[c];
        }
        if (vis && run < 0) { run = c; rx = x; ry = y; }
        if (!vis && run >= 0) {
            /* the run ended at the previous column */
            const int pc = c - 1;
            const int64_t px = pc == c1 ? x1 : -(int64_t)s_clip_x + (int64_t)pc * 2 * s_clip_x / (VPY3D_HORIZON_COLS - 1);
            const int64_t py = x1 == x0 ? y1 : y0 + (y1 - y0) * (px - x0) / (x1 - x0);
            if (px != rx || py != ry) { line_screen(rx, ry, px, py, br); strokes++; }
            run = -1;
        }
    }
    for (int c = c0; c <= c1; c++) {
        const int64_t x = -(int64_t)s_clip_x + (int64_t)c * 2 * s_clip_x / (VPY3D_HORIZON_COLS - 1);
        const int64_t xc = x < x0 ? x0 : (x > x1 ? x1 : x);
        const int64_t y = x1 == x0 ? (y0 > y1 ? y0 : y1) : y0 + (y1 - y0) * (xc - x0) / (x1 - x0);
        if (y > s_row[c]) s_row[c] = (int32_t)y;
    }
    return strokes;
}

int vpy3d_terrain(const int16_t *h, int cols, int rows, int32_t x0, int32_t z0, int32_t cell, int br)
{
    if (!h || cols < 2 || rows < 1) return 0;
    for (int c = 0; c < VPY3D_HORIZON_COLS; c++) s_hz[c] = INT32_MIN;
    /* near to far: the row whose middle is nearest the camera first */
    int32_t mid0[3], mid1[3];
    vpy3d_to_camera(x0 + cell * (cols - 1) / 2, 0, z0, mid0);
    vpy3d_to_camera(x0 + cell * (cols - 1) / 2, 0, z0 + cell * (rows - 1), mid1);
    const int forward = mid0[2] <= mid1[2];
    int strokes = 0;
    for (int k = 0; k < rows; k++) {
        const int r = forward ? k : rows - 1 - k;
        int64_t px = 0, py = 0; int have = 0;
        for (int c = 0; c < VPY3D_HORIZON_COLS; c++) s_row[c] = INT32_MIN;
        for (int c = 0; c < cols; c++) {
            int32_t cam[3], sx, sy;
            vpy3d_to_camera(x0 + cell * c, h[r * cols + c], z0 + cell * r, cam);
            if (!vpy3d_project(cam, &sx, &sy)) { have = 0; continue; }   /* behind us: break the row */
            if (have) strokes += hz_segment(px, py, sx, sy, br);
            px = sx; py = sy; have = 1;
        }
        for (int c = 0; c < VPY3D_HORIZON_COLS; c++) if (s_row[c] > s_hz[c]) s_hz[c] = s_row[c];
    }
    return strokes;
}
