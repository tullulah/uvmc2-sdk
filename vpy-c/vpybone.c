/*
 * vpybone.c — see vpybone.h. Quaternions Q14; positions in world units.
 */
#include "vpybone.h"
#include "vpy.h"
#include "vpyik.h"

#define N1 16384

static vpyb_stats_t s_stats;
const vpyb_stats_t *vpyb_stats(void) { return &s_stats; }
void vpyb_reset_stats(void) { vpyb_stats_t z = { 0 }; s_stats = z; }

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

/* ── quaternions, Q14 ──────────────────────────────────────────────────────── */
vpyb_quat vpyb_quat_identity(void) { vpyb_quat q = { N1, 0, 0, 0 }; return q; }

/* back to unit length: worked in Q28 so the root keeps 14 bits */
static vpyb_quat qnorm(int64_t w, int64_t x, int64_t y, int64_t z)
{
    const int64_t l = isqrt64(w * w + x * x + y * y + z * z);
    vpyb_quat q;
    if (l == 0) return vpyb_quat_identity();
    q.w = (int32_t)(w * N1 / l); q.x = (int32_t)(x * N1 / l);
    q.y = (int32_t)(y * N1 / l); q.z = (int32_t)(z * N1 / l);
    return q;
}

vpyb_quat vpyb_quat_axis(int32_t ax, int32_t ay, int32_t az, int angle)
{
    const int64_t l = isqrt64((int64_t)ax * ax + (int64_t)ay * ay + (int64_t)az * az);
    if (l == 0) return vpyb_quat_identity();
    const int64_t s = vpy_sin_q14(angle / 2), c = vpy_cos_q14(angle / 2);
    return qnorm(c * l, s * ax, s * ay, s * az);
}

static vpyb_quat qmul(vpyb_quat a, vpyb_quat b)
{
    const int64_t w = (int64_t)a.w * b.w - (int64_t)a.x * b.x - (int64_t)a.y * b.y - (int64_t)a.z * b.z;
    const int64_t x = (int64_t)a.w * b.x + (int64_t)a.x * b.w + (int64_t)a.y * b.z - (int64_t)a.z * b.y;
    const int64_t y = (int64_t)a.w * b.y - (int64_t)a.x * b.z + (int64_t)a.y * b.w + (int64_t)a.z * b.x;
    const int64_t z = (int64_t)a.w * b.z + (int64_t)a.x * b.y - (int64_t)a.y * b.x + (int64_t)a.z * b.w;
    /* renormalised every product: a long chain of bones otherwise drifts off
     * unit length one rounding at a time */
    return qnorm(w, x, y, z);
}
static vpyb_quat qconj(vpyb_quat q) { q.x = -q.x; q.y = -q.y; q.z = -q.z; return q; }

/* v turned by q: v + 2w(q×v) + 2 q×(q×v) */
static void qrot(vpyb_quat q, const int32_t v[3], int32_t out[3])
{
    const int64_t vx = v[0], vy = v[1], vz = v[2];
    /* t = 2 (q×v) is Q14 × units; w t and q×t are then both Q28 × units */
    const int64_t tx = 2 * (q.y * vz - q.z * vy), ty = 2 * (q.z * vx - q.x * vz), tz = 2 * (q.x * vy - q.y * vx);
    const int64_t cx = q.y * tz - q.z * ty, cy = q.z * tx - q.x * tz, cz = q.x * ty - q.y * tx;
    const int64_t h = (int64_t)1 << 27;                   /* round to nearest */
    out[0] = (int32_t)(vx + ((q.w * tx + cx + h) >> 28));
    out[1] = (int32_t)(vy + ((q.w * ty + cy + h) >> 28));
    out[2] = (int32_t)(vz + ((q.w * tz + cz + h) >> 28));
}

static void qmatrix(vpyb_quat q, int32_t m[9])
{
    const int64_t w = q.w, x = q.x, y = q.y, z = q.z;
    m[0] = (int32_t)(N1 - ((y * y + z * z) >> 13)); m[1] = (int32_t)((x * y - w * z) >> 13); m[2] = (int32_t)((x * z + w * y) >> 13);
    m[3] = (int32_t)((x * y + w * z) >> 13); m[4] = (int32_t)(N1 - ((x * x + z * z) >> 13)); m[5] = (int32_t)((y * z - w * x) >> 13);
    m[6] = (int32_t)((x * z - w * y) >> 13); m[7] = (int32_t)((y * z + w * x) >> 13); m[8] = (int32_t)(N1 - ((x * x + y * y) >> 13));
}

/* a to b, `f` of the way (Q8), the short way round */
static vpyb_quat nlerp(vpyb_quat a, vpyb_quat b, int32_t f)
{
    const int64_t d = (int64_t)a.w * b.w + (int64_t)a.x * b.x + (int64_t)a.y * b.y + (int64_t)a.z * b.z;
    if (d < 0) { b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z; }
    const int64_t g = 256 - f;
    return qnorm(a.w * g + b.w * f, a.x * g + b.x * f, a.y * g + b.y * f, a.z * g + b.z * f);
}

/* the shortest turn that takes direction u onto direction v */
static vpyb_quat from_to(const int32_t u[3], const int32_t v[3])
{
    const int64_t lu = isqrt64((int64_t)u[0] * u[0] + (int64_t)u[1] * u[1] + (int64_t)u[2] * u[2]);
    const int64_t lv = isqrt64((int64_t)v[0] * v[0] + (int64_t)v[1] * v[1] + (int64_t)v[2] * v[2]);
    if (!lu || !lv) return vpyb_quat_identity();
    int64_t a[3], b[3];
    for (int k = 0; k < 3; k++) { a[k] = (int64_t)u[k] * N1 / lu; b[k] = (int64_t)v[k] * N1 / lv; }
    const int64_t d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) >> 14;
    if (d < -N1 + 16) {
        /* opposite: half a turn about any axis square to u */
        const int64_t ax = a[0] < 0 ? -a[0] : a[0], ay = a[1] < 0 ? -a[1] : a[1];
        if (ax < ay) return qnorm(0, 0, -a[2], a[1]);
        return qnorm(0, -a[2], 0, a[0]);
    }
    return qnorm(N1 + d, (a[1] * b[2] - a[2] * b[1]) >> 14, (a[2] * b[0] - a[0] * b[2]) >> 14, (a[0] * b[1] - a[1] * b[0]) >> 14);
}

/* ── the skeleton ──────────────────────────────────────────────────────────── */
void vpyb_init(vpyb_skeleton *s)
{
    s->n = 0;
    s->root_rot = vpyb_quat_identity();
    s->root_pos[0] = s->root_pos[1] = s->root_pos[2] = 0;
}

int vpyb_add(vpyb_skeleton *s, int parent, int32_t ox, int32_t oy, int32_t oz, const vpy_mesh *mesh)
{
    /* the parent must already be there: that keeps the table in parent-first order,
     * which is what lets forward kinematics be one pass */
    if (s->n >= VPYB_MAX_BONES || parent >= s->n || parent < -1) { s_stats.refused++; return -1; }
    const int i = s->n++;
    s->parent[i] = (int8_t)parent;
    s->off[i][0] = ox; s->off[i][1] = oy; s->off[i][2] = oz;
    s->mesh[i] = mesh;
    s->rot[i] = vpyb_quat_identity();
    s->wrot[i] = vpyb_quat_identity();
    s->pos[i][0] = s->pos[i][1] = s->pos[i][2] = 0;
    return i;
}

void vpyb_pose(vpyb_skeleton *s, const int32_t pos[3], vpyb_quat rot)
{
    s->root_rot = rot;
    for (int k = 0; k < 3; k++) s->root_pos[k] = pos[k];
    for (int i = 0; i < s->n; i++) {
        const int p = s->parent[i];
        const vpyb_quat pr = p < 0 ? rot : s->wrot[p];
        const int32_t *pp = p < 0 ? pos : s->pos[p];
        int32_t o[3];
        qrot(pr, s->off[i], o);
        for (int k = 0; k < 3; k++) s->pos[i][k] = pp[k] + o[k];
        s->wrot[i] = qmul(pr, s->rot[i]);
    }
    s_stats.bones = s->n;
}

vpy_xf vpyb_xf(const vpyb_skeleton *s, int bone)
{
    vpy_xf x = vpy3d_identity();
    if (bone < 0 || bone >= s->n) return x;
    qmatrix(s->wrot[bone], x.m);
    for (int k = 0; k < 3; k++) x.t[k] = s->pos[bone][k];
    return x;
}

int vpyb_draw(const vpyb_skeleton *s, int br, int what)
{
    int n = 0;
    for (int i = 0; i < s->n; i++) {
        int drew = 0;
        if ((what & VPYB_DRAW_MESH) && s->mesh[i]) {
            const vpy_xf x = vpyb_xf(s, i);
            vpy3d_draw_mesh(s->mesh[i], &x, br);
            drew = 1;
        }
        if ((what & VPYB_DRAW_LINES) && s->parent[i] >= 0) {
            const int32_t *a = s->pos[s->parent[i]], *b = s->pos[i];
            vpy3d_line_world(a[0], a[1], a[2], b[0], b[1], b[2], br);
            drew = 1;
        }
        n += drew;
    }
    return n;
}

/* ── clips ─────────────────────────────────────────────────────────────────── */
vpyb_quat vpyb_sample(const vpyb_track *tr, int32_t t_q8, uint16_t length, int loop)
{
    if (!tr || tr->n == 0) return vpyb_quat_identity();
    const vpyb_key *k = tr->keys;
    const int n = tr->n;
    const int32_t len = (int32_t)length << 8;
    if (loop && len > 0) { t_q8 %= len; if (t_q8 < 0) t_q8 += len; }
    s_stats.last_t_q8 = t_q8;
    if (n == 1) return k[0].q;
    const int32_t first = (int32_t)k[0].frame << 8, last = (int32_t)k[n - 1].frame << 8;
    if (t_q8 < first || t_q8 >= last) {
        if (!loop) return t_q8 < first ? k[0].q : k[n - 1].q;
        /* the stretch that wraps: from the last key round to the first, a loop later */
        const int32_t span = len - last + first;
        if (span <= 0) return k[n - 1].q;
        const int32_t into = t_q8 >= last ? t_q8 - last : t_q8 + len - last;
        return nlerp(k[n - 1].q, k[0].q, (int32_t)((int64_t)into * 256 / span));
    }
    int i = 0;
    while (i + 1 < n && ((int32_t)k[i + 1].frame << 8) <= t_q8) i++;
    const int32_t a = (int32_t)k[i].frame << 8, b = (int32_t)k[i + 1].frame << 8;
    if (b <= a) return k[i].q;
    return nlerp(k[i].q, k[i + 1].q, (int32_t)((int64_t)(t_q8 - a) * 256 / (b - a)));
}

void vpyb_apply(vpyb_skeleton *s, const vpyb_clip *c, int32_t t_q8)
{
    for (int i = 0; i < s->n && i < c->bones; i++)
        if (c->tracks[i].n) s->rot[i] = vpyb_sample(&c->tracks[i], t_q8, c->length, c->loop);
}

void vpyb_apply_blend(vpyb_skeleton *s, const vpyb_clip *a, int32_t ta_q8,
                      const vpyb_clip *b, int32_t tb_q8, int w_q8)
{
    if (w_q8 < 0) w_q8 = 0;
    if (w_q8 > 256) w_q8 = 256;
    for (int i = 0; i < s->n; i++) {
        const int ha = i < a->bones && a->tracks[i].n, hb = i < b->bones && b->tracks[i].n;
        if (!ha && !hb) continue;
        const vpyb_quat qa = ha ? vpyb_sample(&a->tracks[i], ta_q8, a->length, a->loop) : vpyb_quat_identity();
        const vpyb_quat qb = hb ? vpyb_sample(&b->tracks[i], tb_q8, b->length, b->loop) : vpyb_quat_identity();
        s->rot[i] = !ha ? qb : (!hb ? qa : nlerp(qa, qb, w_q8));
    }
}

/* ── a limb by IK ──────────────────────────────────────────────────────────── */
static int first_child(const vpyb_skeleton *s, int bone)
{
    for (int i = bone + 1; i < s->n; i++) if (s->parent[i] == bone) return i;
    return -1;
}

int vpyb_ik(vpyb_skeleton *s, int upper, const int32_t target[3], const int32_t pole[3])
{
    const int mid = upper >= 0 && upper < s->n ? first_child(s, upper) : -1;
    const int end = mid >= 0 ? first_child(s, mid) : -1;
    if (end < 0) { s_stats.ik_refused++; return -1; }
    const int64_t l1 = isqrt64((int64_t)s->off[mid][0] * s->off[mid][0] + (int64_t)s->off[mid][1] * s->off[mid][1] + (int64_t)s->off[mid][2] * s->off[mid][2]);
    const int64_t l2 = isqrt64((int64_t)s->off[end][0] * s->off[end][0] + (int64_t)s->off[end][1] * s->off[end][1] + (int64_t)s->off[end][2] * s->off[end][2]);
    int32_t joint[3], tip[3];
    const int reached = vpyik_two_bone(s->pos[upper], target, (int32_t)l1, (int32_t)l2, pole, joint, tip);
    if (!reached) s_stats.ik_stretched++;

    const int p = s->parent[upper];
    const vpyb_quat pr = p < 0 ? s->root_rot : s->wrot[p];
    /* the upper bone: turn its world rotation so its child's joint points at the elbow */
    int32_t now[3], want[3];
    qrot(s->wrot[upper], s->off[mid], now);
    for (int k = 0; k < 3; k++) want[k] = joint[k] - s->pos[upper][k];
    const vpyb_quat wu = qmul(from_to(now, want), s->wrot[upper]);
    s->rot[upper] = qmul(qconj(pr), wu);
    /* the middle bone, from where the upper one now leaves it */
    const vpyb_quat wm0 = qmul(wu, s->rot[mid]);
    qrot(wm0, s->off[end], now);
    for (int k = 0; k < 3; k++) want[k] = tip[k] - joint[k];
    const vpyb_quat wm = qmul(from_to(now, want), wm0);
    s->rot[mid] = qmul(qconj(wu), wm);

    vpyb_pose(s, s->root_pos, s->root_rot);
    return reached;
}
