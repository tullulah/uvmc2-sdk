/*
 * vpyent.c — see vpyent.h.
 *
 * EVERY TABLE HERE IS STATIC. A .um2's core 0 has 4 KB of stack (docs/02), and the
 * sort order alone is VPYENT_MAX ints; nothing here puts more than a few hundred bytes
 * on the stack.
 */
#include "vpyent.h"
#include "vpy.h"
#include "vpyphys.h"
#include "vpyimpact.h"
#include "vpyfx.h"
#include "vpycam.h"

#define BR_DEFAULT 100
#define MARK_R_DEFAULT 40           /* the ring physics_demo settled on for a 260 mm crate */
#define MARK_BR_DEFAULT 127

typedef struct {
    uint8_t  used, occ, kind, material, dented, mark_style;
    int16_t  body;
    int16_t  br, mark_br;
    int32_t  occ_h[3];
    int32_t  mark_r;
    const vpy_mesh *mesh;          /* what it is drawn with: `own` once dented */
    void    *user;
    vpy_xf   place;                /* for an entity with no body */
} ent_t;

static ent_t       s_e[VPYENT_MAX];
static vpy_mesh    s_own[VPYENT_MAX];       /* each entity's dented copy */
static vpy3d_marks s_marks[VPYENT_MAX];
/* a body's material, by body id, for vpyimpact_contacts; VPYI_NONE when no entity says */
static uint8_t     s_mat_of_body[VPYP_MAX_BODIES];
static int         s_order[VPYENT_MAX];
static int64_t     s_dist[VPYENT_MAX];
static vpyent_stats_t s_stats;
static int         s_tables_ready;

static void tables_ready(void)
{
    if (s_tables_ready) return;
    for (int i = 0; i < VPYP_MAX_BODIES; i++) s_mat_of_body[i] = VPYI_NONE;
    s_tables_ready = 1;
}

static ent_t *get(int e) { return (e >= 0 && e < VPYENT_MAX && s_e[e].used) ? &s_e[e] : 0; }

int vpyent_create(const vpy_mesh *mesh, int body)
{
    tables_ready();
    for (int e = 0; e < VPYENT_MAX; e++) {
        if (s_e[e].used) continue;
        ent_t *t = &s_e[e];
        t->used = 1; t->occ = VPYENT_OCC_MESH; t->kind = 0; t->material = VPYI_NONE;
        t->dented = 0; t->mark_style = VPY3D_MARK_RING;
        t->body = (int16_t)(body >= 0 && body < VPYP_MAX_BODIES ? body : VPYENT_NONE);
        t->br = BR_DEFAULT; t->mark_br = MARK_BR_DEFAULT; t->mark_r = MARK_R_DEFAULT;
        t->occ_h[0] = t->occ_h[1] = t->occ_h[2] = 0;
        t->mesh = mesh; t->user = 0;
        t->place = vpy3d_identity();
        vpy3d_marks_clear(&s_marks[e]);
        if (t->body >= 0) s_mat_of_body[t->body] = VPYI_NONE;
        s_stats.alive++;
        return e;
    }
    s_stats.refused++;
    return VPYENT_NONE;
}

void vpyent_destroy(int e)
{
    ent_t *t = get(e); if (!t) return;
    if (t->body >= 0) { vpyp_remove(t->body); s_mat_of_body[t->body] = VPYI_NONE; }
    t->used = 0;
    s_stats.alive--;
}

void vpyent_reset(void)
{
    for (int e = 0; e < VPYENT_MAX; e++) if (s_e[e].used) vpyent_destroy(e);
    vpyent_stats_t z = { 0 };
    s_stats = z;
    s_tables_ready = 0; tables_ready();
}

int vpyent_alive(int e) { return get(e) != 0; }
int vpyent_next(int e)
{
    for (int i = e + 1; i < VPYENT_MAX; i++) if (s_e[i].used) return i;
    return VPYENT_NONE;
}
int vpyent_of_body(int body)
{
    if (body < 0) return VPYENT_NONE;
    for (int e = 0; e < VPYENT_MAX; e++) if (s_e[e].used && s_e[e].body == body) return e;
    return VPYENT_NONE;
}

void vpyent_set_place(int e, const vpy_xf *p) { ent_t *t = get(e); if (t && p) t->place = *p; }
void vpyent_place(int e, vpy_xf *out)
{
    ent_t *t = get(e);
    if (!t) { *out = vpy3d_identity(); return; }
    if (t->body < 0 || !vpyp_alive(t->body)) { *out = t->place; return; }
    int32_t x, y, z; vpyp_position(t->body, &x, &y, &z);
    *out = vpy3d_translate(x, y, z);
    vpyp_rotation(t->body, out->m);          /* the turn it really has */
}
int  vpyent_body(int e) { ent_t *t = get(e); return t ? t->body : VPYENT_NONE; }
void vpyent_set_mesh(int e, const vpy_mesh *m) { ent_t *t = get(e); if (t) { t->mesh = m; t->dented = 0; } }
void vpyent_set_brightness(int e, int br) { ent_t *t = get(e); if (t) t->br = (int16_t)br; }
void vpyent_set_occluder(int e, int kind, int32_t hx, int32_t hy, int32_t hz)
{
    ent_t *t = get(e); if (!t) return;
    t->occ = (uint8_t)kind; t->occ_h[0] = hx; t->occ_h[1] = hy; t->occ_h[2] = hz;
}
void vpyent_set_material(int e, int m)
{
    ent_t *t = get(e); if (!t) return;
    t->material = (uint8_t)m;
    if (t->body >= 0) { tables_ready(); s_mat_of_body[t->body] = (uint8_t)m; }
}
void vpyent_set_kind(int e, int k) { ent_t *t = get(e); if (t) t->kind = (uint8_t)k; }
int  vpyent_kind(int e) { ent_t *t = get(e); return t ? t->kind : -1; }
void vpyent_set_user(int e, void *u) { ent_t *t = get(e); if (t) t->user = u; }
void *vpyent_user(int e) { ent_t *t = get(e); return t ? t->user : 0; }

int vpyent_dent(int e, int32_t px, int32_t py, int32_t pz,
                int32_t dx, int32_t dy, int32_t dz, int32_t depth, int32_t radius)
{
    ent_t *t = get(e); if (!t || !t->mesh || depth <= 0) return 0;
    if (!t->dented) {
        /* the entity's own copy, made the first time it is dented; copying again into
         * the same slot resets it in place without using more pool (vpy3d_mesh_copy) */
        if (!vpy3d_mesh_copy(&s_own[e], t->mesh)) { s_stats.dents_refused++; return 0; }
        t->mesh = &s_own[e]; t->dented = 1;
    }
    vpy_xf at; vpyent_place(e, &at);
    int32_t mx, my, mz, ux, uy, uz;
    vpy3d_world_to_model(&at, px, py, pz, &mx, &my, &mz);
    vpy_xf turn = at; turn.t[0] = turn.t[1] = turn.t[2] = 0;   /* a direction: no translation */
    vpy3d_world_to_model(&turn, dx, dy, dz, &ux, &uy, &uz);
    vpy3d_mesh_dent(&s_own[e], mx, my, mz, ux, uy, uz, depth, radius);
    return 1;
}

void vpyent_mark(int e, int32_t x, int32_t y, int32_t z, int32_t nx, int32_t ny, int32_t nz)
{
    ent_t *t = get(e); if (!t) return;
    vpy_xf at; vpyent_place(e, &at);
    vpy3d_marks_add(&s_marks[e], &at, x, y, z, nx, ny, nz);
}
void vpyent_set_marks(int e, int32_t radius, int br, int style)
{
    ent_t *t = get(e); if (!t) return;
    t->mark_r = radius; t->mark_br = (int16_t)br; t->mark_style = (uint8_t)style;
}

/* The entity's occluder corners in the world: 8 for a box, 6 for a sphere's
 * octahedron, the mesh's own vertices for MESH (<= 8). Returns how many, 0 if none. */
static int corners_of(const ent_t *t, const vpy_xf *at, int32_t c[8][3])
{
    if (t->occ == VPYENT_OCC_BOX) {
        for (int k = 0; k < 8; k++) {
            const int64_t l[3] = { (k & 1) ? t->occ_h[0] : -t->occ_h[0], (k & 2) ? t->occ_h[1] : -t->occ_h[1],
                                   (k & 4) ? t->occ_h[2] : -t->occ_h[2] };
            for (int r = 0; r < 3; r++)
                c[k][r] = at->t[r] + (int32_t)((at->m[r*3] * l[0] + at->m[r*3+1] * l[1] + at->m[r*3+2] * l[2]) >> 14);
        }
        return 8;
    }
    if (t->occ == VPYENT_OCC_SPHERE) {
        const int32_t r = t->occ_h[0];
        static const int8_t D[6][3] = { {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1} };
        for (int k = 0; k < 6; k++) for (int a = 0; a < 3; a++) c[k][a] = at->t[a] + D[k][a] * r;
        return 6;
    }
    if (t->occ == VPYENT_OCC_MESH && t->mesh && t->mesh->nv <= 8) {
        /* the mesh's own vertices, gathered from its edges (vpy3d gives edges, not
         * vertices), turned and moved as vpy3d_occl_add_mesh would take them */
        int n = 0;
        const int ne = vpy3d_mesh_edge_count(t->mesh);
        for (int ei = 0; ei < ne && n < 8; ei++) {
            int32_t ab[2][3];
            if (!vpy3d_mesh_edge(t->mesh, ei, ab[0], ab[1])) continue;
            for (int s2 = 0; s2 < 2 && n < 8; s2++) {
                int32_t w[3];
                for (int r = 0; r < 3; r++)
                    w[r] = at->t[r] + (int32_t)(((int64_t)at->m[r*3] * ab[s2][0] + (int64_t)at->m[r*3+1] * ab[s2][1] +
                                                 (int64_t)at->m[r*3+2] * ab[s2][2]) >> 14);
                int dup = 0;
                for (int q = 0; q < n && !dup; q++) dup = c[q][0] == w[0] && c[q][1] == w[1] && c[q][2] == w[2];
                if (!dup) { c[n][0] = w[0]; c[n][1] = w[1]; c[n][2] = w[2]; n++; }
            }
        }
        return n >= 3 ? n : 0;
    }
    return 0;
}

int vpyent_draw(void)
{
    int32_t eye[3]; vpy3d_eye(eye);
    int n = 0;
    /* near to far: insertion sort on the squared distance, computed once per entity */
    for (int e = 0; e < VPYENT_MAX; e++) {
        if (!s_e[e].used) continue;
        vpy_xf at; vpyent_place(e, &at);
        const int64_t dx = at.t[0] - eye[0], dy = at.t[1] - eye[1], dz = at.t[2] - eye[2];
        const int64_t d = dx * dx + dy * dy + dz * dz;
        int j = n++;
        while (j > 0 && s_dist[j - 1] > d) { s_order[j] = s_order[j - 1]; s_dist[j] = s_dist[j - 1]; j--; }
        s_order[j] = e; s_dist[j] = d;
    }
    const int keep_occ = vpy3d_get_mesh_occlusion();
    vpy3d_set_mesh_occlusion(1);             /* a mesh behind a nearer one is cut by it */
    uint32_t missing = 0;
    static int32_t c[8][3];
    for (int i = 0; i < n; i++) {
        const int e = s_order[i];
        ent_t *t = &s_e[e];
        vpy_xf at; vpyent_place(e, &at);
        if (t->mesh) vpy3d_draw_mesh(t->mesh, &at, t->br);
        /* marks after the solid, before its own occluder: they sit on its faces */
        if (s_marks[e].n) vpy3d_marks_draw(&s_marks[e], &at, t->mark_r, t->mark_br, t->mark_style);
        if (t->occ == VPYENT_OCC_NONE) continue;
        const int k = corners_of(t, &at, c);
        if (k > 0) { if (!vpy3d_occl_add((const int32_t (*)[3])c, k)) missing++; }
        else missing++;                      /* MESH with more than 8 vertices and no shape */
    }
    vpy3d_set_mesh_occlusion(keep_occ);
    s_stats.drawn = (uint32_t)n;
    s_stats.occl_missing = missing;
    return n;
}

int vpyent_draw_shadows(int32_t lx, int32_t ly, int32_t lz, int32_t floor_y, int32_t lift, int br)
{
    const int keep = vpy_get_priority();
    vpy_set_priority(VPY_PRI_LOW);           /* a full frame sheds shadows before solids */
    int drawn = 0;
    static int32_t c[8][3];
    for (int e = 0; e < VPYENT_MAX; e++) {
        ent_t *t = &s_e[e];
        if (!t->used || t->body < 0) continue;  /* fixed scenery throws none */
        vpy_xf at; vpyent_place(e, &at);
        const int k = corners_of(t, &at, c);
        if (k == 0) continue;
        int32_t low = c[0][1];
        for (int v = 1; v < k; v++) if (c[v][1] < low) low = c[v][1];
        if (low <= floor_y + lift) continue;  /* resting, or as good as: it hides its own */
        drawn += vpy3d_shadow((const int32_t (*)[3])c, k, lx, ly, lz, floor_y, br) > 0;
    }
    vpy_set_priority(keep);
    return drawn;
}

int vpyent_step(int floor_material)
{
    tables_ready();
    /* THE STOP IS READ BEFORE THE CAMERA STEPS. vpycam_hitstop(n) promises n frames with
     * vpycam_stopped() true, and vpycam_step counts one off: stepping first and asking
     * after held time for n - 1 frames (measured in tools/ent_check.c). */
    const int stopped = vpycam_stopped();
    vpycam_step();
    vpyimpact_step();
    if (stopped) return 0;                   /* a hit-stop holds time: nothing moves */
    vpyp_step();
    vpyimpact_contacts(s_mat_of_body, floor_material);
    vpyfx_step();
    return 1;
}

const vpyent_stats_t *vpyent_stats(void) { return &s_stats; }
