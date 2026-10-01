/*
 * vpy3d.h — the 3D layer of libvpy: transforms, a camera, true perspective,
 * and surface meshes drawn with hidden-line removal.
 *
 * WHAT THIS IS FOR. A vector display has no fill, so a solid drawn as all of
 * its edges reads as a wireframe cage and not as an object. The fix is to give
 * the solid real FACES and draw an edge only when it is a silhouette (one face
 * toward the viewer, one away) or a hard crease with a visible face. A smooth
 * curve then shows only its moving outline, the way a person would draw it.
 * That is what this module does, and it is the part a game cannot reasonably
 * write for itself.
 *
 * INTEGER ONLY, like the rest of libvpy — no <math.h>, no floats anywhere. The
 * same C runs on the cartridge, on PiTrex and in the IDE simulator. Rotations
 * are Q14 (16384 = 1.0) and angles are the 4096-per-turn units of
 * vpy_sin_q14(); 128 steps would show as a staircase on a moving camera.
 *
 * UNITS
 *   model      int16, whatever the model is authored in (mm works well)
 *   world      int32, same unit as the model
 *   camera     int32, +x right, +y up, +z INTO the screen (away from you)
 *   screen     PiTrex deflection units, straight into the libvpy stroke buffer
 *              via vpy_draw_line_dev — so 3D shares the frame, the priorities
 *              and the flush with everything else the game draws
 *   angles     4096 per full turn
 *   brightness 0..127
 *
 * WHAT COSTS TIME is the number of strokes, not their length: a stroke is ~13
 * bus commands on the UVM2 whatever it spans, and a blanked jump before it is
 * ~5 more. So straight runs across several faces are merged into ONE stroke,
 * which is exact — a straight line in 3D projects to a straight line in 2D.
 * Use vpy_set_priority() to say what may be shed when a frame will not fit.
 */
#ifndef VPY3D_H
#define VPY3D_H

#include <stdint.h>
#include "vpy.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- pool sizes -----------------------------------------------------------
 * All meshes share these, filled once at start-up. Override with -D if a game
 * needs more; vpy3d_pool_used() reports the high-water marks so the numbers
 * come from a measurement and not from a guess. */
#ifndef VPY3D_POOL_V
#define VPY3D_POOL_V  512      /* vertices, all meshes together */
#endif
#ifndef VPY3D_POOL_F
#define VPY3D_POOL_F  512      /* faces */
#endif
#ifndef VPY3D_POOL_FV
#define VPY3D_POOL_FV 2048     /* face->vertex index entries */
#endif
#ifndef VPY3D_POOL_E
#define VPY3D_POOL_E  1024     /* edges */
#endif
#ifndef VPY3D_MESH_MAXV
#define VPY3D_MESH_MAXV 256    /* vertices in ONE mesh (the per-draw scratch) */
#endif
#ifndef VPY3D_MESH_MAXF
#define VPY3D_MESH_MAXF 256
#endif

#define VPY3D_ONE 16384        /* Q14: this is 1.0 */

/* ---- a transform ----------------------------------------------------------
 * A 3x3 rotation in Q14 plus an integer translation. Column-major is a trap
 * waiting to happen, so: m[row*3 + col], and applying it is
 *   out[r] = (m[r*3+0]*v0 + m[r*3+1]*v1 + m[r*3+2]*v2) >> 14 + t[r]. */
typedef struct { int32_t m[9]; int32_t t[3]; } vpy_xf;

vpy_xf vpy3d_identity(void);
vpy_xf vpy3d_mul(const vpy_xf *a, const vpy_xf *b);   /* apply b, then a */
vpy_xf vpy3d_rot_x(int ang);
vpy_xf vpy3d_rot_y(int ang);
vpy_xf vpy3d_rot_z(int ang);
vpy_xf vpy3d_translate(int32_t x, int32_t y, int32_t z);

/* ---- the camera ---- */
void vpy3d_set_camera(const vpy_xf *world_to_camera);
/* Point the camera at something. `up` is the world direction that should end
 * up pointing up on screen; pass 0,1,0 unless you are doing something clever.
 * Returns 0 and leaves the camera alone if eye and target coincide. */
int  vpy3d_look_at(int32_t ex, int32_t ey, int32_t ez,
                   int32_t tx, int32_t ty, int32_t tz,
                   int32_t ux, int32_t uy, int32_t uz);
const vpy_xf *vpy3d_camera(void);
/* Where the camera IS, in world coordinates.
 *
 * The transform holds world->camera, so the eye is -R^T * t and not something
 * that can be read off it directly. Worth having as a call: culling an axis
 * aligned face is one comparison against the eye — a +x face is visible when
 * the eye is further out in x than the face is — and that is the whole of
 * hidden-surface removal for a grid of boxes. */
void vpy3d_eye(int32_t *out);

/* Field of view, as deflection units per unit of x over z. Bigger = narrower.
 * The default (28000) is about 58 degrees across the default 15500 clip. */
void vpy3d_set_focal(int32_t focal);

/* ---- one unit is one unit on both axes --------------------------------------
 * x is multiplied by num/den on its way to the screen. The default is 1/1.
 *
 * MEASURED 2026-10-01 by photograph, one console (examples/geometry_card in the
 * starter kit): a 16000-unit square is square on the glass, to within ~10%. The
 * tube is portrait, but that is the shape of the WINDOW (vpy3d_set_clip_xy),
 * not of the unit — as the PiTrex contract and the BIOS have always assumed.
 *
 * For one day (2026-09-30) the default was 4/3, derived from the service manual
 * and never measured; the photograph refuted it. A game composed during that day
 * comes out three quarters as wide as it did then, and that is the correction.
 * The knob is for a console whose size pots are off, not for a default in doubt. */
void vpy3d_set_aspect(int32_t num, int32_t den);
void vpy3d_aspect(int32_t *num, int32_t *den);

/* Half of what is actually on screen, in the Q14 trig's units (VPY_Q14_TURN per
 * turn), each from its own axis's clip, and horizontally with the aspect.
 *
 * READ IT, do not restate it. A light cone, a ray fan or a cull that hardcodes
 * "58 degrees" is a second copy of the lens, and the two drift the first time
 * anyone touches set_focal, set_clip or set_aspect: strokes get generated that
 * the clipper then throws away, or geometry goes missing at the edges and looks
 * like an occlusion bug. Derived by bisection over the renderer's own sine
 * table, so the answer cannot disagree with the picture. */
int vpy3d_h_half_angle(void);
int vpy3d_v_half_angle(void);
/* Nothing closer to the camera than this is drawn; lines crossing it are cut.
 * In world units. Default 600. */
void vpy3d_set_near(int32_t near_z);
/* Half the visible window in deflection units: set_clip sets both axes,
 * set_clip_xy each one. Default 15500 square, which every vpy3d game was composed
 * in. The glass is bigger and portrait — about +-18000 x +-20500 on the one
 * console photographed (2026-10-01) — so a game may open it up; a default moves
 * only with a second console's photograph. The half angles above follow it. */
void vpy3d_set_clip(int32_t half);
void vpy3d_set_clip_xy(int32_t half_x, int32_t half_y);

/* ---- drawing without a mesh ---- */
void vpy3d_line_cam(const int32_t *a, const int32_t *b, int br);   /* camera space */
void vpy3d_line_world(int32_t ax, int32_t ay, int32_t az,
                      int32_t bx, int32_t by, int32_t bz, int br);
/* Project one camera-space point to deflection units. 0 if behind the near
 * plane, in which case sx and sy are untouched. */
int  vpy3d_project(const int32_t *cam, int32_t *sx, int32_t *sy);
/* Move a point from world into camera space. */
void vpy3d_to_camera(int32_t wx, int32_t wy, int32_t wz, int32_t *out);

/* ---- meshes ---------------------------------------------------------------
 * Build once at start-up, draw every frame:
 *
 *   vpy_mesh cube;
 *   vpy3d_mesh_begin(&cube);
 *   int v[8]; for (...) v[i] = vpy3d_vertex(x, y, z);
 *   int f[4] = { v[0], v[1], v[2], v[3] }; vpy3d_face(f, 4);
 *   ...
 *   vpy3d_mesh_end(VPY3D_HARD_45);
 *
 * Wind each face so its vertices go ANTICLOCKWISE seen from outside. If a
 * model comes out inside-out, that winding is why — vpy3d_mesh_end flips any
 * normal that points at the mesh centroid, which rescues most cases but not a
 * face whose plane passes near the centroid. */
typedef struct {
    uint16_t v0, nv;
    uint16_t f0, nf;
    uint16_t e0, ne;
    uint8_t  open;    /* 1 = a plate, not a solid: its outline is always drawn */
} vpy_mesh;

/* Crease threshold: two faces meeting at a sharper angle than this keep their
 * shared edge. Q14 cosine — a bigger number is a stricter test (fewer lines). */
#define VPY3D_HARD_30  14189   /* cos 30 deg */
#define VPY3D_HARD_45  11585   /* cos 45 deg */
#define VPY3D_HARD_60   8192   /* cos 60 deg */
#define VPY3D_HARD_ALL 16384   /* every edge is a crease: a full wireframe */

int  vpy3d_mesh_begin(vpy_mesh *m);           /* 0 if the pools are exhausted */
int  vpy3d_vertex(int x, int y, int z);       /* returns its index, or -1 */
int  vpy3d_face(const int *idx, int n);       /* indices from vpy3d_vertex */
int  vpy3d_quad(int a, int b, int c, int d);
int  vpy3d_tri(int a, int b, int c);
int  vpy3d_mesh_end(int hard_cos_q14);        /* 0 on overflow; see vpy3d_error() */
void vpy3d_mesh_open(vpy_mesh *m, int open);  /* mark a plate after building it */

void vpy3d_draw_mesh(const vpy_mesh *m, const vpy_xf *place, int br);
/* A built mesh's edges in model space, all of them (visible or not). For
 * effects that take a mesh apart — vpyfx_shatter. 0 if `e` is out of range. */
int  vpy3d_mesh_edge_count(const vpy_mesh *m);
int  vpy3d_mesh_edge(const vpy_mesh *m, int e, int32_t a[3], int32_t b[3]);

/* CAGE MODE: draw every edge of the next meshes, the hidden ones too.
 *
 * A solid and a cage of the same shape read as two different objects at any
 * distance and at any brightness — which is what makes this worth having on a
 * monochrome tube, where brightness is usually already spent on depth. It costs
 * the edges it adds and nothing else: no second mesh to author and keep in step
 * with the first, and no geometry that can drift apart from it.
 *
 * A draw-time mode rather than a property of the mesh, like the priority: the
 * same cube can be a solid in one place on the board and a cage in another. */
void vpy3d_set_wire(int on);
int  vpy3d_get_wire(void);

/* Emit a mesh's strokes in an order that follows its own connectivity, so a
 * stroke starts where the last one ended and pays neither a blanked jump nor
 * the beam re-centre that a long jump forces. On by default. Turn it off only
 * to measure what it is worth — it is the difference between a small rotating
 * solid sitting still and one that shakes as it turns. */
void vpy3d_set_chaining(int on);
int  vpy3d_get_chaining(void);

/* ---- compiled meshes (.vmesh) ---------------------------------------------
 * `data` is the byte image from
 *   vpy_cli compile-asset <file>.vmesh --format c --out <name>.h
 * Building it goes through the very same vpy3d_vertex / vpy3d_face /
 * vpy3d_mesh_end as a hand-built mesh — there is ONE mesh builder, so a
 * compiled cube and a cube written out in C draw identically.
 *
 * Returns 0 and leaves `m` alone if the magic or the version is wrong, or if
 * the pools are full; vpy3d_error() says which. */
int vpy3d_load_mesh(vpy_mesh *m, const unsigned char *data);

/* Why the last vpy3d_load_mesh returned 0. */
typedef enum {
    VPY3D_OK = 0,
    VPY3D_ERR_MAGIC,     /* not a .vmesh */
    VPY3D_ERR_VERSION,   /* a newer .vmesh than this runtime understands */
    VPY3D_ERR_POOL,      /* the shared pools are full: raise VPY3D_POOL_* */
    VPY3D_ERR_DATA       /* the image is self-inconsistent */
} vpy3d_error_t;
vpy3d_error_t vpy3d_error(void);

/* ---- ONE SOLID HIDING ANOTHER -----------------------------------------------
 * Everything above removes hidden lines WITHIN a mesh. It knows nothing about a
 * second mesh in front of the first, and with no depth buffer that means the
 * second mesh is not there at all — you see straight through it. It shows up as
 * "the picture is dirty" and it is invisible in the code, so it is worth saying
 * plainly: ANY game with two solids on screen has this until it uses what is
 * below.
 *
 * A convex solid's silhouette is the convex hull of its projected corners, and
 * a line behind it is that line minus the part inside the hull. Exact for
 * convex occluders, and with none added vpy3d_occl_line IS vpy3d_line_world
 * plus one compare — so every stroke in a game can go through it.
 *
 * HOW TO USE IT, and the order is half of it:
 *
 *     vpy3d_occl_reset();                  // once a frame
 *     ... draw the nearest solid ...
 *     vpy3d_occl_add_mesh(&m, &at);        // AFTER drawing it. Never itself.
 *     ... draw the next one, and so on, NEAR TO FAR ...
 *     vpy3d_occl_line(...)                 // for everything that can be hidden
 *
 * There is no depth test. The caller's order is what makes a silhouette mean
 * "what is behind it", and kuroishi found that out the hard way: with the
 * occluder in and the floor still drawn first, the mass went on being
 * transparent and the occluder looked broken. The one case that goes
 * catastrophically wrong — a line wholly in front of the whole solid — is
 * guarded here; nothing else is.
 *
 * Add the nearest solids first: the table holds 64 and past that nothing more
 * hides anything, which is the picture we had before, never a wrong one.
 *
 * WHAT IT DOES NOT CUT, each of which reads as "the occluder is broken":
 *   - vpy3d_draw_mesh, unless vpy3d_set_mesh_occlusion(1) (below). By default
 *     only strokes sent through vpy3d_occl_line are cut, and a mesh drawn after
 *     an occluder was added goes straight through it.
 *   - A visible piece shorter than 1/48 of the line's own length on screen. It
 *     is dropped as a sliver. Relative, not absolute: on a stroke the width of
 *     the screen that is about 2% of the screen.
 * (A line with an end behind the near plane IS cut: it is clipped to the near
 * plane first, exactly as vpy3d_line_cam would, and what is left is tested.)
 *
 * vpy3d_occl_add and _add_mesh return 0 when they refuse, and every refusal is
 * counted in vpy3d_stats(): occl_full is the budget, occl_refused is the rest,
 * and occl_cut is the proof that anything was hidden at all. */
void vpy3d_occl_reset(void);
/* A convex solid by 3 to 8 corners IN WORLD SPACE. 0 if it could not be taken
 * (a corner behind the near plane, or the table full) and then it simply hides
 * nothing.
 *
 * KEEP IT ROUGHLY ON SCREEN. The projection saturates at 2^30 and the clipper
 * multiplies projected coordinates together, so an occluder tens of screens
 * wide — a ground span the length of a level is the obvious way to write one —
 * overflows int64 and yields a hull that is not a hull. Clamp the box to the
 * visible window before adding it; nothing off screen was hiding anything. */
int  vpy3d_occl_add(const int32_t (*corners)[3], int n);
/* The same, from a mesh and the transform it was drawn with — for the common
 * case where the occluder IS the box you just drew. Meshes of more than eight
 * vertices are refused rather than approximated: pass the eight corners of the
 * box you mean. */
int  vpy3d_occl_add_mesh(const vpy_mesh *m, const vpy_xf *place);
/* A line in world space, minus every occluder added so far. */
void vpy3d_occl_line(int32_t ax, int32_t ay, int32_t az,
                     int32_t bx, int32_t by, int32_t bz, int br);
/* The same in camera space, as vpy3d_line_cam is to vpy3d_line_world. */
void vpy3d_occl_line_cam(const int32_t *a, const int32_t *b, int br);

/* MESHES THROUGH THE OCCLUDER, opt-in. With it on, every stroke
 * vpy3d_draw_mesh emits is cut by the occluders added so far, exactly as
 * vpy3d_occl_line would cut it; the mesh's own hidden-line removal runs first,
 * as always. Off by default, and that is deliberate: a game that draws a figure
 * with meshes may depend on it never being cut (one that stands where no solid
 * can be in front of it, say), and switching it on under that game would change
 * its picture.
 *
 * The order rule does not change: draw a mesh, THEN vpy3d_occl_add_mesh it, and
 * meshes near to far. A mesh is never cut by its own silhouette as long as it is
 * added after it is drawn. With nothing added it costs one compare per stroke. */
void vpy3d_set_mesh_occlusion(int on);
int  vpy3d_get_mesh_occlusion(void);
int  vpy3d_occl_count(void);

/* ---- what did it cost, and did anything not fit ---- */
typedef struct {
    uint16_t verts, faces, face_idx, edges;   /* pool high-water marks */
    uint32_t strokes;      /* strokes the last vpy3d_draw_mesh emitted */
    uint32_t culled;       /* edges hidden by the visibility test */
    uint32_t merged;       /* edges folded into a longer run */
    uint32_t chained;      /* strokes that started where the last one ended */
    uint32_t overflow;     /* builds that did not fit a pool. NOT ZERO = broken */
    /* The occluder, per frame. occl_cut is the proof it RAN: with occluders
     * added and a solid on screen behind them, zero means the order is wrong
     * (everything drawn before the silhouettes), not that nothing was hidden. */
    uint32_t occl_cut;     /* lines that lost some part to an occluder */
    uint32_t occl_refused; /* occluders not taken: corner behind near, degenerate,
                              n out of 3..8, a mesh of more than 8 vertices */
    uint32_t occl_full;    /* occluders not taken because the table was full */
} vpy3d_stats_t;
const vpy3d_stats_t *vpy3d_stats(void);
void vpy3d_reset_counts(void);   /* zero the per-frame counters, keep the marks */

#ifdef __cplusplus
}
#endif

#endif /* VPY3D_H */
