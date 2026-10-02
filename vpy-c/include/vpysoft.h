/*
 * vpysoft.h — soft bodies: points joined by springs, that sag, wobble and flap.
 *
 * The same Verlet core as vpyrope (where a point was and where it is says how
 * fast it goes; gravity pulls; then every spring is pulled back towards its
 * length a few times over), with three differences that make it soft:
 *
 *   * a spring has a STIFFNESS, Q8: 256 puts it fully back to length every pass
 *     (a rope's link), less lets it give — that is what makes a jelly jelly;
 *   * a body has many springs in any shape, not one chain;
 *   * a BLOB also keeps its AREA: after the springs, every rim point is pushed
 *     out or in along the rim's normal until the polygon has the area it was
 *     made with. Springs alone let a ring fold flat under its own weight;
 *     pressure is what makes a blob a blob.
 *
 * Builders: a CLOTH (a flag, a net: w × h points, structural springs drawn and
 * shear springs hidden), a BLOB (a ring round a centre point, in the x-y plane),
 * and any vpy3d MESH (every edge a spring — a wobbling crate, a jelly logo).
 * Points can be PINNED — held where the game says every frame (a flagpole, a
 * hand) — and the rest hangs from them.
 *
 * Drawing: every VISIBLE spring is one stroke, at VPY_PRI_LOW and within a
 * stroke budget (vpysoft_set_budget, default 160 a call), like vpyfx — when a
 * frame is full a soft body is shed before the scenery, and what was shed is
 * counted. Through vpy3d (occlude: through the occluder) or in 2D with x, y as
 * device units.
 *
 * Integer only, deterministic. Units: world units, gravity in units/s², one
 * vpysoft_step() per frame at 50 Hz (vpysoft_set_rate otherwise). How far the
 * springs end up from their length, and how far a blob is from its area, is
 * measured every step (vpysoft_stats) rather than assumed.
 */
#ifndef VPYSOFT_H
#define VPYSOFT_H

#include <stdint.h>
#include "vpy3d.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYSOFT_MAX_BODIES
#define VPYSOFT_MAX_BODIES   8
#endif
#ifndef VPYSOFT_MAX_POINTS
#define VPYSOFT_MAX_POINTS   256     /* all bodies together */
#endif
#ifndef VPYSOFT_MAX_SPRINGS
#define VPYSOFT_MAX_SPRINGS  640     /* all bodies together */
#endif

void vpysoft_reset(void);
void vpysoft_set_rate(int steps_per_second);                  /* default 50 */
void vpysoft_set_gravity(int32_t gx, int32_t gy, int32_t gz); /* units/s² */
/* A floor y = `y`: points are kept above it, and slide on it with `friction_q8`
 * of their sideways speed taken away each step (256 = they stick). */
void vpysoft_set_floor(int on, int32_t y, int friction_q8);
void vpysoft_set_iterations(int n);                           /* default 8 */
void vpysoft_set_damping(int q8);                             /* default 254 of 256 */
void vpysoft_set_budget(int strokes);                         /* per draw call; default 160 */

/* A CLOTH of w × h points, `cell` units apart, starting at (x,y,z) and laid out
 * along +x (columns) and -y (rows) — a flag hanging from its top edge. Structural
 * springs (drawn) at `stiff_q8`, shear springs (hidden) at half that. Nothing is
 * pinned: pin what holds it. Returns the body id, or -1 (counted). */
int  vpysoft_cloth(int32_t x, int32_t y, int32_t z, int w, int h, int32_t cell, int stiff_q8);
/* A BLOB: `n` rim points on a circle of `radius` round (x,y,z) in the x-y plane,
 * plus a centre point. Rim springs (drawn) at `stiff_q8`, spokes (hidden) at a
 * quarter of it, and the area kept with `pressure_q8` of the error put back each
 * pass. Point 0 is the centre, 1..n the rim. */
int  vpysoft_blob(int32_t x, int32_t y, int32_t z, int n, int32_t radius, int stiff_q8, int pressure_q8);
/* Every edge of a built vpy3d mesh a spring (drawn), its vertices the points,
 * placed at (x,y,z), plus a hidden centre point (the last) tied to every vertex
 * at half the stiffness — edges alone fold flat. Vertices are found from the
 * edges' ends, so a mesh with up to VPYSOFT_MAX_POINTS - 1 of them works. */
int  vpysoft_mesh(const vpy_mesh *m, int32_t x, int32_t y, int32_t z, int stiff_q8);
void vpysoft_free(int body);

int  vpysoft_points(int body);                                /* or 0 */
void vpysoft_point(int body, int i, int32_t *x, int32_t *y, int32_t *z);
/* Hold point i at (x,y,z); call again every frame to move it. */
void vpysoft_pin(int body, int i, int32_t x, int32_t y, int32_t z);
void vpysoft_unpin(int body, int i);
/* Move every point of a body by (dx,dy,dz) per second, as a push (a hit, a gust). */
void vpysoft_push(int body, int32_t vx, int32_t vy, int32_t vz);

void vpysoft_step(void);
void vpysoft_draw(int body, int br, int occlude);
void vpysoft_draw2d(int body, int br);

typedef struct {
    uint32_t bodies, points, springs;   /* in use */
    uint32_t refused;                   /* bodies not made: a table full, or bad sizes */
    int32_t  stretch;                   /* the worst spring last step: units longer than its length */
    int32_t  area_error_q8;             /* the worst blob: |area - its area| / its area, Q8 (256 = 100%) */
    uint32_t drawn, shed;               /* strokes of the last draw call, and springs left out by the budget */
} vpysoft_stats_t;
const vpysoft_stats_t *vpysoft_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYSOFT_H */
