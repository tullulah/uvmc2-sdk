/*
 * vpyfx.h — particles and debris for libvpy: sparks, bursts, and things that
 * break into pieces.
 *
 * TWO KINDS OF PIECE, BOTH ONE STROKE EACH.
 *   a SPARK is a point that moves, drawn as a short streak along its velocity —
 *   the faster it goes the longer it is, which is motion blur for free.
 *   a STICK is a rigid segment that moves AND spins. vpyfx_shatter() turns
 *   every edge of a mesh into one, flying out from the point of the blow: the
 *   classic vector explosion, and it costs exactly the edges the object had.
 * Both feel gravity, can bounce on a floor, and fade out over their life.
 *
 * A STROKE BUDGET, AND THE LOWEST PRIORITY. Effects draw at most
 * vpyfx_set_budget() strokes a frame (default 160) and draw them at
 * VPY_PRI_LOW, so when a frame is full they are the first thing libvpy sheds —
 * never the scenery. What the budget leaves out is counted (vpyfx_stats()->shed),
 * and so is every piece that had to replace an older one because the pool was
 * full (->recycled). Nothing is dropped silently.
 *
 * INTEGER ONLY AND DETERMINISTIC, like the rest of libvpy: randomness comes from
 * its own generator (vpyfx_seed), so a replay gives the same debris.
 *
 * UNITS — the same as vpyphys: world units (mm works), velocities in units per
 * second, gravity in units/s², spin in 4096ths of a turn per second, life in
 * steps (one vpyfx_step() per frame: 50 = one second at 50 Hz).
 *
 * DRAWING. vpyfx_draw() goes through vpy3d: give it 1 to send the strokes
 * through the occluder (draw effects AFTER the solids they can be behind).
 * vpyfx_draw2d() reads x and y as screen device units and ignores z, for a 2D
 * game.
 */
#ifndef VPYFX_H
#define VPYFX_H

#include <stdint.h>
#include "vpy3d.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYFX_MAX
#define VPYFX_MAX 192             /* pieces alive at once */
#endif

/* ---- the system ----------------------------------------------------------- */
void vpyfx_reset(void);                     /* no pieces, no floor, no gravity */
void vpyfx_seed(uint32_t seed);
void vpyfx_set_rate(int steps_per_second);  /* default 50 */
void vpyfx_set_gravity(int32_t gx, int32_t gy, int32_t gz);   /* units/s² */
/* A floor at y that pieces bounce on: bounce Q8 (256 = perfectly elastic).
 * Pieces that come to rest on it lie there until their life runs out. */
void vpyfx_set_floor(int on, int32_t y, int bounce_q8);
void vpyfx_set_budget(int strokes_per_frame);                 /* default 160 */

/* ---- making pieces --------------------------------------------------------
 * Each returns how many pieces it made. `br` is the starting brightness
 * (0..127); it fades to nothing over `life` steps. */
int vpyfx_spark(int32_t x, int32_t y, int32_t z,
                int32_t vx, int32_t vy, int32_t vz, int life, int br);
/* `count` sparks from a point, in every direction at up to `speed`, on top of
 * the velocity (vx, vy, vz) — what a hit, an explosion or a muzzle wants. */
int vpyfx_burst(int32_t x, int32_t y, int32_t z,
                int32_t vx, int32_t vy, int32_t vz,
                int count, int32_t speed, int life, int br);
/* A rigid segment from a to b, moving at (vx,vy,vz) and spinning at up to
 * `spin` about a random axis. */
int vpyfx_stick(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz,
                int32_t vx, int32_t vy, int32_t vz, int32_t spin, int life, int br);
/* EVERY EDGE OF A MESH BECOMES A STICK. `place` is the transform the mesh was
 * drawn with, (vx,vy,vz) the velocity it had, and each piece is thrown away
 * from (cx,cy,cz) — where the blow landed — at up to `speed`, spinning at up to
 * `spin`. The game removes the object itself. */
int vpyfx_shatter(const vpy_mesh *m, const vpy_xf *place,
                  int32_t vx, int32_t vy, int32_t vz,
                  int32_t cx, int32_t cy, int32_t cz,
                  int32_t speed, int32_t spin, int life, int br);

/* A RING in the plane with normal (nx,ny,nz) round (cx,cy,cz), starting at
 * radius r0 and growing at `speed` units/s: a shockwave to see (vpyp_blast is
 * the one that pushes). `segments` strokes, 3..32, all counted in the budget —
 * a ring the budget cannot fit whole is left out and counted as shed. */
int vpyfx_ring(int32_t cx, int32_t cy, int32_t cz, int32_t nx, int32_t ny, int32_t nz,
               int32_t r0, int32_t speed, int segments, int life, int br);
/* A LINE that stays where it is and fades: no gravity, no spin. One a frame
 * from where an object was to where it is makes a TRAIL behind it. */
int vpyfx_line(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz, int life, int br);

/* ---- every frame ---------------------------------------------------------- */
void vpyfx_step(void);
void vpyfx_draw(int occlude);
void vpyfx_draw2d(void);

typedef struct {
    uint32_t alive;             /* pieces in the air */
    uint32_t drawn;             /* strokes drawn last frame */
    uint32_t shed;              /* pieces the budget left undrawn last frame */
    uint32_t recycled;          /* pieces that replaced an older one: pool full */
} vpyfx_stats_t;
const vpyfx_stats_t *vpyfx_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYFX_H */
