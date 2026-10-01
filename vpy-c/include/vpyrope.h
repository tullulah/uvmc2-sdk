/*
 * vpyrope.h — ropes and chains: points joined by links of fixed length.
 *
 * Each point moves by Verlet integration (where it was and where it is now say
 * how fast it goes), gravity pulls, and then every link is put back to its
 * length a few times over — the standard way, stable, and cheap. A point can
 * be PINNED: held where the game puts it every frame (a hook, a hand, the end
 * of a crane), and the rest hangs and swings from it. A rope is drawn as one
 * chained polyline, which is the cheapest thing the beam can draw.
 *
 * Integer only, deterministic. Units: world units, gravity in units/s², one
 * vpyrope_step() per frame at 50 Hz (vpyrope_set_rate otherwise). How far the
 * links end up from their length is measured every step (vpyrope_stats) — a
 * rope pulled harder than its iterations can hold stretches, and that is
 * visible there, not only on screen.
 */
#ifndef VPYROPE_H
#define VPYROPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYROPE_MAX_ROPES
#define VPYROPE_MAX_ROPES  8
#endif
#ifndef VPYROPE_MAX_POINTS
#define VPYROPE_MAX_POINTS 256   /* all ropes together */
#endif

void vpyrope_reset(void);
void vpyrope_set_rate(int steps_per_second);                  /* default 50 */
void vpyrope_set_gravity(int32_t gx, int32_t gy, int32_t gz); /* units/s² */
void vpyrope_set_floor(int on, int32_t y);
void vpyrope_set_iterations(int n);                           /* default 8 */
/* damping of the swing, Q8: 256 keeps every bit of motion, 250 bleeds a little */
void vpyrope_set_damping(int q8);

/* A rope from a to b in `links` equal links (links + 1 points), at rest.
 * Returns its id, or -1 if the tables are full (counted). */
int  vpyrope_new(int32_t ax, int32_t ay, int32_t az, int32_t bx, int32_t by, int32_t bz, int links);
void vpyrope_free(int rope);
/* Hold point `i` at (x,y,z); call again every frame to move it. */
void vpyrope_pin(int rope, int i, int32_t x, int32_t y, int32_t z);
void vpyrope_unpin(int rope, int i);
int  vpyrope_points(int rope);                                /* links + 1, or 0 */
void vpyrope_point(int rope, int i, int32_t *x, int32_t *y, int32_t *z);

void vpyrope_step(void);
/* the rope as a polyline through vpy3d (occlude: through the occluder), or in 2D
 * with x, y as device units */
void vpyrope_draw(int rope, int br, int occlude);
void vpyrope_draw2d(int rope, int br);

typedef struct {
    uint32_t ropes, points;     /* in use */
    uint32_t refused;           /* ropes not made: tables full */
    int32_t  stretch;           /* worst link last step, units longer than its length */
} vpyrope_stats_t;
const vpyrope_stats_t *vpyrope_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYROPE_H */
