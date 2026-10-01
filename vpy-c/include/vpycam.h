/*
 * vpycam.h — a camera that follows, shakes, and stops time on a hit.
 *
 * FOLLOW. The camera keeps a focus point. Each vpycam_follow() moves it towards
 * the target, but only once the target leaves a DEAD ZONE round the focus (so
 * small moves do not make the whole world slide), LEADING the target by its
 * velocity (so you see where it is going), and SMOOTHLY (a fraction of the gap
 * each step, snapping when it is all but closed — a camera that creeps by one
 * unit a frame re-zeroes the beam in a different place every frame).
 *
 * SHAKE. vpycam_shake() adds trauma; the offset it gives decays to nothing over
 * the frames asked for. Deterministic, from its own generator.
 *
 * HIT-STOP. vpycam_hitstop(n): for the next n frames vpycam_stopped() is 1 — skip
 * the simulation step and keep drawing, and a big hit lands.
 *
 * vpycam_look_at() puts it together: vpy3d's camera at focus + offset, looking at
 * the focus, both shaken.
 *
 * Units: world units; velocities in units per frame (what a game already has).
 * One vpycam_step() per frame. Integer only.
 */
#ifndef VPYCAM_H
#define VPYCAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void vpycam_reset(int32_t fx, int32_t fy, int32_t fz);   /* focus there, no shake, no stop */
void vpycam_seed(uint32_t seed);

/* dead zone half-sizes (x, y, z), lead in frames of the target's velocity, and
 * smoothing: the focus closes 1/smooth of the gap each step (1 = no lag) */
void vpycam_follow_config(int32_t dead_x, int32_t dead_y, int32_t dead_z, int lead_frames, int smooth);
void vpycam_follow(int32_t tx, int32_t ty, int32_t tz, int32_t vx, int32_t vy, int32_t vz);
void vpycam_focus(int32_t *x, int32_t *y, int32_t *z);

void vpycam_shake(int32_t amount, int frames);           /* the larger shake wins */
void vpycam_shake_offset(int32_t *dx, int32_t *dy, int32_t *dz);

void vpycam_hitstop(int frames);                         /* the longer stop wins */
int  vpycam_stopped(void);

void vpycam_step(void);                                  /* once a frame */

/* vpy3d_look_at from focus + (ox, oy, oz) to the focus, with the shake */
int  vpycam_look_at(int32_t ox, int32_t oy, int32_t oz);

#ifdef __cplusplus
}
#endif

#endif /* VPYCAM_H */
