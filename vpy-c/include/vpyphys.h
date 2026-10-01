/*
 * vpyphys.h — rigid bodies, gravity and collisions for libvpy.
 *
 * WHAT THIS IS. Bodies that fall, bounce, slide, come to rest and push each
 * other, and the contacts between them reported with how hard they hit — the
 * part every game with physics needs and nobody should write twice. It draws
 * nothing: the game reads positions back and draws them however it likes (with
 * vpy3d meshes, with 2D shapes). A 2D game uses x and y and leaves z at 0.
 *
 * INTEGER ONLY, like the rest of libvpy, and DETERMINISTIC: the same calls in
 * the same order give the same positions on the cartridge, on the host and in
 * the simulator, bit for bit. Replays and host harnesses depend on that. No
 * wall clock anywhere: the simulation advances one fixed step per vpyp_step().
 *
 * UNITS
 *   position   world units (int32), the same unit as vpy3d — mm works well.
 *              Internally Q8 (1/256 of a unit), which is what makes slow motion
 *              and small gravity possible in integers.
 *   velocity   world units per SECOND
 *   gravity    world units per second², default 0 (set it: 9800 mm/s² is Earth)
 *   mass       any positive integer, relative to the other bodies; 0 = static
 *   rate       steps per second (vpyp_set_rate), default 50 — one per frame
 *   material   restitution and friction in Q8 (256 = 1.0)
 *
 * SHAPES. Spheres and axis-aligned boxes, moving or static, and an optional
 * floor plane. BOXES DO NOT ROTATE: they collide as axis-aligned boxes whatever
 * a game draws, and there is no angular velocity yet. Spinning a box's MESH is
 * cosmetic. Rotation is the next step for this module (see TODO.md).
 *
 * THE SOLVER is sequential impulses with warm starting: gravity into the
 * velocities, contacts found (speculatively — see below), last step's impulses
 * re-applied, a few passes of impulses (restitution and Coulomb friction),
 * positions advanced, overlap pushed apart. A body that stays still for half a
 * second while something holds it up goes to SLEEP and costs almost nothing;
 * it wakes when hit hard, or when whatever held it up goes away.
 *
 * WHAT IT GETS RIGHT, measured on the host (vpy-c/tools/phys_check.c):
 *   - velocity under gravity exactly; position half a step AHEAD of the
 *     continuous formula — the integrator is semi-implicit Euler, the stable
 *     choice, and that is its known bias (2.5% of the drop after 0.5 s at 50 Hz)
 *   - a stack of five boxes holds within 3 units and sleeps; pull the bottom
 *     one out and the rest fall
 *   - a box sliding at 2000 units/s with friction 0.5 stops within 5% of
 *     v²/(2µg); an elastic head-on collision swaps velocities; momentum is kept
 *   - bit-for-bit identical results from the same calls
 *
 * CONTACTS ARE SPECULATIVE: found as far ahead as the bodies can travel in one
 * substep, and the solver lets the gap close but no further. So a fast body
 * does not tunnel through a thin wall (measured: 600 units per step through a
 * 20-unit wall, stopped), but a bounce happens up to one step of travel BEFORE
 * the surface — a ball dropped at 50 Hz bounces a little higher than its
 * restitution says. vpyp_set_substeps() makes that step smaller.
 *
 * WHAT IS COUNTED (vpyp_stats): pairs tested, contacts, bodies awake, and every
 * request that was refused — the table full, the contact list full. A
 * refusal is never silent. vpyp_stats()->bodies against VPYP_MAX_BODIES is how a
 * game sees the table is full BEFORE asking.
 */
#ifndef VPYPHYS_H
#define VPYPHYS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYP_MAX_BODIES
#define VPYP_MAX_BODIES   64      /* every pair is tested: 64 is 2016 pairs a step */
#endif
#ifndef VPYP_MAX_CONTACTS
#define VPYP_MAX_CONTACTS 256     /* 128 overflowed with 64 bodies piled up (measured) */
#endif

#define VPYP_FLOOR   (-2)         /* the floor's id in contacts and ray hits */
#define VPYP_NONE    (-1)

/* ---- the world ----------------------------------------------------------- */
void vpyp_reset(void);                          /* no bodies, no floor, no gravity */
void vpyp_set_rate(int steps_per_second);       /* default 50 */
void vpyp_set_substeps(int n);                  /* default 1; see FAST AND SMALL */
void vpyp_set_gravity(int32_t gx, int32_t gy, int32_t gz);   /* units/s² */
/* A static plane y = `y`, facing up. Off by default. */
void vpyp_set_floor(int on, int32_t y, int restitution_q8, int friction_q8);

/* One fixed step of 1/rate seconds. Call it once per frame at 50 Hz. */
void vpyp_step(void);

/* ---- bodies ---------------------------------------------------------------
 * Each returns the body's id, or VPYP_NONE if the table is full (counted in
 * vpyp_stats()->refused). mass 0 makes it static: it never moves and pushes
 * everything else. */
int  vpyp_add_sphere(int32_t x, int32_t y, int32_t z, int32_t radius, int32_t mass);
int  vpyp_add_box(int32_t x, int32_t y, int32_t z,
                  int32_t half_x, int32_t half_y, int32_t half_z, int32_t mass);
void vpyp_remove(int id);
int  vpyp_alive(int id);

/* Restitution (bounciness) and friction, Q8. Default 64 (0.25) and 128 (0.5).
 * Two bodies in contact use the larger restitution and the geometric mean of
 * the frictions. */
void vpyp_set_material(int id, int restitution_q8, int friction_q8);
/* Which bodies meet which: two bodies collide when (a.mask & b.mask) != 0.
 * Default 1 for every body. A shot that should not hit its shooter puts them
 * in different bits. The floor meets everything. */
void vpyp_set_mask(int id, uint8_t mask);

void vpyp_set_position(int id, int32_t x, int32_t y, int32_t z);   /* teleport */
void vpyp_set_velocity(int id, int32_t vx, int32_t vy, int32_t vz); /* units/s */
/* A kick: mass × units/s, divided by the body's mass. Wakes it. */
void vpyp_apply_impulse(int id, int32_t ix, int32_t iy, int32_t iz);

void vpyp_position(int id, int32_t *x, int32_t *y, int32_t *z);
void vpyp_velocity(int id, int32_t *vx, int32_t *vy, int32_t *vz);  /* units/s */
int  vpyp_sleeping(int id);

/* ---- what touched what, this step ----------------------------------------
 * Every contact of the last vpyp_step, with how hard it was. `impulse` is the
 * normal impulse the solver applied (mass × units/s): 0 for things resting on
 * each other, large for a crash — the number a sound or a dent wants. `b` is
 * VPYP_FLOOR for the floor. `normal` points from a to b, Q14. */
typedef struct {
    int     a, b;
    int32_t x, y, z;            /* a point on the contact, world units */
    int16_t nx, ny, nz;         /* Q14 */
    int32_t depth;              /* overlap found, world units */
    int32_t impulse;            /* mass × units/s */
} vpyp_contact;
int                 vpyp_contact_count(void);
const vpyp_contact *vpyp_contact_get(int i);

/* ---- rays ----------------------------------------------------------------
 * The first thing a ray from (ox,oy,oz) along (dx,dy,dz) meets within
 * `max_dist` units: a body's id, VPYP_FLOOR, or VPYP_NONE. The direction need
 * not be normalised. Bodies whose mask shares no bit with `mask` are ignored. */
typedef struct {
    int     id;
    int32_t x, y, z;            /* the hit point, world units */
    int16_t nx, ny, nz;         /* surface normal there, Q14 */
    int32_t dist;               /* from the origin, world units */
} vpyp_hit;
int vpyp_raycast(int32_t ox, int32_t oy, int32_t oz,
                 int32_t dx, int32_t dy, int32_t dz,
                 int32_t max_dist, uint8_t mask, vpyp_hit *out);

/* ---- what did it cost, and was anything refused --------------------------- */
typedef struct {
    uint32_t bodies;            /* alive */
    uint32_t awake;             /* moving, i.e. costing CPU */
    uint32_t pairs;             /* pairs tested last step */
    uint32_t contacts;          /* contacts found last step */
    uint32_t refused;           /* bodies not added because the table was full: a
                                   limit the game has to handle, counted so it is
                                   never silent */
    uint32_t contacts_dropped;  /* contacts past VPYP_MAX_CONTACTS: two bodies that
                                   may pass through each other. NOT ZERO = raise it */
} vpyp_stats_t;
const vpyp_stats_t *vpyp_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYPHYS_H */
