/*
 * vpyent.h — entities: the bookkeeping every 3D game with physics writes by hand.
 *
 * AN ENTITY IS ONE THING IN THE SCENE: where it is (a vpyphys body's position and
 * turn, or a transform the game sets), what it looks like (a vpy3d mesh, shared, or
 * its own copy once it is dented), what hides behind it (an occluder shape), the
 * marks shots left on it, what it sounds like when it hits (a vpyimpact material),
 * and a kind and a pointer for the game. A fixed table, no allocation.
 *
 * WHY IT EXISTS: THE ORDERING TRAP. The occluder only works drawn near to far, each
 * solid drawn and THEN added as an occluder, marks before the occluder that would
 * hide them, shadows after the solids — and a game that gets one of these backwards
 * gets see-through solids or marks cut by their own object, which read as "the
 * occluder is broken". physics_demo wrote that order out by hand (sort by distance,
 * draw, marks, occluder corners per kind); vpyent_draw() does it once, the same way.
 *
 * THE STEP, IN ORDER: vpyent_step() advances the camera's shake and hit-stop
 * (vpycam_step — do not call it yourself as well), ages the impact sound, and —
 * unless a hit-stop holds time — steps the physics, sounds the loudest contact with
 * each body's material, and steps the effects.
 *
 * It needs vpy3d, vpyphys, vpyimpact, vpyfx and vpycam linked. Integer only and
 * deterministic like all of them. Every refusal is counted (vpyent_stats).
 */
#ifndef VPYENT_H
#define VPYENT_H

#include <stdint.h>
#include "vpy3d.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYENT_MAX
#define VPYENT_MAX 64             /* the same as vpyphys's table: one entity per body */
#endif
#define VPYENT_NONE (-1)

/* What hides what is behind the entity. MESH (the default) uses the entity's own
 * mesh when it has 8 vertices or fewer (vpy3d_occl_add_mesh's limit) — a mesh with
 * more and no other shape given hides nothing, and that is COUNTED (occl_missing),
 * because a solid you can see through is exactly the fault this layer exists to
 * prevent. BOX is a box of half extents turned with the entity (a crate whose mesh
 * has face centres, say). SPHERE is the octahedron inside a ball — a little less
 * than the ball, never more. NONE: it hides nothing (a wire, a plate). */
enum { VPYENT_OCC_MESH = 0, VPYENT_OCC_BOX, VPYENT_OCC_SPHERE, VPYENT_OCC_NONE };

/* A new entity drawn with `mesh` (kept by pointer: it must outlive the entity) and
 * placed by vpyphys body `body`, or by vpyent_set_place() if body is VPYP_NONE (-1).
 * Returns its id, or VPYENT_NONE when the table is full (counted in `refused`). */
int  vpyent_create(const vpy_mesh *mesh, int body);
/* Gone, and its body with it (vpyp_remove: what it held up wakes). */
void vpyent_destroy(int e);
void vpyent_reset(void);                 /* every entity gone, their bodies too */
int  vpyent_alive(int e);
/* Iterate: for (int e = vpyent_next(-1); e >= 0; e = vpyent_next(e)) ... */
int  vpyent_next(int e);
/* The entity a body belongs to — what a vpyp_raycast hit or a contact gives you. */
int  vpyent_of_body(int body);

void vpyent_set_place(int e, const vpy_xf *place);   /* for an entity with no body */
void vpyent_place(int e, vpy_xf *out);               /* where it is now, either way */
int  vpyent_body(int e);
void vpyent_set_mesh(int e, const vpy_mesh *mesh);   /* drops its own dented copy */
void vpyent_set_brightness(int e, int br);           /* default 100 */
void vpyent_set_occluder(int e, int kind, int32_t hx, int32_t hy, int32_t hz);  /* SPHERE: hx = radius */
void vpyent_set_material(int e, int material);       /* a VPYI_* value; default VPYI_NONE */
void vpyent_set_kind(int e, int kind);
int  vpyent_kind(int e);
void vpyent_set_user(int e, void *user);
/* WHAT A MESH IS NOT: a figure's legs drawn as lines, a face, a shadow under it, a
 * cage round it. Drawn after vpyent_draw() they are in the wrong place in the order:
 * the occluder has no depth test, so every solid added before them — the platform the
 * figure stands on included — cuts them wherever it is on screen, and a figure on a
 * box loses its legs into it (Spike 3D, 2026-10-04). An entity's EXTRA is called by
 * vpyent_draw right after the entity (and its marks), before its own occluder: what
 * it draws with vpy3d_occl_line, or as meshes, is cut by what is nearer and by
 * nothing else. `place` is where the entity is; `user` what vpyent_set_user gave. */
typedef void (*vpyent_extra_fn)(int e, const vpy_xf *place, void *user);
void vpyent_set_extra(int e, vpyent_extra_fn fn);     /* 0: none (the default) */
/* THE ORDER IS BY CENTRES, and a big flat solid's centre can be nearer the eye than
 * something standing on its far half — which is then drawn after the solid's occluder
 * went in, and is cut by it (Spike on a platform, 2026-10-04). An entity with a sort
 * bias sorts as if it were `nearer` world units nearer the eye: for what stands on
 * solids, about the half size of the largest it stands on. 0 is the default. */
void vpyent_set_sort_bias(int e, int32_t nearer);
void *vpyent_user(int e);

/* A dent at a world point, pushed along a world direction (vpy3d_mesh_dent). The
 * first dent gives the entity its own copy of the mesh; 0 if vpy3d's pools cannot
 * hold one (counted in dents_refused) — size them with VPY3D_POOL_* for as many
 * dentable entities as you have. */
int  vpyent_dent(int e, int32_t px, int32_t py, int32_t pz,
                 int32_t dx, int32_t dy, int32_t dz, int32_t depth, int32_t radius);
/* A mark where a shot landed: a vpyp_hit / vpy3d_hit's point and normal as given. */
void vpyent_mark(int e, int32_t x, int32_t y, int32_t z, int32_t nx, int32_t ny, int32_t nz);
void vpyent_set_marks(int e, int32_t radius, int br, int style);   /* default 40, 127, ring */

/* THE SCENE, NEAR TO FAR: every entity by its distance from vpy3d's eye, nearest
 * first; each drawn (with mesh occlusion on for the call), its marks, then its
 * occluder. Call it after vpy3d_occl_reset() and after whatever must be in FRONT of
 * every entity (an aim, a HUD in the world), and before what everything may hide
 * (effects, shadows, the floor). Returns how many were drawn. */
int  vpyent_draw(void);

/* SHADOWS, for what is in the air: each entity whose lowest occluder corner is more
 * than `lift` above `floor_y` throws its occluder's outline on the floor along the
 * light (vpy3d_shadow), at VPY_PRI_LOW so a full frame sheds them first. Resting
 * things hide their own. Call it after the solids and the effects. */
int  vpyent_draw_shadows(int32_t lx, int32_t ly, int32_t lz, int32_t floor_y, int32_t lift, int br);

/* One frame of the world: see THE STEP above. `floor_material` is what the floor
 * sounds like (VPYI_NONE: whatever lands on it decides). Returns 1 if time moved,
 * 0 if a hit-stop held it. */
int  vpyent_step(int floor_material);

typedef struct {
    uint32_t alive;             /* entities in use */
    uint32_t refused;           /* creates refused: the table was full */
    uint32_t drawn;             /* entities drawn by the last vpyent_draw */
    uint32_t occl_missing;      /* last draw: entities that should hide and could not */
    uint32_t dents_refused;     /* dents not made: no room for the entity's own mesh */
} vpyent_stats_t;
const vpyent_stats_t *vpyent_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYENT_H */
