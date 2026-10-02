/*
 * vpybone.h — skeletal animation: a tree of rigid bones, posed by keyframed clips.
 *
 * RIGID PARTS FIRST. Each bone is a JOINT placed at an offset from its parent's
 * joint (in the parent's frame) and turned by its own local rotation; whatever
 * hangs from it — a limb's mesh — moves rigidly with it. That is the cheap and
 * exact kind of skinning: no vertex is shared between bones, so nothing stretches,
 * and on a vector display a limb drawn as its own small mesh reads perfectly well.
 *
 * ROTATIONS ARE QUATERNIONS IN Q14 (w, x, y, z; 16384 = 1.0), like the rest of
 * libvpy's fixed point. Forward kinematics composes them root first — a bone's
 * parent is always added before it, so one pass in table order is enough — and
 * hands each bone a vpy_xf (Q14 matrix + position) for vpy3d_draw_mesh.
 *
 * CLIPS are per-bone tracks of keys (frame, rotation), sampled with normalised
 * linear interpolation (nlerp: as smooth as slerp for the small steps between
 * keys, and needs no trigonometry), looping or held at the ends. Two clips blend
 * with a weight — walk into run — and a limb can be handed to the two-bone IK
 * (vpyik_two_bone) for feet on uneven ground or a hand on a lever.
 *
 * Integer only and deterministic. A skeleton is the caller's (declare it static:
 * it is ~2 KB at VPYB_MAX_BONES and a .um2's core 0 has 4 KB of stack). Refusals
 * are counted (vpyb_stats), never silent.
 *
 * TIME is in frames, Q8 (256 = one frame), so a clip can be played slower or
 * faster than one key per frame without stepping.
 */
#ifndef VPYBONE_H
#define VPYBONE_H

#include <stdint.h>
#include "vpy3d.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VPYB_MAX_BONES
#define VPYB_MAX_BONES 24
#endif

typedef struct { int32_t w, x, y, z; } vpyb_quat;          /* Q14, unit length */

/* `angle` (4096 per turn) about the axis (ax, ay, az), any length. */
vpyb_quat vpyb_quat_axis(int32_t ax, int32_t ay, int32_t az, int angle);
vpyb_quat vpyb_quat_identity(void);

typedef struct {
    uint8_t         n;
    int8_t          parent[VPYB_MAX_BONES];       /* -1 for a root */
    int32_t         off[VPYB_MAX_BONES][3];       /* joint offset from the parent's joint */
    const vpy_mesh *mesh[VPYB_MAX_BONES];         /* drawn at the bone's frame; may be 0 */
    vpyb_quat       rot[VPYB_MAX_BONES];          /* local rotation (what clips set) */
    /* forward kinematics, written by vpyb_pose */
    vpyb_quat       wrot[VPYB_MAX_BONES];
    int32_t         pos[VPYB_MAX_BONES][3];
    vpyb_quat       root_rot;
    int32_t         root_pos[3];
} vpyb_skeleton;

void vpyb_init(vpyb_skeleton *s);
/* A bone at (ox, oy, oz) from `parent`'s joint (-1: a root, offset from the
 * skeleton's position). Returns its index, or -1 if the table is full or the
 * parent does not exist yet (counted in vpyb_stats()->refused). */
int  vpyb_add(vpyb_skeleton *s, int parent, int32_t ox, int32_t oy, int32_t oz, const vpy_mesh *mesh);

/* Forward kinematics: every bone's world rotation and joint position, the
 * skeleton placed at `pos` turned by `rot`. */
void vpyb_pose(vpyb_skeleton *s, const int32_t pos[3], vpyb_quat rot);
/* A bone's world frame, for vpy3d (and for attaching things to it: a sword in a hand). */
vpy_xf vpyb_xf(const vpyb_skeleton *s, int bone);
/* Draw it: each bone's mesh at its frame (VPYB_DRAW_MESH), a line from each joint
 * to its parent's (VPYB_DRAW_LINES), or both. Returns the bones drawn. */
#define VPYB_DRAW_MESH  1
#define VPYB_DRAW_LINES 2
int  vpyb_draw(const vpyb_skeleton *s, int br, int what);

/* ---- clips ------------------------------------------------------------------ */
typedef struct { uint16_t frame; vpyb_quat q; } vpyb_key;  /* keys in rising frame order */
typedef struct { const vpyb_key *keys; uint8_t n; } vpyb_track;   /* n = 0: bone untouched */
typedef struct {
    const vpyb_track *tracks;    /* one per bone, in bone order */
    uint8_t  bones;              /* how many tracks */
    uint16_t length;             /* frames; a loop wraps here, back to frame 0 */
    uint8_t  loop;
} vpyb_clip;

/* A track's rotation at `t_q8` (frames, Q8). */
vpyb_quat vpyb_sample(const vpyb_track *tr, int32_t t_q8, uint16_t length, int loop);
/* Set the skeleton's local rotations from a clip at `t_q8`. */
void vpyb_apply(vpyb_skeleton *s, const vpyb_clip *c, int32_t t_q8);
/* The same from two clips, `w_q8` of the way from a to b (0 = all a, 256 = all b).
 * A bone only one clip animates takes that clip's rotation. */
void vpyb_apply_blend(vpyb_skeleton *s, const vpyb_clip *a, int32_t ta_q8,
                      const vpyb_clip *b, int32_t tb_q8, int w_q8);

/* ---- a limb by IK --------------------------------------------------------------
 * The limb is `upper`, its child and its grandchild (the first child each time:
 * hip, knee, ankle). After vpyb_pose, this turns `upper` and its child so the
 * grandchild's joint lands on `target` (or as near as the limb reaches), the knee
 * bending towards `pole`, and poses again. Returns 1 if reached, 0 if stretched,
 * -1 if `upper` has no child and grandchild (counted as ik_refused). */
int  vpyb_ik(vpyb_skeleton *s, int upper, const int32_t target[3], const int32_t pole[3]);

typedef struct {
    uint32_t refused;       /* bones not added: table full, or no such parent */
    uint32_t ik_refused;    /* vpyb_ik on a bone without two bones below it */
    uint32_t ik_stretched;  /* vpyb_ik targets out of reach */
    uint32_t bones;         /* bones posed by the last vpyb_pose */
    int32_t  last_t_q8;     /* the last time a clip was sampled at, after looping */
} vpyb_stats_t;
const vpyb_stats_t *vpyb_stats(void);
void vpyb_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYBONE_H */
