/*
 * vpyik.h — two-bone inverse kinematics: where the elbow (or knee) goes.
 *
 * An arm is a root (shoulder), two bones of fixed length and an end (hand).
 * Given where the hand should be, vpyik_two_bone finds the elbow: the hand on
 * the target if it can be reached, the arm stretched straight towards it if
 * not. Of the circle of elbows that would do, the one bending towards `pole`
 * (a point the elbow should lean to — in front of a knee, behind an elbow).
 *
 * Feet on uneven ground, a hand on a lever, a head that looks: each is one call
 * a frame. Integer only (law of cosines with an integer square root).
 */
#ifndef VPYIK_H
#define VPYIK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 if the target was reached, 0 if the arm had to stretch towards it. */
int vpyik_two_bone(const int32_t root[3], const int32_t target[3], int32_t l1, int32_t l2,
                   const int32_t pole[3], int32_t joint[3], int32_t end[3]);

#ifdef __cplusplus
}
#endif

#endif /* VPYIK_H */
