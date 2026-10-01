/*
 * vpyai.h — steering for things that move, and paths across a grid.
 *
 * STEERING. Each behaviour answers "what velocity would I like now?" and
 * vpyai_steer() turns the velocity a body has towards it by at most so much a
 * step — which is what makes a chaser swing round instead of snapping.
 *   seek     straight at a target at full speed
 *   flee     straight away from it
 *   arrive   at it, slowing inside a radius so it stops ON the target
 *   separate a push away from neighbours closer than a radius (flocks, crowds)
 * Vectors are int32 [3]; a 2D game leaves z at 0. Speeds in any unit per
 * whatever step the game uses — they come back in the same.
 *
 * PATHS. A* across a grid of cells: 0 free, 255 a wall, 1..254 free but that
 * much dearer to cross (mud, danger). Four or eight neighbours. The answer is
 * the cheapest path — checked against Dijkstra on random grids — with ties
 * broken the same way every time, and the memory is fixed: VPYAI_MAX_CELLS.
 *
 * Integer only, deterministic.
 */
#ifndef VPYAI_H
#define VPYAI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void vpyai_seek(const int32_t pos[3], const int32_t target[3], int32_t speed, int32_t out[3]);
void vpyai_flee(const int32_t pos[3], const int32_t threat[3], int32_t speed, int32_t out[3]);
void vpyai_arrive(const int32_t pos[3], const int32_t target[3], int32_t speed, int32_t slow_radius, int32_t out[3]);
/* the push away from every neighbour within `radius`, stronger the closer, at
 * most `strength`; neighbours exactly on top of pos are skipped */
void vpyai_separate(const int32_t pos[3], const int32_t (*others)[3], int n,
                    int32_t radius, int32_t strength, int32_t out[3]);
/* move velocity v towards `desired` by at most `max_change` (a turn rate) */
void vpyai_steer(int32_t v[3], const int32_t desired[3], int32_t max_change);

#ifndef VPYAI_MAX_CELLS
#define VPYAI_MAX_CELLS 1024        /* the largest grid a path is found across */
#endif
#define VPYAI_WALL 255

/* The cheapest path from (sx,sy) to (gx,gy), as cells from start to goal into
 * out_xy (x0,y0, x1,y1, ...), at most max_cells of them. Returns how many
 * cells; 0 if there is no way through; -1 if the grid is bigger than
 * VPYAI_MAX_CELLS, an end is off the grid or on a wall, or the path does not
 * fit max_cells. Diagonal moves (diagonal = 1) cost 14 against 10 straight and
 * never cut a wall's corner. */
int vpyai_path(const uint8_t *grid, int w, int h, int sx, int sy, int gx, int gy,
               int diagonal, int16_t *out_xy, int max_cells);
/* what the last path cost: 10 a straight step, 14 a diagonal, plus the cells' own costs */
int32_t vpyai_path_cost(void);

#ifdef __cplusplus
}
#endif

#endif /* VPYAI_H */
