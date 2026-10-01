/*
 * vpyik.c — see vpyik.h. Distances in world units; directions Q14.
 */
#include "vpyik.h"

#define N1 16384

static int64_t isqrt64(int64_t n)
{
    if (n <= 0) return 0;
    int64_t r = 0, bit = (int64_t)1 << 62;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return r;
}

int vpyik_two_bone(const int32_t root[3], const int32_t target[3], int32_t l1, int32_t l2,
                   const int32_t pole[3], int32_t joint[3], int32_t end[3])
{
    int64_t d[3], d2 = 0;
    for (int k = 0; k < 3; k++) { d[k] = (int64_t)target[k] - root[k]; d2 += d[k] * d[k]; }
    int64_t dist = isqrt64(d2);
    int reached = 1;
    const int64_t reach = (int64_t)l1 + l2, inner = l1 > l2 ? (int64_t)l1 - l2 : (int64_t)l2 - l1;
    if (dist == 0) { d[0] = 0; d[1] = N1; d[2] = 0; dist = 1; reached = 0; }
    /* the root -> target direction, Q14 */
    int64_t u[3];
    for (int k = 0; k < 3; k++) u[k] = d[k] * N1 / dist;
    int64_t len = dist;
    if (len > reach) { len = reach; reached = 0; }        /* stretched straight towards it */
    if (len < inner) { len = inner; reached = 0; }        /* folded as far as it goes */
    /* the bend direction: the pole, minus its part along u, normalised */
    int64_t p[3], pu = 0;
    for (int k = 0; k < 3; k++) { p[k] = (int64_t)pole[k] - root[k]; pu += p[k] * u[k]; }
    pu /= N1;
    int64_t b[3], b2 = 0;
    for (int k = 0; k < 3; k++) { b[k] = p[k] - (pu * u[k]) / N1; b2 += b[k] * b[k]; }
    int64_t bl = isqrt64(b2);
    if (bl == 0) {                                        /* the pole on the line: any side will do */
        const int ax = (u[0] < 0 ? -u[0] : u[0]) < (u[1] < 0 ? -u[1] : u[1]) ? 0 : 1;
        int64_t e[3] = { 0, 0, 0 }; e[ax] = N1;
        b[0] = e[1] * u[2] - e[2] * u[1]; b[1] = e[2] * u[0] - e[0] * u[2]; b[2] = e[0] * u[1] - e[1] * u[0];
        for (int k = 0; k < 3; k++) b[k] /= N1;
        bl = isqrt64(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        if (bl == 0) bl = 1;
    }
    /* law of cosines: how far along u the joint sits, and how far out to the side */
    const int64_t a = ((int64_t)l1 * l1 - (int64_t)l2 * l2 + len * len) / (2 * len);
    const int64_t h = isqrt64((int64_t)l1 * l1 - a * a);
    for (int k = 0; k < 3; k++) {
        joint[k] = (int32_t)(root[k] + (u[k] * a) / N1 + (b[k] * h) / bl);
        end[k] = (int32_t)(root[k] + (u[k] * len) / N1);
    }
    return reached;
}
