/* ai_check — vpyai's steering and paths, against known answers and Dijkstra.
 *
 *   cc -O2 -w -Iinclude tools/ai_check.c vpyai.c -lm -o /tmp/ai_check && /tmp/ai_check
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "vpyai.h"

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

/* Dijkstra with the same costs and the same corner rule: the reference */
static uint32_t dijkstra(const uint8_t *g, int w, int h, int s, int t, int diag)
{
    static uint32_t d[4096]; static uint8_t done[4096];
    for (int i = 0; i < w * h; i++) { d[i] = 0xFFFFFFFFu; done[i] = 0; }
    d[s] = 0;
    static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 }, DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    for (;;) {
        int c = -1;
        for (int i = 0; i < w * h; i++) if (!done[i] && d[i] != 0xFFFFFFFFu && (c < 0 || d[i] < d[c])) c = i;
        if (c < 0) return 0xFFFFFFFFu;
        if (c == t) return d[c];
        done[c] = 1;
        const int cx = c % w, cy = c / w;
        for (int k = 0; k < (diag ? 8 : 4); k++) {
            const int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const int nc = ny * w + nx;
            if (g[nc] == 255) continue;
            if (k >= 4 && (g[cy * w + nx] == 255 || g[ny * w + cx] == 255)) continue;
            const uint32_t nd = d[c] + (k >= 4 ? 14u : 10u) + g[nc];
            if (nd < d[nc]) d[nc] = nd;
        }
    }
}

int main(void)
{
    int32_t p[3] = { 0, 0, 0 }, t[3] = { 300, 400, 0 }, v[3];
    vpyai_seek(p, t, 100, v);
    CHECK(v[0] == 60 && v[1] == 80, "seek: (60, 80) towards (300, 400) at 100 -> (%d, %d)", v[0], v[1]);
    vpyai_flee(p, t, 100, v);
    CHECK(v[0] == -60 && v[1] == -80, "flee: straight away (%d, %d)", v[0], v[1]);
    vpyai_arrive(p, t, 100, 1000, v);
    CHECK(v[0] == 30 && v[1] == 40, "arrive: 500 from the target inside a 1000 radius, half speed (%d, %d)", v[0], v[1]);
    int32_t q[3] = { 295, 396, 0 }; vpyai_arrive(q, t, 100, 1000, v);
    CHECK(v[0] == 5 && v[1] == 4, "arrive: 5 units away it covers exactly the rest, (%d, %d), not a stall short of it", v[0], v[1]);

    int32_t vel[3] = { 100, 0, 0 }; const int32_t want[3] = { 0, 100, 0 };
    vpyai_steer(vel, want, 20);
    CHECK(abs(vel[0] - 86) <= 1 && abs(vel[1] - 14) <= 1, "steer: turns by at most 20 a step (%d, %d)", vel[0], vel[1]);

    const int32_t others[2][3] = { { 50, 0, 0 }, { 0, 0, 0 } };
    vpyai_separate(p, others, 2, 100, 40, v);
    CHECK(v[0] == -20 && v[1] == 0, "separate: half the strength from a neighbour half a radius away; one on top skipped (%d, %d)", v[0], v[1]);

    /* paths */
    static uint8_t g[32 * 32]; static int16_t path[2 * 1024];
    memset(g, 0, sizeof g);
    int n = vpyai_path(g, 32, 32, 0, 0, 10, 0, 0, path, 1024);
    CHECK(n == 11 && vpyai_path_cost() == 100, "open grid, 10 cells right: %d cells, cost %d", n, vpyai_path_cost());
    for (int y = 0; y < 31; y++) g[y * 32 + 16] = 255;     /* a wall with a gap at the bottom */
    n = vpyai_path(g, 32, 32, 0, 0, 31, 0, 1, path, 1024);
    int through = 0, hits = 0;
    for (int i = 0; i < n; i++) { if (path[2 * i] == 16 && path[2 * i + 1] == 31) through = 1; if (g[path[2 * i + 1] * 32 + path[2 * i]] == 255) hits++; }
    CHECK(n > 0 && through && !hits, "a wall with one gap: the path goes through the gap, never through the wall");
    g[31 * 32 + 16] = 255;
    CHECK(vpyai_path(g, 32, 32, 0, 0, 31, 0, 1, path, 1024) == 0, "the gap closed: no path (0)");
    CHECK(vpyai_path(g, 40, 40, 0, 0, 1, 1, 1, path, 1024) == -1, "a grid past VPYAI_MAX_CELLS: refused (-1)");

    /* optimal against Dijkstra, on random grids with walls and costs */
    srand(42); int ok = 1, tried = 0;
    for (int trial = 0; trial < 200; trial++) {
        for (int i = 0; i < 32 * 32; i++) { const int r = rand() % 100; g[i] = r < 22 ? 255 : (r < 40 ? (uint8_t)(rand() % 30) : 0); }
        const int s = rand() % (32 * 32), e = rand() % (32 * 32);
        g[s] = 0; g[e] = 0;
        const int diag = trial & 1;
        n = vpyai_path(g, 32, 32, s % 32, s / 32, e % 32, e / 32, diag, path, 1024);
        const uint32_t ref = dijkstra(g, 32, 32, s, e, diag);
        if (ref == 0xFFFFFFFFu) { if (n != 0) ok = 0; continue; }
        tried++;
        if (n <= 0 || (uint32_t)vpyai_path_cost() != ref) { ok = 0; printf("        trial %d: A* %d, Dijkstra %u\n", trial, vpyai_path_cost(), ref); }
    }
    CHECK(ok, "200 random 32x32 grids (walls, mud, 4 and 8 neighbours): A* cost = Dijkstra's on all %d with a path", tried);

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
