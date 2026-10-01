/*
 * vpyai.c — see vpyai.h.
 */
#include "vpyai.h"

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

/* (to - from) scaled to `len` */
static void toward(const int32_t from[3], const int32_t to[3], int64_t len, int32_t out[3])
{
    int64_t d[3], l2 = 0;
    for (int k = 0; k < 3; k++) { d[k] = (int64_t)to[k] - from[k]; l2 += d[k] * d[k]; }
    const int64_t l = isqrt64(l2);
    for (int k = 0; k < 3; k++) out[k] = l ? (int32_t)(d[k] * len / l) : 0;
}

void vpyai_seek(const int32_t pos[3], const int32_t target[3], int32_t speed, int32_t out[3])
{
    toward(pos, target, speed, out);
}
void vpyai_flee(const int32_t pos[3], const int32_t threat[3], int32_t speed, int32_t out[3])
{
    toward(threat, pos, speed, out);
}
void vpyai_arrive(const int32_t pos[3], const int32_t target[3], int32_t speed, int32_t slow_radius, int32_t out[3])
{
    int64_t l2 = 0;
    for (int k = 0; k < 3; k++) { const int64_t d = (int64_t)target[k] - pos[k]; l2 += d * d; }
    const int64_t l = isqrt64(l2);
    /* full speed outside the radius; inside, as fast as the distance left */
    int64_t s = (slow_radius > 0 && l < slow_radius) ? (int64_t)speed * l / slow_radius : speed;
    /* the last few units round the slowed speed to nothing, and a body that
     * never covers them never arrives: take what is left in one go */
    if (s == 0) s = l;
    toward(pos, target, s > l ? l : s, out);     /* never overshoot in one step */
}

void vpyai_separate(const int32_t pos[3], const int32_t (*others)[3], int n,
                    int32_t radius, int32_t strength, int32_t out[3])
{
    int64_t acc[3] = { 0, 0, 0 };
    for (int i = 0; i < n; i++) {
        int64_t d[3], l2 = 0;
        for (int k = 0; k < 3; k++) { d[k] = (int64_t)pos[k] - others[i][k]; l2 += d[k] * d[k]; }
        const int64_t l = isqrt64(l2);
        if (l == 0 || l >= radius) continue;
        const int64_t push = (int64_t)strength * (radius - l) / radius;   /* stronger the closer */
        for (int k = 0; k < 3; k++) acc[k] += d[k] * push / l;
    }
    const int64_t al = isqrt64(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    for (int k = 0; k < 3; k++) out[k] = (int32_t)(al > strength ? acc[k] * strength / al : acc[k]);
}

void vpyai_steer(int32_t v[3], const int32_t desired[3], int32_t max_change)
{
    int64_t d[3], l2 = 0;
    for (int k = 0; k < 3; k++) { d[k] = (int64_t)desired[k] - v[k]; l2 += d[k] * d[k]; }
    const int64_t l = isqrt64(l2);
    if (l <= max_change) { for (int k = 0; k < 3; k++) v[k] = desired[k]; return; }
    for (int k = 0; k < 3; k++) v[k] += (int32_t)(d[k] * max_change / l);
}

/* ── A* ──────────────────────────────────────────────────────────────────── */
static uint32_t s_g[VPYAI_MAX_CELLS];             /* cost so far; UINT32_MAX = not reached */
static int16_t  s_from[VPYAI_MAX_CELLS];
static uint8_t  s_closed[VPYAI_MAX_CELLS];
/* a cell can be in the heap more than once (each time its cost improves); twice
 * the cells has held on every grid tried, and if it ever does not the search
 * says so (-1) rather than drop a cell and return a dearer path */
#define HEAP_MAX (2 * VPYAI_MAX_CELLS)
/* EACH ENTRY CARRIES ITS OWN KEY. Measured: ordering the heap by the cell's
 * CURRENT f broke it — a cell found again more cheaply had its f lowered while
 * an older entry for it sat deeper in the heap, the heap's order no longer held,
 * and on 2 random grids of 196 the path came out 1-2 dearer than Dijkstra's. A
 * snapshot of (f, h) per entry keeps the order true; the stale entry is skipped
 * when it surfaces, the cell being closed by then. */
typedef struct { uint32_t f; int32_t h; int16_t c; } entry_t;
static entry_t  s_heap[HEAP_MAX];
static int      s_heap_full;
static int      s_nheap;
static int32_t  s_cost;

static int less(const entry_t *a, const entry_t *b)
{
    if (a->f != b->f) return a->f < b->f;
    if (a->h != b->h) return a->h < b->h;        /* nearer the goal first */
    return a->c < b->c;                          /* and then always the same way */
}
static void heap_push(int c, uint32_t f, int32_t h)
{
    if (s_nheap >= HEAP_MAX) { s_heap_full = 1; return; }
    int i = s_nheap++;
    s_heap[i].f = f; s_heap[i].h = h; s_heap[i].c = (int16_t)c;
    while (i > 0) {
        const int p = (i - 1) / 2;
        if (!less(&s_heap[i], &s_heap[p])) break;
        const entry_t t = s_heap[i]; s_heap[i] = s_heap[p]; s_heap[p] = t; i = p;
    }
}
static int heap_pop(void)
{
    const int top = s_heap[0].c;
    s_heap[0] = s_heap[--s_nheap];
    int i = 0;
    for (;;) {
        const int l = 2 * i + 1, r = l + 1;
        int m = i;
        if (l < s_nheap && less(&s_heap[l], &s_heap[m])) m = l;
        if (r < s_nheap && less(&s_heap[r], &s_heap[m])) m = r;
        if (m == i) break;
        const entry_t t = s_heap[i]; s_heap[i] = s_heap[m]; s_heap[m] = t; i = m;
    }
    return top;
}

/* the cheapest a path could still cost from (x,y): octile (or Manhattan) distance */
static int32_t heuristic(int x, int y, int gx, int gy, int diagonal)
{
    const int dx = x > gx ? x - gx : gx - x, dy = y > gy ? y - gy : gy - y;
    if (!diagonal) return 10 * (dx + dy);
    const int lo = dx < dy ? dx : dy, hi = dx < dy ? dy : dx;
    return 14 * lo + 10 * (hi - lo);
}

int32_t vpyai_path_cost(void) { return s_cost; }

int vpyai_path(const uint8_t *grid, int w, int h, int sx, int sy, int gx, int gy,
               int diagonal, int16_t *out_xy, int max_cells)
{
    s_cost = 0;
    if (w <= 0 || h <= 0 || w * h > VPYAI_MAX_CELLS) return -1;
    if (sx < 0 || sy < 0 || sx >= w || sy >= h || gx < 0 || gy < 0 || gx >= w || gy >= h) return -1;
    if (grid[sy * w + sx] == VPYAI_WALL || grid[gy * w + gx] == VPYAI_WALL) return -1;
    const int n = w * h;
    for (int i = 0; i < n; i++) { s_g[i] = 0xFFFFFFFFu; s_closed[i] = 0; s_from[i] = -1; }
    s_nheap = 0; s_heap_full = 0;
    const int start = sy * w + sx, goal = gy * w + gx;
    s_g[start] = 0;
    { const int32_t h0 = heuristic(sx, sy, gx, gy, diagonal); heap_push(start, (uint32_t)h0, h0); }
    static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 }, DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    int found = 0;
    while (s_nheap) {
        const int c = heap_pop();
        if (s_closed[c]) continue;                       /* an older, dearer entry */
        s_closed[c] = 1;
        if (c == goal) { found = 1; break; }
        const int cx = c % w, cy = c / w;
        for (int d = 0; d < (diagonal ? 8 : 4); d++) {
            const int nx = cx + DX[d], ny = cy + DY[d];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const int nc = ny * w + nx;
            if (grid[nc] == VPYAI_WALL || s_closed[nc]) continue;
            /* a diagonal never squeezes past the corner of a wall */
            if (d >= 4 && (grid[cy * w + nx] == VPYAI_WALL || grid[ny * w + cx] == VPYAI_WALL)) continue;
            const uint32_t g = s_g[c] + (d >= 4 ? 14u : 10u) + grid[nc];
            if (g >= s_g[nc]) continue;
            s_g[nc] = g; s_from[nc] = (int16_t)c;
            const int32_t hn = heuristic(nx, ny, gx, gy, diagonal);
            heap_push(nc, g + (uint32_t)hn, hn);
        }
    }
    if (s_heap_full) return -1;                          /* could not be sure: say so */
    if (!found) return 0;
    /* walk back from the goal, then write start-first */
    int len = 0;
    for (int c = goal; c >= 0; c = s_from[c]) len++;
    if (len > max_cells) return -1;
    int i = len - 1;
    for (int c = goal; c >= 0; c = s_from[c], i--) { out_xy[2 * i] = (int16_t)(c % w); out_xy[2 * i + 1] = (int16_t)(c / w); }
    s_cost = (int32_t)s_g[goal];
    return len;
}
