/* uvm2_dump_test — uvm2_dump_list() on the host: what it writes, and what it refuses.
 *
 *   tools/build_host_tools.sh && /tmp/uvm2-tools/uvm2_dump_test /tmp/list.bin
 *   python3 tools/list_from_sd.py /tmp/list.bin /tmp/list.json
 *   python3 tools/beam_sim.py /tmp/list.json
 *
 * The real uvm2_draw.c builds a frame of a few strokes; uvm2_sd_write2 is replaced by one that
 * writes a host file. It checks that the file holds EXACTLY the list the executor was handed,
 * and that a dump with no closed frame, or with a frame open over it, is refused and counted
 * rather than written. Exit status is the number of failures.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "uvm2_bus.h"
#include "uvm2_draw.h"
#include "uvm2_sd.h"

uvm2_stats_t uvm2_stats;
volatile uint8_t uvm2_cached_buttons = 0xFFu;          /* nothing pressed (active low) */
int uvm2_sd_error = UVM2_SD_OK;

static uint8_t  g_exec[65536 * 3];
static uint32_t g_exec_n;
uint32_t uvm2_exec(const uint8_t *c, uint32_t n)
{
    memcpy(g_exec, c, n * 3u); g_exec_n = n;
    return n;
}

static const char *g_out;
static uint32_t g_writes;
int uvm2_sd_write2(const char *path, const unsigned char *a, uint32_t na,
                   const unsigned char *b, uint32_t nb)
{
    (void)path;
    FILE *f = fopen(g_out, "wb");
    if (!f) { uvm2_sd_error = UVM2_SD_IO_ERROR; return 0; }
    fwrite(a, 1, na, f); fwrite(b, 1, nb, f); fclose(f);
    g_writes++;
    return 1;
}

static int fails;
#define CHECK(c, ...) do { const int ok_ = (c); printf(ok_ ? "  ok   " : "  FAIL "); if (!ok_) fails++; printf(__VA_ARGS__); printf("\n"); } while (0)

static void strokes(void)
{
    uvm2_draw_intensity(100);
    uvm2_draw_move_abs(-60, -40);
    uvm2_draw_delta(120, 0);
    uvm2_draw_delta(0, 80);
    uvm2_draw_delta(-120, 0);
    uvm2_draw_delta(0, -80);
    uvm2_draw_move_abs(10, 10);
    uvm2_draw_delta(30, 30);
}

int main(int argc, char **argv)
{
    g_out = argc > 1 ? argv[1] : "/tmp/list.bin";
    uvm2_draw_init();

    CHECK(!uvm2_dump_list("list.bin") && uvm2_dump_diag.error == UVM2_DUMP_EMPTY,
          "before any frame: refused as EMPTY (error %d)", (int)uvm2_dump_diag.error);

    uvm2_frame_begin(); strokes(); uvm2_frame_end();
    const int ok = uvm2_dump_list("list.bin");
    CHECK(ok && uvm2_dump_diag.error == UVM2_DUMP_OK && g_writes == 1,
          "after a frame: written (%u commands)", uvm2_dump_diag.commands);

    FILE *f = fopen(g_out, "rb");
    static uint8_t file[32 + 65536 * 3];
    const size_t len = f ? fread(file, 1, sizeof file, f) : 0;
    if (f) fclose(f);
    uint32_t n = (uint32_t)file[8] | file[9] << 8 | file[10] << 16 | (uint32_t)file[11] << 24;
    CHECK(len == 32u + g_exec_n * 3u && n == g_exec_n && !memcmp(file + 32, g_exec, g_exec_n * 3u),
          "the file is the list the executor got: %u commands, %zu bytes", g_exec_n, len);
    CHECK(!memcmp(file, "UVML", 4), "magic UVML");

    uvm2_frame_begin(); uvm2_draw_intensity(50);
    CHECK(!uvm2_dump_list("list.bin") && uvm2_dump_diag.error == UVM2_DUMP_EMPTY && g_writes == 1,
          "with a frame open over the single buffer: refused, nothing written");
    uvm2_frame_end();

    CHECK(uvm2_dump_diag.ok == 1 && uvm2_dump_diag.failed == 2,
          "counted: %u written, %u refused", uvm2_dump_diag.ok, uvm2_dump_diag.failed);

    /* the button trigger: once per press, never while held */
    int fired = 0;
    uvm2_cached_buttons = 0xFFu;               fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    uvm2_cached_buttons = (uint8_t)~0x01u;     fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    uvm2_cached_buttons = (uint8_t)~0x03u;     fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    uvm2_cached_buttons = (uint8_t)~0x03u;     fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    uvm2_cached_buttons = 0xFFu;               fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    uvm2_cached_buttons = (uint8_t)~0x07u;     fired += uvm2_dump_list_on_buttons("l", 0x03) == 1;
    CHECK(fired == 2, "buttons 1+2: dumped on each press, not while held (%d of 2)", fired);

    /* leave the first frame's dump in the file for list_from_sd.py */
    uvm2_frame_begin(); strokes(); uvm2_frame_end();
    uvm2_dump_list("list.bin");

    printf("%s (%d failed) -> %s\n", fails ? "FAILED" : "ALL OK", fails, g_out);
    return fails;
}
