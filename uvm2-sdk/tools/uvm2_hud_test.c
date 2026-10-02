/* uvm2_hud_test — the diagnostics HUD on the host: the combo, what it adds, what it says.
 *
 *   tools/build_host_tools.sh && /tmp/uvm2-tools/uvm2_hud_test /tmp/hud.json
 *   python3 tools/beam_sim.py /tmp/hud.json
 *
 * The real uvm2_draw.c, uvm2_hud.c and uvm2_text.c build frames; the executor is replaced by a
 * copy of the list. Exit status is the number of failures.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "uvm2_bus.h"
#include "uvm2_draw.h"

uvm2_stats_t uvm2_stats;
volatile uint8_t uvm2_cached_buttons = 0xFFu;          /* nothing pressed (active low) */

static uint8_t  g_exec[65536 * 3];
static uint32_t g_exec_n;
uint32_t uvm2_exec(const uint8_t *c, uint32_t n) { memcpy(g_exec, c, n * 3u); g_exec_n = n; return n; }

static uint32_t list_cycles(void)
{
    uint32_t t = 0;
    for (uint32_t i = 0; i < g_exec_n; i++) t += 1u + (((uint32_t)g_exec[3*i+2] << 4) | (g_exec[3*i+1] >> 4));
    return t;
}

static int fails;
#define CHECK(c, ...) do { const int ok_ = (c); printf(ok_ ? "  ok   " : "  FAIL "); if (!ok_) fails++; printf(__VA_ARGS__); printf("\n"); } while (0)

static void frame(void)
{
    uvm2_frame_begin();
    uvm2_draw_intensity(100);
    uvm2_draw_move_abs(-60, -40);
    uvm2_draw_delta(120, 0); uvm2_draw_delta(0, 80); uvm2_draw_delta(-120, 0); uvm2_draw_delta(0, -80);
    uvm2_frame_end();
}

int main(int argc, char **argv)
{
    const char *out = argc > 1 ? argv[1] : "/tmp/hud.json";
    uvm2_draw_init();
    uvm2_pacer_cycles = 30000u;                 /* the 50 Hz pace: the filler must still pad */

    /* 1. off: nothing added */
    frame(); frame();
    const uint32_t n_off = g_exec_n, cyc_off = list_cycles();
    CHECK(uvm2_hud == 0 && uvm2_hud_stats.drawn == 0, "off by default: the frame is %u commands, %u cycles", n_off, cyc_off);

    /* 2. the combo: 1+4 held 100 frames toggles once; held longer, nothing more */
    uvm2_cached_buttons = (uint8_t)~0x09u;
    int on_at = -1;
    for (int i = 1; i <= 150; i++) { frame(); if (uvm2_hud && on_at < 0) on_at = i; }
    CHECK(on_at == 100 && uvm2_hud == 1 && uvm2_hud_stats.toggles == 1, "buttons 1+4 held: on at frame %d (100), once in 150", on_at);
    uvm2_cached_buttons = 0xFFu; frame();
    uvm2_cached_buttons = (uint8_t)~0x0Fu;      /* all four: the debug cart's exit */
    for (int i = 0; i < 150; i++) frame();
    uvm2_cached_buttons = (uint8_t)~0x0Bu;      /* 1+2+4 */
    for (int i = 0; i < 150; i++) frame();
    CHECK(uvm2_hud == 1 && uvm2_hud_stats.toggles == 1, "all four, or 1+2+4, held 3 s: no toggle");
    uvm2_cached_buttons = 0xFFu;

    /* 3. on: the text goes at the end, and the frame still pads to the pace */
    frame(); frame();
    const uint32_t n_on = g_exec_n, cyc_on = list_cycles();
    CHECK(uvm2_hud_stats.cmds > 100 && uvm2_hud_stats.cmds < 1600,
          "on: the HUD's own share %u commands, %u bus cycles (under its 1600-command room)", uvm2_hud_stats.cmds, uvm2_hud_stats.cycles);
    CHECK(cyc_off == 30000 && cyc_on == 30000, "both frames still pad to the pace: %u and %u cycles", cyc_off, cyc_on);
    (void)n_on;

    /* 4. what it says: the game's figures, read before the HUD draws */
    uvm2_frame_begin();
    uvm2_draw_intensity(100);
    uvm2_draw_move_abs(-60, -40);
    uvm2_draw_delta(120, 0); uvm2_draw_delta(0, 80); uvm2_draw_delta(-120, 0); uvm2_draw_delta(0, -80);
    const uint32_t game_cmds = uvm2_list_commands(), game_cyc = uvm2_list_cycles(), game_vec = uvm2_stats.vectors;
    uvm2_frame_end();
    char want[64];
    snprintf(want, sizeof want, " C%u D0 Z0", game_cyc);
    CHECK(strstr(uvm2_hud_text[0], want) && uvm2_hud_text[0][0] == 'F', "line 1 '%s' — the game's cycles, D and Z", uvm2_hud_text[0]);
    snprintf(want, sizeof want, " N%u V%u", game_cmds, game_vec);
    CHECK(strstr(uvm2_hud_text[1], want) != 0, "line 2 '%s' — the game's commands and vectors, not the HUD's", uvm2_hud_text[1]);
    uvm2_stats.stack0_peak = 2624; uvm2_stats.stack1_peak = 812; uvm2_stats.stack_overflow = 1; uvm2_stats.dropped = 3;
    frame();
    CHECK(!strncmp(uvm2_hud_text[1], "S2624 812! N", 12), "an overflow shows '!': '%s'", uvm2_hud_text[1]);
    CHECK(strstr(uvm2_hud_text[0], " D3 Z0 !!") != 0, "dropped not zero shows '!!': '%s'", uvm2_hud_text[0]);
    uvm2_stats.stack_overflow = 0;
    /* 5. its own frame for beam_sim: no ramp with the clamp on */
    FILE *f = fopen(out, "w");
    fprintf(f, "[");
    for (uint32_t i = 0; i < g_exec_n; i++) fprintf(f, "%s[%u,%u,%u]", i ? "," : "", g_exec[3*i], g_exec[3*i+1], g_exec[3*i+2]);
    fprintf(f, "]\n"); fclose(f);

    /* 6. off again with the combo */
    uvm2_cached_buttons = (uint8_t)~0x09u;
    for (int i = 0; i < 100; i++) frame();
    uvm2_cached_buttons = 0xFFu; frame();
    CHECK(uvm2_hud == 0 && g_exec_n == n_off, "held again: off, and the frame is back to %u commands", g_exec_n);

    printf("%s (%d failed) -> %s\n", fails ? "FAILED" : "ALL OK", fails, out);
    return fails;
}
