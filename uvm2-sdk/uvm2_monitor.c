/* uvm2_monitor.c — a debug build's USB serial port: the monitor (monitor/monitor.c) loaded off
 * the SD card into the PSRAM. Only with UVM2_USB_MONITOR; nothing at all otherwise.
 *
 * The multicart's own firmware drops its USB port when it launches a game, so a game that wants
 * the IDE to reach it brings its own. TinyUSB is ~18 KB and a big .um2 has not got that in SRAM,
 * so the monitor lives in the PSRAM: MONITOR.BIN, built once per SDK, copied to the card's root.
 *
 * At boot (main, after the PSRAM is up) uvm2_monitor_load copies it into the PSRAM through an
 * SRAM buffer, checking every piece, then checks its magic and clears its .bss.
 *
 * IT RUNS ON THE GAME'S CORE, at the end of each frame (uvm2_frame_end), never on the core that
 * replays the lists: a monitor that hangs there blacks the screen out, and nothing can say why.
 * Here a hang stops the game, and the other core goes on drawing.
 *
 * USB comes up in steps, one per frame, each with its check (uvm2_monitor_state, readable over
 * SWD):
 *
 *    1 no MONITOR.BIN (or no PSRAM)   2 bad image   3 loaded
 *   10 clocks read   11 clk_usb running (set up here if it was not)
 *   12 USB block out of reset (with a time limit)   13 tud_init returned   14 running
 *
 * UVM2_USB_MONITOR_DIAG: one step every two seconds, and four lines in the corner of the screen
 * saying which step is done and what USB looks like, so a hang leaves the line of the step
 * before it up for a photo (how the monitor was brought up on the UVMC2, which has no SWD):
 *
 *     MON <step> F<frames>
 *     PLL<locked> CLK<enabled> RST<usb out of reset> I<irqs> M<mounted>
 *     PS<psram: 0 never tried, 1 failed, 2 up> N<bytes loaded> X<failure> W<first word>
 *     USB<clk_usb in kHz, measured> S<USB interrupt sources seen> C<SIE_STATUS now> */
#include <stdint.h>
#include <string.h>
#ifdef UVM2_USB_MONITOR
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/resets.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "uvm2_sd.h"
#include "uvm2_draw.h"
#include "uvm2_bus.h"
#include "uvm2_text.h"

/* THE MONITOR'S PSRAM WINDOW, 256 KB from 0x117C0000 (0x157C0000 uncached), and what else
 * lives in it, all through the uncached alias:
 *   0x157C0000  MONITOR.BIN, its .data and .bss (~40 KB)
 *   0x157E0000  the reset button's copy of .data (uvm2_core1.c, up to 2 KB)
 *   0x157F0000  the RTT ring of DEBUG_PRINT / vpy_log (sdk_rp2350.c, 4 KB)
 *   0x157F1000  the pause sign's commands (built by the monitor, replayed by core 1) */
#define UVM2_MONITOR_ADDR      0x117C0000u   /* monitor/monitor.ld */
#define UVM2_MONITOR_UNCACHED  0x157C0000u
#define UVM2_MONITOR_MAX       (256u * 1024u)
#define UVM2_MONITOR_MAGIC     0x4E4F4D55u   /* "UMON" */
#ifdef UVM2_USB_MONITOR_DIAG
#define STEP_FRAMES            100u          /* two seconds at 50 Hz: long enough for a photo */
#else
#define STEP_FRAMES            1u
#endif

/* What the game lends the monitor: the SD card, a file written in pieces (its uploads). The
 * monitor calls these from its `task`, i.e. here, on the game's core, between frames. */
typedef struct {
    uint32_t version;
    int (*open)(const char *path);
    int (*put)(const unsigned char *data, uint32_t n);
    int (*close)(void);
    void *rtt;      /* version 2: the game's RTT block (sdk_rp2350.c), its text goes out on USB */
    volatile uint32_t *paused;   /* version 3: set while the debugger holds the game */
    /* version 4: the game's drawing, for what the monitor puts on screen from its `task`
     * (inside the game's frame): the upload's bar, and the pause sign it builds once.
     * Function pointers and not code here: a big game has no SRAM left for it (ROTJ). */
    void (*print_text)(int x, int y, const char *str, int scale, int intensity);
    void (*draw_reset)(void);
    void (*draw_intensity)(int brightness);
    void (*draw_move_abs)(int x, int y);
    void (*draw_delta)(int dx, int dy);
    uint32_t (*list_commands)(void);
    uint32_t (*list_cycles)(void);
    const uint8_t *(*list_buffer)(void);
    void (*list_rewind)(uint32_t count, uint32_t cycles);
    volatile uint32_t *pause_n;  /* commands of the sign the monitor left at UVM2_PAUSE_CMDS */
    void (*clear_frame)(void);   /* version 5: the game's drawing out of this frame (the upload screen) */
} uvm2_monitor_host;
extern char _SEGGER_RTT[] __attribute__((weak));   /* absent in a game without the C runtime */
static volatile uint32_t s_paused, s_pause_n;
static const uvm2_monitor_host s_host = {
    5, uvm2_sd_stream_open, uvm2_sd_stream_put, uvm2_sd_stream_close, _SEGGER_RTT, &s_paused,
    uvm2_print_text, uvm2_draw_reset, uvm2_draw_intensity, uvm2_draw_move_abs, uvm2_draw_delta,
    uvm2_list_commands, uvm2_list_cycles, uvm2_list_buffer, uvm2_list_rewind, &s_pause_n,
    uvm2_list_clear_frame,
};

/* THE PAUSE SIGN, as on the Vectrex Studio cartridge (its firmware, bios.rs): two bars in the
 * top right corner over the last picture while the debugger holds the game. Core 1 redraws
 * that picture on its own (UVM2_REPLAY_LAST) and then calls uvm2_replay_extra, which adds the
 * sign. The monitor builds it (monitor.c), on top of the first frame it sees, and takes it
 * back out of that frame (list_rewind); the commands are in the PSRAM, in the monitor's window
 * after the RTT ring (sdk_rp2350.c), through the uncached alias. Core 1 copies them to its
 * stack before replaying them: a list replayed straight out of the PSRAM is what the stream
 * does not take (uvm2_draw.c). */
#define PAUSE_MAX 160
#define UVM2_PAUSE_CMDS ((const volatile uint8_t *)0x157F1000u)

/* Core 1, after each redraw of the last picture (uvm2_core1.c). */
void uvm2_replay_extra(void)
{
    const uint32_t n = s_pause_n;
    if (!s_paused || !n || n > PAUSE_MAX) return;
    uint8_t cmds[PAUSE_MAX * 3];
    for (uint32_t i = 0; i < n * 3u; i++) cmds[i] = UVM2_PAUSE_CMDS[i];
    uvm2_exec(cmds, n);
}

typedef struct {
    uint32_t magic, version;
    uint32_t *bss_start, *bss_end;
    void (*init)(const uvm2_monitor_host *host);   /* before version 4 it took nothing */
    void (*task)(void);
    uint32_t (*status)(void);   /* version 2: irqs << 8 | connected << 1 | mounted */
    uint32_t (*seen)(void);     /* version 3: the USB interrupt sources seen (INTS bits) */
} uvm2_monitor_api;

extern int uvm2_psram_ready;
extern void uvm2_print_text_chained(int x, int y, const char *str, int scale, int intensity);
static const uvm2_monitor_api *s_mon;
static uint32_t s_version, s_frames, s_next;
static int s_pll, s_clk, s_rst;
static uint32_t s_usb_khz;
volatile int uvm2_monitor_state;   /* the step above */

/* What the load saw, for the line on screen: bytes read, where it failed, the first word. */
static uint32_t s_bytes, s_fail, s_word;

/* The file in 512-byte pieces through SRAM, each copied into the PSRAM by the CPU and read
 * back: the card's driver never writes the PSRAM itself, and a piece that does not read back
 * as written says so (s_fail = 0x10000 + offset). The piece is on the stack: this runs once,
 * at boot, and a static buffer cost a big game the last 1 KB it had (ROTJ's .bss overflowed). */
static int load_image(void)
{
    uint32_t words[128];
    uint8_t *buf = (uint8_t *)words;
    const uint32_t piece = sizeof words;
    volatile uint32_t *dst = (volatile uint32_t *)UVM2_MONITOR_UNCACHED;
    for (uint32_t off = 0; off < UVM2_MONITOR_MAX; off += piece) {
        uint32_t n = uvm2_sd_read_from("MONITOR.BIN", buf, piece, off);
        if (n == 0) break;
        for (uint32_t i = n; i < piece; i++) buf[i] = 0;
        const uint32_t *w = words;
        for (uint32_t i = 0; i < (n + 3u) / 4u; i++) dst[(off >> 2) + i] = w[i];
        __asm volatile ("dsb" ::: "memory");
        for (uint32_t i = 0; i < (n + 3u) / 4u; i++)
            if (dst[(off >> 2) + i] != w[i]) { s_fail = 0x10000u + off + i * 4u; return 0; }
        s_bytes = off + n;
        if (n < piece) break;
    }
    return s_bytes >= 6u * 4u;
}

void uvm2_monitor_load(void)
{
    uvm2_monitor_state = 1;
    if (uvm2_psram_ready != 1) return;
    if (!load_image()) return;
    const volatile uvm2_monitor_api *u = (const volatile uvm2_monitor_api *)UVM2_MONITOR_UNCACHED;
    s_word = u->magic;
    uvm2_monitor_state = 2;
    if (u->magic != UVM2_MONITOR_MAGIC) return;
    /* its .bss, zeroed through the same alias */
    uint32_t lo = (uint32_t)u->bss_start, hi = (uint32_t)u->bss_end;
    if (lo < UVM2_MONITOR_ADDR || hi > UVM2_MONITOR_ADDR + UVM2_MONITOR_MAX || hi < lo) { s_fail = 2; return; }
    for (uint32_t a = lo; a < hi; a += 4)
        *(volatile uint32_t *)(a - UVM2_MONITOR_ADDR + UVM2_MONITOR_UNCACHED) = 0;
    s_version = u->version;
    __asm volatile ("dsb; isb" ::: "memory");
    s_mon = (const uvm2_monitor_api *)UVM2_MONITOR_ADDR;   /* runs through the cached window */
    uvm2_monitor_state = 3;
}

static void read_clocks(void)
{
    s_pll = (pll_usb_hw->cs & PLL_CS_LOCK_BITS) != 0;
    s_clk = (clocks_hw->clk[clk_usb].ctrl & CLOCKS_CLK_USB_CTRL_ENABLE_BITS) != 0;
    s_rst = (resets_hw->reset_done & RESETS_RESET_USBCTRL_BITS) != 0;
#ifdef UVM2_USB_MONITOR_DIAG
    s_usb_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
#endif
}

/* One step of the bring-up; returns the step reached. */
static int step(int st)
{
    switch (st) {
    case 3:
        read_clocks();
        return 10;
    case 10:
        /* clk_usb at 48 MHz from PLL_USB. Left alone if it already runs: whoever set it up
         * (the multicart's firmware, for its own USB port) may be feeding other clocks from it.
         * And it always runs here -- the multicart's firmware and the bootrom's BOOTSEL both
         * leave it at 48 MHz (measured) -- so only the diagnostic build carries the set-up:
         * pll_init and clock_configure are ~400 bytes a big game has not got (ROTJ). Without
         * the clock the monitor stays here, off, and the game runs as ever. */
#ifdef UVM2_USB_MONITOR_DIAG
        if (!s_pll) { pll_init(pll_usb, 1, 1440000000u, 6, 5); }
        if (!(clocks_hw->clk[clk_usb].ctrl & CLOCKS_CLK_USB_CTRL_ENABLE_BITS))
            clock_configure_undivided(clk_usb, 0, CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48000000u);
        read_clocks();
#endif
        return s_pll && s_clk ? 11 : 10;
    case 11: {
        /* the controller out of reset, waiting at most 10 ms: without its clock it never
         * comes out, and the pico-sdk's own wait (in tud_init) would hang for ever */
        reset_block(RESETS_RESET_USBCTRL_BITS);
        unreset_block(RESETS_RESET_USBCTRL_BITS);
        uint32_t t0 = time_us_32();
        while (!(resets_hw->reset_done & RESETS_RESET_USBCTRL_BITS) && time_us_32() - t0 < 10000u) { }
        read_clocks();
        return s_rst ? 12 : 11;   /* not out of reset: stay, it says RST0 */
    }
    case 12:
        s_mon->init(&s_host);
        return 13;
    default:
        return 14;
    }
}

#ifdef UVM2_USB_MONITOR_DIAG
static char *put_dec(char *p, uint32_t v)
{
    char b[10];
    int n = 0;
    do { b[n++] = (char)('0' + v % 10u); v /= 10u; } while (v && n < 10);
    while (n) *p++ = b[--n];
    return p;
}

static char *put_hex(char *p, uint32_t v)
{
    for (int i = 28; i >= 0; i -= 4) *p++ = "0123456789ABCDEF"[(v >> i) & 15u];
    return p;
}

static char *put_str(char *p, const char *s) { while (*s) *p++ = *s++; return p; }

static void show(void)
{
    char l[48], *p = l;
    p = put_str(p, "MON ");
    p = put_dec(p, (uint32_t)uvm2_monitor_state);
    p = put_str(p, " F");
    p = put_dec(p, s_frames);
    *p = 0;
    uvm2_print_text_chained(-110, -80, l, 3, 100);

    uint32_t st = (uvm2_monitor_state == 14 && s_version >= 2) ? s_mon->status() : 0;
    p = l;
    p = put_str(p, "PLL"); p = put_dec(p, (uint32_t)s_pll);
    p = put_str(p, " CLK"); p = put_dec(p, (uint32_t)s_clk);
    p = put_str(p, " RST"); p = put_dec(p, (uint32_t)s_rst);
    p = put_str(p, " I"); p = put_dec(p, st >> 8);
    p = put_str(p, " M"); p = put_dec(p, st & 3u);
    *p = 0;
    uvm2_print_text_chained(-110, -100, l, 3, 100);

    p = l;
    p = put_str(p, "PS"); p = put_dec(p, (uint32_t)(uvm2_psram_ready + 1));
    p = put_str(p, " N"); p = put_dec(p, s_bytes);
    p = put_str(p, " X"); p = put_hex(p, s_fail);
    p = put_str(p, " W"); p = put_hex(p, s_word);
    *p = 0;
    uvm2_print_text_chained(-120, -122, l, 2, 100);

    uint32_t seen = (uvm2_monitor_state == 14 && s_version >= 3) ? s_mon->seen() : 0;
    p = l;
    p = put_str(p, "USB"); p = put_dec(p, s_usb_khz);
    p = put_str(p, " S"); p = put_hex(p, seen);
    p = put_str(p, " C"); p = put_hex(p, *(volatile uint32_t *)0x50110050u);   /* SIE_STATUS */
    *p = 0;
    uvm2_print_text_chained(-120, -136, l, 2, 100);
}
#endif

/* At the end of every frame of the game, before it is published (uvm2_frame_end). */
void uvm2_monitor_frame(void)
{
    s_frames++;
#ifdef UVM2_USB_MONITOR_DIAG
    show();
#endif
    if (!s_mon) return;
    int st = uvm2_monitor_state;
    if (st == 14) { s_mon->task(); return; }
    /* With the lines on, the one just drawn says the step before: a hang inside the next
     * leaves it on screen. */
    if (s_frames < s_next) return;
    s_next = s_frames + STEP_FRAMES;
    uvm2_monitor_state = step(st);
}
#else
void uvm2_monitor_load(void) { }
#endif
