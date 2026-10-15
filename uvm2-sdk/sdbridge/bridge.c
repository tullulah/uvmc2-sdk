/* bridge.c — the UVMC2 SD bridge: a RAM-only program that puts files on the cartridge's SD
 * card over USB, with nothing on the card and no game running, and shows on the Vectrex what
 * it is doing.
 *
 * HOW IT GETS THERE. The multicart's own firmware (Ralf's, in the flash) carries the pico-sdk's
 * USB "Reset" interface next to its menu's serial port, so the host sends the chip to the
 * bootrom's BOOTSEL mode from the menu (picotool reboot -u -f), loads this into SRAM and runs
 * it (picotool load -x). The flash is never touched.
 *
 * IT IS BUILT AS A UVMC2 GAME (build.sh: the SDK's pico/ recipe, dual core): the SDK halts the
 * 6809 and core 1 replays the picture, so it can draw. The picture changes only now and then
 * (UVM2_REPLAY_LAST redraws the last one meanwhile), and core 0 spends its time on USB.
 *
 * GOING BACK. Resetting only the chip would leave the 6809 lost (the cartridge cannot reset
 * the console), so `b` does not reset: it shows DONE and waits for the console's RESET button,
 * which the SDK sees through the VIA (its reset watch, counting only: UVM2_RESET_WATCH_ONLY); then it reboots into
 * the multicart's firmware with the button still down, and the console comes up in its menu.
 * Nobody presses it within a minute: it reboots anyway (and RESET then brings the menu).
 *
 * It speaks the debug monitor's protocol (../monitor/monitor.c) on the monitor's USB ids, so
 * the IDE uploads through either the same way:
 *   v                 -> "UVMC2-BRIDGE 2"
 *   u <size> <path>   -> "OK", then <size> raw bytes, then "DONE <size> <crc32>" (or "ERR ...")
 *   x                 -> "OK", then the chip goes to BOOTSEL: the host loads the game it just
 *                        put on the card into SRAM and runs it (picotool load -x)
 *   b                 -> "OK"; the screen says DONE, RESET on the console goes back to the menu
 * A command line ending in CR LF: the LF after it is not taken for the file's first byte. */
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "boot/picoboot_constants.h"
#include "tusb.h"
#include "uvm2_sd.h"
#include "uvm2_draw.h"
#include "uvm2_text.h"
#include "uvm2_bus.h"
#include "vs_logo.h"

static char s_line[96];
static unsigned s_len;
static int s_line_cr;
static uint8_t s_chunk[8192];

/* ── THE SCREEN ─────────────────────────────────────────────────────────────────────── */

static char s_title[32] = "WAITING FOR THE PC";
static char s_detail[32];
static uint32_t s_done, s_total;    /* the bar, when s_total != 0 */
static uint32_t s_shown_at;

static void text_upper(char *d, const char *s, unsigned n)
{
    unsigned k = 0;
    while (s[k] && k < n - 1) { char c = s[k]; d[k++] = (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }
    d[k] = 0;
}

static void line(int x0, int y0, int dx, int dy)
{
    uvm2_draw_move_abs(x0, y0);
    while (dx || dy) {                   /* a delta is at most 127 a side */
        int sx = dx > 100 ? 100 : dx < -100 ? -100 : dx;
        int sy = dy > 100 ? 100 : dy < -100 ? -100 : dy;
        uvm2_draw_delta(sx, sy);
        dx -= sx; dy -= sy;
    }
}

static void screen(void)
{
    uvm2_frame_begin();
    /* the Vectrex Studio logo turning, as in the RP2350 cartridge's intro: a turn every 2.56 s */
    vs_logo_draw((int)(time_us_32() / 20000u), 0, 72, 8, 100,
                 uvm2_draw_reset, uvm2_draw_intensity, uvm2_draw_move_abs, uvm2_draw_delta);
    uvm2_print_text(-110, 25, s_title, 3, 110);
    if (s_detail[0]) uvm2_print_text(-110, 0, s_detail, 3, 90);
    if (s_total) {
        const int w = 200, x = -100, y = -45, h = 16;
        uvm2_draw_reset();
        uvm2_draw_intensity(90);
        line(x, y, w, 0); line(x + w, y, 0, h); line(x + w, y + h, -w, 0); line(x, y + h, 0, -h);
        int fill = (int)((uint64_t)(w - 4) * s_done / s_total);
        if (fill > 0) for (int k = 3; k < h - 2; k += 3) line(x + 2, y + k, fill, 0);
        char pct[8];
        uint32_t p = (uint32_t)((uint64_t)100u * s_done / s_total);
        int n = 0;
        if (p >= 100) pct[n++] = '1';
        if (p >= 10) pct[n++] = (char)('0' + (p / 10) % 10);
        pct[n++] = (char)('0' + p % 10);
        pct[n++] = '%'; pct[n] = 0;
        uvm2_print_text(-20, -75, pct, 3, 100);
    }
    uvm2_frame_end();
    s_shown_at = time_us_32();
}

/* A new picture every 40 ms, for the logo to turn: frame_begin can wait for the buffer core 1
 * is drawing (a frame at most), and USB is not serviced meanwhile -- bulk transfers wait. */
static void screen_maybe(void)
{
    if (time_us_32() - s_shown_at > 40000u) screen();
}

static void say(const char *title, const char *detail)
{
    text_upper(s_title, title, sizeof s_title);
    text_upper(s_detail, detail ? detail : "", sizeof s_detail);
    screen();
}

/* ── USB ────────────────────────────────────────────────────────────────────────────── */

static void put(const char *s) { tud_cdc_write(s, (uint32_t)strlen(s)); tud_cdc_write_flush(); }

static void put_u(uint32_t v, int hex)
{
    char b[12];
    int n = 0;
    if (hex) { for (int i = 28; i >= 0; i -= 4) b[n++] = "0123456789abcdef"[(v >> i) & 15u]; }
    else { char t[10]; int k = 0; do { t[k++] = (char)('0' + v % 10u); v /= 10u; } while (v); while (k) b[n++] = t[--k]; }
    tud_cdc_write(b, (uint32_t)n);
}

/* "ERR <what> sd=<uvm2_sd_error> fr=<FatFs FRESULT> st=<step reached>": why, for the host. */
static void sd_err(const char *what)
{
    tud_cdc_write_str(what);
    tud_cdc_write_str(" sd="); put_u((uint32_t)uvm2_sd_error, 0);
    tud_cdc_write_str(" fr="); put_u(uvm2_sd_diag.fresult, 0);
    tud_cdc_write_str(" st="); put_u(uvm2_sd_diag.step, 0);
    put("\r\n");
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *d, uint32_t n)
{
    static const uint32_t t[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };
    while (n--) {
        crc ^= *d++;
        crc = (crc >> 4) ^ t[crc & 15u];
        crc = (crc >> 4) ^ t[crc & 15u];
    }
    return crc;
}

/* Waiting on the port keeps USB serviced, and the bar moving. */
static uint32_t read_some(uint8_t *dst, uint32_t want)
{
    for (;;) {
        tud_task();
        if (!tud_mounted()) return 0;
        uint32_t n = tud_cdc_read(dst, want);
        if (n) return n;
        screen_maybe();
    }
}

static void upload(uint32_t size, const char *path)
{
    if (size == 0 || !path[0]) { put("ERR\r\n"); return; }
    if (s_line_cr) {                     /* the LF after the CR, if it is there */
        uint8_t c;
        while (!tud_cdc_available()) tud_task();
        if (tud_cdc_peek(&c) && c == '\n') tud_cdc_read(&c, 1);
    }
    s_done = 0; s_total = size;
    say("UPLOADING", path);
    if (!uvm2_sd_stream_open(path)) { s_total = 0; say("SD CARD ERROR", "NO CARD?"); sd_err("ERR open"); return; }
    put("OK\r\n");
    uint32_t got = 0, crc = 0xFFFFFFFFu;
    int ok = 1;
    while (got < size) {
        uint32_t want = size - got;
        if (want > sizeof s_chunk) want = sizeof s_chunk;
        uint32_t have = 0;
        while (have < want) {
            uint32_t n = read_some(s_chunk + have, want - have);
            if (n == 0) { uvm2_sd_stream_close(); s_total = 0; say("CANCELLED", "USB UNPLUGGED"); return; }
            have += n;
        }
        got += have;
        if (ok) {
            crc = crc32_update(crc, s_chunk, have);
            ok = uvm2_sd_stream_put(s_chunk, have);           /* failed: read on, drop it */
        }
        s_done = got;
        screen_maybe();
    }
    if (ok) ok = uvm2_sd_stream_close();
    if (!ok) { s_total = 0; say("SD CARD ERROR", "WRITE FAILED"); sd_err("ERR write"); return; }
    s_total = 0;
    say("UPLOADED", path);
    tud_cdc_write_str("DONE ");
    put_u(size, 0);
    tud_cdc_write_str(" ");
    put_u(crc ^ 0xFFFFFFFFu, 1);
    put("\r\n");
}

static int s_finished;

static void command(const char *l)
{
    if (l[0] == 'v') { put("UVMC2-BRIDGE 2\r\n"); return; }
    if (l[0] == 'u') {
        const char *p = l + 1;
        uint32_t n = 0;
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') n = n * 10u + (uint32_t)(*p++ - '0');
        while (*p == ' ') p++;
        upload(n, p);
        return;
    }
    if (l[0] == 'x') {                   /* to BOOTSEL: the host runs the game from RAM */
        put("OK\r\n");
        say("STARTING THE GAME", 0);
        uint32_t t0 = time_us_32();
        while (time_us_32() - t0 < 20000u) tud_task();   /* the answer out first */
        rom_reset_usb_boot_extra(-1, 0, false);
    }
    if (l[0] == 'b') {
        put("OK\r\n");
        say("DONE", "PRESS RESET ON THE CONSOLE");
        s_finished = 1;
        return;
    }
    put("ERR\r\n");
}

static void back_to_menu(void)
{
    rom_reboot(BOOT_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);
    for (;;) { }
}

int main(void)
{
    screen();
    tud_init(0);
    uint32_t seen0 = 0, finished_at = 0;
    for (;;) {
        tud_task();
        if (tud_mounted()) screen_maybe();   /* not before: enumeration wants answers at once */
        if (s_finished) {
            if (!finished_at) { finished_at = time_us_32(); seen0 = uvm2_reset_seen; }
            if (uvm2_reset_seen != seen0) back_to_menu();                 /* RESET is down */
            if (time_us_32() - finished_at > 60000000u) back_to_menu();   /* nobody came */
        }
        char c;
        if (!tud_cdc_available() || tud_cdc_read(&c, 1) != 1) continue;
        if (c == '\r' || c == '\n') {
            s_line_cr = c == '\r';
            if (s_len) { s_line[s_len] = 0; command(s_line); s_len = 0; }
        } else if (s_len < sizeof s_line - 1) {
            s_line[s_len++] = c;
        }
    }
}
