/* monitor.c — the UVMC2 debug monitor (MONITOR.BIN): a USB serial port in the cartridge's
 * PSRAM, loaded off the SD card by a game built with UVM2_USB_MONITOR (uvm2_monitor.c).
 *
 * WHY IN THE PSRAM: a .um2 is loaded into the 520 KB of SRAM, and TinyUSB with a CDC port is
 * ~18 KB (measured): a big game has not got that. The PSRAM (8 MB, brought up by the SDK) has.
 *
 * It has no start-up code of its own: the game calls `init` once and then `task` once per
 * frame, from the game's core (uvm2_frame_end). The image starts with that table. TinyUSB
 * itself runs from the USB interrupt (poll), which `task` only backs up.
 *
 * BUILT WITH HARDWARE SPIN LOCKS (CMakeLists.txt): the RP2350 default, software ones, are
 * exclusive loads/stores, and an exclusive store to the PSRAM never succeeds.
 *
 * PROTOCOL: text lines, answers end in CR LF.
 *   v                    -> "UVMC2-MON 2"
 *   r <hex addr> <n>     -> n bytes (n <= 256) as hex, one line
 *   w <hex addr> <hex>   -> "OK": the bytes written (SRAM only)
 *   u <size> <path>      -> "OK", then exactly <size> raw bytes, then "DONE <size> <crc32>";
 *                           "ERR ..." instead if the file cannot be opened or written. The file
 *                           goes onto the SD card as it arrives (the game writes it, see task).
 *   b                    -> "OK", then the chip resets (back to the multicart's menu)
 */
#include <stdint.h>
#include <string.h>
#include "tusb.h"
#include "hardware/timer.h"

#define MON_MAGIC 0x4E4F4D55u   /* "UMON" */

static char s_line[96];
static unsigned s_len;

static void put(const char *s) { tud_cdc_write(s, (uint32_t)strlen(s)); }

static void put_hex(uint32_t v, int digits)
{
    char b[9];
    for (int i = digits - 1; i >= 0; i--) { b[i] = "0123456789abcdef"[v & 15]; v >>= 4; }
    tud_cdc_write(b, (uint32_t)digits);
}

static void put_dec_u(uint32_t v)
{
    char b[10];
    int n = 0;
    do { b[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    char o[10];
    for (int i = 0; i < n; i++) o[i] = b[n - 1 - i];
    tud_cdc_write(o, (uint32_t)n);
}

static uint32_t parse_hex(const char **p)
{
    uint32_t v = 0;
    while (**p == ' ') (*p)++;
    if ((*p)[0] == '0' && ((*p)[1] == 'x' || (*p)[1] == 'X')) *p += 2;
    for (;;) {
        char c = **p;
        int d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
        if (d < 0) break;
        v = v * 16u + (uint32_t)d; (*p)++;
    }
    return v;
}

static uint32_t parse_dec(const char **p)
{
    uint32_t v = 0;
    while (**p == ' ') (*p)++;
    while (**p >= '0' && **p <= '9') { v = v * 10u + (uint32_t)(**p - '0'); (*p)++; }
    return v;
}

/* Readable without a fault: SRAM, scratch, the PSRAM window, the flash window. */
static int readable(uint32_t a, uint32_t n)
{
    uint32_t e = a + n;
    return (a >= 0x20000000u && e <= 0x20082000u) || (a >= 0x10000000u && e <= 0x11800000u) || (a >= 0x15000000u && e <= 0x15800000u);
}

/* What the game lends the monitor (uvm2_monitor.c): the SD card, a file written in pieces. */
typedef struct {
    uint32_t version;
    int (*open)(const char *path);
    int (*put)(const unsigned char *data, uint32_t n);
    int (*close)(void);
} uvm2_monitor_host;
static const uvm2_monitor_host *s_host;

/* AN UPLOAD: `u` names the file; the game's core opens it (UP_OPEN), then the interrupt fills
 * s_chunk from the port and the game's core writes each full chunk to the card. While a chunk
 * waits to be written the port is not read, so TinyUSB holds the host off (NAK) on its own. */
enum { UP_NONE, UP_OPEN, UP_DATA, UP_SKIP };
static volatile int s_up;
static char s_up_path[64];
static uint32_t s_up_total, s_up_crc;
static volatile uint32_t s_up_got, s_chunk_n;
static volatile int s_chunk_full;
/* The `u` line ended in CR: its LF is still in the port, and must not become the file's first
 * byte (it did, the first upload came out one byte late). */
static int s_eat_lf, s_line_cr;
static uint8_t s_chunk[4096];

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

static void command(const char *l)
{
    if (l[0] == 'v') { put("UVMC2-MON 2\r\n"); return; }
    if (l[0] == 'r') {
        const char *p = l + 1;
        uint32_t a = parse_hex(&p), n = parse_dec(&p);
        if (n == 0 || n > 256 || !readable(a, n)) { put("ERR\r\n"); return; }
        for (uint32_t i = 0; i < n; i++) put_hex(*(volatile uint8_t *)(a + i), 2);
        put("\r\n");
        return;
    }
    if (l[0] == 'w') {
        const char *p = l + 1;
        uint32_t a = parse_hex(&p), n = 0;
        while (*p == ' ') p++;
        const char *h = p;
        while (h[0] && h[1]) { h += 2; n++; }
        if (n == 0 || a < 0x20000000u || a + n > 0x20082000u) { put("ERR\r\n"); return; }
        for (uint32_t i = 0; i < n; i++) {
            char b[3] = { p[0], p[1], 0 };
            const char *q = b;
            *(volatile uint8_t *)(a + i) = (uint8_t)parse_hex(&q);
            p += 2;
        }
        put("OK\r\n");
        return;
    }
    if (l[0] == 'u') {
        const char *p = l + 1;
        uint32_t n = parse_dec(&p);
        while (*p == ' ') p++;
        size_t k = strlen(p);
        if (!s_host || s_up != UP_NONE || n == 0 || k == 0 || k >= sizeof s_up_path) { put("ERR\r\n"); return; }
        memcpy(s_up_path, p, k + 1);
        s_up_total = n;
        s_up_got = 0;
        s_up_crc = 0xFFFFFFFFu;
        s_chunk_n = 0;
        s_chunk_full = 0;
        s_eat_lf = s_line_cr;
        s_up = UP_OPEN;   /* the game's core opens it and answers */
        return;
    }
    if (l[0] == 'b') {
        put("OK\r\n");
        tud_cdc_write_flush();
        busy_wait_us_32(20000);   /* the answer out of the endpoint first */
        *(volatile uint32_t *)0xE000ED0Cu = 0x05FA0004u;   /* AIRCR SYSRESETREQ */
        for (;;) { }
    }
    put("ERR\r\n");
}

/* For the game's diagnostic line: irqs << 8 | connected << 1 | mounted, and every USB
 * interrupt source seen so far (USB INTS bits, OR-ed). */
static volatile uint32_t s_irqs, s_seen;
static volatile int s_ready;
static uint32_t mon_seen(void) { return s_seen; }
static uint32_t mon_status(void)
{
    return (s_irqs << 8) | ((uint32_t)tud_cdc_connected() << 1) | (uint32_t)tud_mounted();
}

/* TinyUSB's work and the commands. Run from the USB interrupt itself (counted_irq, below):
 * the game's core reaches the monitor only once a frame, 20 ms or more apart, and that was too
 * slow for the host -- it saw the reset and the SETUPs, got some answers, gave up and
 * suspended the port (measured on the UVMC2: INTS seen 0x15010, never mounted). */
static void poll(void)
{
    tud_task();
    for (;;) {
        int up = s_up;
        if (up == UP_OPEN) break;                     /* the game's core has not opened it yet */
        if (up == UP_DATA || up == UP_SKIP) {
            if (s_chunk_full) break;                  /* the last chunk is not written yet */
            if (s_eat_lf) {
                uint8_t c;
                if (!tud_cdc_peek(&c)) break;
                if (c == '\n') tud_cdc_read(&c, 1);
                s_eat_lf = 0;
                continue;
            }
            uint32_t want = s_up_total - s_up_got, room = sizeof s_chunk - s_chunk_n;
            if (want > room) want = room;
            uint32_t n = tud_cdc_read(s_chunk + s_chunk_n, want);
            if (n == 0) break;
            s_chunk_n += n;
            s_up_got += n;
            if (s_chunk_n == sizeof s_chunk || s_up_got == s_up_total) s_chunk_full = 1;
            continue;
        }
        char c;
        if (!tud_cdc_available() || tud_cdc_read(&c, 1) != 1) break;
        if (c == '\r' || c == '\n') {
            s_line_cr = c == '\r';
            if (s_len) { s_line[s_len] = 0; command(s_line); s_len = 0; }
        } else if (s_len < sizeof s_line - 1) {
            s_line[s_len++] = c;
        }
    }
    tud_cdc_write_flush();
}

#define USBCTRL_IRQ 14u
#define NVIC_ISER (*(volatile uint32_t (*)[2])0xE000E100u)
#define NVIC_ICER (*(volatile uint32_t (*)[2])0xE000E180u)
#define NVIC_ICPR (*(volatile uint32_t (*)[2])0xE000E280u)
#define SCB_VTOR  (*(volatile uint32_t *)0xE000ED08u)

static void mon_init(const uvm2_monitor_host *host)
{
    s_host = host;
    tud_init(0);
    s_ready = 1;
}

/* TinyUSB with the interrupt masked, so the two never run it at the same time. */
static void poll_masked(void)
{
    NVIC_ICER[0] = 1u << USBCTRL_IRQ;
    __asm volatile ("dsb; isb" ::: "memory");
    poll();
    NVIC_ISER[0] = 1u << USBCTRL_IRQ;
}

static void reply(const char *a, uint32_t size, uint32_t crc)
{
    NVIC_ICER[0] = 1u << USBCTRL_IRQ;
    __asm volatile ("dsb; isb" ::: "memory");
    put(a);
    if (size) { put(" "); put_dec_u(size); put(" "); put_hex(crc, 8); }
    put("\r\n");
    NVIC_ISER[0] = 1u << USBCTRL_IRQ;
}

/* Once a frame, from the game's core, NOT from the interrupt: the card is the game's (its own
 * reads and writes happen on this core, between frames), and a 4 KB write takes milliseconds.
 * Besides the upload, it only backs the interrupt up for an event it left queued. */
static void mon_task(void)
{
    poll_masked();
    if (s_up == UP_OPEN) {
        int ok = s_host->open(s_up_path);
        s_up = ok ? UP_DATA : UP_NONE;
        reply(ok ? "OK" : "ERR open", 0, 0);
    }
    if (s_chunk_full) {
        uint32_t n = s_chunk_n;
        if (s_up == UP_DATA) {
            s_up_crc = crc32_update(s_up_crc, s_chunk, n);
            if (!s_host->put(s_chunk, n)) s_up = UP_SKIP;   /* the rest is read and dropped */
        }
        int last = s_up_got == s_up_total;
        s_chunk_n = 0;
        __asm volatile ("dmb" ::: "memory");
        s_chunk_full = 0;
        if (last) {
            int ok = s_up == UP_DATA && s_host->close();
            s_up = UP_NONE;
            if (ok) reply("DONE", s_up_total, s_up_crc ^ 0xFFFFFFFFu);
            else reply("ERR write", 0, 0);
        }
    }
    poll_masked();   /* the next chunk, and the answers out */
}

/* What the game finds at the start of the image (uvm2_monitor.c): the magic, a version,
 * where the image's .bss is (the loader zeroes it), and the entry points. Version 4: init
 * takes the game's uvm2_monitor_host. */
extern uint32_t __mon_bss_start, __mon_bss_end;
typedef struct {
    uint32_t magic, version;
    uint32_t *bss_start, *bss_end;
    void (*init)(const uvm2_monitor_host *host);
    void (*task)(void);
    uint32_t (*status)(void);
    uint32_t (*seen)(void);
} uvm2_monitor_api;

__attribute__((section(".mon_entry"), used))
const uvm2_monitor_api uvm2_monitor_entry = {
    MON_MAGIC, 4, &__mon_bss_start, &__mon_bss_end, mon_init, mon_task, mon_status, mon_seen,
};

/* What the pico-sdk's runtime would supply and a blob without it must: its panic prints
 * through newlib's syscalls, which do not exist here (-Wl,--wrap=panic sends calls here). */
void __wrap_panic(const char *fmt, ...) { (void)fmt; for (;;) { } }
void hard_assertion_failure(void) { for (;;) { } }

/* The USB interrupt, without the pico-sdk's hardware_irq: that one compares the vector table
 * against ITS OWN default handler, but the table is the GAME's (VTOR), with the game's
 * default in every free slot -- it would take USBCTRL for taken. The monitor owns one IRQ,
 * so it writes that slot itself and enables it on the core that calls init (the game's). -Wl,--wrap sends the driver's calls here. */
typedef void (*irq_handler_t)(void);
static irq_handler_t s_handler;
static void counted_irq(void)   /* counted for mon_status / mon_seen */
{
    s_irqs++;
    s_seen |= *(volatile uint32_t *)0x50110098u;   /* USB INTS */
    s_handler();
    if (s_ready) poll();
}
void __wrap_irq_add_shared_handler(unsigned num, irq_handler_t handler, uint8_t prio)
{
    (void)prio;
    s_handler = handler;
    ((volatile uint32_t *)SCB_VTOR)[16 + num] = (uint32_t)counted_irq;
    __asm volatile ("dsb; isb" ::: "memory");
}
void __wrap_irq_remove_handler(unsigned num, irq_handler_t handler) { (void)num; (void)handler; }
void __wrap_irq_set_enabled(unsigned num, bool enabled)
{
    if (enabled) { NVIC_ICPR[num / 32] = 1u << (num % 32); NVIC_ISER[num / 32] = 1u << (num % 32); }
    else NVIC_ICER[num / 32] = 1u << (num % 32);
}

/* newlib's malloc is linked in by the library code and never called: no heap here. */
void *_sbrk(int incr) { (void)incr; return (void *)-1; }
