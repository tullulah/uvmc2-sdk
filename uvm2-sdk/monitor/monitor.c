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
 * PROTOCOL (prototype): text lines.
 *   v                 -> "UVMC2-MON 1"
 *   r <hex addr> <n>  -> n bytes (n <= 256) as hex, one line
 */
#include <stdint.h>
#include <string.h>
#include "tusb.h"

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

static void command(const char *l)
{
    if (l[0] == 'v') { put("UVMC2-MON 1\r\n"); return; }
    if (l[0] == 'r') {
        const char *p = l + 1;
        uint32_t a = parse_hex(&p), n = parse_dec(&p);
        if (n == 0 || n > 256 || !readable(a, n)) { put("ERR\r\n"); return; }
        for (uint32_t i = 0; i < n; i++) put_hex(*(volatile uint8_t *)(a + i), 2);
        put("\r\n");
        return;
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
    while (tud_cdc_available()) {
        char c;
        if (tud_cdc_read(&c, 1) != 1) break;
        if (c == '\r' || c == '\n') {
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

static void mon_init(void)
{
    tud_init(0);
    s_ready = 1;
}

/* Once a frame, from the game: only a fallback for an event the interrupt left queued. The
 * interrupt is masked meanwhile, so the two never run TinyUSB at the same time. */
static void mon_task(void)
{
    NVIC_ICER[0] = 1u << USBCTRL_IRQ;
    __asm volatile ("dsb; isb" ::: "memory");
    poll();
    NVIC_ISER[0] = 1u << USBCTRL_IRQ;
}

/* What the game finds at the start of the image (uvm2_monitor.c): the magic, a version,
 * where the image's .bss is (the loader zeroes it), and the two entry points. */
extern uint32_t __mon_bss_start, __mon_bss_end;
typedef struct {
    uint32_t magic, version;
    uint32_t *bss_start, *bss_end;
    void (*init)(void);
    void (*task)(void);
    uint32_t (*status)(void);
    uint32_t (*seen)(void);
} uvm2_monitor_api;

__attribute__((section(".mon_entry"), used))
const uvm2_monitor_api uvm2_monitor_entry = {
    MON_MAGIC, 3, &__mon_bss_start, &__mon_bss_end, mon_init, mon_task, mon_status, mon_seen,
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
