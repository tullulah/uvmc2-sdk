/* uvm2_host_stubs.c — what uvm2_draw.c calls in its neighbours, for a host tool.
 *
 * uvm2_draw.c is built into host tools on its own, without the bus, the sample
 * player, the config loader or the Rust runtime around it. Each tool used to
 * carry its own copy of these stubs, and every time uvm2_draw.c gained a
 * neighbour the copies fell behind: on 2026-10-01 six of the eight tools no
 * longer linked. Link this file instead:
 *
 *     cc ... tools/<tool>.c uvm2_draw.c tools/uvm2_host_stubs.c <libvectrex_draw_cabi.a>
 *
 * tools/build_host_tools.sh builds them all with the flags they need — among
 * them -DUVM2_HZ=0, without which a tool measures the 50 Hz padding.
 *
 * EVERYTHING HERE IS WEAK, so a tool that defines one of these itself — to
 * count calls, say — keeps its own. A new neighbour of uvm2_draw.c belongs here
 * once, not in every tool.
 */
#include <stdint.h>

#define WEAK __attribute__((weak))

/* the config loader (uvm2_config.c): no SD card on a host */
WEAK void uvm2_config_load(void) {}
WEAK volatile int uvm2_have_calibration = 0;

/* the sample player (uvm2_smp.c): silent */
WEAK unsigned uvm2_smp_s_active = 0;           /* uvm2_smp_active() is inline over this */
WEAK int      uvm2_smp_due(uint32_t c, uint8_t *v) { (void)c; (void)v; return 0; }
WEAK uint8_t  uvm2_smp_mixer(void) { return 0; }
WEAK int      uvm2_smp_needs_latch(void) { return 0; }
WEAK void     uvm2_smp_frame(uint32_t c) { (void)c; }
WEAK uint32_t uvm2_smp_injected = 0;

/* the bus layer's clock (uvm2_bus.c): a host has no TIMER0, and 0 is the honest
 * answer — see uvm2_now_us in uvm2_bus.h */
WEAK uint32_t uvm2_now_us(void) { return 0; }

/* the bus's single accesses: nothing to drive */
WEAK void    uvm2_bus_delay(uint32_t c) { (void)c; }
WEAK void    uvm2_via_write(uint32_t r, uint32_t d) { (void)r; (void)d; }
WEAK uint8_t uvm2_via_read(uint32_t r) { (void)r; return 0; }

/* the Rust beam model's static library asks for this; nothing unwinds here */
WEAK void rust_eh_personality(void) {}

/* The button cache core 1 fills on the console (uvm2_core1.c). uvm2_hud.c reads it; a tool
 * that tests buttons defines its own, which wins over this weak one. Nothing pressed. */
__attribute__((weak)) volatile uint8_t uvm2_cached_buttons = 0xFFu;
