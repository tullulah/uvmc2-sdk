/*
 * rp2350_jack.c — uvm2_jack on the debug cartridge: the game's jack sound out of USB headphones.
 *
 * The debug cart has no DAC and no jack (its GPIO43-47 are not connected), but its USB-C is
 * wired as the UVMC2's: VBUS on the +5V rail, 5.1k on CC1/CC2, D+/D- on the RP2350's own PHY
 * (uvm2_usbaudio.h). So here the jack IS the headphones: uvm2_jack_init starts the port as a
 * host, uvm2_jack_space asks for samples only while a device is playing, and the samples go to
 * uvm2_usbaudio. Unplugged, it asks for none, and a game's console PSG path (which it keeps on
 * here) is what is heard: see uvm2_jack_usb_playing.
 *
 * Built only with RP2350_USB_AUDIO=1 (uvm2.mk, rp2350-cart), together with uvm2_usbaudio.c.
 * Neither links the pico-sdk: games on this cart are bare images under the Rust BIOS. The few
 * pico-sdk functions uvm2_usbaudio.c calls are below, on the registers; its headers come from
 * the pico-sdk's tree (UVM2_PICO_SDK) with pico_bare/ standing in for CMake's generated ones.
 *
 * THE INTERRUPT. The BIOS's vector table is in flash (VTOR = 0x10000000 on both cores, read
 * over SWD from the menu), so a handler cannot be written into it: the first
 * irq_set_exclusive_handler copies the table this core is using into RAM, patches the one
 * entry and points this core's VTOR at the copy. SVCall and everything else stay the BIOS's.
 * VTOR and the NVIC are per core on the M33, so the BIOS's other core is not touched.
 *
 * THE CLOCKS are the firmware's: it leaves PLL_USB locked at 48 MHz and clk_usb running, and
 * uvm2_usbaudio only configures them when they are not. pll_init and clock_configure_undivided
 * are therefore never reached on this cart; they are here to link, and say so if they are.
 */
#include <stdint.h>
#include <string.h>
#include "uvm2_jack.h"
#include "uvm2_usbaudio.h"
#include "hardware/structs/timer.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/irq.h"

/* uvm2_setting_audio (0 = jack) is sdk_rp2350.c's, a copy of the OPTIONS screen's choice. */

/* ── the pico-sdk functions uvm2_usbaudio.c calls ── */

void busy_wait_us_32(uint32_t us)
{
    const uint32_t t0 = timer0_hw->timerawl;
    while (timer0_hw->timerawl - t0 < us) { }
}

/* see the header comment: the firmware's clocks are already what is asked */
volatile uint32_t rp2350_jack_clock_calls;
void pll_init(PLL pll, uint refdiv, uint vco_freq, uint post_div1, uint post_div2)
{
    (void)pll; (void)refdiv; (void)vco_freq; (void)post_div1; (void)post_div2;
    rp2350_jack_clock_calls++;
}
void clock_configure_undivided(clock_handle_t clock, uint32_t src, uint32_t auxsrc, uint32_t src_freq)
{
    (void)clock; (void)src; (void)auxsrc; (void)src_freq;
    rp2350_jack_clock_calls++;
}

#define SCB_VTOR   (*(volatile uint32_t *)0xE000ED08u)
#define NVIC_ISER  ((volatile uint32_t *)0xE000E100u)
#define NVIC_ICER  ((volatile uint32_t *)0xE000E180u)
#define NVIC_ICPR  ((volatile uint32_t *)0xE000E280u)
#define NVIC_IPR   ((volatile uint8_t  *)0xE000E400u)
#define VECTORS    (16 + NUM_IRQS)

/* 16 system entries + the RP2350's 52 IRQs = 68 words; VTOR wants the table aligned to the
   next power of two of its size, 512 bytes */
static uint32_t s_vectors[VECTORS] __attribute__((aligned(512)));

void irq_set_exclusive_handler(uint num, irq_handler_t handler)
{
    if (SCB_VTOR != (uint32_t)s_vectors) {
        memcpy(s_vectors, (const void *)SCB_VTOR, sizeof s_vectors);
        __asm__ volatile("dsb" ::: "memory");
        SCB_VTOR = (uint32_t)s_vectors;
        __asm__ volatile("dsb\n isb" ::: "memory");
    }
    s_vectors[16 + num] = (uint32_t)handler;
    __asm__ volatile("dsb" ::: "memory");
}

void irq_set_enabled(uint num, bool enabled)
{
    if (enabled) {
        NVIC_ICPR[num >> 5] = 1u << (num & 31);
        NVIC_ISER[num >> 5] = 1u << (num & 31);
    } else {
        NVIC_ICER[num >> 5] = 1u << (num & 31);
    }
}

void irq_set_priority(uint num, uint8_t hardware_priority) { NVIC_IPR[num] = hardware_priority; }

/* ── the jack ── */

static uint32_t s_ring[4096];   /* the stream's ring (uvm2_usbaudio_start): there is no DAC to borrow from */
static int s_on;

int uvm2_jack_init(void)
{
    if (!s_on) { uvm2_usbaudio_start(s_ring, sizeof s_ring / sizeof s_ring[0]); s_on = 1; }
    return 1;
}

int uvm2_jack_space(void)
{
    if (!s_on) return 0;
    uvm2_usbaudio_poll();
    return uvm2_usbaudio_playing() ? uvm2_usbaudio_space() : 0;
}

void uvm2_jack_write_lr(const int16_t *l, const int16_t *r, int n) { uvm2_usbaudio_write_lr(l, r, n); }

void uvm2_jack_write(const int16_t *s, int n) { uvm2_usbaudio_write_lr(s, s, n); }

int uvm2_jack_ok(void) { return s_on; }
int uvm2_jack_set(int on) { (void)on; return s_on; }

/* 1 while headphones take the sound: a game's console path goes quiet then (az_psg.c, and the
 * xx_jack_enabled of the other jack modules, which ask it to choose their path each frame).
 * It polls the host too, at most once a millisecond: a game on its console path never calls
 * uvm2_jack_space, and without a poll the headphones would never be enumerated. */
int uvm2_jack_usb_playing(void)
{
    static uint32_t s_last;
    if (!s_on) return 0;
    const uint32_t now = timer0_hw->timerawl;
    if (now - s_last >= 1000u) { s_last = now; uvm2_usbaudio_poll(); }
    return uvm2_usbaudio_playing();
}

/* For a game that decides its sound path ONCE, at boot (cine_jack.c: several games set their
 * synthesis up only if the jack is on): wait for the headphones to be up before it asks. Up to
 * 300 ms for something to be plugged in at all, then up to 2 s for it to enumerate and start;
 * so with nothing plugged the boot is 300 ms longer, and with headphones it is as long as they
 * take. Returns uvm2_jack_usb_playing(). */
int uvm2_jack_usb_wait(void)
{
    enum { ST_WAIT = 1, ST_ENUM = 2 };          /* uvm2_usbaudio.c's */
    if (!s_on) return 0;
    const uint32_t t0 = timer0_hw->timerawl;
    for (;;) {
        uvm2_usbaudio_poll();
        const int st = uvm2_usbaudio_state();
        const uint32_t dt = timer0_hw->timerawl - t0;
        if (st != ST_WAIT && st != ST_ENUM) break;          /* playing, or a device it cannot use */
        if (st == ST_WAIT && dt >= 300000u) break;          /* nothing plugged in */
        if (dt >= 2300000u) break;
        busy_wait_us_32(1000u);
    }
    return uvm2_usbaudio_playing();
}
