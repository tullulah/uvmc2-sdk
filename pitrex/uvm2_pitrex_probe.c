/* uvm2_pitrex_probe.c — what the PiTrex's handshake does, measured on the board.
 *
 * WHY. The first kernel measured that two writes made back to back through vectrexwrite
 * land TWO bus cycles apart, not one. On the cartridges one command is one cycle, and the
 * beam model is written against that. Whether the second cycle belongs to the board or to
 * the way the handshake is driven decides what the executor can do, and the board's logic
 * (a CPLD) is not published. So it is asked directly, at start-up, and the answers go to
 * UVM2.LOG:
 *
 *   anatomy   for a request raised at several moments after a write: how long until the
 *             board starts the access (RDY low), and how long the access lasts.
 *   hold      LATCH_EN is left HIGH after an access and only data and address change. If
 *             the board treats the request as a level, a second access follows with no new
 *             edge; the two registers are read back to see what really landed.
 *
 * Nothing here draws, and nothing in the executor depends on it: it only reports.
 *
 * The pin map is the board's (pitrexio-gpio.c): data on GPIO16-23, A0-A3 on GPIO7-10, A12
 * and A14 on GPIO11-12, R/#W on 13, RDY on 24, LATCH_EN on 25.
 */
#include <stdint.h>
#include "uvm2_bus.h"
#include "uvm2_pitrex.h"
#include "baremetal/pitrexio-gpio.h"

#define VIA_BASE        0xD000u

#define BCM_PERI_BASE   0x3F000000u
#define GPSET0          (*(volatile uint32_t *)(BCM_PERI_BASE + 0x20001Cu))
#define GPCLR0          (*(volatile uint32_t *)(BCM_PERI_BASE + 0x200028u))
#define GPLEV0          (*(volatile uint32_t *)(BCM_PERI_BASE + 0x200034u))

#define PIN_RDY         (1u << 24)
#define PIN_LATCH_EN    (1u << 25)
#define PIN_RW          (1u << 13)
#define DATA_SHIFT      16
/* The VIA's select lines (A12, A14) and its register on A0-A3, as vectrexwrite builds it. */
#define ADDR_BITS(reg)  ((0x30u | ((reg) & 0x0Fu)) << 7)
#define DATA_ADDR_MASK  ((0xFFu << DATA_SHIFT) | ADDR_BITS(0x0Fu))

uvm2_pitrex_probe_t uvm2_pitrex_probe;

static inline uint32_t ccnt(void)
{
    uint32_t v;
    __asm__ volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(v));
    return v;
}

static inline void barrier(void) { __asm__ volatile("dmb" ::: "memory"); }

static inline void spin(uint32_t from, uint32_t span)
{
    while ((uint32_t)(ccnt() - from) < span) { }
}

/* Wait for RDY to reach `level`, at most `limit` CPU cycles. Returns the cycle counter when
 * it did, and 0 in *ok if it never did. */
static uint32_t wait_rdy(uint32_t level, uint32_t limit, uint32_t *ok)
{
    const uint32_t t0 = ccnt();
    for (;;) {
        uint32_t now = ccnt();
        if (((GPLEV0 & PIN_RDY) != 0u) == (level != 0u)) { *ok = 1; return now; }
        if ((uint32_t)(now - t0) > limit) { *ok = 0; return now; }
    }
}

static inline void put(uint32_t reg, uint32_t data)
{
    barrier(); GPSET0 = (data << DATA_SHIFT) | ADDR_BITS(reg);
}

void uvm2_pitrex_probe_handshake(void)
{
    uvm2_pitrex_probe_t *p = &uvm2_pitrex_probe;
    const uint32_t period = (uvm2_cycles_per_e_q8 >> 8);
    const uint32_t limit  = period * 4u;      /* nothing the board does takes longer */
    uint32_t cpsr, ok;

    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
    __asm__ volatile("cpsid if");

    /* ── anatomy ── */
    for (uint32_t s = 0; s < UVM2_PITREX_PROBE_STEPS; s++) {
        const uint32_t offset = (period * s) / UVM2_PITREX_PROBE_STEPS;
        uint32_t to_start = 0, length = 0, good = 0;
        for (uint32_t r = 0; r < UVM2_PITREX_PROBE_REPEATS; r++) {
            vectrexwrite(VIA_BASE | UVM2_VIA_DDRA, 0xFF);   /* leaves the pins in write mode */
            const uint32_t t0 = ccnt();
            spin(t0, offset);
            barrier(); GPCLR0 = 0xFFFFFFFFu;
            put(UVM2_VIA_DDRA, 0xFF);
            barrier(); GPSET0 = PIN_LATCH_EN;
            const uint32_t t_req = ccnt();
            const uint32_t t_low = wait_rdy(0, limit, &ok);
            uint32_t ok2 = 0;
            const uint32_t t_high = ok ? wait_rdy(1, limit, &ok2) : t_low;
            barrier(); GPCLR0 = 0xFFFFFFFFu;
            barrier(); GPSET0 = PIN_RW;
            if (ok && ok2) { to_start += t_low - t_req; length += t_high - t_low; good++; }
        }
        p->anatomy_good[s]  = good;
        p->anatomy_start[s] = good ? to_start / good : 0;
        p->anatomy_len[s]   = good ? length / good : 0;
    }

    /* ── hold ── two passes with different values, so a register that merely kept what
     * the previous pass wrote cannot pass for a write that landed. */
    static const uint8_t first[UVM2_PITREX_PROBE_HOLDS]  = { 0x5A, 0x3C };
    static const uint8_t second[UVM2_PITREX_PROBE_HOLDS] = { 0xC3, 0xA5 };
    for (uint32_t h = 0; h < UVM2_PITREX_PROBE_HOLDS; h++) {
        vectrexwrite(VIA_BASE | UVM2_VIA_DDRA, 0xFF);
        barrier(); GPCLR0 = 0xFFFFFFFFu;
        put(UVM2_VIA_T1LL, first[h]);
        barrier(); GPSET0 = PIN_LATCH_EN;
        (void)wait_rdy(0, limit, &ok);
        const uint32_t t1 = wait_rdy(1, limit, &ok);
        /* LATCH_EN stays high and R/#W stays low: only data and address change. */
        barrier(); GPCLR0 = DATA_ADDR_MASK;
        put(UVM2_VIA_T1LH, second[h]);
        uint32_t saw_low = 0, saw_high = 0;
        const uint32_t t_low  = wait_rdy(0, limit, &saw_low);
        const uint32_t t_high = saw_low ? wait_rdy(1, limit, &saw_high) : t_low;
        barrier(); GPCLR0 = 0xFFFFFFFFu;
        barrier(); GPSET0 = PIN_RW;

        p->hold_second_seen[h] = saw_low && saw_high;
        p->hold_start[h] = t_low - t1;
        p->hold_end[h]   = t_high - t1;
        p->hold_read_first[h]  = vectrexread(VIA_BASE | UVM2_VIA_T1LL);
        p->hold_read_second[h] = vectrexread(VIA_BASE | UVM2_VIA_T1LH);
        p->hold_want_first[h]  = first[h];
        p->hold_want_second[h] = second[h];
    }

    __asm__ volatile("msr cpsr_c, %0" : : "r"(cpsr));
    p->ran = 1;
}
