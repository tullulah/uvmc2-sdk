/* uvm2_bus_pitrex.c — the bus layer of uvm2_bus.h, on a PiTrex (Raspberry Pi Zero 2 W).
 *
 * THE SAME LIST, ANOTHER WAY TO PUT IT ON THE BUS. uvm2_draw.c and the beam model
 * (vectrex-draw) are the ones every cartridge runs; this file is the only part that knows
 * the board. It replaces uvm2_bus.c, nothing else.
 *
 * WHAT IS DIFFERENT ABOUT THIS BOARD. The RP2350 cartridges drive the 6809 bus themselves
 * and have to find E's phase. The PiTrex does not: the Pi sets address and data, raises
 * LATCH_EN, and the board's own logic performs the access on a cycle of E and answers on
 * RDY (pitrexio-gpio.c, vectrexwrite). So every write is ALREADY synchronised to E by
 * hardware, and what is left to software is choosing WHICH cycle: a request raised before
 * the board's latch point takes the coming cycle, one raised after it takes the next.
 *
 * That latch point is a property of the board and it is MEASURED at start-up
 * (measure_latch_point), not written down: the executor then places each delayed request
 * half a period away from it, the furthest it can be from landing on the wrong cycle.
 *
 * TIME is the ARM cycle counter (PMCCNTR, enabled by the baremetal start-up), turned into
 * bus cycles by uvm2_cycles_per_e_q8, which is measured against the VIA's own timer 2.
 */
#include "uvm2_pitrex_cpu.h"
#include <stdint.h>
#include "uvm2_bus.h"
#include "uvm2_pitrex.h"
#include "baremetal/pitrexio-gpio.h"

uvm2_stats_t uvm2_stats;
uint32_t     uvm2_single_cycles;
uint32_t     uvm2_cycles_per_e_q8;

/* What start-up found, kept for whoever reports it (uvm2_pitrex_main.c writes them to the
 * card): there is no debug probe on this board. */
uvm2_pitrex_bus_t uvm2_pitrex_bus;

/* The VIA sits at $D000 on the Vectrex's bus; a list command carries only the register. */
#define VIA_BASE            0xD000u

/* The Pi's peripherals on a BCM2710 (Zero 2 W, Pi 3). The free-running system timer counts
 * microseconds in CLO. */
#define BCM_PERI_BASE       0x3F000000u
#define BCM_ST_CLO          (*(volatile uint32_t *)(BCM_PERI_BASE + 0x3004u))

/* IFR bit 5: timer 2 has run out. */
#define VIA_IFR_T2          0x20u

/* ── Time ─────────────────────────────────────────────────────────────────── */

static inline uint32_t ccnt(void) { return pitrex_ccnt(); }   /* uvm2_pitrex_cpu.h */

uint32_t uvm2_now_us(void) { return BCM_ST_CLO; }

/* CPU cycles in `cycles` bus cycles. 64 bits because the longest delay of a command (4095)
 * times a period in Q8 (~170000) does not fit in 32. */
static inline uint32_t cpu_cycles(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * uvm2_cycles_per_e_q8) >> 8);
}

static inline void spin_until(uint32_t from, uint32_t span)
{
    while ((uint32_t)(ccnt() - from) < span) { }
}

/* Interrupts off while the bus is being timed: an exception between two writes is a write
 * on the wrong cycle. Returns the CPSR to give back. */
static inline uint32_t irq_off(void)
{
    uint32_t cpsr;
    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
    __asm__ volatile("cpsid if");
    return cpsr;
}
static inline void irq_restore(uint32_t cpsr)
{
    __asm__ volatile("msr cpsr_c, %0" : : "r"(cpsr));
}

/* ── Start-up ─────────────────────────────────────────────────────────────── */

void uvm2_cpu_init(void) { }          /* .bss and the clock are the baremetal start-up's */
void uvm2_set_sys_clock(void) { }
void uvm2_bus_pads(void) { }          /* vectrexinit sets the GPIO, in uvm2_bus_halt */

#define VIA_IER_SET 0x80u      /* writing IER with bit 7 set SETS the other bits given */

/* The 6809 is halted by a jumper on the PiTrex, not by us: vectrexinit only CHECKS it, and
 * that check is the one thing that tells a missing jumper from a broken drawing. */
void uvm2_bus_halt(void)
{
    uvm2_pitrex_bus.init_result = vectrexinit(1);
    /* THE RESET BUTTON'S MARK (uvm2_pitrex.h, UVM2_PITREX_RESET_MARK): one bit of the VIA's
     * interrupt-enable register, set here and never written again. The console's reset
     * clears the whole VIA, so the bit gone IS the button — however short the press, and
     * whatever a list has put back since in the registers it does program. (DDRB, which the
     * cartridges watch, is written by every frame's list here: a quick press was over before
     * anyone looked, and the picture stayed black with the VIA's modes gone.) The bit enables
     * an interrupt on CB1, a line the cartridge port leaves unused; the 6809 is halted. */
    if (uvm2_pitrex_bus.init_result > 0)
        vectrexwrite(VIA_BASE | UVM2_VIA_IER, VIA_IER_SET | UVM2_PITREX_RESET_MARK);
}

/* THE PERIOD OF E IN CPU CYCLES, against the VIA's timer 2, which counts E and nothing
 * else. One-shot from T2_COUNT; IFR is polled until it runs out. The poll costs about one
 * bus cycle, so the error is a couple of cycles in T2_COUNT.
 *
 * BOUNDED. A VIA that never raises the flag would hang here, before anything is on screen
 * and with nothing on the card to say why. After T2_TIMEOUT_US the nominal period is used
 * and e_measured = 0 goes into the report. */
#define T2_COUNT            50000u    /* 33 ms: long enough for 0.01 %, short enough to boot */
/* Four times its length. In 64 bits: T2_COUNT * 1000000 alone does not fit in 32, and
 * the first kernel timed out after 1.6 ms because of it. */
#define T2_TIMEOUT_US       ((uint32_t)(4ull * T2_COUNT * 1000000ull / UVM2_BUS_HZ))
/* The ARM clock the baremetal start-up sets (pi_support.c, "Speed arm"). Only the fallback
 * when the measurement fails. */
#define ARM_HZ_NOMINAL      1000000000u
void uvm2_measure_e(void)
{
    uint32_t done = 0;
    uint32_t cpsr = irq_off();
    vectrexwrite(VIA_BASE | UVM2_VIA_T2CL, (uint8_t)(T2_COUNT & 0xFFu));
    vectrexwrite(VIA_BASE | UVM2_VIA_T2CH, (uint8_t)(T2_COUNT >> 8));
    const uint32_t t0 = ccnt();
    const uint32_t us0 = uvm2_now_us();
    uint32_t t1 = t0;
    while ((uint32_t)(uvm2_now_us() - us0) < T2_TIMEOUT_US) {
        if (vectrexread(VIA_BASE | UVM2_VIA_IFR) & VIA_IFR_T2) { t1 = ccnt(); done = 1; break; }
    }
    irq_restore(cpsr);
    uvm2_pitrex_bus.e_measured = done;
    uvm2_cycles_per_e_q8 = done
        ? (uint32_t)(((uint64_t)(t1 - t0) << 8) / T2_COUNT)
        : (uint32_t)(((uint64_t)ARM_HZ_NOMINAL << 8) / UVM2_BUS_HZ);
}

/* WHERE THE BOARD LATCHES A REQUEST, measured.
 *
 * After a write returns, a second one is requested `offset` CPU cycles later and the time
 * from one return to the next is taken. While the request still catches the coming cycle
 * that time is one period; once it is too late it is two. The offset where it changes is
 * the latch point, as seen from the return of the previous write — which is exactly the
 * reference the executor has.
 *
 * The register written is DDRA with the value it already holds: a write that changes
 * nothing, however many times it lands. */
#define LATCH_SPAN_PERIODS  2u        /* UVM2_PITREX_LATCH_STEPS offsets across this many periods */
#define LATCH_REPEATS       64u       /* writes averaged at each offset */
static void measure_latch_point(void)
{
    const uint32_t period = cpu_cycles(1);

    for (uint32_t s = 0; s < UVM2_PITREX_LATCH_STEPS; s++) {
        uint32_t offset = (period * LATCH_SPAN_PERIODS * s) / UVM2_PITREX_LATCH_STEPS;
        uint32_t total = 0;
        uint32_t cpsr = irq_off();
        for (uint32_t r = 0; r < LATCH_REPEATS; r++) {
            vectrexwrite(VIA_BASE | UVM2_VIA_DDRA, 0xFF);
            uint32_t t0 = ccnt();
            spin_until(t0, offset);
            vectrexwrite(VIA_BASE | UVM2_VIA_DDRA, 0xFF);
            total += ccnt() - t0;
        }
        irq_restore(cpsr);
        uvm2_pitrex_bus.latch_curve[s] = total / LATCH_REPEATS;
    }

    /* The first offset whose write takes half a period more than an immediate one is past
     * the latch point for the coming cycle. Compared against the immediate write, not
     * against one period: what two back-to-back writes cost is itself a result
     * (back_to_back), and it is not assumed to be one cycle. */
    const uint32_t base = uvm2_pitrex_bus.latch_curve[0];
    uint32_t found = 0;
    uvm2_pitrex_bus.back_to_back = base;
    uvm2_pitrex_bus.write_cycles = (base + period / 2u) / period;
    for (uint32_t s = 1; s < UVM2_PITREX_LATCH_STEPS && !found; s++) {
        if (uvm2_pitrex_bus.latch_curve[s] > base + period / 2u) {
            uvm2_pitrex_bus.latch_point = (period * LATCH_SPAN_PERIODS * s) / UVM2_PITREX_LATCH_STEPS;
            found = 1;
        }
    }
    uvm2_pitrex_bus.latch_found = found;
    /* Not found means every offset took the same cycle count: the model above does not
     * describe this board. Half a period is then the neutral guess, and latch_found = 0 is
     * on the card for whoever reads the log. */
    if (!found) uvm2_pitrex_bus.latch_point = period / 2u;
}

void uvm2_bus_init(void)
{
    pitrex_ccnt_enable();      /* the Zero 1's start-up leaves it off (uvm2_pitrex_cpu.h) */
    uvm2_bus_halt();
    if (uvm2_pitrex_bus.init_result <= 0) return;
    uvm2_pitrex_bus.stage = UVM2_PITREX_STAGE_MEASURE_E;
    uvm2_measure_e();
    uvm2_pitrex_bus.stage = UVM2_PITREX_STAGE_MEASURE_LATCH;
    measure_latch_point();
    uvm2_pitrex_bus.stage = UVM2_PITREX_STAGE_BUS_READY;
}

/* ── Command stream executor ──────────────────────────────────────────────── */

/* One command is one bus cycle plus `delay` more before the next one: the next write has
 * to RETURN delay + 1 cycles after this one did.
 *
 * WHAT THE BOARD ALLOWS, measured (hardware/pitrex/logs): an access is one cycle of E and
 * the board leaves one idle cycle before the next, even with the request held up through
 * it. So a write costs `write_cycles` = 2, not 1, and that is the board's logic, not the
 * way it is driven. Every period the request is raised past the latch point adds one more.
 * For the next write to return `gap` cycles after this one, the request goes up
 *
 *     (gap - write_cycles) periods  +  (latch_point - half a period)
 *
 * after this one returned: the middle of the window that takes that cycle.
 *
 * A COMMAND THAT ASKS FOR LESS THAN A WRITE COSTS RUNS LONG, AND NOTHING IS TAKEN BACK FOR IT.
 *
 * It used to be owed: what a command ran long by came out of the waits that followed, so that
 * the list's clock held. But the waits it came out of are not slack — the one after T1CH is
 * the ramp finishing, the one after the mux opens is the sample-and-hold charging — and a
 * write that lands inside them is Port A changing under a running ramp. Seen on the console
 * (2026-11-24): short strokes out of shape, spurs at the vertices, and with sound samples in
 * the list (six commands with no gap in a row, so six cycles owed) letters with their strokes
 * out of place and lines drawn as dots.
 *
 * So every wait is given whole, and the list is as long as it is. The builder counts the
 * same way (uvm2_draw.c, UVM2_CMD_MIN_CYCLES), so its filler still ends the frame on time.
 * `stretched` counts the commands that asked for less than a write costs. */
/* ── CORE 1 LEAVES THE BUS (going back to the menu, uvm2_pitrex_main.c) ───────
 * Core 0 asks; core 1 answers at the next place it would touch the bus — the start of a
 * list or of a controller read, never inside one — by going back to the baremetal kernel's
 * park loop, where the next program (the menu, another game) finds it. */
volatile uint32_t uvm2_pitrex_park_request;
void parkCore1(void);                                   /* libbaremetal, pi_support.c */

/* BEFORE IT PARKS, CORE 1 FORGETS THE CODE IT RAN. The next program is loaded by core 0 at
 * the same addresses, and core 0 empties ITS caches before starting it; core 1's instruction
 * cache still holds this program's code for those addresses, and so do its branch predictor
 * and its translations. Started again at the new program's entry it would run the old
 * program's instructions: the second game of a session came up with core 1 "running" and
 * never alive, and stayed black (logs of 2026-11-23: Black Widow, Lunar Lander, Tac/Scan). */
static inline void forget_code(void)
{
    __asm__ volatile("mcr p15, 0, %0, c7, c5, 0\n"      /* ICIALLU: this core's instruction cache */
                     "mcr p15, 0, %0, c7, c5, 6\n"      /* BPIALL:  its branch predictor */
                     "mcr p15, 0, %0, c8, c7, 0\n"      /* TLBIALL: its translations */
                     : : "r"(0) : "memory");
    PITREX_DSB();
    PITREX_ISB();
}

static inline void park_if_asked(void)
{
#ifdef UVM2_DUAL_CORE
    if (uvm2_pitrex_park_request && UVM2_CPUID == 1u) { forget_code(); parkCore1(); }
#endif
}

uint32_t uvm2_exec(const uint8_t *cmds, uint32_t count)
{
    park_if_asked();
    const uint32_t period = cpu_cycles(1);
    const uint32_t write_cycles = uvm2_pitrex_bus.write_cycles;
    /* May be negative (a latch point earlier than half a period after the return). */
    const int32_t lead = (int32_t)uvm2_pitrex_bus.latch_point - (int32_t)(period / 2u);
    uint32_t cycles = 0, stretched = 0;
    uint32_t cpsr = irq_off();

    while (count--) {
        uint32_t v     = (uint32_t)cmds[0] | ((uint32_t)cmds[1] << 8) | ((uint32_t)cmds[2] << 16);
        uint32_t out   = UVM2_CMD_REG_DATA(v);
        uint32_t delay = UVM2_CMD_DELAY(v);
        cmds += 3;

        vectrexwrite(VIA_BASE | (out >> 8), (uint8_t)out);

        /* Cycles until the next write: what the list wants, and never less than a write. */
        const uint32_t wanted = delay + 1u;
        if (wanted > write_cycles) {
            uint32_t t0 = ccnt();
            int32_t span = (int32_t)cpu_cycles(wanted - write_cycles) + lead;
            if (span > 0) spin_until(t0, (uint32_t)span);
            cycles += wanted;
        } else {
            cycles += write_cycles;                 /* the next write goes out at once */
            if (wanted < write_cycles) stretched++;
        }
    }

    irq_restore(cpsr);
    uvm2_pitrex_bus.stretched = stretched;
    uvm2_pitrex_bus.late_max  = 0;      /* nothing is owed any more (see above) */
    uvm2_pitrex_bus.stage = UVM2_PITREX_STAGE_DRAWING;
#ifndef UVM2_DUAL_CORE
    /* One core: the controllers are read here, where the cartridges' core 1 reads them. */
    uvm2_pitrex_between_lists();
#endif
    return cycles;
}

void uvm2_bus_delay(uint32_t cycles)
{
    spin_until(ccnt(), cpu_cycles(cycles));
}

/* ── Single accesses ──────────────────────────────────────────────────────── */

void uvm2_via_write(uint32_t reg, uint32_t data)
{
    park_if_asked();
    vectrexwrite(VIA_BASE | (reg & 0xFu), (uint8_t)data);
    uvm2_single_cycles += 1u;
}

uint8_t uvm2_via_read(uint32_t reg)
{
    park_if_asked();
    uint8_t v = vectrexread(VIA_BASE | (reg & 0xFu));
    uvm2_single_cycles += 1u;
    return v;
}
