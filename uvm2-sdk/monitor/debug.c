/* debug.c — the monitor's debugger: breakpoints, pause, steps and registers of the game's core,
 * with no SWD probe (the UVMC2 has none on the bench). Cortex-M33 MONITOR-MODE debug:
 *
 *   DEMCR.MON_EN   a BKPT, or a step, or a pause, raises the DebugMonitor exception instead of
 *                  halting the core for a probe. Its handler (debugmon_entry) is ours, in the
 *                  PSRAM, written into the game's vector table.
 *   BKPT           the breakpoints are BKPT instructions written over the game's code, which
 *                  in a .um2 lives in SRAM; the original halfword is kept and put back.
 *   MON_STEP       one instruction, then DebugMonitor again: what the steps are made of.
 *   MON_PEND       DebugMonitor at the next chance: the pause.
 *
 * WHILE THE GAME IS HELD the handler waits (WFI) and the USB interrupt, which has the higher
 * priority, goes on answering the IDE: registers, memory, breakpoints, continue. Core 1 goes on
 * redrawing the last picture (UVM2_REPLAY_LAST in a monitor build).
 *
 * STEPS ARE DONE HERE, by ranges, not one round trip per instruction: `s <lo> <hi> <skip>`
 * single-steps until the PC leaves [lo, hi). With skip set, a call made from inside the range
 * is run at full speed: a temporary BKPT at its return address, and stepping resumes there.
 * Step into = the line's range, no skip; over = the line's range, skip; out = the function's
 * range, skip.
 *
 * Only the game's core (core 0) is debugged: a breakpoint in code that core 1 runs (the SDK's
 * replay) would fault there. The IDE only offers lines of the game's own sources.
 *
 * Protocol (lines; answers OK / ERR unless said; events start with '!'):
 *   Z <addr>.. / z <addr>.. set / clear breakpoints (even, in SRAM), several per line. The
 *                           monitor keeps the original halfwords and steps off a breakpoint
 *                           by itself on c and s.
 *   D                       detach: every breakpoint out, the game running (the IDE's session
 *                           ends, or the IDE went away)
 *   H                       pause                       -> "OK", then "!S pause <pc>"
 *   c                       continue (halted)
 *   s <lo> <hi> <skip>      step by range (halted)      -> "OK", then "!S step <pc>"
 *   R                       registers (halted)          -> "R r0 .. r12 sp lr pc xpsr" (hex)
 *   P <n> <value>           write register n (0-12 r, 13 sp no, 14 lr, 15 pc, 16 xpsr)
 *   Q                       state -> "Q run" / "Q halt <reason> <pc>"
 *   events: "!S bp|step|pause|bkpt <pc>" when the game stops. */
#include <stdint.h>
#include <string.h>
#include "monitor.h"

#define DEMCR       (*(volatile uint32_t *)0xE000EDFCu)
#define DFSR        (*(volatile uint32_t *)0xE000ED30u)
#define SHPR3       (*(volatile uint32_t *)0xE000ED20u)
#define SCB_VTOR    (*(volatile uint32_t *)0xE000ED08u)
#define DEMCR_MON_EN   (1u << 16)
#define DEMCR_MON_PEND (1u << 17)
#define DEMCR_MON_STEP (1u << 18)
#define DFSR_HALTED    (1u << 0)
#define DFSR_BKPT      (1u << 1)
#define BKPT_INSN      0xBE00u
#define SRAM_LO        0x20000000u
#define SRAM_HI        0x20080000u

#define MAX_BP 256   /* a step's stops are breakpoints too: every statement of a function */
static uint32_t s_bp_addr[MAX_BP];
static uint16_t s_bp_orig[MAX_BP];
static int s_bp_n;

/* The held context: the exception frame (r0-r3, r12, lr, pc, xpsr) and r4-r11, which the
 * entry pushed (written back on the way out). */
static volatile uint32_t *s_frame;
static uint32_t *s_saved;
static uint32_t s_excret;

enum { RUN, HALT_BP, HALT_STEP, HALT_PAUSE, HALT_BKPT };
static volatile int s_state = RUN;
static volatile int s_event;          /* a stop the interrupt has still to report */

enum { GO_NONE, GO_CONTINUE, GO_STEP };
static volatile int s_go;
static uint32_t s_lo, s_hi;
static int s_skip, s_stepping;
static uint32_t s_prev_pc;            /* where the last single step started */
static uint32_t s_tmp_bp;             /* the return address a skipped call runs to (0 none) */
static uint32_t s_over_bp;            /* a breakpoint lifted for one step, to put back after */

static int bp_find(uint32_t a)
{
    for (int i = 0; i < s_bp_n; i++) if (s_bp_addr[i] == a) return i;
    return -1;
}

static void insn_write(uint32_t a, uint16_t v)
{
    *(volatile uint16_t *)a = v;
    __asm volatile ("dsb; isb" ::: "memory");
}

static int bp_set(uint32_t a)
{
    if ((a & 1u) || a < SRAM_LO || a + 2u > SRAM_HI) return 0;
    if (bp_find(a) >= 0) return 1;
    if (s_bp_n == MAX_BP) return 0;
    s_bp_addr[s_bp_n] = a;
    s_bp_orig[s_bp_n] = *(volatile uint16_t *)a;
    s_bp_n++;
    if (a != s_over_bp) insn_write(a, BKPT_INSN);
    return 1;
}

static int bp_clear(uint32_t a)
{
    int i = bp_find(a);
    if (i < 0) return 0;
    if (a != s_over_bp) insn_write(a, s_bp_orig[i]);
    s_bp_n--;
    s_bp_addr[i] = s_bp_addr[s_bp_n];
    s_bp_orig[i] = s_bp_orig[s_bp_n];
    return 1;
}

/* The temporary breakpoint of a skipped call: its own slot, not a user breakpoint. */
static uint16_t s_tmp_orig;
static void tmp_set(uint32_t a)
{
    s_tmp_bp = a;
    if (bp_find(a) >= 0) return;          /* a user breakpoint there already stops it */
    s_tmp_orig = *(volatile uint16_t *)a;
    insn_write(a, BKPT_INSN);
}
static void tmp_clear(void)
{
    if (s_tmp_bp && bp_find(s_tmp_bp) < 0) insn_write(s_tmp_bp, s_tmp_orig);
    s_tmp_bp = 0;
}

/* The held context's SP: past the frame (8 words, or 26 with the FPU's), and the alignment
 * word the core added if xPSR bit 9 says so. */
static uint32_t held_sp(void)
{
    uint32_t sp = (uint32_t)s_frame + ((s_excret & (1u << 4)) ? 0x20u : 0x68u);
    if (s_frame[7] & (1u << 9)) sp += 4u;
    return sp;
}

/* Was the instruction at `pc` a call whose return address is now in LR? BL (32-bit) or
 * BLX register (16-bit), with the Thumb bit. */
static int just_called(uint32_t pc, uint32_t lr)
{
    uint16_t h = *(volatile uint16_t *)pc;
    if ((h & 0xF800u) == 0xF000u && (((*(volatile uint16_t *)(pc + 2u)) & 0xD000u) == 0xD000u))
        return lr == ((pc + 4u) | 1u);
    if ((h & 0xFF87u) == 0x4780u) return lr == ((pc + 2u) | 1u);
    return 0;
}

static void stop(int why)
{
    s_stepping = 0;
    tmp_clear();
    s_state = why;
    s_event = 1;
    mon_wake_usb();                       /* the interrupt reports it */
}

/* Leave the held context, running or stepping. Off a user breakpoint: lift it, one step,
 * put it back (s_over_bp). */
static void resume(void)
{
    uint32_t pc = s_frame[6];
    s_state = RUN;
    if (bp_find(pc) >= 0) {
        s_over_bp = pc;
        insn_write(pc, s_bp_orig[bp_find(pc)]);
        s_prev_pc = pc;
        DEMCR |= DEMCR_MON_STEP;
    } else if (s_stepping) {
        s_prev_pc = pc;
        DEMCR |= DEMCR_MON_STEP;
    }
}

/* DebugMonitor, from debugmon_entry. Runs at the lowest priority (SHPR3), under the USB
 * interrupt, so the IDE is still answered while this waits. */
void debugmon_c(uint32_t *frame, uint32_t excret, uint32_t *saved)
{
    uint32_t dfsr = DFSR;
    DFSR = dfsr;                          /* write-1-to-clear */
    DEMCR &= ~(DEMCR_MON_STEP | DEMCR_MON_PEND);
    s_frame = frame;
    s_saved = saved;
    s_excret = excret;
    uint32_t pc = frame[6];

    /* The step off a lifted breakpoint is done: put it back. */
    if (s_over_bp) {
        int i = bp_find(s_over_bp);
        if (i >= 0) insn_write(s_over_bp, BKPT_INSN);
        s_over_bp = 0;
        if (!s_stepping && s_state == RUN && !(dfsr & DFSR_BKPT)) return;   /* continuing */
    }

    if (dfsr & DFSR_BKPT) {
        if (s_tmp_bp && pc == s_tmp_bp) {
            tmp_clear();                  /* back from a skipped call: step on below */
            if (bp_find(pc) >= 0) { stop(HALT_BP); goto held; }
        } else if (bp_find(pc) >= 0) {
            stop(HALT_BP); goto held;
        } else {
            stop(HALT_BKPT); goto held;   /* a BKPT in the game itself */
        }
    } else if (!s_stepping) {
        stop(HALT_PAUSE); goto held;      /* MON_PEND */
    }

    /* Stepping. */
    if (pc >= s_lo && pc < s_hi) { s_prev_pc = pc; DEMCR |= DEMCR_MON_STEP; return; }
    if (s_skip && s_prev_pc >= s_lo && s_prev_pc < s_hi && just_called(s_prev_pc, frame[5])) {
        tmp_set(frame[5] & ~1u);          /* the call runs at full speed */
        return;
    }
    stop(HALT_STEP);

held:
    while (s_go == GO_NONE) __asm volatile ("wfi");
    int go = s_go;
    s_go = GO_NONE;
    if (go == GO_STEP) s_stepping = 1;
    resume();
}

/* The entry: which stack the frame is on, r4-r11 saved (and written back: the IDE may change
 * them), then the C part. Ten words keep the stack 8-byte aligned. */
__attribute__((naked)) void debugmon_entry(void)
{
    __asm volatile (
        "tst lr, #4            \n"
        "ite eq                \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "mov r1, lr            \n"
        "push {r4-r12, lr}     \n"
        "mov r2, sp            \n"
        "bl debugmon_c         \n"
        "pop {r4-r12, pc}      \n"
    );
}

void dbg_init(void)
{
    ((volatile uint32_t *)SCB_VTOR)[12] = (uint32_t)debugmon_entry;
    SHPR3 = (SHPR3 & ~0xFFu) | 0xE0u;     /* DebugMonitor: the lowest priority there is */
    __asm volatile ("dsb; isb" ::: "memory");
    DEMCR |= DEMCR_MON_EN;
}

static const char *const s_why[] = { "run", "bp", "step", "pause", "bkpt" };

/* From poll (the USB interrupt): a stop to report. */
void dbg_events(void)
{
    if (!s_event) return;
    s_event = 0;
    mon_put("!S ");
    mon_put(s_why[s_state]);
    mon_put(" ");
    mon_put_hex(s_frame ? s_frame[6] : 0u, 8);
    mon_put("\r\n");
}

static uint32_t reg_get(int n)
{
    if (n <= 3) return s_frame[n];
    if (n <= 11) return s_saved[n - 4];
    if (n == 12) return s_frame[4];
    if (n == 13) return held_sp();
    if (n == 14) return s_frame[5];
    if (n == 15) return s_frame[6];
    return s_frame[7];
}

static int reg_set(int n, uint32_t v)
{
    if (n <= 3) s_frame[n] = v;
    else if (n <= 11) s_saved[n - 4] = v;
    else if (n == 12) s_frame[4] = v;
    else if (n == 14) s_frame[5] = v;
    else if (n == 15) s_frame[6] = v | 1u;   /* stacked PC: the return address, Thumb bit... */
    else if (n == 16) s_frame[7] = v;
    else return 0;                           /* SP is where the frame is: not movable here */
    if (n == 15) s_frame[6] &= ~1u;          /* ...is NOT set in a stacked PC */
    return 1;
}

/* A command line, from poll. 1 = it was one of ours. */
int dbg_command(const char *l)
{
    const char *p = l + 1;
    int halted = s_state != RUN;
    switch (l[0]) {
    case 'Z': case 'z': {
        int ok = 1;
        for (;;) {
            while (*p == ' ') p++;
            if (!*p) break;
            uint32_t a = mon_parse_hex(&p);
            if (l[0] == 'Z') ok &= bp_set(a); else bp_clear(a);   /* clearing one not set is fine */
        }
        mon_put(ok ? "OK\r\n" : "ERR\r\n");
        return 1;
    }
    case 'D':
        while (s_bp_n) bp_clear(s_bp_addr[s_bp_n - 1]);
        mon_put("OK\r\n");
        if (halted) s_go = GO_CONTINUE;
        return 1;
    case 'H':
        mon_put("OK\r\n");
        if (!halted) DEMCR |= DEMCR_MON_PEND;
        else s_event = 1;                    /* already: say so again */
        return 1;
    case 'c':
        if (!halted) { mon_put("ERR\r\n"); return 1; }
        mon_put("OK\r\n");
        s_go = GO_CONTINUE;
        return 1;
    case 's':
        if (!halted) { mon_put("ERR\r\n"); return 1; }
        s_lo = mon_parse_hex(&p);
        s_hi = mon_parse_hex(&p);
        s_skip = (int)mon_parse_hex(&p);
        mon_put("OK\r\n");
        s_go = GO_STEP;
        return 1;
    case 'R':
        if (!halted) { mon_put("ERR\r\n"); return 1; }
        mon_put("R");
        for (int n = 0; n <= 16; n++) { mon_put(" "); mon_put_hex(reg_get(n), 8); }
        mon_put("\r\n");
        return 1;
    case 'P': {
        if (!halted) { mon_put("ERR\r\n"); return 1; }
        int n = (int)mon_parse_hex(&p);
        uint32_t v = mon_parse_hex(&p);
        mon_put(n >= 0 && n <= 16 && reg_set(n, v) ? "OK\r\n" : "ERR\r\n");
        return 1;
    }
    case 'Q':
        mon_put("Q ");
        mon_put(halted ? "halt " : "run");
        if (halted) { mon_put(s_why[s_state]); mon_put(" "); mon_put_hex(s_frame[6], 8); }
        mon_put("\r\n");
        return 1;
    }
    return 0;
}
