/* uvm2_pitrex_main.c — start-up of an SDK game on a PiTrex.
 *
 * The baremetal kernel (libbaremetal: boot, MMU, clock, UART, the SD card's FAT) calls
 * main(). On the cartridges that place is uvm2_pico_main.c: bring the runtime up, then run
 * the game's own main, which the build renames to uvm2_game_main.
 *
 * THERE IS NO PROBE ON THIS BOARD, so what start-up measures goes to the SD card
 * (uvm2_pitrex_report): put the card in a computer and read UVM2.LOG.
 */
#include "uvm2_pitrex_cpu.h"
#define STR_(x) #x
#define STR(x)  STR_(x)
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "uvm2_bus.h"
#include "uvm2_draw.h"
#include "uvm2_input.h"
#include "uvm2_audio.h"
#include "uvm2_config.h"
#include "uvm2_pitrex.h"
#include "baremetal/ff.h"
#include "baremetal/pi_support.h"     /* GlobalMemSettings: what the loader leaves for a program */
#include "baremetal/globalVars.h"      /* CORE1_STATE: where the start-up keeps each core's state */

/* What a fault left (see "A PROGRAM THAT FAULTS SAYS SO", below). */
enum { FAULT_KIND, FAULT_PC, FAULT_LR, FAULT_SP, FAULT_ADDR, FAULT_STATUS, FAULT_CORE, FAULT_CPSR, FAULT_WORDS };
volatile uint32_t uvm2_pitrex_fault[FAULT_WORDS];   /* FAULT_KIND 0 = none */
void uvm2_pitrex_fault_core0(void);

/* THE FIRST FAULT, KEPT OUTSIDE THE PROGRAM. A fault's report is written by the program that
 * faulted, with whatever it broke; when that report faults too, the card gets an empty file
 * and the screen stays black with nothing said (2026-11-24: five empty reports in a row). So
 * the first fault of a start is also copied here, to memory no image owns and no start-up
 * clears — the megabyte below the resident image holds the stacks and, at its very top, the
 * kernel's own few words (globalVars.h, GLOBAL_VARS); this sits just under those. Whatever
 * program starts next — the menu, as a rule — prints it in its own reports. */
#define FAULT_KEPT_MAGIC  0x464C5431u               /* "FLT1" */
#define FAULT_KEPT_NAME   16u
typedef struct {
    uint32_t magic;
    uint32_t again;                                 /* faults that came after it, before a report got out */
    uint32_t words[FAULT_WORDS];
    char     name[FAULT_KEPT_NAME];                 /* the program it happened in */
} fault_kept_t;
#define FAULT_KEPT        ((volatile fault_kept_t *)(GLOBAL_VARS - 0x100u))

/* THE WAY BACK, KEPT THE SAME WAY. A program that hands the machine back to the menu leaves
 * its name here just before it does; the menu's reports say so. A return that ends in a
 * black screen is then one of two things, and the card tells them apart: the menu's report 0
 * names the program (the menu started again, and died later) or it does not (it never got
 * that far). Under the fault record. */
#define BACK_KEPT_MAGIC   0x42414B31u               /* "BAK1" */
typedef struct {
    uint32_t magic;
    uint32_t count;                                 /* returns since the card was started */
    char     name[FAULT_KEPT_NAME];                 /* the last program that came back */
} back_kept_t;
#define BACK_KEPT         ((volatile back_kept_t *)(GLOBAL_VARS - 0x180u))

/* Why uvm2_pitrex_to_menu came back instead of leaving (uvm2_pitrex.h has the values). */
int uvm2_pitrex_to_menu_refused;

void uvm2_core1_start(void);
void uvm2_romzip_load(void);            /* uvm2_romzip.c */

int uvm2_game_main(void);

/* ONE FILE PER REPORT (UVM2_0.LOG, UVM2_1.LOG, ...): a report that dies half-way leaves an
 * empty file, and with a single name it took the previous, good one with it. The number is
 * the order of the calls: 0 before the bus, 1 after measuring it, then (dual core) 2 once
 * core 1 has the loop, and the game's own after that. */
#define REPORT_MAX    4096
#define REPORT_FILES  10u
#define REPORT_FOLDER "/logs"         /* the card's, wherever the program's folder is */
#ifndef UVM2_PITREX_NAME                  /* pitrex.mk: the image's name */
#define UVM2_PITREX_NAME "uvm2"
#endif

/* libbaremetal: core 0's CPUECTLR at start-up (the Zero 2's start-up only: weak, 0 on a Zero) */
extern unsigned int auxControl __attribute__((weak));
#define AUX_CONTROL (&auxControl ? auxControl : 0u)

static uint32_t s_report_index;
static uint32_t s_frames;                  /* frame closes seen (uvm2_pitrex_menu_poll) */
static int32_t  s_last_open = -1, s_last_write = -1, s_last_close = -1;
static uint32_t s_last_written, s_last_wanted;
static volatile uint32_t s_core1_cpuectlr, s_core1_sctlr, s_core1_beats;

static inline uint32_t read_cpuectlr(void)
{
#ifdef PITREX_ARMV6
    return 0u;          /* a Cortex-A53 register: the Zero's ARM11 has none */
#endif
    uint32_t lo, hi;
    __asm__ volatile("mrrc p15, 1, %0, %1, c15" : "=r"(lo), "=r"(hi));
    (void)hi;
    return lo;
}
static inline uint32_t read_sctlr(void)
{
    uint32_t v;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(v));
    return v;
}

/* The game's own lines for the report (CRLF-terminated). A game that has something to say
 * defines it; the default says nothing. */
__attribute__((weak)) const char *uvm2_pitrex_game_note(void) { return ""; }

/* REPORTS ONLY WHEN SOMETHING WENT WRONG, UNLESS ASKED (pitrex.mk, PITREX_REPORTS=1). Each
 * report is a file written to the card, six per run: at start-up, before the first frame, at
 * frame 600 — where it stops the picture for about half a second — and on the way out. For
 * the card a player gets that is time and wear for nothing. A fault is still written: it is
 * the only trace there is, and by then nothing is running to be slowed down. */
static int report_wanted(void)
{
#ifdef UVM2_PITREX_REPORTS
    return 1;
#else
    return uvm2_pitrex_fault[FAULT_KIND] != 0u || FAULT_KEPT->magic == FAULT_KEPT_MAGIC;
#endif
}

int uvm2_pitrex_report(void)
{
    static char txt[REPORT_MAX];
    if (!report_wanted()) return 1;
    const uvm2_pitrex_bus_t *b = &uvm2_pitrex_bus;
    const uint32_t q8 = uvm2_cycles_per_e_q8;
    int n = 0;

#define LINE(...) do { if (n < REPORT_MAX) n += snprintf(txt + n, (size_t)(REPORT_MAX - n), __VA_ARGS__); } while (0)
    LINE("uvmc2-sdk on PiTrex   report %u\r\n", (unsigned)s_report_index);
    LINE("previous report  open %d  write %d  close %d  written %u of %u   (FatFs results, 0 = OK, -1 = none yet)\r\n",
         (int)s_last_open, (int)s_last_write, (int)s_last_close,
         (unsigned)s_last_written, (unsigned)s_last_wanted);
    LINE("cpuectlr         core0 at boot %08X  core0 now %08X  core1 %08X   (bit 6 = caches coherent between cores)\r\n",
         (unsigned)AUX_CONTROL, (unsigned)read_cpuectlr(), (unsigned)s_core1_cpuectlr);
    LINE("sctlr            core0 %08X  core1 %08X   (bit 0 MMU, bit 2 data cache)\r\n",
         (unsigned)read_sctlr(), (unsigned)s_core1_sctlr);
    LINE("menu             %d   (1 = started by the PiTrex loader: the four buttons held go back to it)\r\n",
         uvm2_pitrex_has_menu());
    LINE("core1 beats      %u   (counts while core 1 is alive, before it takes the loop)\r\n",
         (unsigned)s_core1_beats);
    LINE("stage            %u   (1 first bus access, 2 measuring E, 3 measuring the latch, 4 bus ready, 5 drawing)\r\n",
         (unsigned)b->stage);
    LINE("init_result      %d   (1 = bus ready, -1 = /HALT not asserted)\r\n", (int)b->init_result);
    LINE("cpu_per_e_q8     %u   (%u.%02u CPU cycles per bus cycle)\r\n",
         (unsigned)q8, (unsigned)(q8 >> 8), (unsigned)(((q8 & 0xFFu) * 100u) >> 8));
#ifdef UVM2_DUAL_CORE
    LINE("core1            started %u   state before %u, after %u   (2 = parked, 3 = running)\r\n",
         (unsigned)b->core1_started, (unsigned)b->core1_state_before, (unsigned)b->core1_state_after);
    LINE("core 1 us        exec %u  input %u  rest %u  wait %u  overrun %u  idle frames %u\r\n",
         (unsigned)uvm2_stats.us_exec, (unsigned)uvm2_stats.us_input, (unsigned)uvm2_stats.us_rest,
         (unsigned)uvm2_stats.us_wait, (unsigned)uvm2_stats.overrun, (unsigned)uvm2_stats.idle_frames);
#else
    LINE("core1            not used (single-core build)\r\n");
#endif
    LINE("e_measured       %u   (0 = the VIA's timer 2 never ran out: nominal period used)\r\n", (unsigned)b->e_measured);
    LINE("back_to_back     %u   CPU cycles between two writes with no wait\r\n", (unsigned)b->back_to_back);
    LINE("write_cycles     %u   bus cycles one write costs\r\n", (unsigned)b->write_cycles);
    LINE("stretched        %u   commands of the last list that asked for less than that\r\n", (unsigned)b->stretched);
    LINE("late_max         %u   bus cycles the latest write of the last list was behind\r\n", (unsigned)b->late_max);
    LINE("latch_found      %u\r\n", (unsigned)b->latch_found);
    LINE("latch_point      %u   CPU cycles after a write returns\r\n", (unsigned)b->latch_point);
    LINE("latch_curve      offset -> cycles to the next write's return (CPU cycles)\r\n");
    for (uint32_t s = 0; s < UVM2_PITREX_LATCH_STEPS; s++) {
        uint32_t offset = (uint32_t)(((uint64_t)q8 * 2u * s / UVM2_PITREX_LATCH_STEPS) >> 8);
        LINE("  %5u -> %5u\r\n", (unsigned)offset, (unsigned)b->latch_curve[s]);
    }
    {
        const uvm2_pitrex_probe_t *pr = &uvm2_pitrex_probe;
        LINE("probe ran        %u\r\n", (unsigned)pr->ran);
        LINE("anatomy          request offset -> LATCH_EN to RDY low, RDY low to high, completed\r\n");
        for (uint32_t s = 0; s < UVM2_PITREX_PROBE_STEPS; s++) {
            uint32_t offset = (uint32_t)(((uint64_t)q8 * s / UVM2_PITREX_PROBE_STEPS) >> 8);
            LINE("  %5u -> %5u  %5u  %u\r\n", (unsigned)offset, (unsigned)pr->anatomy_start[s],
                 (unsigned)pr->anatomy_len[s], (unsigned)pr->anatomy_good[s]);
        }
        for (uint32_t h = 0; h < UVM2_PITREX_PROBE_HOLDS; h++)
            LINE("hold %u           second access seen %u  start %u  end %u  first %02X/%02X  second %02X/%02X (read/wanted)\r\n",
                 (unsigned)h, (unsigned)pr->hold_second_seen[h], (unsigned)pr->hold_start[h],
                 (unsigned)pr->hold_end[h], pr->hold_read_first[h], pr->hold_want_first[h],
                 pr->hold_read_second[h], pr->hold_want_second[h]);
    }
    LINE("last frame: commands %u  bus_cycles %u  vectors %u  moves %u  dropped %u  recals %u\r\n",
         (unsigned)uvm2_stats.commands, (unsigned)uvm2_stats.bus_cycles,
         (unsigned)uvm2_stats.vectors_last, (unsigned)uvm2_stats.moves_last,
         (unsigned)uvm2_stats.dropped, (unsigned)uvm2_stats.recals);
    LINE("game core us     %u between two frame closes (the game's own work), %u closing the frame\r\n",
         (unsigned)uvm2_stats.us_wait, (unsigned)uvm2_stats.us_frame_end);
    LINE("frames           %u published by the game\r\n", (unsigned)s_frames);
    LINE("frame period us: last %u  min %u  max %u\r\n",
         (unsigned)uvm2_stats.us_frame_last, (unsigned)uvm2_stats.us_frame_min,
         (unsigned)uvm2_stats.us_frame_max);
    if (uvm2_pitrex_fault[FAULT_KIND]) {
        LINE("FAULT            kind %u (1 memory access, 2 jump into nothing, 3 not an instruction)  core %u\r\n",
             (unsigned)uvm2_pitrex_fault[FAULT_KIND], (unsigned)uvm2_pitrex_fault[FAULT_CORE]);
        LINE("FAULT            pc %08X  touching %08X  status %08X\r\n",
             (unsigned)uvm2_pitrex_fault[FAULT_PC], (unsigned)uvm2_pitrex_fault[FAULT_ADDR],
             (unsigned)uvm2_pitrex_fault[FAULT_STATUS]);
        LINE("FAULT            lr %08X  sp %08X  cpsr %08X   (addr2line -e build_pitrex/%s.elf <pc> <lr>)\r\n",
             (unsigned)uvm2_pitrex_fault[FAULT_LR], (unsigned)uvm2_pitrex_fault[FAULT_SP],
             (unsigned)uvm2_pitrex_fault[FAULT_CPSR], UVM2_PITREX_NAME);
    } else {
        LINE("fault            none\r\n");
    }
    {
        extern uint32_t uvm2_smp_stream_starved;
        extern uint32_t uvm2_smp_stream_fill(void);
        LINE("sound stream     waiting %u  starved %u   (samples: in the ring now, and due with none written)\r\n",
             (unsigned)uvm2_smp_stream_fill(), (unsigned)uvm2_smp_stream_starved);
    }
    if (BACK_KEPT->magic == BACK_KEPT_MAGIC) {
        char name[FAULT_KEPT_NAME + 1u];
        for (uint32_t i = 0; i < FAULT_KEPT_NAME; i++) name[i] = BACK_KEPT->name[i];
        name[FAULT_KEPT_NAME] = 0;
        LINE("came back        %u times, the last from %s\r\n", (unsigned)BACK_KEPT->count, name);
    }
    LINE("to_menu refused  %d   (0 no, 1 no menu on this card, 2 this IS the menu, 3 core 1 did not park)\r\n",
         uvm2_pitrex_to_menu_refused);
    if (FAULT_KEPT->magic == FAULT_KEPT_MAGIC) {
        const volatile fault_kept_t *k = FAULT_KEPT;
        char name[FAULT_KEPT_NAME + 1u];
        for (uint32_t i = 0; i < FAULT_KEPT_NAME; i++) name[i] = k->name[i];
        name[FAULT_KEPT_NAME] = 0;
        LINE("EARLIER FAULT    in %s: kind %u  core %u  pc %08X  touching %08X  status %08X\r\n",
             name, (unsigned)k->words[FAULT_KIND], (unsigned)k->words[FAULT_CORE],
             (unsigned)k->words[FAULT_PC], (unsigned)k->words[FAULT_ADDR], (unsigned)k->words[FAULT_STATUS]);
        LINE("EARLIER FAULT    lr %08X  sp %08X  cpsr %08X  and %u more before a report got out\r\n",
             (unsigned)k->words[FAULT_LR], (unsigned)k->words[FAULT_SP], (unsigned)k->words[FAULT_CPSR],
             (unsigned)k->again);
    }
    LINE("%s", uvm2_pitrex_game_note());
#undef LINE
    if (n > REPORT_MAX) n = REPORT_MAX;

    /* logs/<program>_<n>.log: each program its own, so that the one that did not start is
     * not overwritten by the next one that did. */
    char name[64];
    snprintf(name, sizeof name, REPORT_FOLDER "/%s_%u.log", UVM2_PITREX_NAME,
             (unsigned)(s_report_index < REPORT_FILES ? s_report_index : REPORT_FILES - 1u));
    s_report_index++;
    f_mkdir(REPORT_FOLDER);                    /* there already: not an error */

    FIL f;
    UINT written = 0;
    s_last_wanted = (uint32_t)n; s_last_written = 0; s_last_write = -1; s_last_close = -1;
    s_last_open = (int32_t)f_open(&f, name, FA_WRITE | FA_CREATE_ALWAYS);
    if (s_last_open != FR_OK) return 0;
    s_last_write = (int32_t)f_write(&f, txt, (UINT)n, &written);
    s_last_written = (uint32_t)written;
    s_last_close = (int32_t)f_close(&f);
    return s_last_write == FR_OK && s_last_close == FR_OK && written == (UINT)n;
}

#ifdef UVM2_DUAL_CORE
/* ── THE SECOND CORE ──────────────────────────────────────────────────────────
 *
 * The loop is the cartridges' (uvm2_core1.c): it replays each list, reads the controllers
 * into the caches, drains the PSG queue and paces the frame. From here on core 1 is the
 * ONLY one that touches the Vectrex bus — the board's handshake keeps state (the pin
 * directions) that two cores would trample.
 *
 * The baremetal start-up leaves core 1 parked, waiting for an address in its mailbox;
 * startCore1 (pi_support.c) hands it over and returns WITHOUT saying whether it was taken.
 * So the core says so itself, and a core that never arrives is reported instead of leaving
 * a black screen: the game would wait for its first frame for ever. */
void startCore1(unsigned int entry);                     /* libbaremetal, pi_support.c */

static void (*s_core1_entry)(void);
static volatile uint32_t s_core1_alive;
static volatile uint32_t s_core1_go;

static inline void tlb_flush(void);      /* below, with the PSRAM windows */

__attribute__((used)) void core1_run(void)
{
    tlb_flush();                     /* the PSRAM windows (psram_windows), on this core too */
    s_core1_cpuectlr = read_cpuectlr();
    s_core1_sctlr    = read_sctlr();
    s_core1_alive = 1u;
    PITREX_DMB();
    /* DIAGNOSIS: hold here, off the bus, until core 0 has written its report with this
     * core alive. If that report lands and the next one does not, it is the loop (the bus)
     * that breaks the card's writes, not the second core being awake. */
    while (!s_core1_go) { s_core1_beats++; }
    s_core1_entry();
    for (;;) { }
}

/* The park loop hands the core over with whatever stack the previous program left it (a
 * core parked from inside a game is still at that game's depth), so the entry starts from the
 * top of core 1's own stack, the one the kernel's start-up gave it (globalVars.h). */
__attribute__((naked)) static void core1_entry(void)
{
    __asm__ volatile("ldr sp, =" STR(__svc_stack_top1) "\n\t"
                     "b   core1_run\n\t"
                     ".ltorg");
}

#define CORE1_START_TIMEOUT_US 500000u    /* startCore1 itself gives up after 500 tries of 1 ms */

void multicore_launch_core1(void (*entry)(void))
{
    if (s_core1_alive) return;                           /* already in the loop */
    uvm2_pitrex_bus.core1_state_before = *(volatile uint32_t *)CORE1_STATE;
    s_core1_entry = entry;
    PITREX_DMB();
    /* Whoever parked core 1 — this SDK, which makes it forget (uvm2_bus_pitrex.c, forget_code),
     * or a program of the baremetal tree, which does not — it must not run a previous
     * program's code at this one's addresses: every core's instruction cache, branch predictor
     * and translations are dropped from here (the ...IS forms reach the other cores while they
     * are in the coherent group, CPUECTLR bit 6, which the report shows for both). */
    PITREX_FORGET_CODE_ALL_CORES();      /* ICIALLUIS, BPIALLIS, TLBIALLIS: uvm2_pitrex_cpu.h */
    PITREX_DSB();
    PITREX_ISB();
    startCore1((unsigned int)core1_entry);
    const uint32_t t0 = uvm2_now_us();
    while (!s_core1_alive && (uint32_t)(uvm2_now_us() - t0) < CORE1_START_TIMEOUT_US) { }
    uvm2_pitrex_bus.core1_state_after = *(volatile uint32_t *)CORE1_STATE;
    uvm2_pitrex_bus.core1_started = s_core1_alive;
}
#else
/* ── ONE CORE (PITREX_DUAL_CORE=0, to bisect) ─────────────────────────────────
 * What core 1 does between lists happens right after each list (uvm2_exec calls this), and
 * starting or stopping "core 1" is nothing. */
volatile uint8_t  uvm2_cached_buttons = 0xFFu;   /* raw, active LOW */
volatile uint32_t uvm2_cached_axes;

void uvm2_pitrex_between_lists(void)
{
    uvm2_cached_buttons = uvm2_read_buttons();
    uvm2_cached_axes    = uvm2_read_axes();
}

void uvm2_core1_start(void) { }
void uvm2_core1_stop(void)  { }
#endif

/* ── BACK TO THE MENU ─────────────────────────────────────────────────────────
 *
 * When the image was started by the PiTrex loader (the menu), the loader stays in memory
 * above LOADER_START and leaves its address in the kernel's globals. Holding the four buttons
 * of controller 1 goes back to it — the same gesture as in the programs built on the
 * baremetal tree's own library (vectrexInterface.c, "button 1+2+3+4 -> go menu") — held for
 * MENU_HOLD_US so that a game using all four does not leave by accident.
 *
 * Booted directly as the card's kernel there is no loader, and the gesture does nothing.
 *
 * uvm2_frame_end calls the poll once a frame, on the game's core. */
#define REPORT_AT_FRAME       600u     /* ten seconds at 60 Hz: past the start-up screens */
#define MENU_HOLD_US      1000000u
#define CORE1_PARK_US      500000u     /* a list is 20 ms; a frame drawn with nothing left out, 10x that */
#define LOADER_MAX_BYTES  1500000u     /* the bound the kernel's own getLoadParameter uses */
#define BUTTONS_PAD1         0x0Fu

extern volatile uint8_t  uvm2_cached_buttons;            /* raw, a zero bit is a button held */
extern volatile uint32_t uvm2_pitrex_park_request;       /* uvm2_bus_pitrex.c */

int uvm2_pitrex_has_menu(void)
{
    if (*(volatile int *)LOADER_AVAILABLE != 1) return 0;
    const uintptr_t st = (uintptr_t)*(GlobalMemSettings **)SETTING_STORE;
    return st > LOADER_START && st < LOADER_START + LOADER_MAX_BYTES;
}

/* Core 1 off the bus and back in the kernel's park loop, where the next program (the menu,
 * another game) expects to find it. 1 when it is there; 0 if it did not get there, and then
 * nothing has changed. */
int uvm2_pitrex_park_core1(void)
{
#ifdef UVM2_DUAL_CORE
    uvm2_pitrex_park_request = 1u;
    PITREX_DMB();
    const uint32_t t0 = uvm2_now_us();
    while (*(volatile uint32_t *)CORE1_STATE != CORE_PARKING) {
        if ((uint32_t)(uvm2_now_us() - t0) > CORE1_PARK_US) { uvm2_pitrex_park_request = 0u; return 0; }
    }
#endif
    return 1;
}

void uvm2_pitrex_to_menu(void)
{
    if (!uvm2_pitrex_has_menu()) { uvm2_pitrex_to_menu_refused = UVM2_TO_MENU_NO_MENU; return; }
    GlobalMemSettings *st = *(GlobalMemSettings **)SETTING_STORE;
    if (st->flags & FLAG_LOADER_RUNNING) { uvm2_pitrex_to_menu_refused = UVM2_TO_MENU_IS_MENU; return; }
    if (!uvm2_pitrex_park_core1()) { uvm2_pitrex_to_menu_refused = UVM2_TO_MENU_CORE1; return; }   /* the loader takes the bus */
    uvm2_pitrex_report();                              /* how the program was doing when it was left */
    {
        static const char name[] = UVM2_PITREX_NAME;
        volatile back_kept_t *b = BACK_KEPT;
        if (b->magic != BACK_KEPT_MAGIC) { b->magic = BACK_KEPT_MAGIC; b->count = 0; }
        b->count++;
        for (uint32_t i = 0; i < FAULT_KEPT_NAME; i++) b->name[i] = i < sizeof name ? name[i] : 0;
    }
    st->loader();
}

/* THE CONSOLE'S RESET BUTTON does the same. Core 1 sees it (uvm2_core1.c, reset_poll: the
 * VIA's DDRB reads 0) and leaves the word here. The reset has cleared the whole VIA, and the
 * 6809 stays halted through it — the board holds /HALT — so the bus is still ours; before the
 * menu takes it, the button has to be up again, which is when a data-direction register
 * keeps what is written to it (the baremetal tree's own test, v_resetDetection).
 *
 */
volatile uint32_t uvm2_pitrex_reset_seen;
#define RESET_RELEASE_US 5000000u     /* a finger on the button, at most; then go anyway */

static void reset_to_menu(void)
{
    uvm2_pitrex_reset_seen = 0u;
    if (!uvm2_pitrex_has_menu()) return;      /* booted as the card's kernel: nowhere to go */
    GlobalMemSettings *st = *(GlobalMemSettings **)SETTING_STORE;
    /* In the menu too: its VIA has been cleared like anyone's, and starting the menu again
     * (its own `loader` is itself, from the top) is what programs it back. */
    if (!uvm2_pitrex_park_core1()) return;
    const uint32_t t0 = uvm2_now_us();
    do {
        uvm2_via_write(UVM2_VIA_DDRA, 0xFFu);
    } while (uvm2_via_read(UVM2_VIA_DDRA) != 0xFFu && (uint32_t)(uvm2_now_us() - t0) < RESET_RELEASE_US);
    uvm2_pitrex_report();
    st->loader();
}

void uvm2_pitrex_menu_poll(void)
{
    static uint32_t held_since;       /* 0 = not held; else the time it started, never 0 */
    /* One report with the program running, once it has settled: where its frame goes. */
    if (++s_frames == REPORT_AT_FRAME) {
        uvm2_pitrex_report();
        FAULT_KEPT->magic = 0;            /* an earlier fault has been in five reports by now */
    }
    if (uvm2_pitrex_reset_seen) reset_to_menu();
    /* core 1 faulted and parked itself: say so and leave */
    if (uvm2_pitrex_fault[FAULT_KIND] && uvm2_pitrex_fault[FAULT_CORE] != 0u) uvm2_pitrex_fault_core0();
    if ((uvm2_cached_buttons & BUTTONS_PAD1) != 0u) { held_since = 0; return; }
    const uint32_t now = uvm2_now_us();
    if (!held_since) { held_since = now | 1u; return; }
    if ((uint32_t)(now - held_since) >= MENU_HOLD_US) { held_since = 0; uvm2_pitrex_to_menu(); }
}

/* ── THE CARTRIDGES' PSRAM, AS ADDRESSES ──────────────────────────────────────
 *
 * On the cartridges 8 MB of PSRAM sit at 0x11000000, and the same bytes again at 0x15000000
 * (the RP2350's uncached alias). Games that need the room use those addresses directly: ROM
 * tables written through one window and read through the other, sound banks, a film.
 *
 * A Pi has ordinary memory at both addresses — but as two DIFFERENT 8 MB, so what a game
 * wrote through one window it would not find through the other. The kernel's page table maps
 * memory 1 MB at a time, each megabyte onto itself; here the megabytes of the second window
 * are pointed at the first's instead. Same bytes at both addresses, as on the cartridge, and
 * no game has to know which board it is on.
 *
 * Every program that starts builds the table again, so this is done at each start. Core 1
 * shares the table and drops its own stale translations when it takes the loop. */
#define PSRAM_WINDOW        0x11000000u
#define PSRAM_ALIAS         0x15000000u
#define PSRAM_BYTES         0x00800000u
#define SECTION_BYTES       0x00100000u          /* the page table's unit: 1 MB */
#define SECTION_BASE_MASK   0xFFF00000u
#define PAGE_TABLE          ((volatile uint32_t *)0x4000u)      /* pi_support.c, page_table */

static inline void tlb_flush(void)
{
    __asm__ volatile("mcr p15, 0, %0, c8, c7, 0" : : "r"(0) : "memory");   /* TLBIALL */
    PITREX_DSB(); PITREX_ISB();
}

/* What a game calls before it uses that memory on a cartridge (uvm2_psram.h): bring the QSPI
 * pins up, wake the chip, open its window. Here the two windows are ordinary memory, mapped
 * by psram_windows() before the game starts: there is nothing to bring up, and the answer to
 * "is it there" is yes. */
int  uvm2_psram_ready = 1;            /* what the cartridge's start-up leaves after its bring-up */
void uvm2_qmi_bring_up(void) { }
int  uvm2_psram_init(void) { return 1; }
void uvm2_psram_enable_xip(void) { }

static void psram_windows(void)
{
    for (uint32_t off = 0; off < PSRAM_BYTES; off += SECTION_BYTES) {
        const uint32_t from = (PSRAM_WINDOW + off) / SECTION_BYTES;
        const uint32_t to   = (PSRAM_ALIAS  + off) / SECTION_BYTES;
        /* the first window's own entry (its attributes), with nothing but itself as target */
        PAGE_TABLE[to] = PAGE_TABLE[from];
    }
    PITREX_DSB();
    tlb_flush();
}

/* ── A PROGRAM THAT FAULTS SAYS SO ────────────────────────────────────────────
 *
 * A read of an address this machine does not have (a register of the RP2350 left in a port, a
 * wild pointer), a jump into data: the CPU takes an exception, and the baremetal kernel's
 * handlers print it on a serial port nobody has connected and stop. On the tube that is a
 * black screen that says nothing — and there is no probe here.
 *
 * So the three exceptions a bug raises go through here instead. What the CPU knows at that
 * moment is written down (uvm2_pitrex_fault), the report goes to the card with it — the pc is
 * an address in the program's ELF: `arm-none-eabi-addr2line -e build_pitrex/<name>.elf <pc>`
 * names the line — and the machine goes back to the menu, if there is one.
 *
 * The faulting program never resumes, so the handlers keep nothing: they note the registers,
 * leave the exception's mode for the supervisor's and start on that core's stack from the
 * top. A fault on core 1 (the list's replay) is noted, and that core goes to the park loop,
 * off the bus; the game's core reports it at its next frame close. */
#define FAULT_DATA       1u       /* a read or write of memory that is not there */
#define FAULT_PREFETCH   2u       /* a jump to memory that is not there */
#define FAULT_UNDEFINED  3u       /* executing what is not an instruction */

void uvm2_pitrex_fault_data(void);
void uvm2_pitrex_fault_prefetch(void);
void uvm2_pitrex_fault_undefined(void);
void parkCore1(void);

__asm__(
"    .text\n"
"    .arm\n"
"    .align 2\n"
"    .global uvm2_pitrex_fault_data\n"
"uvm2_pitrex_fault_data:\n"                  /* abort mode; lr is the instruction + 8 */
"    sub   lr, lr, #8\n"
"    ldr   r1, =uvm2_pitrex_fault\n"
"    mrc   p15, 0, r2, c6, c0, 0\n"          /* DFAR: the address it touched */
"    str   r2, [r1, #16]\n"
"    mrc   p15, 0, r2, c5, c0, 0\n"          /* DFSR: why */
"    str   r2, [r1, #20]\n"
"    mov   r2, #1\n"
"    b     1f\n"
"    .global uvm2_pitrex_fault_prefetch\n"
"uvm2_pitrex_fault_prefetch:\n"              /* abort mode; lr is the instruction + 4 */
"    sub   lr, lr, #4\n"
"    ldr   r1, =uvm2_pitrex_fault\n"
"    mrc   p15, 0, r2, c6, c0, 2\n"          /* IFAR */
"    str   r2, [r1, #16]\n"
"    mrc   p15, 0, r2, c5, c0, 1\n"          /* IFSR */
"    str   r2, [r1, #20]\n"
"    mov   r2, #2\n"
"    b     1f\n"
"    .global uvm2_pitrex_fault_undefined\n"
"uvm2_pitrex_fault_undefined:\n"             /* undefined mode; lr is the instruction + 4 */
"    sub   lr, lr, #4\n"
"    ldr   r1, =uvm2_pitrex_fault\n"
"    mov   r2, #0\n"
"    str   r2, [r1, #16]\n"
"    str   r2, [r1, #20]\n"
"    mov   r2, #3\n"
"1:  str   lr, [r1, #4]\n"                   /* where */
"    mrs   r3, spsr\n"
"    str   r3, [r1, #28]\n"
"    cpsid if, #0x13\n"                      /* the supervisor's mode: its lr and sp are the program's */
"    str   lr, [r1, #8]\n"
"    str   sp, [r1, #12]\n"
#ifdef PITREX_ARMV6
"    mov   r3, #0\n"                         /* the Zero's one core (it has no MPIDR) */
"    str   r3, [r1, #24]\n"
"    str   r2, [r1, #0]\n"                   /* the kind last: it is what says there is one */
"    ldr   sp, =" STR(__svc_stack_top0) "\n"
"    b     uvm2_pitrex_fault_core0\n"
#else
"    mrc   p15, 0, r3, c0, c0, 5\n"          /* which core */
"    and   r3, r3, #3\n"
"    str   r3, [r1, #24]\n"
"    str   r2, [r1, #0]\n"                   /* the kind last: it is what says there is one */
"    cmp   r3, #0\n"
"    ldreq sp, =" STR(__svc_stack_top0) "\n"
"    ldrne sp, =" STR(__svc_stack_top1) "\n"
"    beq   uvm2_pitrex_fault_core0\n"
"    b     parkCore1\n"                      /* core 1: off the bus, where the menu expects it */
#endif
"    .ltorg\n"
);

/* The words of the kernel's vector table (address 0) that hold the three handlers' addresses:
 * eight instructions, then eight addresses in the same order (baremetalEntry_7.S). */
#define VECTOR_ADDRESSES   0x20u
#define VECTOR_UNDEFINED   1u
#define VECTOR_PREFETCH    3u
#define VECTOR_DATA        4u

static void fault_handlers(void)
{
    volatile uint32_t *const at = (volatile uint32_t *)VECTOR_ADDRESSES;
    at[VECTOR_UNDEFINED] = (uint32_t)uvm2_pitrex_fault_undefined;
    at[VECTOR_PREFETCH]  = (uint32_t)uvm2_pitrex_fault_prefetch;
    at[VECTOR_DATA]      = (uint32_t)uvm2_pitrex_fault_data;
    PITREX_DSB(); PITREX_ISB();
}

/* Core 0 after a fault, on a fresh stack: the report, then the menu.
 *
 * ONE REPORT, HOWEVER MANY FAULTS. If writing the report faults, the handler lands here again;
 * reporting again would fault again, for ever, on a black screen. The first fault is already
 * kept outside the program (FAULT_KEPT); a second arrival skips the report and goes on to the
 * menu, which prints it. */
__attribute__((used)) void uvm2_pitrex_fault_core0(void)
{
    static uint32_t arrivals;
    volatile fault_kept_t *k = FAULT_KEPT;
    if (k->magic != FAULT_KEPT_MAGIC) {
        static const char name[] = UVM2_PITREX_NAME;
        for (uint32_t i = 0; i < FAULT_WORDS; i++) k->words[i] = uvm2_pitrex_fault[i];
        for (uint32_t i = 0; i < FAULT_KEPT_NAME; i++) k->name[i] = i < sizeof name ? name[i] : 0;
        k->again = 0;
        k->magic = FAULT_KEPT_MAGIC;
    } else {
        k->again++;
    }
    if (arrivals++ == 0u) uvm2_pitrex_report();
    if (uvm2_pitrex_has_menu() && uvm2_pitrex_park_core1()) {
        GlobalMemSettings *st = *(GlobalMemSettings **)SETTING_STORE;
        st->loader();
    }
    for (;;) { }
}

/* A resident image (uvm2_pitrex_loader.c) has things to do before the first report; a
 * program has not. */
__attribute__((weak)) void uvm2_pitrex_early(void) { }

/* WHAT THE MENU HANDED OVER, FOR THE LIBRARY'S OWN READER. The baremetal tree's programs ask
 * for their launch parameters with getLoadParameter (pi_support.c), which reads them through
 * the library's `settings` pointer — and that pointer is filled in by the tree's own start-up,
 * which is not the one that runs here. Left at 0 the reader takes the address of the slot for
 * the record itself, finds it outside the menu's image and answers "no parameters": the
 * Vectrex emulator, told to run Web Wars, showed its own file list instead, and then wrote
 * the button it was started with through the null it had been given — at address 0, the
 * exception vectors (2026-11-24). */
extern GlobalMemSettings *settings;
static void launch_parameters(void)
{
    if (uvm2_pitrex_has_menu()) settings = *(GlobalMemSettings **)SETTING_STORE;
}

/* A PROGRAM'S OWN FOLDER. The baremetal tree's programs open their files by bare name at the
 * card's top ("game.ym", "basic/"); on a tidy card those live in games/<program>/
 * (prune_card.py), and the program is started there. pitrex.mk sets the folder
 * (PITREX_GAME_DIR); a card that has not been laid out that way has no such folder, and the
 * program stays at the top, where its files are. The SDK's own paths start at the top
 * (UVM2_SD_ROOT, REPORT_FOLDER), so they are not moved by this. */
static void game_folder(void)
{
#ifdef UVM2_PITREX_GAME_DIR
    FILINFO info;
    if (f_stat(UVM2_PITREX_GAME_DIR, &info) == FR_OK && (info.fattrib & AM_DIR)) f_chdir(UVM2_PITREX_GAME_DIR);
#endif
}

__attribute__((weak)) void uvm2_pitrex_before_core1(void) { }

void uvm2_runtime_init(void)
{
    uvm2_pitrex_early();
    launch_parameters();
    game_folder();
    psram_windows();
    fault_handlers();
    /* The report BEFORE the first bus access, and again before the first frame: wherever
     * start-up stops, the card says how far it got and what the bus measured. */
    uvm2_pitrex_bus.stage = UVM2_PITREX_STAGE_BUS_INIT;
    uvm2_pitrex_report();
    uvm2_bus_init();
    uvm2_pitrex_report();
#ifdef UVM2_PITREX_PROBE
    /* The handshake probe has answered (hardware/pitrex/logs): it runs only when asked. */
    if (uvm2_pitrex_bus.init_result > 0) {
        uvm2_pitrex_probe_handshake();
        uvm2_pitrex_report();
    }
#endif
    if (uvm2_pitrex_bus.init_result <= 0) {
        /* No /HALT: the 6809 owns the bus and anything we wrote would fight it. */
        for (;;) { }
    }
    uvm2_draw_init();
    uvm2_input_set_analog(1);
    uvm2_stop_music();
    /* A frame open before the game's main, as the .um2's start-up leaves it: a port that draws
     * and then calls v_WaitRecal has never opened one itself. */
    uvm2_frame_begin();
    /* The romset, before the game, as on the .um2: a port's main() loads its ROMs first thing
     * from what this left in memory. A game with no romset: nothing happens. */
    uvm2_romzip_load();
    /* The last moment this core has the bus to itself: a program that wants it whole for a
     * while (the menu's start-up voice, written straight to the DAC) does it here. */
    uvm2_pitrex_before_core1();
#ifdef UVM2_DUAL_CORE
    /* Last: everything above wrote to the bus from this core, and after this only core 1
     * may. */
    uvm2_core1_start();
    uvm2_pitrex_report();                 /* core 1 awake, still off the bus */
    if (!uvm2_pitrex_bus.core1_started) {
        for (;;) { }          /* nobody would replay the lists: the report says so */
    }
    s_core1_go = 1u;
    PITREX_DMB();
    {
        const uint32_t t0 = uvm2_now_us();
        while ((uint32_t)(uvm2_now_us() - t0) < 200000u) { }   /* let it take the loop */
    }
    uvm2_pitrex_report();                 /* core 1 in the loop, on the bus */
    /* The calibration wizard if buttons 2+3 are held at start-up, as on the .um2. */
    uvm2_config_boot_combo();
    /* And the game's OPTIONS screen, if it declared one (UVM2_SETTING_OPTIONS), as on the
     * .um2: every launch with START IN MENU, button 4 held with START IN GAME. A resident image
     * is a menu already and declares none. */
    uvm2_options_boot();
#endif
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;
    uvm2_runtime_init();
    return uvm2_game_main();
}
