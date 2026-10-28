/* uvm2_core1.c — the second core, replaying frames while the game thinks.
 *
 * This is the split the multicart's own games have used from the start:
 *
 *     core 0   builds a command list into one of two buffers, publishes it
 *     core 1   replays it over the bus, reads the controls, ticks the audio
 *
 * WHAT IT BUYS, AND WHAT IT DOES NOT
 *
 * It does NOT make drawing faster. The replay is paced by the Vectrex's own
 * 1.5 MHz clock — 30000 bus cycles is a 50 Hz frame by definition — and no
 * amount of CPU makes a bus cycle shorter. dkong's stream measured 69718 cycles
 * (46.5 ms), and that number is untouched by this file.
 *
 * What it buys is the overlap. Single-core, a frame is
 *
 *     [ emulate the Z80 ][ build the list ][ replay 46.5 ms ][ pace ]
 *
 * strictly in series, so the game's own time is added on top of the beam's.
 * Split, the emulation for frame n+1 runs while the beam is still drawing frame
 * n, and the frame costs max(logic, replay) instead of their sum.
 *
 * WHO OWNS THE BUS
 *
 * Core 1, exclusively, from the moment it starts. That is the whole discipline
 * here and it is not negotiable: two cores driving the same GPIO would put two
 * writers on the VIA with no arbitration, which is the same fault we shipped on
 * the cartridge when 40 games claimed dual-core while compiled single.
 *
 * It works out cleanly because every bus contact on this target was already
 * funnelled into SYS_WAIT_RECAL: the replay, the button/axis reads and the audio
 * tick. Those move here wholesale. The game keeps reading buttons and axes
 * through the syscalls, which have always answered from a cache rather than from
 * the wire, so nothing on the game side changes.
 *
 * The two exceptions are the syscalls that write the bus at an arbitrary moment:
 * SYS_PSG_WRITE goes through a small queue drained here at the frame boundary,
 * and the raw SYS_BUS_READ/WRITE pair is refused (see uvm2_svc.c) because it has
 * no meaning while another core owns the pins.
 */

#include "uvm2_bus.h"
#ifdef UVM2_CORE0_WORKER
#include "uvm2_bus_stream.h"
#endif

extern uint32_t uvm2_sig_written[2], uvm2_sig_n[2];

/* Disagreements between what core 0 wrote and what core 1 reads out of the SAME buffer. If
 * this does not climb, cross-core coherence is ruled out and the fault is one of timing. */
uint32_t uvm2_sig_failures, uvm2_sig_compared, uvm2_sig_read, uvm2_sig_last_ok;
#include "uvm2_draw.h"
#include "uvm2_input.h"
#include "uvm2_audio.h"

#ifdef UVM2_DUAL_CORE

#include "pico/multicore.h"
#include "pico/time.h"
#ifdef UVM2_BIOS
#define INPUT_HELD() 0
#endif
#ifndef UVM2_BIOS
#include "pico/bootrom.h"
#include "boot/picoboot_constants.h"
#endif

/* Published by uvm2_draw.c. `request` counts frames core 0 has finished
 * building, `done` frames core 1 has finished replaying; the buffer for frame n
 * is n & 1. Both are plain counters that only ever increase, so a torn read is
 * impossible on a 32-bit load and no lock is needed — the ordering is carried by
 * the barrier core 0 issues before publishing. */
extern volatile uint32_t uvm2_frame_request;
extern volatile uint32_t uvm2_frame_done;
extern const uint8_t    *uvm2_frame_buffer(uint32_t frame);
extern uint32_t          uvm2_frame_length(uint32_t frame);

/* THE FRAME PERIOD, MEASURED, so the ratio above is computed against something real. The
 * pacer fills it in when it closes each pass, so the ratio uses the PREVIOUS frame's period
 * — one frame of lag in a diagnostic changes no decision, and measuring it before the frame
 * has ended is impossible. */
static uint32_t s_cycle_us;

#ifdef UVM2_REPLAY_LAST
/* A REDRAW MUST NOT RUN WHILE THE PROGRAM IS INSIDE THE BIOS. On our own cartridge the
 * program's core reads the SD card inside some calls (SYS_LAUNCH loads the game, its ROM
 * and its samples), and a card read while this core drives the cartridge bus comes back
 * as garbage: the games loaded corrupt and died at their first call into the BIOS
 * (dkong, SM64, 2026-10-02). Before UVM2_REPLAY_LAST this core was idle then, because a
 * program inside a call publishes no list. So the BIOS raises uvm2_replay_hold around
 * every call and waits for uvm2_replaying to drop; a redraw starts only with the hold
 * down, and announces itself before looking (with a barrier on each side, so one of the
 * two always sees the other). */
volatile uint32_t uvm2_replay_hold;
volatile uint32_t uvm2_replaying;
#endif

/* PSG writes the game issued mid-frame, drained between frames.
 *
 * A ring rather than a flag per register: a game that writes the same register
 * twice in a frame means both writes, and .vmus playback does exactly that on
 * the volume registers. Single producer (core 0), single consumer (core 1), so
 * head and tail need no lock. Overflow drops the OLDEST rather than the newest,
 * because for a PSG the newest write is the current state and the one that must
 * survive. */
#define PSG_QUEUE_LEN 64u
static volatile uint32_t s_psg_reg[PSG_QUEUE_LEN];
static volatile uint32_t s_psg_val[PSG_QUEUE_LEN];
static volatile uint32_t s_psg_head;      /* written by core 0 */
static volatile uint32_t s_psg_tail;      /* written by core 1 */

void uvm2_psg_queue(uint32_t reg, uint32_t value)
{
    uint32_t h = s_psg_head;
    /* Register 7 is the mixer, and the sample injector composes its own channel over
     * whatever it believes the mixer to be. Tell it, or a game that drives the PSG
     * itself gets its channels silenced once a frame -- see uvm2_audio_note_mixer.
     * Noted HERE, on the core that builds the list, because that is the core that reads
     * it back through uvm2_smp_mixer. */
    if (reg == 7u) { extern void uvm2_smp_note_mixer(uint8_t);
                     uvm2_smp_note_mixer((uint8_t)value); }
    s_psg_reg[h % PSG_QUEUE_LEN] = reg;
    s_psg_val[h % PSG_QUEUE_LEN] = value;
    __asm volatile ("dmb" ::: "memory");  /* the data before the index */
    s_psg_head = h + 1;
}

static void psg_drain(void)
{
    uint32_t h = s_psg_head;
    uint32_t t = s_psg_tail;

    /* If the producer ran away, skip forward and keep only the newest entries
     * the ring still holds — the older ones have already been overwritten. */
    if (h - t > PSG_QUEUE_LEN) t = h - PSG_QUEUE_LEN;

    __asm volatile ("dmb" ::: "memory");
    while (t != h) {
        uvm2_psg_write(s_psg_reg[t % PSG_QUEUE_LEN], s_psg_val[t % PSG_QUEUE_LEN]);
        t++;
    }
    s_psg_tail = t;
}

/* Cached controls, served to the game by SYS_READ_BUTTONS / SYS_READ_AXES.
 * Defined here in dual-core builds because this is the core that fills them.
 *
 * RELEASED IS 0xFF, AND THE INITIAL VALUE IS NOT COSMETIC. These are active low —
 * SYS_READ_BUTTONS answers `~uvm2_cached_buttons`, so a zero bit means PRESSED. Left in
 * .bss they start at 0, i.e. all four buttons held, and every game that waits for a
 * button starts by itself before core 1 gets to fill the cache at the end of frame one.
 * Single-core never showed it because core 0 fills s_buttons inside the same syscall path
 * the game polls from; dual-core fills it asynchronously, so the game can read first.
 * Seen on SnowBros 2026-08-25, the first VPy game to actually launch core 1. */
volatile uint8_t  uvm2_cached_buttons = 0xFFu;

#ifndef UVM2_BIOS
/* THE CONSOLE'S RESET BUTTON: ONE PRESS STARTS THE GAME AGAIN, TWO IN A ROW (within
 * UVM2_RESET_DOUBLE_US) GO BACK TO THE UVMC2's MENU.
 *
 * The reset button is not wired to the cartridge (the UVMC2's schematic: its one button, SW1,
 * is the RP2350's BOOTSEL, and /RUN has only a pull-up). What it does reach is the VIA: the
 * console's reset puts the 6522's registers back to zero, DDRB among them -- and every frame's
 * list sets DDRB to 0x9F at its start (uvm2_draw.c, via_setup). So DDRB reading 0 between
 * frames says the VIA was reset since: the button. That is Ralf's own test (2026-10-15: "I am
 * just checking if DDRB is not zero. Reset sets it back to zero and I set it to the correct
 * value at the begin of each frame"); the cartridge then reboots into the firmware with
 * rom_reboot(BOOT_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0).
 *
 * PRESSES, NOT A HOLD. The button gives the VIA a pulse, not a reset held while it is down:
 * measured on the UVMC2 (2026-10-15), a short press and one held 3 s each read DDRB = 0 for
 * one frame and once, the next frame's list having set it again; letting go reads nothing.
 * So how long it was held cannot be told -- the old scheme (held 3 s: the menu; 1 to 3 s and
 * let go: the game again) could never work -- and the two are told apart by counting presses.
 * After one, the game waits UVM2_RESET_DOUBLE_US for a second before it starts again.
 *
 * The first version watched T1 instead (Ralf's old trick: a VIA in reset stops its timers),
 * reading its high byte twice between frames, and saw presses nobody made: on the UVMC2 it
 * restarted dkong, the playroom and Major Havoc with nobody near the button. DDRB cannot
 * come back to 0 by itself: 0 false presses in thousands of frames, measured.
 *
 * Between frames (CLAUDE.md invariant 7: reads happen between frames), core 1 reads DDRB once.
 * `uvm2_reset_seen` counts the presses, `uvm2_reset_polls` proves the watch runs, and
 * UVM2_RESET_WATCH_ONLY makes it count and do nothing (the SD bridge's build, which acts on
 * the count itself).
 *
 * ON IN EVERY DUAL-CORE GAME since v1.4.0 (UVM2_RESET_WATCH, uvm2_bus.h); it was opt-in
 * (-DUVM2_RESET_BUTTON) while the T1 test misfired. UVM2_NO_RESET_BUTTON turns it off. Off, it
 * costs nothing: no bus read, no warm-start mark, no copy of .data. */
#define UVM2_RESET_DOUBLE_US 1000000u   /* a second press within this: the menu */
#define UVM2_RESET_BOUNCE_US  150000u   /* and not before this: one press bouncing */
volatile uint32_t uvm2_reset_seen, uvm2_reset_polls;   /* presses noticed, and reads made */
/* No longer kept (a press has no length to measure, above); defined for what reads them. */
volatile uint32_t uvm2_reset_held_us, uvm2_reset_held_max_us;
volatile uint8_t  uvm2_reset_last_t1;
/* THE CONTROLLERS ARE NOT READ WHILE THE VIA IS IN RESET, nor for UVM2_RESET_SETTLE frames
 * after: a VIA in reset reads back zeros, which on the active-low buttons is "all pressed".
 * Reported on the UVMC2 2026-10-03: a short press of the reset button dropped a crate and
 * punched the ball in soft_demo — buttons 3 and 2, seen down for a frame or two. The game
 * keeps the last good reading instead; the frame after, the list has programmed the VIA
 * again. */
#define UVM2_RESET_SETTLE 2u
static uint32_t s_input_hold;
#define INPUT_HELD() (s_input_hold != 0u)

/* THE GAME, STARTED AGAIN, without the menu (one press). The image is still in SRAM, so the
 * bootrom is asked to boot it again from there (BOOT_TYPE_RAM_IMAGE, the way the launcher
 * started it). What would not be clean is .data: a RAM image keeps its initial values in
 * place, so whatever the game changed would start the next run. So .data is copied aside as
 * the game starts and put back just before the reboot; .bss and the stacks are cleared by the
 * start-up itself.
 *
 * THE COPY IS IN THE PSRAM (UVM2_RESTART_PSRAM, the monitor's window, between its image and
 * the RTT ring): a big game has no SRAM for it. It is taken FIRST in main, before anything
 * writes a global and before the PSRAM is up, so it goes to core 1's stack first (core 1
 * has not started; UVM2_RESTART_SNAP bytes at most) and on to the PSRAM once it answers
 * (uvm2_restart_keep). No PSRAM, or a bigger .data: one press is the menu, as two are.
 *
 * Put back by the GAME's core (uvm2_frame_end, uvm2_restart_if_asked), not by core 1, which
 * sees the press: the game must not be writing its globals while they are copied. */
#if UVM2_RESET_WATCH
#define UVM2_RESTART_SNAP   2048u       /* bytes of .data that can be put back (core 1's stack) */
#define UVM2_RESTART_PSRAM  ((volatile uint8_t *)0x157E0000u)
#define UVM2_IMAGE_BASE     0x20000000u /* where the launcher loads a .um2 (uvm2_game.ld) */
#define UVM2_IMAGE_WINDOW   (496u * 1024u)
extern uint8_t __data_start__[], __data_end__[];
extern uint32_t __StackOneBottom[];
extern int uvm2_psram_ready;
static uint32_t s_snap_n;               /* bytes copied to core 1's stack (0: none) */
static int      s_snap_ok;              /* and kept in the PSRAM */
volatile uint32_t uvm2_restart_refused;
volatile uint32_t uvm2_restart_asked;   /* core 1 saw one press and its wait ran out */
/* ONE PRESS SEEN AND THE WAIT FOR A SECOND STILL RUNNING. The game went on being drawn for
 * that second, so a reset looked like a flicker and then the game carrying on — and then,
 * out of nowhere, starting again (kuroishi, 2026-10-28). While this is set, uvm2_frame_end
 * throws the game's frame away and shows RESETTING instead. */
volatile uint32_t uvm2_restart_pending;
__attribute__((optimize("Os"))) void uvm2_restart_snapshot(void)
{
#ifdef UVM2_RESET_WATCH_ONLY
    return;   /* it only counts presses (the SD bridge): nothing restarts */
#endif
    const uint32_t n = (uint32_t)(__data_end__ - __data_start__);
    if (n > UVM2_RESTART_SNAP) { uvm2_restart_refused++; return; }
    uint8_t *t = (uint8_t *)__StackOneBottom;
    for (uint32_t i = 0; i < n; i++) t[i] = __data_start__[i];
    s_snap_n = n;
}
__attribute__((optimize("Os"))) void uvm2_restart_keep(void)
{
    if (!s_snap_n || uvm2_psram_ready != 1) return;
    const uint8_t *t = (const uint8_t *)__StackOneBottom;
    for (uint32_t i = 0; i < s_snap_n; i++) UVM2_RESTART_PSRAM[i] = t[i];
    s_snap_ok = 1;
}
__attribute__((optimize("Os"))) void uvm2_restart_if_asked(void)
{
    if (!uvm2_restart_asked) return;
    __asm volatile ("cpsid i" ::: "memory");      /* nothing runs on what is being put back */
    for (uint32_t i = 0; i < s_snap_n; i++) __data_start__[i] = UVM2_RESTART_PSRAM[i];
    rom_reboot(BOOT_TYPE_RAM_IMAGE | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, UVM2_IMAGE_BASE, UVM2_IMAGE_WINDOW);
}
#else
volatile uint32_t uvm2_restart_refused;
void uvm2_restart_snapshot(void) { }
void uvm2_restart_keep(void) { }
void uvm2_restart_if_asked(void) { }
#endif
/* THE VECTREX'S WARM-START MARK. On a reset its BIOS checks Vec_Cold_Flag at $CBFE: $7321
 * there means warm — no logo, straight to the cartridge, which is how the UVMC2's firmware
 * comes back to its menu ("loading", then the menu) after a game of Ralf's. Without it the
 * console cold-starts: logo first. Reported on the UVMC2 2026-10-03 with this SDK: the
 * reboot worked but went through the full start. Written ONCE, as core 1 starts, with the
 * 6809 halted and the bus ours; nothing in the SDK writes the console's RAM afterwards. Not
 * at the moment of the reboot: with the reset button down the 6809 may be driving the bus. */
#define VEC_COLD_FLAG_ADDR 0xCBFEu
#define VEC_COLD_FLAG_WARM 0x7321u
/* Once a frame on core 1: small rather than fast (it reads one register), so a big game's
 * SRAM pays as little as it can for it (ROTJ with the monitor is at its edge). */
__attribute__((optimize("Os"))) static void reset_poll(void)
{
#if !UVM2_RESET_WATCH
    return;     /* not watched: see UVM2_RESET_WATCH */
#else
    static uint32_t first_at;
    static uint8_t down, pending;
    /* Not before a frame has been drawn: until a list has set it, DDRB is 0 from the start-up
     * (seen on the UVMC2: two "presses" at every boot). */
    if (uvm2_frame_done == 0u) return;
    uvm2_reset_polls++;
    if (uvm2_via_read(UVM2_VIA_DDRB) == 0) {   /* 0x9F as the frame left it, 0 if reset */
        s_input_hold = UVM2_RESET_SETTLE;
        if (!down) {
            down = 1;
            uvm2_reset_seen++;
#ifndef UVM2_RESET_WATCH_ONLY
            const uint32_t now = time_us_32();
            /* a second pulse this soon is the same press bouncing, not a second press */
            if (pending && now - first_at < UVM2_RESET_BOUNCE_US) return;
            /* the second press within the wait, or a game that cannot start again: the menu */
            if ((pending && now - first_at < UVM2_RESET_DOUBLE_US) || !s_snap_ok)
                rom_reboot(BOOT_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);
            pending = 1;
            uvm2_restart_pending = 1;
            first_at = now;
#endif
        }
    } else {
        if (s_input_hold) s_input_hold--;
        down = 0;
        /* one press and the wait ran out: the game's core starts it again (uvm2_frame_end) */
        if (pending && time_us_32() - first_at >= UVM2_RESET_DOUBLE_US) { pending = 0; uvm2_restart_asked = 1; }
    }
#endif
}

/* THE STACK PAINT (see stack0_peak in uvm2_bus.h). The pico-sdk memory map puts core 0's stack
 * in SCRATCH_Y, from __StackTop down to __StackOneTop, and core 1's below it, from
 * __StackOneTop down to __StackOneBottom. A word that still holds the paint has never been
 * stack; the lowest word that does not is the deepest the stack has been. */
extern uint32_t __StackOneBottom[], __StackOneTop[], __StackTop[];
#define STACK_PAINT 0x57AC57ACu
#define STACK_CHECK_FRAMES 50           /* once a second at 50 Hz: a scan is ~1500 loads */
/* Core 0, before core 1 starts: its whole stack (it is not running yet) and core 0's own,
 * up to a margin below where core 0 is now. */
void uvm2_stack_paint(void)
{
    uint32_t sp;
    __asm volatile ("mov %0, sp" : "=r"(sp));
    for (uint32_t *p = __StackOneTop; p < (uint32_t *)(sp - 256u); p++) *p = STACK_PAINT;
    for (uint32_t *p = __StackOneBottom; p < __StackOneTop; p++) *p = STACK_PAINT;
}
static uint32_t stack_peak(const uint32_t *floor, const uint32_t *top)
{
    const uint32_t *p = floor;
    while (p < top && *p == STACK_PAINT) p++;
    return (uint32_t)(top - p) * 4u;
}
static void stack_check(void)
{
    uvm2_stats.stack0_peak = stack_peak(__StackOneTop, __StackTop);
    uvm2_stats.stack1_peak = stack_peak(__StackOneBottom, __StackOneTop);
    if (__StackOneTop[0]    != STACK_PAINT) uvm2_stats.stack_overflow |= 1u;
    if (__StackOneBottom[0] != STACK_PAINT) uvm2_stats.stack_overflow |= 2u;
}
#endif
volatile uint32_t uvm2_cached_axes;

/* Vectrex time not yet handed to the sequencer, in bus cycles. */
static uint32_t s_audio_acc;

void uvm2_core1_gap(void);   /* per-pass hook; weak, below */
/* After each redraw of the last list (UVM2_REPLAY_LAST): a BIOS may draw over it -- the
 * Vectrex Studio cartridge's pause sign while a debugger holds the program. Weak, absent
 * unless someone defines it. */
extern void uvm2_replay_extra(void) __attribute__((weak));

#ifdef UVM2_CORE0_WORKER
/* THE GAME'S WORK, ON THE CORE THAT REPLAYS (our cartridge's BIOS: API set_worker).
 *
 * Replaying a list used to keep this core busy for the whole pass: uvm2_exec pushes the
 * words in small batches and each batch waits for the previous DMA, so ~30 ms of a 30 ms
 * pass went into a wait loop, measured on Virtua Fighter (exec_acc = 97% of the time).
 * With a worker registered, the pass goes out the way the single-core path already sends
 * it: the whole list accumulated, ONE DMA fired, and this core is free until the bus is
 * done. The worker runs in that window — VF renders its next display list here while core
 * 1 emulates the next frame, instead of the two adding up on core 1.
 *
 * The worker may build and publish a list (the draw calls and wait_recal of the table):
 * the deferred frame opens at its first draw and waits for frame_done >= its number - 2,
 * which the pass being replayed already satisfies, as long as it publishes at most one
 * list per call. It is not called while the program is inside a BIOS call (the hold). */
void (*volatile uvm2_worker)(void);

/* THE PASS IN CHUNKS, NOT AS ONE LIST (UVM2_WORKER_STREAM, 2026-10-20, Starblade).
 *
 * The worker needs this core free while the beam draws, and the list path gives that by
 * holding the WHOLE pass as bus words: Starblade's pass is ~12k words (~6000 commands, a
 * write each plus a park after most), 47 KB a buffer and two of them, in an image with
 * a few hundred bytes of SRAM left. And the list does not go to the PSRAM (it always gave
 * trouble).
 *
 * But the words are a pure function of the commands, which are ALREADY in SRAM (s_cmds):
 * one 3-byte command is a write word and, when it has a delay, a park word. So the pass is
 * converted a chunk at a time into two small buffers (512 bytes in all), each read by a DMA
 * channel chained to the other: when a chunk ends the next one starts in hardware, and the
 * end-of-transfer interrupt refills the buffer that just finished while the other plays
 * (~0.17 ms). Converting a chunk takes a few us (Starblade: 48k bus cycles in 12k words).
 *
 * WHILE THE WORKER RUNS, THE PASS LOOPS: at the end of the commands it starts over. A
 * worker that outlasts the beam (Starblade's drawing passes: 70 ms against a 32 ms pass)
 * would otherwise leave the beam off until it returned, the flicker UVM2_REPLAY_LAST was
 * written to remove. Starting over is what the replay does, minus the controllers and the
 * PSG between passes, which wait for the worker. When it returns the loop is told to stop
 * at the end of the commands, and the pass ends there as before.
 *
 * A LATE REFILL REPEATS A CHUNK, IT NEVER SENDS GARBAGE: each channel reads its buffer as a
 * ring. The PIO's own stall flag counts what the bus waited for (uvm2_stream_starved; the UVM2
 * emulator does not model it and reads 0). The first version, one channel re-armed from the
 * interrupt, had only the FIFO's 4 words of slack and stalled ~11 times a second on the
 * console (2026-10-24): see s_chunk.
 *
 * The pump is also safe to call by polling (uvm2_stream_pump); stream_end does, as a backstop.
 * The UVM2 emulator models this interrupt since 2026-10-21 (DMA_IRQ_1, core 1); before that
 * the stream only moved when polled and the beam starved between polls. */
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#ifndef UVM2_STREAM_CHUNK_LOG2
#define UVM2_STREAM_CHUNK_LOG2 6u   /* 64 words: 256 bytes a buffer, ~0.17 ms of bus each */
#endif
#define UVM2_STREAM_CHUNK (1u << UVM2_STREAM_CHUNK_LOG2)
/* TWO CHANNELS, CHAINED (2026-10-24, the first console run). With one channel re-armed from its
 * own interrupt, the re-arm had what the PIO's FIFO holds, 4 words (~2.7 us), and on the
 * console it missed ~11 times a second (uvm2_stream_starved, the PIO's TXSTALL): core 1
 * stalled on a PSRAM miss while core 0 competes for the same QMI, and a stall with the ramp
 * open is a glitch on the screen. The UVM2 emulator charges no PSRAM time and never saw it.
 * Now channel A chains to B and B to A, so the next chunk starts in hardware the moment the
 * previous one ends; the interrupt only refills the buffer that just finished, and has a whole
 * chunk (~0.17 ms) to do it in. Each channel reads its buffer as a RING (the buffer aligned to
 * its size): a refill late even past that would replay a stale chunk of real bus words, never
 * memory beyond it. At the end of the commands the running channel's chain is cut. */
static uint32_t s_chunk[2][UVM2_STREAM_CHUNK] __attribute__((aligned(UVM2_STREAM_CHUNK * 4u)));
static uint32_t s_chunk_n[2];
static int      s_ch[2] = { 0, -1 };   /* A is vectrex-bus's channel (PIO0 SM0's TX); B claimed */
static const uint8_t *s_sc_cmds;    /* the pass's commands */
static uint32_t s_sc_count, s_sc_at; /* how many, the next one to convert */
static uint32_t s_sc_cycles;        /* bus cycles of one pass of the commands */
static uint32_t s_sc_park;          /* the park of the last word emitted: what the bus may still
                                       be doing when the FIFO is already empty */
static volatile int s_sc_last;      /* the buffer whose chunk ends the pass (its chain cut), -1 */
static int      s_sc_first;         /* the pass's first chunk is still the one to end */
static volatile int s_sc_loop;      /* 1: start over at the end of the commands */
static volatile int s_sc_active;    /* a chunk is in flight */
uint32_t uvm2_stream_chunks, uvm2_stream_starved, uvm2_stream_loops;
/* passes started, and the microseconds with no pass going out: from the end of one (its last
 * chunk sent) to the start of the next. What the beam spends without a list. */
uint32_t uvm2_stream_passes, uvm2_stream_gap_us, uvm2_stream_end_us, uvm2_stream_last_park;
static uint32_t s_sc_end_us;

/* the next commands as words into b; 0 words when the commands are done and not looping */
static uint32_t stream_fill(uint32_t *b)
{
    uint32_t n = 0;
    while (n + 2u <= UVM2_STREAM_CHUNK) {
        if (s_sc_at >= s_sc_count) {
            if (!s_sc_loop || n) break;   /* a loop starts in a buffer of its own */
            s_sc_at = 0;
            uvm2_stream_loops++;
        }
        const uint8_t *c = s_sc_cmds + 3u * s_sc_at++;
        uint32_t v     = (uint32_t)c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16);
        uint32_t delay = UVM2_CMD_DELAY(v);
        b[n++] = vbus_word(UVM2_VIA_WORD(UVM2_CMD_REG_DATA(v)));
        s_sc_park = delay;
        if (delay) b[n++] = vbus_repeat(delay);
    }
    return n;
}

/* buffer k's channel, reading its buffer as a ring, chained to the other (or to itself: no
 * chain); triggered only to start the pass */
static void stream_arm(int k, int chain, bool trigger)
{
    const uint ch = (uint)s_ch[k];
    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, UVM2_STREAM_CHUNK_LOG2 + 2u);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, 0, true));
    channel_config_set_chain_to(&c, chain ? (uint)s_ch[k ^ 1] : ch);
    dma_channel_configure(ch, &c, &pio0->txf[0], s_chunk[k], s_chunk_n[k], trigger);
}

/* buffer k's chunk has gone out (its channel finished, the other one already running it its
 * place): refill it, and arm it again for when the other finishes, or end the pass */
static void stream_done(int k)
{
    /* STARVED = the state machine stalled on an empty FIFO (FDEBUG.TXSTALL, sticky). Not
     * "the FIFO is empty": a park word keeps the bus busy for its whole length with the FIFO
     * already empty, and counting that read 119 starvations in a run with none. */
    /* NOT AT THE FIRST: the state machine sat stalled waiting for the pass, and the flag,
     * cleared when the pass was armed, set itself again before the first word arrived (a
     * stall a pass, or nearly: 22 in 141 passes on the console). From the first chunk's end
     * on, a stall is the bus really waiting. */
    if (s_sc_first) s_sc_first = 0;
    else if (pio0->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + 0u))) uvm2_stream_starved++;
    pio0->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + 0u);
    uvm2_stream_chunks++;
    if (s_sc_last == k) { s_sc_active = 0; s_sc_end_us = time_us_32(); return; }   /* the pass is over */
    if (s_sc_last >= 0) return;          /* the other one ends it: nothing more to send */
    s_chunk_n[k] = stream_fill(s_chunk[k]);
    if (s_chunk_n[k]) { stream_arm(k, 1, false); return; }
    /* nothing after the chunk now running: cut its chain (to itself), in ONE write: cleared
     * first and set after, it would point at channel 0 in between */
    s_sc_last = k ^ 1;
    {   io_rw_32 *ctrl = &dma_channel_hw_addr((uint)s_ch[k ^ 1])->al1_ctrl;
        *ctrl = (*ctrl & ~DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) |
                ((uint32_t)s_ch[k ^ 1] << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB); }
}

static void stream_service(void)
{
    for (int i = 0; i < 2; i++) {
        /* the one that started first ends first: A's flag, then B's, in the order they ran
         * is not guaranteed here, so each is taken as it is seen; at most one is pending
         * unless a whole chunk was missed */
        const uint32_t bit = 1u << s_ch[i];
        if (dma_hw->ints1 & bit) { dma_hw->ints1 = bit; if (s_sc_active) stream_done(i); }
    }
}

static void __not_in_flash_func(stream_isr)(void) { stream_service(); }

/* By polling, as a backstop: interrupts off so it never runs inside the handler. */
void uvm2_stream_pump(void)
{
    if (!s_sc_active) return;
    const uint32_t save = save_and_disable_interrupts();
    stream_service();
    restore_interrupts(save);
}

void uvm2_bus_return_to_stream(void);   /* uvm2_bus.c: the bus back from SIO to the PIO */
static void stream_begin(const uint8_t *cmds, uint32_t count)
{
    static int s_irq_on;
    if (!s_irq_on) {   /* on this core: the handler runs where the worker does. EXCLUSIVE: the
                        * shared-handler chain is ~0.8 KB of SRAM, and nothing else in the
                        * SDK takes DMA_IRQ_1 (the jack's channel runs without interrupts) */
        /* B: the lowest free channel above A. Not dma_claim_unused_channel: it would hand
         * out channel 0, which vectrex-bus drives without claiming */
        for (uint ch = 1; ch < NUM_DMA_CHANNELS; ch++)
            if (!dma_channel_is_claimed(ch)) { dma_channel_claim(ch); s_ch[1] = (int)ch; break; }
        if (!dma_channel_is_claimed(0)) dma_channel_claim(0);
        irq_set_exclusive_handler(DMA_IRQ_1, stream_isr);
        irq_set_priority(DMA_IRQ_1, 0);   /* the highest: its deadline is the bus's */
        irq_set_enabled(DMA_IRQ_1, true);
        s_irq_on = 1;
    }
    uvm2_bus_return_to_stream();
    vbus_flush();                                   /* whatever single accesses left queued */
    while (dma_channel_is_busy((uint)s_ch[0]) || dma_channel_is_busy((uint)s_ch[1])) { }
    s_sc_cmds = cmds; s_sc_count = count; s_sc_at = 0; s_sc_park = 0;
    s_sc_cycles = 0;
    if (!count) return;                             /* an empty list would loop on nothing */
    for (uint32_t i = 0; i < count; i++) {          /* what one pass costs, for the stats */
        const uint8_t *c = cmds + 3u * i;
        s_sc_cycles += 1u + UVM2_CMD_DELAY((uint32_t)c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16));
    }
    s_sc_loop = 1;
    s_chunk_n[0] = stream_fill(s_chunk[0]);
    s_chunk_n[1] = stream_fill(s_chunk[1]);
    if (!s_chunk_n[0]) return;
    s_sc_last = s_chunk_n[1] ? -1 : 0;
    if (s_chunk_n[1]) stream_arm(1, 1, false);
    if (s_sc_end_us) uvm2_stream_gap_us += time_us_32() - s_sc_end_us;
    uvm2_stream_passes++;
    s_sc_active = 1;
    s_sc_first = 1;
    dma_hw->ints1 = (1u << s_ch[0]) | (1u << s_ch[1]);
    hw_set_bits(&dma_hw->inte1, (1u << s_ch[0]) | (1u << s_ch[1]));
    stream_arm(0, s_chunk_n[1] != 0, true);         /* the pass starts */
}

/* FOR A WORKER THAT DOES NOT RETURN AT ONCE (Starblade's drawing, a coroutine): it publishes
 * its list, asks the pass to end at the end of its commands instead of looping, and goes on
 * with its next frame until it sees the pass over; then it returns, and the loop serves the
 * new list without having waited the rest of the pass out. */
int  uvm2_stream_running(void) { return s_sc_active; }
uint32_t uvm2_stream_ended_us(void) { return s_sc_end_us; }
void uvm2_stream_finish(void)  { s_sc_loop = 0; }

/* stop at the end of the commands, wait for the bus to be done with all of it */
static uint32_t stream_end(void)
{
    const uint32_t t0 = time_us_32();
    s_sc_loop = 0;
    while (s_sc_active) uvm2_stream_pump();
    hw_clear_bits(&dma_hw->inte1, (1u << s_ch[0]) | (1u << s_ch[1]));
    while (!((pio0->fstat >> PIO_FSTAT_TXEMPTY_LSB) & 1u)) { }
    /* AN EMPTY FIFO IS NOT A FREE BUS: the last word may be a park still running. Its
     * periods, plus two, counted on E (SIO bit 31) as vectrex-bus's drain does. The LAST
     * word's, not the longest of the pass (vectrex-bus's PARK_MAX is per drain, i.e. per
     * batch): a long park in the middle of the list would be waited out at every end. */
    for (uint32_t i = 0; i < s_sc_park + 2u; i++) {
        uint32_t g = 0;
        while (sio_hw->gpio_in & (1u << 31)) { if (++g > 100000u) break; }
        g = 0;
        while (!(sio_hw->gpio_in & (1u << 31))) { if (++g > 100000u) break; }
    }
    uvm2_stream_last_park = s_sc_park;
    uvm2_stream_end_us += time_us_32() - t0;
    return s_sc_cycles;
}
#endif
static void core1_main(void)
{
    uint32_t served = 0;
#if !defined(UVM2_BIOS) && UVM2_RESET_WATCH   /* only the watch's reboot needs it */
    uvm2_mem_write(VEC_COLD_FLAG_ADDR,      VEC_COLD_FLAG_WARM >> 8);   /* 6809: big-endian */
    uvm2_mem_write(VEC_COLD_FLAG_ADDR + 1u, VEC_COLD_FLAG_WARM & 0xFFu);
#endif

    for (;;) {
        /* REALLY TIMED, in microseconds. We got here after two rounds of guessing where
         * ~17 ms per frame that were not the beam came from: the audio was blamed and
         * measured not to be it. A clock costs less than a hypothesis. */
        uint32_t t_w0 = time_us_32();
#ifdef UVM2_REPLAY_LAST
        int replay = 0;
#endif
        /* WITHOUT A LIST THERE ARE NO CONTROLS — unless we read them here. The cache is
         * refreshed after replaying a list, so a game that stops drawing never sees a
         * button change: Major Havoc pauses while button 1 is held (no IRQs, no GO, no
         * list), read the stale "pressed" byte forever and never left the pause; the
         * same starvation made dkong's "hold 50 frames" count in microseconds. The beam
         * is clamped at centre since the previous frame's close, so reading here is as
         * safe as reading after a list. Once per frame period, and the hook runs too so
         * the cart's music and SD work keep going while the game shows nothing. */
        {
            const uint32_t period_us = uvm2_pacer_cycles ? uvm2_pacer_cycles * 2u / 3u : 20000u;
            uint32_t t_idle = t_w0;
            while (uvm2_frame_request == served) {         /* nothing published yet */
#ifdef UVM2_REPLAY_LAST
                /* NO NEW LIST: DRAW THE LAST ONE AGAIN. Between lists the beam was simply off,
                 * so a game slower than 50 Hz flickered at its own rate (SM64 at ~14 Hz:
                 * 22 ms drawn out of every 70, 2026-10-02). The buffer of `served` is held by
                 * stepping frame_done back to served - 1, which uvm2_frame_begin waits on
                 * before writing frame served + 2 into it; then the request is checked AGAIN
                 * after a barrier. If the game published meanwhile, the hold is dropped and
                 * the new list is served, since the producer may already be past its wait. */
                if (served != 0u && !uvm2_replay_hold) {
                    uvm2_replaying = 1u;
                    __asm volatile ("dmb" ::: "memory");
                    if (uvm2_replay_hold) {                 /* the program just went in */
                        uvm2_replaying = 0u;
                        __asm volatile ("dmb" ::: "memory");
                    } else {
                        uvm2_frame_done = served - 1u;
                        __asm volatile ("dmb" ::: "memory");
                        if (uvm2_frame_request == served) { replay = 1; break; }
                        uvm2_frame_done = served;
                        uvm2_replaying = 0u;
                        __asm volatile ("dmb" ::: "memory");
                        break;
                    }
                }
#endif
                if (time_us_32() - t_idle >= period_us) {
                    t_idle = time_us_32();
                    uvm2_single_cycles = 0;
#ifndef UVM2_BIOS
                    reset_poll();                 /* first: it decides whether input is read */
#endif
#ifndef UVM2_NO_INPUT
                    /* NOT THROUGH A VIA THAT WAS JUST RESET. Before the first frame nothing
                     * has programmed it on a restart: the console's reset button put DDRB
                     * back to 0 (see reset_poll), so the reads below write port B into
                     * nothing, the PSG is never selected and port A reads 0x00 — which on
                     * the active-low buttons is ALL PRESSED. uvm2_config_boot_combo read that
                     * as buttons 2+3 held and opened the calibration wizard on every single
                     * press of reset (kuroishi, 2026-10-28). From the launcher it never
                     * showed: the UVMC2's firmware leaves the VIA set up. 0x9F is what every
                     * list sets at its start (uvm2_draw.c, via_setup). Only before the first
                     * frame: after it, DDRB at 0 is reset_poll's signal and must stay one. */
                    if (uvm2_frame_done == 0u && uvm2_via_read(UVM2_VIA_DDRB) == 0)
                        uvm2_via_write(UVM2_VIA_DDRB, 0x9F);
                    if (!INPUT_HELD()) {
                        uvm2_cached_buttons = uvm2_read_buttons();
                        uvm2_cached_axes    = uvm2_read_axes();
                    }
#endif
                    uvm2_draw_invalidate();
                    uvm2_core1_gap();
                    uvm2_stats.idle_frames++;
                }
            }
        }
        uint32_t t0 = time_us_32();
        uvm2_stats.us_wait = t0 - t_w0;
        uvm2_stats.us_c1_wait_acc += t0 - t_w0;   /* what core 1 waited for core 0 */
#ifdef UVM2_REPLAY_LAST
        if (replay) uvm2_stats.replays++;
        else
#endif
        served++;
        __asm volatile ("dmb" ::: "memory");       /* the buffer before the count */

#ifdef UVM2_CMDS_STAGE_SRAM
        /* CONTROL: the list lives in PSRAM but is REPLAYED from SRAM.
         *
         * The hypothesis under test: an XIP cache miss (16 KB) against the PSRAM takes
         * longer than one E period, so the write loses its phase — and the 6800 bus rule
         * is about PHASE, not settling time: it does not degrade, it fails. Hence
         * scribbles mixed in with good vectors rather than a blurry drawing.
         *
         * Copying the whole thing before starting removes EVERY miss from the window with
         * the beam lit. If this cleans it up, the cause is the fetch and the real fix is
         * to bring it in by DMA into a small ring while drawing. If it does NOT clean up,
         * the data is wrong and the fetch had nothing to do with it.
         *
         * This is a CONTROL, not the solution: it spends in SRAM exactly what we were
         * trying to save. */
        static uint8_t s_stage[UVM2_CMD_CAPACITY * 3u];
        const uint8_t  *src = uvm2_frame_buffer(served);
        const uint32_t  n   = uvm2_frame_length(served);
        for (uint32_t i = 0; i < n * 3u && i < UVM2_CMD_CAPACITY * 3u; i++) s_stage[i] = src[i];
        uint32_t cycles = uvm2_exec(s_stage, n);
#else
        {   /* The other half of the signature: what core 1 is ACTUALLY going to replay. */
            const uint8_t *b = uvm2_frame_buffer(served);
            uint32_t n = uvm2_frame_length(served), h = 2166136261u;
            for (uint32_t i = 0; i < n * 3u; i++) { h ^= b[i]; h *= 16777619u; }
            uvm2_sig_read = h;
            if (n == uvm2_sig_n[served & 1u]) {
                uvm2_sig_compared++;
                if (h != uvm2_sig_written[served & 1u]) uvm2_sig_failures++;
                else                                     uvm2_sig_last_ok = h;
            }
        }
        uint32_t cycles;
#ifdef UVM2_CORE0_WORKER
        void (*const work)(void) = uvm2_worker;
        if (work) {
#ifdef UVM2_WORKER_STREAM
            stream_begin(uvm2_frame_buffer(served), uvm2_frame_length(served));
            if (!uvm2_replay_hold) work();    /* the pass loops by DMA while the game works */
            cycles = stream_end();            /* and the bus is done before anything else */
#else
            vbus_list_begin();
            cycles = uvm2_exec(uvm2_frame_buffer(served), uvm2_frame_length(served));
            vbus_list_end();
            vbus_list_fire();                 /* the beam draws by DMA from here */
            if (!uvm2_replay_hold) work();    /* ...while the game works on this core */
            vbus_list_wait();                 /* and the bus is done before anything else */
#endif
        } else
#endif
        cycles = uvm2_exec(uvm2_frame_buffer(served), uvm2_frame_length(served));
#endif
        uint32_t t1 = time_us_32();
        uvm2_stats.us_exec = t1 - t0;
        /* AND THE SAME NUMBER IN A FIELD THAT DOES NOT SHARE A WRITER. `us_exec` is also
         * written by uvm2_svc.c with something ELSE, so reading it does not say whose it
         * is. */
        uvm2_stats.us_c1_exec_acc += t1 - t0;
        uvm2_stats.us_c1_exec_n++;
#ifndef UVM2_BIOS
        {
            static uint32_t since_check;
            if (++since_check >= STACK_CHECK_FRAMES) { since_check = 0; stack_check(); }
        }
        reset_poll();
#endif

        /* HOW LONG THE LIST TAKES AGAINST WHAT IT ASKS FOR, which is the one figure that
         * separates two opposite faults and which we did not have.
         *
         * `exec_cycles` was DECLARED AND NEVER WRITTEN — a dead counter that reads exactly
         * like "not needed". The executor does return the cycles; nobody was saving them.
         *
         * The list knows what it ought to cost (uvm2_list_cycles, counted while building
         * it) and the wall clock says what it did cost. At 1.5 MHz, 1000 cycles are 667 us:
         * if the ratio comes out ~1.0 the bus is running at its own pace and the problem is
         * the SIZE of the list; if it comes out 2.0 the list fits and something is SLOWING
         * the executor down — and the suspect is core 0 emulating the 6502 against the same
         * memory.
         *
         * And this is what the bisection asks for: the fixed-geometry player (no emulation)
         * does NOT flicker and the game DOES. The difference between the two is exactly
         * core 0's load. */
        uvm2_stats.exec_cycles = cycles;
        {
            extern uint32_t uvm2_frame_cycles(uint32_t);
            const uint32_t asked = uvm2_frame_cycles(served);   /* the one just replayed */
            /* IT IS NOT MEASURED AGAINST `t1 - t0`, AND THAT WAS THE TRAP.
             *
             * On the SIO path `uvm2_exec` waits edge by edge on CLK, so its duration IS
             * the bus time and the ratio meant what it promised. With PIO+DMA the same
             * code queues words, calls `vbus_flush` and returns: the PIO keeps playing
             * afterwards. Measuring against that gave **120%**, i.e. "the bus runs 20%
             * faster than its nominal rate" — impossible, because it is locked to the
             * console's CLK. What it was measuring was how fast WE QUEUE, which is exactly
             * what the DMA is for.
             *
             * Against the FRAME PERIOD it does mean something, and it is what you want to
             * know: 100 = the bus is busy for the whole frame (drawing is the limit),
             * 50 = for half the frame the bus is idle and the limit is somewhere else.
             * Believing that 120% cost a whole session: you do not publish a number whose
             * name does not match what was measured. */
            const uint32_t us = s_cycle_us;   /* the PREVIOUS frame's: see below */
            if (asked && us) {
                /* ratio in hundredths: 100 = the bus busy for the whole frame */
                uint32_t r = (uint32_t)(((uint64_t)asked * 100u * 100u) / ((uint64_t)us * 150u));
                uvm2_stats.exec_ratio_last = r;
                if (r > uvm2_stats.exec_ratio_max) uvm2_stats.exec_ratio_max = r;
                if (uvm2_stats.exec_ratio_min == 0u || r < uvm2_stats.exec_ratio_min)
                    uvm2_stats.exec_ratio_min = r;
                /* the whole shape: 100 = the bus busy for the whole frame */
                static const uint32_t T[7] = { 50u, 70u, 85u, 95u, 105u, 130u, 200u };
                unsigned b = 7u;
                for (unsigned i = 0; i < 7u; i++) if (r < T[i]) { b = i; break; }
                uvm2_stats.hist_ratio[b]++;
            }
        }

        /* Between frames, with the beam clamped at centre by the last command of
         * the stream — the only window in which anything else may drive Port A
         * or Port B. Same window the single-core path used, same order.
         */
        uvm2_single_cycles = 0;
#ifndef UVM2_NO_INPUT
        if (!INPUT_HELD()) {                       /* see UVM2_RESET_SETTLE */
            uvm2_cached_buttons = uvm2_read_buttons();
            uvm2_cached_axes    = uvm2_read_axes();
        }
#endif
        uint32_t t2 = time_us_32();
        uvm2_stats.us_input = t2 - t1;
        psg_drain();
        uvm2_core1_gap();
#ifndef UVM2_NO_AUDIO
        /* Advance the sequencer by VECTREX TIME ELAPSED, not once per frame. That is what
         * the single-core path does, and for a measured reason: a frame that blows through
         * the 50 Hz budget is worth two frames of music, and dkong spends 59,000 cycles
         * against 30,000. One tick per frame plays the track at the average speed.
         *
         * Written while "the music does not play in dual core" was still open, as the step
         * that equalised the two paths before comparing them. That issue is closed: .vmus
         * music plays in dual core on the console (dkong, the ports). */
        s_audio_acc += cycles;
        while (s_audio_acc >= UVM2_AUDIO_CYCLES) {
            s_audio_acc -= UVM2_AUDIO_CYCLES;
            uvm2_audio_tick();
        }
#endif
        /* Counted, not assumed: an axis conversion costs as many bus cycles as
         * its SAR needed, and the PSG queue's depth varies with the music. */
        cycles += uvm2_single_cycles;
        /* Redraw state that the input read and the PSG writes just clobbered:
         * one CD4052 serves the pots and the beam on shared select lines, so a
         * joystick conversion lands in the beam's own sample-and-holds. */
        uvm2_draw_invalidate();

        /* THE LOCK, AT RUN TIME. THIS is where it really rules: with dual core,
         * uvm2_frame_end returns before reaching its own pacer, so the one that counts is
         * this one. (I found out because the knob's symbol did not even reach the ELF:
         * nobody referenced the other one.)
         *
         * `cycles` ALWAYS into bus_cycles, no matter what. Previously, in the locked
         * branch, it did `bus_cycles = UVM2_CYCLES_PER_FRAME`: the counter was OVERWRITTEN
         * with the budget and read 30000 whether the drawing cost half that or twice. That
         * is why it did not move while sweeping VCAP, and why I published a "IT FITS" that
         * was false.
         *
         * 0 = free (Asteroids redrew as soon as it finished its list). != 0 = fixed period
         * in bus cycles; 30000 = 50 Hz. */
        uvm2_stats.bus_cycles = cycles;

        /* THE PACE IS SET AGAINST THE CLOCK, NOT AGAINST THE LIST.
         *
         * This used to wait `pacer - cycles` bus cycles, and that assumes that between one
         * frame's end and the next only the list has happened. It is false for two reasons
         * at once: between lists this core reads the controllers, drains the PSG and serves
         * the SD, and besides, `cycles` is what the list BELIEVES it costs, not how long
         * the bus takes. The single-core path already fixed this same thing in
         * uvm2_frame_end and its note says so: "list padded to exactly 30000 cycles and
         * frames from 20.5 to 26.5 ms".
         *
         * Now the target is "the previous frame ended at T, this one ends at T + period".
         * Whatever is left over is waited out with TIMER0; if we have already overshot,
         * that is a real overrun and it re-locks from now rather than dragging the lag
         * along.
         *
         * `uvm2_pacer_cycles` is still the knob (0 = free) and is converted to microseconds
         * here: the bus is 1.5 MHz, i.e. cycles * 2/3. */
        {
            static uint32_t s_end_us;
            const uint32_t now = time_us_32();
            if (s_end_us) s_cycle_us = now - s_end_us;   /* how long this pass lasted */
            if (uvm2_pacer_cycles == 0u) {
                s_end_us = now;
            } else {
                const uint32_t period_us = (uvm2_pacer_cycles * 2u) / 3u;
                const uint32_t target    = s_end_us + period_us;
                const int32_t  remaining = (int32_t)(target - now);
                if (remaining > 0 && (uint32_t)remaining <= period_us) {
                    while ((int32_t)(target - time_us_32()) > 0) { }
                    s_end_us = target;
                } else {
                    uvm2_stats.overrun++;
                    s_end_us = now;
                }
            }
        }

        uvm2_stats.us_rest = time_us_32() - t2;
#ifdef UVM2_REPLAY_LAST
        /* After a REDRAW of the last list (the program published nothing new: it is slow, or
         * a debugger has it paused) whatever the BIOS adds on top: our cartridge's pause sign. */
        if (replay && uvm2_replay_extra) uvm2_replay_extra();
#endif
        __asm volatile ("dmb" ::: "memory");       /* the work before the flag */
        uvm2_frame_done = served;
#ifdef UVM2_REPLAY_LAST
        if (replay) {
            __asm volatile ("dmb" ::: "memory");
            uvm2_replaying = 0u;
        }
#endif
    }
}

/* STOPPING CORE 1, for what cannot be done with it running: writing to flash. Erasing or
 * programming flash hangs any core that executes or reads from XIP, and the list executor
 * lives here. `multicore_reset_core1` really does stop it; to come back, `uvm2_core1_start`,
 * which already begins by resetting it. */
/* OUR OWN CARTRIDGE'S BIOS runs this same loop ON CORE 0 (there the program runs on core 1
 * and core 0 replays; it is the UVM2 mirrored with the cores swapped). The loop and a
 * per-pass hook are exposed for whatever the BIOS does between lists besides the
 * controllers and the PSG: its music player and the SD work. */
/* A BIOS MUST DEFINE IT, AND THE LINK SAYS SO. On 2026-10-01 the SDK's translation
 * renamed this hook (it was uvm2_core1_hueco) and the BIOS kept the old name; this
 * weak, empty default took its place without a word, and every BIOS built since
 * lost what its hook does — the menu's music and SD previews among it — found only
 * because a dump through the same hook printed nothing. So in a BIOS build
 * (UVM2_BIOS) there is no default: a BIOS that does not define uvm2_core1_gap
 * does not link. A .um2 keeps the empty one; it has nothing to do there. */
#ifndef UVM2_BIOS
__attribute__((weak)) void uvm2_core1_gap(void) { }
#endif
void uvm2_core1_loop(void) { core1_main(); }

void uvm2_core1_stop(void)
{
    multicore_reset_core1();
}

void uvm2_core1_start(void)
{
    /* RESET BEFORE LAUNCHING, and this is what makes loading over SWD possible.
     *
     * `multicore_launch_core1` assumes core 1 is stopped. On a cold boot it is; loading a
     * new image over SWD on top of one that was already running, it is NOT — core 1 is
     * still in the PREVIOUS image's loop, the FIFO handshake never arrives and the new
     * image hangs there forever. Found on 2026-08-24 with tools/probe.sh: pc in
     * multicore_fifo_rvalid, lr in multicore_launch_core1_raw. The symptom is "it does not
     * draw", i.e. indistinguishable from a bug in the program.
     *
     * I tried to solve it from outside by powering the core down through the PSM and hung
     * the whole console. From inside it is one line, the pico-sdk provides it, and on a cold
     * boot it does nothing. With this, tools/load.sh works for dual-core images too and the
     * test cycle goes from a minute (pull the SD, copy, menu) to a few seconds. */
    multicore_reset_core1();
    multicore_launch_core1(core1_main);
}

#endif /* UVM2_DUAL_CORE */
