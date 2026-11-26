/* uvm2_pitrex.h — what is specific to the PiTrex build of the SDK.
 *
 * The game-facing API is the same as on the cartridges (uvm2_draw.h, uvm2_input.h, ...).
 * This header only carries what the board's own layer exposes: what start-up measured, and
 * the report that puts it on the SD card, because a PiTrex has no debug probe.
 */
#ifndef UVM2_PITREX_H
#define UVM2_PITREX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Offsets tried when looking for the board's latch point (uvm2_bus_pitrex.c). */
/* The Pi's peripheral base, by board: RASPPI comes from pitrex.mk (1 = BCM2835 on the first
 * Zero, 3 = BCM2710 on the Zero 2 W). The first Zero's MMU maps nothing at 0x3F000000, so a
 * Zero 2 address there is a data abort on the first timer read. */
#if RASPPI == 1
#define UVM2_PITREX_PERI_BASE 0x20000000u
#else
#define UVM2_PITREX_PERI_BASE 0x3F000000u
#endif

#define UVM2_PITREX_LATCH_STEPS 32u

/* How far start-up got. The report is written BEFORE the bus is touched and again after,
 * so a console that is off, or a board that never answers on RDY, leaves stage 1 on the
 * card instead of nothing at all. */
enum {
    UVM2_PITREX_STAGE_NONE = 0,
    UVM2_PITREX_STAGE_BUS_INIT,       /* about to make the first bus access */
    UVM2_PITREX_STAGE_MEASURE_E,
    UVM2_PITREX_STAGE_MEASURE_LATCH,
    UVM2_PITREX_STAGE_BUS_READY,      /* everything below is filled in */
    UVM2_PITREX_STAGE_DRAWING,        /* frames have gone out */
};

typedef struct {
    uint32_t stage;           /* UVM2_PITREX_STAGE_* */
    int32_t  init_result;     /* vectrexinit: 1 = bus ready, -1 = /HALT not asserted (jumper) */
    uint32_t e_measured;      /* 1 = uvm2_cycles_per_e_q8 came from the VIA's timer 2 */
    uint32_t latch_found;     /* 1 = the curve below has the step the model expects */
    uint32_t latch_point;     /* CPU cycles after a write returns: the last moment a request
                               * still takes the coming bus cycle */
    uint32_t back_to_back;    /* CPU cycles between two writes requested with no wait */
    uint32_t write_cycles;    /* the same in bus cycles, rounded: what one command costs */
    uint32_t stretched;       /* commands of the last list that asked for fewer cycles than
                               * a write costs on this board, and so ran long */
    uint32_t core1_started;   /* 1 = core 1 took the loop (dual-core builds) */
    uint32_t core1_state_before, core1_state_after;   /* the start-up's CORE1_STATE around it */
    uint32_t late_max;        /* the most bus cycles any write of the last list was behind */
    /* Mean CPU cycles from one write's return to the next one's, for a request raised
     * (2 periods * i / UVM2_PITREX_LATCH_STEPS) after the first. */
    uint32_t latch_curve[UVM2_PITREX_LATCH_STEPS];
} uvm2_pitrex_bus_t;

extern uvm2_pitrex_bus_t uvm2_pitrex_bus;

/* The handshake, asked directly (uvm2_pitrex_probe.c). Times in CPU cycles. */
#define UVM2_PITREX_PROBE_STEPS   8u    /* request moments tried across one period */
#define UVM2_PITREX_PROBE_REPEATS 32u
#define UVM2_PITREX_PROBE_HOLDS   2u

typedef struct {
    uint32_t ran;
    /* A request raised (period * i / STEPS) after a write returned: */
    uint32_t anatomy_start[UVM2_PITREX_PROBE_STEPS];   /* LATCH_EN high -> RDY low */
    uint32_t anatomy_len[UVM2_PITREX_PROBE_STEPS];     /* RDY low -> RDY high */
    uint32_t anatomy_good[UVM2_PITREX_PROBE_STEPS];    /* handshakes that completed */
    /* LATCH_EN held high after an access, data and address changed: */
    uint32_t hold_second_seen[UVM2_PITREX_PROBE_HOLDS];   /* RDY went low and high again */
    uint32_t hold_start[UVM2_PITREX_PROBE_HOLDS];         /* from the first access's end */
    uint32_t hold_end[UVM2_PITREX_PROBE_HOLDS];
    uint8_t  hold_want_first[UVM2_PITREX_PROBE_HOLDS], hold_read_first[UVM2_PITREX_PROBE_HOLDS];
    uint8_t  hold_want_second[UVM2_PITREX_PROBE_HOLDS], hold_read_second[UVM2_PITREX_PROBE_HOLDS];
} uvm2_pitrex_probe_t;

extern uvm2_pitrex_probe_t uvm2_pitrex_probe;
void uvm2_pitrex_probe_handshake(void);

/* Read the controllers into the caches the wizard and the HUD use. Called by uvm2_exec
 * after every list. */
void uvm2_pitrex_between_lists(void);

/* Write the start-up measurements and the last frame's counters to UVM2.LOG on the card.
 * Safe to call between frames; returns 0 if the file could not be written. */
int uvm2_pitrex_report(void);

/* The bit of the VIA's interrupt-enable register that says "no reset since start-up"
 * (uvm2_bus_pitrex.c, uvm2_bus_halt): CB1's enable. */
#define UVM2_PITREX_RESET_MARK 0x10u

/* The PiTrex menu (the loader that started this image, if one did). uvm2_frame_end polls the
 * four buttons of controller 1; a game may also leave by itself. */
int  uvm2_pitrex_has_menu(void);
void uvm2_pitrex_to_menu(void);
/* It does not return when it works. When it does return, why: */
#define UVM2_TO_MENU_NO_MENU 1    /* the card's kernel is this program: there is no menu */
#define UVM2_TO_MENU_IS_MENU 2    /* this program is the menu */
#define UVM2_TO_MENU_CORE1   3    /* core 1 did not leave the bus in time */
extern int uvm2_pitrex_to_menu_refused;
void uvm2_pitrex_menu_poll(void);
int  uvm2_pitrex_park_core1(void);   /* core 1 off the bus, parked: before handing the machine over */

/* Optional, defined by the game: its own lines appended to every report (CRLF-terminated). */
const char *uvm2_pitrex_game_note(void);

#ifdef __cplusplus
}
#endif

#endif /* UVM2_PITREX_H */
