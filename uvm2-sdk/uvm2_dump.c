/* uvm2_dump.c — the frame's command list, written to the SD card.
 *
 * WHY. The list is the one thing that says what the beam was asked to do, and until now the
 * only way to get it out was the debug cartridge's BIOS printing it over RTT, sixteen
 * commands at a time, with a probe on the board. That found the 2026-10-01 asterisk — and it
 * exists on one cartridge. This writes the same list to the card, with no probe, on whatever
 * cartridge runs the SDK; tools/list_from_sd.py turns it into what tools/beam_sim.py reads.
 *
 * WHICH CORE, AND WHEN. Core 0 only, and that is checked, not hoped for:
 *   - the SD driver is not re-entrant, and everything else that touches the card (the config,
 *     the sample loader, the game's own files) runs on core 0;
 *   - core 1 owns the Vectrex bus (invariant 3) and must never sit in a card write;
 *   - the list it reads is the LAST CLOSED frame's, which core 0 — its only writer — is not
 *     writing (see uvm2_last_list in uvm2_draw.c, and why it is chosen by frame and not by
 *     length).
 * The card is not the Vectrex bus, so invariant 7 (reads between frames) is not at stake; but
 * a write takes milliseconds to tens of them, and while it runs core 0 builds nothing: the
 * screen goes dark for that long. A debug capture, not something to call every frame.
 *
 * THE FILE, little-endian:
 *     0  'UVML'                 magic
 *     4  u16 version (1)        6  u16 header size (32)
 *     8  u32 commands           12 u32 frame number (uvm2_frame_count)
 *    16  u32 buffer (0/1)       20 u32 uvm2_stats.dropped of that frame
 *    24  u32 uvm2_stats.ramps_clamped of that frame
 *    28  u32 FNV-1a of the command bytes
 *    32  commands, 3 bytes each, exactly as uvm2_draw.c packs them
 * The length and the hash are there so a reader can REFUSE a file that is not one whole list.
 * A non-zero `dropped` means the list overflowed: the reader prints it, because then the list
 * is not evidence of anything (CLAUDE.md, "Failures must be loud").
 *
 * UNDER THE DEBUG CARTRIDGE'S BIOS (UVM2_BIOS) THE CORES ARE SWAPPED: the game builds the list
 * on core 1 and core 0 — the BIOS — replays it, owns the card, and runs uvm2_core1_gap()
 * between lists. So there the dump runs on core 0 FROM THAT GAP, and the list is chosen by
 * the executor's own counters (list_to_dump below), never by uvm2_last_list: that one waits
 * for the executor to finish a frame, and called from inside the executor it would wait for
 * itself for ever. The BIOS has no FAT allocation, so its uvm2_sd_write2 overwrites an
 * EXISTING file in place (made on the PC, big enough); bytes past the list are whatever the
 * file held, and the header's count is what says where the list ends.
 *
 * FAILURES ARE COUNTED AND NAMED: uvm2_dump_diag.ok / .failed and .error (UVM2_DUMP_*); for
 * UVM2_DUMP_SD, uvm2_sd_error says which card failure it was.
 */
#include <stdint.h>
#include "uvm2_bus.h"
#include "uvm2_draw.h"
#include "uvm2_sd.h"

uvm2_dump_diag_t uvm2_dump_diag;

/* WHERE THE COLD CODE GOES. Empty here; the debug cartridge's BIOS defines it to a flash
 * section, because that BIOS runs its code from SRAM and an unexplained input failure appears
 * as that code grows (its build.rs). A dump runs once per button press: it can run from flash. */
#ifndef UVM2_COLD
#define UVM2_COLD
#endif

extern const uint8_t *uvm2_last_list(uint32_t *n, uint32_t *which);

#ifdef UVM2_BIOS
/* THE LIST THE EXECUTOR IS NOT GOING TO TOUCH, from inside its gap. Two states call the gap:
 *   - after replaying frame `served`: uvm2_frame_done is still served - 1 (it is set after the
 *     gap) and uvm2_frame_request >= served;
 *   - idle, nothing new published: uvm2_frame_done == uvm2_frame_request == served.
 * So the frame is done + 1 in the first case and done in the second. Either way its buffer is
 * stable for as long as we are in the gap: the builder reuses a buffer only after
 * uvm2_frame_done has passed it (uvm2_frame_end's wait), and only the executor — this core,
 * busy here — moves uvm2_frame_done. If the builder publishes one more frame meanwhile, the
 * choice lands on that one, which is complete and just as stable. */
UVM2_COLD static const uint8_t *list_to_dump(uint32_t *n, uint32_t *which, uint32_t *frame)
{
    extern volatile uint32_t uvm2_frame_done, uvm2_frame_request;
    extern const uint8_t *uvm2_frame_buffer(uint32_t frame);      /* uvm2_draw.c */
    extern uint32_t       uvm2_frame_length(uint32_t frame);
    const uint32_t done = uvm2_frame_done, req = uvm2_frame_request;
    const uint32_t f = done == req ? done : done + 1u;
    *frame = f; *which = f & 1u;
    if (f == 0u) { *n = 0; return 0; }          /* nothing published yet */
    __asm volatile ("dmb" ::: "memory");       /* the counters before the buffer */
    *n = uvm2_frame_length(f);
    return uvm2_frame_buffer(f);
}
#else
static const uint8_t *list_to_dump(uint32_t *n, uint32_t *which, uint32_t *frame)
{
    *frame = uvm2_frame_count();
    return uvm2_last_list(n, which);
}
#endif

UVM2_COLD static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

UVM2_COLD static int refuse(int why)
{
    uvm2_dump_diag.error = why;
    uvm2_dump_diag.failed++;
    return 0;
}

UVM2_COLD int uvm2_dump_list(const char *path)
{
#if !defined(UVM2_HOST)
    if (UVM2_CPUID != 0u) return refuse(UVM2_DUMP_CORE1);
#endif
    uint32_t n = 0, which = 0, frame = 0;
    const uint8_t *list = list_to_dump(&n, &which, &frame);
    if (!list || n == 0u) return refuse(UVM2_DUMP_EMPTY);

    uint32_t h = 2166136261u;
    for (uint32_t i = 0; i < n * 3u; i++) { h ^= list[i]; h *= 16777619u; }

    uint8_t head[UVM2_DUMP_HEADER];
    head[0] = 'U'; head[1] = 'V'; head[2] = 'M'; head[3] = 'L';
    head[4] = UVM2_DUMP_VERSION; head[5] = 0; head[6] = UVM2_DUMP_HEADER; head[7] = 0;
    put32(head + 8,  n);
    put32(head + 12, frame);
    put32(head + 16, which);
    put32(head + 20, uvm2_stats.dropped);
    put32(head + 24, uvm2_stats.ramps_clamped);
    put32(head + 28, h);

    if (!uvm2_sd_write2(path, head, UVM2_DUMP_HEADER, list, n * 3u)) return refuse(UVM2_DUMP_SD);
    uvm2_dump_diag.error = UVM2_DUMP_OK;
    uvm2_dump_diag.ok++;
    uvm2_dump_diag.commands = n;
    return 1;
}

int uvm2_dump_list_on_buttons(const char *path, uint8_t mask)
{
    extern volatile uint8_t uvm2_cached_buttons;      /* uvm2_core1.c; active LOW */
    static uint8_t was_held;
    const uint8_t pressed = (uint8_t)~uvm2_cached_buttons;
    const uint8_t held = mask != 0u && (pressed & mask) == mask;
    const int edge = held && !was_held;
    was_held = held;
    if (!edge) return 0;
    return uvm2_dump_list(path) ? 1 : -1;
}
