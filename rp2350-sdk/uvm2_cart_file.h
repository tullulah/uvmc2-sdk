/*
 * uvm2_cart_file.h — reading the SD card from a game on the DEBUG CARTRIDGE (sdk_rp2350.c).
 *
 * The UVMC2 reads its card on the game's own core (uvm2_sd.h, FatFs). This cartridge cannot:
 * while one core drives the Vectrex's bus a card read from the other comes back as garbage, so
 * the BIOS does the reading itself, in the executor's gap between lists, and a game asks for
 * it and polls. A read therefore takes about a list per 8 KB, and a game that wants to keep
 * its frame rate starts one, draws, and looks again next frame.
 *
 * Paths are a file in the root or one folder down ("HZOO/SFX.PCM"), long names allowed.
 * BIOS table version 10 or later; on an older one every call fails.
 */
#ifndef UVM2_CART_FILE_H
#define UVM2_CART_FILE_H

#include <stdint.h>

/* 1 and the file's size if it is there. BLOCKING, about one list: it opens the file by reading
 * its first byte (the BIOS opens a file on its first read). */
int uvm2_cart_file_open(const char *path, uint32_t *size);
/* Starts reading n bytes at `off` into dst (SRAM, or the PSRAM through its uncached alias
 * 0x15000000). 0 started, else an error (one read at a time: -1 if one is still going). */
int uvm2_cart_read_start(const char *path, uint32_t off, void *dst, uint32_t n);
/* >0 still reading, 0 done, <0 failed. */
int uvm2_cart_read_poll(void);
/* How long the BIOS may read in each gap between lists, in microseconds (default 1500). Every
 * microsecond of it delays the next list: in play the picture trembles past ~1.5 ms, but a
 * loading screen can give the card most of the frame. Table version 11; older, ignored. */
void uvm2_cart_set_gap_us(uint32_t us);

/* Microseconds, free-running (TIMER0's raw count): what pico/time.h's time_us_32 is, for code
 * that measures and would otherwise need the pico-sdk. */
static inline uint32_t uvm2_cart_us(void) { return *(volatile uint32_t *)0x400B0028u; }

/* The bytes uvm2_sd_next(f, ...) can return NOW without waiting for the BIOS: 0 while the next
 * block is still being read (and it asks for it if nobody has). A loader with a time budget asks
 * this first and leaves the rest for the next frame, instead of losing one on a wait. */
struct uvm2_sd_file_s;
uint32_t uvm2_sd_ready(const void *f);

/* ALSO ON THIS CART, the UVMC2's own card API (uvm2_sd.h): uvm2_sd_open / _next / _seek /
 * _close / _find, over the reader above. A sequential reader keeps its frame rate: two 8 KB
 * buffers, and while the game takes one the BIOS fills the next in the gap. And uvm2_psram.h's
 * bring-up, a no-op here (the BIOS brought the PSRAM up before the game started). */

#endif /* UVM2_CART_FILE_H */
