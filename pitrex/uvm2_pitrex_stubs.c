/* uvm2_pitrex_stubs.c — the neighbours of uvm2_draw.c that the PiTrex build does not have
 * YET. Each one is a piece of the port still to be done, named here so that it is one list
 * and not a set of link errors:
 *
 *   the start-up card (uvm2_intro.c)   skipped
 *
 * Everything is WEAK: the day the real file is added to pitrex.mk it wins, and its stub
 * here is deleted.
 */
#include <stdint.h>

#define WEAK __attribute__((weak))

/* The start-up card with its jingle (uvm2_intro.c): the jingle is the jack's. */
WEAK void uvm2_intro(void) { }

/* The Rust beam model's static library asks for this; nothing unwinds here. */
WEAK void rust_eh_personality(void) {}

/* newlib's exit path names it (pulled in by qsort/setjmp users); a kernel image never exits. */
WEAK void _fini(void) {}
