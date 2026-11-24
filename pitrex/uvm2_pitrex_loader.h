/* uvm2_pitrex_loader.h — a RESIDENT PiTrex image: the program the card's bootstrap loads at
 * LOADER_START and that starts every other program (a menu). Built with PITREX_LOADER = 1
 * (pitrex.mk); the how and why of the two-stage start is in uvm2_pitrex_loader_entry.S.
 *
 * Read by the assembler too: only #defines outside the __ASSEMBLER__ fence.
 */
#ifndef UVM2_PITREX_LOADER_H
#define UVM2_PITREX_LOADER_H

/* Bytes just past the image that survive a program coming back (zero on a fresh start). A
 * multiple of 16. */
#define UVM2_LOADER_KEEP_BYTES 4096

#ifndef __ASSEMBLER__
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The kept block: UVM2_LOADER_KEEP_BYTES, zero the first time, then whatever was left in it
 * before the last launch. */
void *uvm2_pitrex_loader_keep(void);

/* Read a program image from the card (a path such as "piZero2/aae.img") to where programs
 * are linked and start it. `param1` and `param2` are what the program is given (text; NULL
 * for none) and `button` the button that started it — the three things the baremetal tree's
 * programs read back with getLoadParameter.
 *
 * Does not return when it works. Returns 0 if the file could not be read (nothing has
 * changed: the menu goes on) and -1 if core 1 would not leave the bus. */
int uvm2_pitrex_launch(const char *path, const char *param1, const char *param2, int button);

#ifdef __cplusplus
}
#endif
#endif /* !__ASSEMBLER__ */

#endif /* UVM2_PITREX_LOADER_H */
