/* uvm2_pitrex_loader_entry.S — where a RESIDENT image starts (pitrex.mk, PITREX_LOADER = 1).
 *
 * THE PITREX'S TWO-STAGE START. The card's kernel is a small bootstrap (the baremetal tree's
 * pitrex7.img): it brings the machine up, reads piZero2/loader.pit to LOADER_START (64 MB,
 * out of the way) and calls it. That resident program — the menu — reads the program the user
 * picks to 0x8000, where every program is linked, and starts it as a kernel; a program that
 * wants to go back calls an address the resident one left for it.
 *
 * So a resident image is this SDK's same code, linked 64 MB up. THIS OBJECT GOES FIRST IN THE
 * LINK: the default linker script puts .init at 0x8000, and the .org below pushes everything
 * after it to LOADER_START, which is the baremetal tree's own way of placing its loader
 * (loaderEntry.S). pitrex.mk then cuts the first LOADER_START - 0x8000 bytes off the image.
 *
 * TWO WAYS IN:
 *   uvm2_loader_entry   fresh from the card. .data is as the file had it: a copy is kept
 *                       just past the image, before anything can change it.
 *   uvm2_loader_again   a program came back. The image is still in memory, but with .data as
 *                       the menu left it and whatever stack the program was on: .data is put
 *                       back from the copy, the stack starts again from the top, and from
 *                       there on it is the same start. Without this the SDK would begin with
 *                       "core 1 is already running" and every other piece of state from the
 *                       previous run.
 *
 * Past the image, in this order (uvm2_pitrex_loader.c knows the same layout):
 *   __end__ rounded up to 16      UVM2_LOADER_KEEP_BYTES that survive a return: zeroed on a
 *                                 fresh start only (where the menu's cursor was)
 *   after that                    the copy of .data
 * The C heap starts at 0x5000000 (the baremetal tree's linkerHeapDefBoot.ld), far above both.
 */
#include "baremetal/globalVars.h"
#include "uvm2_pitrex_loader.h"

    .section .init
    .code 32
    .align 2
    .org (LOADER_START - 0x8000)

    .global uvm2_loader_entry
uvm2_loader_entry:
    bl      keep_and_copy_addr          @ r4 = the kept block, r5 = the copy of .data
    mov     r0, r4                      @ a fresh start: nothing kept yet
    add     r1, r4, #UVM2_LOADER_KEEP_BYTES
    mov     r2, #0
1:  cmp     r0, r1
    strlo   r2, [r0], #4
    blo     1b
    ldr     r0, =__data_start           @ .data, as the file had it
    ldr     r1, =__bss_start__
    mov     r2, r5
2:  cmp     r0, r1
    ldrlo   r3, [r0], #4
    strlo   r3, [r2], #4
    blo     2b
    b       start

    .global uvm2_loader_again
uvm2_loader_again:
    ldr     sp, =__svc_stack_top0       @ the program that came back never resumes
    bl      keep_and_copy_addr
    ldr     r0, =__data_start
    ldr     r1, =__bss_start__
    mov     r2, r5
3:  cmp     r0, r1
    ldrlo   r3, [r2], #4
    strlo   r3, [r0], #4
    blo     3b

start:
    mov     r0, #0
    ldr     r1, =__bss_start__
    ldr     r2, =__bss_end__
4:  cmp     r1, r2
    strlo   r0, [r1], #4
    blo     4b
    bl      main
5:  b       5b

keep_and_copy_addr:
    ldr     r4, =__end__
    add     r4, r4, #15
    bic     r4, r4, #15
    add     r5, r4, #UVM2_LOADER_KEEP_BYTES
    bx      lr

    .ltorg
