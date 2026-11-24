/* uvm2_pitrex_loader.c — what a resident PiTrex image does that a program does not: it
 * mounts the card itself, tells the programs it starts where to come back to, and starts
 * them. See uvm2_pitrex_loader_entry.S for the two-stage start this belongs to.
 *
 * THE CONTRACT WITH THE PROGRAMS is the baremetal tree's (pi_support.h, globalVars.h), so
 * that its own programs — the Vectrex emulators, the arcade ones — start from this image and
 * come back to it exactly as they do with its loader:
 *   LOADER_AVAILABLE   1 while a resident image is in memory
 *   SETTING_STORE      the address of a GlobalMemSettings: the three parameters of the
 *                      launch, and `loader`, the address to call to come back
 *   0x8000             where a program is read to and started, with the MMU off and the
 *                      registers the firmware gave the first kernel
 */
#include <stdint.h>
#include <string.h>
#include "uvm2_bus.h"
#include "uvm2_pitrex.h"
#include "uvm2_pitrex_loader.h"
#include "baremetal/ff.h"
#include "baremetal/pi_support.h"
#include "baremetal/globalVars.h"

#define PROGRAM_ADDR      0x8000u
/* A program may be as big as the space below this image. */
#define PROGRAM_MAX_BYTES (LOADER_START - PROGRAM_ADDR - 0x100000u)

void uvm2_loader_again(void);                 /* uvm2_pitrex_loader_entry.S */
extern char __end__[];

static FATFS s_fs;

/* IN .data ON PURPOSE (the non-zero initialiser): the baremetal tree's programs only accept a
 * settings block that sits within 1.5 MB of LOADER_START (getLoadParameter), and .bss — the
 * menu's tables — can end beyond that. .data comes right after the code. */
static GlobalMemSettings s_settings = { .resetType = RESET_TYPE_CONTINUE_LOADER };

void *uvm2_pitrex_loader_keep(void)
{
    return (void *)(((uintptr_t)__end__ + 15u) & ~(uintptr_t)15u);
}

/* THE HEAP. The baremetal library's sbrk (cstubs.c) starts the C heap at `_end`: the image's
 * own end, which is exactly where a resident image keeps what must survive a program — the
 * kept block and the copy of .data (uvm2_pitrex_loader_entry.S). A menu that never calls
 * malloc does not notice. One that does writes its game list over the saved .data, and the
 * next return restores the C library's state from text: VMMenu came back to a fault in
 * __sinit, reading a pointer made of the letters of a game's name, thirteen times in a row,
 * and the screen stayed black (2026-11-24; the card's log named the address). So a resident
 * image's heap starts past both. The link wraps the library's two entries (pitrex.mk); the
 * break itself is in .bss, so each start begins with an empty heap again. */
extern char __data_start[];
static char *s_heap_break;

static void *heap_more(int incr)
{
    if (!s_heap_break) {
        const uintptr_t copy_bytes = (uintptr_t)&__bss_start__ - (uintptr_t)__data_start;
        s_heap_break = (char *)(((uintptr_t)uvm2_pitrex_loader_keep() + UVM2_LOADER_KEEP_BYTES
                                 + copy_bytes + 15u) & ~(uintptr_t)15u);
    }
    char *was = s_heap_break;
    s_heap_break += incr;
    return was;
}
void *__wrap_sbrk(int incr)  { return heap_more(incr); }
void *__wrap__sbrk(int incr) { return heap_more(incr); }

/* Called first thing by start-up (uvm2_pitrex_main.c): before its first report, which
 * needs the card. The bootstrap mounted the card for ITSELF; this image carries its own copy
 * of the file system's state, and after a program came back it is zero again. */
void uvm2_pitrex_early(void)
{
    f_mount(&s_fs, "", 1);
    s_settings.loader    = uvm2_loader_again;
    s_settings.resetType = RESET_TYPE_CONTINUE_LOADER;
    s_settings.flags     = FLAG_LOADER_RUNNING;      /* "already in the menu": the way back is a no-op here */
    *(GlobalMemSettings **)SETTING_STORE = &s_settings;
    *(volatile int *)LOADER_AVAILABLE = 1;
}

static void copy_param(char *dst, const char *src)
{
    const size_t max = sizeof s_settings.parameter1 - 1u;
    size_t n = src ? strlen(src) : 0u;
    if (n > max) n = max;
    memcpy(dst, src, n);
    dst[n] = 0;
}

int uvm2_pitrex_launch(const char *path, const char *param1, const char *param2, int button)
{
    FIL f;
    UINT got = 0;
    /* The image first, while the picture is still up: nothing lives at PROGRAM_ADDR while
     * this image runs, and a file that fails to read leaves the menu as it was. */
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    const FRESULT r = f_read(&f, (void *)PROGRAM_ADDR, PROGRAM_MAX_BYTES, &got);
    f_close(&f);
    if (r != FR_OK || got == 0) return 0;

    /* The program takes the bus. */
    if (!uvm2_pitrex_park_core1()) return -1;

    copy_param(s_settings.parameter1, param1);
    copy_param(s_settings.parameter2, param2);
    s_settings.parameter3[0] = (char)button;
    s_settings.parameter3[1] = 0;
    s_settings.flags = FLAG_SELECTION_MADE;          /* and no longer "in the menu" */

    /* As the baremetal tree's loader does it (loaderMain.c, startWithCurrentSettings): the
     * program is a kernel and sets the machine up again from nothing. */
    __asm__ volatile("isb\n dsb\n dmb" ::: "memory");
    mmu_disable();
    cache_flush();
    {
        const uint32_t *boot = (const uint32_t *)R0_STORE;       /* r0-r2 as the firmware gave them */
        ((void (*)(uint32_t, uint32_t, uint32_t))PROGRAM_ADDR)(boot[0], boot[1], boot[2]);
    }
    return -1;       /* not reached */
}
