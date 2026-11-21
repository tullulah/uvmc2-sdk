/* uvm2_romzip.c — read the romset off the SD card and publish it. Here the GAME does it,
 * because the UVM2 firmware does not.
 *
 * THE LOADING MACHINERY DOES NOT CHANGE. romzip.c still reads a zip FROM MEMORY through an
 * 'RMZ1' descriptor in game_header[3]; all that changes is who fills it in. On a cartridge
 * whose firmware serves romsets, the firmware fills it before starting the game; here we
 * fill it ourselves after reading the card.
 *
 * The descriptor exists from the start with its magic at ZERO, so if the read fails
 * romzip_open_cart() returns 0 exactly as always and the game paints its "no romset" sign —
 * which is the correct answer, not a hang.
 */
/* Boot-only code (runs once, before the game): built for size, not speed. Every byte of
 * text here is SRAM the game cannot use, and aztarac's image is that tight. */
#pragma GCC optimize("Os")
#include "uvm2_sd.h"
#include <stdint.h>

/* HOW MUCH ROMSET FITS. This is a static array in SRAM and the UVM2 image lives in 496 KB,
 * so it cannot be set "big just in case": at 64 KB, one of the games would not link (region
 * RAM overflowed by 37700 bytes). It is set per game, like AAE_ROM_STAGE:
 *
 *     UVM2_CFLAGS += -DUVM2_ROMZIP_MAX=$$((24*1024))
 *
 * The game's zip decides: look at `ls -l roms/<game>.zip` and round up. Coming up short does
 * NOT hang — uvm2_sd_read returns 0 with UVM2_SD_TOO_BIG and the game paints its "no
 * romset" sign. */
#ifndef UVM2_ROMZIP_MAX
#define UVM2_ROMZIP_MAX (24u * 1024u)
#endif

/* Weak: a game with no external ROM does not define it and this comes to nothing. */
__attribute__((weak)) extern const char game_romset_name[];

#ifndef UVM2_ROMZIP_PATH_MAX
#define UVM2_ROMZIP_PATH_MAX 96     /* the longest path to a romset that is looked at */
#endif

/* THE ROMSET, IN PSRAM. 23 KB of SRAM that were occupied for the WHOLE session by something
 * only used at startup: it is read off the card, decompressed into the game's ROM tables,
 * and never needed again. For some games that is more than they have left free.
 *
 * THROUGH THE UNCACHED ALIAS (0x15400000). Writing through the normal window CORRUPTS the
 * data: measured, 2512 bad words out of 4096; through the alias, zero. And here the PSRAM is
 * at its best — written once and read once, with nothing timing-dependent about it, which is
 * exactly the opposite of the command list.
 *
 * 4 MB in, so nothing collides if the two ever coexist. */
#ifdef UVM2_ROMZIP_IN_PSRAM
#  ifndef UVM2_ROMZIP_PSRAM_BASE
#    define UVM2_ROMZIP_PSRAM_BASE 0x15400000u
#  endif
static unsigned char *const s_zip = (unsigned char *)(uintptr_t)UVM2_ROMZIP_PSRAM_BASE;
#else
static unsigned char s_zip[UVM2_ROMZIP_MAX] __attribute__((aligned(8)));
#endif
static uintptr_t s_desc[4];                    /* [0]=magic [1]=base [2]=size */

/* IT IS NOT CALLED `game_header`, AND THAT IS THE FIX.
 *
 * It used to be: a STRONG game_header[4] that deliberately overrode romzip.c's weak one to
 * publish the descriptor in word 3. That worked for the emulated ports... and broke every
 * game that generates its own strong game_header in its .S. Two strong definitions of the
 * same symbol:
 *
 *     multiple definition of `game_header'
 *
 * So none of those games would link for the UVM2, and the failure did not show in the
 * emulated ports because there the only rival game_header is weak.
 *
 * Publishing it under its own name means nobody fights over anything: the game keeps ITS
 * header and romzip.c looks here first.
 *
 * IT IS DEFINED HERE, which is where it is filled in. For a while it was defined in
 * romzip.c and declared here, and that broke any image linking the SDK WITHOUT romzip.c —
 * the test bench, for instance. The other way round always works: the UVM2 SDK goes into
 * every UVM2 image, and whoever does not link it does not link this either. See the note in
 * romzip.c for the two weak-symbol versions that failed before. */
unsigned long uvm2_romzip_desc = 0;

uint32_t uvm2_romzip_bytes = 0;                /* so it can be diagnosed from outside */
int      uvm2_romzip_error = 0;
/* WHERE IT WAS FOUND, for the game's "no romset" sign and for SWD. Empty: not found. */
char     uvm2_romzip_path[UVM2_ROMZIP_PATH_MAX];

/* ── WHERE THE ROMSET IS: ANYWHERE ON THE CARD ─────────────────────────────────────────
 *
 * The images can live in folders of the card's owner's choosing (ATARI/, CINEMATRONICS/...)
 * and the romsets in any other, so the zip is looked for by NAME, in this order:
 *
 *   1. where it was found last time: .conf/<romset without .zip>.loc, one line, the path;
 *   2. .roms/<romset>, then roms/<romset> (an older card's), and nothing has to be written;
 *   3. the whole card, folder by folder, the first file with that name (case-insensitive).
 *      Found there, its path is written to the .loc, so the next start goes straight to it.
 *
 * A romset moved after that: the .loc points at nothing, the read fails, and it falls
 * through to 2 and 3 again, rewriting the .loc. The scan only runs when it has to.
 *
 * Its own small file, not a line in the game's config/<GAME>.cfg: the romset is read HERE,
 * before the game's code has run, and the game names its .cfg in uvm2_game_settings, which
 * comes later. One file per romset also means two games never rewrite each other's.
 *
 * Before core 1 starts: nothing drives the bus yet, so the card reads and writes are safe. */

static int name_eq(const char *a, const char *b)    /* ASCII, case-insensitive */
{
    for (;; a++, b++) {
        char x = *a, y = *b;
        if (x >= 'a' && x <= 'z') x -= 32;
        if (y >= 'a' && y <= 'z') y -= 32;
        if (x != y) return 0;
        if (!x) return 1;
    }
}

static int cat(char *d, int n, int max, const char *s)
{
    while (*s && n < max - 1) d[n++] = *s++;
    d[n] = 0;
    return *s ? -1 : n;          /* -1: did not fit */
}

static uint32_t try_path(const char *path)
{
    uint32_t n = uvm2_sd_read(path, s_zip, UVM2_ROMZIP_MAX);
    uvm2_romzip_error = uvm2_sd_error;
    return n;
}

/* .conf/<name minus its extension>.loc */
static void loc_path(const char *name, char *d, int max)
{
    int n = cat(d, 0, max, UVM2_SD_CONF);
    const char *p = name;
    const char *dot = 0;
    for (const char *q = p; *q; q++) if (*q == '.') dot = q;
    while (*p && p != dot && n < max - 5) d[n++] = *p++;
    d[n] = 0;
    cat(d, n, max, ".loc");
}

/* Depth-first over the card, an explicit stack of open folders (no recursion: this runs on
 * the boot stack). Leaves the path of the first match in `out`.
 *
 * The stack is the caller's (UVM2_SD_FIND_SCRATCH bytes, 8-aligned). The romset loader lends
 * its buffer, which holds nothing while the scan runs (the reads before it failed, the one
 * after it overwrites it): 648 bytes that would otherwise stay taken in SRAM the whole game,
 * and aztarac's image did not fit by 1196. */
struct scan_stack {
    uvm2_sd_dir dirs[UVM2_SD_FIND_DEPTH];
    int         base[UVM2_SD_FIND_DEPTH];   /* length of `out` at each level */
};
_Static_assert(sizeof(struct scan_stack) <= UVM2_SD_FIND_SCRATCH, "UVM2_SD_FIND_SCRATCH too small");
_Static_assert(sizeof(struct scan_stack) <= UVM2_ROMZIP_MAX, "the scan's stack must fit in the romset buffer");

static int scan(const char *want, char *out, int max, void *scratch)
{
    struct scan_stack *const st = (struct scan_stack *)scratch;
    uvm2_sd_dir *const dirs = st->dirs;
    int *const base = st->base;
    char name[UVM2_ROMZIP_PATH_MAX];
    int depth = 0;
    out[0] = 0;
    if (!uvm2_sd_dir_open("", &dirs[0])) return 0;
    base[0] = 0;
    for (;;) {
        uint32_t size;
        const int k = uvm2_sd_dir_next(&dirs[depth], name, sizeof name, &size);
        if (k == UVM2_SD_DIR_END) {
            uvm2_sd_dir_close(&dirs[depth]);
            if (depth == 0) return 0;
            depth--;
            continue;
        }
        const int b = base[depth];
        if (k == UVM2_SD_DIR_FILE) {
            if (name_eq(name, want) && cat(out, b, max, name) > 0) {
                for (; depth >= 0; depth--) uvm2_sd_dir_close(&dirs[depth]);
                return 1;
            }
            continue;
        }
        /* a folder: in, unless too deep or its path does not fit */
        if (depth + 1 >= UVM2_SD_FIND_DEPTH) continue;
        int n = cat(out, b, max, name);
        if (n < 0 || (n = cat(out, n, max, "/")) < 0) continue;
        if (!uvm2_sd_dir_open(out, &dirs[depth + 1])) continue;
        base[++depth] = n;
    }
}

/* The path that worked, to the .loc: overwrite in place if there is one, create it if not. */
static void save_loc(const char *loc, const char *path)
{
    uint32_t n = 0;
    while (path[n]) n++;
    if (!uvm2_sd_overwrite(loc, (const unsigned char *)path, n))
        uvm2_sd_create(loc, (const unsigned char *)path, n);
}

/* The .loc's path, trimmed of the padding uvm2_sd_create puts after it. 0 if there is none. */
static uint32_t read_loc(const char *loc, char *path, int max)
{
    uint32_t n = uvm2_sd_read_from(loc, (unsigned char *)path, (uint32_t)max - 1, 0);
    while (n && (path[n - 1] == ' ' || path[n - 1] == '\n' || path[n - 1] == '\r')) n--;
    path[n] = 0;
    return n;
}

static int exists(const char *path)
{
    uvm2_sd_file f;
    if (!uvm2_sd_open(path, &f)) return 0;
    uvm2_sd_close(&f);
    return 1;
}

/* uvm2_sd.h. The romset's order, for any file a game carries beside its image, plus the
 * card's root, where such files lived before .roms/ (KUROISHI.PAK). */
int uvm2_sd_find(const char *name, char *out, int max, void *scratch)
{
    char loc[48];
    loc_path(name, loc, sizeof loc);
    if (read_loc(loc, out, max) && exists(out)) return 1;
    if (cat(out, cat(out, 0, max, UVM2_SD_ROMS), max, name) > 0 && exists(out)) return 1;
    if (cat(out, cat(out, 0, max, UVM2_SD_ROMS_OLD), max, name) > 0 && exists(out)) return 1;
    if (cat(out, 0, max, name) > 0 && exists(out)) return 1;
    if (scan(name, out, max, scratch) && exists(out)) { save_loc(loc, out); return 1; }
    out[0] = 0;
    return 0;
}

void uvm2_romzip_load(void)
{
    uvm2_romzip_desc = (unsigned long)(uintptr_t)s_desc;
    uvm2_romzip_path[0] = 0;

    if (!&game_romset_name || !game_romset_name[0]) return;   /* game with no romset */

    char path[UVM2_ROMZIP_PATH_MAX], loc[48];
    loc_path(game_romset_name, loc, sizeof loc);

    /* 1. where it was last time */
    if (read_loc(loc, path, sizeof path) && (uvm2_romzip_bytes = try_path(path))) goto found;
    /* 2. .roms/<romset>, then the visible roms/ of an older card. The scan below skips every
     *    name starting with '.', so the hidden folder has to be asked for by name. */
    if (cat(path, cat(path, 0, sizeof path, UVM2_SD_ROMS), sizeof path, game_romset_name) > 0
        && (uvm2_romzip_bytes = try_path(path))) goto found;
    if (cat(path, cat(path, 0, sizeof path, UVM2_SD_ROMS_OLD), sizeof path, game_romset_name) > 0
        && (uvm2_romzip_bytes = try_path(path))) goto found;
    /* 3. anywhere, and remember it */
    if (scan(game_romset_name, path, sizeof path, s_zip) && (uvm2_romzip_bytes = try_path(path))) {
        save_loc(loc, path);
        goto found;
    }
    return;                      /* not on the card: the game paints its "no romset" sign */

found:
    cat(uvm2_romzip_path, 0, sizeof uvm2_romzip_path, path);
    s_desc[1] = (uintptr_t)s_zip;
    s_desc[2] = uvm2_romzip_bytes;
    s_desc[0] = 0x315A4D52u;                   /* 'RMZ1' — written last */
}
