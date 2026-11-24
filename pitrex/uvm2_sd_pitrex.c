/* uvm2_sd_pitrex.c — the part of uvm2_sd.h the PiTrex build uses, over the baremetal
 * kernel's own FatFs.
 *
 * On the cartridges uvm2_sd.c carries its own SD driver and FatFs. On a PiTrex the card is
 * already mounted by the kernel that booted from it (bareMetalMain.c), read/write, so this
 * is only the SDK's contract mapped onto f_open and friends.
 *
 * Only what the config and the wizard need is here. The rest of uvm2_sd.h (romsets, the
 * open-file reader, streams) arrives with the first game that asks for it.
 */
#include <stdint.h>
#include <string.h>
#include "uvm2_sd.h"
#include "baremetal/ff.h"

/* A config file is one sector of text, so that the cartridges can rewrite it in place
 * (uvm2_sd.h). The same size here keeps a card interchangeable between boards. */
#define CONFIG_BYTES 512u

unsigned char uvm2_sd_sector[520];
int uvm2_sd_error = UVM2_SD_OK;

/* THE GAMES' OWN FILES ARE IN gamedata/ (hardware/pitrex/prune_card.py lays the card out that
 * way: games/ for the programs, gamedata/ for what they read, roms/ for the romsets and the
 * cartridges). A game asks for its folder by its own name — Hidden Zoo for "HZOO/...", as it
 * does on the cartridges, where it is at the top. So a relative path that is not on the card
 * is looked for again under gamedata/. Reading only: what a game writes goes where it says. */
#define GAMES_DIR "gamedata/"
#define GAMES_PATH_MAX 128

/* Whether a path is on the card. "" is the top, which always is; a folder may come with its
 * trailing '/' (the card-wide search, uvm2_romzip.c's scan, opens "kuroishi/"), which f_stat
 * does not take. Both went unseen the first time: the search started in gamedata/ instead of
 * at the top, and never got into a folder — Kuroishi found no KUROISHI.PAK and played its
 * effects without music (2026-11-24). */
static int on_card(const char *path)
{
    FILINFO info;
    char bare[GAMES_PATH_MAX];
    size_t n = strlen(path);
    if (n == 0) return 1;
    if (n >= sizeof bare) return 0;
    memcpy(bare, path, n + 1u);
    while (n > 0 && bare[n - 1u] == '/') bare[--n] = 0;
    return n == 0 || f_stat(bare, &info) == FR_OK;
}

static const char *in_games(const char *path, char *buf, size_t max)
{
    const size_t n = sizeof GAMES_DIR - 1u;
    if (!path || path[0] == '/' || on_card(path)) return path;
    if (n + strlen(path) >= max) return path;
    memcpy(buf, GAMES_DIR, n);
    strcpy(buf + n, path);
    return on_card(buf) ? buf : path;
}

int uvm2_sd_init(void) { return 1; }      /* mounted by the kernel before main() */

uint32_t uvm2_sd_read_from(const char *path, unsigned char *dst, uint32_t max, uint32_t from)
{
    char where[GAMES_PATH_MAX];
    path = in_games(path, where, sizeof where);
    FIL f;
    UINT got = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    if (f_lseek(&f, from) == FR_OK) f_read(&f, dst, (UINT)max, &got);
    f_close(&f);
    return (uint32_t)got;
}

uint32_t uvm2_sd_read(const char *path, unsigned char *dst, uint32_t max)
{
    char where[GAMES_PATH_MAX];
    path = in_games(path, where, sizeof where);
    FIL f;
    UINT got = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    /* Not fitting IS a failure here, unlike uvm2_sd_read_from. */
    if (f_size(&f) <= max) f_read(&f, dst, (UINT)max, &got);
    f_close(&f);
    return (uint32_t)got;
}

/* A file that lives with the romsets: .roms/<name>, else roms/<name>. */
uint32_t uvm2_sd_read_rom(const char *name, unsigned char *dst, uint32_t max)
{
    static const char *const dirs[2] = { UVM2_SD_ROMS, UVM2_SD_ROMS_OLD };
    char path[96];
    for (int d = 0; d < 2; d++) {
        const size_t n = strlen(dirs[d]);
        if (n + strlen(name) >= sizeof path) continue;
        memcpy(path, dirs[d], n);
        strcpy(path + n, name);
        const uint32_t got = uvm2_sd_read(path, dst, max);
        if (got) return got;
    }
    return 0;
}

/* Create every folder on the way to `path`. An existing one is not an error. */
static void make_folders(const char *path)
{
    char dir[64];
    for (uint32_t i = 0; path[i] && i < sizeof dir - 1; i++) {
        if (path[i] == '/' && i > 0) {
            memcpy(dir, path, i);
            dir[i] = 0;
            f_mkdir(dir);
        }
    }
}

static int write_file(const char *path, const unsigned char *data, uint32_t n, BYTE mode)
{
    FIL f;
    UINT put = 0;
    if (f_open(&f, path, mode) != FR_OK) return 0;
    FRESULT r = f_write(&f, data, (UINT)n, &put);
    f_close(&f);
    return r == FR_OK && put == (UINT)n;
}

/* The content padded with spaces to one sector, ending in a newline. */
static void pad_sector(const unsigned char *data, uint32_t n)
{
    if (n > CONFIG_BYTES) n = CONFIG_BYTES;
    memset(uvm2_sd_sector, ' ', CONFIG_BYTES);
    memcpy(uvm2_sd_sector, data, n);
    uvm2_sd_sector[CONFIG_BYTES - 1] = '\n';
}

int uvm2_sd_create(const char *path, const unsigned char *data, uint32_t n)
{
    make_folders(path);
    pad_sector(data, n);
    return write_file(path, uvm2_sd_sector, CONFIG_BYTES, FA_WRITE | FA_CREATE_ALWAYS);
}

int uvm2_sd_overwrite(const char *path, const unsigned char *data, uint32_t n)
{
    FILINFO info;
    /* The caller tells these two apart from a card that failed: they mean "create it". */
    if (f_stat(path, &info) != FR_OK)  { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    if (info.fsize < CONFIG_BYTES)     { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    uvm2_sd_error = UVM2_SD_OK;
    pad_sector(data, n);
    /* FA_OPEN_EXISTING: it does not create, and writing the first sector of a file that is
     * at least that long does not resize it. */
    return write_file(path, uvm2_sd_sector, CONFIG_BYTES, FA_WRITE | FA_OPEN_EXISTING);
}

int uvm2_sd_write(const char *path, const unsigned char *data, uint32_t n)
{
    make_folders(path);
    return write_file(path, data, n, FA_WRITE | FA_CREATE_ALWAYS);
}

int uvm2_sd_remove(const char *path)
{
    return f_unlink(path) == FR_OK;
}

/* A file written in pieces (uvm2_sd.h): what a package's installer copies out. One at a time. */
static FIL s_stream;
static int s_streaming;
int uvm2_sd_stream_open(const char *path)
{
    if (s_streaming) { f_close(&s_stream); s_streaming = 0; }
    make_folders(path);
    if (f_open(&s_stream, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return 0;
    s_streaming = 1;
    return 1;
}
int uvm2_sd_stream_put(const unsigned char *data, uint32_t n)
{
    UINT put = 0;
    return s_streaming && f_write(&s_stream, data, (UINT)n, &put) == FR_OK && put == n;
}
int uvm2_sd_stream_close(void)
{
    if (!s_streaming) return 0;
    s_streaming = 0;
    return f_close(&s_stream) == FR_OK;
}

/* ── A FILE READ IN PIECES (uvm2_sd_open / next / seek / close) ───────────────
 *
 * The contract's handle keeps FatFs's FIL in 128 opaque bytes. This kernel's FatFs is built
 * with a sector buffer inside each FIL (_FS_TINY 0), which does not fit there, so the FILs
 * live here and the handle carries which one it is. A handful: a game streams a romset, a
 * pack of sounds, a film — never many at once. */
#define OPEN_FILES 4u
#define OPEN_MAGIC 0x55325344u            /* "U2SD" */

static FIL     s_fil[OPEN_FILES];
static uint8_t s_fil_used[OPEN_FILES];

static FIL *fil_of(const uvm2_sd_file *f)
{
    const uint32_t slot = (uint32_t)f->fil[0];
    return (f->magic == OPEN_MAGIC && slot < OPEN_FILES && s_fil_used[slot]) ? &s_fil[slot] : 0;
}

void uvm2_sd_close(uvm2_sd_file *f)
{
    FIL *fil = fil_of(f);
    if (fil) {
        f_close(fil);
        s_fil_used[(uint32_t)f->fil[0]] = 0;
    }
    f->magic = 0;
    f->ok = 0;
}

int uvm2_sd_open(const char *path, uvm2_sd_file *f)
{
    uint32_t slot;
    char where[GAMES_PATH_MAX];
    path = in_games(path, where, sizeof where);
    uvm2_sd_close(f);                      /* a reopen closes the old one, as on the cartridges */
    for (slot = 0; slot < OPEN_FILES && s_fil_used[slot]; slot++) { }
    if (slot == OPEN_FILES) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    if (f_open(&s_fil[slot], path, FA_READ) != FR_OK) { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    s_fil_used[slot] = 1;
    f->fil[0]  = slot;
    f->cluster = 0;
    f->sec     = 0;
    f->pos     = 0;
    f->len     = (uint32_t)f_size(&s_fil[slot]);
    f->magic   = OPEN_MAGIC;
    f->ok      = 1;
    uvm2_sd_error = UVM2_SD_OK;
    return 1;
}

uint32_t uvm2_sd_next(uvm2_sd_file *f, unsigned char *dst, uint32_t max)
{
    FIL *fil = fil_of(f);
    UINT got = 0;
    if (!fil || !f->ok) return 0;
    if (f_read(fil, dst, (UINT)max, &got) != FR_OK || got == 0) f->ok = 0;
    f->pos += (uint32_t)got;
    return (uint32_t)got;
}

int uvm2_sd_seek(uvm2_sd_file *f, uint32_t pos)
{
    FIL *fil = fil_of(f);
    if (!fil) return 0;
    if (pos > f->len) pos = f->len;
    if (f_lseek(fil, pos) != FR_OK) { f->ok = 0; return 0; }
    f->pos = pos;
    f->ok  = 1;
    return 1;
}

/* ── A FOLDER, LISTED (uvm2_sd_dir_open / next / close) ───────────────────────
 * Same arrangement as the open files above: FatFs's DIR lives here, the handle says which.
 * The romset search walks UVM2_SD_FIND_DEPTH folders deep, one open per level. */
#define OPEN_DIRS      (UVM2_SD_FIND_DEPTH + 2u)
#define OPEN_DIR_MAGIC 0x55324452u        /* "U2DR" */

static DIR     s_dir[OPEN_DIRS];
static uint8_t s_dir_used[OPEN_DIRS];

static DIR *dir_of(const uvm2_sd_dir *d)
{
    const uint32_t slot = (uint32_t)d->dir[0];
    return (d->magic == OPEN_DIR_MAGIC && slot < OPEN_DIRS && s_dir_used[slot]) ? &s_dir[slot] : 0;
}

void uvm2_sd_dir_close(uvm2_sd_dir *d)
{
    DIR *dir = dir_of(d);
    if (dir) {
        f_closedir(dir);
        s_dir_used[(uint32_t)d->dir[0]] = 0;
    }
    d->magic = 0;
}

int uvm2_sd_dir_open(const char *path, uvm2_sd_dir *d)
{
    uint32_t slot;
    char where[GAMES_PATH_MAX];
    path = in_games(path, where, sizeof where);
    d->magic = 0;
    for (slot = 0; slot < OPEN_DIRS && s_dir_used[slot]; slot++) { }
    if (slot == OPEN_DIRS) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    /* "" and "/" are the root in the contract; FatFs wants "/" for it. A folder comes from the
     * card-wide search with its trailing '/' ("gamedata/"), which this kernel's FatFs does not
     * open: the search never got below the top, and Kuroishi's pack in gamedata/kuroishi/ was
     * "not on the card" (its report, 2026-11-24: found nowhere in 74 ms, 35 pieces missing). */
    char bare[GAMES_PATH_MAX];
    size_t n = strlen(path);
    if (n >= sizeof bare) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    memcpy(bare, path, n + 1u);
    while (n > 1u && bare[n - 1u] == '/') bare[--n] = 0;
    if (f_opendir(&s_dir[slot], (bare[0] == 0) ? "/" : bare) != FR_OK) { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    s_dir_used[slot] = 1;
    d->dir[0] = slot;
    d->count  = 0;
    d->magic  = OPEN_DIR_MAGIC;
    uvm2_sd_error = UVM2_SD_OK;
    return 1;
}

int uvm2_sd_dir_next(uvm2_sd_dir *d, char *name, uint32_t max, uint32_t *size)
{
    DIR *dir = dir_of(d);
    FILINFO info;
    if (!dir || max == 0) return UVM2_SD_DIR_END;
    for (;;) {
        if (f_readdir(dir, &info) != FR_OK || info.fname[0] == 0) return UVM2_SD_DIR_END;
        /* Hidden, system and dot names are skipped, as on the cartridges. */
        if (info.fname[0] == '.' || (info.fattrib & (AM_HID | AM_SYS))) continue;
        strncpy(name, info.fname, max - 1u);
        name[max - 1u] = 0;
        if (size) *size = (info.fattrib & AM_DIR) ? 0u : (uint32_t)info.fsize;
        d->count++;
        return (info.fattrib & AM_DIR) ? UVM2_SD_DIR_FOLDER : UVM2_SD_DIR_FILE;
    }
}
