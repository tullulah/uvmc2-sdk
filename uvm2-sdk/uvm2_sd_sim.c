/* uvm2_sd_sim.c — uvm2_sd.h for the WASM simulator: the card is a folder in emscripten's file
 * system, /sd, which Vectrex Studio fills from ~/VectrexStudio/sd before main() runs.
 *
 * WHY. A game that reads its card (a menu of games, saved games, music) would otherwise need
 * two versions of that code, or say "NO SD" in the simulator. With this the same calls work in
 * both, and only the Makefile's sim rule changes: compile this file instead of uvm2_sd.c, and
 * export _uvm2_sd_sim so the IDE knows to copy the card in (it does not for games that never
 * read it: the copy is megabytes).
 *
 * The IDE hands the files over by lower-case path, so every path is lower-cased here: the
 * card stays case-insensitive as on FAT, and names come back in lower case.
 */
#include "uvm2_sd.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define SIM_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define SIM_EXPORT
#endif

#ifndef CARD_ROOT
#define CARD_ROOT "/sd"         /* -DCARD_ROOT=\"...\" points a host test elsewhere */
#endif

int uvm2_sd_error = UVM2_SD_OK;
struct uvm2_sd_diag uvm2_sd_diag;

/* The IDE looks for this export to decide whether to copy the card in. Returns 1. */
SIM_EXPORT int uvm2_sd_sim(void) { return 1; }

int uvm2_sd_init(void)
{
    struct stat st;
    return stat(CARD_ROOT, &st) == 0 && S_ISDIR(st.st_mode);
}

/* "<root>/<path in lower case>", without a trailing '/'. 0 if it does not fit. */
static int card_path(const char *path, char *out, size_t max)
{
    size_t n = strlen(CARD_ROOT);
    if (n + 1 + strlen(path) + 1 > max) return 0;
    memcpy(out, CARD_ROOT, n);
    if (*path && *path != '/') out[n++] = '/';
    for (; *path; path++) out[n++] = (*path >= 'A' && *path <= 'Z') ? *path + ('a' - 'A') : *path;
    while (n > 1 && out[n - 1] == '/') n--;
    out[n] = 0;
    return 1;
}

static FILE *open_card(const char *path, const char *mode)
{
    char p[256];
    uvm2_sd_error = UVM2_SD_OK;
    if (!uvm2_sd_init()) { uvm2_sd_error = UVM2_SD_NO_CARD; return NULL; }
    if (!card_path(path, p, sizeof p)) { uvm2_sd_error = UVM2_SD_MISSING; return NULL; }
    FILE *f = fopen(p, mode);
    if (!f) uvm2_sd_error = mode[0] == 'r' ? UVM2_SD_MISSING : UVM2_SD_IO_ERROR;
    return f;
}

static uint32_t file_len(FILE *f)
{
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    return n < 0 ? 0 : (uint32_t)n;
}

uint32_t uvm2_sd_read(const char *path, unsigned char *dst, uint32_t max)
{
    FILE *f = open_card(path, "rb");
    if (!f) return 0;
    uint32_t len = file_len(f);
    if (len > max) { fclose(f); uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    uint32_t got = (uint32_t)fread(dst, 1, len, f);
    fclose(f);
    if (got != len) { uvm2_sd_error = UVM2_SD_IO_ERROR; return 0; }
    return got;
}

uint32_t uvm2_sd_read_from(const char *path, unsigned char *dst, uint32_t max, uint32_t from)
{
    FILE *f = open_card(path, "rb");
    if (!f) return 0;
    uint32_t len = file_len(f), got = 0;
    if (from < len) {
        fseek(f, (long)from, SEEK_SET);
        got = (uint32_t)fread(dst, 1, len - from < max ? len - from : max, f);
    }
    fclose(f);
    return got;
}

/* Missing folders are created, as on the card. */
static void make_parents(const char *path)
{
    char p[256];
    if (!card_path(path, p, sizeof p)) return;
    for (char *s = p + 1; *s; s++) {
        if (*s != '/') continue;
        *s = 0;
        mkdir(p, 0777);
        *s = '/';
    }
}

int uvm2_sd_write2(const char *path, const unsigned char *a, uint32_t na,
                   const unsigned char *b, uint32_t nb)
{
    make_parents(path);
    FILE *f = open_card(path, "wb");
    if (!f) return 0;
    int ok = fwrite(a, 1, na, f) == na && (!nb || fwrite(b, 1, nb, f) == nb);
    fclose(f);
    if (!ok) uvm2_sd_error = UVM2_SD_TOO_BIG;
    return ok;
}

int uvm2_sd_write(const char *path, const unsigned char *data, uint32_t n)
{
    return uvm2_sd_write2(path, data, n, 0, 0);
}

int uvm2_sd_create(const char *path, const unsigned char *data, uint32_t n)
{
    unsigned char sector[512];
    if (n > 512) n = 512;
    memcpy(sector, data, n);
    memset(sector + n, ' ', 512 - n);
    sector[511] = '\n';
    return uvm2_sd_write(path, sector, 512);
}

int uvm2_sd_overwrite(const char *path, const unsigned char *data, uint32_t n)
{
    FILE *f = open_card(path, "r+b");
    if (!f) { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    if (file_len(f) < 512 || n > 512) { fclose(f); uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    int ok = fwrite(data, 1, n, f) == n;
    fclose(f);
    return ok;
}

/* ── AN OPEN FILE: the FILE * rides in fil[0] ───────────────────────────────────────── */
#define UVM2_SD_OPEN_MAGIC 0x4E45504Fu   /* 'OPEN', as in uvm2_sd.c */
#define FILE_OF(f) (*(FILE **)(void *)(f)->fil)

void uvm2_sd_close(uvm2_sd_file *f)
{
    if (f->magic != UVM2_SD_OPEN_MAGIC) return;
    fclose(FILE_OF(f));
    f->magic = 0;
    f->ok = 0;
    if (uvm2_sd_diag.streams) uvm2_sd_diag.streams--;
}

int uvm2_sd_open(const char *path, uvm2_sd_file *f)
{
    uvm2_sd_close(f);
    f->cluster = f->sec = f->pos = f->len = 0;
    f->ok = 0;
    f->magic = 0;
    FILE *h = open_card(path, "rb");
    if (!h) return 0;
    FILE_OF(f) = h;
    f->len = file_len(h);
    f->ok = 1;
    f->magic = UVM2_SD_OPEN_MAGIC;
    uvm2_sd_diag.streams++;
    return 1;
}

int uvm2_sd_seek(uvm2_sd_file *f, uint32_t pos)
{
    if (f->magic != UVM2_SD_OPEN_MAGIC || pos > f->len) return 0;
    fseek(FILE_OF(f), (long)pos, SEEK_SET);
    f->pos = pos;
    f->ok = 1;
    return 1;
}

uint32_t uvm2_sd_next(uvm2_sd_file *f, unsigned char *dst, uint32_t max)
{
    if (!f->ok || f->magic != UVM2_SD_OPEN_MAGIC) return 0;
    uint32_t want = f->len - f->pos;
    if (want > max) want = max;
    uint32_t got = want ? (uint32_t)fread(dst, 1, want, FILE_OF(f)) : 0;
    f->pos += got;
    if (got != want) { f->ok = 0; uvm2_sd_error = UVM2_SD_IO_ERROR; }
    return got;
}

/* ── A FOLDER: the DIR * in dir[0], and in dir[1] the folder's own path (to stat entries) ── */
#define UVM2_SD_DIR_MAGIC 0x52494444u    /* 'DDIR', as in uvm2_sd.c */
#define DIR_OF(d) (*(DIR **)(void *)(d)->dir)
#define DIR_PATH(d) (*(char **)(void *)&(d)->dir[1])

void uvm2_sd_dir_close(uvm2_sd_dir *d)
{
    if (d->magic != UVM2_SD_DIR_MAGIC) return;
    closedir(DIR_OF(d));
    free(DIR_PATH(d));
    d->magic = 0;
    if (uvm2_sd_diag.streams) uvm2_sd_diag.streams--;
}

int uvm2_sd_dir_open(const char *path, uvm2_sd_dir *d)
{
    uvm2_sd_dir_close(d);
    d->magic = 0;
    d->count = 0;
    uvm2_sd_error = UVM2_SD_OK;
    if (!uvm2_sd_init()) { uvm2_sd_error = UVM2_SD_NO_CARD; return 0; }
    char p[256];
    if (!card_path(path, p, sizeof p)) { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    DIR *h = opendir(p);
    if (!h) { uvm2_sd_error = UVM2_SD_MISSING; return 0; }
    char *copy = malloc(strlen(p) + 1);
    if (!copy) { closedir(h); uvm2_sd_error = UVM2_SD_IO_ERROR; return 0; }
    strcpy(copy, p);
    DIR_OF(d) = h;
    DIR_PATH(d) = copy;
    d->magic = UVM2_SD_DIR_MAGIC;
    uvm2_sd_diag.streams++;
    return 1;
}

int uvm2_sd_dir_next(uvm2_sd_dir *d, char *name, uint32_t max, uint32_t *size)
{
    if (max) name[0] = 0;
    if (size) *size = 0;
    if (d->magic != UVM2_SD_DIR_MAGIC) return UVM2_SD_DIR_END;
    struct dirent *e;
    while ((e = readdir(DIR_OF(d))) != NULL) {
        if (e->d_name[0] == '.') continue;
        char p[256];
        struct stat st;
        if (snprintf(p, sizeof p, "%s/%s", DIR_PATH(d), e->d_name) >= (int)sizeof p || stat(p, &st) != 0)
            continue;
        if (max) {
            uint32_t i = 0;
            for (; e->d_name[i] && i < max - 1; i++) name[i] = e->d_name[i];
            name[i] = 0;
        }
        const int folder = S_ISDIR(st.st_mode);
        if (size && !folder) *size = (uint32_t)st.st_size;
        d->count++;
        return folder ? UVM2_SD_DIR_FOLDER : UVM2_SD_DIR_FILE;
    }
    return UVM2_SD_DIR_END;
}
