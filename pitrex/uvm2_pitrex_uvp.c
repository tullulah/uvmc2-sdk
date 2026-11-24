/* uvm2_pitrex_uvp.c — see uvm2_pitrex_uvp.h. Over the kernel's FatFs; a package is a STORED zip
 * (tools/uvp.py), so a file is copied out in pieces and nothing needs an inflater. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "baremetal/ff.h"
#include "uvm2_toml.h"
#include "uvm2_pitrex_uvp.h"

#define UVP_EXT        ".uvp"
#define PATH_MAX_UVP   128
#define COPY_CHUNK     65536u
#define ZIP_TAIL_MAX   (65535u + 22u)      /* the end record and the longest comment */
#define ZIP_END_SIG    0x06054b50u
#define ZIP_DIR_SIG    0x02014b50u
#define ZIP_LOCAL_SIG  0x04034b50u
#define MANIFEST       "manifest.toml"

#if defined(__ARM_ARCH) && __ARM_ARCH < 7
#define THIS_BOARD     "pitrex_zero1"
#else
#define THIS_BOARD     "pitrex_zero2"
#endif

/* ── small pieces ── */
static uint32_t le16(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8; }
static uint32_t le32(const uint8_t *p) { return le16(p) | le16(p + 2) << 16; }

static uint32_t s_crc_table[256];
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    if (!s_crc_table[1])
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            s_crc_table[i] = c;
        }
    crc = ~crc;
    while (n--) crc = s_crc_table[(crc ^ *p++) & 0xFFu] ^ (crc >> 8);
    return ~crc;
}

static int ends_with(const char *s, const char *tail)
{
    size_t a = strlen(s), b = strlen(tail);
    if (a < b) return 0;
    for (size_t i = 0; i < b; i++) {
        char x = s[a - b + i], y = tail[i];
        if (x >= 'A' && x <= 'Z') x += 32;
        if (x != y) return 0;
    }
    return 1;
}

/* every folder on the way to `path` */
static void make_folders(const char *path)
{
    char dir[PATH_MAX_UVP];
    for (size_t i = 1; path[i] && i < sizeof dir - 1; i++)
        if (path[i] == '/') { memcpy(dir, path, i); dir[i] = 0; f_mkdir(dir); }
}

/* where a file of the package goes on this card; 0 if it does not go on a PiTrex card */
static int destination(const char *inner, char *out, size_t max)
{
    static const struct { const char *from, *to; } MAP[] = {
        { "pitrex/zero2/", "/games/" },
        { "pitrex/zero1/", "/piZero1/" },
        { "gamedata/",     "/gamedata/" },
    };
    for (unsigned i = 0; i < sizeof MAP / sizeof MAP[0]; i++) {
        const size_t n = strlen(MAP[i].from);
        if (strncmp(inner, MAP[i].from, n) == 0 && inner[n]) {
            snprintf(out, max, "%s%s", MAP[i].to, inner + n);
            return 1;
        }
    }
    return 0;
}

/* ── the manifest ── */
typedef struct {
    char id[UVM2_TOML_NAME_MAX], title[UVM2_TOML_VALUE_MAX], menu[UVM2_TOML_NAME_MAX];
    char image[UVM2_TOML_NAME_MAX];       /* this board's image, its file name; "" if none */
    int  entries;
} manifest_t;

static const char *base_name(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

static int manifest_cb(void *user, const char *table, int index, const char *key, const char *value)
{
    manifest_t *m = user;
    (void)index;
    if (strcmp(table, "package") == 0) {
        if (strcmp(key, "id") == 0)    snprintf(m->id, sizeof m->id, "%s", value);
        if (strcmp(key, "title") == 0) snprintf(m->title, sizeof m->title, "%s", value);
        if (strcmp(key, "menu") == 0)  snprintf(m->menu, sizeof m->menu, "%s", value);
    }
    if (strcmp(table, "targets") == 0 && strcmp(key, THIS_BOARD) == 0)
        snprintf(m->image, sizeof m->image, "%s", base_name(value));
    if (strcmp(table, "entry") == 0 && strcmp(key, "title") == 0) m->entries++;
    return 1;
}

/* ── one package ── */
typedef struct {
    FIL      f;
    uint8_t *dir;                 /* the central directory */
    uint32_t dir_n, count;
} zip_t;

static int zip_open(zip_t *z, const char *path)
{
    memset(z, 0, sizeof *z);
    if (f_open(&z->f, path, FA_READ) != FR_OK) return 0;
    const uint32_t size = (uint32_t)f_size(&z->f);
    const uint32_t tail = size < ZIP_TAIL_MAX ? size : ZIP_TAIL_MAX;
    uint8_t *t = malloc(tail);
    UINT got = 0;
    int ok = 0;
    if (t && f_lseek(&z->f, size - tail) == FR_OK && f_read(&z->f, t, tail, &got) == FR_OK && got == tail) {
        for (int32_t i = (int32_t)tail - 22; i >= 0; i--) {
            if (le32(t + i) != ZIP_END_SIG) continue;
            z->count = le16(t + i + 10);
            z->dir_n = le32(t + i + 12);
            const uint32_t at = le32(t + i + 16);
            z->dir = malloc(z->dir_n);
            ok = z->dir && f_lseek(&z->f, at) == FR_OK && f_read(&z->f, z->dir, z->dir_n, &got) == FR_OK && got == z->dir_n;
            break;
        }
    }
    free(t);
    if (!ok) { free(z->dir); z->dir = 0; f_close(&z->f); }
    return ok;
}

static void zip_close(zip_t *z) { free(z->dir); z->dir = 0; f_close(&z->f); }

/* the entries, one by one: name, method, crc, size, and where the local header is */
typedef struct { char name[PATH_MAX_UVP]; uint32_t method, crc, size, local; } entry_t;

static int zip_entry(const zip_t *z, uint32_t *at, entry_t *e)
{
    if (*at + 46u > z->dir_n || le32(z->dir + *at) != ZIP_DIR_SIG) return 0;
    const uint8_t *d = z->dir + *at;
    const uint32_t name_n = le16(d + 28), extra = le16(d + 30), note = le16(d + 32);
    e->method = le16(d + 10);
    e->crc    = le32(d + 16);
    e->size   = le32(d + 24);
    e->local  = le32(d + 42);
    const uint32_t n = name_n < sizeof e->name - 1 ? name_n : sizeof e->name - 1;
    memcpy(e->name, d + 46, n);
    e->name[n] = 0;
    *at += 46u + name_n + extra + note;
    return 1;
}

static int zip_seek_data(zip_t *z, const entry_t *e)
{
    uint8_t h[30];
    UINT got = 0;
    if (f_lseek(&z->f, e->local) != FR_OK || f_read(&z->f, h, 30, &got) != FR_OK || got != 30) return 0;
    if (le32(h) != ZIP_LOCAL_SIG) return 0;
    return f_lseek(&z->f, e->local + 30u + le16(h + 26) + le16(h + 28)) == FR_OK;
}

static char *zip_read_all(zip_t *z, const char *name, uint32_t *n)
{
    uint32_t at = 0;
    entry_t e;
    while (zip_entry(z, &at, &e)) {
        if (strcmp(e.name, name) != 0) continue;
        char *buf = (e.method == 0) ? malloc(e.size + 1u) : 0;
        UINT got = 0;
        if (!buf || !zip_seek_data(z, &e) || f_read(&z->f, buf, e.size, &got) != FR_OK || got != e.size) {
            free(buf);
            return 0;
        }
        buf[e.size] = 0;
        *n = e.size;
        return buf;
    }
    return 0;
}

/* copies one entry out; 1 if it arrived whole (its CRC matches) */
static int extract(zip_t *z, const entry_t *e, const char *dst, uint8_t *chunk,
                   uvm2_uvp_progress progress, const char *title, uint32_t *done, uint32_t total)
{
    FIL out;
    uint32_t left = e->size, crc = 0;
    if (e->method != 0 || !zip_seek_data(z, e)) return 0;
    make_folders(dst);
    if (f_open(&out, dst, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return 0;
    while (left) {
        UINT got = 0, put = 0;
        const UINT want = left < COPY_CHUNK ? left : COPY_CHUNK;
        if (f_read(&z->f, chunk, want, &got) != FR_OK || got != want) break;
        crc = crc32_update(crc, chunk, got);
        if (f_write(&out, chunk, got, &put) != FR_OK || put != got) break;
        left -= got;
        *done += got;
        if (progress) progress(title, *done, total);
    }
    f_close(&out);
    if (left || crc != e->crc) { f_unlink(dst); return 0; }
    return 1;
}

static int record_path(const char *id, char *out, size_t max)
{
    return snprintf(out, max, UVP_RECORDS "/%s.toml", id) < (int)max;
}

static int install(const char *path, uvm2_uvp_progress progress)
{
    zip_t z;
    manifest_t m;
    uint32_t man_n = 0, at, total = 0, done = 0;
    entry_t e;
    char dst[PATH_MAX_UVP], rec[PATH_MAX_UVP];
    int ok = 1;
    if (!zip_open(&z, path)) return 0;
    char *man = zip_read_all(&z, MANIFEST, &man_n);
    memset(&m, 0, sizeof m);
    if (!man) { zip_close(&z); return 0; }
    uvm2_toml_read(man, man_n, manifest_cb, &m);
    if (!m.id[0] || !record_path(m.id, rec, sizeof rec)) { free(man); zip_close(&z); return 0; }

    /* the same game again (a new version): the old one's files go first; what it wrote stays */
    uvm2_uvp_uninstall(m.id);

    for (at = 0; zip_entry(&z, &at, &e); ) if (destination(e.name, dst, sizeof dst)) total += e.size;
    uint8_t *chunk = malloc(COPY_CHUNK);
    /* the list of what this put on the card, kept as the record's [installed] table */
    size_t list_max = 64u, list_n = 0;
    char *list = malloc(list_max);
    if (!chunk || !list) ok = 0;
    if (list) list[0] = 0;
    for (at = 0; ok && zip_entry(&z, &at, &e); ) {
        if (!destination(e.name, dst, sizeof dst)) continue;
        if (!extract(&z, &e, dst, chunk, progress, m.title, &done, total)) { ok = 0; break; }
        const size_t need = list_n + strlen(dst) + 8u;
        if (need > list_max) { list_max = need * 2u; char *l = realloc(list, list_max); if (!l) { ok = 0; break; } list = l; }
        list_n += (size_t)sprintf(list + list_n, "%s\"%s\"", list_n ? ", " : "", dst);
    }
    zip_close(&z);
    if (ok) {
        FIL r;
        UINT put;
        f_mkdir(UVP_RECORDS);
        ok = f_open(&r, rec, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK;
        if (ok) {
            f_write(&r, man, man_n, &put);
            static const char head[] = "\n[installed]\nfiles = [";
            f_write(&r, head, sizeof head - 1u, &put);
            f_write(&r, list, (UINT)list_n, &put);
            f_write(&r, "]\n", 2, &put);
            f_close(&r);
        }
    }
    if (!ok) {
        /* what arrived goes again: a half-installed game is worse than none */
        for (char *p = list; p && (p = strchr(p, '"')); ) {
            char *q = strchr(p + 1, '"');
            if (!q) break;
            *q = 0;
            f_unlink(p + 1);
            p = q + 1;
        }
    }
    free(chunk);
    free(list);
    free(man);
    return ok;
}

int uvm2_uvp_install_pending(uvm2_uvp_progress progress, char *failed, uint32_t failed_max)
{
    DIR d;
    FILINFO info;
    char names[8][PATH_MAX_UVP];
    int n = 0, installed = 0;
    if (failed && failed_max) failed[0] = 0;
    /* the names first: the folder changes as packages go */
    if (f_opendir(&d, "/") != FR_OK) return 0;
    while (n < 8 && f_readdir(&d, &info) == FR_OK && info.fname[0]) {
        if ((info.fattrib & AM_DIR) || info.fname[0] == '.' || !ends_with(info.fname, UVP_EXT)) continue;
        snprintf(names[n++], PATH_MAX_UVP, "/%s", info.fname);
    }
    f_closedir(&d);
    for (int i = 0; i < n; i++) {
        if (install(names[i], progress)) { f_unlink(names[i]); installed++; }
        else if (failed) snprintf(failed, failed_max, "%s", names[i] + 1);
    }
    return installed;
}

/* ── the menu ── */
typedef struct {
    manifest_t *m;
    uvm2_uvp_entry entry;
    void *user;
    char title[UVM2_TOML_VALUE_MAX], param[UVM2_TOML_VALUE_MAX], menu[UVM2_TOML_NAME_MAX];
    int index;
} lister_t;

static void flush_entry(lister_t *l)
{
    if (l->index < 0) return;
    l->entry(l->user, l->menu[0] ? l->menu : l->m->menu, l->title, l->m->id, l->m->image, l->param);
    l->index = -1;
}

static int entries_cb(void *user, const char *table, int index, const char *key, const char *value)
{
    lister_t *l = user;
    if (strcmp(table, "entry") != 0) return 1;
    if (index != l->index) {
        flush_entry(l);
        l->index = index;
        l->title[0] = l->param[0] = l->menu[0] = 0;
    }
    if (strcmp(key, "title") == 0) snprintf(l->title, sizeof l->title, "%s", value);
    if (strcmp(key, "param") == 0) snprintf(l->param, sizeof l->param, "%s", value);
    if (strcmp(key, "menu") == 0)  snprintf(l->menu, sizeof l->menu, "%s", value);
    return 1;
}

static char *read_file(const char *path, uint32_t *n)
{
    FIL f;
    UINT got = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    const uint32_t size = (uint32_t)f_size(&f);
    char *buf = malloc(size + 1u);
    if (buf && f_read(&f, buf, size, &got) == FR_OK && got == size) buf[size] = 0;
    else { free(buf); buf = 0; }
    f_close(&f);
    *n = size;
    return buf;
}

void uvm2_uvp_menu(uvm2_uvp_entry entry, void *user)
{
    DIR d;
    FILINFO info;
    char path[PATH_MAX_UVP];
    if (f_opendir(&d, UVP_RECORDS) != FR_OK) return;
    while (f_readdir(&d, &info) == FR_OK && info.fname[0]) {
        if ((info.fattrib & AM_DIR) || !ends_with(info.fname, ".toml")) continue;
        snprintf(path, sizeof path, UVP_RECORDS "/%s", info.fname);
        uint32_t n = 0;
        char *text = read_file(path, &n);
        if (!text) continue;
        manifest_t m;
        memset(&m, 0, sizeof m);
        uvm2_toml_read(text, n, manifest_cb, &m);
        if (m.id[0] && m.image[0]) {
            if (m.entries == 0) entry(user, m.menu, m.title, m.id, m.image, "");
            else {
                lister_t l = { &m, entry, user, "", "", "", -1 };
                uvm2_toml_read(text, n, entries_cb, &l);
                flush_entry(&l);
            }
        }
        free(text);
    }
    f_closedir(&d);
}

/* ── uninstalling ── */
static int files_cb(void *user, const char *table, int index, const char *key, const char *value)
{
    (void)user; (void)index;
    if (strcmp(table, "installed") == 0 && strcmp(key, "files") == 0) {
        char dir[PATH_MAX_UVP];
        f_unlink(value);
        /* the folders it leaves empty, up to (not including) the top ones */
        snprintf(dir, sizeof dir, "%s", value);
        for (char *s; (s = strrchr(dir, '/')) && s != dir; ) {
            *s = 0;
            if (strchr(dir + 1, '/') == 0) break;          /* /gamedata, /games: they stay */
            if (f_unlink(dir) != FR_OK) break;              /* not empty: the game's own files */
        }
    }
    return 1;
}

int uvm2_uvp_uninstall(const char *id)
{
    char rec[PATH_MAX_UVP];
    uint32_t n = 0;
    if (!record_path(id, rec, sizeof rec)) return 0;
    char *text = read_file(rec, &n);
    if (!text) return 0;
    uvm2_toml_read(text, n, files_cb, 0);
    free(text);
    f_unlink(rec);
    return 1;
}
