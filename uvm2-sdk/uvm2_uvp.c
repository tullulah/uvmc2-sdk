/* uvm2_uvp.c — see uvm2_uvp.h. Over uvm2_sd.h only, so the same code installs on every board;
 * a package is a STORED zip (tools/uvp.py), copied out in pieces, with no inflater and nothing
 * allocated: the buffers below are static and sized for a package, not for a game. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "uvm2_sd.h"
#include "uvm2_toml.h"
#include "uvm2_uvp.h"

#define UVP_EXT        ".uvp"
#define PATH_MAX_UVP   128
/* A PIECE COPIED AT A TIME. On the PiTrex the card driver (the baremetal tree's sd.c) sends one
 * read or write command per call — CMD18 / CMD25 for several blocks — and the card's cost is
 * per command, not per byte: copying in 4 KB pieces made installing crawl there, where the
 * UVMC2, with its own driver, was quick (2026-11-24). The Pi has memory to spare for a big
 * piece; the UVMC2's SRAM does not, and does not need it. */
#ifdef UVM2_PITREX
#define CHUNK          (256u * 1024u)
#else
#define CHUNK          4096u
#endif
#define ZIP_TAIL_MAX   1024u           /* the end record (tools/uvp.py writes no comment) */
/* A package's central directory and its manifest are read whole. On the PiTrex memory is
 * plentiful; the UVMC2 installer keeps them small, and tools/uvp.py refuses to pack, for a
 * UVMC2 target, a package that would not fit (UVMC2_DIR_MAX, UVMC2_TEXT_MAX there). WW VI's
 * 122 files did not fit in 16 KB: its record came to ~20 KB and it was never installed
 * (2026-11-25). */
#ifdef UVM2_PITREX
#define ZIP_DIR_MAX    (128u * 1024u)
#define TEXT_MAX       (128u * 1024u)  /* a manifest, a record */
#else
#define ZIP_DIR_MAX    16384u          /* the central directory: ~200 files */
#define TEXT_MAX       16384u          /* a manifest, a record */
#endif
#define PENDING_MAX    128             /* a clean card: every game a package (make_uvps.py) */
#define ZIP_END_SIG    0x06054b50u
#define ZIP_DIR_SIG    0x02014b50u
#define ZIP_LOCAL_SIG  0x04034b50u
#define MANIFEST       "manifest.toml"

#ifdef UVM2_PITREX
#define TOP            "/"
#if defined(__ARM_ARCH) && __ARM_ARCH < 7
#define THIS_BOARD     "pitrex_zero1"
#else
#define THIS_BOARD     "pitrex_zero2"
#endif
static const struct { const char *from, *to; } MAP[] = {
    { "pitrex/zero2/", "/games/" },
    { "pitrex/zero1/", "/piZero1/" },
    { "gamedata/",     "/gamedata/" },
    { "roms/",         "/roms/" },          /* only a package for private testing has any */
};
#else
#define TOP            ""
#define THIS_BOARD     "uvmc2"
static const struct { const char *from, *to; } MAP[] = {
    { "uvmc2/",        "" },
    { "gamedata/",     "" },
    { "roms/",         "roms/" },           /* only a package for private testing has any */
};
#endif
#define RECORDS        TOP ".uvp"
#ifdef UVM2_PITREX
extern uint32_t uvm2_now_us(void);       /* how long an install took, in its record */
/* WHERE THE TIME GOES, in the record too: reading the package, writing the files, opening and
 * closing them. A slow install is one of the three, and this says which. */
static uint32_t s_us_read, s_us_write, s_us_open;
#define TIMED(acc, expr) do { const uint32_t t_ = uvm2_now_us(); expr; acc += uvm2_now_us() - t_; } while (0)
#else
#define TIMED(acc, expr) do { expr; } while (0)
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
        char x = s[a - b + i];
        if (x >= 'A' && x <= 'Z') x += 32;
        if (x != tail[i]) return 0;
    }
    return 1;
}

static int destination(const char *inner, char *out, size_t max)
{
    for (unsigned i = 0; i < sizeof MAP / sizeof MAP[0]; i++) {
        const size_t n = strlen(MAP[i].from);
        if (strncmp(inner, MAP[i].from, n) == 0 && inner[n])
            return snprintf(out, max, "%s%s", MAP[i].to, inner + n) < (int)max;
    }
    return 0;
}

static const char *base_name(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

/* a whole small file into `buf` (TEXT_MAX); its length, or 0 */
static uint32_t read_text(const char *path, char *buf)
{
    uvm2_sd_file f;
    uint32_t n = 0, got;
    if (!uvm2_sd_open(path, &f)) return 0;
    if (f.len >= TEXT_MAX) { uvm2_sd_close(&f); return 0; }
    while (n < f.len && (got = uvm2_sd_next(&f, (unsigned char *)buf + n, f.len - n)) > 0) n += got;
    uvm2_sd_close(&f);
    buf[n] = 0;
    return n == f.len ? n : 0;
}

/* ── the manifest ── */
typedef struct {
    char id[UVM2_TOML_NAME_MAX], title[UVM2_TOML_VALUE_MAX], menu[UVM2_TOML_NAME_MAX];
    char image[UVM2_TOML_NAME_MAX];           /* this board's executable, its file name */
    int  entries;
} manifest_t;

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

/* ── the package, a zip ── */
static uint8_t  s_dir[ZIP_DIR_MAX];
static uint32_t s_dir_n;
static uint8_t  s_chunk[CHUNK];
static char     s_text[TEXT_MAX];            /* the manifest, then the record built on it */
static char     s_record[TEXT_MAX];

static int read_at(uvm2_sd_file *f, uint32_t at, uint8_t *dst, uint32_t n)
{
    uint32_t got = 0, k;
    if (!uvm2_sd_seek(f, at)) return 0;
    while (got < n && (k = uvm2_sd_next(f, dst + got, n - got)) > 0) got += k;
    return got == n;
}

static int zip_open(uvm2_sd_file *f, const char *path)
{
    if (!uvm2_sd_open(path, f)) return 0;
    const uint32_t tail = f->len < ZIP_TAIL_MAX ? f->len : ZIP_TAIL_MAX;
    if (!read_at(f, f->len - tail, s_chunk, tail)) return 0;
    for (int32_t i = (int32_t)tail - 22; i >= 0; i--) {
        if (le32(s_chunk + i) != ZIP_END_SIG) continue;
        s_dir_n = le32(s_chunk + i + 12);
        const uint32_t at = le32(s_chunk + i + 16);
        return s_dir_n <= ZIP_DIR_MAX && read_at(f, at, s_dir, s_dir_n);
    }
    return 0;
}

typedef struct { char name[PATH_MAX_UVP]; uint32_t method, crc, size, local; } entry_t;

static int zip_entry(uint32_t *at, entry_t *e)
{
    if (*at + 46u > s_dir_n || le32(s_dir + *at) != ZIP_DIR_SIG) return 0;
    const uint8_t *d = s_dir + *at;
    const uint32_t name_n = le16(d + 28);
    e->method = le16(d + 10);
    e->crc    = le32(d + 16);
    e->size   = le32(d + 24);
    e->local  = le32(d + 42);
    const uint32_t n = name_n < sizeof e->name - 1 ? name_n : sizeof e->name - 1;
    memcpy(e->name, d + 46, n);
    e->name[n] = 0;
    *at += 46u + name_n + le16(d + 30) + le16(d + 32);
    return 1;
}

/* where an entry's bytes start */
static int zip_data(uvm2_sd_file *f, const entry_t *e, uint32_t *start)
{
    uint8_t h[30];
    if (!read_at(f, e->local, h, 30) || le32(h) != ZIP_LOCAL_SIG) return 0;
    *start = e->local + 30u + le16(h + 26) + le16(h + 28);
    return 1;
}

static int extract(uvm2_sd_file *f, const entry_t *e, const char *dst,
                   uvm2_uvp_progress progress, const char *title, uint32_t *done, uint32_t total)
{
    uint32_t start, left = e->size, crc = 0;
    int opened, put;
    if (e->method != 0 || !zip_data(f, e, &start) || !uvm2_sd_seek(f, start)) return 0;
    TIMED(s_us_open, opened = uvm2_sd_stream_open(dst));
    if (!opened) return 0;
    while (left) {
        const uint32_t want = left < CHUNK ? left : CHUNK;
        uint32_t got = 0, k;
        TIMED(s_us_read, while (got < want && (k = uvm2_sd_next(f, s_chunk + got, want - got)) > 0) got += k);
        if (got != want) break;
        crc = crc32_update(crc, s_chunk, got);
        TIMED(s_us_write, put = uvm2_sd_stream_put(s_chunk, got));
        if (!put) break;
        left -= got;
        *done += got;
        if (progress) progress(title, *done, total);
    }
    TIMED(s_us_open, uvm2_sd_stream_close());
    if (left || crc != e->crc) { uvm2_sd_remove(dst); return 0; }
    return 1;
}

static int record_path(const char *id, char *out, size_t max)
{
    return snprintf(out, max, RECORDS "/%s.toml", id) < (int)max;
}

/* every file listed in a record's [installed] table, removed */
static int files_cb(void *user, const char *table, int index, const char *key, const char *value)
{
    (void)user; (void)index;
    if (strcmp(table, "installed") == 0 && strcmp(key, "files") == 0) {
        char dir[PATH_MAX_UVP];
        uvm2_sd_remove(value);
        /* the folders it leaves empty, up to (not including) the card's top folders */
        snprintf(dir, sizeof dir, "%s", value);
        for (char *s; (s = strrchr(dir, '/')) && s != dir; ) {
            *s = 0;
            const char *rest = dir + (dir[0] == '/' ? 1 : 0);
            if (strchr(rest, '/') == 0 && strlen(TOP)) break;   /* /gamedata, /games stay */
            if (!uvm2_sd_remove(dir)) break;                    /* not empty: the game's own */
        }
    }
    return 1;
}

static int install(const char *path, uvm2_uvp_progress progress)
{
    uvm2_sd_file f;
    manifest_t m;
    uint32_t at, total = 0, done = 0, man_n = 0, start;
    entry_t e;
    char dst[PATH_MAX_UVP], rec[PATH_MAX_UVP];
    int ok = 1;
    if (!zip_open(&f, path)) return 0;
    for (at = 0; zip_entry(&at, &e); ) {
        if (strcmp(e.name, MANIFEST) != 0) continue;
        if (e.method == 0 && e.size < TEXT_MAX && zip_data(&f, &e, &start) && read_at(&f, start, (uint8_t *)s_text, e.size))
            man_n = e.size;
        break;
    }
    if (!man_n) { uvm2_sd_close(&f); return 0; }
    s_text[man_n] = 0;
    memset(&m, 0, sizeof m);
    uvm2_toml_read(s_text, man_n, manifest_cb, &m);
    if (!m.id[0] || !record_path(m.id, rec, sizeof rec)) { uvm2_sd_close(&f); return 0; }

    /* the same game again (a new version): the old one's files go first; what it wrote stays */
    uvm2_uvp_uninstall(m.id);

    /* the record: the manifest without its [[file]] list (tools/uvp.py writes it last; the
     * [installed] list below says the same, with the card's paths), then what this puts on
     * the card */
    {
        char *list = strstr(s_text, "\n[[file]]");
        if (list) list[1] = 0;
    }
    uint32_t r = (uint32_t)snprintf(s_record, TEXT_MAX, "%s\n[installed]\nfiles = [", s_text);
    for (at = 0; zip_entry(&at, &e); ) if (destination(e.name, dst, sizeof dst)) total += e.size;
    int first = 1;
#ifdef UVM2_PITREX
    const uint32_t t0 = uvm2_now_us();
    s_us_read = s_us_write = s_us_open = 0;
#endif
    for (at = 0; ok && zip_entry(&at, &e); ) {
        if (!destination(e.name, dst, sizeof dst)) continue;
        ok = extract(&f, &e, dst, progress, m.title, &done, total);
        if (ok) {
            r += (uint32_t)snprintf(s_record + r, TEXT_MAX - r, "%s\"%s\"", first ? "" : ", ", dst);
            first = 0;
            if (r >= TEXT_MAX - 4u) ok = 0;
        }
    }
    uvm2_sd_close(&f);
    if (ok) {
        r += (uint32_t)snprintf(s_record + r, TEXT_MAX - r, "]\nbytes = %u\n", (unsigned)total);
#ifdef UVM2_PITREX
        r += (uint32_t)snprintf(s_record + r, TEXT_MAX - r, "install_ms = %u\nread_ms = %u\nwrite_ms = %u\nopen_close_ms = %u\n",
                                (unsigned)((uvm2_now_us() - t0) / 1000u), (unsigned)(s_us_read / 1000u),
                                (unsigned)(s_us_write / 1000u), (unsigned)(s_us_open / 1000u));
#endif
        ok = uvm2_sd_stream_open(rec) && uvm2_sd_stream_put((const unsigned char *)s_record, r);
        uvm2_sd_stream_close();
    }
    if (!ok) {
        /* what arrived goes again: a half-installed game is worse than none */
        s_record[r < TEXT_MAX ? r : TEXT_MAX - 1] = 0;
        char *list = strstr(s_record, "\n[installed]\nfiles = [");
        for (char *p = list; p && (p = strchr(p, '"')); ) {
            char *q = strchr(p + 1, '"');
            if (!q) break;
            *q = 0;
            uvm2_sd_remove(p + 1);
            p = q + 1;
        }
    }
    return ok;
}

int uvm2_uvp_install_pending(uvm2_uvp_progress progress, char *failed, uint32_t failed_max)
{
    uvm2_sd_dir d;
    char name[PATH_MAX_UVP], names[PENDING_MAX][PATH_MAX_UVP];
    uint32_t size;
    int n = 0, k, installed = 0;
    if (failed && failed_max) failed[0] = 0;
    /* the names first: the folder changes as packages go */
    if (!uvm2_sd_dir_open(TOP, &d)) return 0;
    while (n < PENDING_MAX && (k = uvm2_sd_dir_next(&d, name, sizeof name, &size)) != UVM2_SD_DIR_END)
        if (k == UVM2_SD_DIR_FILE && name[0] != '.' && ends_with(name, UVP_EXT))
            snprintf(names[n++], PATH_MAX_UVP, TOP "%s", name);
    uvm2_sd_dir_close(&d);
    for (int i = 0; i < n; i++) {
        if (install(names[i], progress)) { uvm2_sd_remove(names[i]); installed++; }
        else if (failed) snprintf(failed, failed_max, "%s", base_name(names[i]));
    }
    return installed;
}

/* ── the list ── */
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

void uvm2_uvp_list(uvm2_uvp_entry entry, void *user)
{
    uvm2_sd_dir d;
    char name[PATH_MAX_UVP], path[PATH_MAX_UVP];
    uint32_t size;
    int k;
    if (!uvm2_sd_dir_open(RECORDS, &d)) return;
    while ((k = uvm2_sd_dir_next(&d, name, sizeof name, &size)) != UVM2_SD_DIR_END) {
        if (k != UVM2_SD_DIR_FILE || !ends_with(name, ".toml")) continue;
        snprintf(path, sizeof path, RECORDS "/%s", name);
        const uint32_t n = read_text(path, s_text);
        if (!n) continue;
        manifest_t m;
        memset(&m, 0, sizeof m);
        uvm2_toml_read(s_text, n, manifest_cb, &m);
        if (!m.id[0] || !m.image[0]) continue;
        if (m.entries == 0) entry(user, m.menu, m.title, m.id, m.image, "");
        else {
            lister_t l = { &m, entry, user, "", "", "", -1 };
            uvm2_toml_read(s_text, n, entries_cb, &l);
            flush_entry(&l);
        }
    }
    uvm2_sd_dir_close(&d);
}

int uvm2_uvp_uninstall(const char *id)
{
    char rec[PATH_MAX_UVP];
    if (!record_path(id, rec, sizeof rec)) return 0;
    /* into s_record: install() calls this with the new package's manifest in s_text */
    const uint32_t n = read_text(rec, s_record);
    if (!n) return 0;
    uvm2_toml_read(s_record, n, files_cb, 0);
    uvm2_sd_remove(rec);
    return 1;
}
