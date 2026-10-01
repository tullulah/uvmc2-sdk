/* uvm2_sd.h — read a file off the UVM2's SD card, from inside the game itself.
 *
 * WHY IT EXISTS. The UVM2's firmware loads the .um2 and steps aside: it does not serve
 * romsets. Our own cartridge does — it reads roms/<game>.zip and publishes a descriptor — and
 * without that the 44 AAE ports paint their "no romset" sign on this board. Embedding the zip
 * in the image works but puts the ROM back into the binary, which is exactly what the move to
 * external ROMs removed.
 *
 * So we read it ourselves. After the reset the module owns the machine, the SD included.
 */
#ifndef UVM2_SD_H
#define UVM2_SD_H

#include <stdint.h>

/* 0 = no card, or it did not start. Non-zero = ready to read. */
int uvm2_sd_init(void);

/* Copies <path> (for example "roms/dkong.zip") into dst. Returns the bytes read, or 0.
 * FAT12/16/32 or exFAT, long names, any depth, case-insensitive (FatFs underneath). */
uint32_t uvm2_sd_read(const char *path, unsigned char *dst, uint32_t max);

/** Creates a 512-byte text file from up to 512 bytes of content, padded with spaces and a
 *  newline so uvm2_sd_overwrite can rewrite it later. Missing folders are created and an
 *  existing file is replaced. 1 if it was created. */
int uvm2_sd_create(const char *path, const unsigned char *data, uint32_t n);

/** Overwrites IN PLACE the first sector of a file that ALREADY EXISTS (max 512 bytes). It does
 *  not create and does not resize: MISSING if the file is not there, TOO_BIG if it is shorter
 *  than 512 bytes. 1 if it was written. */
int uvm2_sd_overwrite(const char *path, const unsigned char *data, uint32_t n);

/** Writes a file of ANY size; missing folders are created and an existing file is replaced.
 *  1 if it was written. This is the one for dumping traces, saved games or captures. */
int uvm2_sd_write(const char *path, const unsigned char *data, uint32_t n);
/** The same file written from TWO pieces, one after the other (a header and a body kept
 *  apart). 1 if it was written. */
int uvm2_sd_write2(const char *path, const unsigned char *a, uint32_t na,
                   const unsigned char *b, uint32_t nb);

/* A CHUNK of the file, starting `from` bytes in. Returns what was copied, which may be less
 * than `max` without that being a failure (unlike uvm2_sd_read, where not fitting IS one). It
 * exists so long captures can be replayed without loading them whole into RAM. */
uint32_t uvm2_sd_read_from(const char *path, unsigned char *dst, uint32_t max, uint32_t from);

/* ── AN OPEN FILE, TO READ IT IN SLICES WITHOUT PAYING FOR IT N TIMES ─────────────────────
 *
 * uvm2_sd_read_from is O(n^2) as soon as the file is large: every call mounts the volume
 * again, walks the directory, and seeks from the start of the file to the offset. Measured on
 * the console loading 1.8 MB of audio: minutes.
 *
 * This opens once and keeps reading: the next slice starts exactly where the previous one
 * ended. While ANY file is open the volume is not remounted by the other calls — a remount
 * would invalidate the open file — so a game that streams also skips the per-call card
 * re-initialisation; uvm2_sd_diag.streams says how many are open.
 *
 * Closing is optional: it only lets the other calls go back to remounting on every call. The
 * first five fields are read and written by the IDE's emulator and by games (`ok`), so they
 * keep their place. */
typedef struct {
    uint32_t cluster;    /* the file's first cluster                        */
    uint32_t sec;        /* unused since FatFs; kept for the layout          */
    uint32_t pos;        /* bytes delivered so far                           */
    uint32_t len;        /* file size                                        */
    int      ok;         /* 0 = not open, exhausted, or failed               */
    uint32_t magic;      /* set while open, so a reopen closes the old one   */
    /* FatFs's FIL, opaque so this header does not drag ff.h into every game. uvm2_sd.c
     * asserts at compile time that it fits. */
    uint64_t fil[16];
} uvm2_sd_file;

/* 1 if found. Paths as everywhere else: any depth, long names, case-insensitive. */
int      uvm2_sd_open(const char *path, uvm2_sd_file *f);
/* The next bytes, up to `max`. Returns what was copied; 0 is the end of the file (or an
 * error: then `ok` is 0 and uvm2_sd_error says which). */
uint32_t uvm2_sd_next(uvm2_sd_file *f, unsigned char *dst, uint32_t max);
/* Optional; see above. */
void     uvm2_sd_close(uvm2_sd_file *f);

/* What the mount understood about the disk, so it can be inspected over SWD without guessing.
 * A MISSING can be an absent file or a misread volume, and from outside they look identical;
 * this separates them. It is read in one go:
 *
 *     tools/probe.sh <&uvm2_sd_diag> 14
 */
struct uvm2_sd_diag {
    uint32_t magic;          /* 'SDDG' = 0x47444453; 0 if nothing was ever tried */
    uint32_t step;           /* last call got to: 1 mount, 2 open, 3 data       */
    uint32_t fresult;        /* FatFs FRESULT of the last call (0 = FR_OK)      */
    uint32_t fs_type;        /* 0 none, 1 FAT12, 2 FAT16, 3 FAT32, 4 exFAT       */
    uint32_t volbase;        /* first sector of the volume; 0 = no partition table */
    uint32_t csize;          /* sectors per cluster                             */
    uint32_t n_fatent;       /* clusters + 2                                    */
    uint32_t fatbase, dirbase, database;   /* sectors (dirbase: root cluster on FAT32/exFAT) */
    uint32_t reads, writes;  /* blocks moved since boot: proof the card was touched */
    uint32_t streams;        /* files open through uvm2_sd_open: no remount while > 0 */
    uint32_t baud;           /* SPI clock in Hz, as the divider ACTUALLY landed          */
};
extern struct uvm2_sd_diag uvm2_sd_diag;

/* Last failure, so "no card" and "file missing" do not look the same. */
extern int uvm2_sd_error;
#define UVM2_SD_OK          0
#define UVM2_SD_NO_CARD     1
#define UVM2_SD_NO_INIT     2
#define UVM2_SD_NO_FAT      3
#define UVM2_SD_MISSING     4
#define UVM2_SD_TOO_BIG     5   /* also: card or directory full */
#define UVM2_SD_IO_ERROR    6   /* a block failed mid-way; FatFs's reason in uvm2_sd_diag.fresult */

#endif
