/* uvm2_sd.c — SPI to the SD card (the SPI0 peripheral), with FatFs on top of it.
 *
 * THE FILE SYSTEM IS FatFs (third_party/fatfs), NOT OURS. Until 2026-09-28 this file carried a
 * hand-written FAT16/FAT32 reader and writer. It worked on the cards it was tested on and knew
 * nothing about exFAT — which is what large cards come formatted as. The stock firmware's menu
 * does read exFAT, so such a card starts the .um2 and then every file the GAME asks for failed
 * with NO_FAT: a user report of arcade ports that "worked for basic functions" until they
 * needed their romset. FatFs reads and writes FAT12/16/32 and exFAT, long names, MBR and GPT;
 * what is left here is the part nobody else can write: the SPI to this board's card.
 *
 * THE PIN RANGE COMES FROM THE SCHEMATIC AND IS CONFIRMED (GPIO32..39); the one-to-one
 * assignment does NOT. An off-by-one here breaks the driver WITHOUT REPORTING ANYTHING —
 * it talks to the wrong GPIO and the card simply never answers. That is why uvm2_sd_error
 * tells "no card" apart from "did not start": with a single failure code, a wrong pin and
 * an absent card look exactly the same.
 */
#include "uvm2_sd.h"
#include "ff.h"
#include "diskio.h"

int uvm2_sd_error = UVM2_SD_OK;

/* THE PINS WERE MEASURED OFF THE CARTRIDGE FIRMWARE ITSELF, not read off the schematic.
 *
 * When it loads a module, the UVM2 firmware leaves the WHOLE high I/O bank at FUNCSEL 31
 * (null) — checked over SWD: from GPIO32 to GPIO47 not one is configured. But while it is
 * sitting in ITS OWN MENU the SD is up and running, and there IO_BANK0 can be read without
 * touching anything. That gave (2026-08-20):
 *
 *     GPIO34  FUNCSEL 1 = SPI0_SCLK   STATUS OETOPAD           -> driving low: SCK
 *     GPIO35  FUNCSEL 1 = SPI0_TX     STATUS 0                 -> MOSI
 *     GPIO36  FUNCSEL 1 = SPI0_RX     STATUS INFROMPAD|IRQ     -> reads high: MISO, pulled up
 *     GPIO39  FUNCSEL 5 = SIO         STATUS OUT|OE|IN|IRQ     -> driving high: CS, active low
 *
 * The previous list came from reading schematic labels at the limit of their resolution and
 * was SHIFTED BY TWO, with CS on yet another pin. The symptom was exactly the one this
 * comment warns about: the card does not answer and uvm2_sd_error stays at NO_INIT.
 *
 * If this ever has to be revisited, that is the recipe: leave the console in the cartridge
 * menu and dump IO_BANK0 (0x40028100, 32 words) over SWD. It is a MEASUREMENT, not a read.
 *
 * ── AND THE SCHEMATIC AGREES, ONCE IT IS READ PROPERLY (2026-09-29) ───────────────────
 *
 * The board's net names are dual — `MISO/SDIO_DAT0`, `SDCS/SDIO_DAT3` — because the socket
 * is wired for BOTH: 1-bit SPI, which is what this file does, and 4-bit SDIO, which nothing
 * does yet. The full map, with the schematic's package pin numbers:
 *
 *     GPIO34  pin 43   SCK/SDIO_CLK      SCK   / CLK
 *     GPIO35  pin 44   MOSI/SDIO_CMD     MOSI  / CMD
 *     GPIO36  pin 45   MISO/SDIO_DAT0    MISO  / DAT0
 *     GPIO37  pin 46   SDIO_DAT1               / DAT1     unused in SPI mode
 *     GPIO38  pin 47   SDIO_DAT2               / DAT2     unused in SPI mode
 *     GPIO39  pin 48   SDCS/SDIO_DAT3    CS    / DAT3     10K pull-up (R12)
 *     GPIO40  pin 49   SD_DETECT                          the socket's CD pin
 *
 * CS AND DAT3 BEING THE SAME PIN IS THE SD SPEC, not a coincidence, and it is what anchors
 * the map: the measurement found CS on GPIO39 and the schematic calls GPIO39 DAT3.
 *
 * WHY THE EARLIER READ WAS OFF BY TWO: the net labels line up with the PACKAGE PIN numbers
 * (43, 44, 45...), and whoever read them lined them up with the GPIO numbers starting at 32.
 * Two independent sources now tell the same story, which is the first time that has been
 * true here.
 *
 * ── THERE IS A CARD DETECT AFTER ALL, AND THIS COMMENT SAID THERE WAS NOT ─────────────
 *
 * It said "SD_DETECT does not exist in this wiring", and that was wrong. What is true is
 * narrower: the FIRMWARE configures no pin for it. The board has one, on GPIO40.
 *
 * The confusion was the same off-by-two: an earlier version polled GPIO38 and always read 0,
 * and 0 was taken for "no card". GPIO38 is DAT2 — an unowned pad with the reset pull-down on
 * it, which reads 0 whatever the socket is doing. A diagnostic that cannot fail diagnoses
 * nothing, and that part of the lesson stands.
 *
 * NOBODY HAS TRIED GPIO40. It is not used here because the driver does not need it — every
 * mount re-initialises the card, which is how a swap is noticed — but "untried" and
 * "does not exist" are different claims and only one of them was ever true. */

/* ── THE HOST SIDE ────────────────────────────────────────────────────────────────────
 * With -DUVM2_SD_HOST the very same code compiles against a FILE instead of the card. It
 * exists so WRITING can be tested without risking anybody's card, against real FAT16, FAT32
 * and exFAT images that `fsck_msdos` / `fsck_exfat` then validate: tools/uvm2_sd_test.sh. */
#ifndef UVM2_SD_HOST
#define PIN_SCK   34
#define PIN_MOSI  35
#define PIN_MISO  36
#define PIN_CS    39

#define PADS_BANK0 0x40038000u
#define IO_BANK0   0x40028000u
#define SIO        0xD0000000u
#define R(a)       (*(volatile uint32_t *)(uintptr_t)(a))

/* GP32-47 live in the HIGH SIO bank: different registers, and the bit is pin-32.
 *
 * THE HIGH BANK OFFSETS ARE NOT THE RP2040 ONES, and that already cost a black screen:
 * there HI_OUT_SET sits at 0x028 and here at 0x01C, so writing the RP2040 number lands on
 * GPIO_OE / GPIO_OE_SET of the LOW bank — that is, on the direction of the CARTRIDGE BUS
 * pins. The symptom was not an error: the Vectrex clock stopped advancing and
 * uvm2_frame_end waited forever.
 *
 * The values come from hardware/regs/sio.h in the pico-sdk, and the static_assert below
 * checks them against that header when it is available. Do not write them from memory. */
#define SIO_HI_IN       0x008u
#define SIO_HI_OUT_SET  0x01Cu
#define SIO_HI_OUT_CLR  0x024u
#define SIO_HI_OE_SET   0x03Cu
#define SIO_HI_OE_CLR   0x044u

#if defined(__has_include)
#  if __has_include(<hardware/regs/sio.h>) && __has_include(<hardware/platform_defs.h>)
     /* sio.h wraps every number in _u(), which lives in platform_defs.h: without it the
      * value is not a constant and the static_assert does not compile. */
#    include <hardware/platform_defs.h>
#    include <hardware/regs/sio.h>
_Static_assert(SIO_HI_IN      == SIO_GPIO_HI_IN_OFFSET,      "SIO_HI_IN");
_Static_assert(SIO_HI_OUT_SET == SIO_GPIO_HI_OUT_SET_OFFSET, "SIO_HI_OUT_SET");
_Static_assert(SIO_HI_OUT_CLR == SIO_GPIO_HI_OUT_CLR_OFFSET, "SIO_HI_OUT_CLR");
_Static_assert(SIO_HI_OE_SET  == SIO_GPIO_HI_OE_SET_OFFSET,  "SIO_HI_OE_SET");
_Static_assert(SIO_HI_OE_CLR  == SIO_GPIO_HI_OE_CLR_OFFSET,  "SIO_HI_OE_CLR");
#  endif
#endif

static inline uint32_t hi(int pin) { return 1u << (pin - 32); }

/* ONE PLACE WRITES THE PADS, whichever function ends up driving the pin. The two RP2350
 * traps below are the reason: they are properties of the PAD, so they apply just the same
 * when the SPI peripheral owns the pin as when SIO does, and a second copy of this would be
 * a second chance to forget one of them.
 *
 * `funcsel` 5 is SIO (and only then does `is_output` mean anything — the OE registers are
 * SIO's); 1 is SPI0 on these three pins. */
static void cfg_pad(int pin, unsigned funcsel, int is_output, int pull_up)
{
    /* The pad comes up ISOLATED on the RP2350: without clearing ISO the pin is mute and
     * says nothing about it. */
    volatile uint32_t *pad = (volatile uint32_t *)(uintptr_t)(PADS_BANK0 + 4 + 4*pin);
    /* PDE is SET in the reset value (0x0116). Leaving it is the sister trap of ISO: on
     * MISO that internal pull-down fights the board's pull-up — the one you can see as a
     * high read on GPIO36 in the firmware menu — and can pull the line down. */
    *pad = (*pad & ~((1u<<8) | (1u<<7) | (1u<<2))) | (1u<<6) | (pull_up ? (1u<<3) : 0u);
                                       /* ISO=0, OD=0, PDE=0, IE=1, PUE where asked */
    R(IO_BANK0 + 8*pin + 4) = funcsel;
    if (funcsel == 5u) {
        if (is_output) R(SIO + SIO_HI_OE_SET) = hi(pin);
        else           R(SIO + SIO_HI_OE_CLR) = hi(pin);
    }
}
/* A plain GPIO, as this file used to configure all four: inputs get the pull-up. */
static void cfg_pin(int pin, int is_output) { cfg_pad(pin, 5u, is_output, !is_output); }
static inline void set_pin(int pin, int v)
{
    if (v) R(SIO + SIO_HI_OUT_SET) = hi(pin);
    else   R(SIO + SIO_HI_OUT_CLR) = hi(pin);
}
static inline int get_pin(int pin) { return (R(SIO + SIO_HI_IN) >> (pin - 32)) & 1u; }

static void cs(int on) { set_pin(PIN_CS, !on); }   /* CS is active LOW */

/* ── THE TRANSPORT: SPI0, WITH THE OLD BIT-BANG STILL HERE TO BISECT AGAINST ─────────
 *
 * Both halves below export the same four calls, so everything above and below this section
 * reads the same either way:
 *
 *     sd_pins_init()          put SCK / MOSI / MISO under whoever is driving them
 *     sd_speed(slow)          the card demands <= 400 kHz until it leaves idle
 *     xfer(byte)              one byte out, one byte in
 *     xfer_in(buf, n)         n bytes in, 0xFF out (the read direction)
 *     xfer_out(buf, n)        n bytes out, reply discarded (the write direction)
 *
 * WHY THE PERIPHERAL. The bit-bang costs ~63 cycles per bit, which the debug cartridge's own
 * driver had already written down as "~1.7 ms per 512-byte block" — 301 KB/s, and that number
 * sat in the repo through a whole session of proposing to go and measure it. 8 MB of PSRAM
 * through it is 27 seconds. SPI0 is on GPIO34/35/36 ON BOTH CARTRIDGES, which is not luck: it
 * is why the stock firmware's own menu has them at FUNCSEL 1, the measurement this file's pin
 * comment is built on.
 *
 * WHY 12 MHz AND NOT 25. 25 MHz is the SD spec's ceiling for SPI mode and every card must do
 * it, so the divider is not the limit — the limit is that NOTHING HERE CHECKS THE CRC. At
 * 2.4 MHz that was never a question; a corrupt byte at 25 MHz on a socket at the end of a
 * cartridge would arrive silently and look like a corrupt file system. 12 MHz is a 5x win
 * with the edge rates still nowhere near the cable. Raising it is one constant, and the
 * honest order is: check the CRC16 on reads FIRST, then raise it.
 *
 * UVM2_SD_BITBANG brings the old path back verbatim. Both talk to the same card over the same
 * pins, so a card that reads one way and not the other says the speed is the fault and not
 * the code — which is the only reason to keep dead code around. */
#ifndef UVM2_SD_BITBANG

#include "hardware/spi.h"

#ifndef UVM2_SD_BAUD_SLOW
#define UVM2_SD_BAUD_SLOW    400000u    /* CMD0..CMD41: the card's own start-up limit */
#endif
/* 12.5 MHz AND NOT 12: clk_peri is 150 MHz here and the divider is an integer, so 12.5 is
 * 150/12 exactly while a rounder-looking 12 000 000 lands on 150/14 = 10.7 — spi_set_baudrate
 * only ever rounds DOWN, and asking for the decimal that looks tidier throws away 17%.
 * uvm2_sd_diag.baud reports where it actually landed, whatever is asked for here. */
#ifndef UVM2_SD_BAUD_FAST
#define UVM2_SD_BAUD_FAST  12500000u    /* see above before raising this */
#endif

/* EVERY FIFO WAIT IS BOUNDED, and this is not defensive decoration — it is the difference
 * between a driver that reports and a machine that stops.
 *
 * The bit-bang could not hang: its loops ran a fixed number of times whatever the card
 * did. The peripheral's do. `while (!writable) {}` waits for hardware that, where SPI0 is
 * not present, never answers — and the IDE's emulator is exactly such an environment. The
 * symptom was the worst kind: not an error, not a black screen, but core 0 spinning at
 * 99.9% inside spi_is_writable with the game never starting. Every .um2 that so much as
 * reads its own configuration stopped booting there, and that is every one of them.
 *
 * The bound is generous on purpose: at the slow clock a byte is ~20 us, so 100000 spins is
 * orders of magnitude past any real transfer and cannot trip on a slow card. What it
 * catches is a peripheral that was never going to answer.
 *
 * On a timeout the transfer is abandoned and uvm2_sd_error says so, which turns a hang into
 * "no card" — the game then paints its own missing-romset notice instead of freezing. An
 * unbounded spin has cost this project a hung cartridge before, and the lesson written down
 * then was exactly this one. */
#define SPI_SPIN_LIMIT  100000u

static int spi_timed_out;   /* sticky for the rest of the transfer; cleared at init */

static void sd_pins_init(void)
{
    /* spi_init sets 8 bits, CPOL 0, CPHA 0, MSB first — SD's mode 0 — and takes the block
     * out of reset. It has to run BEFORE the pads hand the pins over, or SCK idles wherever
     * an unconfigured peripheral leaves it while the card is watching. */
    spi_init(spi0, UVM2_SD_BAUD_SLOW);
    cfg_pad(PIN_SCK,  1u, 1, 0);
    cfg_pad(PIN_MOSI, 1u, 1, 0);
    cfg_pad(PIN_MISO, 1u, 0, 1);         /* pull-up: the board has one and PDE fights it */
}
static void sd_speed(int slow)
{
    /* What the divider actually gave, not what was asked for: clk_peri / (prescale*postdiv)
     * rarely lands on the number. It is in the diag so a rate measured upstairs can be
     * divided by something true. */
    uvm2_sd_diag.baud = spi_set_baudrate(spi0, slow ? UVM2_SD_BAUD_SLOW : UVM2_SD_BAUD_FAST);
}
/* The SDK's spi_*_blocking helpers spin without a bound, so the FIFO is handled here
 * instead: same protocol, same order, one counter. */
static int spi_wait(uint32_t mask)
{
    uint32_t n = SPI_SPIN_LIMIT;
    while (n--) if (spi_get_hw(spi0)->sr & mask) return 1;
    spi_timed_out = 1;
    uvm2_sd_error = UVM2_SD_NO_CARD;
    return 0;
}

static uint8_t xfer(uint8_t out)
{
    if (spi_timed_out) return 0xFF;
    if (!spi_wait(SPI_SSPSR_TNF_BITS)) return 0xFF;
    spi_get_hw(spi0)->dr = (uint32_t)out;
    if (!spi_wait(SPI_SSPSR_RNE_BITS)) return 0xFF;
    return (uint8_t)spi_get_hw(spi0)->dr;
}
static void xfer_in(unsigned char *buf, int n)
{
    int i;
    for (i = 0; i < n; i++) buf[i] = xfer(0xFF);
}
static void xfer_out(const unsigned char *buf, int n)
{
    int i;
    for (i = 0; i < n; i++) (void)xfer(buf[i]);
}

#else   /* UVM2_SD_BITBANG */

/* Half a clock cycle. Slow while starting up (the card demands it until it leaves idle)
 * and fast afterwards; without the two speeds it either never starts or takes forever. */
static volatile int s_slow = 1;
static void tick(void) { volatile int n = s_slow ? 24 : 1; while (n--) { } }

static void sd_pins_init(void)
{
    cfg_pin(PIN_SCK, 1); cfg_pin(PIN_MOSI, 1);
    cfg_pin(PIN_MISO, 0);
    set_pin(PIN_SCK, 0); set_pin(PIN_MOSI, 1);
}
static void sd_speed(int slow) { s_slow = slow; uvm2_sd_diag.baud = slow ? 100000u : 2400000u; }

static uint8_t xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        set_pin(PIN_MOSI, (out >> i) & 1);
        tick();
        set_pin(PIN_SCK, 1);             /* rising edge: MISO is sampled */
        in = (uint8_t)((in << 1) | get_pin(PIN_MISO));
        tick();
        set_pin(PIN_SCK, 0);
    }
    return in;
}
static void xfer_in(unsigned char *buf, int n)
{
    for (int i = 0; i < n; i++) buf[i] = xfer(0xFF);
}
static void xfer_out(const unsigned char *buf, int n)
{
    for (int i = 0; i < n; i++) xfer(buf[i]);
}

#endif  /* UVM2_SD_BITBANG */

static uint8_t command(uint8_t idx, uint32_t arg, uint8_t crc)
{
    /* One guard byte and the six-byte frame, sent together: over the peripheral that is one
     * FIFO burst instead of seven round trips, and the bytes on the wire are identical. */
    const unsigned char frame[7] = {
        0xFF, (unsigned char)(0x40 | idx),
        (unsigned char)(arg >> 24), (unsigned char)(arg >> 16),
        (unsigned char)(arg >> 8),  (unsigned char)arg,
        crc
    };
    xfer_out(frame, 7);
    for (int i = 0; i < 10; i++) {           /* R1: first byte without bit 7 */
        uint8_t r = xfer(0xFF);
        if (!(r & 0x80)) return r;
    }
    return 0xFF;
}

static int s_sdhc = 0;

int uvm2_sd_init(void)
{
    uvm2_sd_error = UVM2_SD_OK;
    /* CS stays a plain GPIO whichever transport is in use: it has to be held down across a
     * whole multi-byte transaction, which is not something a peripheral's own CS does, and
     * it is the ONE pin that differs between the two cartridges. */
    cfg_pin(PIN_CS, 1);
    cs(0);
#ifndef UVM2_SD_BITBANG
    spi_timed_out = 0;      /* a previous mount that timed out must not poison this one */
#endif
    sd_pins_init();
    sd_speed(1);

    /* 80 pulses with CS high: the card asks for them before it enters SPI mode. */
    for (int i = 0; i < 10; i++) xfer(0xFF);

    cs(1);
    if (command(0, 0, 0x95) != 0x01) { cs(0); uvm2_sd_error = UVM2_SD_NO_INIT; return 0; }

    uint8_t r = command(8, 0x1AA, 0x87);     /* a v2 card answers 0x01 + 4 bytes */
    int v2 = (r == 0x01);
    if (v2) for (int i = 0; i < 4; i++) xfer(0xFF);

    for (int attempt = 0; ; attempt++) {
        command(55, 0, 0xFF);
        if (command(41, v2 ? 0x40000000u : 0, 0xFF) == 0x00) break;
        if (attempt > 20000) { cs(0); uvm2_sd_error = UVM2_SD_NO_INIT; return 0; }
    }
    if (v2) {                                 /* CMD58: does it address by blocks? */
        if (command(58, 0, 0xFF) == 0x00) {
            uint8_t ocr = xfer(0xFF); xfer(0xFF); xfer(0xFF); xfer(0xFF);
            s_sdhc = (ocr & 0x40) != 0;
        }
    }
    cs(0);
    sd_speed(0);
    return 1;
}

#else   /* UVM2_SD_HOST */
int uvm2_host_read(uint32_t lba, unsigned char *b);
int uvm2_host_write(uint32_t lba, const unsigned char *b);
int uvm2_sd_init(void) { return 1; }
#endif

#ifdef UVM2_SD_HOST
static int read_block(uint32_t lba, unsigned char *buf) { return uvm2_host_read(lba, buf); }
static int write_block(uint32_t lba, const unsigned char *buf) { return uvm2_host_write(lba, buf); }
#else
static int read_block(uint32_t lba, unsigned char *buf)
{
    cs(1);
    if (command(17, s_sdhc ? lba : lba * 512u, 0xFF) != 0x00) { cs(0); return 0; }
    uint8_t t = 0xFF;
    for (int i = 0; i < 200000 && t == 0xFF; i++) t = xfer(0xFF);
    if (t != 0xFE) { cs(0); return 0; }
    xfer_in(buf, 512);
    xfer(0xFF); xfer(0xFF);                   /* CRC, which we do not check */
    cs(0);
    return 1;
}

/* WRITE ONE BLOCK (CMD24). FatFs decides WHAT is written — FAT copies, FSInfo, exFAT bitmap
 * and entry-set checksums — and this only has to report honestly whether it landed. */
static int write_block(uint32_t lba, const unsigned char *buf)
{
    cs(1);
    if (command(24, s_sdhc ? lba : lba * 512u, 0xFF) != 0x00) { cs(0); return 0; }
    xfer(0xFF);                                /* one guard byte before the token */
    xfer(0xFE);                                /* start-of-block token */
    xfer_out(buf, 512);
    xfer(0xFF); xfer(0xFF);                    /* CRC, which the card ignores over SPI */
    /* The data response: xxx00101 = accepted. Anything else is a failure, and it has to be
     * caught here rather than discovered on the next read. */
    uint8_t r = 0xFF;
    for (int i = 0; i < 1000 && (r & 0x11) != 0x01; i++) r = xfer(0xFF);
    if ((r & 0x1F) != 0x05) { cs(0); return 0; }
    /* AND WAIT FOR IT TO RELEASE BUSY. The card holds MISO low while it programs; leaving
     * early leaves the write half done and the next command fails without saying why. */
    for (int i = 0; i < 500000; i++) if (xfer(0xFF) != 0x00) break;
    cs(0);
    return 1;
}

#endif  /* UVM2_SD_HOST */

/* ── THE DISK UNDER FatFs ─────────────────────────────────────────────────────────── */
struct uvm2_sd_diag uvm2_sd_diag;

/* 0 = the last uvm2_sd_init() failed. FatFs asks through disk_status() before every access. */
static int s_ready = 0;

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    /* EVERY MOUNT RE-INITIALISES THE CARD, as the old reader did on every call: there is no
     * card-detect pin, so this is the only way a swapped or re-seated card is noticed. */
    s_ready = uvm2_sd_init();
    return s_ready ? 0 : STA_NOINIT;
}
DSTATUS disk_status(BYTE pdrv) { return (pdrv || !s_ready) ? STA_NOINIT : 0; }

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv || !s_ready) return RES_NOTRDY;
    /* The card addresses 32 bits of sectors (2 TB). A GPT volume past that is not an SD card. */
    if (sector + count > 0x100000000ull) return RES_PARERR;
    for (UINT i = 0; i < count; i++, buff += 512) {
        uvm2_sd_diag.reads++;
        if (!read_block((uint32_t)sector + i, buff)) return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv || !s_ready) return RES_NOTRDY;
    if (sector + count > 0x100000000ull) return RES_PARERR;
    for (UINT i = 0; i < count; i++, buff += 512) {
        uvm2_sd_diag.writes++;
        if (!write_block((uint32_t)sector + i, buff)) return RES_ERROR;
    }
    return RES_OK;
}

/* CTRL_SYNC is the only command FatFs sends with this configuration, and write_block already
 * waits for the card to release busy before returning. */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)buff;
    if (pdrv || !s_ready) return RES_NOTRDY;
    return cmd == CTRL_SYNC ? RES_OK : RES_PARERR;
}

/* ── THE API ────────────────────────────────────────────────────────────────────────── */
static FATFS s_fs;

/* FatFs's answer onto the codes callers already test. The raw FRESULT stays in
 * uvm2_sd_diag.fresult, because IO_ERROR covers several of them. */
static int fail(FRESULT fr)
{
    uvm2_sd_diag.fresult = (uint32_t)fr;
    switch (fr) {
    case FR_OK:            uvm2_sd_error = UVM2_SD_OK;       return 1;
    case FR_NOT_READY:     uvm2_sd_error = UVM2_SD_NO_INIT;  break;  /* also: no card */
    case FR_NO_FILESYSTEM: uvm2_sd_error = UVM2_SD_NO_FAT;   break;
    case FR_NO_FILE:
    case FR_NO_PATH:
    case FR_INVALID_NAME:  uvm2_sd_error = UVM2_SD_MISSING;  break;
    case FR_DENIED:        uvm2_sd_error = UVM2_SD_TOO_BIG;  break;  /* card or directory full */
    default:               uvm2_sd_error = UVM2_SD_IO_ERROR; break;
    }
    return 0;
}

/* Mounts afresh on every call, like the old reader, so the card may change between calls —
 * EXCEPT while a file is open through uvm2_sd_open: remounting would invalidate it. */
static int begin(void)
{
    uvm2_sd_error = UVM2_SD_OK;
    uvm2_sd_diag.magic = 0x47444453u;                          /* 'SDDG' */
    if (uvm2_sd_diag.streams && s_fs.fs_type) { uvm2_sd_diag.step = 2; return 1; }
    uvm2_sd_diag.step = 1;
    uvm2_sd_diag.fs_type = 0;
    if (!fail(f_mount(&s_fs, "", 1))) return 0;
    uvm2_sd_diag.fs_type  = s_fs.fs_type;
    uvm2_sd_diag.volbase  = (uint32_t)s_fs.volbase;
    uvm2_sd_diag.csize    = s_fs.csize;
    uvm2_sd_diag.n_fatent = s_fs.n_fatent;
    uvm2_sd_diag.fatbase  = (uint32_t)s_fs.fatbase;
    uvm2_sd_diag.dirbase  = (uint32_t)s_fs.dirbase;
    uvm2_sd_diag.database = (uint32_t)s_fs.database;
    uvm2_sd_diag.step = 2;
    return 1;
}

/* Creates every folder in `path` that is missing. One that already exists is not an error. */
static int make_parents(const char *path)
{
    char dir[128];
    for (int i = 0; path[i]; i++) {
        if (i >= (int)sizeof dir - 1) return fail(FR_INVALID_NAME);
        dir[i] = path[i];
        if (path[i] != '/' || i == 0) continue;
        dir[i] = 0;
        FRESULT fr = f_mkdir(dir);
        if (fr != FR_OK && fr != FR_EXIST) return fail(fr);
        dir[i] = '/';
    }
    return 1;
}

/* Opens, and reports the length saturated to 32 bits: past 4 GB the callers say TOO_BIG. */
static int open_file(FIL *f, const char *path, BYTE mode, uint32_t *len)
{
    if (!fail(f_open(f, path, mode))) return 0;
    const FSIZE_t sz = f_size(f);
    *len = sz > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)sz;
    uvm2_sd_diag.step = 3;
    return 1;
}

/* A short read is an I/O failure, not the end of the file: the length was checked first. */
static uint32_t read_at(FIL *f, unsigned char *dst, uint32_t n, uint32_t from)
{
    UINT got = 0;
    FRESULT fr = f_lseek(f, from);
    if (fr == FR_OK) fr = f_read(f, dst, n, &got);
    if (fr == FR_OK && got != n) fr = FR_DISK_ERR;
    f_close(f);
    return fail(fr) ? got : 0;
}

/* A chunk starting at `from_off`: whatever fits, and coming up short is NOT a failure. */
uint32_t uvm2_sd_read_from(const char *path, unsigned char *dst, uint32_t max, uint32_t from_off)
{
    FIL f;
    uint32_t len;
    if (!begin() || !open_file(&f, path, FA_READ, &len)) return 0;
    if (from_off >= len) { f_close(&f); return 0; }     /* past the end: zero, not an error */
    uint32_t want = len - from_off;
    if (want > max) want = max;
    return read_at(&f, dst, want, from_off);
}

/* The WHOLE file: not fitting IS a failure, because half a romset is not a romset. It is
 * checked BEFORE reading, so a file that is too big costs no transfer at all. */
uint32_t uvm2_sd_read(const char *path, unsigned char *dst, uint32_t max)
{
    FIL f;
    uint32_t len;
    if (!begin() || !open_file(&f, path, FA_READ, &len)) return 0;
    if (len > max) { f_close(&f); uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    return read_at(&f, dst, len, 0);
}

/* The config files' sector: the text, padded with spaces and a closing newline so the file
 * stays readable on a PC and drags nothing along from what was there before. */
static void pad_sector(unsigned char *b, const unsigned char *data, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = data[i];
    for (uint32_t i = n; i < 512; i++) b[i] = ' ';
    b[511] = '\n';
}

static int write_all(FIL *f, const unsigned char *data, uint32_t n)
{
    UINT put = 0;
    FRESULT fr = f_write(f, data, n, &put);
    if (fr == FR_OK && put != n) fr = FR_DENIED;              /* the card filled up */
    FRESULT fc = f_close(f);                                  /* the entry is written HERE */
    return fail(fr != FR_OK ? fr : fc);
}

/* OVERWRITES THE FIRST SECTOR OF A FILE THAT ALREADY EXISTS, without changing its size or
 * its clusters. It does NOT create: if the file is not there, MISSING, and uvm2_config falls
 * back to uvm2_sd_create. A file shorter than 512 bytes is TOO_BIG, as before. */
int uvm2_sd_overwrite(const char *path, const unsigned char *data, uint32_t n)
{
    static unsigned char b[512];
    FIL f;
    uint32_t len;
    uvm2_sd_error = UVM2_SD_OK;
    if (n == 0 || n > 512) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    if (!begin() || !open_file(&f, path, FA_WRITE | FA_OPEN_EXISTING, &len)) return 0;
    if (len < 512) { f_close(&f); uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    pad_sector(b, data, n);
    return write_all(&f, b, 512);
}

/* A 512-byte text file, padded like uvm2_sd_overwrite leaves it, so the next save can
 * overwrite it in place. Creates the folders on the way; an existing file is replaced. */
int uvm2_sd_create(const char *path, const unsigned char *data, uint32_t n)
{
    static unsigned char b[512];
    FIL f;
    uint32_t len;
    uvm2_sd_error = UVM2_SD_OK;
    if (n == 0 || n > 512) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    if (!begin() || !make_parents(path)) return 0;
    if (!open_file(&f, path, FA_WRITE | FA_CREATE_ALWAYS, &len)) return 0;
    pad_sector(b, data, n);
    return write_all(&f, b, 512);
}

/* A file of ANY size, for traces, saved games and captures. Creates the folders on the way;
 * an existing file is replaced. */
int uvm2_sd_write(const char *path, const unsigned char *data, uint32_t n)
{
    FIL f;
    uint32_t len;
    uvm2_sd_error = UVM2_SD_OK;
    if (n == 0) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    if (!begin() || !make_parents(path)) return 0;
    if (!open_file(&f, path, FA_WRITE | FA_CREATE_ALWAYS, &len)) return 0;
    return write_all(&f, data, n);
}

/* The same, from two pieces: a header and a body that live in different places (a command
 * list dump: its header on the stack, the list in its own buffer, which may be in PSRAM). One
 * file, one close; either piece may be empty, not both. */
int uvm2_sd_write2(const char *path, const unsigned char *a, uint32_t na,
                   const unsigned char *b, uint32_t nb)
{
    FIL f;
    uint32_t len;
    uvm2_sd_error = UVM2_SD_OK;
    if (na + nb == 0) { uvm2_sd_error = UVM2_SD_TOO_BIG; return 0; }
    if (!begin() || !make_parents(path)) return 0;
    if (!open_file(&f, path, FA_WRITE | FA_CREATE_ALWAYS, &len)) return 0;
    if (na) {
        UINT put = 0;
        FRESULT fr = f_write(&f, a, na, &put);
        if (fr == FR_OK && put != na) fr = FR_DENIED;          /* the card filled up */
        if (fr != FR_OK) { f_close(&f); return fail(fr); }
    }
    if (!nb) return fail(f_close(&f));
    return write_all(&f, b, nb);
}

/* ── AN OPEN FILE ──────────────────────────────────────────────────────────────────────── */
#define UVM2_SD_OPEN_MAGIC 0x4E45504Fu   /* 'OPEN' */
#define FIL_OF(f) ((FIL *)(void *)(f)->fil)
_Static_assert(sizeof(((uvm2_sd_file *)0)->fil) >= sizeof(FIL), "uvm2_sd_file.fil is smaller than FatFs's FIL");

void uvm2_sd_close(uvm2_sd_file *f)
{
    if (f->magic != UVM2_SD_OPEN_MAGIC) return;
    f_close(FIL_OF(f));
    f->magic = 0;
    f->ok = 0;
    if (uvm2_sd_diag.streams) uvm2_sd_diag.streams--;
}

int uvm2_sd_open(const char *path, uvm2_sd_file *f)
{
    uint32_t len;
    uvm2_sd_close(f);                         /* reopening the same struct: drop the old one */
    f->cluster = f->sec = f->pos = f->len = 0;
    f->ok = 0;
    f->magic = 0;
    if (!begin() || !open_file(FIL_OF(f), path, FA_READ, &len)) return 0;
    f->cluster = (uint32_t)FIL_OF(f)->obj.sclust;
    f->len = len;
    f->ok = 1;
    f->magic = UVM2_SD_OPEN_MAGIC;
    uvm2_sd_diag.streams++;
    return 1;
}

uint32_t uvm2_sd_next(uvm2_sd_file *f, unsigned char *dst, uint32_t max)
{
    if (!f->ok || f->magic != UVM2_SD_OPEN_MAGIC) return 0;
    uint32_t want = f->len - f->pos;
    if (want > max) want = max;
    if (want == 0) return 0;                  /* the end: not an error */
    UINT got = 0;
    FRESULT fr = f_read(FIL_OF(f), dst, want, &got);
    if (fr == FR_OK && got != want) fr = FR_DISK_ERR;
    f->pos += got;
    if (!fail(fr)) f->ok = 0;                 /* loud: uvm2_sd_error and diag.fresult */
    return got;
}
