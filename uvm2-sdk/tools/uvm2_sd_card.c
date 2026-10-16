/* uvm2_sd_card.c — an SD card in SPI mode, byte by byte, for uvm2_sd.c built with
 * -DUVM2_SD_FAKECARD. The driver's own protocol code runs against it; the blocks come from
 * the disk image through uvm2_host_read/uvm2_host_write (tools/uvm2_sd_test.c).
 *
 * IT IS STRICT WHERE A REAL CARD IS UNFORGIVING, and every breach is a VIOLATION, counted
 * and printed, rather than a wrong byte the file system may or may not notice:
 *
 *   - the byte after CMD12's frame is a stuff byte with bit 7 CLEAR, so a driver that takes
 *     it for the R1 is caught by what follows;
 *   - after CMD12 and after a write the card is BUSY (MISO low) for a while, and a command
 *     sent before it lets go is a violation;
 *   - a command other than CMD12 in the middle of a CMD18 run, or CS released in the middle
 *     of one, is a violation;
 *   - every data token is preceded by some 0xFF bytes of access latency, varying per block.
 *
 * Faults, from the environment, comma-separated in UVM2_CARD:
 *
 *   nocmd18   CMD18 answers "illegal command": the driver must fall back to CMD17
 *   errtoken  the first CMD18 run sends an error token instead of its second block
 *   sdsc      a byte-addressed card (no CCS in the OCR): every address must be a multiple of 512
 *
 * At exit it prints one line, which tools/uvm2_sd_test.sh reads:
 *
 *   card cmd17=<n> cmd18=<n> cmd12=<n> cmd24=<n> blocks=<n> clocked=<bytes> viol=<n>
 *
 * `blocks` counts blocks delivered, not the one a CMD18 run had started when CMD12 arrived.
 * `clocked` is every byte that crossed the wire with CS low. It is NOT the card's time: the
 * access latency of a real card is what a simulator cannot know. It is the protocol's share.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int uvm2_host_read(uint32_t lba, unsigned char *b);
int uvm2_host_write(uint32_t lba, const unsigned char *b);

static int f_nocmd18, f_errtoken, f_sdsc;
static int ready_env;

static struct {
    unsigned cmd17, cmd18, cmd12, cmd24, blocks, viol;
    unsigned long clocked;
} st;

static enum { IDLE, STREAM, STREAM_DEAD, WTOKEN, WDATA } mode = IDLE;
static int      sel;
static int      idle_state = 1, app_cmd, acmd41_tries;
static unsigned char q[1024];
static unsigned qh, qt;
static unsigned busy;                 /* bytes of 0x00 still to come once q is empty */
static unsigned char frame[6];
static unsigned nf;
static uint32_t stream_lba;
static unsigned stream_block, runs;
static unsigned char wbuf[514];
static unsigned nw;
static uint32_t wlba;
static uint32_t rng = 12345;

static void viol(const char *what)
{
    if (st.viol++ < 10) fprintf(stderr, "card: VIOLATION: %s\n", what);
}

static void report(void)
{
    printf("card cmd17=%u cmd18=%u cmd12=%u cmd24=%u blocks=%u clocked=%lu viol=%u\n",
           st.cmd17, st.cmd18, st.cmd12, st.cmd24, st.blocks, st.clocked, st.viol);
}

static void env(void)
{
    if (ready_env) return;
    ready_env = 1;
    const char *e = getenv("UVM2_CARD");
    if (e) {
        f_nocmd18  = strstr(e, "nocmd18")  != 0;
        f_errtoken = strstr(e, "errtoken") != 0;
        f_sdsc     = strstr(e, "sdsc")     != 0;
    }
    atexit(report);
}

static void push(uint8_t b)
{
    if (qt >= sizeof q) { viol("card queue overflow (simulator bug)"); return; }
    q[qt++] = b;
}
static void reset_q(void) { qh = qt = 0; }

/* 0..8 bytes of 0xFF: the card looking for the block before it says 0xFE. */
static void latency(void)
{
    rng = rng * 1103515245u + 12345u;
    for (unsigned n = (rng >> 16) % 9u; n; n--) push(0xFF);
}

static int to_lba(uint32_t arg, uint32_t *lba)
{
    if (!f_sdsc) { *lba = arg; return 1; }
    if (arg % 512u) { viol("byte address not a multiple of 512 on an SDSC card"); return 0; }
    *lba = arg / 512u;
    return 1;
}

static void push_block(uint32_t lba)
{
    unsigned char b[512];
    if (!uvm2_host_read(lba, b)) { viol("read past the end of the image"); memset(b, 0, 512); }
    latency();
    push(0xFE);
    for (int i = 0; i < 512; i++) push(b[i]);
    push(0x5A); push(0xA5);           /* a CRC nobody checks */
    st.blocks++;
}

static void refill_stream(void)
{
    reset_q();
    if (f_errtoken && runs == 1 && stream_block == 1) {
        latency();
        push(0x08);                   /* data error token: out of range */
        mode = STREAM_DEAD;
        return;
    }
    push_block(stream_lba + stream_block);
    stream_block++;
}

static void r1(uint8_t v) { push(0xFF); push(v); }   /* NCR of one byte, then R1 */

static void exec(void)
{
    const unsigned cmd = frame[0] & 0x3F;
    const uint32_t arg = (uint32_t)frame[1] << 24 | (uint32_t)frame[2] << 16 |
                         (uint32_t)frame[3] << 8 | frame[4];
    const int      app = app_cmd;
    uint32_t       lba;
    app_cmd = 0;

    if (mode == STREAM || mode == STREAM_DEAD) {
        if (cmd != 12) { viol("command other than CMD12 during a CMD18 run"); return; }
        if (mode == STREAM && qh != qt) st.blocks--;   /* prefetched, never delivered */
        reset_q();
        st.cmd12++;
        mode = IDLE;
        push(0x3F);                   /* the stuff byte: bit 7 clear, NOT an R1 */
        push(0xFF);
        push(0x00);                   /* the R1 */
        busy = 40;
        return;
    }
    reset_q();
    switch (cmd) {
    case 0:
        if (frame[5] != 0x95) viol("CMD0 without its CRC");
        idle_state = 1; acmd41_tries = 0;
        r1(0x01);
        return;
    case 8:
        if (frame[5] != 0x87) viol("CMD8 without its CRC");
        r1(idle_state ? 0x01 : 0x00);
        push(0x00); push(0x00); push(0x01); push(0xAA);
        return;
    case 55:
        app_cmd = 1;
        r1(idle_state ? 0x01 : 0x00);
        return;
    case 41:
        if (!app) { r1(0x04); return; }
        if (++acmd41_tries >= 3) idle_state = 0;
        r1(idle_state ? 0x01 : 0x00);
        return;
    case 58:
        r1(idle_state ? 0x01 : 0x00);
        push(f_sdsc ? 0x80 : 0xC0); push(0xFF); push(0x80); push(0x00);
        return;
    case 12:
        viol("CMD12 outside a CMD18 run");
        r1(0x04);
        return;
    case 17: case 18: case 24:
        if (idle_state) { viol("data command before the card left idle"); r1(0x01); return; }
        if (!to_lba(arg, &lba)) { r1(0x40); return; }   /* address error */
        if (cmd == 17) { st.cmd17++; r1(0x00); push_block(lba); return; }
        if (cmd == 18) {
            if (f_nocmd18) { r1(0x04); return; }
            st.cmd18++; runs++;
            r1(0x00);
            mode = STREAM; stream_lba = lba; stream_block = 0;
            return;
        }
        st.cmd24++; r1(0x00);
        mode = WTOKEN; wlba = lba;
        return;
    default:
        r1(0x04);
    }
}

void uvm2_card_cs(int selected)
{
    env();
    if (!selected && sel) {
        if (mode == STREAM || mode == STREAM_DEAD) viol("CS released in the middle of a CMD18 run");
        if (mode == WTOKEN || mode == WDATA) viol("CS released in the middle of a write");
        mode = IDLE;
        reset_q();
        nf = 0;
    }
    sel = selected;
}

uint8_t uvm2_card_xfer(uint8_t in)
{
    env();
    if (!sel) return 0xFF;            /* the 80 start-up clocks, CS high */
    st.clocked++;

    const int was_busy = qh == qt && busy;   /* MISO is low on THIS byte */
    uint8_t out;
    if (qh == qt && mode == STREAM) refill_stream();
    if (qh != qt)  out = q[qh++];
    else if (busy) { busy--; out = 0x00; }
    else           out = 0xFF;

    switch (mode) {
    case WTOKEN:
        if (in == 0xFE) { mode = WDATA; nw = 0; }
        else if (in != 0xFF) viol("garbage before the write's data token");
        return out;
    case WDATA:
        wbuf[nw++] = in;
        if (nw == sizeof wbuf) {
            if (!uvm2_host_write(wlba, wbuf)) viol("write past the end of the image");
            mode = IDLE;
            reset_q();
            push(0xE5);               /* data response: xxx0 010 1 = accepted */
            busy = 60;
        }
        return out;
    default:
        break;
    }

    if (nf == 0) {
        if ((in & 0xC0) != 0x40) return out;          /* 0xFF: the host is only clocking */
        if (was_busy) viol("command sent while the card was busy");
    }
    frame[nf++] = in;
    if (nf == 6) { nf = 0; exec(); }
    return out;
}
