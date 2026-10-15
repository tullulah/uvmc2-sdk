/* gdb.c — the monitor's second USB serial port speaks gdb's Remote Serial Protocol, so a plain
 * arm-none-eabi-gdb debugs the game on the UVMC2 with no probe and no program in between:
 *
 *     arm-none-eabi-gdb game.elf -ex "target remote /dev/cu.usbmodemXXX3"
 *
 * (the monitor's second port; tools/uvmc2.py gdb prints it). Attaching pauses the game; break,
 * step, next, finish, bt, print, set var, info registers, Ctrl-C, continue and detach work.
 * Underneath it is the same debugger as the IDE's (debug.c): breakpoints the monitor writes
 * as BKPTs and keeps, single steps, the registers of the held context. Use one of the two at
 * a time.
 *
 * The registers are described to gdb as an M-profile core (target.xml): r0-r12, sp, lr, pc,
 * xpsr, 32 bits each, in that order in 'g'. Only the game's core (core 0) is seen. */
#include <stdint.h>
#include <string.h>
#include "tusb.h"
#include "monitor.h"

#define GDB_ITF 1

static char s_pkt[600];
static unsigned s_n;
static enum { IDLE, DATA, SUM1, SUM2 } s_rx;
static int s_noack;
static int s_owe_stop;            /* c / s / ? / Ctrl-C: the stop reply comes when it stops */

static void out(const char *s, unsigned n) { tud_cdc_n_write(GDB_ITF, s, n); }

static void reply(const char *data)
{
    unsigned sum = 0, n = (unsigned)strlen(data);
    for (unsigned i = 0; i < n; i++) sum += (uint8_t)data[i];
    char t[3] = { '#', "0123456789abcdef"[(sum >> 4) & 15], "0123456789abcdef"[sum & 15] };
    out("$", 1); out(data, n); out(t, 3);
    tud_cdc_n_write_flush(GDB_ITF);
}

static int hexval(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; }

static uint32_t hexnum(const char **p)
{
    uint32_t v = 0;
    int d;
    while ((d = hexval(**p)) >= 0) { v = v * 16u + (uint32_t)d; (*p)++; }
    return v;
}

/* A register as gdb sends and wants it: target byte order, little-endian. */
static void put_le32(char *d, uint32_t v)
{
    for (int i = 0; i < 4; i++) { uint8_t b = (uint8_t)(v >> (8 * i)); d[2 * i] = "0123456789abcdef"[b >> 4]; d[2 * i + 1] = "0123456789abcdef"[b & 15]; }
}
static uint32_t get_le32(const char *s)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v |= (uint32_t)((hexval(s[2 * i]) << 4) | hexval(s[2 * i + 1])) << (8 * i);
    return v;
}

static const char s_target_xml[] =
    "<?xml version=\"1.0\"?><!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
    "<target version=\"1.0\"><architecture>arm</architecture>"
    "<feature name=\"org.gnu.gdb.arm.m-profile\">"
    "<reg name=\"r0\" bitsize=\"32\"/><reg name=\"r1\" bitsize=\"32\"/><reg name=\"r2\" bitsize=\"32\"/>"
    "<reg name=\"r3\" bitsize=\"32\"/><reg name=\"r4\" bitsize=\"32\"/><reg name=\"r5\" bitsize=\"32\"/>"
    "<reg name=\"r6\" bitsize=\"32\"/><reg name=\"r7\" bitsize=\"32\"/><reg name=\"r8\" bitsize=\"32\"/>"
    "<reg name=\"r9\" bitsize=\"32\"/><reg name=\"r10\" bitsize=\"32\"/><reg name=\"r11\" bitsize=\"32\"/>"
    "<reg name=\"r12\" bitsize=\"32\"/><reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>"
    "<reg name=\"lr\" bitsize=\"32\"/><reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
    "<reg name=\"xpsr\" bitsize=\"32\"/></feature></target>";

/* The stop reply: SIGINT for a pause (Ctrl-C), SIGTRAP for a breakpoint or a step. */
static void stop_reply(int why) { reply(why == DBG_PAUSE ? "S02" : "S05"); }

/* From debug.c (through dbg_events) when the game stops. */
void gdb_stopped(int why)
{
    if (!s_owe_stop) return;
    s_owe_stop = 0;
    stop_reply(why);
}

static void packet(char *p)
{
    static char r[600];
    const char *q = p + 1;
    int halted = dbg_halted();
    switch (p[0]) {
    case '?':
        if (halted) stop_reply(dbg_reason());
        else { s_owe_stop = 1; dbg_pause(); }    /* attaching pauses the game */
        return;
    case 'g':
        if (!halted) { reply("E01"); return; }
        for (int n = 0; n < 17; n++) put_le32(r + 8 * n, dbg_reg(n));
        r[8 * 17] = 0;
        reply(r);
        return;
    case 'G':
        if (!halted) { reply("E01"); return; }
        for (int n = 0; n < 17 && strlen(q) >= (size_t)(8 * n + 8); n++) if (n != 13) dbg_set_reg(n, get_le32(q + 8 * n));
        reply("OK");
        return;
    case 'p': {
        int n = (int)hexnum(&q);
        if (!halted || n > 16) { reply("E01"); return; }
        put_le32(r, dbg_reg(n)); r[8] = 0;
        reply(r);
        return;
    }
    case 'P': {
        int n = (int)hexnum(&q);
        if (!halted || *q != '=') { reply("E01"); return; }
        reply(dbg_set_reg(n, get_le32(q + 1)) ? "OK" : "E01");
        return;
    }
    case 'm': {
        uint32_t a = hexnum(&q); q++;
        uint32_t n = hexnum(&q);
        if (n > 256 || !mon_readable(a, n)) { reply("E14"); return; }
        for (uint32_t i = 0; i < n; i++) {
            uint8_t b = *(volatile uint8_t *)(a + i);
            r[2 * i] = "0123456789abcdef"[b >> 4]; r[2 * i + 1] = "0123456789abcdef"[b & 15];
        }
        r[2 * n] = 0;
        reply(r);
        return;
    }
    case 'M': {
        uint32_t a = hexnum(&q); q++;
        uint32_t n = hexnum(&q); q++;
        if (!mon_writable(a, n)) { reply("E14"); return; }
        for (uint32_t i = 0; i < n; i++) *(volatile uint8_t *)(a + i) = (uint8_t)((hexval(q[2 * i]) << 4) | hexval(q[2 * i + 1]));
        __asm volatile ("dsb; isb" ::: "memory");
        reply("OK");
        return;
    }
    case 'Z': case 'z': {
        int type = (int)hexnum(&q); q++;
        uint32_t a = hexnum(&q);
        if (type > 1) { reply(""); return; }            /* watchpoints: not here */
        reply(dbg_bp(p[0] == 'Z', a) ? "OK" : "E01");   /* hardware ones are the same BKPTs */
        return;
    }
    case 'c':
        if (!halted) { reply("E01"); return; }
        s_owe_stop = 1;
        dbg_resume(0);
        return;
    case 's':
        if (!halted) { reply("E01"); return; }
        s_owe_stop = 1;
        dbg_resume(1);
        return;
    case 'D':
        dbg_detach();
        reply("OK");
        return;
    case 'k':
        dbg_detach();
        return;
    case 'H': case 'T':
        reply("OK");
        return;
    case 'q':
        if (!strncmp(p, "qSupported", 10)) { reply("PacketSize=256;qXfer:features:read+;QStartNoAckMode+"); return; }
        if (!strcmp(p, "qAttached")) { reply("1"); return; }
        if (!strcmp(p, "qC")) { reply("QC1"); return; }
        if (!strcmp(p, "qfThreadInfo")) { reply("m1"); return; }
        if (!strcmp(p, "qsThreadInfo")) { reply("l"); return; }
        if (!strncmp(p, "qXfer:features:read:target.xml:", 31)) {
            const char *o = p + 31;
            uint32_t off = hexnum(&o); o++;
            uint32_t len = hexnum(&o);
            uint32_t total = sizeof s_target_xml - 1;
            if (off >= total) { reply("l"); return; }
            uint32_t n = total - off;
            if (n > len) n = len;
            if (n > sizeof r - 2) n = sizeof r - 2;
            r[0] = off + n < total ? 'm' : 'l';
            memcpy(r + 1, s_target_xml + off, n);
            r[1 + n] = 0;
            reply(r);
            return;
        }
        reply("");
        return;
    case 'Q':
        if (!strcmp(p, "QStartNoAckMode")) { reply("OK"); s_noack = 1; return; }
        reply("");
        return;
    default:
        reply("");   /* not supported: gdb falls back (X -> M, vCont -> c/s) */
        return;
    }
}

/* From poll (the USB interrupt): what gdb sent. */
void gdb_poll(void)
{
    if (!tud_cdc_n_connected(GDB_ITF) && !tud_cdc_n_available(GDB_ITF)) return;
    uint8_t c;
    while (tud_cdc_n_read(GDB_ITF, &c, 1) == 1) {
        switch (s_rx) {
        case IDLE:
            if (c == '$') { s_rx = DATA; s_n = 0; }
            else if (c == 0x03) { if (!dbg_halted()) { s_owe_stop = 1; dbg_pause(); } }   /* Ctrl-C */
            break;
        case DATA:
            if (c == '#') s_rx = SUM1;
            else if (s_n < sizeof s_pkt - 1) s_pkt[s_n++] = (char)c;
            break;
        case SUM1:
            s_rx = SUM2;
            break;
        case SUM2:
            s_rx = IDLE;
            s_pkt[s_n] = 0;
            if (!s_noack) out("+", 1);
            packet(s_pkt);
            break;
        }
    }
    tud_cdc_n_write_flush(GDB_ITF);
}
