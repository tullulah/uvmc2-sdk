/*
 * uvm2_usbaudio.c — see uvm2_usbaudio.h. A USB host of its own on the RP2350's controller,
 * for one thing: a USB Audio Class device (1 or 2) to play the jack's sound.
 *
 * HOW THE CONTROLLER IS DRIVEN (the RP2350's USB is the RP2040's; its host side as TinyUSB's
 * hcd_rp2040.c drives it, pico-sdk 2.2.0):
 *   - Control transfers go through EPX, polled: SETUP with SEND_SETUP|START_TRANS until
 *     SIE_STATUS.TRANS_COMPLETE, then one packet per START_TRANS (LAST set) until BUF_STATUS
 *     bit 0, then the status stage the same way. Only during enumeration, which blocks the
 *     game's core for a few tens of ms when headphones are plugged in.
 *   - The stream goes through EPX too, set to ISOCHRONOUS once the device is configured: one
 *     transaction a packet, started with SOF_SYNC from the interrupt of the last one, so one
 *     goes out each millisecond. (An interrupt endpoint set to isochronous sent one packet and
 *     never came back: see stream_start.)
 *   - Isochronous at full speed is always DATA0 and has no handshake.
 *
 * THE SOUND: a ring of stereo frames at UVM2_JACK_RATE, written by uvm2_jack (the same
 * samples as the DAC gets), read by the interrupt and resampled (linear) to the device's
 * rate, 48 or 44.1 kHz, a millisecond's worth per packet. The USB frame clock and the jack's
 * PIO both come from the RP2350's crystal, so the ring neither fills nor empties over time;
 * a device with its own clock (asynchronous) asks for feedback this does not give, and may
 * slip a sample now and then.
 *
 * MEASURED on the UVMC2 (dual core, PIO+DMA stream), Apple USB-C EarPods (05AC:1108, UAC2,
 * 16-bit stereo at 48 kHz): 1000 packets a second, none of them empty over 20,000, a clean
 * test tone (hardware/uvm2/usbtone). What it took: EPX and not an interrupt endpoint, and
 * SVCall below the USB interrupt (uvm2_usbaudio_start).
 */
#include "uvm2_usbaudio.h"
#include "uvm2_jack.h"

#if defined(VPY_RP2350) && !defined(UVM2_USB_MONITOR) && !defined(UVM2_NO_USB_AUDIO)
#include <string.h>
#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/resets.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico.h"
#include "pico/time.h"

/* the controller's atomic set / clear aliases (as TinyUSB's rp2040_usb.h names them) */
#define usb_hw_set   ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

enum { ST_OFF, ST_WAIT, ST_ENUM, ST_PLAY, ST_CANT, ST_FAIL };

#define ADDR        1u                  /* the one device's address */
#define EPX_OFF     0x180u              /* EPX's buffer in the DPRAM (control transfers) */
#define ISO_OFF     0x200u              /* the stream's buffer: 64-byte aligned, up to 1023 */
#define HID_OFF     0x600u              /* the buttons' reports (past the stream's 1023) */
#define SETTLE_US   100000u             /* attach to first request: USB 2.0 §7.1.7.3 */
#define XFER_US     50000u              /* one control transfer, at most */
#define SIE_BASE    (USB_SIE_CTRL_SOF_EN_BITS | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS | \
                     USB_SIE_CTRL_PULLDOWN_EN_BITS | USB_SIE_CTRL_EP0_INT_1BUF_BITS)

/* ── the ring ── */
/* THE RING IS THE JACK'S (uvm2_usbaudio_start). Its own was 8 KB of .bss in every game, and
 * with v1.4.15 nine AAE ports stopped linking; the jack's DMA ring sits idle with SOUND USB,
 * since uvm2_jack stops the DAC before it routes here. Back to JACK, the DAC writes the ring
 * again, but the reader below only takes what uvm2_usbaudio_write_lr wrote (s_w - r), so the
 * headphones get silence, not the DAC's samples. */
#define RING        s_ring_words
#define PREFILL     1024u               /* frames behind the writer when a stream starts: 32 ms,
                                           more than one game frame's burst (640 at 50 Hz) */
static uint32_t *s_ring;                /* L << 16 | R; the jack's, set by uvm2_usbaudio_start */
static uint32_t s_ring_words = 1u;
static volatile uint32_t s_w, s_r;      /* free-running: frames written / read */
static uint32_t s_frac, s_step;         /* the reader's position between frames, Q16 */

/* ── the device ── */
static int s_state, s_started, s_irq;
static uint32_t s_t_attach;
static uint8_t s_mps0 = 8;
static uint16_t s_vid, s_pid;
static uint8_t s_cfg[1024];
static int s_uac;                       /* 1 or 2 */
static int s_ac_if, s_as_if, s_as_alt, s_ep, s_mps, s_ch, s_sub, s_bits, s_sync, s_clock_id;
static uint32_t s_rate, s_rate_acc;
static uint32_t s_rates1[8];            /* UAC1: the rates its format lists */
static int s_nrates1;
static volatile uint32_t s_packets, s_under, s_irqs;
static uint32_t s_t_sec, s_last_p, s_last_u, s_pps, s_ups;   /* per second, for the report */

static int s_err_step;                  /* which request failed, for the report */

/* ── the headphones' buttons (their HID interface) and the volume they move ── */
enum { BTN_VOLUP, BTN_VOLDN, BTN_PLAY, BTN_MUTE, NBTN };
static const uint16_t BTN_USAGE[NBTN] = { 0xE9, 0xEA, 0xCD, 0xE2 };   /* HID Consumer page */
static int s_hid_if, s_hid_ep, s_hid_mps, s_hid_ival, s_hid_rlen, s_hid_ids, s_hid_pid;
static uint8_t s_rdesc[256];
static int16_t s_bit_id[NBTN], s_bit_at[NBTN];  /* a button as one bit of an input report, -1 none */
static int s_arr_id = -1, s_arr_at, s_arr_size, s_arr_count, s_arr_lmin, s_arr_n;
static uint16_t s_arr_usage[16];        /* a button as a value of an array field: its usages */
static volatile uint32_t s_buttons, s_hid_reports;
#define VOL_STEPS   16                  /* 3 dB each: 16 is full, 0 is silent */
#define VOL_DEFAULT 12                  /* -12 dB: the EarPods at full are loud in a quiet room */
static volatile int s_vol = VOL_DEFAULT, s_mute;
static volatile uint32_t s_vol_t;       /* when the buttons last moved it */
static int s_vol_saved = -1;            /* what was last written to the config */
#define VOL_SAVE_US 2000000u            /* saved once it has stayed put this long */
/* THE SAVED VOLUME (uvm2_config.c, the console's config). Weak, for a build without the config
   (the debug board's): there the level simply starts at VOL_DEFAULT every time. */
__attribute__((weak)) volatile int32_t uvm2_setting_usb_volume = -1;
__attribute__((weak)) int uvm2_config_save(void);
static int32_t s_gain_q15;              /* what fill_packet multiplies by */

/* ════════════════════════════════════════ the controller */

static void usb_clock(void)
{
    /* clk_usb at 48 MHz from PLL_USB, as uvm2_monitor.c does it: left alone if it runs */
    if (!(pll_usb_hw->cs & PLL_CS_LOCK_BITS)) pll_init(pll_usb, 1, 1440000000u, 6, 5);
    if (!(clocks_hw->clk[clk_usb].ctrl & CLOCKS_CLK_USB_CTRL_ENABLE_BITS))
        clock_configure_undivided(clk_usb, 0, CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48000000u);
}

static void host_up(void)
{
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset((void *)usbh_dpram, 0, sizeof(*usbh_dpram));
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS | USB_MAIN_CTRL_HOST_NDEVICE_BITS;
    usb_hw->sie_ctrl = SIE_BASE;
    usb_hw->inte = 0;
}

static int speed(void)
{
    return (int)((usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB);
}

static void start(uint32_t flags)
{
    usb_hw->sie_ctrl = SIE_BASE | flags;
    busy_wait_at_least_cycles(12);
    usb_hw->sie_ctrl = SIE_BASE | flags | USB_SIE_CTRL_START_TRANS_BITS;
}

/* until the setup went (sie) or EPX's buffer came back (buf); 0, or -1 timeout, -2 stall */
static int wait_done(int buf)
{
    const uint32_t t0 = time_us_32();
    for (;;) {
        const uint32_t st = usb_hw->sie_status;
        if (st & USB_SIE_STATUS_STALL_REC_BITS) {
            usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
            return -2;
        }
        if (st & (USB_SIE_STATUS_RX_TIMEOUT_BITS | USB_SIE_STATUS_DATA_SEQ_ERROR_BITS))
            usb_hw_clear->sie_status = st & (USB_SIE_STATUS_RX_TIMEOUT_BITS | USB_SIE_STATUS_DATA_SEQ_ERROR_BITS);
        if (buf ? (usb_hw->buf_status & 1u) : (st & USB_SIE_STATUS_TRANS_COMPLETE_BITS)) {
            usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
            if (buf) usb_hw_clear->buf_status = 1u;
            return 0;
        }
        if (!speed() || time_us_32() - t0 > XFER_US) return -1;
    }
}

/* one EPX packet: OUT with its bytes or IN into dst; returns its length or < 0 */
static int packet(int in, int pid, const uint8_t *src, uint8_t *dst, int n)
{
    volatile uint8_t *buf = (volatile uint8_t *)usbh_dpram + EPX_OFF;
    uint32_t bc = (uint32_t)n | USB_BUF_CTRL_LAST | (pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID);
    if (!in) {
        for (int i = 0; i < n; i++) buf[i] = src[i];
        bc |= USB_BUF_CTRL_FULL;
    }
    usbh_dpram->epx_buf_ctrl = bc;
    usbh_dpram->epx_buf_ctrl = bc | USB_BUF_CTRL_AVAIL;
    start(in ? USB_SIE_CTRL_RECEIVE_DATA_BITS : USB_SIE_CTRL_SEND_DATA_BITS);
    const int e = wait_done(1);
    if (e) return e;
    const int got = (int)(usbh_dpram->epx_buf_ctrl & USB_BUF_CTRL_LEN_MASK);
    if (in) for (int i = 0; i < got && i < n; i++) dst[i] = buf[i];
    return in ? got : n;
}

/* a control transfer to `addr`; returns the bytes of the data stage, or < 0 */
static int ctrl(uint8_t addr, uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, uint8_t *data, uint16_t len)
{
    const uint8_t setup[8] = { rt, req, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8),
                               (uint8_t)len, (uint8_t)(len >> 8) };
    for (int i = 0; i < 8; i++) usbh_dpram->setup_packet[i] = setup[i];
    usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER | EPX_OFF;
    usb_hw->dev_addr_ctrl = addr;
    usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
    usb_hw_clear->buf_status = 1u;
    start(USB_SIE_CTRL_SEND_SETUP_BITS);
    int e = wait_done(0);
    if (e) return e;
    const int in = rt & 0x80;
    int done = 0, pid = 1;
    while (done < len) {
        int n = len - done;
        if (n > s_mps0) n = s_mps0;
        const int got = packet(in, pid, data + done, data + done, n);
        if (got < 0) return got;
        done += got;
        pid ^= 1;
        if (in && got < n) break;               /* a short packet ends the data stage */
    }
    e = packet(!in, 1, 0, 0, 0);                 /* status: the other way, DATA1, empty */
    return e < 0 ? e : done;
}

/* ════════════════════════════════════════ what the device is */

/* its playback: the streaming alternate with an isochronous OUT endpoint, 16-bit stereo
   preferred, then 24-bit stereo, then mono; and where its sample rate is set */
static int parse(const uint8_t *d, int len)
{
    int cls = -1, sub = -1, ifn = -1, alt = 0;
    int link = -1, ch = 0, subslot = 0, bits = 0, ep = 0, mps = 0, sync = 0, nr = 0;
    uint32_t rates[8];
    int best = -1;
    int term_id[8], term_clock[8], nterm = 0, first_clock = -1;
    s_uac = 0;
    s_hid_if = -1; s_hid_ep = 0; s_hid_rlen = 0;
    for (int i = 0; i + 2 <= len && d[i] >= 2; i += d[i]) {
        const uint8_t *x = d + i;
        const int at_end = i + d[i] + 1 >= len || d[i + d[i] + 1] == 4;   /* next is an interface */
        if (x[1] == 4 && x[0] >= 9) {                                   /* INTERFACE */
            ifn = x[2]; alt = x[3]; cls = x[5]; sub = x[6];
            if (cls == 1 && sub == 1) { s_ac_if = ifn; s_uac = x[7] == 0x20 ? 2 : 1; }
            if (cls == 3 && s_hid_if < 0) s_hid_if = ifn;           /* HID: the buttons */
            link = -1; ch = 0; subslot = 0; bits = 0; ep = 0; mps = 0; sync = 0; nr = 0;
        } else if (x[1] == 0x24 && cls == 1 && sub == 1) {              /* AudioControl */
            if (x[2] == 0x0A && x[0] >= 4 && first_clock < 0) first_clock = x[3];       /* UAC2 clock source */
            if (x[2] == 0x02 && nterm < 8) {                            /* input terminal */
                term_id[nterm] = x[3];
                term_clock[nterm] = (s_uac == 2 && x[0] >= 8) ? x[7] : -1;
                nterm++;
            }
        } else if (x[1] == 0x24 && cls == 1 && sub == 2) {              /* AudioStreaming */
            if (x[2] == 1) {                                            /* AS_GENERAL */
                link = x[3];
                if (s_uac == 2 && x[0] >= 11) ch = x[10];
            } else if (x[2] == 2 && x[3] == 1) {                        /* FORMAT_TYPE I */
                if (s_uac == 2 && x[0] >= 6) { subslot = x[4]; bits = x[5]; }
                else if (x[0] >= 8) {
                    ch = x[4]; subslot = x[5]; bits = x[6];
                    for (int k = 0; k < x[7] && nr < 8 && 8 + 3 * k + 2 < x[0]; k++)
                        rates[nr++] = x[8 + 3 * k] | x[9 + 3 * k] << 8 | (uint32_t)x[10 + 3 * k] << 16;
                }
            }
        } else if (x[1] == 0x21 && cls == 3 && ifn == s_hid_if && x[0] >= 9) {   /* HID descriptor */
            if (x[6] == 0x22) s_hid_rlen = x[7] | x[8] << 8;                 /* its report descriptor */
        } else if (x[1] == 5 && x[0] >= 7 && cls == 3 && ifn == s_hid_if && (x[3] & 3) == 3 && (x[2] & 0x80)) {
            if (!s_hid_ep) { s_hid_ep = x[2]; s_hid_mps = x[4] > 64 ? 64 : x[4]; s_hid_ival = x[6]; }
        } else if (x[1] == 5 && x[0] >= 7 && cls == 1 && sub == 2) {   /* ENDPOINT */
            if ((x[3] & 3) == 1 && !(x[2] & 0x80)) { ep = x[2]; mps = x[4] | (x[5] & 7) << 8; sync = (x[3] >> 2) & 3; }
        }
        /* an alternate is complete when the next descriptor starts another interface */
        if (at_end && cls == 1 && sub == 2 && alt > 0 && ep && ch >= 1 && ch <= 2 &&
            subslot >= 2 && subslot <= 4) {
            const int score = (ch == 2 ? 10 : 0) + (subslot == 2 ? 3 : subslot == 3 ? 2 : 1);
            if (score > best) {
                best = score;
                s_as_if = ifn; s_as_alt = alt; s_ep = ep; s_mps = mps; s_ch = ch;
                s_sub = subslot; s_bits = bits; s_sync = sync;
                s_nrates1 = nr; for (int k = 0; k < nr; k++) s_rates1[k] = rates[k];
                s_clock_id = first_clock;
                for (int k = 0; k < nterm; k++) if (term_id[k] == link && term_clock[k] >= 0) s_clock_id = term_clock[k];
            }
            ep = 0;                     /* this alternate is counted */
        }
    }
    return best >= 0;
}

/* the sample rate: 48 kHz if it takes it */
static int set_rate(void)
{
    if (s_uac == 2) {
        if (s_clock_id < 0) return -1;
        uint8_t f[4] = { 0x80, 0xBB, 0x00, 0x00 };                     /* 48000 */
        ctrl(ADDR, 0x21, 0x01, 0x0100, (uint16_t)(s_clock_id << 8 | s_ac_if), f, 4);   /* SET_CUR SAM_FREQ */
        if (ctrl(ADDR, 0xA1, 0x01, 0x0100, (uint16_t)(s_clock_id << 8 | s_ac_if), f, 4) == 4) {
            s_rate = f[0] | f[1] << 8 | (uint32_t)f[2] << 16 | (uint32_t)f[3] << 24;
        } else {
            s_rate = 48000u;
        }
    } else {
        s_rate = s_nrates1 ? s_rates1[0] : 48000u;
        for (int k = 0; k < s_nrates1; k++) if (s_rates1[k] == 48000u || s_rates1[k] == 44100u) { s_rate = s_rates1[k]; break; }
        uint8_t f[3] = { (uint8_t)s_rate, (uint8_t)(s_rate >> 8), (uint8_t)(s_rate >> 16) };
        ctrl(ADDR, 0x22, 0x01, 0x0100, (uint16_t)s_ep, f, 3);         /* SET_CUR to the endpoint */
    }
    if (s_rate < 8000u || s_rate > 96000u) return -1;
    if ((s_rate + 999u) / 1000u * (uint32_t)s_ch * (uint32_t)s_sub > (uint32_t)s_mps) return -1;
    return 0;
}

/* ════════════════════════════════════════ the stream */

static inline int16_t lerp16(int a, int b, uint32_t f) { return (int16_t)(a + (((b - a) * (int)(f >> 1)) >> 15)); }

static void __not_in_flash_func(fill_packet)(void)
{
    s_rate_acc += s_rate;
    const uint32_t nf = s_rate_acc / 1000u;
    s_rate_acc -= nf * 1000u;
    volatile uint8_t *p = (volatile uint8_t *)usbh_dpram + ISO_OFF;
    uint32_t r = s_r;
    for (uint32_t k = 0; k < nf; k++) {
        int16_t l = 0, rr = 0;
        if (s_w - r >= 2u) {
            const uint32_t a = s_ring[r & (RING - 1u)], b = s_ring[(r + 1u) & (RING - 1u)];
            l = lerp16((int16_t)(a >> 16), (int16_t)(b >> 16), s_frac);
            rr = lerp16((int16_t)a, (int16_t)b, s_frac);
            s_frac += s_step;
            r += s_frac >> 16;
            s_frac &= 0xFFFFu;
        } else {
            s_under++;
        }
        int16_t v[2] = { (int16_t)((l * s_gain_q15) >> 15), (int16_t)((rr * s_gain_q15) >> 15) };
        if (s_ch == 1) v[0] = (int16_t)((v[0] + v[1]) / 2);
        for (int c = 0; c < s_ch; c++) {
            for (int z = 2; z < s_sub; z++) *p++ = 0;                  /* 24/32-bit: zeros below */
            *p++ = (uint8_t)v[c];
            *p++ = (uint8_t)((uint16_t)v[c] >> 8);
        }
    }
    s_r = r;
    const uint32_t bytes = nf * (uint32_t)s_ch * (uint32_t)s_sub;
    const uint32_t bc = bytes | USB_BUF_CTRL_FULL | USB_BUF_CTRL_LAST | USB_BUF_CTRL_DATA0_PID;
    usbh_dpram->epx_buf_ctrl = bc;
    usbh_dpram->epx_buf_ctrl = bc | USB_BUF_CTRL_AVAIL;
    /* SOF_SYNC: it goes out after the next start of frame, so one a millisecond however soon
       after the last one this runs */
    start(USB_SIE_CTRL_SEND_DATA_BITS | USB_SIE_CTRL_SOF_SYNC_BITS);
    s_packets++;
}

static void set_gain(void)
{
    int32_t g = 32767;                                  /* full, then -3 dB a step: x 1/sqrt(2) */
    for (int v = VOL_STEPS; v > s_vol; v--) g = (g * 23170) >> 15;
    s_gain_q15 = (s_mute || s_vol <= 0) ? 0 : g;
}

static uint32_t bits(const uint8_t *d, int n, int at, int size)
{
    uint32_t v = 0;
    for (int k = 0; k < size && k < 32; k++) {
        const int b = at + k;
        if (b >> 3 >= n) break;
        v |= (uint32_t)(d[b >> 3] >> (b & 7) & 1u) << k;
    }
    return v;
}

/* one input report: which buttons are down; a button going down moves the volume */
static void __not_in_flash_func(hid_report)(const uint8_t *r, int n)
{
    int id = 0;
    if (s_hid_ids && n > 0) { id = r[0]; r++; n--; }
    uint32_t down = 0;
    for (int k = 0; k < NBTN; k++)
        if (s_bit_at[k] >= 0 && s_bit_id[k] == id && bits(r, n, s_bit_at[k], 1)) down |= 1u << k;
    if (s_arr_id == id && s_arr_size > 0)
        for (int c = 0; c < s_arr_count; c++) {
            const int v = (int)bits(r, n, s_arr_at + c * s_arr_size, s_arr_size) - s_arr_lmin;
            if (v < 0 || v >= s_arr_n) continue;
            for (int k = 0; k < NBTN; k++) if (s_arr_usage[v] == BTN_USAGE[k]) down |= 1u << k;
        }
    const uint32_t pressed = down & ~s_buttons;
    if (pressed & (1u << BTN_VOLUP) && s_vol < VOL_STEPS) { s_vol++; s_mute = 0; }
    if (pressed & (1u << BTN_VOLDN) && s_vol > 0) s_vol--;
    if (pressed & (1u << BTN_MUTE)) s_mute = !s_mute;
    if (pressed) { set_gain(); s_vol_t = time_us_32(); }
    s_buttons = down;
    s_hid_reports++;
}

/* the buttons' endpoint: interrupt IN on the controller's interrupt endpoint 1, which polls it
   by itself every bInterval ms (isochronous was what did not work there, not this) */
static void __not_in_flash_func(hid_arm)(void)
{
    const uint32_t bc = (uint32_t)s_hid_mps | USB_BUF_CTRL_LAST |
                        (s_hid_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID);
    usbh_dpram->int_ep_buffer_ctrl[0].ctrl = bc;
    usbh_dpram->int_ep_buffer_ctrl[0].ctrl = bc | USB_BUF_CTRL_AVAIL;
}

static void __not_in_flash_func(usb_irq)(void)
{
    const uint32_t bs = usb_hw->buf_status;
    s_irqs++;
    if (bs) usb_hw_clear->buf_status = bs;
    usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS | USB_SIE_STATUS_RX_TIMEOUT_BITS |
                               USB_SIE_STATUS_DATA_SEQ_ERROR_BITS | USB_SIE_STATUS_NAK_REC_BITS |
                               USB_SIE_STATUS_ACK_REC_BITS;
    if (bs & 1u) fill_packet();                        /* EPX's packet went: the next one */
    if (bs & 0x0Cu) {                                  /* interrupt endpoint 1: a button report */
        const int n = (int)(usbh_dpram->int_ep_buffer_ctrl[0].ctrl & USB_BUF_CTRL_LEN_MASK);
        uint8_t r[64];
        const volatile uint8_t *src = (volatile uint8_t *)usbh_dpram + HID_OFF;
        for (int i = 0; i < n && i < 64; i++) r[i] = src[i];
        s_hid_pid ^= 1;
        hid_report(r, n < 64 ? n : 64);
        hid_arm();
    }
}

static void hid_start(void)
{
    if (s_hid_if < 0 || !s_hid_ep) return;
    usbh_dpram->int_ep_ctrl[0].ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
                                      (3u << EP_CTRL_BUFFER_TYPE_LSB) | HID_OFF |      /* 3 = interrupt */
                                      (uint32_t)(s_hid_ival > 1 ? s_hid_ival - 1 : 0) << EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    usb_hw->int_ep_addr_ctrl[0] = ADDR | (uint32_t)(s_hid_ep & 0x0F) << USB_ADDR_ENDP1_ENDPOINT_LSB;   /* IN */
    s_hid_pid = 0;
    hid_arm();
    usb_hw_set->int_ep_ctrl = 1u << 1;
}

/* the report descriptor: where the Consumer page's volume, play and mute usages land in an
   input report, as single bits (a Variable field) or as values of an Array field */
static void parse_report(const uint8_t *d, int len)
{
    int page = 0, rsize = 0, rcount = 0, rid = 0, lmin = 0, nu = 0, umin = -1, umax = -1;
    uint16_t usage[16];
    int ids[8], offs[8], nid = 0;
    for (int k = 0; k < NBTN; k++) { s_bit_id[k] = 0; s_bit_at[k] = -1; }
    s_arr_id = -1; s_hid_ids = 0;
    for (int i = 0; i < len; ) {
        const uint8_t b = d[i];
        if (b == 0xFE) { i += 3 + (i + 1 < len ? d[i + 1] : 0); continue; }      /* a long item */
        int sz = b & 3; if (sz == 3) sz = 4;
        const int type = (b >> 2) & 3, tag = b >> 4;
        uint32_t v = 0;
        for (int k = 0; k < sz && i + 1 + k < len; k++) v |= (uint32_t)d[i + 1 + k] << (8 * k);
        int32_t sv = (int32_t)v;
        if (sz == 1 && (v & 0x80)) sv = (int32_t)v - 0x100;
        if (sz == 2 && (v & 0x8000)) sv = (int32_t)v - 0x10000;
        i += 1 + sz;
        if (type == 1) {                                         /* global */
            if (tag == 0) page = (int)v;
            else if (tag == 1) lmin = sv;
            else if (tag == 7) rsize = (int)v;
            else if (tag == 8) { rid = (int)v; s_hid_ids = 1; }
            else if (tag == 9) rcount = (int)v;
        } else if (type == 2) {                                  /* local */
            if (tag == 0 && nu < 16) usage[nu++] = (uint16_t)v;
            else if (tag == 1) umin = (int)v;
            else if (tag == 2) umax = (int)v;
        } else if (type == 0) {                                  /* main */
            if (tag == 8) {                                      /* Input */
                int slot = -1;
                for (int k = 0; k < nid; k++) if (ids[k] == rid) slot = k;
                if (slot < 0 && nid < 8) { slot = nid++; ids[slot] = rid; offs[slot] = 0; }
                const int at = slot >= 0 ? offs[slot] : 0;
                if (page == 0x0C && !(v & 1)) {                  /* Consumer, data (not constant) */
                    if (v & 2) {                                 /* Variable: a bit a usage */
                        for (int j = 0; j < rcount; j++) {
                            const int u = j < nu ? usage[j] : umin >= 0 ? umin + j : nu ? usage[nu - 1] : 0;
                            for (int k = 0; k < NBTN; k++)
                                if (u == BTN_USAGE[k] && rsize == 1) { s_bit_id[k] = (int16_t)rid; s_bit_at[k] = (int16_t)(at + j); }
                        }
                    } else if (s_arr_id < 0) {                   /* Array: values name usages */
                        s_arr_id = rid; s_arr_at = at; s_arr_size = rsize; s_arr_count = rcount; s_arr_lmin = lmin;
                        s_arr_n = 0;
                        if (nu) for (int j = 0; j < nu && s_arr_n < 16; j++) s_arr_usage[s_arr_n++] = usage[j];
                        else if (umin >= 0) for (int u = umin; u <= umax && s_arr_n < 16; u++) s_arr_usage[s_arr_n++] = (uint16_t)u;
                    }
                }
                if (slot >= 0) offs[slot] += rsize * rcount;
            }
            nu = 0; umin = umax = -1;                            /* locals end with the main item */
        }
    }
    (void)umax;
}

/* THE STREAM GOES THROUGH EPX, not an interrupt endpoint. Interrupt endpoint 1 set to
   isochronous took the first packet and never raised its BUF_STATUS (on the cart: P1, the
   EarPods silent), so host mode does not run isochronous there. EPX, which carries control
   transfers too, is free once the device is configured: each packet is a transaction of its
   own, started from the interrupt of the last one with SOF_SYNC. */
static void stream_start(void)
{
    for (uint32_t i = 0; i < RING; i++) s_ring[i] = 0;
    s_r = 0; s_w = PREFILL; s_frac = 0; s_rate_acc = 0; s_packets = 0; s_under = 0; s_irqs = 0;
    s_step = (uint32_t)(((uint64_t)UVM2_JACK_RATE << 16) / s_rate);
    usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
                           (1u << EP_CTRL_BUFFER_TYPE_LSB) | ISO_OFF;          /* 1 = isochronous */
    usb_hw->dev_addr_ctrl = ADDR | (uint32_t)(s_ep & 0x0F) << USB_ADDR_ENDP_ENDPOINT_LSB;
    if (!s_irq) { irq_set_exclusive_handler(USBCTRL_IRQ, usb_irq); s_irq = 1; }
    usb_hw_clear->buf_status = 0xFFFFFFFFu;
    usb_hw->inte = USB_INTE_BUFF_STATUS_BITS;
    irq_set_enabled(USBCTRL_IRQ, true);
    fill_packet();
}

static void stream_stop(void)
{
    irq_set_enabled(USBCTRL_IRQ, false);
    usb_hw_clear->int_ep_ctrl = 1u << 1;
    usbh_dpram->int_ep_ctrl[0].ctrl = 0;
    usb_hw->inte = 0;
    usb_hw->sie_ctrl = SIE_BASE;
    usbh_dpram->epx_ctrl = 0;
    usbh_dpram->epx_buf_ctrl = 0;
}

/* attach to playing, in one go (a few tens of ms) */
static void enumerate(void)
{
    uint8_t b[18];
    s_mps0 = 8;
    s_err_step = 1;
    if (ctrl(0, 0x80, 6, 0x0100, 0, b, 8) < 8) goto fail;             /* device descriptor, first 8 */
    s_mps0 = b[7] ? b[7] : 8;
    s_err_step = 2;
    if (ctrl(0, 0x00, 5, ADDR, 0, 0, 0) < 0) goto fail;               /* SET_ADDRESS */
    busy_wait_us_32(2000);                                             /* recovery: 2 ms */
    s_err_step = 3;
    if (ctrl(ADDR, 0x80, 6, 0x0100, 0, b, 18) < 18) goto fail;
    s_vid = (uint16_t)(b[8] | b[9] << 8); s_pid = (uint16_t)(b[10] | b[11] << 8);
    s_err_step = 4;
    if (ctrl(ADDR, 0x80, 6, 0x0200, 0, s_cfg, 9) < 9) goto fail;      /* configuration, its length */
    int total = s_cfg[2] | s_cfg[3] << 8;
    if (total > (int)sizeof s_cfg) total = sizeof s_cfg;
    s_err_step = 5;
    if (ctrl(ADDR, 0x80, 6, 0x0200, 0, s_cfg, (uint16_t)total) < total) goto fail;
    s_err_step = 6;
    if (ctrl(ADDR, 0x00, 9, s_cfg[5], 0, 0, 0) < 0) goto fail;        /* SET_CONFIGURATION */
    if (!parse(s_cfg, total)) { s_state = ST_CANT; return; }
    s_err_step = 7;
    if (ctrl(ADDR, 0x01, 11, (uint16_t)s_as_alt, (uint16_t)s_as_if, 0, 0) < 0) goto fail;   /* SET_INTERFACE */
    s_err_step = 8;
    if (set_rate()) { s_state = ST_CANT; return; }
    /* the buttons: not having them is no reason not to play */
    if (s_hid_if >= 0 && s_hid_ep && s_hid_rlen) {
        int n = s_hid_rlen < (int)sizeof s_rdesc ? s_hid_rlen : (int)sizeof s_rdesc;
        n = ctrl(ADDR, 0x81, 6, 0x2200, (uint16_t)s_hid_if, s_rdesc, (uint16_t)n);   /* report descriptor */
        if (n > 0) parse_report(s_rdesc, n); else s_hid_ep = 0;
        ctrl(ADDR, 0x21, 0x0A, 0, (uint16_t)s_hid_if, 0, 0);       /* SET_IDLE 0: report on change */
    }
    s_err_step = 0;
    set_gain();
    stream_start();
    hid_start();
    s_state = ST_PLAY;
    return;
fail:
    s_state = ST_FAIL;
}

/* ════════════════════════════════════════ the interface */

/* THE GAME'S CORE WAITS FOR THE FRAME INSIDE AN SVC (uvm2_svc.c, SYS_WAIT_RECAL), and SVCall
 * is left at the highest priority there is: no interrupt of the default priority runs until it
 * returns, which is most of the frame. On the cart that let one packet in four through (263 a
 * second, P/PS on usbtone's screen, every one of them completing cleanly) and the EarPods
 * played chopped-up noise. So SVCall goes below the USB interrupt, on this core only.
 *
 * ONLY WITH CORE 1: there core 0's SVCs never touch the Vectrex's bus (the raw VIA calls are
 * compiled out, input comes from core 1's cache), so being interrupted cannot upset a bus
 * phase. On one core the frame's replay runs in that SVC and must not be interrupted: there
 * USB audio plays chopped, and that is the price, not a broken picture.
 * Nothing may execute `svc` from an interrupt of priority 0x40..0xB0 after this (it would be
 * a HardFault); no handler in the SDK does. */
#define SVC_PRIO 0xC0u
#define USB_PRIO 0x40u

void uvm2_usbaudio_start(uint32_t *ring, uint32_t words)
{
    if (s_started) return;
    if (!ring || words < 2u * PREFILL || (words & (words - 1u))) return;   /* no ring, no stream */
    s_ring = ring;
    s_ring_words = words;
    s_started = 1;
    usb_clock();
    host_up();
    if (uvm2_setting_usb_volume >= 0 && uvm2_setting_usb_volume <= VOL_STEPS) s_vol = (int)uvm2_setting_usb_volume;
    s_vol_saved = s_vol;
#ifdef UVM2_DUAL_CORE
    volatile uint32_t *shpr2 = (volatile uint32_t *)0xE000ED1Cu;      /* SCB SHPR2: SVCall in 31:24 */
    *shpr2 = (*shpr2 & 0x00FFFFFFu) | SVC_PRIO << 24;
    irq_set_priority(USBCTRL_IRQ, USB_PRIO);
#endif
    s_state = ST_WAIT;
}

void uvm2_usbaudio_poll(void)
{
    if (!s_started) return;
    const int sp = speed();
    /* the volume the buttons moved, saved from here (the game's core, between frames) and never
       from the interrupt: once it has stayed put a while, so a run of presses is one write */
    if (s_vol != s_vol_saved && time_us_32() - s_vol_t >= VOL_SAVE_US) {
        s_vol_saved = s_vol;
        uvm2_setting_usb_volume = s_vol;
        if (uvm2_config_save) uvm2_config_save();
    }
    if (time_us_32() - s_t_sec >= 1000000u) {          /* packets and empty packets a second */
        s_t_sec = time_us_32();
        s_pps = s_packets - s_last_p; s_last_p = s_packets;
        s_ups = s_under - s_last_u;   s_last_u = s_under;
    }
    switch (s_state) {
    case ST_WAIT:
        if (sp == 2) { s_state = ST_ENUM; s_t_attach = time_us_32(); }
        break;
    case ST_ENUM:
        if (!sp) { s_state = ST_WAIT; break; }
        if (time_us_32() - s_t_attach >= SETTLE_US) enumerate();
        break;
    default:                                    /* playing, or a device it cannot use */
        if (!sp) {
            if (s_state == ST_PLAY) stream_stop();
            host_up();                          /* a clean controller for the next one */
            s_state = ST_WAIT;
        }
        break;
    }
}

void uvm2_usbaudio_write_lr(const int16_t *l, const int16_t *r, int n)
{
    if (s_state != ST_PLAY) return;
    uint32_t w = s_w;
    if (w - s_r > RING - (uint32_t)n) return;   /* the reader stopped: nothing to keep */
    for (int i = 0; i < n; i++, w++)
        s_ring[w & (RING - 1u)] = (uint32_t)(uint16_t)l[i] << 16 | (uint16_t)r[i];
    s_w = w;
}

int uvm2_usbaudio_space(void)
{
    if (s_state != ST_PLAY) return 0;
    const uint32_t fill = s_w - s_r;
    return fill >= PREFILL ? 0 : (int)(PREFILL - fill);
}

int uvm2_usbaudio_playing(void) { return s_state == ST_PLAY; }
int uvm2_usbaudio_volume(void) { return s_mute ? -s_vol - 1 : s_vol; }
void uvm2_usbaudio_set_volume(int v) { s_vol = v < 0 ? 0 : v > VOL_STEPS ? VOL_STEPS : v; s_mute = 0; set_gain(); s_vol_saved = s_vol; }
uint32_t uvm2_usbaudio_buttons(void) { return s_buttons; }
int uvm2_usbaudio_state(void) { return s_state; }

static char *put(char *q, const char *s) { while (*s) *q++ = *s++; return q; }
static char *num(char *q, uint32_t v) { char t[10]; int n = 0; do t[n++] = (char)('0' + v % 10), v /= 10; while (v); while (n) *q++ = t[--n]; return q; }
static char *hex4(char *q, unsigned v) { for (int s = 12; s >= 0; s -= 4) { unsigned d = v >> s & 15; *q++ = (char)(d < 10 ? '0' + d : 'A' + d - 10); } return q; }

const char *uvm2_usbaudio_report(void)
{
    static char b[96];
    static const char *const name[] = { "OFF", "WAIT", "ENUM", "PLAY", "CANT", "FAIL" };
    char *q = put(b, "USB ");
    q = put(q, name[s_state]);
    if (s_state == ST_FAIL) { q = put(q, " AT "); q = num(q, (uint32_t)s_err_step); }
    if (s_state >= ST_PLAY) {
        *q++ = ' '; q = hex4(q, s_vid); *q++ = ':'; q = hex4(q, s_pid);
        q = put(q, " UAC"); q = num(q, (uint32_t)s_uac);
    }
    if (s_state == ST_PLAY) {
        *q++ = ' '; q = num(q, (uint32_t)s_ch); q = put(q, "X"); q = num(q, (uint32_t)s_bits);
        *q++ = ' '; q = num(q, s_rate);
        q = put(q, " P"); q = num(q, s_packets);
        q = put(q, " U"); q = num(q, s_under);
        q = put(q, " PS"); q = num(q, s_pps);         /* packets a second: 1000 */
        q = put(q, " US"); q = num(q, s_ups);         /* of them, empty: 0 */
        q = put(q, " VOL"); q = num(q, (uint32_t)s_vol); if (s_mute) q = put(q, "M");
        if (s_hid_ep) { q = put(q, " BTN"); q = num(q, s_buttons); q = put(q, " R"); q = num(q, s_hid_reports); }
        else q = put(q, " NO BTN");
    }
    *q = 0;
    return b;
}

#else
/* No USB audio: the host build, a debug build with the monitor on the port, or asked off. */
void uvm2_usbaudio_start(uint32_t *ring, uint32_t words) { (void)ring; (void)words; }
void uvm2_usbaudio_poll(void) { }
void uvm2_usbaudio_write_lr(const int16_t *l, const int16_t *r, int n) { (void)l; (void)r; (void)n; }
int  uvm2_usbaudio_space(void) { return 0; }
int  uvm2_usbaudio_playing(void) { return 0; }
int  uvm2_usbaudio_volume(void) { return 0; }
void uvm2_usbaudio_set_volume(int v) { (void)v; }
uint32_t uvm2_usbaudio_buttons(void) { return 0; }
int  uvm2_usbaudio_state(void) { return 0; }
const char *uvm2_usbaudio_report(void) { return "USB OFF"; }
#endif
