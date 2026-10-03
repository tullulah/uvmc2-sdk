/*
 * uvm2_input.c — buttons and joysticks over the halted bus.
 *
 * Input cannot be recorded into the command stream: it needs the data bus
 * turned around mid-cycle, so these run as direct accesses.  They must run
 * BETWEEN frames, while /ZERO holds the beam clamped at centre — the sequences
 * below drive Port B for the PSG and the mux, which disturbs the ramp state.
 * So they run right after replaying a frame.
 *
 * The byte sequences are kept exactly as they were proven on hardware.
 * Deliberately literal: the Vectrex documentation
 * disagrees with itself about the polarity of Port B bit 0 (the VIA register
 * map calls 0 "enable mux", the BIOS Joy_Digital listing treats 1 as enable),
 * and these sequences are the version that demonstrably works.  Do not
 * "clean these up" without a Vectrex in front of you.
 */

#include "uvm2_bus.h"
#include "uvm2_input.h"

/* Port B control bytes for the AY-3-8910 handshake (BC1 = bit 3, BDIR = bit 4).
 *
 * BIT 7 GOES INSIDE THE CONSTANT, not at the point of use. PB7 is /RAMP, and these bytes
 * go out on Port B while Port A -- which IS the beam's DAC -- carries first the register
 * number and then the value. With bit 7 at zero the integrators run free with that garbage
 * on the DAC: a bright segment from the origin in an arbitrary direction, on EVERY PSG
 * access. And there is one per sound frame and another per controller read.
 *
 * The axis readers already protected themselves by writing UVM2_PB_RAMP_OFF | ... on every
 * line; uvm2_psg_write, uvm2_psg_read and uvm2_read_buttons were left out. Putting it in
 * the definition means no call site can forget it.
 *
 * The PSG does not notice: it only looks at BC1 (bit 3) and BDIR (bit 4). Bit 0 = 1 in all
 * three also leaves the analog mux disabled, which is what they already did.
 *
 * That this window was the culprit was already MEASURED, in this same file below: "with the
 * input read removed entirely the stray bright vectors dropped from 4-5 to 1" (hardware,
 * 2026-08-04). */
#define PSG_LATCH_ADDR  (UVM2_PB_RAMP_OFF | 0x19u)  /* BDIR | BC1 → latch reg nº */
#define PSG_READ        (UVM2_PB_RAMP_OFF | 0x09u)  /* BC1  → reg on the bus     */
#define PSG_INACTIVE    (UVM2_PB_RAMP_OFF | 0x01u)
#define PSG_WRITE       (UVM2_PB_RAMP_OFF | 0x11u)  /* BDIR → write to the PSG   */

/* PSG register 14 carries both joystick button ports: bits 0-3 = J1,
 * bits 4-7 = J2, active low. */
#define PSG_REG_BUTTONS 0x0Eu

static int s_analog = 0;

void uvm2_input_set_analog(int enable) { s_analog = enable; }

uint8_t uvm2_read_buttons(void)
{
    uint8_t raw;

    /* TO WRITE THE REGISTER NUMBER, PORT A HAS TO BE AN OUTPUT.
     *
     * This used to be taken for granted — "whoever ran before will have left it like that"
     * — which is an assumption about ordering, not a guarantee. If the port is still an
     * input, the 0x0E never reaches the bus, the PSG does NOT latch register 14 and keeps
     * the last one it was given; reading it returns THAT. It fits what was measured under
     * dual core: a constant 0x3F, which is exactly the contents of register 7, the mixer
     * the audio writes.
     *
     * On a single core it worked by accident of ordering. A read cannot depend on who ran
     * before it. */
    uvm2_via_write(UVM2_VIA_DDRA, 0xFF);

    uvm2_via_write(UVM2_VIA_PORTA, PSG_REG_BUTTONS);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_LATCH_ADDR);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);

    uvm2_via_write(UVM2_VIA_DDRA,  0x00);           /* Port A → input */
    uvm2_via_write(UVM2_VIA_PORTB, PSG_READ);
    raw = uvm2_via_read(UVM2_VIA_PORTA);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);
    uvm2_via_write(UVM2_VIA_DDRA,  0xFF);           /* Port A → output (the DAC) */

    /* Leave Port B where the drawing code expects it. */
    uvm2_via_write(UVM2_VIA_PORTB, UVM2_PB_IDLE);

    /* Returned RAW, exactly as the chip presents it: active-low, J1 in bits
     * 0-3 and J2 in bits 4-7.  The two button syscalls want different shapes
     * of this (see uvm2_svc.c), so inverting here would only mean undoing it. */
    return raw;
}

/* One axis, successive approximation — the BIOS Joy_Analog algorithm: walk the
 * DAC bit by bit, keeping each bit the comparator agrees with.  Costs about
 * seven extra read cycles per axis over the digital path.
 *
 * VALIDATED on the console 2026-10-02 (a debug cartridge, values read over SWD): right
 * +127, left -128, up +127, down -128; at rest X 4..12 and Y 31..48, not 0. It is now the
 * only axis read — see uvm2_read_axes for why the digital one went. */
static int read_axis_analog(int channel)
{
    const uint32_t sel     = (uint32_t)(channel << 1);
    const uint32_t inhibit = UVM2_PB_RAMP_OFF | 0x01u | sel;  /* PB0=1 -> mux OFF */
    const uint32_t enable  = UVM2_PB_RAMP_OFF | 0x00u | sel;  /* PB0=0 -> mux ON  */
    uint8_t pa = 0x00;      /* the D/A value, and the answer */
    uint8_t b  = 0x80;      /* bit under test — STARTS AT THE SIGN BIT */

    uvm2_via_write(UVM2_VIA_DDRA, 0xFF);   /* the DAC goes out on port A */

    /* Select while inhibited, enable to let the pot charge C307, then inhibit
     * again and convert off the held charge. That order is the BIOS's and it
     * matters: converting with the mux connected re-charges the cap from the
     * pot mid-conversion. */
    uvm2_via_write(UVM2_VIA_PORTB, inhibit);
    uvm2_via_write(UVM2_VIA_PORTA, 0x00);
    uvm2_via_write(UVM2_VIA_PORTB, enable);
    uvm2_bus_delay(32u * 5u);
    uvm2_via_write(UVM2_VIA_PORTB, inhibit);

    /* Successive approximation, transcribed from the cart's joy_analog (which is
     * itself Joy_Analog at $F1F5). The previous version here started at 0x40 and
     * never touched bit 7, so it could only ever return 0..0x7F: a centred stick
     * read ~64, every game saw "hard right", and Asteroids would only rotate one
     * way. The sign bit is the FIRST thing the comparator decides. */
    for (;;) {
        uvm2_bus_delay(10u);
        if ((uvm2_via_read(UVM2_VIA_PORTB) & 0x20u) == 0) {
            pa ^= b;                                  /* this bit overshot */
            uvm2_via_write(UVM2_VIA_PORTA, pa);
        }
        b >>= 1;
        if (b == 0) break;
        pa |= b;
        uvm2_via_write(UVM2_VIA_PORTA, pa);
    }

    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);
    return (int8_t)pa;
}

/* THE AXES COME FROM THE ANALOG READ ALONE, and the digital answer is made from it.
 *
 * The digital read probed the comparator with the DAC at +64 or -64, whichever way the stick
 * leant, and that probe left a DOT on the tube: with the brightness up, a spot on the
 * diagonal that followed the stick (south-west at rest or up, north-east down) in every game
 * and in the BIOS menu, and Minestorm had none. Bisected on the console 2026-10-02 over SWD:
 * the dot went with the axis read switched off, went with the read kept but the probe
 * dropped, and did not come back with the analog read — the BIOS's own Joy_Analog. Zeroing
 * the DAC after the probe did NOT cure it, so it is the probe itself, not what it leaves.
 *
 * THE REST IS NOT ZERO. Measured on the same console: X 4..12, Y 31..48. The first reading
 * is taken as the stick's centre — unless it is past REST_MAX, which is a stick already
 * pushed at power-on, and then 0 is the centre — and every reading is taken from it.
 *   digital (the default): -127 / 0 / +127, past DIGITAL_AT either way, as before — and a
 *            rest of 40 in Y no longer reads as "up" to `if (J1_Y() > 32)`
 *   analog (uvm2_input_set_analog(1)): the centred value, 0 inside DEADZONE — the rest
 *            wanders by ~8 either side, measured */
#define REST_MAX    64   /* half the travel: further out at power-on is a pushed stick */
#define DIGITAL_AT  64   /* the old digital read's own threshold: its probe sat at +-64 */
#define DEADZONE    16   /* twice the rest's measured wander (~8) */
static int     s_centred;
static int     s_cx, s_cy;
static int clamp8(int v) { return v > 127 ? 127 : (v < -128 ? -128 : v); }
static int shape(int v)
{
    if (!s_analog) return v > DIGITAL_AT ? 127 : (v < -DIGITAL_AT ? -127 : 0);
    if (v > -DEADZONE && v < DEADZONE) return 0;
    return clamp8(v);
}

uint32_t uvm2_read_axes(void)
{
    int jx = read_axis_analog(0);
    int jy = read_axis_analog(1);
    int j2x = 0, j2y = 0;

    if (!s_centred) {
        s_cx = (jx > -REST_MAX && jx < REST_MAX) ? jx : 0;
        s_cy = (jy > -REST_MAX && jy < REST_MAX) ? jy : 0;
        s_centred = 1;
    }
    jx = shape(clamp8(jx - s_cx));
    jy = shape(clamp8(jy - s_cy));

    uvm2_via_write(UVM2_VIA_PORTB, UVM2_PB_IDLE);

    return ((uint32_t)(uint8_t)(int8_t)jx  << 24)
         | ((uint32_t)(uint8_t)(int8_t)jy  << 16)
         | ((uint32_t)(uint8_t)(int8_t)j2x <<  8)
         |  (uint32_t)(uint8_t)(int8_t)j2y;
}

/* PSG register write, through the VIA's AY handshake. */
void uvm2_psg_write(uint32_t reg, uint32_t value)
{
    /* PORT A HAS TO BE AN OUTPUT: both the register number AND the value go out on it. If
     * it is not, neither arrives, and the symptom is bewildering — the sequencer fires at
     * the right instant, so the timing matches the real music, but the wrong registers and
     * values are heard. Observed on the console under dual core, 2026-08-12.
     *
     * It is the SAME bug uvm2_read_buttons and uvm2_psg_read had, and they were fixed
     * without looking at this one. All three took the port direction for granted, i.e. bet
     * that whoever ran before left it right. */
    uvm2_via_write(UVM2_VIA_DDRA, 0xFF);
    uvm2_via_write(UVM2_VIA_PORTA, reg & 0x0Fu);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_LATCH_ADDR);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);

    uvm2_via_write(UVM2_VIA_PORTA, value & 0xFFu);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_WRITE);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);
    uvm2_via_write(UVM2_VIA_PORTB, UVM2_PB_IDLE);
}

uint8_t uvm2_psg_read(uint32_t reg)
{
    uint8_t v;

    /* Same as uvm2_read_buttons: to put the register number on the bus, port A has to be
     * an OUTPUT. Taking it for granted is betting that whoever ran before left it so. */
    uvm2_via_write(UVM2_VIA_DDRA, 0xFF);
    uvm2_via_write(UVM2_VIA_PORTA, reg & 0x0Fu);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_LATCH_ADDR);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);

    uvm2_via_write(UVM2_VIA_DDRA,  0x00);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_READ);
    v = uvm2_via_read(UVM2_VIA_PORTA);
    uvm2_via_write(UVM2_VIA_PORTB, PSG_INACTIVE);
    uvm2_via_write(UVM2_VIA_DDRA,  0xFF);
    uvm2_via_write(UVM2_VIA_PORTB, UVM2_PB_IDLE);
    return v;
}
