#!/usr/bin/env python3
"""beam_sim.py — what a command list does to the beam, cycle by cycle.

    python3 tools/beam_sim.py list.json [picture.svg]

Reads a command list (JSON [b0, b1, b2] per command, as list_from_rtt.py
writes or uvm2_draw.c packs: data, reg | delay<<4, delay>>4), plays it against
a simple model of the Vectrex's beam, and reports what the list asks for that
cannot be right. Optionally draws what is lit.

THE MODEL, and its limits. It is the SR dialect uvm2_draw.c speaks:
  - ACR 0x98: T1 one-shot drives /RAMP on PB7 — writing T1CH starts a ramp
    that runs for the T1 count, one count per bus cycle;
  - the shift register's last bit is the beam: SR=1 lit, SR=0 blanked;
  - PCR's CA2 is /ZERO: 0xCC clamps both integrators to the centre, 0xCE frees;
  - the mux puts Port A into a sample-and-hold: channel 0 Y, 1 the zero
    reference, 2 brightness;
  - while a ramp runs, X integrates Port A and Y the Y hold, both against the
    reference.
It is IDEAL: no leakage, no settling, no analog anything. So what it finds is
what the LIST gets wrong — a beam that cannot be where it is drawn — not what
the console's electronics do to a list that is right.

WHAT IT REPORTS
  ramps with the clamp on   must be 0: the integrators are held at the centre,
                            so a lit ramp draws a line out of (0,0) instead of
                            its stroke (the 2026-10-01 asterisk)
  lit with the clamp on     the same fault, counted in cycles
  cycles                    the frame's length; 30000 is 50 Hz
"""
import json
import sys

REG = {0: "ORB", 1: "ORA", 2: "DDRB", 3: "DDRA", 4: "T1CL", 5: "T1CH", 6: "T1LL", 7: "T1LH",
       8: "T2CL", 9: "T2CH", 10: "SR", 11: "ACR", 12: "PCR", 13: "IFR", 14: "IER", 15: "ORA-nh"}


def s8(v):
    return v - 256 if v > 127 else v


def decode(c):
    return c[1] & 15, c[0], (c[2] << 4) | (c[1] >> 4)


def simulate(cmds):
    x = y = 0.0
    porta = 0
    portb = 0x81
    acr = 0
    t1l = 0
    ramp = 0
    yh = zr = 0
    lit = False
    zero = True
    cycles = 0
    clamped_ramps = []
    lit_clamped = 0
    segs = []
    for i, c in enumerate(cmds):
        reg, data, delay = decode(c)
        if reg in (1, 15):
            porta = data
        elif reg == 0:
            portb = data
        elif reg == 4:
            t1l = data
        elif reg == 5:
            ramp = (data << 8) | t1l if acr & 0x80 else 0
            if zero and ramp:
                clamped_ramps.append(i)
        elif reg == 11:
            acr = data
        elif reg == 12:
            zero = (data & 0x0E) == 0x0C
        elif reg == 10:
            lit = (data & 1) == 1
        if not (portb & 1):
            ch = (portb >> 1) & 3
            if ch == 0:
                yh = porta
            elif ch == 1:
                zr = porta
        x0, y0 = x, y
        for _ in range(1 + delay):
            if zero:
                x = y = 0.0
            elif ramp:
                x += s8(porta) - s8(zr)
                y += s8(yh) - s8(zr)
            if ramp:
                ramp -= 1
            if lit and zero:
                lit_clamped += 1
            cycles += 1
        if lit and (x0, y0) != (x, y):
            segs.append((x0, y0, x, y))
    return cycles, clamped_ramps, lit_clamped, segs


def svg(segs, path):
    if not segs:
        return
    xs = [v for s in segs for v in (s[0], s[2])]
    ys = [v for s in segs for v in (s[1], s[3])]
    m = max(1.0, max(abs(v) for v in xs + ys)) * 1.05
    out = ["<svg xmlns='http://www.w3.org/2000/svg' viewBox='%g %g %g %g' width='700' height='700' "
           "style='background:#000'><g transform='scale(1,-1)'>" % (-m, -m, 2 * m, 2 * m)]
    w = m / 300
    for a, b, c, d in segs:
        out.append("<line x1='%g' y1='%g' x2='%g' y2='%g' stroke='#8f8' stroke-width='%g'/>" % (a, b, c, d, w))
    out.append("</g></svg>")
    open(path, "w").write("".join(out))


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    cmds = json.load(open(sys.argv[1]))
    cycles, clamped, lit_clamped, segs = simulate(cmds)
    print("commands              %d" % len(cmds))
    print("cycles                %d%s" % (cycles, "  (a 50 Hz frame is 30000)" if cycles != 30000 else ""))
    print("lit moves             %d" % len(segs))
    print("ramps, clamp on       %d%s" % (len(clamped), "  <- MUST BE 0" if clamped else ""))
    print("lit cycles, clamp on  %d" % lit_clamped)
    if clamped:
        print("  first at command %d; the clamp is released (PCR=CE) at command %s"
              % (clamped[0], next((i for i, c in enumerate(cmds) if decode(c)[0] == 12 and (c[0] & 0x0E) == 0x0E), "never")))
    if len(sys.argv) == 3:
        svg(segs, sys.argv[2])
        print("picture -> %s" % sys.argv[2])
    sys.exit(1 if clamped else 0)


if __name__ == "__main__":
    main()
