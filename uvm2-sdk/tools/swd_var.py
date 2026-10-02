#!/usr/bin/env python3
"""swd_var.py — read or write a global by NAME on a running console, without stopping it.

    python3 tools/swd_var.py <game.elf> <symbol>             read it
    python3 tools/swd_var.py <game.elf> <symbol> <value>     write it (1-, 2- or 4-byte globals)
    python3 tools/swd_var.py <game.elf> <symbol> ... --no-verify

WHAT IT IS FOR. Every runtime knob in the SDK is a `volatile` global so that it can be
changed while a game runs and the tube watched: `uvm2_pacer_cycles`, `uvm2_zero_offset`,
`uvm2_filler_clamp`... Writing one live is an A/B comparison with nothing else changed —
no rebuild, no reflash, the same brightness and the same console. That is how, on
2026-10-02, a dark diagonal was traced to the frame filler (`uvm2_filler_clamp` 0 against
1) and a stray dot to the joystick read (switching parts of it off one at a time).

THE SAME RULES AS stats.py, and for the same reasons: the address comes from the ELF every
run (an address from another build reads another variable and still looks like a number),
the ELF must be the running image (checked against the target's RAM unless --no-verify),
and probe-rs read/write go through the AHB-AP and do not halt the core. Values are
little-endian; what is written is read back and printed, so a write that did not land is
visible.

A WRITE IS NOT PERSISTENT: the next power cycle starts from the compiled-in value. Make a
setting that won the comparison the default in the source, with the comparison in its
comment.
"""
import pathlib
import re
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import stats  # noqa: E402  (the shared probe-rs and ELF helpers)


def fail(msg):
    sys.exit("swd_var.py: " + msg)


def sized_symbols(elf):
    """{name: (address, size in bytes)} for data symbols the ELF gives a size to."""
    for nm in ("arm-none-eabi-nm", "llvm-nm", "nm"):
        try:
            txt = subprocess.run([nm, "-S", elf], check=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL).stdout.decode()
            break
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    else:
        fail(f"cannot read symbols from {elf}: no working nm")
    out = {}
    for line in txt.splitlines():
        m = re.match(r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([bBdDrRgGsStT])\s+(\S+)$", line.strip())
        if not m:
            continue
        size = int(m.group(2), 16)
        # A TEXT SYMBOL IS TAKEN ONLY IF IT IS THE SIZE OF A VARIABLE. The debug cartridge's
        # BIOS links its initialised data into the RAM-code section, so `uvm2_zero_offset`
        # shows up there as 'T'; a function is never 4 bytes or less, and writing over one
        # is the one thing this must not do.
        if m.group(3) in "tT" and size > 4:
            continue
        out[m.group(4)] = (int(m.group(1), 16), size)
    return out


def probe(args):
    out = subprocess.run(["probe-rs"] + args + ["--chip", stats.CHIP, "--speed", stats.SPEED],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if out.returncode:
        fail("probe-rs " + args[0] + " failed:\n" + out.stderr.decode().strip() +
             "\n(is the console on, the cartridge seated and the probe plugged in?)")
    return out.stdout.decode()


WIDTH = {1: "b8", 2: "b16", 4: "b32"}


def read(addr, size):
    if size in WIDTH:
        txt = probe(["read", WIDTH[size], hex(addr), "1"])
        return [int(t, 16) for t in txt.split()][:1], size
    words = (size + 3) // 4
    if words > 64:
        fail(f"{size} bytes is a large block; read it in parts with probe-rs (large reads have "
             "coincided with console freezes on another RP2350 board)")
    txt = probe(["read", "b32", hex(addr), str(words)])
    return [int(t, 16) for t in txt.split()][:words], 4


def show(name, addr, size, vals, width):
    bits = width * 8
    for i, v in enumerate(vals):
        signed = v - (1 << bits) if v >> (bits - 1) else v
        label = name if len(vals) == 1 else f"{name}[word {i}]"
        print(f"{label} @0x{addr + i * width:08x} = 0x{v:0{width * 2}x}  ({v}, signed {signed})")


def main():
    args = [a for a in sys.argv[1:] if a != "--no-verify"]
    if len(args) not in (2, 3):
        sys.exit(__doc__)
    elf, name = args[0], args[1]
    syms = sized_symbols(elf)
    if name not in syms:
        close = [s for s in syms if name in s][:8]
        fail(f"{name} is not a data symbol of {elf}" + (f"; did you mean: {', '.join(close)}" if close else ""))
    addr, size = syms[name]
    if "--no-verify" not in sys.argv:
        stats.verify(elf, stats.symbols(elf))
    if len(args) == 2:
        vals, width = read(addr, size)
        show(name, addr, size, vals, width)
        return
    if size not in WIDTH:
        fail(f"{name} is {size} bytes; only 1-, 2- and 4-byte globals are written")
    value = int(args[2], 0) & ((1 << (8 * size)) - 1)
    before, _ = read(addr, size)
    probe(["write", WIDTH[size], hex(addr), hex(value)])
    after, _ = read(addr, size)
    print(f"{name} @0x{addr:08x}: 0x{before[0]:0{size * 2}x} -> 0x{after[0]:0{size * 2}x}")
    if after[0] != value:
        fail(f"wrote 0x{value:x} and read back 0x{after[0]:x}: the write did not land")


if __name__ == "__main__":
    main()
