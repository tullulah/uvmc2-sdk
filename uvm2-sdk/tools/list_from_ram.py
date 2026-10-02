#!/usr/bin/env python3
"""list_from_ram.py — the command list straight out of a running console's RAM, over SWD.

    python3 tools/list_from_ram.py <game.elf> list.json
    python3 tools/list_from_ram.py <game.elf> list.json --psram CAPACITY   (UVM2_CMDS_IN_PSRAM builds)
    python3 tools/list_from_ram.py <game.elf> list.json --no-verify

Writes the JSON beam_sim.py reads, as list_from_sd.py and list_from_rtt.py do — the third
way to get the list the console was given, and the one that needs neither a card nor a
build with a dump in it: only the probe. Used on 2026-10-02 to read the debug cartridge's
menu list and show that nothing in it put the beam where a stray dot was.

WHICH BUFFER. There are two; frame k is built into buffer k & 1. `uvm2_frame_done` counts
the frames core 1 has replayed, so buffer (done & 1) holds the last one replayed — and the
builder does not reuse it until the frame after the one it is building now. That is a
window of a frame or two, and a read over SWD takes longer, so:

THE LIST IS READ TWICE AND MUST COME OUT THE SAME. A static screen (a menu, a paused game,
a calibration pattern) gives the same list every frame and passes. A moving one does not —
the frame changed under the read — and this REFUSES rather than hand beam_sim a mix of two
frames, which would prove nothing. For a moving scene, have the game call uvm2_dump_list
(07-measuring.md) or freeze it.

THE COST. This is a large read — 3 bytes a command, a few KB to tens of KB — and every
probe-rs read competes with the drawing core for the bus; large block reads have coincided
with console freezes on another RP2350 board. Read once, when the screen is the one you
want, and not in a loop. With the list in PSRAM it is read through the UNCACHED alias, as
anything verified in PSRAM must be (CLAUDE.md, invariant 6).
"""
import json
import pathlib
import re
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import stats  # noqa: E402

PSRAM_UNCACHED = 0x15000000          # UVM2_PSRAM_CMDS_BASE's default


def fail(msg):
    sys.exit("list_from_ram.py: " + msg)


def read_bytes(addr, n):
    words = (n + 3) // 4
    out = subprocess.run(["probe-rs", "read", "--chip", stats.CHIP, "--speed", stats.SPEED,
                          "b32", hex(addr), str(words)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if out.returncode:
        fail("probe-rs read failed:\n" + out.stderr.decode().strip())
    vals = [int(t, 16) for t in out.stdout.decode().split() if re.fullmatch(r"[0-9a-fA-F]{8}", t)]
    if len(vals) != words:
        fail(f"read 0x{addr:08x}: expected {words} words, got {len(vals)}")
    b = b"".join(v.to_bytes(4, "little") for v in vals)
    return b[:n]


def sym_size(elf, name):
    txt = subprocess.run(["arm-none-eabi-nm", "-S", elf], stdout=subprocess.PIPE).stdout.decode()
    for line in txt.splitlines():
        p = line.split()
        if len(p) == 4 and p[3] == name:
            return int(p[1], 16)
    return None


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        sys.exit(__doc__)
    elf, out = args[0], args[1]
    psram = None
    if "--psram" in args:
        psram = int(args[args.index("--psram") + 1], 0)
    syms = stats.symbols(elf)
    for need in ("s_len", "uvm2_frame_done"):
        if need not in syms:
            fail(f"{need} is not in {elf}: this reads the dual-core list (the baseline; CLAUDE.md "
                 "invariant 1)")
    if "--no-verify" not in args:
        stats.verify(elf, syms)

    if psram:
        base, capacity = PSRAM_UNCACHED, psram
    else:
        if "s_cmds" not in syms:
            fail("s_cmds is not in the ELF: a UVM2_CMDS_IN_PSRAM build? give --psram CAPACITY "
                 "(UVM2_CMD_CAPACITY, in commands)")
        base, size = syms["s_cmds"], sym_size(elf, "s_cmds")
        if not size:
            fail("cannot size s_cmds from the ELF")
        capacity = size // 2 // 3

    done = stats.read_words(syms["uvm2_frame_done"], 1)[0]
    which = done & 1
    lists = []
    for attempt in range(2):
        n = stats.read_words(syms["s_len"] + 4 * which, 1)[0]
        if n == 0 or n > capacity:
            fail(f"buffer {which} says {n} commands (capacity {capacity}): nothing to read yet, "
                 "or the ELF is not this image")
        lists.append(read_bytes(base + which * capacity * 3, n * 3))
    if lists[0] != lists[1]:
        fail("the two reads DISAGREE: the frame changed under the read. Freeze the screen, or "
             "dump it from the game with uvm2_dump_list.")
    b = lists[0]
    cmds = [list(b[i:i + 3]) for i in range(0, len(b), 3)]
    json.dump(cmds, open(out, "w"))
    print(f"frame {done}, buffer {which}: {len(cmds)} commands, read twice and identical -> {out}")


if __name__ == "__main__":
    main()
