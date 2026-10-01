#!/usr/bin/env python3
"""list_from_rtt.py — the command list, put back together from the BIOS's RTT dump.

    python3 tools/list_from_rtt.py rtt.log list.json

The debug cartridge's BIOS, built with CMD_DUMP, prints the frame's command
list over RTT sixteen commands at a time while buttons 1 and 2 are held:

    CMDS buffer <b> total <n>
    CMD <first> [b0, b1, b2, b0, b1, b2, ...]      (3 bytes a command)

A frozen frame repeats, so the dump holds several complete passes. This joins
them, keeps the complete ones, and REFUSES if they disagree — a list that
changed under the dump is not one list, and simulating a mix of two would
prove nothing. The output is a JSON array of [b0, b1, b2] commands: what
beam_sim.py reads.

Written 2026-10-01, when exactly this — a dump and a replay of it — found the
zero clamp held over the frame's first strokes (uvm2_stats.ramps_clamped).
"""
import json
import re
import sys


def passes(lines):
    out, cur = [], None
    for line in lines:
        m = re.search(r"CMDS buffer (\d+) total (\d+)", line)
        if m:
            cur = {"total": int(m.group(2)), "cmds": {}}
            out.append(cur)
            continue
        m = re.search(r"CMD (\d+) \[([0-9, ]+)\]", line)
        if m and cur is not None:
            pos = int(m.group(1))
            b = [int(x) for x in m.group(2).split(",")]
            for k in range(len(b) // 3):
                cur["cmds"][pos + k] = b[3 * k:3 * k + 3]
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    ps = passes(open(sys.argv[1], errors="replace"))
    full = [p for p in ps if len(p["cmds"]) == p["total"]]
    if not full:
        sys.exit("list_from_rtt: no complete pass in %d (hold the buttons longer)" % len(ps))
    ref = [full[0]["cmds"][i] for i in range(full[0]["total"])]
    for p in full[1:]:
        if [p["cmds"][i] for i in range(p["total"])] != ref:
            sys.exit("list_from_rtt: the complete passes DISAGREE — the frame changed under the dump")
    json.dump(ref, open(sys.argv[2], "w"))
    print("%d passes, %d complete and identical, %d commands -> %s"
          % (len(ps), len(full), len(ref), sys.argv[2]))


if __name__ == "__main__":
    main()
