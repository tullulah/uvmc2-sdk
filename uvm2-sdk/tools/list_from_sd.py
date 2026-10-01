#!/usr/bin/env python3
"""list_from_sd.py — the command list uvm2_dump_list() wrote to the SD card, as JSON.

    python3 tools/list_from_sd.py /Volumes/<card>/list.bin list.json
    python3 tools/beam_sim.py list.json picture.svg

The file is a 32-byte header and the frame's commands, 3 bytes each (the format
is at the top of uvm2_dump.c). The output is what list_from_rtt.py writes and
beam_sim.py reads: a JSON array of [b0, b1, b2].

IT REFUSES, rather than guess, a file that is not one whole list: a wrong
magic or version, a length that does not match the command count, or a hash
that does not match the bytes. A file LONGER than the list is accepted: the debug
cartridge's BIOS cannot allocate, so it overwrites a pre-made file in place and
the bytes past the list are whatever was there — the count and the hash decide.
And it says out loud when the frame DROPPED
commands — then the list overflowed and is not evidence of anything.
"""
import json
import struct
import sys

MAGIC = b"UVML"
VERSION = 1
HEADER = 32


def fnv1a(data):
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def read(path):
    raw = open(path, "rb").read()
    if len(raw) < HEADER:
        sys.exit("list_from_sd: %s is %d bytes, shorter than the header" % (path, len(raw)))
    magic, ver, hsize, n, frame, buf, dropped, clamped, h = struct.unpack("<4sHHIIIIII", raw[:HEADER])
    if magic != MAGIC:
        sys.exit("list_from_sd: bad magic %r (not a uvm2_dump_list file)" % magic)
    if ver != VERSION or hsize != HEADER:
        sys.exit("list_from_sd: version %d, header %d — this reader knows %d/%d" % (ver, hsize, VERSION, HEADER))
    body = raw[HEADER:]
    if len(body) < n * 3:
        sys.exit("list_from_sd: header says %d commands (%d bytes), file has %d" % (n, n * 3, len(body)))
    body = body[:n * 3]                     # a pre-sized file (the BIOS's) has a tail
    if fnv1a(body) != h:
        sys.exit("list_from_sd: hash mismatch — the list is not the one the header describes")
    meta = {"commands": n, "frame": frame, "buffer": buf, "dropped": dropped, "ramps_clamped": clamped}
    return [list(body[3 * i:3 * i + 3]) for i in range(n)], meta


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    cmds, m = read(sys.argv[1])
    json.dump(cmds, open(sys.argv[2], "w"))
    print("frame %(frame)d, buffer %(buffer)d: %(commands)d commands, "
          "dropped %(dropped)d, ramps_clamped %(ramps_clamped)d" % m)
    if m["dropped"]:
        print("WARNING: the frame DROPPED %d commands — the list overflowed and is not evidence"
              % m["dropped"])
    print("-> %s" % sys.argv[2])


if __name__ == "__main__":
    main()
