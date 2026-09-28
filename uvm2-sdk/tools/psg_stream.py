#!/usr/bin/env python3
"""psg_stream.py — frame images -> the compiled event stream the SDK plays.

THE FORMAT, which three sequencers read and none of them generates:

    MUSIC: [u32 num_events][u32 loop event byte offset][events at +8]
    SFX:   [u32 num_events][events at +4]
    event: [delay, num_writes, (reg,val)*num_writes]
           num_writes 0xFF -> loop (music only), 0x00 -> end
           delay = frames to wait BEFORE this event fires (the first fires at once)

The readers: uvm2_audio.c (the UVM2 multicart), vpy.c (PiTrex and the host), and
the Vectrex Studio cartridge firmware (music.rs, on core 1). Three
implementations of the PLAYER is unavoidable — three platforms — but there is
exactly one WRITER, and this is it.

IT LIVES HERE AND NOT IN A GAME'S tools/ because the second game needed it. It
was written for snowbros_sbt (YM3812 register logs off the arcade sound ROM) and
kuroishi asks the same question of a completely different input (.vmus notes it
composes itself). What the two share is not the music, it is the encoding — and
the encoding is where the expensive mistakes were made. The comments below are
the record of them; read them before changing a line.

The input is `images`: one entry per 50 Hz frame, each an indexable of 11 PSG
register values (0..10). What produces them is the caller's business.
"""

# Which PSG registers each kind of stream is allowed to write. The split is the
# runtime contract, and it has to be enforced on the WRITES, not just on the
# mixer bits: a music stream that merely sets C's period and volume to zero —
# which the first event does for every register it has never written — silences
# an effect that happens to be playing when the track starts.
# Register 6 (noise period) is unavoidably shared: the PSG has ONE noise
# generator. An effect using noise retunes the music's drums for its duration.
MUSIC_REGS = (0, 1, 2, 3, 6, 7, 8, 9)      # voices A and B, noise, mixer
SFX_REGS   = (4, 5, 6, 7, 10)              # voice C, noise, mixer


def encode(images, loop_frame, is_music):
    allowed = MUSIC_REGS if is_music else SFX_REGS
    """Frame images -> the SDK's compiled event stream bytes.

    AN EVENT'S DELAY BYTE IS THE WAIT *BEFORE* IT FIRES, not after. That is
    what the sequencers implement (uvm2_audio.c music_tick): firing an event
    advances the pointer past its writes and reads the delay from THERE — i.e.
    from the next event — so a delay byte is consumed on approach to its own
    event. The loop marker agrees (`s_mus_delay = s_mus_ptr[0]` after the
    jump). Writing the gap as the current event's delay instead shifts the
    whole timeline by one event, which is a real one-frame-per-note drift.
    Event 0 fires immediately, so its own delay byte is never read.
    """
    # Diffs are taken against the last value actually EMITTED, never against
    # the previous frame's image. With a per-frame comparison a register that
    # creeps by one step per frame (which is exactly what an FM envelope decay
    # looks like after conversion) would trip the deadband every single frame
    # and never be written at all — the note would keep its attack volume for
    # its whole length.
    events = []                                # (absolute frame, [(reg,val)...])
    sent = [None] * 11
    loop_event_idx = None
    for f, img in enumerate(images):
        writes = []
        for r in allowed:
            if img[r] == sent[r]:
                continue
            # Volume: a 1-step change is inaudible on a 4-bit log-ish PSG
            # volume, so it is not worth an event — but reaching or leaving
            # silence always is, or notes never start and never stop.
            if 8 <= r <= 10 and sent[r] is not None and img[r] and sent[r] \
                    and abs(img[r] - sent[r]) < 2:
                continue
            writes.append((r, img[r]))
        if loop_frame is not None and f == loop_frame:
            if not writes:                     # the loop must land on a REAL event
                writes = [(7, img[7])]
            loop_event_idx = len(events)
        if writes:
            events.append((f, writes))
            for r, v in writes:
                sent[r] = v

    # (delay_byte, writes) pairs, long gaps split with harmless filler events
    # (rewriting the mixer with the value it already has costs 2 bus writes).
    #
    # A DELAY BYTE OF N PRODUCES A GAP OF N+1 FRAMES, so the byte is gap-1.
    # Read the sequencer: firing an event sets delay = N and returns; the next
    # N ticks each decrement it and return; the tick after that fires. Encoding
    # the gap itself therefore stretches EVERY gap by one frame — which on the
    # level theme, whose average gap is 2.3 frames, played the whole track ~40%
    # slow (measured: 300 ms between note onsets where the arcade has 200).
    # Consecutive frames are gap 1 -> delay 0, which is the "fires immediately"
    # case, so the arithmetic is consistent at the bottom end too.
    timed = []
    loop_out_idx = loop_event_idx
    mixer = 0x3F
    last_f = events[0][0] if events else 0
    for i, (f, writes) in enumerate(events):
        gap = f - last_f
        last_f = f
        # Each emitted event accounts for (its delay byte + 1) frames, so a
        # filler with byte 253 covers 254 of them.
        while gap > 254:
            timed.append((253, [(7, mixer)]))
            if loop_out_idx is not None and i <= loop_out_idx:
                loop_out_idx += 1
            gap -= 254
        if loop_event_idx is not None and i == loop_event_idx:
            loop_out_idx = len(timed)
        timed.append((max(0, gap - 1), writes))
        for r, v in writes:
            if r == 7:
                mixer = v
    # terminator: its delay byte holds the last event's duration
    tail = max(0, min(254, len(images) - last_f))
    timed.append((max(0, tail - 1), None))     # None -> loop/end marker

    body = bytearray()
    offsets = []
    for delay, writes in timed:
        offsets.append(len(body))
        body.append(delay)
        if writes is None:
            body.append(0xFF if is_music else 0x00)
        else:
            body.append(len(writes))
            for r, v in writes:
                body.append(r)
                body.append(v)

    out = bytearray()
    out += len(timed).to_bytes(4, 'little')
    if is_music:
        out += (8 + offsets[loop_out_idx or 0]).to_bytes(4, 'little')
    out += body
    return bytes(out)
