#!/usr/bin/env python3
"""
hex_to_capture.py - turn the payload's "[TX] PACKET bytes: .." serial lines
(PRINT_TX_HEX=1 in Stack_with_LoRa) into an adapter capture file for
tools/gs_decode_sim.cpp --capture, emulating the ground-station adapter's
persistent 128-byte buffer (a packet overwrites only its own length; the
rest keeps the previous bytes).  Lets the whole encode -> pack -> adapter ->
Nidhoggr chain be checked from a serial log, without a radio link.

usage:  python3 tools/hex_to_capture.py serial_log.txt capture.bin
"""
import re, sys

src, out = sys.argv[1], sys.argv[2]
buf = bytearray(b"\xA5" * 128)          # adapter buffer before the first packet
n = 0
with open(src, "r", errors="replace") as f, open(out, "wb") as o:
    for line in f:
        m = re.search(r"\[TX\] PACKET bytes:((?: [0-9A-Fa-f]{2})+)", line)
        if not m:
            continue
        pkt = bytes.fromhex(m.group(1).replace(" ", ""))
        buf[:min(len(pkt), 128)] = pkt[:128]
        o.write(bytes([min(len(pkt), 255)]) + buf)   # [len][128-byte adapter buffer]
        n += 1
print(f"{n} packets -> {out}")
