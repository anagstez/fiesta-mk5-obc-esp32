#!/usr/bin/env python3
"""Screenshot of the display: 'F' over USB makes the firmware print the next frame as hex.

usage: screenshot.py OUT.png [port]   (stdlib + pyserial; writes a PNG without PIL)
"""
import os, struct, sys, time, zlib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obc_serial import open_board

W, H = 284, 76
s = open_board(sys.argv[2] if len(sys.argv) > 2 else None)
for _ in range(4):                               # the USB FIFO sometimes drops a few bytes: retry
    s.reset_input_buffer()
    s.write(b"F\n")
    rows, started, t = [], False, time.time()
    while time.time() - t < 20:
        line = s.readline().decode(errors="replace").strip()
        if line.startswith("FBBEGIN"):
            started, rows = True, []
        elif line == "FBEND" and started:
            break
        elif started:
            rows.append(line)
    raw = bytes.fromhex("".join(rows))
    if len(raw) == W * H * 2:
        break
else:
    sys.exit("could not get a complete frame")

img = bytearray()
for y in range(H):
    img.append(0)
    for x in range(W):
        v = raw[(y * W + x) * 2] << 8 | raw[(y * W + x) * 2 + 1]            # big-endian RGB565
        img += bytes([(v >> 11 & 31) * 255 // 31, (v >> 5 & 63) * 255 // 63, (v & 31) * 255 // 31])


def chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))


open(sys.argv[1], "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)) +
                              chunk(b"IDAT", zlib.compress(bytes(img))) + chunk(b"IEND", b""))
