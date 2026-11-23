#!/usr/bin/env python3
"""Record the firmware's CAN dump ('D' over USB) to a timestamped log.

usage: rec.py OUT.log [seconds] [port]      (needs pyserial; port default $OBC_PORT or the first /dev/cu.usbmodem*)

Each line is "<seconds since start> <bus> <id> [dlc] <bytes>"; bus M = MS-CAN, H = HS-CAN.
The firmware prints a frame only when its data changed, so this is a change log, not a rate log.
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obc_serial import open_board

out = open(sys.argv[1], "w", buffering=1)
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
s = open_board(sys.argv[3] if len(sys.argv) > 3 else None)
s.write(b"D\n")                                  # toggles the dump
t0, checked = time.time(), False
while time.time() - t0 < dur:
    line = s.readline().decode(errors="replace").rstrip()
    if not line:
        continue
    if not checked and "dump" in line:
        checked = True
        if "OFF" in line:                        # it was already on, we just switched it off
            s.write(b"D\n")
    out.write(f"{time.time() - t0:8.3f} {line}\n")
s.write(b"D\n")                                  # leave the dump off
