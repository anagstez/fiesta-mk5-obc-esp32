#!/usr/bin/env python3
"""Voice-guided pedal test (macOS `say`): records the dump with #LABEL marker lines.

usage: guided.py OUT.log [port]
The schedule below is what produced dumps/guided2-*.log: 5 s holds, 8 s rests.
"""
import os, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obc_serial import open_board

STEPS = [(10, "brake now. press and hold", "BRAKE_ON"), (15, "release brake", "RELEASE"),
         (23, "brake now. press and hold", "BRAKE_ON"), (28, "release brake", "RELEASE"),
         (36, "clutch now. press and hold", "CLUTCH_ON"), (41, "release clutch", "RELEASE"),
         (49, "clutch now. press and hold", "CLUTCH_ON"), (54, "release clutch", "RELEASE"),
         (62, "brake and clutch together now", "BOTH_ON"), (67, "release both", "RELEASE"),
         (75, "gas now. raise revs", "GAS_ON"), (80, "release gas", "RELEASE")]
END = 90
out = open(sys.argv[1], "w", buffering=1)
s = open_board(sys.argv[2] if len(sys.argv) > 2 else None, timeout=0.2)
s.write(b"D\n")
t0, checked, k = time.time(), False, 0
subprocess.Popen(["say", "Recording starts. Get ready. Do nothing for ten seconds."])
while time.time() - t0 < END:
    t = time.time() - t0
    if k < len(STEPS) and t >= STEPS[k][0]:
        out.write(f"{t:8.3f} #LABEL {STEPS[k][2]}\n")
        subprocess.Popen(["say", STEPS[k][1]])
        k += 1
    line = s.readline().decode(errors="replace").rstrip()
    if not line:
        continue
    if not checked and "dump" in line:
        checked = True
        if "OFF" in line:
            s.write(b"D\n")
    out.write(f"{time.time() - t0:8.3f} {line}\n")
subprocess.Popen(["say", "Done."])
s.write(b"D\n")
