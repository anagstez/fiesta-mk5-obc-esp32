#!/usr/bin/env python3
"""Voice-guided lights test (macOS `say`): records the dump with `# STEP` marker lines.

usage: lights.py [OUT.log]    (default dumps/lights-<date>-<time>.log)
8 steps, 6 s each: all off, side lights, low beam, high beam, high off, front fog, rear fog, all off.
The dump is a toggle ('D'), so the script checks the state and starts it only if it is off.
"""
import os, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obc_serial import open_board

STEPS = [("ALL OFF", "All off. Do not touch the lights"), ("SIDE", "Side lights on"), ("LOW", "Low beam on"),
         ("HIGH", "High beam on, low beam stays"), ("LOW AGAIN", "High beam off, low beam stays"),
         ("FOG FRONT", "Front fog lights on"), ("FOG REAR", "Rear fog light on"), ("ALL OFF 2", "All lights off")]
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dumps",
                                                          "lights-" + time.strftime("%Y%m%d-%H%M") + ".log")
s = open_board(timeout=0.2)
f = open(out, "w", buffering=1)
t0 = time.time()


def pump(sec):
    end = time.time() + sec
    while time.time() < end:
        line = s.readline().decode(errors="replace").rstrip()
        if line:
            f.write("%.3f %s\n" % (time.time() - t0, line))


for _ in range(3):                                  # 'D' toggles: retry until the 'dump ON' line or frames show up
    s.write(b"D\n")
    n = f.tell()
    pump(2)
    f.flush()
    if "dump ON" in open(out).read() or " M " in open(out).read():
        break
for name, say in STEPS:
    subprocess.run(["say", say])
    f.write("%.3f # STEP %s\n" % (time.time() - t0, name))
    pump(6)
s.write(b"D\n")
pump(1)
f.write("%.3f # STEP END\n" % (time.time() - t0))
print(out)
