#!/usr/bin/env python3
"""Per-ID inventory of a dump log: frames, change rate, and how many bytes ever change.

usage: analyze.py LOG [LOG ...]
Plain stdlib; the regex ignores log lines of the firmware that are interleaved with frames.
"""
import collections, re, sys

FRAME = re.compile(r"^\s*([\d.]+) ([MH]) ([0-9A-F]{3}) \[(\d)\]((?: [0-9A-F]{2}){1,8})\s*$")


def load(path):
    for line in open(path):
        m = FRAME.match(line)
        if m and len(m[5].split()) == int(m[4]):
            yield float(m[1]), m[2], m[3], [int(x, 16) for x in m[5].split()]


if __name__ == "__main__":
    for path in sys.argv[1:]:
        st = collections.defaultdict(lambda: {"n": 0, "v": collections.defaultdict(set)})
        span = 0.0
        for t, bus, ident, data in load(path):
            span = max(span, t)
            s = st[(bus, ident)]
            s["n"] += 1
            for j, b in enumerate(data):
                s["v"][j].add(b)
        print(f"== {path}  ({span:.0f} s)")
        print("bus id    changes  /s    distinct values per byte")
        for (bus, ident), s in sorted(st.items(), key=lambda x: (x[0][0], int(x[0][1], 16))):
            print(f"{bus}   0x{ident}  {s['n']:7d} {s['n'] / span:5.1f}  " +
                  " ".join(f"{len(s['v'][j]):3d}" for j in sorted(s["v"])))
