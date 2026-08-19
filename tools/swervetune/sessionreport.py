"""Analyze a recorded driving session for wobble and heading drift.

Reads every chunk of a session (runs/*<label>-NNN*.csv.gz) and reports, with timestamps:

  DRIFT episodes    heading pulled away from the hold target while translating with the
                    rotation stick idle (|htgt - heading| growing past 3 deg)
  WOBBLE episodes   yaw oscillation: heading direction reversals above 0.4 deg at > 1.5/s
                    over a sliding window
  POD episodes      a pod's wheel reversing direction > 4/s while its servo is engaged
                    (chatter), or flip toggling

Each episode lines up against what the sticks were doing (cf/cs/ct means over the window).

    python sessionreport.py <label>
"""

from __future__ import annotations

import glob
import gzip
import math
import os
import sys

from swervebench import parse_csv, _mean

RUN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def reversals_idx(vals, hyst):
    """Indices where direction reverses (with hysteresis)."""
    out = []
    dirn = 0
    ext = None
    for k, v in enumerate(vals):
        if v != v:
            continue
        if ext is None:
            ext = v
            continue
        dv = v - ext
        if dirn >= 0 and dv > 0:
            ext = v
            dirn = 1
        elif dirn <= 0 and dv < 0:
            ext = v
            dirn = -1
        elif dirn == 1 and dv < -hyst:
            out.append(k)
            dirn = -1
            ext = v
        elif dirn == -1 and dv > hyst:
            out.append(k)
            dirn = 1
            ext = v
    return out


def analyze_chunk(path):
    tr = parse_csv(gzip.open(path, "rt", encoding="utf-8").read())
    t = tr["t"]
    n = len(t)
    if n < 40:
        return []
    heading = tr.get("heading", [float("nan")] * n)
    htgt = tr.get("htgt", [float("nan")] * n)
    cf = tr.get("cf", [0] * n)
    cs = tr.get("cs", [0] * n)
    ct = tr.get("ct", [0] * n)

    episodes = []

    # sliding 1.5 s windows, 0.75 s step
    span = t[-1]
    w = 0.0
    while w + 1.5 <= span:
        idx = [k for k in range(n) if w <= t[k] <= w + 1.5]
        if len(idx) < 15:
            w += 0.75
            continue
        trans = _mean([math.hypot(cf[k], cs[k]) for k in idx])
        turn_cmd = _mean([abs(ct[k]) for k in idx])
        herr = [wrap180(htgt[k] - heading[k]) for k in idx
                if htgt[k] == htgt[k] and heading[k] == heading[k]]
        hs = [heading[k] for k in idx if heading[k] == heading[k]]
        hrel = [wrap180(v - hs[0]) for v in hs] if hs else []

        # drift: hold target receding while translating, rotation quiet
        if herr and trans > 0.12 and turn_cmd < 0.12:
            worst = max(abs(v) for v in herr)
            if worst > 3.0:
                episodes.append((w, "DRIFT", f"heading {worst:.1f} deg off target",
                                 trans, turn_cmd))
        # wobble: yaw reversals
        revs = reversals_idx(hrel, 0.4)
        if len(revs) / 1.5 > 1.5 and (max(hrel) - min(hrel)) > 1.2:
            episodes.append((w, "WOBBLE",
                             f"yaw osc {len(revs)/1.5:.1f} rev/s span "
                             f"{max(hrel)-min(hrel):.1f} deg", trans, turn_cmd))
        # pod chatter
        for i in range(4):
            wheel = [tr[f"p{i}_wheel"][k] for k in idx]
            pwr = [tr[f"p{i}_pwr"][k] for k in idx]
            engaged = sum(1 for v in pwr if v == v and abs(v) > 0.05)
            if engaged < len(idx) * 0.4:
                continue
            wrel = []
            base = None
            for v in wheel:
                if v != v:
                    wrel.append(float("nan"))
                    continue
                if base is None:
                    base = v
                wrel.append(wrap180(v - base))
            revs_p = reversals_idx(wrel, 0.6)
            if len(revs_p) / 1.5 > 4.0:
                episodes.append((w, f"POD{i}",
                                 f"wheel chatter {len(revs_p)/1.5:.1f} rev/s",
                                 trans, turn_cmd))
        w += 0.75

    return episodes


def main(label):
    files = sorted(glob.glob(os.path.join(RUN_DIR, f"*{label}-*.csv.gz")))
    if not files:
        print(f"no chunks matching '{label}'")
        return
    print(f"session '{label}': {len(files)} chunks")
    totals = {}
    for f in files:
        eps = analyze_chunk(f)
        name = os.path.basename(f)
        if eps:
            print(f"\n{name}:")
            merged = []
            for (w, kind, desc, trans, turn) in eps:
                if merged and merged[-1][1] == kind and w - merged[-1][0] <= 0.8:
                    merged[-1] = (w, kind, desc, trans, turn, merged[-1][5] + 1)
                    continue
                merged.append((w, kind, desc, trans, turn, 1))
            for (w, kind, desc, trans, turn, reps) in merged:
                dur = "" if reps == 1 else f" (~{reps*0.75:.1f}s)"
                print(f"  t={w:5.1f}s  {kind:<7} {desc}{dur}  "
                      f"[sticks: trans {trans:.2f} turn {turn:.2f}]")
                totals[kind] = totals.get(kind, 0) + reps
    print("\ntotals:", totals if totals else "clean session - no episodes found")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "drive")
