"""Full-speed diagonal runs, heading as the wobble meter.

The box diagonal is the longest line available, and chassis heading is the cleanest wobble
metric there is: pod oscillation asymmetries become yaw jitter, and the IMU does not share the
pod encoders' noise. Each run targets the far corner (inset), commanding field-frame
translation at high power with zero rotation input - the soft heading lock holds heading, and
whatever jitter remains IS the wobble. The recorder runs so per-pod wheel motion can be
correlated (pod 0 is the suspect).

Per run: heading p95 span, heading reversal count (0.4 deg hysteresis), end drift, mean speed.
Per pod from the chunk: wheel p95 span in the mid-window.

    python diagtest.py run <tag> <power> [n_runs]
"""

from __future__ import annotations

import gzip
import glob
import json
import math
import os
import sys
import time

from swervebench import Bench, parse_csv, _mean, _archive
from boxdrive import Driver, wrap180
from podhold import _p95span

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "diagtest.jsonl")
INSET = 8.0


def one_run(b, d, tag, corner_from, corner_to, power):
    d.drive_to(corner_from[0], corner_from[1], 0.32, 7.0)
    d.cmd(0, 0, 0)
    time.sleep(0.9)

    b.cmd("recStart", label=f"diag-{tag}")
    hs, ts, poses = [], [], []
    # Bearing locked at launch: recomputing it each poll makes the pod demand itself swing
    # near arrival, which pollutes the wobble read. Constant demand, run until the fence zone.
    x0p, y0p, h0p, _ = d.pose()
    m0 = math.hypot(corner_to[0] - x0p, corner_to[1] - y0p)
    ux, uy = (corner_to[0] - x0p) / m0, (corner_to[1] - y0p) / m0
    t0 = time.time()
    while time.time() - t0 < 4.0:
        x, y, hrad, cl = d.pose()
        hs.append(math.degrees(hrad))
        ts.append(time.time() - t0)
        poses.append((x, y))
        dx, dy = corner_to[0] - x, corner_to[1] - y
        if (dx * ux + dy * uy) < 8 or cl:
            break
        f = (ux * math.cos(hrad) + uy * math.sin(hrad)) * power
        s = (-ux * math.sin(hrad) + uy * math.cos(hrad)) * power
        d.cmd(round(f, 3), round(s, 3), 0)
        time.sleep(0.055)
    d.cmd(0, 0, 0)
    time.sleep(0.35)
    b.cmd("recStop")
    time.sleep(0.25)

    # heading metrics over the run's middle (skip launch 0.4 s)
    mid = [(ts[k], hs[k]) for k in range(len(ts)) if ts[k] >= 0.4]
    rel = [wrap180(h - mid[0][1]) for (_, h) in mid] if mid else [0]
    hspan = _p95span(rel)
    # heading reversals with hysteresis
    rev = 0
    dirn = 0
    ext = None
    for v in rel:
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
        elif dirn == 1 and dv < -0.4:
            rev += 1
            dirn = -1
            ext = v
        elif dirn == -1 and dv > 0.4:
            rev += 1
            dirn = 1
            ext = v
    run_len = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])
    dur = ts[-1] if ts else 1
    speed = run_len / max(0.2, dur)

    # per-pod wheel spans from the chunk
    csv_text = b.rec_csv()
    tr = parse_csv(csv_text)
    t = tr["t"]
    span_t = t[-1] if t else 1
    lo, hi = 0.4, max(0.8, span_t - 0.25)
    pods = []
    for i in range(4):
        wheel = tr[f"p{i}_wheel"]
        ws = [wheel[k] for k in range(len(t)) if lo <= t[k] <= hi and wheel[k] == wheel[k]]
        relw = [wrap180(w - ws[0]) for w in ws] if ws else [0]
        pods.append(round(_p95span(relw), 1))
    _archive(csv_text, {"label": f"diag-{tag}"})

    return {"hspan": round(hspan, 2), "hrev": rev, "drift": round(rel[-1], 2),
            "run_in": round(run_len, 1), "speed": round(speed, 1), "pods": pods}


def campaign(tag, power, n):
    b = Bench()
    d = Driver(b)
    st = b.state()
    corners = [(d.x0, d.y0), (d.x1, d.y1), (d.x1, d.y0), (d.x0, d.y1)]
    print(f"{tag}: power {power}, {n} diagonal runs, V {st['voltage']:.2f}, "
          f"diag ~{math.hypot(d.x1-d.x0, d.y1-d.y0):.0f} in")
    b.cmd("setPublishHz", value=10)
    rows = []
    pairs = [(corners[0], corners[1]), (corners[1], corners[0]),
             (corners[2], corners[3]), (corners[3], corners[2])]
    for k in range(n):
        a, c = pairs[k % 4]
        r = one_run(b, d, f"{tag}-{k+1}", a, c, power)
        rows.append(r)
        print(f"  run {k+1}: {r['run_in']} in @ {r['speed']} in/s  hdg span {r['hspan']} "
              f"rev {r['hrev']} drift {r['drift']:+.2f}  pods {r['pods']}")
    b.cmd("setPublishHz", value=20)
    b.cmd("stop")
    summ = {"tag": tag, "power": power, "volts": st["voltage"],
            "t": time.strftime("%H:%M:%S"),
            "hspan_mean": round(_mean([r["hspan"] for r in rows]), 2),
            "hspan_max": max(r["hspan"] for r in rows),
            "hrev_mean": round(_mean([r["hrev"] for r in rows]), 1),
            "pod_span_max": [max(r["pods"][i] for r in rows) for i in range(4)],
            "rows": rows}
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(summ) + "\n")
    print(f"  => hdg span mean {summ['hspan_mean']} max {summ['hspan_max']} deg, "
          f"rev mean {summ['hrev_mean']}, pod span max {summ['pod_span_max']}")
    return summ


if __name__ == "__main__":
    tag = sys.argv[2] if len(sys.argv) > 2 else "base"
    power = float(sys.argv[3]) if len(sys.argv) > 3 else 0.8
    n = int(sys.argv[4]) if len(sys.argv) > 4 else 4
    campaign(tag, power, n)
