"""Fast straight-line dashes, scored on path straightness and pod wiggle.

The drill the driver asked for: drive forward fast, watch the wiggle, make it straight. Each
set positions the robot at one end of the box's long axis, then dashes back and forth at a
fixed RAW forward power (no field-frame correction - straightness must come from the pods, not
from the scorer steering). Per dash: lateral bow (max |y| deviation from the start line), end
drift, heading wander. Per set: pod engaged-reversal rate and tracking error from a loop-rate
recorder chunk.

    python dashtune.py set <tag> <power> [n_dashes]
    python dashtune.py report
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

from swervebench import Bench, parse_csv, _mean, _archive
from boxdrive import Driver, score_chunk, wrap180

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "dashtune.jsonl")
INSET = 9.0
SETTLE_S = 0.8


def run_set(tag, power, n):
    b = Bench()
    d = Driver(b)
    # dash along x (the box's long axis; heading 0 faces +x)
    x_lo = d.x0 + (INSET - 10.0) + 0.0
    x_lo, x_hi = d.x0, d.x1
    yc = (d.y0 + d.y1) / 2
    st = b.state()
    gains = [{k: p.get(k) for k in ("kp", "kd", "ks", "ksband")} for p in st["pods"]]
    print(f"{tag}: power {power}, {n} dashes over x {x_lo:.0f}..{x_hi:.0f} at y {yc:.0f}, "
          f"V {st['voltage']:.2f}")

    b.cmd("setPublishHz", value=10)
    dashes = []
    b.cmd("recStart", label=f"dash-{tag}")
    for k in range(n):
        fwd = (k % 2 == 0)
        # position to the launch end, settle
        sx = x_lo if fwd else x_hi
        d.drive_to(sx, yc, 0.3, 4.0)
        d.cmd(0, 0, 0)
        time.sleep(SETTLE_S)

        # Creep first so the pods are ALIGNED to forward before the power lands - otherwise the
        # launch is a sideways shove through half-swung pods and the "drift" measures the
        # transient, not the tracking.
        t0 = time.time()
        while time.time() - t0 < 0.5:
            d.cmd(0.08 if fwd else -0.08, 0, 0)
            time.sleep(0.07)

        x, y0, h0, _ = d.pose()
        ys, hs, xs = [], [], []
        t0 = time.time()
        while time.time() - t0 < 3.5:
            px, py, ph, clamped = d.pose()
            xs.append(px)
            ys.append(py)
            hs.append(math.degrees(ph))
            done = (px >= x_hi) if fwd else (px <= x_lo)
            if done or clamped:
                break
            d.cmd(power if fwd else -power, 0, 0)
            time.sleep(0.07)
        d.cmd(0, 0, 0)
        time.sleep(0.4)
        bow = max(abs(v - y0) for v in ys) if ys else float("nan")
        drift = (ys[-1] - y0) if ys else float("nan")   # signed: + is field +y (left at hdg 0)
        run = abs(xs[-1] - xs[0]) if xs else float("nan")
        crab = math.degrees(math.atan2(drift, run)) if run and run > 1 else float("nan")
        hwander = max(abs(wrap180(v - math.degrees(h0))) for v in hs) if hs else float("nan")
        dashes.append({"dir": "+x" if fwd else "-x", "bow": round(bow, 2),
                       "drift": round(drift, 2), "run": round(run, 1),
                       "crab": round(crab, 2), "hdg": round(hwander, 2)})
        print(f"  dash {k+1} {'+x' if fwd else '-x'}: run {run:.0f} in  drift {drift:+.1f} in "
              f"(crab {crab:+.1f} deg)  bow {bow:.1f}  hdg wander {hwander:.1f} deg")
    b.cmd("recStop")
    time.sleep(0.3)
    b.cmd("setPublishHz", value=20)
    b.cmd("stop")

    csv_text = b.rec_csv()
    pods, hz = score_chunk(csv_text)
    out = {"tag": tag, "power": power, "t": time.strftime("%H:%M:%S"),
           "volts": st["voltage"], "hz": round(hz, 1), "gains": gains,
           "bow_mean": round(_mean([x["bow"] for x in dashes]), 2),
           "bow_max": max(x["bow"] for x in dashes),
           "hdg_max": max(x["hdg"] for x in dashes),
           "dashes": dashes, "pods": pods}
    out["trace_file"] = _archive(csv_text, {"label": f"dash-{tag}"})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(out) + "\n")
    print(f"  => bow mean {out['bow_mean']} max {out['bow_max']} in, hdg max {out['hdg_max']} "
          f"deg, loop {hz:.0f} Hz")
    for r in pods:
        print(f"     pod {r['pod']}: rev/s {r['eng_rev_s']}  |err| rms {r['err_rms']}")
    return out


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    if mode == "set":
        run_set(sys.argv[2], float(sys.argv[3]), int(sys.argv[4]) if len(sys.argv) > 4 else 4)
    else:
        for line in open(OUT, encoding="utf-8"):
            r = json.loads(line)
            worst = max(p["eng_rev_s"] for p in r["pods"])
            print(f"{r['tag']:<20} pow {r['power']:.2f} V {r['volts']:.2f} bow "
                  f"{r['bow_mean']:>5.2f}/{r['bow_max']:<5.2f} hdg {r['hdg_max']:>5.1f} "
                  f"rev/s {worst}")
