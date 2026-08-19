"""Tune the heading-hold PIDF with in-place rotation steps.

Each trial commands headingGoto +/-N degrees (alternating sign, so the net rotation over a cell
is ~zero and the robot stays planted) and polls /state while the loop corrects. Scored on what
"accurate and snappy" actually means:

  settle   time until |error| enters and STAYS inside 2 deg
  over     worst excursion past the target, deg
  ss       |error| at the end of the window
  osc      error sign changes that swing past 1 deg on both sides (shake)
  diseng   whether the hold state machine disengaged cleanly (correcting == False at end)

Gains are set at runtime via setHeadingPidf, so a whole sweep needs no redeploy.

    python headtune.py baseline
    python headtune.py cell <hkp> <hkd> [step_deg] [n]
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

from swervebench import Bench, _mean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "headtune.jsonl")
WINDOW_S = 3.2
SETTLE_BAND = 2.0
OSC_BAND = 1.0


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def one_step(b: Bench, deg: float) -> dict:
    b.cmd("headingGoto", deg=deg)
    t0 = time.time()
    ts, errs, cors = [], [], []
    while time.time() - t0 < WINDOW_S:
        st = b.state()
        h = st["heading"]
        ts.append(time.time() - t0)
        errs.append(wrap180(h["targetDeg"] - h["deg"]))
        cors.append(bool(h.get("correcting")))
        time.sleep(0.02)

    # The first sample or two can predate the command draining on the robot (error still ~0
    # against the OLD target); drop the initial 0.15 s so neither settle nor overshoot reads
    # the pre-step state.
    keep = [k for k in range(len(ts)) if ts[k] >= 0.15]
    ts = [ts[k] for k in keep]
    errs = [errs[k] for k in keep]
    cors = [cors[k] for k in keep]

    # settle: last time |err| exceeded the band
    settle = 0.0
    for k, e in enumerate(errs):
        if abs(e) > SETTLE_BAND:
            settle = None if k == len(errs) - 1 else ts[k + 1]
    # overshoot: worst excursion past the target, signed by the COMMANDED direction - never by
    # the first sample, which cost a -90 trial a phantom "90 deg overshoot".
    sign0 = 1.0 if deg >= 0 else -1.0
    over = max([0.0] + [-e * sign0 for e in errs])
    # oscillations: sign changes swinging past OSC_BAND both ways
    osc = 0
    armed = 0
    for e in errs:
        if abs(e) < OSC_BAND:
            continue
        s = 1 if e > 0 else -1
        if armed == 0:
            armed = s
        elif s != armed:
            osc += 1
            armed = s
    return {
        "deg": deg, "settle": settle, "over": round(over, 2),
        "ss": round(abs(errs[-1]), 2) if errs else float("nan"),
        "osc": osc, "diseng": (not cors[-1]) if cors else None,
        "n_samples": len(ts),
    }


def run_cell(hkp: float, hkd: float, step_deg: float, n: int, tag: str) -> None:
    b = Bench()
    b.cmd("setHeadingPidf", hkp=hkp, hkd=hkd, hkf=0)
    time.sleep(0.3)
    print(f"{tag}: hkp {hkp} hkd {hkd}, {n} x ±{step_deg} deg, V {b.voltage():.2f}")
    rows = []
    for i in range(n):
        deg = step_deg if i % 2 == 0 else -step_deg
        r = one_step(b, deg)
        rows.append(r)
        print(f"  [{i+1}/{n}] {deg:+.0f}: settle {r['settle'] if r['settle'] is not None else 'NEVER'}"
              f"  over {r['over']:5.2f}  ss {r['ss']:5.2f}  osc {r['osc']}  diseng {r['diseng']}")
        time.sleep(0.6)
    b.cmd("setHeadingHold", value="false")
    b.cmd("stop")

    settled = [r["settle"] for r in rows if r["settle"] is not None]
    summary = {
        "tag": tag, "hkp": hkp, "hkd": hkd, "step": step_deg,
        "t": time.strftime("%H:%M:%S"), "volts": b.voltage(),
        "settle_mean": round(_mean(settled), 3) if settled else None,
        "settled_frac": len(settled) / len(rows),
        "over_max": max(r["over"] for r in rows),
        "ss_mean": round(_mean([r["ss"] for r in rows]), 2),
        "osc_max": max(r["osc"] for r in rows),
        "rows": rows,
    }
    print(f"  => settle {summary['settle_mean']} ({100*summary['settled_frac']:.0f}% settled)  "
          f"over_max {summary['over_max']:.2f}  ss {summary['ss_mean']:.2f}  osc_max {summary['osc_max']}")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(summary) + "\n")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    if mode == "baseline":
        run_cell(1.20, 0.030, 90.0, 4, "baseline")
    else:
        hkp, hkd = float(sys.argv[2]), float(sys.argv[3])
        step = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0
        n = int(sys.argv[5]) if len(sys.argv) > 5 else 4
        run_cell(hkp, hkd, step, n, f"kp{hkp:g}-kd{hkd:g}-s{step:g}")
