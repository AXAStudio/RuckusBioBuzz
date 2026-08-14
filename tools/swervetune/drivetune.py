"""Tune pod tracking under actual driving, not static steps.

Static step tuning is blind to the driving regime: drive-motor load changes the steering
friction, and the demand moves constantly - every stick release snaps the pods to the X and
back. This harness drives the robot forward/back with zero-input pauses (net displacement ~0,
stays inside a 30 inch box), records every loop, and scores each pod on:

  - err_p95 / err_rms while a drive command is active (tracking tightness)
  - excess wheel reversals per second: wheel direction changes minus target direction
    changes. The shake the driver feels is the wheel reversing when the demand did not.
  - settle after each X-snap transition (zero input -> pods swing ~45 deg)

Usage:
    python drivetune.py baseline                 # capture + score at current gains
    python drivetune.py cfg <tag> pod0spec pod1spec pod2spec pod3spec
    e.g. python drivetune.py cfg domtest "dom=true" "dom=true" "dom=true" "dom=true"
    (specs are merged onto CURRENT gains per pod; empty string = leave pod alone)

Each run appends a scored summary to current_runs/drivetune.jsonl and archives the trace.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

from swervebench import Bench, parse_csv, _mean, _archive

CYCLES = 6
FWD_POWER = 0.15
FWD_S = 0.6
PAUSE_S = 0.45
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "drivetune.jsonl")

POD_COUNT = 4

# A wheel-direction change only counts once the wheel has actually moved this far the other
# way; below it, encoder noise reads as reversals. ~4x the encoder LSB.
REV_HYST_DEG = 0.5


def drive_phase(b: Bench, f: float, s: float, t: float, dur: float) -> None:
    t0 = time.time()
    while time.time() - t0 < dur:
        b.cmd("drive", retries=1, f=f, s=s, t=t)
        time.sleep(0.09)


def reversals(series: list[float], hyst: float) -> int:
    """Direction changes of a position series with hysteresis, ignoring NaN."""
    xs = [x for x in series if x == x]
    if len(xs) < 3:
        return 0
    count = 0
    direction = 0
    extreme = xs[0]
    for x in xs[1:]:
        if direction >= 0 and x > extreme:
            extreme = x
            direction = 1
        elif direction <= 0 and x < extreme:
            extreme = x
            direction = -1
        elif direction == 1 and x < extreme - hyst:
            count += 1
            direction = -1
            extreme = x
        elif direction == -1 and x > extreme + hyst:
            count += 1
            direction = 1
            extreme = x
    return count


def unwrap(deg: list[float]) -> list[float]:
    out, acc, prev = [], 0.0, None
    for d in deg:
        if d != d:
            out.append(float("nan"))
            continue
        if prev is not None:
            step = d - prev
            if step > 180:
                acc -= 360
            elif step < -180:
                acc += 360
        out.append(d + acc)
        prev = d
    return out


def run(tag: str, pod_specs: list[dict] | None) -> None:
    b = Bench()
    if pod_specs is not None:
        for i, spec in enumerate(pod_specs):
            if spec:
                b.cmd("setPidf", scope="one", pod=i, **spec)
                time.sleep(0.15)
        time.sleep(0.3)

    st = b.state()
    gains = [{k: p.get(k) for k in ("kp", "kd", "ks", "ksband", "dom")} for p in st["pods"]]
    print(f"{tag}: V {st['voltage']:.2f}")
    for i, g in enumerate(gains):
        print(f"  pod {i}: {g}")

    b.cmd("recStart", label=f"drivetune-{tag}")
    time.sleep(0.3)
    for c in range(CYCLES):
        drive_phase(b, FWD_POWER, 0, 0, FWD_S)
        drive_phase(b, 0, 0, 0, PAUSE_S)
        drive_phase(b, -FWD_POWER, 0, 0, FWD_S)
        drive_phase(b, 0, 0, 0, PAUSE_S)
    b.cmd("drive", retries=1, f=0, s=0, t=0)
    time.sleep(0.2)
    b.cmd("recStop")
    time.sleep(0.3)
    b.cmd("stop")

    csv_text = b.rec_csv()
    tr = parse_csv(csv_text)
    dts = [x for x in tr["dt"] if x == x and x > 0]
    span = sum(dts)
    out = {"tag": tag, "t": time.strftime("%H:%M:%S"), "volts": st["voltage"],
           "hz": 1.0 / _mean(dts), "span_s": span, "gains": gains, "pods": []}

    print(f"\n  {span:.1f} s at {out['hz']:.0f} Hz true")
    print(f"  {'pod':>3} {'err_rms':>8} {'err_p95':>8} {'tgt_rev/s':>9} {'whl_rev/s':>9} "
          f"{'excess/s':>8}")
    for i in range(POD_COUNT):
        err = [abs(x) for x in tr[f"p{i}_err"] if x == x]
        err_sorted = sorted(err)
        wheel_un = unwrap(tr[f"p{i}_wheel"])
        tgt_un = unwrap(tr[f"p{i}_tgt"])
        wrev = reversals(wheel_un, REV_HYST_DEG) / span
        trev = reversals(tgt_un, REV_HYST_DEG) / span
        row = {
            "pod": i,
            "err_rms": math.sqrt(sum(x * x for x in err) / len(err)) if err else float("nan"),
            "err_p95": err_sorted[int(0.95 * (len(err_sorted) - 1))] if err_sorted else float("nan"),
            "tgt_rev_s": trev, "wheel_rev_s": wrev, "excess_rev_s": wrev - trev,
        }
        out["pods"].append(row)
        print(f"  {i:>3} {row['err_rms']:>8.2f} {row['err_p95']:>8.2f} {trev:>9.2f} "
              f"{wrev:>9.2f} {row['excess_rev_s']:>8.2f}")

    out["trace_file"] = _archive(csv_text, {"label": f"drivetune-{tag}"})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(out) + "\n")


def parse_spec(s: str) -> dict:
    if not s.strip():
        return {}
    out = {}
    for part in s.split(","):
        k, v = part.split("=")
        out[k.strip()] = v.strip()
    return out


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    if mode == "baseline":
        run("baseline", None)
    else:
        tag = sys.argv[2]
        specs = [parse_spec(s) for s in sys.argv[3:7]]
        run(tag, specs)
