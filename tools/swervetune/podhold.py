"""Pod-PIDF tuning against the two regimes the driver actually hears and sees.

HOLD: release the sticks, X-lock engages, and the pods hunt audibly. Trigger: short forward
burst, release, record 6 s. Scored per pod over the window after the X engages:
    quiet_after  time until the servo stays below 0.02 power for a full second (the sound)
    act_energy   sum of |servo power changes| - actuation energy, the hunt's loudness
    err_pp       pod error peak-to-peak once engaged

STEADY: drive forward at 0.4 across the box and look only at the middle of the run:
    err_rms / err_pp   tracking error while holding 90 deg under drive load
    rev_s              wheel direction reversals per second (the visible wiggle)

    python podhold.py trial <tag>          # one hold + one steady at current gains
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

from swervebench import Bench, parse_csv, _mean, _archive
from boxdrive import Driver, wrap180

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "podhold.jsonl")




def _p95span(xs):
    s = sorted(xs)
    if len(s) < 5:
        return (s[-1] - s[0]) if s else float("nan")
    return s[int(0.95 * (len(s) - 1))] - s[int(0.05 * (len(s) - 1))]

def hold_trial(b, d, tag):
    yc = (d.y0 + d.y1) / 2
    cx = (d.x0 + d.x1) / 2
    d.drive_to(cx, yc, 0.3, 4.0)
    d.cmd(0, 0, 0)
    time.sleep(1.2)
    # burst then release
    t0 = time.time()
    while time.time() - t0 < 0.7:
        d.cmd(0.35, 0, 0)
        time.sleep(0.07)
    b.cmd("recStart", label=f"hold-{tag}")
    d.cmd(0, 0, 0)
    time.sleep(6.0)
    b.cmd("recStop")
    time.sleep(0.3)
    csv_text = b.rec_csv()
    tr = parse_csv(csv_text)
    t = tr["t"]
    rows = []
    for i in range(4):
        pwr = tr[f"p{i}_pwr"]
        err = tr[f"p{i}_err"]
        # analysis window: from 1.2 s (X engaged, arrival transient past) to end
        idx = [k for k in range(len(t)) if t[k] >= 2.0 and pwr[k] == pwr[k]]
        quiet_after = None
        run_start = None
        for k in idx:
            if abs(pwr[k]) < 0.02:
                if run_start is None:
                    run_start = t[k]
                if t[k] - run_start >= 1.0 and quiet_after is None:
                    quiet_after = run_start - 2.0
            else:
                run_start = None
        act = sum(abs(pwr[idx[j]] - pwr[idx[j-1]]) for j in range(1, len(idx)))
        errs = [err[k] for k in idx if err[k] == err[k]]
        rows.append({
            "pod": i,
            "quiet_after": round(quiet_after, 2) if quiet_after is not None else None,
            "act_energy": round(act, 1),
            "err_pp": round(_p95span(errs), 2) if errs else None,
        })
    _archive(csv_text, {"label": f"hold-{tag}"})
    return rows


def _field_x(d, power):
    """Field-frame +x demand, whatever the robot's heading. The old robot-frame
    forward drove into the fence whenever heading was rotated from the box axis
    (the exit tests field px) and measured pods fighting the clamp."""
    _, _, h, _ = d.pose()
    d.cmd(round(math.cos(h) * power, 3), round(-math.sin(h) * power, 3), 0)


def steady_trial(b, d, tag):
    yc = (d.y0 + d.y1) / 2
    d.drive_to(d.x0, yc, 0.3, 5.0)
    d.cmd(0, 0, 0)
    time.sleep(0.8)
    t0 = time.time()
    while time.time() - t0 < 0.5:
        _field_x(d, 0.08)
        time.sleep(0.07)
    b.cmd("recStart", label=f"steady-{tag}")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        px, _, _, cl = d.pose()
        if px >= d.x1 or cl:
            break
        _field_x(d, 0.3)
        time.sleep(0.07)
    b.cmd("recStop")
    d.cmd(0, 0, 0)
    time.sleep(0.4)
    csv_text = b.rec_csv()
    tr = parse_csv(csv_text)
    t = tr["t"]
    span = t[-1] if t else 0
    lo, hi = 0.9, max(1.3, span - 0.3)
    rows = []
    for i in range(4):
        err = tr[f"p{i}_err"]
        wheel = tr[f"p{i}_wheel"]
        idx = [k for k in range(len(t)) if lo <= t[k] <= hi]
        errs = [err[k] for k in idx if err[k] == err[k]]
        ws = [(t[k], wheel[k]) for k in idx if wheel[k] == wheel[k]]
        rev = 0
        dirn = 0
        ext = None
        for (tk, x) in ws:
            if ext is None:
                ext = x
                continue
            dx = wrap180(x - ext)
            if dirn >= 0 and dx > 0:
                ext = x
                dirn = 1
            elif dirn <= 0 and dx < 0:
                ext = x
                dirn = -1
            elif dirn == 1 and dx < -0.5:
                rev += 1
                dirn = -1
                ext = x
            elif dirn == -1 and dx > 0.5:
                rev += 1
                dirn = 1
                ext = x
        dur = max(0.1, hi - lo)
        rows.append({
            "pod": i,
            "err_rms": round(math.sqrt(sum(x * x for x in errs) / len(errs)), 2) if errs else None,
            "err_pp": round(_p95span(errs), 2) if errs else None,
            "rev_s": round(rev / dur, 2),
        })
    _archive(csv_text, {"label": f"steady-{tag}"})
    return rows


def trial(tag):
    b = Bench()
    d = Driver(b)
    st = b.state()
    gains = [{k: p.get(k) for k in ("kp", "kd", "ks", "ksband", "cache", "dom", "pulsed")}
             for p in st["pods"]]
    b.cmd("setPublishHz", value=10)
    hold = hold_trial(b, d, tag)
    steady = steady_trial(b, d, tag)
    b.cmd("setPublishHz", value=20)
    b.cmd("stop")
    out = {"tag": tag, "t": time.strftime("%H:%M:%S"), "volts": st["voltage"],
           "gains": gains, "hold": hold, "steady": steady}
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(out) + "\n")
    print(f"{tag} (V {st['voltage']:.2f}):")
    print(f"  {'pod':>3} {'quiet_after':>11} {'act_energy':>10} {'holdpp':>7} | "
          f"{'err_rms':>7} {'steadypp':>8} {'rev/s':>6}")
    for i in range(4):
        h, s = hold[i], steady[i]
        print(f"  {i:>3} {str(h['quiet_after']):>11} {h['act_energy']:>10.1f} "
              f"{str(h['err_pp']):>7} | {str(s['err_rms']):>7} {str(s['err_pp']):>8} "
              f"{s['rev_s']:>6.2f}")
    return out


if __name__ == "__main__":
    trial(sys.argv[1] if len(sys.argv) > 1 else "base")
