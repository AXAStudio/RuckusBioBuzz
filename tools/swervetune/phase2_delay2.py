"""Transport delay, measured without a detection threshold.

The first pass declared "first motion" at 0.3 deg of travel, which is not a delay - it is the time
the pod spends accelerating through 0.3 deg, and it inflates the answer by however long that takes.

For a first-order velocity response behind a transport delay,

    theta(t) = V * [ (t - td) - tau * (1 - exp(-(t - td)/tau)) ]

the late-time asymptote is theta = V * (t - td - tau), which crosses zero at t = td + tau. Fitting
that asymptote and subtracting the separately measured tau gives td with no threshold in it.

Also reports where the recorded timestamp sits relative to the servo write, since the recorder
samples at the end of the loop body while the write happens at the start of it.

    python phase2_delay2.py
"""

from __future__ import annotations

import math
import time

from swervebench import Bench, parse_csv, _mean, _stdev
from phase2_plant import unwrap, segments

POD_COUNT = 4
POWER = 1.00
DWELL_S = 0.35
REPEATS = 6
VEL_WINDOW = 5


def fit_delay(tr: dict, pod: int) -> dict | None:
    t = tr["t"]
    ang = unwrap(tr[f"p{pod}_wheel"])
    pwr = tr[f"p{pod}_pwr"]
    segs = segments(t, pwr)
    if not segs:
        return None
    a, z = segs[0]["start"], segs[0]["end"]
    if z - a < VEL_WINDOW + 6:
        return None

    t0 = t[a]
    base = ang[max(0, a - 1)]
    rel = [(t[k] - t0, abs(ang[k] - base)) for k in range(a, z + 1)]

    # A backward difference estimates the velocity at the MIDDLE of its window, not at its end.
    # Timestamping it at the end adds half a window - about 17 ms here - straight onto any delay
    # measured from it, which is most of why the first pass came out so large.
    vs = []
    for k in range(a + VEL_WINDOW, z + 1):
        dt = t[k] - t[k - VEL_WINDOW]
        if dt > 1e-6:
            mid = 0.5 * (t[k] + t[k - VEL_WINDOW]) - t0
            vs.append((mid, abs(ang[k] - ang[k - VEL_WINDOW]) / dt))
    if len(vs) < 8:
        return None
    v_inf = _mean([v for _, v in vs[int(2 * len(vs) / 3):]])
    if not v_inf or v_inf < 50:
        return None

    # v(t) = V(1 - exp(-(t - td)/tau)) rearranges to ln(1 - v/V) = td/tau - t/tau, linear in t.
    # Slope gives tau and intercept gives td, so the two separate instead of collapsing into the
    # same quantity the way a 63%-crossing and an asymptote intercept do.
    pts = []
    for tv, v in vs:
        frac = v / v_inf
        if 0.15 < frac < 0.85:
            pts.append((tv, math.log(1.0 - frac)))
    if len(pts) < 4:
        return None
    n = len(pts)
    sx = sum(x for x, _ in pts)
    sy = sum(y for _, y in pts)
    sxx = sum(x * x for x, _ in pts)
    sxy = sum(x * y for x, y in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    slope = (n * sxy - sx * sy) / den
    if slope >= -1e-6:
        return None
    tau_s = -1.0 / slope
    td_s = ((sy - slope * sx) / n) * tau_s
    if not (0.0 <= td_s < 0.3) or not (0.005 < tau_s < 0.5):
        return None
    tau = tau_s

    # Least-squares line through the last 60% of the travel, where velocity is terminal.
    late = [(x, y) for x, y in rel if x >= 0.4 * rel[-1][0]]
    n = len(late)
    if n < 4:
        return None
    sx = sum(x for x, _ in late)
    sy = sum(y for _, y in late)
    sxx = sum(x * x for x, _ in late)
    sxy = sum(x * y for x, y in late)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-12:
        return None
    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n
    if slope <= 1e-6:
        return None

    zero_cross = -intercept / slope          # independent check: equals td + tau
    return {
        "v_inf": slope,
        "tau_ms": tau * 1000.0,
        "td_ms": td_s * 1000.0,
        "zero_ms": zero_cross * 1000.0,
    }


def main() -> None:
    b = Bench()
    print(f"battery {b.voltage():.2f} V")
    print(f"threshold-free transport delay, power {POWER}, {REPEATS} repeats per pod\n")
    print(f"  {'pod':>3} {'td ms':>16} {'tau ms':>16} {'v_inf deg/s':>12}")

    tds, taus = [], []
    for pod in range(POD_COUNT):
        rows = []
        for n in range(REPEATS):
            sign = 1 if n % 2 == 0 else -1
            b.cmd("select", pod=pod)
            b.cmd("recStart", label=f"delay2-p{pod}-{n}")
            time.sleep(0.2)
            b.cmd("rawServo", pow=sign * POWER, sec=DWELL_S, pod=pod)
            time.sleep(DWELL_S + 0.45)
            b.cmd("recStop")
            time.sleep(0.15)
            r = fit_delay(parse_csv(b.rec_csv()), pod)
            if r:
                rows.append(r)
        if not rows:
            print(f"  {pod:>3}   no usable fit")
            continue
        d = [r["td_ms"] for r in rows]
        tu = [r["tau_ms"] for r in rows]
        tds.extend(d)
        taus.extend(tu)
        print(f"  {pod:>3} {_mean(d):>8.1f} +/-{_stdev(d):>5.1f} {_mean(tu):>8.1f} +/-{_stdev(tu):>5.1f} "
              f"{_mean([r['v_inf'] for r in rows]):>12.0f}")
    b.stop()

    if tds:
        print(f"\n  transport delay td = {_mean(tds):.1f} +/- {_stdev(tds):.1f} ms")
        print(f"  velocity lag  tau = {_mean(taus):.1f} +/- {_stdev(taus):.1f} ms")
        print(f"  total equivalent  = {_mean(tds) + _mean(taus):.1f} ms")
        print("\n  Both timestamps come from the on-robot recorder and the servo write happens in")
        print("  the same loop iteration as the first of them, so no part of this is the laptop,")
        print("  the HTTP poll or the state publish. The recorder samples at the end of the loop")
        print("  body while the write is at the start, which biases td low by about one loop.")


if __name__ == "__main__":
    main()
