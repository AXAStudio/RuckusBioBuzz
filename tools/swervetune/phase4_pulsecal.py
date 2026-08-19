"""Calibrate how far one open-loop pulse actually moves a pod.

This is the number the whole pulsed approach turns on. A pulse that travels further than the
tolerance it is trying to reach cannot converge - the pod overshoots, pulses back, and hunts at the
pulse repetition rate, which is exactly what a fixed 20 ms pulse against a 0.5 deg tolerance did.

Travel should be close to (terminal speed at that power) x (pulse duration), but the servo PWM
frame quantises the duration and the friction drop makes the terminal speed direction-dependent, so
it is measured rather than predicted.

    python phase4_pulsecal.py
"""

from __future__ import annotations

import math
import time

from swervebench import Bench, parse_csv, _mean, _stdev, _archive
from phase2_plant import unwrap, segments

POD_COUNT = 4
POWERS = (0.040, 0.050, 0.060)
DURATIONS_MS = (20, 30, 45, 70)
REPEATS = 3


def one_pulse(b: Bench, pod: int, power: float, ms: float, sign: int) -> float | None:
    """Fires a single pulse from rest and returns total travel once the pod has stopped."""
    b.cmd("select", pod=pod)
    b.cmd("recStart", label=f"pulsecal-p{pod}")
    time.sleep(0.25)
    b.cmd("rawServo", pow=sign * power, sec=ms / 1000.0, pod=pod)
    # Long enough for the pulse plus a full coast-down; tau is about 42 ms.
    time.sleep(ms / 1000.0 + 0.60)
    b.cmd("recStop")
    time.sleep(0.15)

    csv_text = b.rec_csv()
    _archive(csv_text, {"label": f"pulsecal-p{pod}-{power:.3f}-{ms:.0f}ms"})
    tr = parse_csv(csv_text)
    t = tr["t"]
    ang = unwrap(tr[f"p{pod}_wheel"])
    pwr = tr[f"p{pod}_pwr"]
    segs = segments(t, pwr)
    if not segs:
        return None
    a = segs[0]["start"]
    base = ang[max(0, a - 1)]
    # Settled value from the tail, after the coast is over.
    tail = [ang[k] for k in range(len(t)) if t[k] >= t[-1] - 0.20]
    if not tail:
        return None
    return abs(_mean(tail) - base)


def main() -> None:
    b = Bench()
    print(f"battery {b.voltage():.2f} V")
    print("Travel per single pulse, from rest, degrees\n")

    header = "  pwr    ms  " + "".join(f"  p{p}+    p{p}-  " for p in range(POD_COUNT))
    print(header)
    rates = {}
    reps = {}
    for power in POWERS:
        for ms in DURATIONS_MS:
            cells = ""
            for pod in range(POD_COUNT):
                for sign in (1, -1):
                    vals = []
                    for _ in range(REPEATS):
                        v = one_pulse(b, pod, power, ms, sign)
                        if v is not None:
                            vals.append(v)
                    m = _mean(vals) if vals else float("nan")
                    rates[(power, ms, pod, sign)] = m
                    reps[(power, ms, pod, sign)] = vals
                    cells += f"{m:6.2f} "
            print(f"  {power:<5.3f} {ms:>3.0f} {cells}")
    b.stop()

    print("\nImplied travel rate, deg per second of pulse (travel / duration):")
    print(f"  {'power':>6} " + " ".join(f"{ms:>5.0f}ms" for ms in DURATIONS_MS))
    for power in POWERS:
        cells = ""
        for ms in DURATIONS_MS:
            vs = [rates[(power, ms, p, s)] for p in range(POD_COUNT) for s in (1, -1)]
            vs = [v for v in vs if v == v]
            cells += f"{(_mean(vs) / (ms / 1000.0)) if vs else float('nan'):>7.0f}"
        print(f"  {power:>6.3f} {cells}")

    print("\nSmallest reliable pulse, and what tolerance it permits:")
    print("  Two scatters matter and they are not the same thing. Between-condition is how much")
    print("  the pods and directions differ from each other, which a single pulse power has to")
    print("  straddle. Within-condition is how much one pod repeats, which no calibration can")
    print("  remove. The mechanical pass is aimed at both; report both.")
    for power in POWERS:
        cells = [(p, s) for p in range(POD_COUNT) for s in (1, -1)]
        means = [rates[(power, DURATIONS_MS[0], p, s)] for p, s in cells]
        means = [v for v in means if v == v]
        withins = [
            reps[(power, DURATIONS_MS[0], p, s)]
            for p, s in cells
            if len(reps.get((power, DURATIONS_MS[0], p, s), [])) > 1
        ]
        within_sd = _mean([_stdev(v) for v in withins]) if withins else float("nan")
        if not means:
            continue
        print(f"\n  power {power:.3f}, {DURATIONS_MS[0]:.0f} ms:")
        print(f"    travel {min(means):.2f}-{max(means):.2f} deg, "
              f"mean {_mean(means):.2f}")
        print(f"    between-condition sd {_stdev(means):.2f} deg, "
              f"within-condition sd {within_sd:.2f} deg")
        combined = _mean(means) + math.sqrt(
            _stdev(means) ** 2 + (0.0 if within_sd != within_sd else within_sd ** 2)
        )
        print(f"    mean + combined sd = {combined:.2f} deg   (target for pulsed control: 0.50)")


if __name__ == "__main__":
    main()
