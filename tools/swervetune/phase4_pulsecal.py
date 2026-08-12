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

import time

from swervebench import Bench, parse_csv, _mean, _stdev
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

    tr = parse_csv(b.rec_csv())
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
    for power in POWERS:
        vs = [rates[(power, DURATIONS_MS[0], p, s)] for p in range(POD_COUNT) for s in (1, -1)]
        vs = [v for v in vs if v == v]
        if not vs:
            continue
        print(f"  power {power:.3f}, {DURATIONS_MS[0]:.0f} ms: travel "
              f"{min(vs):.2f}-{max(vs):.2f} deg (mean {_mean(vs):.2f}, sd {_stdev(vs):.2f})")
        print(f"      -> tolerance must exceed {max(vs):.2f} deg or the pod hunts")


if __name__ == "__main__":
    main()
