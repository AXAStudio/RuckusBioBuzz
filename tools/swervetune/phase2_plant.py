"""Phase 2: measure the pod as a plant, before any gain is touched.

Everything here is open loop. The point is to find out what the hardware can and cannot do, so that
later on a criterion that cannot be met is recognised as a hardware limit rather than blamed on the
controller - and so that the deadband compensation in Phase 3 has real numbers behind it.

    python phase2_plant.py breakaway
    python phase2_plant.py slew
    python phase2_plant.py all
"""

from __future__ import annotations

import math
import sys
import time

from swervebench import Bench, parse_csv, _mean, _stdev

POD_COUNT = 4

# Rest-to-motion threshold. The measured encoder noise floor is sigma = 0.05 deg, so 0.3 deg is
# 6 sigma - motion this large is not the ADC.
MOVE_THRESHOLD_DEG = 0.30

# Geometric-ish ladder, fine where breakaway is expected and coarse above it.
STAIRCASE_POWERS = [
    0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040,
    0.050, 0.060, 0.070, 0.085, 0.100, 0.120, 0.150,
]
DWELL_S = 0.45
GAP_S = 0.40


def unwrap(deg: list[float]) -> list[float]:
    """Removes 360 degree jumps so angular travel can be summed across the encoder wrap."""
    out, acc = [], 0.0
    for k, d in enumerate(deg):
        if k > 0:
            step = d - deg[k - 1]
            if step > 180:
                acc -= 360
            elif step < -180:
                acc += 360
        out.append(d + acc)
    return out


def segments(t: list[float], pwr: list[float]) -> list[dict]:
    """Splits a recording into runs of constant non-zero commanded power."""
    segs, cur = [], None
    for k, p in enumerate(pwr):
        if p is None or (isinstance(p, float) and math.isnan(p)):
            p = 0.0
        if abs(p) < 1e-9:
            if cur:
                segs.append(cur)
                cur = None
            continue
        if cur and abs(p - cur["power"]) < 1e-9:
            cur["end"] = k
        else:
            if cur:
                segs.append(cur)
            cur = {"power": p, "start": k, "end": k}
    if cur:
        segs.append(cur)
    return segs


def staircase(b: Bench, pod: int, sign: int) -> list[dict]:
    """Applies each power from rest and reports how far and how fast the pod moved."""
    powers = [sign * p for p in STAIRCASE_POWERS]
    b.cmd("select", pod=pod)
    b.cmd("recStart", label=f"stair-p{pod}-{'pos' if sign > 0 else 'neg'}")
    time.sleep(0.25)
    for p in powers:
        b.cmd("rawServo", pow=p, sec=DWELL_S, pod=pod)
        time.sleep(DWELL_S + GAP_S)
    b.cmd("recStop")
    time.sleep(0.25)

    tr = parse_csv(b.rec_csv())
    t = tr["t"]
    wheel = unwrap(tr[f"p{pod}_wheel"])
    pwr = tr[f"p{pod}_pwr"]

    rows = []
    for seg in segments(t, pwr):
        a, z = seg["start"], seg["end"]
        if z - a < 2:
            continue
        # Baseline from the sample before power was applied, so the reading is from a pod at rest.
        base = wheel[max(0, a - 1)]
        travel = wheel[z] - base
        dur = t[z] - t[a]
        # Terminal rate over the back half, which is free of the initial acceleration.
        mid = (a + z) // 2
        rate = (wheel[z] - wheel[mid]) / max(1e-6, t[z] - t[mid])
        rows.append(
            {
                "power": seg["power"],
                "travel_deg": travel,
                "duration_s": dur,
                "rate_deg_s": rate,
                "moved": abs(travel) >= MOVE_THRESHOLD_DEG,
                "sustained": abs(rate) >= 5.0,
            }
        )
    return rows


def first_true(rows: list[dict], key: str) -> float:
    """Lowest |power| whose row has ``key`` set, and stays set for every higher power."""
    ordered = sorted(rows, key=lambda r: abs(r["power"]))
    for k, r in enumerate(ordered):
        if r[key] and all(x[key] for x in ordered[k:]):
            return abs(r["power"])
    return float("nan")


def run_breakaway(b: Bench) -> None:
    print("Breakaway and deadband, from rest, both directions")
    print(f"  motion threshold {MOVE_THRESHOLD_DEG} deg, sustained threshold 5 deg/s, "
          f"dwell {DWELL_S}s\n")
    summary = []
    for pod in range(POD_COUNT):
        row = {"pod": pod}
        for sign, name in ((1, "pos"), (-1, "neg")):
            rows = staircase(b, pod, sign)
            row[f"breakaway_{name}"] = first_true(rows, "moved")
            row[f"deadband_{name}"] = first_true(rows, "sustained")
            row[f"rows_{name}"] = rows
        summary.append(row)
        time.sleep(0.3)
    b.stop()

    print(f"  {'pod':>3} {'break+':>7} {'break-':>7} {'dead+':>7} {'dead-':>7} {'mean':>7}")
    for r in summary:
        vals = [r["breakaway_pos"], r["breakaway_neg"]]
        print(
            f"  {r['pod']:>3} {r['breakaway_pos']:>7.3f} {r['breakaway_neg']:>7.3f} "
            f"{r['deadband_pos']:>7.3f} {r['deadband_neg']:>7.3f} {_mean(vals):>7.3f}"
        )

    print("\n  Detail - travel (deg) per applied power, from rest:")
    hdr = "  pwr    " + "".join(f"p{p}+     p{p}-    " for p in range(POD_COUNT))
    print(hdr)
    for k, p in enumerate(STAIRCASE_POWERS):
        cells = ""
        for r in summary:
            for name in ("pos", "neg"):
                rows = r[f"rows_{name}"]
                v = rows[k]["travel_deg"] if k < len(rows) else float("nan")
                cells += f"{v:>7.1f} "
        print(f"  {p:<6.3f} {cells}")

    breaks = [r["breakaway_pos"] for r in summary] + [r["breakaway_neg"] for r in summary]
    good = [x for x in breaks if not math.isnan(x)]
    if good:
        print(f"\n  spread: min {min(good):.3f}  max {max(good):.3f}  "
              f"ratio {max(good)/max(1e-9,min(good)):.2f}x")


def run_slew(b: Bench) -> None:
    """Maximum pod rotation rate at full command - the ceiling on any achievable rise time."""
    print("\nMax slew rate at |power| = 1.0")
    print(f"  {'pod':>3} {'peak+ deg/s':>12} {'peak- deg/s':>12} {'latency+ ms':>12}")
    for pod in range(POD_COUNT):
        out = {}
        lat = float("nan")
        for sign, name in ((1, "pos"), (-1, "neg")):
            b.cmd("select", pod=pod)
            b.cmd("recStart", label=f"slew-p{pod}-{name}")
            time.sleep(0.25)
            b.cmd("rawServo", pow=sign * 1.0, sec=0.7, pod=pod)
            time.sleep(1.2)
            b.cmd("recStop")
            time.sleep(0.2)

            tr = parse_csv(b.rec_csv())
            t, pwr = tr["t"], tr[f"p{pod}_pwr"]
            wheel = unwrap(tr[f"p{pod}_wheel"])
            segs = segments(t, pwr)
            if not segs:
                out[name] = float("nan")
                continue
            a, z = segs[0]["start"], segs[0]["end"]
            rates = [
                (wheel[k] - wheel[k - 1]) / max(1e-6, t[k] - t[k - 1]) for k in range(a + 1, z + 1)
            ]
            rates.sort(key=abs)
            # 90th percentile rather than the max: one long loop next to a short one puts a
            # spurious spike in a first-difference rate.
            out[name] = rates[int(0.9 * (len(rates) - 1))] if rates else float("nan")

            if sign > 0:
                base = wheel[max(0, a - 1)]
                for k in range(a, z + 1):
                    if abs(wheel[k] - base) >= MOVE_THRESHOLD_DEG:
                        lat = (t[k] - t[a]) * 1000.0
                        break
        print(f"  {pod:>3} {out.get('pos', float('nan')):>12.0f} "
              f"{out.get('neg', float('nan')):>12.0f} {lat:>12.0f}")
    b.stop()
    print("  latency includes one loop period and one 20 ms servo PWM frame, and is quantised")
    print("  by the loop rate - treat it as an upper bound.")


def run_looprate(b: Bench) -> None:
    """Loop rate in the modes that matter, since it bounds the derivative and the relay period."""
    print("\nLoop rate by mode")
    b.stop()
    time.sleep(0.5)
    for name, setup in (
        ("IDLE", lambda: None),
        ("PID hold (4 pods)", lambda: b.cmd("pidStepAll", deg=0)),
        ("DRIVE", lambda: [b.cmd("drive", f=0.2, s=0, t=0) or time.sleep(0.15) for _ in range(6)]),
    ):
        setup()
        time.sleep(1.2)
        vals = []
        for _ in range(12):
            vals.append(b.state()["loopHz"])
            time.sleep(0.12)
            if name == "DRIVE":
                b.cmd("drive", f=0.2, s=0, t=0)
        print(f"  {name:<20} {_mean(vals):6.1f} Hz  (sigma {_stdev(vals):.1f})  "
              f"period {1000.0/_mean(vals):.1f} ms")
        b.stop()
        time.sleep(0.6)


def main() -> None:
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    b = Bench()
    print(f"battery {b.voltage():.2f} V\n")
    if what in ("breakaway", "all"):
        run_breakaway(b)
    if what in ("slew", "all"):
        run_slew(b)
    if what in ("loop", "all"):
        run_looprate(b)
    b.stop()


if __name__ == "__main__":
    main()
