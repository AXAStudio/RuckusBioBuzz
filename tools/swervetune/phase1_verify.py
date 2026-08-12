"""Phase 1: confirm the calibration is still valid, without overwriting it.

The dashboard's own ``wireScan`` and ``sweep`` routines write their results straight into the
calibration. That is right during bring-up and wrong here, for a reason that is easy to miss:
``angleOffsetRad`` is stored as a *raw* angle, and the raw angle is derived from the analog range.
Re-running ``sweep`` therefore silently invalidates all four forward zeros, and recovering them
means physically pointing every wheel forward again.

So this measures the same three things from open-loop spins and compares them against what is
stored, reporting drift instead of committing it.

    python phase1_verify.py
"""

from __future__ import annotations

import time

from swervebench import Bench, parse_csv

POD_COUNT = 4

SPIN_POWER = 0.45
SPIN_SECONDS = 1.2

# A channel counts as having responded if it moved at least this much in total, matching
# SwerveBringUp.SCAN_MIN_RESPONSE_V.
MIN_RESPONSE_V = 0.15

# Analog range drift worth reporting. 0.05 V is about 5.6 degrees of pod angle, well above the
# 1 mV read granularity and well below anything that would pass unnoticed while driving.
RANGE_DRIFT_V = 0.05


def spin_and_measure(b: Bench, pod: int) -> dict:
    """Spins one pod open loop and reports what every analog slot did while it turned."""
    b.cmd("select", pod=pod)
    b.cmd("recStart", label=f"phase1-spin-p{pod}")
    time.sleep(0.25)
    b.cmd("rawServo", pow=SPIN_POWER, sec=SPIN_SECONDS, pod=pod)
    time.sleep(SPIN_SECONDS + 0.5)
    b.cmd("recStop")
    time.sleep(0.2)

    tr = parse_csv(b.rec_csv())
    out = {"pod": pod, "channels": []}
    for i in range(POD_COUNT):
        v = [x for x in tr[f"p{i}_v"] if x == x]
        total = 0.0
        signed = 0.0
        for k in range(1, len(v)):
            d = v[k] - v[k - 1]
            if abs(d) < 1.65:  # reject the 0 <-> 3.3 V wrap, not real travel
                total += abs(d)
                signed += d
        out["channels"].append(
            {
                "slot": i,
                "total_v": total,
                "signed_v": signed,
                "min_v": min(v) if v else float("nan"),
                "max_v": max(v) if v else float("nan"),
            }
        )
    return out


def main() -> None:
    b = Bench()
    st = b.state()

    print(f"voltage {st['voltage']:.2f} V   loopHz {st['loopHz']:.0f}")
    print(f"errors: {st['errors']}")
    print(f"notes:  {st['notes']}")
    print()

    stored = {p["i"]: p for p in st["pods"]}
    results = []
    for pod in range(POD_COUNT):
        results.append(spin_and_measure(b, pod))
        time.sleep(0.3)
    b.stop()

    print("Encoder pairing - driving each pod's servo, which analog slot moved:")
    print(f"  {'driven':>6} {'slot0':>8} {'slot1':>8} {'slot2':>8} {'slot3':>8}  verdict")
    pairing_ok = True
    for r in results:
        tot = [c["total_v"] for c in r["channels"]]
        best = max(range(POD_COUNT), key=lambda i: tot[i])
        runner = sorted(tot, reverse=True)[1]
        ok = best == r["pod"] and tot[best] >= MIN_RESPONSE_V and tot[best] > 2.0 * runner
        pairing_ok &= ok
        cells = " ".join(f"{v:>8.2f}" for v in tot)
        print(f"  {r['pod']:>6} {cells}  {'OK' if ok else f'MISMATCH -> slot {best}'}")
    print()

    print("Servo direction - positive power must make the reading increase:")
    dir_ok = True
    for r in results:
        own = r["channels"][r["pod"]]
        ok = own["signed_v"] > 0
        dir_ok &= ok
        print(
            f"  pod {r['pod']}  signed {own['signed_v']:+8.2f} V  "
            f"srvRev={stored[r['pod']]['srvRev']}  {'OK' if ok else 'WRONG DIRECTION'}"
        )
    print()

    print("Analog range - measured this spin vs stored calibration:")
    range_ok = True
    for r in results:
        own = r["channels"][r["pod"]]
        s = stored[r["pod"]]
        dmin = own["min_v"] - s["minV"]
        dmax = own["max_v"] - s["maxV"]
        # Only an outward drift matters. A spin that happens to stop short of an endpoint reads a
        # narrower range than the truth, which is a sampling artefact, not drift.
        ok = dmin > -RANGE_DRIFT_V and dmax < RANGE_DRIFT_V
        range_ok &= ok
        print(
            f"  pod {r['pod']}  stored [{s['minV']:.3f}, {s['maxV']:.3f}]  "
            f"measured [{own['min_v']:.3f}, {own['max_v']:.3f}]  "
            f"delta [{dmin:+.3f}, {dmax:+.3f}]  {'OK' if ok else 'DRIFTED'}"
        )
    print()

    span_deg = [
        (r["channels"][r["pod"]]["max_v"] - r["channels"][r["pod"]]["min_v"]) for r in results
    ]
    print("Sanity: a full spin should sweep very nearly the whole stored span.")
    for r, span in zip(results, span_deg):
        s = stored[r["pod"]]
        stored_span = s["maxV"] - s["minV"]
        print(
            f"  pod {r['pod']}  swept {span:.3f} V of {stored_span:.3f} V "
            f"({100.0 * span / stored_span:.1f}%)"
        )
    print()
    print(f"PAIRING {'OK' if pairing_ok else 'FAILED'}   "
          f"DIRECTION {'OK' if dir_ok else 'FAILED'}   "
          f"RANGE {'OK' if range_ok else 'DRIFTED'}")


if __name__ == "__main__":
    main()
