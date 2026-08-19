"""Where do the forced 180 degree traverses fall, and is that acceptable?

A traverse costs about 260 ms with the wheel pointed the wrong way. Its *frequency* is not
something the window width can change: position tracks demand one for one, each traverse removes
exactly 180 degrees of accumulated position, and position is bounded by the window, so over a
sweep of D degrees the count is D/180 whatever the width. A jump lands exactly 180 degrees from
the edge it left, not at the far end of the window.

What the width does buy is hysteresis - the doubly covered overlap, W - 180 - which is the margin
against demand wobble near a seam forcing repeated flips. And with the greedy "nearest
representation" rule the pod jumps only when cornered, so the traverses land at the two ends of the
clamped band, at headings separated by (clamped width) mod 180.

The dwell set, mod 180 in the tool's wheel frame (forward = 90):

    forward 90.00    X-lock RF/LB 133.51    strafe 0.00/180    X-lock LF/RB 46.49

X-locks are atan2(146.42, +/-154.24) = +/-43.51 off the chassis diagonal, so the gaps are unequal:
forward-to-X-lock 43.51, strafe-to-X-lock 46.49.

    python seam.py --raw0 12.5 --raw1 282.5 --offset 338.2
    python seam.py --pod 0
"""

from __future__ import annotations

import argparse

DWELL = {
    "forward": 90.00,
    "X-lock RF/LB": 133.51,
    "strafe": 0.00,
    "X-lock LF/RB": 46.49,
}

# Below this, a traverse heading is close enough to a dwell heading to fire while parked.
TOO_CLOSE_DEG = 10.0

SPLINE_TEETH = 25
TOOTH_DEG = 360.0 / SPLINE_TEETH


def wheel_from_raw(raw_deg: float, offset_deg: float, encoder_reversed: bool = False) -> float:
    zeroed = raw_deg - offset_deg
    wheel = (zeroed - 90.0) if encoder_reversed else (90.0 - zeroed)
    return wheel % 180.0


def sep(a: float, b: float) -> float:
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def clearance(seam_wheel: float) -> tuple[float, str]:
    worst, at = 180.0, ""
    for label, deg in DWELL.items():
        d = sep(seam_wheel, deg)
        if d < worst:
            worst, at = d, label
    return worst, at


def best_placement(separation: float, step: float = 0.05) -> tuple[float, float]:
    """Scans for the placement of two traverse headings that maximises the worst clearance."""
    best_t, best_c = 0.0, -1.0
    t = 0.0
    while t < 180.0:
        c = min(clearance(t)[0], clearance((t + separation) % 180.0)[0])
        if c > best_c:
            best_c, best_t = c, t
        t += step
    return best_t, best_c


def report(seam_wheel: float, name: str) -> float:
    print(f"\n  {name} at wheel {seam_wheel:.2f} deg "
          f"(forward + {(seam_wheel - 90.0) % 180.0:.2f} deg)")
    for label, deg in DWELL.items():
        d = sep(seam_wheel, deg)
        print(f"      to {label:<14} {d:6.2f} deg{'  <-- TOO CLOSE' if d < TOO_CLOSE_DEG else ''}")
    worst, at = clearance(seam_wheel)
    print(f"      minimum clearance {worst:.2f} deg, to {at}")
    return worst


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw0", type=float, help="raw encoder deg at servo position 0.0")
    ap.add_argument("--raw1", type=float, help="raw encoder deg at servo position 1.0")
    ap.add_argument("--offset", type=float, help="the pod's angleOffsetRad, in degrees")
    ap.add_argument("--margin", type=float, default=3.0,
                    help="end-stop clamp margin, deg; traverses happen at the CLAMPED edges")
    ap.add_argument("--reversed", action="store_true")
    ap.add_argument("--pod", type=int, help="read the calibration live from the robot")
    a = ap.parse_args()

    raw0, raw1, offset, rev, margin = a.raw0, a.raw1, a.offset, a.reversed, a.margin
    if a.pod is not None:
        from swervebench import Bench
        p = Bench().state()["pods"][a.pod]
        raw0, raw1, offset, rev = p["raw0"], p["raw1"], p["offsetDeg"], p["encRev"]
        margin = p.get("clampMargin", margin)
        print(f"pod {a.pod}: raw0={raw0:.2f} raw1={raw1:.2f} offset={offset:.2f} "
              f"reversed={rev} clampMargin={margin:.2f}")
        if not p.get("posCalibrated"):
            print("\n  NOTE: posCalibrated is false, so raw0/raw1 are placeholder defaults and\n"
                  "  everything below is meaningless. Calibrate the endpoints first.")
    if raw0 is None or raw1 is None or offset is None:
        ap.error("need --raw0, --raw1 and --offset, or --pod")

    travel = abs(raw1 - raw0)
    lo, hi = min(raw0, raw1) + margin, max(raw0, raw1) - margin
    band = hi - lo
    overlap = band - 180.0

    print(f"\nprogrammed travel {travel:.1f} deg, clamped to {band:.1f} deg "
          f"({margin:.1f} deg held back each end)")
    if band < 180.0:
        print(f"  BAND TOO NARROW: {band:.1f} deg cannot cover 180 deg of headings. "
              f"Some headings are unreachable.")
        return
    print(f"  hysteresis (overlap) {overlap:.1f} deg - the margin against demand wobble near a")
    print(f"  seam forcing repeated flips. Traverse frequency is one per 180 deg of demand sweep")
    print(f"  regardless of width, so this is what the width actually buys.")

    # Greedy tracking corners the pod at the clamped ends, so that is where traverses land.
    sep_deg = band % 180.0
    w_lo = wheel_from_raw(lo, offset, rev)
    w_hi = wheel_from_raw(hi, offset, rev)
    c_lo = report(w_lo, "traverse heading (low end)")
    c_hi = report(w_hi, "traverse heading (high end)")
    worst = min(c_lo, c_hi)

    print(f"\n  the two traverse headings are {sep(w_lo, w_hi):.2f} deg apart "
          f"(expect {min(sep_deg, 180 - sep_deg):.2f}, from the clamped width)")

    best_t, best_c = best_placement(min(sep_deg, 180.0 - sep_deg))
    print(f"  best achievable for this width: {best_c:.2f} deg, with the low end at "
          f"wheel {best_t:.2f} deg")

    print()
    if worst >= 0.85 * best_c:
        print(f"  VERDICT: {worst:.2f} deg minimum clearance, against {best_c:.2f} achievable.")
        print("  Good enough - no re-clocking needed.")
    elif worst >= TOO_CLOSE_DEG:
        print(f"  VERDICT: {worst:.2f} deg, against {best_c:.2f} achievable. Usable but leaving")
        print(f"  margin on the table; one spline tooth is {TOOTH_DEG:.1f} deg.")
    else:
        print(f"  VERDICT: {worst:.2f} deg minimum clearance - a traverse will fire while the pods")
        print("  are parked. Re-clock the horn.")

    if worst < 0.85 * best_c:
        best = None
        for teeth in range(-SPLINE_TEETH // 2, SPLINE_TEETH // 2 + 1):
            shifted = min(
                clearance((w_lo + teeth * TOOTH_DEG) % 180.0)[0],
                clearance((w_hi + teeth * TOOTH_DEG) % 180.0)[0],
            )
            if best is None or shifted > best[1]:
                best = (teeth, shifted)
        print(f"  Best tooth shift: {best[0]:+d} ({best[0] * TOOTH_DEG:+.1f} deg) giving "
              f"{best[1]:.2f} deg.")
        print("\n  Re-clocking is a mechanical disturbance: do it BEFORE pod 0's CR baseline, then")
        print("  re-zero, then baseline, then reflash - so the before/after still isolates the")
        print("  mode change rather than including a horn that moved.")


if __name__ == "__main__":
    main()
