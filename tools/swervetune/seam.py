"""Where does the positional travel window's seam actually fall, and is that acceptable?

The seam is the window edge, and crossing it forces a 180 degree traverse of roughly 240 ms plus
settling. Put it near a heading the pods dwell at and that traverse fires while parked.

The dwell set, mod 180 degrees in the tool's wheel frame (forward = 90):

    forward   90.00     X-lock RF/LB  133.51     strafe  0.00/180     X-lock LF/RB  46.49

X-locks are atan2(146.42, +/-154.24) = +/-43.51 degrees off the chassis diagonal, so the gaps are
not equal: forward-to-X-lock is 43.51 and strafe-to-X-lock is 46.49. Maximum achievable clearance
is therefore 23.25 degrees, at the two strafe-to-X-lock midpoints only.

Run after the two-point endpoint calibration to find out whether the band landed somewhere usable,
or whether the horn needs re-clocking. Re-clocking is a mechanical disturbance and breaks the
within-pod A/B, so if it is needed it must happen BEFORE pod 0's CR baseline, not between.

    python seam.py --raw0 12.5 --raw1 212.5 --offset 338.2
    python seam.py --pod 0                       # read the live calibration off the robot
"""

from __future__ import annotations

import argparse

# Wheel headings the pods rest at, mod 180, tool frame.
DWELL = {
    "forward": 90.00,
    "X-lock RF/LB": 133.51,
    "strafe": 0.00,
    "X-lock LF/RB": 46.49,
}

# Below this, a seam is close enough to a dwell heading to risk traversing while parked.
TOO_CLOSE_DEG = 10.0

# Axon spline granularity. One tooth is the finest re-clocking step available.
SPLINE_TEETH = 25
TOOTH_DEG = 360.0 / SPLINE_TEETH


def wheel_from_raw(raw_deg: float, offset_deg: float, encoder_reversed: bool = False) -> float:
    """Inverse of the tool's raw-encoder-to-wheel-heading mapping, reduced mod 180.

    Mod 180 because a pod treats a heading and that heading plus 180 as the same demand, so that
    is the space the dwell set and the seam both live in.
    """
    zeroed = raw_deg - offset_deg
    wheel = (90.0 - zeroed) if not encoder_reversed else (zeroed - 90.0)
    return wheel % 180.0


def sep(a: float, b: float) -> float:
    """Separation on a 180 degree circle."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def report(seam_wheel: float, name: str) -> float:
    print(f"\n  {name} at wheel {seam_wheel:.2f} deg "
          f"(forward + {(seam_wheel - 90.0) % 180.0:.2f} deg)")
    worst = 180.0
    worst_at = ""
    for label, deg in DWELL.items():
        d = sep(seam_wheel, deg)
        flag = "  <-- TOO CLOSE" if d < TOO_CLOSE_DEG else ""
        print(f"      to {label:<14} {d:6.2f} deg{flag}")
        if d < worst:
            worst, worst_at = d, label
    print(f"      minimum clearance {worst:.2f} deg, to {worst_at}")
    return worst


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw0", type=float, help="raw encoder deg at servo position 0.0")
    ap.add_argument("--raw1", type=float, help="raw encoder deg at servo position 1.0")
    ap.add_argument("--offset", type=float, help="the pod's angleOffsetRad, in degrees")
    ap.add_argument("--reversed", action="store_true", help="encoderReversed")
    ap.add_argument("--pod", type=int, help="read raw0/raw1/offset live from the robot instead")
    a = ap.parse_args()

    raw0, raw1, offset, rev = a.raw0, a.raw1, a.offset, a.reversed
    if a.pod is not None:
        from swervebench import Bench
        p = Bench().state()["pods"][a.pod]
        raw0, raw1 = p["raw0"], p["raw1"]
        offset, rev = p["offsetDeg"], p["encRev"]
        print(f"pod {a.pod}: raw0={raw0:.2f} raw1={raw1:.2f} offset={offset:.2f} reversed={rev}")
    if raw0 is None or raw1 is None or offset is None:
        ap.error("need --raw0, --raw1 and --offset, or --pod")

    span = abs(raw1 - raw0)
    print(f"\ntravel {span:.1f} deg of encoder, i.e. {span:.1f} deg of pod "
          f"({'ok' if span >= 190 else 'SHORT - needs >= 190 for 180 of headings plus overlap'})")

    # Both ends are the same seam mod 180, but print both: they should agree, and disagreeing means
    # the travel is not what the programmer was told to set.
    w0 = wheel_from_raw(raw0, offset, rev)
    w1 = wheel_from_raw(raw1, offset, rev)
    c0 = report(w0, "seam (position 0 end)")
    c1 = report(w1, "seam (position 1 end)")

    print(f"\n  the two ends differ by {sep(w0, w1):.2f} deg in heading space "
          f"(expect {span - 180.0:.1f}, the overlap)")

    worst = min(c0, c1)
    print()
    if worst >= 20.0:
        print(f"  VERDICT: {worst:.2f} deg minimum clearance. Good - near the 23.25 deg maximum.")
        print("  No re-clocking needed.")
    elif worst >= TOO_CLOSE_DEG:
        print(f"  VERDICT: {worst:.2f} deg minimum clearance. Usable but not ideal.")
        print(f"  One spline tooth is {TOOTH_DEG:.1f} deg; check whether moving one tooth improves it.")
    else:
        print(f"  VERDICT: {worst:.2f} deg minimum clearance - too close, the seam will traverse")
        print("  while the pods are parked. Re-clock the horn.")
        best = None
        for teeth in range(-SPLINE_TEETH // 2, SPLINE_TEETH // 2 + 1):
            shifted = min(
                min(sep((w0 + teeth * TOOTH_DEG) % 180.0, d) for d in DWELL.values()),
                min(sep((w1 + teeth * TOOTH_DEG) % 180.0, d) for d in DWELL.values()),
            )
            if best is None or shifted > best[1]:
                best = (teeth, shifted)
        print(f"  Best available: {best[0]:+d} teeth ({best[0] * TOOTH_DEG:+.1f} deg) giving "
              f"{best[1]:.2f} deg clearance.")
        print("\n  Re-clocking is a mechanical disturbance. Do it BEFORE pod 0's CR baseline, then")
        print("  re-zero the pod, then baseline, then reflash - so the before/after still isolates")
        print("  the mode change rather than including a horn that moved.")


if __name__ == "__main__":
    main()
