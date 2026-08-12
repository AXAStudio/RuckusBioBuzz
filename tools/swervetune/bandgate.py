"""Hard gate: does the second flash place the travel band where the first one did?

The A/B flashes pod 0 twice - Servo Mode to find the band and check the seam, back to CR for the
baseline, then Servo Mode again for the test. That ordering is what keeps any re-clocking away from
the middle of the comparison, but it only works if the second flash reproduces the first. If the
servo's internal zero lands somewhere else, the calibration from the first flash is wrong for the
second, and every number downstream is measured against a mapping that does not describe the
hardware. Silent, and it would look like a result.

    python bandgate.py --save 0      # after the first calibration
    python bandgate.py --check 0     # after the second; exits non-zero if it moved
"""

from __future__ import annotations

import json
import os
import sys

from swervebench import Bench

REF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "band_reference.json")

# An endpoint that has shifted by this much moves the seam by the same amount, against 18.24 deg of
# clearance. Two degrees spends about a ninth of the margin, which is tolerable; more is not, and
# means the servo's internal zero is not reproducible and the whole approach needs rethinking.
ENDPOINT_TOL_DEG = 2.0

# The span is the programmed travel. It should reproduce far more tightly than placement, because
# it is a programmer setting rather than a zero reference - a span that moves means the flash did
# not take identically.
SPAN_TOL_DEG = 1.0


def read(pod: int) -> dict:
    p = Bench().state()["pods"][pod]
    return {
        "pod": pod,
        "raw0": p["raw0"],
        "raw1": p["raw1"],
        "span": abs(p["raw1"] - p["raw0"]),
        "offsetDeg": p["offsetDeg"],
        "calibrated": p.get("posCalibrated"),
    }


def main() -> None:
    if len(sys.argv) < 3 or sys.argv[1] not in ("--save", "--check"):
        print(__doc__)
        sys.exit(2)
    pod = int(sys.argv[2])
    now = read(pod)

    if not now["calibrated"]:
        print(f"pod {pod} is not calibrated yet - run the endpoint calibration first")
        sys.exit(2)

    if sys.argv[1] == "--save":
        with open(REF, "w", encoding="utf-8") as fh:
            json.dump(now, fh, indent=2)
        print(f"saved pod {pod} band reference:")
        print(f"  raw0 {now['raw0']:.2f} deg, raw1 {now['raw1']:.2f} deg, "
              f"span {now['span']:.2f} deg")
        return

    if not os.path.exists(REF):
        print("no saved reference - run --save after the first calibration")
        sys.exit(2)
    with open(REF, encoding="utf-8") as fh:
        ref = json.load(fh)
    if ref["pod"] != pod:
        print(f"reference is for pod {ref['pod']}, not {pod}")
        sys.exit(2)

    d0 = now["raw0"] - ref["raw0"]
    d1 = now["raw1"] - ref["raw1"]
    dspan = now["span"] - ref["span"]

    print(f"  {'':<8} {'first':>9} {'second':>9} {'delta':>9} {'tol':>7}")
    print(f"  {'raw0':<8} {ref['raw0']:>9.2f} {now['raw0']:>9.2f} {d0:>+9.2f} "
          f"{ENDPOINT_TOL_DEG:>7.1f}")
    print(f"  {'raw1':<8} {ref['raw1']:>9.2f} {now['raw1']:>9.2f} {d1:>+9.2f} "
          f"{ENDPOINT_TOL_DEG:>7.1f}")
    print(f"  {'span':<8} {ref['span']:>9.2f} {now['span']:>9.2f} {dspan:>+9.2f} "
          f"{SPAN_TOL_DEG:>7.1f}")

    ok = (abs(d0) <= ENDPOINT_TOL_DEG and abs(d1) <= ENDPOINT_TOL_DEG
          and abs(dspan) <= SPAN_TOL_DEG)
    print()
    if ok:
        print("  PASS - the band reproduced. The first flash's seam analysis still applies and the")
        print("  A/B can proceed.")
        return

    print("  STOP - the band did not reproduce.")
    print("  The calibration from the first flash does not describe this one, so the seam analysis")
    print("  is void and any step response measured now would be scored against a wrong mapping.")
    print("  Do not run the positional trials. Re-flash and re-measure to find out whether the")
    print("  placement is random or drifting; if it will not reproduce, positional mode is not")
    print("  usable here regardless of how well it holds position.")
    sys.exit(1)


if __name__ == "__main__":
    main()
