"""Multi-size validation at an already-set per-pod assignment.

Assumes the pods are ALREADY holding their final per-pod gains (set via perpod_tune.py assign or
setPidf) - this script deliberately does not touch gains, so what it measures is exactly what is
about to ship. Runs the step sizes the criteria name plus the flip boundary, and reports per pod.

    python perpod_valsteps.py <tag>
"""

from __future__ import annotations

import sys
import time

from swervebench import Bench, _mean

TAG = sys.argv[1] if len(sys.argv) > 1 else "valsteps"


def collect(b: Bench, step: float, repeats: int, label: str) -> list[dict]:
    pods = []
    for n in range(repeats):
        r = b.step_trial(step_deg=step, base_deg=0.0, hold_s=4.0,
                         label=f"{TAG}-{label}-{n+1}", notes={"perpod_val": True})
        pods.extend(p for p in r["pods"] if p.get("ok"))
        time.sleep(0.3)
    return pods


def col(pods, key, pod=None):
    return [p[key] for p in pods
            if (pod is None or p["pod"] == pod)
            and key in p and p[key] is not None
            and not (isinstance(p[key], float) and p[key] != p[key])]


def main() -> None:
    b = Bench()
    st = b.state()
    print("Per-pod gains under validation:")
    for p in st["pods"]:
        print(f"  pod {p['i']} {p['label']}: kP {p['kp']} kD {p['kd']} kS {p['ks']} "
              f"band {p['ksband']}")
    print(f"battery {st['voltage']:.2f} V\n")

    b.step_trial(step_deg=90, hold_s=2.0, label=f"{TAG}-discard", save=False)
    s45 = collect(b, 45, 5, "45")
    s15 = collect(b, 15, 5, "15")
    flip = collect(b, 90, 4, "flip90")
    b.stop()

    for name, pods, skey in (("45 deg", s45, "settle_1_0"), ("15 deg", s15, "settle_0_5"),
                             ("90 deg flip-boundary", flip, "settle_2_0")):
        print(f"\n{name}:")
        print(f"  {'pod':>3} {'n':>3} {'settle':>7} {'|ss|':>6} {'ssmax':>6} {'ring':>5} "
              f"{'ppmax':>6} {'flips':>6}")
        for i in range(4):
            rows = [p for p in pods if p["pod"] == i]
            if not rows:
                continue
            s = col(rows, skey)
            ss = [abs(x) for x in col(rows, "steady_state_deg")]
            print(f"  {i:>3} {len(rows):>3} {_mean(s) if s else float('nan'):>7.3f} "
                  f"{_mean(ss):>6.2f} {max(ss):>6.2f} "
                  f"{_mean(col(rows, 'rings')):>5.1f} "
                  f"{max(col(rows, 'post_settle_pp_deg')):>6.2f} "
                  f"{sum(col(rows, 'flip_events')):>6.0f}")


if __name__ == "__main__":
    main()
