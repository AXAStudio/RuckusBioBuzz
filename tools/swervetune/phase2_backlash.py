"""Phase 2: directional dead zone - the floor under steady-state error and post-settle jitter.

Approaches one heading repeatedly from below and from above and compares where the pod comes to
rest. The gap between the two means is the dead zone: everything inside it is a position the pod
can sit at while the controller has no authority left to correct it.

Two caveats worth stating plainly, because they bound what this number means:

* The Axon's analog output measures its own output shaft. Any lash between the servo and the pod
  is downstream of the sensor and therefore invisible here - and, more importantly, invisible to
  the control loop too, so no gain can correct it. Detecting that needs an external angle
  reference against the wheel itself.
* This measures stiction and lash together. At 1:1 with a CR servo there is no way to separate
  them electrically, and for tuning purposes the sum is the quantity that matters.

Run with high static-friction authority so the pod is pushed right up against the mechanical
limit; at the gentler settings used for normal operation the pod stops where the *controller* runs
out of push, which measures the tuning rather than the mechanism.

    python phase2_backlash.py
"""

from __future__ import annotations

import time

from swervebench import Bench, _mean, _stdev

POD_COUNT = 4

TARGET_DEG = 45.0
APPROACH_DEG = 20.0
REPEATS = 8

# Deliberately more authority than the 0.025-0.050 measured breakaway, tapering over half a degree,
# so the pod keeps pushing until the mechanism itself stops it.
PROBE = dict(kp=0.20, ki=0.0, kd=0.020, kf=0.0, cache=0.005, ks=0.055, ksband=0.5)

# The gentler set actually being considered for competition, measured alongside for comparison.
OPERATING = dict(kp=0.20, ki=0.0, kd=0.020, kf=0.0, cache=0.01, ks=0.035, ksband=2.0)


def approaches(b: Bench, label: str, repeats: int = REPEATS) -> dict:
    """Alternates approaches from below and above, returning settled error per pod per direction."""
    below: dict[int, list[float]] = {i: [] for i in range(POD_COUNT)}
    above: dict[int, list[float]] = {i: [] for i in range(POD_COUNT)}

    for n in range(repeats):
        r = b.step_trial(
            step_deg=+APPROACH_DEG, base_deg=TARGET_DEG - APPROACH_DEG,
            hold_s=3.0, pre_settle_s=1.2, label=f"{label}-below-{n+1}",
            notes={"kind": "backlash", "direction": "below"},
        )
        for p in r["pods"]:
            if p.get("ok"):
                below[p["pod"]].append(p["steady_state_deg"])

        r = b.step_trial(
            step_deg=-APPROACH_DEG, base_deg=TARGET_DEG + APPROACH_DEG,
            hold_s=3.0, pre_settle_s=1.2, label=f"{label}-above-{n+1}",
            notes={"kind": "backlash", "direction": "above"},
        )
        for p in r["pods"]:
            if p.get("ok"):
                above[p["pod"]].append(p["steady_state_deg"])

    return {"below": below, "above": above}


def report(name: str, data: dict) -> list[dict]:
    """Reports each pod's dead zone with the uncertainty on it.

    The uncertainty is the point. A dead zone quoted without it is worthless here: a configuration
    that limit-cycles produces settled values whose scatter is several times the gap being
    measured, and the difference of two such means is noise wearing a number's clothing.
    """
    print(f"\n{name}")
    print(f"  {'pod':>3} {'from below':>21} {'from above':>21} {'dead zone':>16}")
    print(f"  {'':>3} {'mean +/- sd (n)':>21} {'mean +/- sd (n)':>21} {'deg +/- se':>16}")
    rows = []
    for i in range(POD_COUNT):
        lo, hi = data["below"][i], data["above"][i]
        if len(lo) < 2 or len(hi) < 2:
            print(f"  {i:>3}  insufficient data")
            continue
        zone = abs(_mean(lo) - _mean(hi))
        se = ((_stdev(lo) ** 2) / len(lo) + (_stdev(hi) ** 2) / len(hi)) ** 0.5
        rows.append({"pod": i, "zone": zone, "se": se, "significant": zone > 2 * se})
        flag = "" if zone > 2 * se else "  (not significant)"
        print(
            f"  {i:>3} {_mean(lo):>10.2f} +/-{_stdev(lo):>4.2f} ({len(lo):>2}) "
            f"{_mean(hi):>10.2f} +/-{_stdev(hi):>4.2f} ({len(hi):>2}) "
            f"{zone:>7.2f} +/-{se:>5.2f}{flag}"
        )
    return rows


def main() -> None:
    b = Bench()
    print(f"battery {b.voltage():.2f} V")
    print(f"target {TARGET_DEG:g} deg, approached from +/-{APPROACH_DEG:g} deg, "
          f"{REPEATS} repeats each way")

    b.set_pidf(**OPERATING)
    b.step_trial(step_deg=APPROACH_DEG, base_deg=TARGET_DEG - APPROACH_DEG, hold_s=2.0,
                 label="deadzone-discard", save=False)
    operating = approaches(b, "deadzone-operating", repeats=REPEATS)
    b.stop()

    rows = report(
        f"Operating set (kS={OPERATING['ks']}, band={OPERATING['ksband']} deg)", operating
    )
    if not rows:
        return

    print()
    worst = max(r["zone"] for r in rows)
    print(f"  dead zone {min(r['zone'] for r in rows):.2f} to {worst:.2f} deg "
          f"(mean {_mean([r['zone'] for r in rows]):.2f})")
    print()
    print("  This is an UPPER BOUND on the mechanical dead zone, which is the useful direction:")
    print("  the controller demonstrably brings the pod to rest within this gap, so whatever lash")
    print("  and stiction the mechanism has cannot be larger than it. A pod that resolves to")
    print(f"  {min(r['zone'] for r in rows):.2f} deg does not have 1.5 deg of backlash.")
    print()
    if worst > 1.5:
        print(f"  => worst pod {worst:.2f} deg exceeds 1.5 deg: mechanical inspection warranted.")
    else:
        print(f"  => worst pod {worst:.2f} deg is under 1.5 deg, so criteria 5 and 6 are NOT")
        print("     mechanically blocked. Any remaining shortfall is tuning, not hardware.")
    for r in rows:
        if r["zone"] > 0.5:
            print(f"     pod {r['pod']}: {r['zone']:.2f} deg leaves little margin under the "
                  f"0.5 deg peak-to-peak of criterion 6.")


if __name__ == "__main__":
    main()
