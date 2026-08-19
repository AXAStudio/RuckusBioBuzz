"""Randomised gain grid.

Every earlier sweep ran in blocks, so any drift across the session - and the pack drifts about
0.6 V over a long block - lands as a systematic difference between configurations tested early and
late. Randomising the trial order converts that bias into noise, and logging the voltage per trial
means the drift also becomes free coverage of the low-battery criterion instead of a confound.

The cost is a gain change before every single trial, and pods are rebuilt on a gain change, which
resets the PIDF and makes the first step afterwards unrepresentative. So each trial is preceded by
a short throwaway step.

    python randgrid.py
"""

from __future__ import annotations

import random
import time

from swervebench import Bench, _mean, _stdev
from trials import summarise, format_summary

SEED = 20260812

FIXED = dict(ki=0.0, kf=0.0, ks=0.035, ksband=2.0, cache=0.01, dom=False, pulsed=False)

# Centred on the corrected lag-cancellation optimum kD = kP * tau with tau = 42 ms, which is
# 0.0059 at kP 0.14, 0.0084 at kP 0.20 and 0.0118 at kP 0.28. Every gain tested before the tau
# correction sat above this band.
KP_VALUES = (0.14, 0.20, 0.28)
KD_VALUES = (0.004, 0.007, 0.010, 0.014)
REPEATS = 4
STEP_DEG = 90


def main() -> None:
    b = Bench()
    v0 = b.voltage()

    plan = [
        (kp, kd, rep)
        for kp in KP_VALUES
        for kd in KD_VALUES
        for rep in range(REPEATS)
    ]
    random.Random(SEED).shuffle(plan)
    print(f"randomised grid: {len(plan)} trials, seed {SEED}, battery {v0:.2f} V\n")

    results: dict[tuple[float, float], list[dict]] = {}
    for n, (kp, kd, rep) in enumerate(plan):
        b.set_pidf(kp=kp, kd=kd, **FIXED)
        # Pods rebuild on a gain change; discard the first step after one.
        b.step_trial(step_deg=STEP_DEG, hold_s=1.2, pre_settle_s=1.0,
                     label="randgrid-discard", save=False)
        r = b.step_trial(
            step_deg=STEP_DEG, hold_s=5.0,
            label=f"randgrid-kp{kp:.2f}-kd{kd:.3f}-{rep+1}",
            notes={"config": f"kP={kp:.2f} kD={kd:.3f}", "order": n, "seed": SEED},
        )
        results.setdefault((kp, kd), []).append(r)
        if n % 6 == 0:
            print(f"  ... {n+1}/{len(plan)} done, {r['voltage_mean']:.2f} V")
        time.sleep(0.3)

    b.stop()
    v1 = b.voltage()
    print(f"\nbattery {v0:.2f} -> {v1:.2f} V across the block\n")

    rows = []
    for (kp, kd), rs in sorted(results.items()):
        s = summarise(rs, STEP_DEG)
        s["kp"], s["kd"] = kp, kd
        rows.append(s)

    print(f"  {'kP':>5} {'kD':>6} {'kD/kP*tau':>10} {'settle':>8} {'set%':>5} {'over%':>7} "
          f"{'|ss|':>6} {'ssmax':>6} {'ring':>5} {'ringmax':>8} {'ppmax':>7} {'V':>6}")
    for s in rows:
        # 1.0 means the PD zero sits exactly on the measured plant lag pole.
        ratio = s["kd"] / (s["kp"] * 0.042)
        print(
            f"  {s['kp']:>5.2f} {s['kd']:>6.3f} {ratio:>10.2f} "
            f"{_fmt(s['settle_mean']):>8} {100*s['settle_fraction']:>4.0f}% "
            f"{s['overshoot_mean']:>7.1f} {s['ss_abs_mean']:>6.2f} {s['ss_abs_max']:>6.2f} "
            f"{s['rings_mean']:>5.2f} {s['rings_max']:>8.0f} {s['pp_max']:>7.2f} "
            f"{s['voltage_mean']:>6.2f}"
        )

    # Rank on the criteria that are actually failing, not on a single metric.
    def score(s):
        return (
            (s["settle_mean"] if s["settle_mean"] == s["settle_mean"] else 9.0)
            + 2.0 * s["ss_abs_max"]
            + 0.15 * s["rings_max"]
            + 0.5 * s["pp_max"]
        )

    best = min(rows, key=score)
    print(f"\nbest by combined settle/residual/ring/peak-to-peak score:")
    print(format_summary(f"  kP={best['kp']:.2f} kD={best['kd']:.3f}", best))

    # Voltage is randomised against configuration, so a correlation here is drift, not gain effect.
    vs = [r["voltage_mean"] for rs in results.values() for r in rs]
    print(f"\n  voltage across trials: {min(vs):.2f}-{max(vs):.2f} V "
          f"(mean {_mean(vs):.2f}, sd {_stdev(vs):.3f})")


def _fmt(v: float) -> str:
    return "  --  " if v != v else f"{v:.3f}"


if __name__ == "__main__":
    main()
