"""Confirm the zero-loose configurations at n ~ 60, then pick on residual within that set.

Selection here is lexicographic, not blended. A configuration that never draws the loose mode is
worth more than one with a better average that sometimes does, because the loose mode is 25 deg of
peak-to-peak and no amount of good average recovers a run that does it. So: filter to P(loose) = 0
first, minimise residual second.

n = 16 only bounds a zero count below about 19% by the rule of three, which is far too loose to
choose on. 60 pod-runs brings that to about 5%.

    python ploose.py
"""

from __future__ import annotations

import math
import random
import time

from swervebench import Bench, _mean, _stdev

LOOSE_PP_DEG = 2.0
TRIALS_PER_CONFIG = 15          # 15 trials x 4 pods = 60 pod-runs
STEP_DEG = 90
SEED = 31337

BASE = dict(ki=0.0, kf=0.0, cache=0.01, dom=False, pulsed=False)

# The four best zero-loose configurations by residual at t=3 s, from re-scoring 85 archived
# configurations. All at kP 0.20, which is where the residual lives; they differ in damping and in
# how sharply the static-friction term tapers.
CONFIGS = {
    "kD.020 b2.0": dict(kp=0.20, kd=0.020, ks=0.035, ksband=2.0),
    "kD.014 b1.2": dict(kp=0.20, kd=0.014, ks=0.035, ksband=1.2),
    "kD.010 b2.0": dict(kp=0.20, kd=0.010, ks=0.035, ksband=2.0),
    "kD.022 b2.0": dict(kp=0.20, kd=0.022, ks=0.035, ksband=2.0),
}


def wilson_upper(k: int, n: int, z: float = 1.96) -> float:
    """Upper bound on a rate from k events in n trials. Works when k is 0, unlike the normal one."""
    if n == 0:
        return 1.0
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    m = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c + m) / d


def main() -> None:
    b = Bench()
    v0 = b.voltage()
    plan = [(name, i) for name in CONFIGS for i in range(TRIALS_PER_CONFIG)]
    random.Random(SEED).shuffle(plan)
    print(f"P(loose) confirmation: {len(CONFIGS)} configs x {TRIALS_PER_CONFIG} trials, "
          f"randomised, battery {v0:.2f} V\n")

    pods: dict[str, list[dict]] = {k: [] for k in CONFIGS}
    for n, (name, i) in enumerate(plan):
        b.set_pidf(**BASE, **CONFIGS[name])
        b.step_trial(step_deg=STEP_DEG, hold_s=1.2, pre_settle_s=1.0,
                     label="ploose-discard", save=False)
        r = b.step_trial(step_deg=STEP_DEG, hold_s=5.0,
                         label=f"ploose-{name.replace(' ', '_').replace('.', '_')}-{i+1}",
                         notes={"config": name, "order": n})
        pods[name].extend(p for p in r["pods"] if p.get("ok"))
        if n % 10 == 0:
            print(f"  ... {n+1}/{len(plan)}, {r['voltage_mean']:.2f} V")
        time.sleep(0.3)
    b.stop()
    print(f"\nbattery {v0:.2f} -> {b.voltage():.2f} V\n")

    print(f"  {'config':>12} {'n':>4} {'loose':>7} {'P<=':>6} {'r@3s':>7} {'r3max':>7} "
          f"{'set2.0':>8} {'set%':>5} {'ring':>5} {'ppmax':>7}")
    rows = []
    for name, ps in CONFIGS.items():
        p = pods[name]
        if not p:
            continue
        pp = [x["post_settle_pp_deg"] for x in p if not math.isnan(x["post_settle_pp_deg"])]
        loose = sum(1 for v in pp if v >= LOOSE_PP_DEG)
        r3 = [abs(x["err_at_3s"]) for x in p if not math.isnan(x.get("err_at_3s", float("nan")))]
        s2 = [x["settle_2_0"] for x in p
              if x.get("settle_2_0") is not None and not math.isnan(x["settle_2_0"])]
        row = {
            "name": name, "n": len(p), "loose": loose,
            "upper": wilson_upper(loose, len(p)),
            "r3": _mean(r3), "r3max": max(r3) if r3 else float("nan"),
            "s2": _mean(s2) if s2 else float("nan"), "s2frac": len(s2) / len(p),
            "ring": max(x["rings"] for x in p), "ppmax": max(pp) if pp else float("nan"),
        }
        rows.append(row)
        print(f"  {name:>12} {row['n']:>4} {loose:>7} {100*row['upper']:>5.0f}% "
              f"{row['r3']:>7.2f} {row['r3max']:>7.2f} {_f(row['s2']):>8} "
              f"{100*row['s2frac']:>4.0f}% {row['ring']:>5.0f} {row['ppmax']:>7.2f}")

    clean = [r for r in rows if r["loose"] == 0]
    print()
    if not clean:
        print("  No configuration held zero loose runs at n=60. The bimodality is not eliminated")
        print("  by gains alone within this set; lowest rate was "
              f"{min(rows, key=lambda r: r['loose'])['name']} at "
              f"{min(r['loose'] for r in rows)}/{rows[0]['n']}.")
        return

    best = min(clean, key=lambda r: r["r3"])
    print(f"  {len(clean)} of {len(rows)} held zero loose runs at n~60 "
          f"(true rate now bounded below {100*best['upper']:.0f}%).")
    print(f"  Lowest residual among them: {best['name']} at {best['r3']:.2f} deg mean, "
          f"{best['r3max']:.2f} max.")
    if best["r3max"] <= 1.5:
        print("  => under 1.5 deg worst case with zero loose runs. The residual wall was a")
        print("     selection artefact: the blended score was picking configurations that")
        print("     sometimes draw the loose mode.")
    else:
        print(f"  => mean is good but worst case is {best['r3max']:.2f} deg, so the residual")
        print("     limit survives selection on P(loose).")


def _f(v) -> str:
    return "  --  " if v is None or (isinstance(v, float) and v != v) else f"{v:.3f}"


if __name__ == "__main__":
    main()
