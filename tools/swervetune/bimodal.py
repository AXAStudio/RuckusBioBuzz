"""Is the per-pod table about pods, or about trials?

The per-pod summary showed values clustering hard - 0.34 six times, 0.22 twice, then a lone 12.03
and a lone 12.15 - and the "worst pod" moved with every step size. Both are signatures of a
bimodal per-trial outcome rather than a per-pod difference. If that is what it is, criterion 10
(pod-to-pod spread) is not failing, it is unmeasurable: it is reporting which pod happened to draw
the bad mode.

Pools every archived step trial regardless of pod, looks for separated modes in post-settle
peak-to-peak, then asks what distinguishes the bad-mode trials.

    python bimodal.py
"""

from __future__ import annotations

import glob
import gzip
import math
import os

from swervebench import parse_csv, score_step, _mean, _stdev, RUN_DIR

# Trials from configurations that were deliberately unstable would manufacture a second mode, so
# only pooled runs that were plausible operating points are included.
EXCLUDE = ("kD_0_055", "kD_0_070", "kD_0_110", "kP_0_28", "kS_045", "band_1_0", "kI_0_8",
           "ppow_055", "ppow_045", "PULSE", "backlash", "delay", "creep", "stair", "noise")


def gather() -> list[dict]:
    pods = []
    for f in sorted(glob.glob(os.path.join(RUN_DIR, "*.csv.gz"))):
        name = os.path.basename(f)
        if any(x in name for x in EXCLUDE):
            continue
        try:
            r = score_step(parse_csv(gzip.open(f, "rt", encoding="utf-8").read()))
        except Exception:
            continue
        for p in r["pods"]:
            if p.get("ok") and not math.isnan(p.get("post_settle_pp_deg", float("nan"))):
                p["file"] = name
                pods.append(p)
    return pods


def histogram(values: list[float], edges: list[float]) -> None:
    total = len(values)
    for lo, hi in zip(edges, edges[1:]):
        n = sum(1 for v in values if lo <= v < hi)
        bar = "#" * int(round(60 * n / max(1, total)))
        print(f"  {lo:>6.2f} - {hi:>6.2f} deg  {n:>4}  {bar}")
    n = sum(1 for v in values if v >= edges[-1])
    bar = "#" * int(round(60 * n / max(1, total)))
    print(f"  {edges[-1]:>6.2f} +          {n:>4}  {bar}")


def main() -> None:
    pods = gather()
    if not pods:
        print("no usable archived trials")
        return
    pp = [p["post_settle_pp_deg"] for p in pods]
    print(f"pooled {len(pods)} pod-runs from {len({p['file'] for p in pods})} trials\n")

    print("post-settle peak-to-peak distribution:")
    histogram(pp, [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0])

    # A gap test: sort and find the largest relative jump in the middle of the distribution.
    s = sorted(pp)
    best_gap, split = 0.0, None
    for i in range(int(0.05 * len(s)), int(0.95 * len(s))):
        if s[i] > 1e-6 and s[i + 1] / s[i] > best_gap:
            best_gap, split = s[i + 1] / s[i], 0.5 * (s[i] + s[i + 1])
    print(f"\n  largest relative gap in the interior: {best_gap:.1f}x at {split:.2f} deg")

    good = [p for p in pods if p["post_settle_pp_deg"] < split]
    bad = [p for p in pods if p["post_settle_pp_deg"] >= split]
    print(f"  tight mode: {len(good)} runs ({100*len(good)/len(pods):.0f}%), "
          f"pp {_mean([p['post_settle_pp_deg'] for p in good]):.2f} "
          f"+/-{_stdev([p['post_settle_pp_deg'] for p in good]):.2f} deg")
    print(f"  loose mode: {len(bad)} runs ({100*len(bad)/len(pods):.0f}%), "
          f"pp {_mean([p['post_settle_pp_deg'] for p in bad]):.2f} "
          f"+/-{_stdev([p['post_settle_pp_deg'] for p in bad]):.2f} deg")
    if not bad:
        return

    print("\n  Which pod draws the loose mode:")
    print(f"  {'pod':>3} {'runs':>6} {'loose':>6} {'rate':>7}")
    for i in range(4):
        tot = [p for p in pods if p["pod"] == i]
        lo = [p for p in bad if p["pod"] == i]
        if tot:
            print(f"  {i:>3} {len(tot):>6} {len(lo):>6} {100*len(lo)/len(tot):>6.0f}%")

    print("\n  What distinguishes a loose-mode run:")
    for key, label in (
        ("steady_state_deg", "final error (deg)"),
        ("overshoot_pct", "overshoot (%)"),
        ("rest_power_rms", "resting power RMS"),
        ("travel_deg", "travel (deg)"),
    ):
        g = [abs(p[key]) for p in good if key in p and not math.isnan(p[key])]
        d = [abs(p[key]) for p in bad if key in p and not math.isnan(p[key])]
        if g and d:
            print(f"    {label:<22} tight {_mean(g):>7.2f}  loose {_mean(d):>7.2f}")

    flips_g = sum(1 for p in good if p.get("flip_events", 0) > 0)
    flips_b = sum(1 for p in bad if p.get("flip_events", 0) > 0)
    print(f"    {'runs with a mid-move flip':<22} tight {100*flips_g/len(good):>6.0f}%  "
          f"loose {100*flips_b/len(bad):>6.0f}%")

    # Landing position relative to the deadband edge: with breakaway 0.025-0.060 and the gains in
    # use, the controller runs out of authority a few degrees out. If the loose mode is about
    # landing near that edge, final error should separate the two groups - which the row above
    # answers directly.
    print("\n  Interpretation: if the two modes are separated and the loose rate is similar")
    print("  across pods, the per-pod table is sampling one bimodal process, and criterion 10")
    print("  is measuring which pod drew the bad mode rather than any property of the pod.")


if __name__ == "__main__":
    main()
