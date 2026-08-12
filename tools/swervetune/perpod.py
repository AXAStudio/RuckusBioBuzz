"""Re-score archived runs per pod instead of pooled.

Pooling four pods hides the thing that matters most. If one pod is mechanically different, every
worst-case column reports that pod and every mean reports the other three, which reads as a
frontier when it is actually one outlier.

    python perpod.py gate-90        # section 7 columns, per pod
    python perpod.py --voltage      # trial order and battery, to check for confounding
"""

from __future__ import annotations

import glob
import gzip
import json
import math
import os
import sys

from swervebench import parse_csv, score_step, _mean, _stdev, TRIAL_LOG, RUN_DIR

POD_COUNT = 4


def load(pattern: str) -> list[dict]:
    out = []
    for f in sorted(glob.glob(os.path.join(RUN_DIR, f"*{pattern}*.csv.gz"))):
        tr = parse_csv(gzip.open(f, "rt", encoding="utf-8").read())
        r = score_step(tr)
        r["file"] = os.path.basename(f)
        out.append(r)
    return out


def col(pods, key):
    return [p[key] for p in pods
            if key in p and p[key] is not None
            and not (isinstance(p[key], float) and math.isnan(p[key]))]


def table(pattern: str, settle_key: str = "settle_1_0") -> None:
    runs = load(pattern)
    if not runs:
        print(f"no archived runs matching '{pattern}'")
        return

    by_pod: dict[int, list[dict]] = {i: [] for i in range(POD_COUNT)}
    for r in runs:
        for p in r["pods"]:
            if p.get("ok"):
                by_pod[p["pod"]].append(p)

    volts = _mean([r["voltage_mean"] for r in runs])
    n = sum(len(v) for v in by_pod.values())
    print(f"{pattern}: {len(runs)} runs, {n} pod-runs, {volts:.2f} V\n")

    print(f"  {'pod':>3} {'n':>3} {'settle mean':>12} {'settled':>8} {'over max':>9} "
          f"{'|ss| mean':>10} {'|ss| max':>9} {'ring max':>9} {'pp max':>8} {'rest max':>9}")
    for i in range(POD_COUNT):
        pods = by_pod[i]
        if not pods:
            continue
        s = col(pods, settle_key)
        ss = [abs(x) for x in col(pods, "steady_state_deg")]
        print(
            f"  {i:>3} {len(pods):>3} "
            f"{(_mean(s) if s else float('nan')):>12.3f} "
            f"{100 * len(s) / len(pods):>7.0f}% "
            f"{max(col(pods, 'overshoot_pct')):>9.1f} "
            f"{_mean(ss):>10.2f} {max(ss):>9.2f} "
            f"{max(col(pods, 'rings')):>9.0f} "
            f"{max(col(pods, 'post_settle_pp_deg')):>8.2f} "
            f"{max(col(pods, 'rest_power_rms')):>9.4f}"
        )

    # Criterion 10 both ways: with every pod, and with the worst one set aside.
    means = {}
    for i in range(POD_COUNT):
        s = col(by_pod[i], settle_key)
        if s:
            means[i] = _mean(s)
    if len(means) > 1:
        spread = 100 * (max(means.values()) - min(means.values())) / _mean(list(means.values()))
        worst = max(means, key=lambda k: means[k])
        rest = {k: v for k, v in means.items() if k != worst}
        spread_rest = (
            100 * (max(rest.values()) - min(rest.values())) / _mean(list(rest.values()))
            if len(rest) > 1 else float("nan")
        )
        print(f"\n  criterion 10 settle spread: all four {spread:.0f}%, "
              f"excluding pod {worst} {spread_rest:.0f}%")


def voltage_order() -> None:
    """Trial order against battery, to see whether any comparison is confounded by pack drift."""
    if not os.path.exists(TRIAL_LOG):
        print("no trial log")
        return
    rows = []
    for line in open(TRIAL_LOG, encoding="utf-8"):
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if r.get("kind") != "step":
            continue
        cfg = (r.get("notes") or {}).get("config") or r.get("label", "")
        rows.append((r.get("timestamp", ""), cfg, r.get("voltage_mean")))

    groups: dict[str, list] = {}
    for i, (ts, cfg, v) in enumerate(rows):
        groups.setdefault(cfg, []).append((i, v))

    print(f"{'config':<34} {'first':>6} {'last':>6} {'V mean':>8} {'V range':>16}")
    for cfg, items in groups.items():
        vs = [v for _, v in items if v]
        if not vs:
            continue
        idx = [i for i, _ in items]
        print(f"{cfg[:34]:<34} {min(idx):>6} {max(idx):>6} {_mean(vs):>8.2f} "
              f"{min(vs):>7.2f}-{max(vs):<7.2f}")


if __name__ == "__main__":
    if "--voltage" in sys.argv:
        voltage_order()
    else:
        pattern = sys.argv[1] if len(sys.argv) > 1 else "gate-90"
        key = "settle_0_5" if "15" in pattern else "settle_1_0"
        table(pattern, key)
