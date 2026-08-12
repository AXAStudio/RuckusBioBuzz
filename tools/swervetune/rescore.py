"""Re-score every archived trial against the corrected criteria, grouped by configuration.

Criterion 1 is now time to enter and stay within +/-2.0 deg, and criterion 5 is the residual
sampled at a fixed t = 3 s. The old pair both keyed on 1.0 deg, which made settle time
hypersensitive precisely when the residual sat near 1 deg - the same gains gave 0.455 s in one
block and 1.254 s in the next.

Also reports P(loose), the fraction of pod-runs landing in the wide mode of the bimodal
peak-to-peak distribution. That is the selection variable: a configuration that never draws the
loose mode is worth more than one with a better blended score that sometimes does.

    python rescore.py            # every configuration
    python rescore.py --clean    # only those with zero loose runs
"""

from __future__ import annotations

import glob
import gzip
import math
import os
import re
import sys

from swervebench import parse_csv, score_step, _mean, _stdev, RUN_DIR

# Squarely inside the empty valley between the tight mode (0.23 +/- 0.20 deg) and the loose one
# (25.65 +/- 12.44 deg).
LOOSE_PP_DEG = 2.0

SKIP = ("backlash", "deadzone", "delay", "creep", "stair", "noise", "pulsecal",
        "discard", "phase1", "range-")


def config_of(name: str) -> str:
    """Filename minus timestamp and repeat suffix."""
    s = re.sub(r"^\d{8}-\d{6}_", "", name).replace(".csv.gz", "")
    return re.sub(r"-\d+$", "", s)


def load() -> dict[str, list[dict]]:
    groups: dict[str, list[dict]] = {}
    for f in sorted(glob.glob(os.path.join(RUN_DIR, "*.csv.gz"))):
        name = os.path.basename(f)
        if any(x in name for x in SKIP):
            continue
        try:
            r = score_step(parse_csv(gzip.open(f, "rt", encoding="utf-8").read()))
        except Exception:
            continue
        cfg = config_of(name)
        for p in r["pods"]:
            if p.get("ok"):
                p["voltage"] = r.get("voltage_mean")
                groups.setdefault(cfg, []).append(p)
    return groups


def col(pods, key):
    return [p[key] for p in pods
            if key in p and p[key] is not None
            and not (isinstance(p[key], float) and math.isnan(p[key]))]


def main() -> None:
    clean_only = "--clean" in sys.argv
    groups = load()
    if not groups:
        print("no archived step trials")
        return

    rows = []
    for cfg, pods in groups.items():
        pp = col(pods, "post_settle_pp_deg")
        if not pp:
            continue
        loose = sum(1 for v in pp if v >= LOOSE_PP_DEG)
        s2 = col(pods, "settle_2_0")
        r3 = [abs(x) for x in col(pods, "err_at_3s")]
        rows.append({
            "cfg": cfg,
            "n": len(pods),
            "p_loose": loose / len(pods),
            "loose": loose,
            "settle2_mean": _mean(s2) if s2 else float("nan"),
            "settle2_frac": len(s2) / len(pods),
            "r3_mean": _mean(r3) if r3 else float("nan"),
            "r3_max": max(r3) if r3 else float("nan"),
            "rings_max": max(col(pods, "rings"), default=0),
            "pp_max": max(pp),
            "volts": _mean(col(pods, "voltage")),
        })

    if clean_only:
        rows = [r for r in rows if r["loose"] == 0]
        print(f"configurations with zero loose runs ({len(rows)} of {len(groups)})\n")
    else:
        print(f"{len(rows)} configurations, criterion 1 = settle to +/-2.0 deg, "
              f"criterion 5 = |err| at t=3 s\n")

    # Zero-loose first, then lowest residual: the selection the blended score got wrong.
    rows.sort(key=lambda r: (r["p_loose"], r["r3_mean"] if r["r3_mean"] == r["r3_mean"] else 9))

    print(f"  {'n':>4} {'loose':>7} {'set2.0':>8} {'set%':>5} {'r@3s':>6} {'r3max':>6} "
          f"{'ring':>5} {'ppmax':>7} {'V':>6}  config")
    for r in rows[:30]:
        print(
            f"  {r['n']:>4} {100*r['p_loose']:>6.0f}% "
            f"{_f(r['settle2_mean']):>8} {100*r['settle2_frac']:>4.0f}% "
            f"{_f(r['r3_mean']):>6} {_f(r['r3_max']):>6} {r['rings_max']:>5.0f} "
            f"{r['pp_max']:>7.2f} {r['volts']:>6.2f}  {r['cfg'][:44]}"
        )

    zero = [r for r in rows if r["loose"] == 0]
    if zero and not clean_only:
        print(f"\n  {len(zero)} configurations drew zero loose runs. Rule of three: with n=16 that"
              f"\n  only bounds the true rate below {100*3/16:.0f}%, so these need n ~ 60 to confirm.")
        best = min(zero, key=lambda r: r["r3_mean"] if r["r3_mean"] == r["r3_mean"] else 9)
        print(f"  lowest residual among them: {best['cfg']} at {_f(best['r3_mean'])} deg "
              f"(n={best['n']})")


def _f(v) -> str:
    return "  --  " if v is None or (isinstance(v, float) and v != v) else f"{v:.3f}"


if __name__ == "__main__":
    main()
