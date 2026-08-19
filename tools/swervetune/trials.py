"""Repeated step trials at a given gain set, summarised across pods and repeats.

A single step tells you very little: the pod-to-pod spread and the run-to-run spread are both
comparable to the effects being chased, so every configuration is measured several times and
reported as mean plus spread.

    python trials.py --kp 0.3 --cache 0 --repeats 3 --step 90 --label cache-zero
"""

from __future__ import annotations

import argparse
import math
import time

from swervebench import Bench, format_step, _mean, _stdev

CRITERIA = {
    90: {"settle": ("settle_1_0", 0.350), "overshoot": 5.0, "ss": 1.0},
    15: {"settle": ("settle_0_5", 0.200), "overshoot": 0.0, "ss": 1.0},
}


def summarise(results: list[dict], step_deg: float) -> dict:
    """Pools every pod of every repeat into one row per metric."""
    pods = [p for r in results for p in r["pods"] if p.get("ok")]
    if not pods:
        return {}

    def col(key: str) -> list[float]:
        return [p[key] for p in pods if not (isinstance(p[key], float) and math.isnan(p[key]))]

    settle_key = CRITERIA.get(int(step_deg), CRITERIA[90])["settle"][0]
    settled = col(settle_key)

    out = {
        "n_pods": len(pods),
        "settle_key": settle_key,
        "settle_mean": _mean(settled),
        "settle_sigma": _stdev(settled),
        "settle_fraction": len(settled) / len(pods),
        "overshoot_mean": _mean(col("overshoot_pct")),
        "overshoot_max": max(col("overshoot_pct"), default=float("nan")),
        "ss_abs_mean": _mean([abs(p["steady_state_deg"]) for p in pods]),
        "ss_abs_max": max((abs(p["steady_state_deg"]) for p in pods), default=float("nan")),
        "rings_mean": _mean([float(p["rings"]) for p in pods]),
        "rings_max": max((p["rings"] for p in pods), default=0),
        "flips_total": sum(p["flip_events"] for p in pods),
        "pp_max": max(col("post_settle_pp_deg"), default=float("nan")),
        "rest_rms_mean": _mean(col("rest_power_rms")),
        "parked_fraction": sum(1 for p in pods if p.get("parked")) / len(pods),
        "pulses_post_mean": _mean([float(p.get("pulses_post", 0)) for p in pods]),
        "pulses_post_max": max((p.get("pulses_post", 0) for p in pods), default=0),
        "rise_mean": _mean(col("rise_10_90_s")),
        "loopHz_mean": _mean([r["loopHz_mean"] for r in results]),
        "loopHz_min": min(r["loopHz_min"] for r in results),
        "voltage_mean": _mean([r["voltage_mean"] for r in results]),
    }

    # Per-pod settle spread, for the pod-to-pod criterion.
    per_pod = {}
    for p in pods:
        per_pod.setdefault(p["pod"], []).append(p.get(settle_key))
    means = [
        _mean([x for x in v if x is not None and not math.isnan(x)]) for v in per_pod.values()
    ]
    means = [m for m in means if not math.isnan(m)]
    out["pod_spread_pct"] = (
        100.0 * (max(means) - min(means)) / _mean(means) if len(means) > 1 and _mean(means) else
        float("nan")
    )
    return out


def format_summary(label: str, s: dict) -> str:
    if not s:
        return f"{label}: no usable pods"
    return (
        f"{label}\n"
        f"  settle({s['settle_key']}) {_n(s['settle_mean'])}s +/-{_n(s['settle_sigma'])} "
        f"({100*s['settle_fraction']:.0f}% of pods settled)   rise {_n(s['rise_mean'])}s\n"
        f"  overshoot mean {s['overshoot_mean']:.1f}% max {s['overshoot_max']:.1f}%   "
        f"|ss| mean {s['ss_abs_mean']:.2f} max {s['ss_abs_max']:.2f} deg\n"
        f"  rings mean {s['rings_mean']:.2f} max {s['rings_max']}   flips {s['flips_total']}   "
        f"pp max {s['pp_max']:.2f} deg   rest RMS {s['rest_rms_mean']:.4f}\n"
        f"  parked {100*s['parked_fraction']:.0f}%   "
        f"pulses/3s {s['pulses_post_mean']:.1f} mean {s['pulses_post_max']} max\n"
        f"  loop {s['loopHz_mean']:.0f} Hz (min {s['loopHz_min']:.0f})   "
        f"V {s['voltage_mean']:.2f}   pod spread {_n(s['pod_spread_pct'])}%"
    )


def _n(v) -> str:
    return "--" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}"


def run(
    b: Bench,
    label: str,
    step: float = 90,
    repeats: int = 3,
    base: float = 0.0,
    hold: float = 5.0,
    verbose: bool = False,
    **gains,
) -> dict:
    """Applies gains, discards one settling step, then measures ``repeats`` trials."""
    if gains:
        st = b.set_pidf(**gains)
        applied = st["pods"][0]
        for k, want in gains.items():
            if want is None or k == "scope":
                continue
            got = applied.get(k)
            if got is None or abs(got - want) > 1e-6:
                raise SystemExit(f"gain {k} did not take: asked {want}, pod reports {got}")

    # Pods are rebuilt on any gain change, which constructs a fresh PIDFController whose first
    # derivative sample is computed against a zero previous error and a construction-time
    # timestamp. Throw the first step away rather than let that land in the data.
    b.step_trial(step_deg=step, base_deg=base, hold_s=2.0, label=f"{label}-discard", save=False)
    time.sleep(0.5)

    results = []
    for n in range(repeats):
        r = b.step_trial(
            step_deg=step, base_deg=base, hold_s=hold, label=f"{label}-{n+1}",
            notes={"config": label, **{k: v for k, v in gains.items() if v is not None}},
        )
        results.append(r)
        if verbose:
            print(format_step(r))
        time.sleep(0.6)

    s = summarise(results, step)
    print(format_summary(f"{label}  step={step:g}deg  n={repeats}", s))
    return s


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="trial")
    ap.add_argument("--step", type=float, default=90)
    ap.add_argument("--base", type=float, default=0.0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--hold", type=float, default=5.0)
    ap.add_argument("--kp", type=float)
    ap.add_argument("--ki", type=float)
    ap.add_argument("--kd", type=float)
    ap.add_argument("--kf", type=float)
    ap.add_argument("--cache", type=float)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    b = Bench()
    run(b, a.label, step=a.step, repeats=a.repeats, base=a.base, hold=a.hold,
        verbose=a.verbose, kp=a.kp, ki=a.ki, kd=a.kd, kf=a.kf, cache=a.cache)
    b.stop()


if __name__ == "__main__":
    main()
