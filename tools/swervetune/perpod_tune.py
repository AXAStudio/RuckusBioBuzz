"""Per-pod turn-gain tuning on the game tiles.

Every previous sweep chose ONE gain set for all four pods. This one lets each pod keep its own,
because criterion 10 (pod-to-pod settle spread <= 15%) has never been met and the pooled residual
hides per-pod structure. Every trial steps all four pods together (they are mechanically
independent in steering), so one trial yields one pod-run per pod for whatever cell each pod is
holding - four cells of data per ~9 s trial.

Conventions carried over from the 2026-08-13 retune, deliberately:
  - randomised interleaved cell order, seeded, so battery drift cannot favour a cell
  - loose run = post-settle peak-to-peak >= 2.0 deg (the measured valley of the bimodal
    distribution, per ploose.py), Wilson upper bound for zero counts
  - lexicographic selection: fewest loose runs first (compared by Wilson upper), then lowest
    |err at 3 s| mean. A good average never recovers a 25-degree wobble.
  - n >= 25 per cell before believing anything, winners confirmed at n >= 55 pooled
  - loop rate quoted as 1/mean(dt), never mean(loopHz)

Usage:
    python perpod_tune.py sweep  <tag> <n_per_cell> <cellspec> [cellspec ...]
    python perpod_tune.py assign <tag> <n_trials> <podspec> <podspec> <podspec> <podspec>
    python perpod_tune.py report <logfile ...>

cellspec:  kp=0.32,kd=0.022,ks=0.035          (applied to ALL pods for its trials)
podspec:   same syntax, one per pod 0..3      (per-pod assignment, held for all trials)

Results append to current_runs/perpod_<tag>.jsonl, one JSON object per trial, so an interrupted
sweep loses at most one trial.
"""

from __future__ import annotations

import json
import math
import os
import random
import sys
import time

from swervebench import Bench, _mean, _stdev

SEED = 20260813
LOOSE_PP_DEG = 2.0
STEP_DEG = 90.0
HOLD_S = 5.0
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs")


def wilson_upper(k: int, n: int, z: float = 1.96) -> float:
    if n == 0:
        return 1.0
    p = k / n
    denom = 1 + z * z / n
    centre = p + z * z / (2 * n)
    spread = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (centre + spread) / denom


def parse_spec(spec: str) -> dict:
    out = {}
    for part in spec.split(","):
        k, v = part.split("=")
        out[k.strip()] = float(v)
    return out


def cell_key(c: dict) -> str:
    return " ".join(f"{k}{c[k]:g}" for k in sorted(c))


def set_all(b: Bench, cell: dict) -> None:
    b.cmd("setPidf", scope="all", **{k: v for k, v in cell.items()})
    time.sleep(0.25)


def set_per_pod(b: Bench, cells: list[dict]) -> None:
    for i, cell in enumerate(cells):
        b.cmd("setPidf", scope="one", pod=i, **{k: v for k, v in cell.items()})
        time.sleep(0.1)
    time.sleep(0.25)


def one_trial(b: Bench, label: str, pod_cells: list[str]) -> dict:
    r = b.step_trial(step_deg=STEP_DEG, base_deg=0.0, hold_s=HOLD_S, label=label,
                     notes={"perpod": True, "pod_cells": pod_cells})
    rows = []
    for p in r["pods"]:
        if not p.get("ok"):
            rows.append({"pod": p["pod"], "ok": False})
            continue
        rows.append({
            "pod": p["pod"], "ok": True, "cell": pod_cells[p["pod"]],
            "e3": p.get("err_at_3s"), "ss": p.get("steady_state_abs_deg"),
            "settle2": p.get("settle_2_0"), "rings": p.get("rings"),
            "pp": p.get("post_settle_pp_deg"), "over": p.get("overshoot_pct"),
            "rise": p.get("rise_10_90_s"), "flips": p.get("flip_events"),
            "rest": p.get("rest_power_rms"),
            "loose": bool(p.get("post_settle_pp_deg", 0) >= LOOSE_PP_DEG),
        })
    return {
        "t": time.strftime("%H:%M:%S"), "label": label,
        "volts": r.get("voltage_mean"), "hz": r.get("loop_hz_true"),
        "trace": r.get("trace_file"), "rows": rows,
    }


def append_log(tag: str, obj: dict) -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"perpod_{tag}.jsonl"), "a", encoding="utf-8") as fh:
        fh.write(json.dumps(obj) + "\n")


def sweep(tag: str, n_per_cell: int, cells: list[dict]) -> None:
    b = Bench()
    order = []
    for c in cells:
        order += [c] * n_per_cell
    random.Random(SEED).shuffle(order)

    print(f"{tag}: {len(cells)} cells x {n_per_cell} trials, seed {SEED}, "
          f"loose >= {LOOSE_PP_DEG} deg p-p, V {b.voltage():.2f}")
    b.step_trial(step_deg=STEP_DEG, hold_s=2.0, label=f"{tag}-discard", save=False)

    for n, cell in enumerate(order, 1):
        set_all(b, cell)
        key = cell_key(cell)
        rec = one_trial(b, f"{tag}-{n:03d}", [key] * 4)
        append_log(tag, rec)
        worst = max((row.get("pp") or 0) for row in rec["rows"] if row.get("ok"))
        print(f"[{n:3d}/{len(order)}] {key:<28} V {rec['volts']:.2f} "
              f"hz {rec['hz']:.0f} worst-pp {worst:5.2f}", flush=True)
        time.sleep(0.3)
    b.stop()
    report([os.path.join(OUT_DIR, f"perpod_{tag}.jsonl")])


def assign(tag: str, n_trials: int, pods: list[dict]) -> None:
    b = Bench()
    set_per_pod(b, pods)
    keys = [cell_key(c) for c in pods]
    print(f"{tag}: per-pod assignment, {n_trials} trials, V {b.voltage():.2f}")
    for i, k in enumerate(keys):
        print(f"  pod {i}: {k}")
    b.step_trial(step_deg=STEP_DEG, hold_s=2.0, label=f"{tag}-discard", save=False)

    for n in range(1, n_trials + 1):
        rec = one_trial(b, f"{tag}-{n:03d}", keys)
        append_log(tag, rec)
        worst = max((row.get("pp") or 0) for row in rec["rows"] if row.get("ok"))
        print(f"[{n:3d}/{n_trials}] V {rec['volts']:.2f} hz {rec['hz']:.0f} "
              f"worst-pp {worst:5.2f}", flush=True)
        time.sleep(0.3)
    b.stop()
    report([os.path.join(OUT_DIR, f"perpod_{tag}.jsonl")])


def report(paths: list[str]) -> None:
    by = {}  # (pod, cell) -> rows
    volts = []
    for path in paths:
        for line in open(path, encoding="utf-8"):
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            volts.append(rec.get("volts"))
            for row in rec["rows"]:
                if row.get("ok"):
                    by.setdefault((row["pod"], row["cell"]), []).append(row)

    volts = [v for v in volts if v]
    print(f"\n{len(volts)} trials, V {min(volts):.2f}-{max(volts):.2f}" if volts else "")
    print(f"  {'pod':>3} {'cell':<28} {'n':>3} {'loose':>6} {'P<=':>5} {'|e3|':>6} "
          f"{'e3max':>6} {'set2.0':>7} {'set%':>5} {'ring':>5} {'ppmax':>6} {'rise':>6}")
    for (pod, cell) in sorted(by):
        rows = by[(pod, cell)]
        e3 = [abs(r["e3"]) for r in rows if r.get("e3") == r.get("e3") and r.get("e3") is not None]
        s2 = [r["settle2"] for r in rows
              if r.get("settle2") is not None and r["settle2"] == r["settle2"]]
        pp = [r["pp"] for r in rows if r.get("pp") == r.get("pp")]
        rise = [r["rise"] for r in rows if r.get("rise") is not None and r["rise"] == r["rise"]]
        loose = sum(1 for r in rows if r["loose"])
        print(f"  {pod:>3} {cell:<28} {len(rows):>3} "
              f"{loose:>4}/{len(rows):<3} {100*wilson_upper(loose, len(rows)):>4.0f}% "
              f"{_mean(e3):>6.2f} {max(e3) if e3 else float('nan'):>6.2f} "
              f"{_mean(s2):>7.3f} {100*len(s2)/len(rows):>4.0f}% "
              f"{_mean([r['rings'] for r in rows]):>5.2f} "
              f"{max(pp) if pp else float('nan'):>6.2f} {_mean(rise):>6.3f}")

    # Lexicographic pick per pod across everything loaded.
    print("\n  per-pod lexicographic pick (fewest loose by Wilson upper, then lowest |e3| mean):")
    pods = sorted({p for (p, _) in by})
    for pod in pods:
        cand = []
        for (p, cell), rows in by.items():
            if p != pod or len(rows) < 15:
                continue
            loose = sum(1 for r in rows if r["loose"])
            e3 = [abs(r["e3"]) for r in rows
                  if r.get("e3") == r.get("e3") and r.get("e3") is not None]
            cand.append((wilson_upper(loose, len(rows)), _mean(e3), cell, loose, len(rows)))
        if not cand:
            continue
        cand.sort()
        up, e3m, cell, loose, n = cand[0]
        print(f"    pod {pod}: {cell}  ({loose}/{n} loose, P<={100*up:.0f}%, |e3| {e3m:.2f})")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    if mode == "sweep":
        sweep(sys.argv[2], int(sys.argv[3]), [parse_spec(s) for s in sys.argv[4:]])
    elif mode == "assign":
        assign(sys.argv[2], int(sys.argv[3]), [parse_spec(s) for s in sys.argv[4:8]])
    else:
        report(sys.argv[2:])
