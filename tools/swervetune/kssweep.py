"""Sweep one gain against scored 90 degree steps, randomised and interleaved.

Default sweep is kS, bracketing the breakaway measured on the surface under the robot: 0.050 mean
on the tiles after lubrication, against a shipped 0.035 that cannot break stiction at all.

Randomised and interleaved because the battery drifts across a sweep of this length, and running
the arms in blocks turns that drift into a bias in favour of whichever arm ran while the pack was
full. Interleaving turns it into noise instead.

Selection follows the rule set earlier in this project and not relaxed since: filter to zero
loose runs first, then minimise residual. A setting that is quiet on average but occasionally
throws a 25 degree excursion is worse than one that is slightly worse on average and never does.

    python kssweep.py [repeats]
"""

import json
import random
import statistics as st
import sys
import time

from swervebench import Bench

# Overridable so the same harness can sweep kD at a fixed kS without a second copy of the
# randomise/interleave/score logic drifting out of step with this one.
#   python kssweep.py <repeats> <param> <v1,v2,...> [fixed_ks]
PARAM = "ks"
ARMS = [0.035, 0.045, 0.050, 0.055]
KP, KD, CACHE, BAND = 0.200, 0.022, 0.01, 2.0
STEP_DEG, BASE_DEG = 90.0, 90.0
LOOSE_PP_DEG = 5.0


def summarise(tag, pods, hz, volts):
    ok = [p for p in pods if not p.get("parked")]
    if not ok:
        return f"{tag}: no usable pod-runs"
    ss = [abs(p["steady_state_abs_deg"]) for p in ok
          if p["steady_state_abs_deg"] == p["steady_state_abs_deg"]]
    e3 = [abs(p["err_at_3s"]) for p in ok if p["err_at_3s"] == p["err_at_3s"]]
    rg = [p["rings"] for p in ok if p["rings"] == p["rings"]]
    pp = [p["post_settle_pp_deg"] for p in ok
          if p["post_settle_pp_deg"] == p["post_settle_pp_deg"]]
    loose = sum(1 for x in pp if x > LOOSE_PP_DEG)
    return {
        "tag": tag, "n": len(ok), "loose": loose,
        "ss_mean": st.mean(ss), "ss_max": max(ss),
        "e3_mean": st.mean(e3), "e3_max": max(e3),
        "rings_mean": st.mean(rg), "rings_max": max(rg),
        "pp_mean": st.mean(pp), "pp_max": max(pp),
        "hz": st.mean(hz), "v_lo": min(volts), "v_hi": max(volts),
    }


def main():
    global PARAM, ARMS, KD
    repeats = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    if len(sys.argv) > 3:
        PARAM = sys.argv[2]
        ARMS = [float(x) for x in sys.argv[3].split(",")]
    fixed_ks = float(sys.argv[4]) if len(sys.argv) > 4 else None
    b = Bench()
    v0 = b.state()["voltage"]
    print(f"{PARAM} sweep {ARMS}  kP {KP}  kD {KD}  cache {CACHE}"
          + (f"  kS fixed {fixed_ks}" if fixed_ks is not None else "") + f"   V {v0:.2f}")

    order = [k for k in ARMS for _ in range(repeats)]
    random.shuffle(order)
    print(f"{len(order)} runs, randomised\n")

    acc = {k: {"pods": [], "hz": [], "v": []} for k in ARMS}
    for n, ks in enumerate(order, 1):
        kw = dict(kp=KP, ki=0.0, kd=KD, kf=0.0, ks=fixed_ks if fixed_ks is not None else 0.035,
                  ksband=BAND, cache=CACHE, dom=False, pulsed=False, scope="all")
        kw[PARAM] = ks
        b.set_pidf(**kw)
        time.sleep(0.4)
        r = b.step_trial(step_deg=STEP_DEG, base_deg=BASE_DEG,
                         label=f"{PARAM}{ks:g}-{len(acc[ks]['pods']) // 4 + 1}",
                         notes={"experiment": f"{PARAM}_sweep_tiles", PARAM: ks,
                                "fixed_ks": fixed_ks, "surface": "tiles"})
        acc[ks]["pods"] += r["pods"]
        acc[ks]["hz"].append(r["loop_hz_true"])
        acc[ks]["v"].append(r["voltage_mean"])
        usable = [p for p in r["pods"] if not p.get("parked")]
        worst = max((p["post_settle_pp_deg"] for p in usable
                     if p["post_settle_pp_deg"] == p["post_settle_pp_deg"]), default=float("nan"))
        print(f"[{n:3d}/{len(order)}] {PARAM} {ks:.3f}  V {r['voltage_mean']:.2f}  "
              f"worst p-p {worst:5.2f}")

    rows = [summarise(f"{PARAM} {k:.3f}", acc[k]["pods"], acc[k]["hz"], acc[k]["v"])
            for k in ARMS]
    rows = [r for r in rows if isinstance(r, dict)]

    print("\n" + "=" * 86)
    print(f"{'arm':>10}{'n':>5}{'loose':>7}{'|ss| mean':>11}{'|ss| max':>10}"
          f"{'rings':>8}{'p-p mean':>10}{'p-p max':>9}{'V range':>14}")
    for r in rows:
        print(f"{r['tag']:>10}{r['n']:>5}{r['loose']:>7}{r['ss_mean']:>11.2f}{r['ss_max']:>10.2f}"
              f"{r['rings_mean']:>8.2f}{r['pp_mean']:>10.2f}{r['pp_max']:>9.2f}"
              f"{r['v_lo']:>7.2f}-{r['v_hi']:.2f}")

    clean = [r for r in rows if r["loose"] == 0]
    print()
    if clean:
        best = min(clean, key=lambda r: r["ss_mean"])
        print(f"zero-loose arms: {[r['tag'] for r in clean]}")
        print(f"=> lowest residual among them: {best['tag']} "
              f"at {best['ss_mean']:.2f} deg mean, {best['ss_max']:.2f} max")
    else:
        best = min(rows, key=lambda r: r["loose"])
        print(f"NO arm was clean of loose runs. Fewest: {best['tag']} "
              f"({best['loose']}/{best['n']}) - do not treat this as a pass.")

    with open(f"current_runs/{PARAM}_sweep_tiles.json", "w") as fh:
        json.dump(rows, fh, indent=1)
    print(f"\nV {v0:.2f} -> {b.state()['voltage']:.2f}")


if __name__ == "__main__":
    raise SystemExit(main())
