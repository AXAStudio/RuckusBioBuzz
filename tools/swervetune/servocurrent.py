"""Servo current against commanded power, per pod, as a torque proxy.

Current is the only torque proxy available on this hardware. Below breakaway the servo is pushing
and the pod is not moving, so the pod's own stiction is the load and the servo is genuinely
stalled - a command-to-torque map at zero speed, with nothing to damage and no hard stop needed.

This exists because "breakaway is 0.035, so 96.5% of torque authority is unused" was wrong: 0.035
is a normalised CRServo command, which sets pulse width and therefore a speed setpoint, not a
torque fraction. Current is a real proxy; a command fraction is not.

Two things the first version of this script got wrong, both fixed here:

  - It subtracted a MEDIAN baseline from a MEAN dwell. The channel is heavy-tailed - released, it
    reads about -1 mA median while 5% of samples spike past 100 mA - so mixing the two estimators
    inflated every net figure by roughly the spike contribution.
  - It reported a peak column. Peaks are the same magnitude with the servos released, so that
    column was noise end to end.

Driving raises the spike RATE from ~8% to ~28-30% across all four pods, which is what says the
draw is real and merely sampled intermittently. That makes the mean correct and the median wrong:
the median discards exactly the samples carrying the signal.

Baselines come from the released gaps inside each pod's own staircase rather than a separate
recording, so drift and temperature are controlled for.

    python servocurrent.py <label>          e.g. "unweighted", "weighted15lb"
"""

import gzip
import json
import os
import random
import statistics as st
import sys
import time

from swervebench import Bench, parse_csv

POWERS = [round(0.00 + 0.01 * i, 3) for i in range(16)]  # 0.00 .. 0.15

# Axon MINI+ stall current from the vendor documentation: 3200 mA at 4.8 V, 3800 mA at 6.0 V.
# The REV hub's servo rail is 5 V, so this is a linear interpolation between those two points -
# an approximation, and per servo, not per rail. Everything expressed as "% of stall" below
# inherits that assumption, and also assumes current is proportional to torque, which holds for
# the motor itself but ignores the servo's own electronics.
STALL_MA = 3300.0

DWELL_S = 0.5
GAP_S = 0.4
MOVE_DEG = 1.5
SETTLE_SKIP_S = 0.12
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs")


def unwrap(a):
    out, prev, off = [], None, 0.0
    for x in a:
        if x != x:
            out.append(float("nan"))
            continue
        if prev is not None:
            d = x - prev
            if d > 180:
                off -= 360
            elif d < -180:
                off += 360
        out.append(x + off)
        prev = x
    return out


def ci(v, n=3000):
    if len(v) < 3:
        return float("nan"), float("nan")
    m = sorted(st.mean([random.choice(v) for _ in v]) for _ in range(n))
    return m[int(0.025 * n)], m[int(0.975 * n)]


def analyse(tr, pod):
    t, pwr = tr["t"], tr[f"p{pod}_pwr"]
    wheel = unwrap(tr[f"p{pod}_wheel"])
    chans = {k: tr[k] for k in ("servoMa", "batteryMa") if k in tr}

    rel = [i for i in range(len(pwr)) if pwr[i] == pwr[i] and abs(pwr[i]) < 1e-9]
    base = {k: st.mean([v[i] for i in rel if v[i] == v[i]]) for k, v in chans.items()}

    rows = []
    for target in POWERS:
        if target == 0:
            continue
        idx = [i for i in range(len(pwr)) if pwr[i] == pwr[i] and abs(pwr[i] - target) < 1e-6]
        if len(idx) < 5:
            continue
        t0 = t[idx[0]]
        seg = [i for i in idx if t[i] - t0 >= SETTLE_SKIP_S] or idx
        w = [wheel[i] for i in seg if wheel[i] == wheel[i]]
        row = {"cmd": target,
               "travel": (max(w) - min(w)) if len(w) > 1 else float("nan"),
               "n": len(seg)}
        row["moved"] = row["travel"] > MOVE_DEG
        for k, v in chans.items():
            cur = [v[i] for i in seg if v[i] == v[i]]
            lo, hi = ci(cur)
            row[k] = st.mean(cur) - base[k]
            row[k + "_lo"] = lo - base[k]
            row[k + "_hi"] = hi - base[k]
        rows.append(row)
    return base, rows


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    random.seed(3)
    os.makedirs(OUT, exist_ok=True)
    b = Bench()
    v0 = b.state()["voltage"]
    print(f"[{label}]  V {v0:.2f}   powers {POWERS[0]:.2f}..{POWERS[-1]:.2f}   dwell {DWELL_S}s")

    # rawServo is open loop, but any mode that closes the loop keeps writing to the same servos
    # every iteration and simply overwrites it. In DRIVE with zero sticks that is X-lock, which
    # silently reduced a whole staircase to a handful of stray samples and looked like "the pod
    # never moved" rather than like a mistake. Force IDLE, then verify it took.
    # "stop" is the real command; there is no "setMode" action - sending it lands in the
    # unknown-command branch and latches a phantom error into /state for the whole session.
    b.cmd("stop")
    time.sleep(0.8)
    mode = b.state().get("mode")
    if mode != "IDLE":
        print(f"refusing to run: mode is {mode}, not IDLE - the staircase would be overwritten")
        return 1

    b.cmd("setFastCurrent", value="true")
    b.cmd("setPublishHz", value=4)
    time.sleep(0.6)

    out = {"label": label, "voltage_start": v0, "stall_ma": STALL_MA, "pods": {}}
    for pod in range(4):
        r = b.raw_staircase(pod, POWERS, dwell_s=DWELL_S, gap_s=GAP_S,
                            label=f"current-{label}-p{pod}")
        tr = parse_csv(gzip.open(r["trace_file"], "rt").read())
        base, rows = analyse(tr, pod)
        out["pods"][pod] = {"baseline": base, "rows": rows, "trace": r["trace_file"]}

        has_batt = any("batteryMa" in x for x in rows)
        print(f"\n--- pod {pod} ---   released baseline: "
              + "  ".join(f"{k} {v:.0f} mA" for k, v in base.items()))
        hdr = f"{'cmd':>5}{'servo mA':>20}"
        if has_batt:
            hdr += f"{'batt mA':>20}"
        print(hdr + f"{'travel':>9}{'moved':>7}")
        for x in rows:
            line = f"{x['cmd']:5.2f}{x['servoMa']:9.0f} [{x['servoMa_lo']:4.0f},{x['servoMa_hi']:4.0f}]"
            if has_batt:
                line += f"{x['batteryMa']:9.0f} [{x['batteryMa_lo']:4.0f},{x['batteryMa_hi']:4.0f}]"
            line += f"{x['travel']:9.1f}{'  yes' if x['moved'] else '   no':>7}"
            print(line)
        b.cmd("stop")
        time.sleep(0.4)

    b.cmd("setFastCurrent", value="false")
    b.cmd("setPublishHz", value=20)
    out["voltage_end"] = b.state()["voltage"]

    print("\n" + "=" * 72)
    print(f"breakaway, and the current it drew, as % of {STALL_MA:.0f} mA stall:")
    for pod, d in out["pods"].items():
        moved = [x for x in d["rows"] if x["moved"]]
        if not moved:
            print(f"  pod {pod}: never moved up to {POWERS[-1]:.2f}")
            continue
        f = moved[0]
        sig = f["servoMa_lo"] > 0
        print(f"  pod {pod}: breakaway {f['cmd']:.2f}  "
              f"servo {f['servoMa']:.0f} mA [{f['servoMa_lo']:.0f},{f['servoMa_hi']:.0f}] "
              f"= {100 * f['servoMa'] / STALL_MA:.2f}% of stall"
              f"{'' if sig else '   (NOT resolved - CI includes zero)'}")

    path = os.path.join(OUT, f"{label}.json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=1)
    print(f"\nsaved {path}")
    print(f"V {out['voltage_start']:.2f} -> {out['voltage_end']:.2f}")


if __name__ == "__main__":
    raise SystemExit(main())
