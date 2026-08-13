"""Servo rail current against commanded power, per pod.

Current is the only torque proxy available on this hardware. Below breakaway the servo is pushing
and the pod is not moving, so the pod's own stiction is the load and the servo is genuinely
stalled - which makes those points a command-to-torque map at zero speed, with nothing to damage
and no hard stop needed.

This exists because "breakaway is 0.035, so 96.5% of torque authority is unused" was wrong: 0.035
is a normalised CRServo command, which sets pulse width and therefore a speed setpoint, not a
torque fraction. Current is a real proxy; a command fraction is not.

Sampling is switched to per-loop for the duration (setFastCurrent) because the 5 Hz idle path
cannot see a peak inside a dwell.

What this still cannot give: the fraction of the servo's capacity in use. That needs a stall
current reference at full command, which is either a datasheet figure or a deliberate stall.

    python servocurrent.py
"""

import gzip
import statistics as st
import time

from swervebench import Bench, parse_csv

POWERS = [round(0.00 + 0.01 * i, 3) for i in range(16)]  # 0.00 .. 0.15
DWELL_S = 0.5
GAP_S = 0.4
MOVE_DEG = 1.5  # wheel travel within a dwell that counts as "it moved"
SETTLE_SKIP_S = 0.12  # drop the inrush at the start of each dwell before averaging


def dwell_windows(t, pwr, target, skip_s):
    """Index ranges where the written power sat at `target`, minus the leading transient."""
    out, start = [], None
    for i, p in enumerate(pwr):
        on = p == p and abs(p - target) < 1e-6 and target != 0
        if on and start is None:
            start = i
        elif not on and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(pwr)))
    trimmed = []
    for a, b in out:
        t0 = t[a]
        c = a
        while c < b and t[c] - t0 < skip_s:
            c += 1
        if b - c >= 3:
            trimmed.append((c, b))
    return trimmed


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


def main():
    b = Bench()
    print(f"V {b.state()['voltage']:.2f}   powers {POWERS[0]:.2f}..{POWERS[-1]:.2f}   "
          f"dwell {DWELL_S}s")

    b.cmd("setFastCurrent", value="true")
    b.cmd("setPublishHz", value=4)  # keep the loop as fast as possible while sampling current
    time.sleep(0.6)

    # Baseline: everything released. This is the zero that per-pod numbers subtract.
    b.cmd("stop")
    time.sleep(0.5)
    b.cmd("recStart", label="current-baseline")
    time.sleep(4.0)
    b.cmd("recStop")
    time.sleep(0.3)
    base_tr = parse_csv(b.rec_csv())
    base_ma = [x for x in base_tr["servoMa"] if x == x]
    base = st.median(base_ma)
    dts = [x for x in base_tr["dt"] if x == x and x > 0]
    print(f"baseline (all released): {base:.0f} mA median, "
          f"{min(base_ma):.0f}-{max(base_ma):.0f} range, "
          f"sampled at {1 / st.mean(dts):.0f} Hz\n")

    results = {}
    for pod in range(4):
        r = b.raw_staircase(pod, POWERS, dwell_s=DWELL_S, gap_s=GAP_S,
                            label=f"current-p{pod}")
        tr = parse_csv(gzip.open(r["trace_file"], "rt").read())

        t = tr["t"]
        ma = tr["servoMa"]
        pwr = tr[f"p{pod}_pwr"]
        wheel = unwrap(tr[f"p{pod}_wheel"])

        print(f"--- pod {pod} ---")
        print(f"{'cmd':>6}{'net mA':>9}{'peak mA':>9}{'travel deg':>12}{'moved':>7}")
        rows = []
        for target in POWERS:
            if target == 0:
                continue
            wins = dwell_windows(t, pwr, target, SETTLE_SKIP_S)
            if not wins:
                continue
            cur, peak, trav = [], [], []
            for a, c in wins:
                seg = [ma[i] for i in range(a, c) if ma[i] == ma[i]]
                w = [wheel[i] for i in range(a, c) if wheel[i] == wheel[i]]
                if seg:
                    cur.append(st.mean(seg))
                    peak.append(max(seg))
                if len(w) >= 2:
                    trav.append(max(w) - min(w))
            if not cur:
                continue
            net = st.mean(cur) - base
            tv = st.mean(trav) if trav else float("nan")
            moved = tv > MOVE_DEG
            rows.append((target, net, max(peak) - base, tv, moved))
            print(f"{target:6.2f}{net:9.0f}{max(peak) - base:9.0f}{tv:12.2f}{'  yes' if moved else '   no':>7}")
        results[pod] = rows
        b.cmd("stop")
        time.sleep(0.4)
        print()

    b.cmd("setFastCurrent", value="false")
    b.cmd("setPublishHz", value=20)

    print("=" * 62)
    print("breakaway and the current it took:")
    for pod, rows in results.items():
        moved = [r for r in rows if r[4]]
        if not moved:
            print(f"  pod {pod}: never moved up to {POWERS[-1]:.2f}")
            continue
        first = moved[0]
        stalled = [r for r in rows if not r[4]]
        top_stall = stalled[-1] if stalled else None
        print(f"  pod {pod}: breakaway {first[0]:.2f} at {first[1]:.0f} mA net"
              + (f"   (last stalled point {top_stall[0]:.2f} at {top_stall[1]:.0f} mA)"
                 if top_stall else ""))
    print(f"\nbaseline {base:.0f} mA subtracted throughout. Current is a torque proxy; converting")
    print("to a fraction of capacity needs a stall-current reference we do not have.")


if __name__ == "__main__":
    raise SystemExit(main())
