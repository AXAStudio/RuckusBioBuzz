"""Phase 2: where the pod's actuation lag actually lives.

The closed loop limit-cycles at about 3.6 Hz. Reconciling that with the phase budget implies of
order 100 ms of lag, and the loop period is 8-13 ms, so most of it is somewhere else. This splits
the lag into the pieces that can be measured separately:

  transport delay  command written -> pod first moves. Loop period, bus transaction, PWM frame
                   phasing and servo electronics, all lumped, because nothing here can separate
                   them further from the outside.
  velocity lag     first motion -> 63% of terminal speed. The motor and gear train spinning up.
                   This is a lag, not a delay: it contributes phase that rises with frequency and
                   it cannot be removed by commanding faster.

The distinction matters for what to do about it. Transport delay is attackable - shorter PWM frame,
faster loop. A velocity time constant is the plant, and the answer to it is lead compensation, not
a faster command path.

    python phase2_delay.py
"""

from __future__ import annotations

import math
import time

from swervebench import Bench, parse_csv, _mean, _stdev
from phase2_plant import unwrap, segments, MOVE_THRESHOLD_DEG

POD_COUNT = 4
POWERS = (0.30, 1.00)
REPEATS = 6
DWELL_S = 0.35

# Velocity is differenced over this many samples: enough to keep the 0.1125 deg encoder step from
# dominating, short enough not to smear the rise being measured.
VEL_WINDOW = 5


def velocity_series(t: list[float], ang: list[float], a: int, z: int) -> list[tuple[float, float]]:
    out = []
    for k in range(a + VEL_WINDOW, z + 1):
        dt = t[k] - t[k - VEL_WINDOW]
        if dt > 1e-6:
            out.append((t[k], (ang[k] - ang[k - VEL_WINDOW]) / dt))
    return out


def analyse(tr: dict, pod: int) -> dict | None:
    t = tr["t"]
    ang = unwrap(tr[f"p{pod}_wheel"])
    pwr = tr[f"p{pod}_pwr"]
    segs = segments(t, pwr)
    if not segs:
        return None
    a, z = segs[0]["start"], segs[0]["end"]
    if z - a < VEL_WINDOW + 3:
        return None

    t0 = t[a]
    base = ang[max(0, a - 1)]

    dead = None
    for k in range(a, z + 1):
        if abs(ang[k] - base) >= MOVE_THRESHOLD_DEG:
            dead = t[k] - t0
            break
    if dead is None:
        return None

    vs = velocity_series(t, ang, a, z)
    if len(vs) < 6:
        return None
    # Terminal speed from the last third, where the rise is over.
    tail = [abs(v) for _, v in vs[int(2 * len(vs) / 3):]]
    v_inf = _mean(tail)
    if not v_inf or math.isnan(v_inf) or v_inf < 20:
        return None

    tau = None
    for tv, v in vs:
        if abs(v) >= 0.632 * v_inf:
            tau = (tv - t0) - dead
            break

    return {
        "dead_ms": dead * 1000.0,
        "tau_ms": None if tau is None else max(0.0, tau) * 1000.0,
        "v_inf": v_inf,
        "loop_ms": 1000.0 * (t[z] - t[a]) / max(1, z - a),
    }


def measure(b: Bench, pod: int, power: float, repeats: int = REPEATS) -> list[dict]:
    rows = []
    for n in range(repeats):
        # Alternate direction so a gravity or preload bias averages out rather than accumulating.
        sign = 1 if n % 2 == 0 else -1
        b.cmd("select", pod=pod)
        b.cmd("recStart", label=f"delay-p{pod}-{power}-{n}")
        time.sleep(0.2)
        b.cmd("rawServo", pow=sign * power, sec=DWELL_S, pod=pod)
        time.sleep(DWELL_S + 0.45)
        b.cmd("recStop")
        time.sleep(0.15)
        r = analyse(parse_csv(b.rec_csv()), pod)
        if r:
            rows.append(r)
    return rows


def main() -> None:
    b = Bench()
    st = b.state()
    print(f"battery {st['voltage']:.2f} V\n")

    print("Servo PWM configuration, read back from the hardware:")
    print(f"  {'pod':>3} {'servo':>6} {'pulse lo':>9} {'pulse hi':>9} {'span':>7} "
          f"{'frame':>8} {'rate':>8}")
    for p in st["pods"]:
        frame = p["pwmFrame"]
        rate = 1e6 / frame if frame else float("nan")
        print(f"  {p['i']:>3} {p['servo']:>6} {p['pwmLo']:>9.0f} {p['pwmHi']:>9.0f} "
              f"{p['pwmHi'] - p['pwmLo']:>7.0f} {frame:>8.0f} {rate:>7.1f}Hz")
    frames = [p["pwmFrame"] for p in st["pods"]]
    if frames and max(frames):
        f = max(frames)
        print(f"\n  A {f / 1000:.0f} ms frame means a new pulse width reaches the servo at most")
        print(f"  {1e6 / f:.0f} times a second, whatever the loop rate. A command written just")
        print(f"  after a frame boundary waits nearly a full frame: {f / 1000:.0f} ms worst case,")
        print(f"  {f / 2000:.0f} ms on average. That is the floor this contributes to transport delay.")

    print(f"\nOpen-loop step response, {REPEATS} repeats per pod per power, "
          f"alternating direction:")
    print(f"  {'pod':>3} {'power':>6} {'dead ms':>16} {'tau ms':>16} {'v_inf deg/s':>13} "
          f"{'loop ms':>8}")
    summary = {}
    for power in POWERS:
        for pod in range(POD_COUNT):
            rows = measure(b, pod, power)
            if not rows:
                print(f"  {pod:>3} {power:>6.2f}   no usable step")
                continue
            dead = [r["dead_ms"] for r in rows]
            taus = [r["tau_ms"] for r in rows if r["tau_ms"] is not None]
            summary.setdefault(power, []).append((_mean(dead), _mean(taus) if taus else float("nan")))
            print(
                f"  {pod:>3} {power:>6.2f} {_mean(dead):>8.1f} +/-{_stdev(dead):>5.1f} "
                f"{_mean(taus):>8.1f} +/-{_stdev(taus):>5.1f} "
                f"{_mean([r['v_inf'] for r in rows]):>13.0f} "
                f"{_mean([r['loop_ms'] for r in rows]):>8.1f}"
            )
    b.stop()

    print("\nDelay budget")
    for power, rows in summary.items():
        d = _mean([r[0] for r in rows])
        tau = _mean([r[1] for r in rows if not math.isnan(r[1])])
        # A first-order lag of time constant tau contributes atan(w*tau) of phase, which near the
        # limit-cycle frequency is close to what a delay of tau would contribute; quoting it as an
        # equivalent delay keeps the budget in one currency.
        print(f"  at power {power:.2f}: transport {d:.0f} ms + velocity lag {tau:.0f} ms "
              f"= {d + tau:.0f} ms equivalent")


if __name__ == "__main__":
    main()
