"""Phase 4 gate: evaluate one candidate gain set against every section 7 criterion, off the ground.

Runs the step sizes the criteria name, plus the flip-boundary sweep that replaces the literal
180 degree case. A 180 degree step is a no-op by construction - move() sees |error| > 90 degrees,
adds pi to the target and the pod holds still while only the drive sign inverts - so testing it
measures nothing. The real hazard at that boundary is the flip decision chattering when the error
sits near 90 degrees, which is what the 85/88/90/92/95 sweep looks for.

    python phase4_gate.py
"""

from __future__ import annotations

import math
import sys
import time

from swervebench import Bench, _mean, _stdev

# Shipped set as of the 2026-08-13 post-lube retune on the game tiles.
CANDIDATE = dict(
    kp=0.320, ki=0.0, kd=0.022, kf=0.0,
    ks=0.035, ksband=2.0, cache=0.01, dom=False,
)

# Measured open-loop breakaway, lowest of the eight pod/direction combinations. Criterion 8 asks
# for resting servo power below this.
# Re-measured after lubrication: 0.030 off the ground, 0.050 on the tiles, uniform across all
# four pods. The lower of the two is the harder bar for criterion 8, so keep using it.
BREAKAWAY_MIN = 0.030

# Stated by the operator, never assumed. This printed "robot on blocks" as a constant and said
# so while sitting on the game tiles.
SURFACE = sys.argv[1] if len(sys.argv) > 1 else "SURFACE NOT STATED - pass it as argv[1]"

REPEATS_MAIN = 10
REPEATS_SMALL = 6


def collect(b: Bench, step: float, repeats: int, label: str, base: float = 0.0) -> list[dict]:
    pods = []
    for n in range(repeats):
        r = b.step_trial(step_deg=step, base_deg=base, hold_s=5.0,
                         label=f"gate-{label}-{n+1}", notes={"gate": label, "step": step})
        pods.extend(p for p in r["pods"] if p.get("ok"))
        time.sleep(0.4)
    return pods


def col(pods: list[dict], key: str) -> list[float]:
    return [p[key] for p in pods
            if key in p and p[key] is not None
            and not (isinstance(p[key], float) and math.isnan(p[key]))]


def settle_stats(pods: list[dict], key: str) -> tuple[float, float, float]:
    """Mean, sigma and the fraction of pod-runs that settled at all."""
    vals = col(pods, key)
    return _mean(vals), _stdev(vals), len(vals) / max(1, len(pods))


def pod_spread(pods: list[dict], key: str) -> float:
    per = {}
    for p in pods:
        v = p.get(key)
        if v is not None and not (isinstance(v, float) and math.isnan(v)):
            per.setdefault(p["pod"], []).append(v)
    means = [_mean(v) for v in per.values() if v]
    if len(means) < 2 or not _mean(means):
        return float("nan")
    return 100.0 * (max(means) - min(means)) / _mean(means)


def line(n: int, name: str, target: str, achieved: str, ok: bool | None) -> str:
    mark = "PASS" if ok else ("FAIL" if ok is False else " -- ")
    return f"  {n:>2}  {name:<34} {target:<18} {achieved:<26} {mark}"


def main() -> None:
    b = Bench()
    st = b.set_pidf(**CANDIDATE)
    p0 = st["pods"][0]
    print("Candidate gain set (all four pods identical):")
    print(f"  kP={p0['kp']}  kI={p0['ki']}  kD={p0['kd']}  kF={p0['kf']}  "
          f"kS={p0['ks']} band={p0['ksband']} deg  cache={p0['cache']}  dom={p0['dom']}")
    print(f"  battery {st['voltage']:.2f} V, " + SURFACE + "\n")

    b.step_trial(step_deg=90, hold_s=2.0, label="gate-discard", save=False)

    print(f"running 90 deg x{REPEATS_MAIN}, 45 deg x{REPEATS_SMALL}, 15 deg x{REPEATS_SMALL}, "
          f"flip boundary x2 each ...")
    s90 = collect(b, 90, REPEATS_MAIN, "90")
    s45 = collect(b, 45, REPEATS_SMALL, "45")
    s15 = collect(b, 15, REPEATS_SMALL, "15")

    flip = {}
    for step in (85, 88, 90, 92, 95):
        flip[step] = collect(b, step, 2, f"flip{step}")

    b.stop()
    volts = b.voltage()

    all_steps = s90 + s45 + s15
    # Criterion 1 was re-spec'd to "enter and STAY within +/-2.0 deg". This scored against
    # settle_1_0 for a while after that, which is a stricter bar than the one agreed and made
    # every run look worse than it was.
    m90, sd90, frac90 = settle_stats(s90, "settle_2_0")
    m15, sd15, frac15 = settle_stats(s15, "settle_0_5")

    ss_all = [abs(x) for x in col(all_steps, "steady_state_deg")]
    over90 = col(s90, "overshoot_pct")
    over15 = col(s15, "overshoot_pct")
    rings90 = col(s90, "rings")
    rings90_1 = col(s90, "rings_1pct")
    pp90 = col(s90, "post_settle_pp_deg")
    rest90 = col(s90, "rest_power_rms")

    print(f"\nSection 7, off the ground, {volts:.2f} V, n={len(s90)} pod-runs at 90 deg\n")
    print(f"  {'#':>2}  {'criterion':<34} {'target':<18} {'achieved':<26} verdict")
    print("  " + "-" * 86)

    print(line(1, "90 deg settle to +/-2.0 deg", "<= 350 ms",
               f"{1000*m90:.0f} ms ({100*frac90:.0f}% settled)", m90 <= 0.350 and frac90 == 1.0))
    print(line(2, "90 deg overshoot", "<= 5%",
               f"{_mean(over90):.1f}% mean, {max(over90):.1f}% max", max(over90) <= 5.0))
    print(line(3, "15 deg settle to +/-0.5 deg", "<= 200 ms, no overshoot",
               f"{1000*m15:.0f} ms, {max(over15) if over15 else 0:.1f}% max",
               m15 <= 0.200 and frac15 == 1.0 and (max(over15) if over15 else 0) <= 0.1))
    print(line(5, "steady-state, all step sizes", "<= 1.0 deg",
               f"{_mean(ss_all):.2f} mean, {max(ss_all):.2f} max", max(ss_all) <= 1.0))
    print(line(6, "post-settle p-p over 3 s", "<= 0.5 deg",
               f"{_mean(pp90):.2f} mean, {max(pp90):.2f} max", max(pp90) <= 0.5))
    print(line(7, "rings per step (0.30 deg thr)", "<= 1",
               f"{_mean(rings90):.2f} mean, {max(rings90):.0f} max", max(rings90) <= 1))
    print(line(7, "rings per step (1% of step)", "<= 1",
               f"{_mean(rings90_1):.2f} mean, {max(rings90_1):.0f} max", max(rings90_1) <= 1))
    print(line(8, "resting servo power RMS", f"< {BREAKAWAY_MIN} breakaway",
               f"{_mean(rest90):.4f} mean, {max(rest90):.4f} max",
               max(rest90) < BREAKAWAY_MIN))
    print(line(10, "pod-to-pod settle spread", "<= 15%",
               f"{pod_spread(s90, 'settle_2_0'):.0f}%", pod_spread(s90, "settle_2_0") <= 15.0))
    print(line(11, "repeatability, sigma of settle", "<= 20% of mean",
               f"{100*sd90/m90 if m90 else float('nan'):.0f}%",
               m90 > 0 and sd90 / m90 <= 0.20))
    print(line(4, "flip boundary (replaces 180)", "no chatter", "see below", None))
    print(line(9, "heading hold under drive", "<= 2 deg", "Phase 5, on ground", None))
    print(line(12, "criteria 1-7 at 11.5 V", "still met", "Phase 5, drained pack", None))

    print(f"\n  Flip boundary sweep - a flip once the pod is moving means the decision chattered:")
    print(f"  {'step':>6} {'flips':>7} {'rings':>7} {'|ss| max':>9} {'pp max':>8}")
    for step, pods in flip.items():
        if not pods:
            continue
        f = sum(p["flip_events"] for p in pods)
        print(f"  {step:>6} {f:>7} {max(col(pods,'rings')):>7.0f} "
              f"{max(abs(x) for x in col(pods,'steady_state_deg')):>9.2f} "
              f"{max(col(pods,'post_settle_pp_deg')):>8.2f}")

    print(f"\n  45 deg: settle(1.0) {1000*settle_stats(s45,'settle_1_0')[0]:.0f} ms, "
          f"|ss| max {max(abs(x) for x in col(s45,'steady_state_deg')):.2f} deg, "
          f"rings max {max(col(s45,'rings')):.0f}")
    print(f"  15 deg: settle(0.5) {1000*m15:.0f} ms, "
          f"|ss| max {max(abs(x) for x in col(s15,'steady_state_deg')):.2f} deg, "
          f"rings max {max(col(s15,'rings')):.0f}")


if __name__ == "__main__":
    main()
