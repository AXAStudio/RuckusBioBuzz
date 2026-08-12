"""Criterion 8, measured as effort at rest: servo holding current.

The original criterion was "resting servo power RMS below the measured deadband", which is a
CR-mode quantity - in position mode there is no commanded power to measure. Reusing post-settle
peak-to-peak instead would just be criterion 6 wearing a hat, and it also misses the failure that
matters: an internal loop that is working hard to hold station shows up as current long before it
shows up as visible motion.

Measured on the BATTERY current channel, not the servo rail. The servo rail reads essentially
nothing on this robot - the turn servos are fed from the servo power module, so the hub supplies
only their PWM signal. Verified: with all four pods holding, the rail moved -0 -> 10 mA against a
44 mA standard deviation, while the battery channel moved 189 -> 205 mA against 12, a 5.6 sigma
effect. The SPM's draw reaches the hub's battery input; its servo rail does not see it.

Aggregate, not per port, which differencing handles: the three pods not under test have their PWM
disabled for the whole measurement, so they contribute a constant and only the pod under test
changes state.

Three conditions per pod:

    A   pod enabled, limp/idle            -> baseline draw of an enabled but undriven servo
    C   pod enabled, holding on target     -> what we care about
    B   pod disabled                       -> resolution check: A - B must be measurable

Pass condition, stated relatively because the old 0.025 threshold is a CR number with no meaning
here: C - A indistinguishable from zero within the rail's own noise. Elevated holding current means
the loop is doing work while on target, which is hunting whether or not the encoder shows it.

    python crit8_current.py            # all four pods
    python crit8_current.py 1          # one pod
"""

from __future__ import annotations

import math
import sys
import time

from swervebench import Bench, parse_csv, _mean, _stdev

SAMPLE_SECONDS = 30.0
POLL_S = 0.22
HOLD_TARGET_DEG = 45.0


def rail_samples(b: Bench, label: str, seconds: float) -> list[float]:
    """Battery current samples. Standard error falls as 1/sqrt(n), and the effect is a few mA
    against a 12 mA spread, so this wants a long dwell rather than a fast one."""
    out = []
    end = time.time() + seconds
    while time.time() < end:
        try:
            out.append(float(b.state()["batteryMa"]))
        except (KeyError, ValueError):
            pass
        time.sleep(POLL_S)
    return out


def measure(b: Bench, pod: int) -> dict:
    # Silence the other three for the whole measurement so they cannot drift into the difference.
    for other in range(4):
        if other != pod:
            b.cmd("setPwmEnable", value="false", pod=other)
    b.cmd("select", pod=pod)
    b.cmd("setPwmEnable", value="true", pod=pod)
    time.sleep(1.0)

    # A: enabled and idle. IDLE releases every servo, so nothing is driven.
    b.cmd("stop")
    time.sleep(1.0)
    a = rail_samples(b, f"crit8-p{pod}-idle", SAMPLE_SECONDS)

    # C: enabled and holding on target.
    b.cmd("pidStepAll", deg=HOLD_TARGET_DEG)
    time.sleep(3.0)                       # settle before sampling, so this is holding not slewing
    c = rail_samples(b, f"crit8-p{pod}-hold", SAMPLE_SECONDS)
    b.cmd("stop")
    time.sleep(0.8)

    # B: disabled, to prove the differencing resolves anything at all.
    b.cmd("setPwmEnable", value="false", pod=pod)
    time.sleep(1.0)
    bb = rail_samples(b, f"crit8-p{pod}-off", SAMPLE_SECONDS)
    for other in range(4):
        b.cmd("setPwmEnable", value="true", pod=other)
    time.sleep(0.5)

    noise = max(_stdev(a) if len(a) > 1 else 0.0, _stdev(c) if len(c) > 1 else 0.0)
    delta = _mean(c) - _mean(a)
    se = math.sqrt(
        (_stdev(a) ** 2 / max(1, len(a))) + (_stdev(c) ** 2 / max(1, len(c)))
    ) if len(a) > 1 and len(c) > 1 else float("nan")

    return {
        "pod": pod,
        "idle": _mean(a), "idle_sd": _stdev(a) if len(a) > 1 else float("nan"), "n_idle": len(a),
        "hold": _mean(c), "hold_sd": _stdev(c) if len(c) > 1 else float("nan"), "n_hold": len(c),
        "off": _mean(bb), "n_off": len(bb),
        "quiescent": _mean(a) - _mean(bb),
        "delta": delta, "se": se, "noise": noise,
        "pass": abs(delta) <= 2 * se if se == se else False,
    }


def main() -> None:
    pods = [int(sys.argv[1])] if len(sys.argv) > 1 else [0, 1, 2, 3]
    b = Bench()
    print(f"battery {b.voltage():.2f} V, {SAMPLE_SECONDS:.0f} s per condition\n")

    rows = [measure(b, p) for p in pods]
    b.stop()

    print(f"  {'pod':>3} {'off mA':>8} {'idle mA':>16} {'hold mA':>16} "
          f"{'hold-idle':>12} {'verdict':>9}")
    for r in rows:
        print(
            f"  {r['pod']:>3} {r['off']:>8.0f} "
            f"{r['idle']:>8.0f} +/-{r['idle_sd']:>4.0f} ({r['n_idle']:>3}) "
            f"{r['hold']:>8.0f} +/-{r['hold_sd']:>4.0f} ({r['n_hold']:>3}) "
            f"{r['delta']:>+7.0f} +/-{r['se']:>3.0f} "
            f"{'PASS' if r['pass'] else 'ELEVATED':>9}"
        )

    # Resolution check. Deliberately NOT idle-versus-off: for a CR servo those two are genuinely
    # almost identical, because an undriven CR servo is not turning its motor whether or not its
    # PWM is enabled, so a null there says nothing about the instrument. What proves resolution is
    # that the per-pod holding deltas add up to the effect measured with the whole fleet holding at
    # once, which was +15 mA against a 2 mA standard error.
    ses = [r["se"] for r in rows if r["se"] == r["se"]]
    total = sum(r["delta"] for r in rows)
    se_total = math.sqrt(sum(s * s for s in ses))
    print(f"\n  sum of per-pod holding deltas: {total:+.0f} +/- {se_total:.0f} mA")
    print("  same quantity measured directly with all four holding: +15 +/- 2 mA")
    if ses:
        print(f"  standard error per pod {_mean(ses):.1f} mA against a per-servo effect of a few "
              f"mA:\n  usable, but only with the long dwell - do not shorten it")

    print("\n  quiescent draw of an enabled but undriven servo (idle - off), expected near zero:")
    for r in rows:
        print(f"    pod {r['pod']}: {r['quiescent']:+.0f} mA")



if __name__ == "__main__":
    main()
