"""Focused pulse-travel recalibration on the current (lubricated) plant.

The step A/B showed pulse ping-pong (25 pulses/3 s, 9.8 deg pp) - the classic
travel > tolerance divergence. Re-measure travel for candidate power x duration
cells around the current setting, report mean and both scatters, and flag any
duration whose within-condition sd says the PWM frame is quantising it.

    python pulsecal2.py
"""

from __future__ import annotations

from swervebench import Bench, _mean, _stdev
from phase4_pulsecal import one_pulse

POWERS = (0.035, 0.045, 0.055)
DURATIONS_MS = (15, 20, 30)
REPEATS = 3


def main() -> None:
    b = Bench()
    print(f"battery {b.voltage():.2f} V")
    print("travel per pulse from rest, deg (mean of both signs, all pods)\n")
    print(f"  {'pwr':>6} {'ms':>4} {'mean':>6} {'min':>6} {'max':>6} "
          f"{'between_sd':>10} {'within_sd':>9} {'mean+sd':>8}")
    for power in POWERS:
        for ms in DURATIONS_MS:
            means, withins = [], []
            for pod in range(4):
                for sign in (1, -1):
                    vals = [v for v in (one_pulse(b, pod, power, ms, sign)
                                        for _ in range(REPEATS)) if v is not None]
                    if vals:
                        means.append(_mean(vals))
                        if len(vals) > 1:
                            withins.append(_stdev(vals))
            if not means:
                print(f"  {power:>6.3f} {ms:>4.0f}   no data")
                continue
            bsd = _stdev(means)
            wsd = _mean(withins) if withins else float("nan")
            comb = _mean(means) + (bsd ** 2 + (0 if wsd != wsd else wsd ** 2)) ** 0.5
            print(f"  {power:>6.3f} {ms:>4.0f} {_mean(means):>6.2f} {min(means):>6.2f} "
                  f"{max(means):>6.2f} {bsd:>10.2f} {wsd:>9.2f} {comb:>8.2f}")
    b.stop()
    print("\npick: largest cell whose mean+sd fits under the tolerance; if 15 ms")
    print("within_sd is an outlier vs 20/30 ms, the PWM frame is quantising it - avoid.")


if __name__ == "__main__":
    main()
