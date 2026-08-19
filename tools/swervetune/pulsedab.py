"""Pulsed final approach vs continuous loop, statically (no robot translation).

Two instruments, both safe without a bounding box:

  STEP   trials.py step cells (90 and 15 deg) with pulsed on/off, interleaved.
         The locked-in numbers: post-settle pp, rest power RMS, pulses fired
         after settle, steady-state error.
  REST   drive 0,0,0 -> X-lock engages -> record 8 s of nothing. act_energy is
         the audible hunt; with pulses it should be ~zero (a few discrete
         nudges at most).

    python pulsedab.py
"""

from __future__ import annotations

import time

from swervebench import Bench, parse_csv, _archive
from podhold import _p95span
import trials


def set_pulsed(b: Bench, on: bool) -> None:
    b.cmd("setPidf", scope="all", pulsed="true" if on else "false")
    time.sleep(0.4)
    got = b.state()["pods"][0].get("pulsed")
    if bool(got) != on:
        raise SystemExit(f"pulsed did not take: asked {on}, pod reports {got}")


def xlock_rest(b: Bench, tag: str, secs: float = 8.0):
    for _ in range(6):
        b.cmd("drive", retries=1, f=0, s=0, t=0)
        time.sleep(0.1)
    time.sleep(1.5)  # X-lock engage (0.35 s) + arrival transient
    b.cmd("recStart", label=f"rest-{tag}")
    t0 = time.time()
    while time.time() - t0 < secs:
        b.cmd("drive", retries=1, f=0, s=0, t=0)
        time.sleep(0.3)
    b.cmd("recStop")
    time.sleep(0.3)
    csv_text = b.rec_csv()
    _archive(csv_text, {"label": f"rest-{tag}"})
    tr = parse_csv(csv_text)
    t = tr["t"]
    print(f"  rest '{tag}' ({t[-1]:.1f} s): "
          f"{'pod':>3} {'act_energy':>10} {'pwr_rms':>8} {'engaged%':>8} {'err_pp':>7}")
    rows = []
    for i in range(4):
        pwr = [v for v in tr[f"p{i}_pwr"] if v == v]
        err = [v for v in tr[f"p{i}_err"] if v == v]
        act = sum(abs(pwr[k] - pwr[k - 1]) for k in range(1, len(pwr)))
        rms = (sum(v * v for v in pwr) / len(pwr)) ** 0.5 if pwr else float("nan")
        eng = 100.0 * sum(1 for v in pwr if abs(v) > 0.02) / len(pwr) if pwr else 0
        pp = _p95span(err) if err else float("nan")
        rows.append({"pod": i, "act": act, "rms": rms, "eng": eng, "pp": pp})
        print(f"  {'':>16} {i:>3} {act:>10.2f} {rms:>8.4f} {eng:>7.1f}% {pp:>7.2f}")
    b.cmd("stop")
    return rows


def main() -> None:
    b = Bench()
    print(f"battery {b.voltage():.2f} V\n")

    print("== X-lock rest (the audible hunt) ==")
    set_pulsed(b, False)
    xlock_rest(b, "cont-a")
    set_pulsed(b, True)
    xlock_rest(b, "pulsed-a")
    set_pulsed(b, False)
    xlock_rest(b, "cont-b")
    set_pulsed(b, True)
    xlock_rest(b, "pulsed-b")

    print("\n== Step cells (interleaved) ==")
    for step in (90.0, 15.0):
        for on in (False, True, False, True):
            set_pulsed(b, on)
            trials.run(b, f"{'pulsed' if on else 'cont'}-{step:g}", step=step,
                       repeats=5, hold=5.0)
            print()
    set_pulsed(b, True)
    b.stop()


if __name__ == "__main__":
    main()
