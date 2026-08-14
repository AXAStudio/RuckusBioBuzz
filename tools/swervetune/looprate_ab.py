"""A/B the dashboard publish rate against pod tracking.

msPublish measured 42.7 ms of a 53.6 ms loop, which put the true control rate at 20-25 Hz while
driving. At a measured pod slew of 214 deg/s that is ~10 deg of pod travel between control
updates, and the steady-state error observed while driving was 7.7-10.5 deg. This tests whether
those two facts are the same fact.

Only the publish rate changes. Gains, surface, battery and stimulus are identical, and the two
arms are interleaved in a randomised order so that battery drift lands as noise rather than as a
bias in favour of whichever arm ran first.

Stimulus is pidStepAll rather than driving: it is the same 90 deg step every other measurement in
this project used, it moves no robot across the floor, and it is safe on blocks or on carpet.

    python looprate_ab.py [repeats]
"""

import random
import statistics as st
import sys
import time

from swervebench import Bench, format_step

# Publish Hz: the shipped default, and as slow as the clamp allows. Measured in DRIVE with a
# stationary X-lock hold, that spread is a true loop rate of ~20-27 Hz against ~58 Hz, which is
# the widest contrast available without touching anything else. The recorder samples every loop
# regardless of publish rate, so the 1 Hz arm loses dashboard refresh, not data.
ARMS = [20.0, 1.0]
STEP_DEG = 90.0
BASE_DEG = 0.0


def summarise(tag, results):
    # ok, not `not parked`: "parked" flags a GOOD run, so the old filter compared the two arms
    # using only their badly-behaved pod-runs. (The 2026-08-13 A/B's null result survives this
    # fix - rescoring both arms over all ok runs still shows no separation - but the filter was
    # wrong all the same.)
    pods = [p for r in results for p in r["pods"] if p.get("ok")]
    if not pods:
        return f"{tag}: no usable pod-runs"
    ss = [abs(p["steady_state_abs_deg"]) for p in pods if p["steady_state_abs_deg"] == p["steady_state_abs_deg"]]
    e3 = [abs(p["err_at_3s"]) for p in pods if p["err_at_3s"] == p["err_at_3s"]]
    rings = [p["rings"] for p in pods if p["rings"] == p["rings"]]
    pp = [p["post_settle_pp_deg"] for p in pods if p["post_settle_pp_deg"] == p["post_settle_pp_deg"]]
    loose = sum(1 for x in pp if x > 5.0)
    hz = [r["loop_hz_true"] for r in results if r.get("loop_hz_true") == r.get("loop_hz_true")]
    return (
        f"{tag}\n"
        f"  loop (true)        {st.mean(hz):6.1f} Hz\n"
        f"  |steady state|     {st.mean(ss):6.2f} deg mean   {max(ss):6.2f} max\n"
        f"  |err at 3 s|       {st.mean(e3):6.2f} deg mean   {max(e3):6.2f} max\n"
        f"  rings              {st.mean(rings):6.2f} mean     {max(rings):6.0f} max\n"
        f"  post-settle p-p    {st.mean(pp):6.2f} deg mean   {max(pp):6.2f} max\n"
        f"  loose (p-p > 5)    {loose:4d} / {len(pp)}\n"
        f"  n pod-runs         {len(pods):4d}"
    )


def main():
    repeats = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    b = Bench()

    st0 = b.state()
    print(f"start: V {st0['voltage']:.2f}  mode {st0.get('mode')}")
    p0 = st0["pods"][0]
    print(f"gains: kP {p0['kp']}  kI {p0['ki']}  kD {p0['kd']}  kF {p0['kf']}  "
          f"kS {p0['ks']}  band {p0['ksband']}  cache {p0['cache']}")
    if st0["errors"]:
        print("errors:")
        for e in st0["errors"]:
            print("  *", e)

    order = [hz for hz in ARMS for _ in range(repeats)]
    random.shuffle(order)
    print(f"\n{len(order)} runs, randomised: "
          + "".join("H" if h == ARMS[0] else "L" for h in order) + "\n")

    out = {hz: [] for hz in ARMS}
    for n, hz in enumerate(order, 1):
        b.cmd("setPublishHz", value=hz)
        time.sleep(0.4)
        r = b.step_trial(
            step_deg=STEP_DEG,
            base_deg=BASE_DEG,
            label=f"pubhz{hz:g}-{len(out[hz]) + 1}",
            notes={"publish_hz": hz, "experiment": "looprate_ab"},
        )
        out[hz].append(r)
        print(f"[{n:3d}/{len(order)}] pub {hz:4.0f} Hz  {format_step(r).splitlines()[0]}")

    b.cmd("setPublishHz", value=ARMS[0])

    print("\n" + "=" * 68)
    for hz in ARMS:
        print(summarise(f"publish {hz:g} Hz", out[hz]))
        print()
    print(f"end: V {b.state()['voltage']:.2f}")


if __name__ == "__main__":
    raise SystemExit(main())
