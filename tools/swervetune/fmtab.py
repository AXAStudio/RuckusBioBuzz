"""Randomised interleaved A/B of the two publish formatters. No motion, ever.

DIAGNOSTIC TOOL. publish() cost 36.7 ms with the robot at rest and no actuator writes, which
ruled out the Lynx bus and left one candidate: fmt() is String.format, and one publish makes
hundreds of calls to it. setFastFmt switches to a hand-rolled formatter at runtime, so both arms
run in one session against one battery and one JIT state.

Arms are interleaved in randomised blocks because battery voltage drifts monotonically and a
back-to-back A-then-B would confound the two. msPublish is an EMA (0.9/0.1), so each switch is
followed by a settling wait before any sample is kept.

    python fmtab.py [--blocks 8] [--settle 3.0] [--sample 5.0]
"""

from __future__ import annotations

import argparse
import json
import random
import statistics as st
import time
import urllib.parse
import urllib.request

BASE = "http://192.168.43.1:8080/swerve"


def state() -> dict:
    return json.load(urllib.request.urlopen(f"{BASE}/state", timeout=8))


def cmd(action: str, **kw) -> None:
    q = urllib.parse.urlencode({"action": action, **kw})
    urllib.request.urlopen(f"{BASE}/cmd?{q}", timeout=8).read()


def arm(fast: bool, settle: float, sample: float) -> dict:
    cmd("setFastFmt", value="true" if fast else "false")
    time.sleep(settle)
    pub, total, calls, chars, tracelen = [], [], [], [], []
    t0 = time.time()
    while time.time() - t0 < sample:
        s = state()
        t = s["timing"]
        p = t["pub"]
        if p["fastFmt"] != fast:
            time.sleep(0.2)
            continue
        pub.append(t["publish"])
        total.append(p["head"] + p["pods"] + p["errs"] + p["trace"] + p["tail"] + p["handoff"])
        calls.append(p["fmtCalls"])
        chars.append(p["chars"])
        tracelen.append(p["traceLen"])
        time.sleep(0.25)
    return {"fast": fast, "n": len(pub), "publish_ms": pub, "sections_ms": total,
            "fmtCalls": calls, "chars": chars, "traceLen": tracelen}


def summarise(rows: list[dict]) -> dict:
    pub = [x for r in rows for x in r["publish_ms"]]
    sec = [x for r in rows for x in r["sections_ms"]]
    calls = [x for r in rows for x in r["fmtCalls"]]
    return {"n": len(pub), "publish_mean": st.mean(pub), "publish_sd": st.pstdev(pub),
            "publish_min": min(pub), "publish_max": max(pub),
            "sections_mean": st.mean(sec), "calls_mean": st.mean(calls),
            "us_per_call": 1000.0 * st.mean(pub) / st.mean(calls)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocks", type=int, default=8)
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--sample", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=20260816)
    a = ap.parse_args()

    s0 = state()
    print(f"mode {s0['mode']}  V {s0['voltage']:.2f}  publishHz {s0['timing']['publishHz']:.0f}  "
          f"traceLen {s0['timing']['pub']['traceLen']}")
    print("NO drive command is sent by this script.\n")

    rng = random.Random(a.seed)
    order = []
    for _ in range(a.blocks):
        pair = [True, False]
        rng.shuffle(pair)
        order.extend(pair)

    runs = {True: [], False: []}
    for i, fast in enumerate(order):
        r = arm(fast, a.settle, a.sample)
        runs[fast].append(r)
        print(f"  block {i + 1:2d}/{len(order)}  {'fast' if fast else 'String.format':>13}  "
              f"publish {st.mean(r['publish_ms']):6.2f} ms  "
              f"sections {st.mean(r['sections_ms']):6.2f}  "
              f"calls {int(st.mean(r['fmtCalls']))}  n={r['n']}")

    A = summarise(runs[False])
    B = summarise(runs[True])
    print("\nRESULT (robot stationary, no actuator writes)")
    print(f"  {'arm':>14} {'n':>4} {'publish ms':>11} {'sd':>6} {'min':>6} {'max':>6} "
          f"{'sections':>9} {'calls':>6} {'us/call':>8}")
    for name, s in (("String.format", A), ("hand-rolled", B)):
        print(f"  {name:>14} {s['n']:>4} {s['publish_mean']:>11.2f} {s['publish_sd']:>6.2f} "
              f"{s['publish_min']:>6.2f} {s['publish_max']:>6.2f} {s['sections_mean']:>9.2f} "
              f"{int(s['calls_mean']):>6} {s['us_per_call']:>8.1f}")
    d = A["publish_mean"] - B["publish_mean"]
    print(f"\n  delta {d:.2f} ms saved per publish ({100 * d / A['publish_mean']:.0f}%), "
          f"{d * s0['timing']['publishHz']:.0f} ms/s of loop time returned at "
          f"{s0['timing']['publishHz']:.0f} Hz publishing")
    with open("current_runs/fmtab.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "volts": s0["voltage"],
                             "mode": s0["mode"], "stringformat": A, "handrolled": B}) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
