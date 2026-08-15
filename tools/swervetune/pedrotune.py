"""Tune the Pedro follower's translational, drive and centripetal controllers, in the box.

Methodology matches Pedro's own tuners, driven autonomously:
  - translational: activate translational+heading only, command lateral hold-point steps,
    score settle / overshoot / residual from the pose
  - drive: activate everything, run bounded lines, score arrival time, end overshoot and
    near-end oscillation along the line axis
  - centripetal: activate everything, run the canonical forward-then-left quadratic at speed,
    score the worst translational error during the curve

Every motion target is validated against the box HERE as well as robot-side (the firmware
refuses anything outside box minus 6 in; this script keeps 8 in so a refusal is a bug, not
routine). Between trials the robot re-centers so every trial starts with room.

    python pedrotune.py trans|drive|cent|all
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

from swervebench import Bench, _mean

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "pedrotune.jsonl")
MARGIN = 8.0


def log(obj):
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(obj) + "\n")


class Rig:
    def __init__(self):
        self.b = Bench()
        st = self.b.state()
        box = st["box"]
        if not box.get("valid"):
            raise SystemExit("no box")
        self.box = box

    def pose(self):
        st = self.b.state()
        return st["pose"]["x"], st["pose"]["y"], st

    def inside(self, x, y):
        return (self.box["minX"] + MARGIN <= x <= self.box["maxX"] - MARGIN
                and self.box["minY"] + MARGIN <= y <= self.box["maxY"] - MARGIN)

    def center(self):
        return ((self.box["minX"] + self.box["maxX"]) / 2,
                (self.box["minY"] + self.box["maxY"]) / 2)

    def start(self, activate):
        self.b.cmd("pedroStart", activate=activate)
        time.sleep(1.0)

    def stop(self):
        self.b.cmd("pedroStop")
        self.b.cmd("stop")

    def hold_step(self, dx, dy, watch_s=2.6):
        """Issues a hold-point step and samples the pose. Caller has validated bounds."""
        x0, y0, _ = self.pose()
        assert self.inside(x0 + dx, y0 + dy), "target outside safety margin"
        self.b.cmd("pedroHold", dx=dx, dy=dy)
        t0 = time.time()
        xs, ys, ts = [], [], []
        while time.time() - t0 < watch_s:
            x, y, _ = self.pose()
            ts.append(time.time() - t0)
            xs.append(x)
            ys.append(y)
            time.sleep(0.045)
        # project onto the step axis
        m = math.hypot(dx, dy)
        ux, uy = dx / m, dy / m
        prog = [ (xs[k]-x0)*ux + (ys[k]-y0)*uy for k in range(len(xs)) ]
        err = [m - p for p in prog]
        settle = None
        for k in range(len(err)):
            if abs(err[k]) > 1.0:
                settle = None if k == len(err) - 1 else ts[k + 1]
            elif settle is None and k == 0:
                settle = 0.0
        over = max([0.0] + [p - m for p in prog])
        return {"settle": round(settle, 3) if settle is not None else None,
                "over": round(over, 2), "ss": round(abs(err[-1]), 2)}

    def recenter(self, activate):
        cx, cy = self.center()
        x, y, _ = self.pose()
        dx, dy = cx - x, cy - y
        if math.hypot(dx, dy) > 3:
            self.b.cmd("pedroHold", dx=round(dx, 1), dy=round(dy, 1))
            time.sleep(2.2)


def tune_translational(rig, cells):
    print("== translational (hold-point lateral steps) ==")
    results = []
    for (tp, td) in cells:
        rig.b.cmd("pedroPidf", tp=tp, td=td)
        time.sleep(0.4)
        rig.start("transheading")
        rig.recenter("transheading")
        rows = []
        for k in range(4):
            step = 8.0 if k % 2 == 0 else -8.0
            x, y, _ = rig.pose()
            if not rig.inside(x, y + step):
                step = -step
            r = rig.hold_step(0, step)
            rows.append(r)
            print(f"  tp {tp} td {td} step {step:+.0f}: {r}")
            time.sleep(0.3)
        rig.stop()
        settled = [r["settle"] for r in rows if r["settle"] is not None]
        summ = {"kind": "trans", "tp": tp, "td": td,
                "settle": round(_mean(settled), 2) if settled else None,
                "settled_frac": len(settled) / len(rows),
                "over_max": max(r["over"] for r in rows),
                "ss": round(_mean([r["ss"] for r in rows]), 2),
                "volts": rig.b.voltage(), "t": time.strftime("%H:%M:%S")}
        print(f"  => {summ}")
        log(summ)
        results.append(summ)
        time.sleep(0.5)
    return results


def tune_drive(rig, cells):
    print("== drive (bounded lines, arrival quality) ==")
    results = []
    for (dp, dd, df) in cells:
        rig.b.cmd("pedroPidf", dp=dp, dd=dd, df=df)
        time.sleep(0.4)
        rig.start("all")
        rig.recenter("all")
        rows = []
        for k in range(4):
            x, y, st = rig.pose()
            room_fwd = rig.box["maxX"] - MARGIN - x
            room_back = x - (rig.box["minX"] + MARGIN)
            dist = min(24.0, room_fwd if k % 2 == 0 else room_back)
            sgn = 1 if k % 2 == 0 else -1
            if dist < 12:
                sgn = -sgn
                dist = min(24.0, room_back if sgn < 0 else room_fwd)
            tgt = x + sgn * dist
            assert rig.inside(tgt, y), "line target outside margin"
            rig.b.cmd("pedroLine", dx=round(sgn * dist, 1), dy=0, power=0.7)
            t0 = time.time()
            xs, ts = [], []
            while time.time() - t0 < 5.0:
                px, py, st2 = rig.pose()
                xs.append(px)
                ts.append(time.time() - t0)
                if abs(px - tgt) < 1.0 and len(xs) > 6 and abs(xs[-1] - xs[-4]) < 0.4:
                    break
                time.sleep(0.05)
            prog = [sgn * (v - x) for v in xs]
            over = max([0.0] + [p - dist for p in prog])
            # oscillation near the end: progression direction reversals in the last 40%
            tail = prog[int(0.6 * len(prog)):]
            revs = sum(1 for i in range(2, len(tail))
                       if (tail[i] - tail[i-1]) * (tail[i-1] - tail[i-2]) < -0.01)
            rows.append({"dist": round(dist, 1), "t": round(ts[-1], 2),
                         "over": round(over, 2), "revs": revs})
            print(f"  dp {dp} dd {dd} df {df}: dist {dist:.0f} in {ts[-1]:.2f}s over "
                  f"{over:.2f} revs {revs}")
            time.sleep(0.4)
        rig.stop()
        summ = {"kind": "drive", "dp": dp, "dd": dd, "df": df,
                "t_mean": round(_mean([r["t"] for r in rows]), 2),
                "over_max": max(r["over"] for r in rows),
                "revs_max": max(r["revs"] for r in rows),
                "volts": rig.b.voltage(), "tt": time.strftime("%H:%M:%S")}
        print(f"  => {summ}")
        log(summ)
        results.append(summ)
        time.sleep(0.5)
    return results


def tune_centripetal(rig, cells):
    print("== centripetal (canonical quadratic curve at speed) ==")
    results = []
    for cent in cells:
        rig.b.cmd("pedroPidf", cent=cent)
        time.sleep(0.4)
        rig.start("all")
        rows = []
        for k in range(2):
            # curve goes |d| forward then d left; start from a spot with room both ways
            cx, cy = rig.center()
            x, y, st = rig.pose()
            d = 13.0 if k % 2 == 0 else -13.0
            # place the robot toward the -x, -sign(d)*y corner-ish of center first
            rig.b.cmd("pedroHold", dx=round(cx - 6 - x, 1),
                      dy=round((cy - math.copysign(7, d)) - y, 1))
            time.sleep(2.2)
            x, y, st = rig.pose()
            hdg = math.radians(st["heading"]["deg"])
            fx, fy = math.cos(hdg), math.sin(hdg)
            lx, ly = -math.sin(hdg), math.cos(hdg)
            cpx = x + abs(d) * fx + d * lx
            cpy = y + abs(d) * fy + d * ly
            c1x, c1y = x + abs(d) * fx, y + abs(d) * fy
            if not (rig.inside(cpx, cpy) and rig.inside(c1x, c1y)):
                print(f"  cent {cent}: skip curve d {d:+.0f}, no room "
                      f"(cp {cpx:.0f},{cpy:.0f})")
                continue
            rig.b.cmd("pedroCurve", d=d, power=0.7)
            t0 = time.time()
            terrs = []
            while time.time() - t0 < 4.5:
                _, _, st2 = rig.pose()
                p = st2["pedro"]
                if p.get("terr") is not None:
                    terrs.append(p["terr"])
                if not p.get("busy") and time.time() - t0 > 1.0:
                    break
                time.sleep(0.06)
            rows.append({"d": d, "terr_max": round(max(terrs), 2) if terrs else None,
                         "terr_mean": round(_mean(terrs), 2) if terrs else None})
            print(f"  cent {cent} d {d:+.0f}: terr max {rows[-1]['terr_max']} "
                  f"mean {rows[-1]['terr_mean']}")
            time.sleep(0.4)
        rig.stop()
        if rows:
            summ = {"kind": "cent", "cent": cent,
                    "terr_max": max(r["terr_max"] for r in rows if r["terr_max"] is not None),
                    "terr_mean": round(_mean([r["terr_mean"] for r in rows
                                              if r["terr_mean"] is not None]), 2),
                    "volts": rig.b.voltage(), "t": time.strftime("%H:%M:%S")}
            print(f"  => {summ}")
            log(summ)
            results.append(summ)
        time.sleep(0.5)
    return results


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    rig = Rig()
    if what in ("trans", "all"):
        tune_translational(rig, [(0.125, 0.008), (0.19, 0.008), (0.19, 0.025), (0.26, 0.025)])
    if what in ("drive", "all"):
        tune_drive(rig, [(0.005, 0.00003, 0.13), (0.008, 0.00003, 0.13),
                         (0.008, 0.0001, 0.10), (0.012, 0.0001, 0.10)])
    if what in ("cent", "all"):
        tune_centripetal(rig, [0.0, 0.0005, 0.002, 0.004])
