"""Rapid autonomous driving inside the saved bounding box, scored for pod oscillation.

The firmware fence (applyBoxLimit) is the hard backstop; this driver additionally aims only at
points well inside the box, so the fence should rarely fire. Each burst is ~25 s of mixed
maneuvers - dashes to random waypoints, strafes, arcs, abrupt reversals, stop transitions -
recorded at loop rate and scored per pod:

  eng_rev/s   wheel direction reversals per second while that pod's servo is engaged
              (|pwr| > 0.05): the judder metric
  |err| rms   pod tracking error while engaged
  fence       how many samples the firmware clamp was active (should be near zero)

    python boxdrive.py burst <tag> <power> [seconds]
    python boxdrive.py report

Results append to current_runs/boxdrive.jsonl; raw chunks land in runs/.
"""

from __future__ import annotations

import json
import math
import os
import random
import sys
import time

from swervebench import Bench, parse_csv, _mean, _archive

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_runs", "boxdrive.jsonl")
SEED = 20260814
INSET_IN = 10.0          # aim only this far inside the fence
ARRIVE_IN = 7.0
CMD_PERIOD_S = 0.075


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class Driver:
    def __init__(self, b: Bench):
        self.b = b
        st = b.state()
        box = st["box"]
        if not box.get("valid"):
            raise SystemExit("No valid box - mark it on the dashboard first.")
        self.x0 = box["minX"] + INSET_IN
        self.x1 = box["maxX"] - INSET_IN
        self.y0 = box["minY"] + INSET_IN
        self.y1 = box["maxY"] - INSET_IN
        if self.x1 - self.x0 < 6 or self.y1 - self.y0 < 6:
            raise SystemExit("Box too small after inset.")
        self.rng = random.Random(SEED)

    def pose(self):
        st = self.b.state()
        p = st["pose"]
        h = st["heading"]["deg"]
        return p["x"], p["y"], math.radians(h), st["box"]["clamped"]

    def cmd(self, f, s, t=0.0):
        self.b.cmd("drive", retries=1, f=round(f, 3), s=round(s, 3), t=round(t, 3))

    def drive_to(self, tx, ty, power, timeout_s, turn=0.0):
        """Field-frame point seek at fixed power. Returns fence-hit count."""
        fence = 0
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            x, y, h, clamped = self.pose()
            fence += 1 if clamped else 0
            dx, dy = tx - x, ty - y
            if math.hypot(dx, dy) < ARRIVE_IN:
                break
            mag = math.hypot(dx, dy)
            ux, uy = dx / mag, dy / mag
            # field -> robot frame
            f = (ux * math.cos(h) + uy * math.sin(h)) * power
            s = (-ux * math.sin(h) + uy * math.cos(h)) * power
            self.cmd(f, s, turn)
            time.sleep(CMD_PERIOD_S)
        return fence

    def rand_point(self):
        return (self.rng.uniform(self.x0, self.x1), self.rng.uniform(self.y0, self.y1))

    def burst(self, power, seconds):
        """Mixed maneuvers for ~seconds. Returns fence-hit count."""
        fence = 0
        t_end = time.time() + seconds
        while time.time() < t_end:
            kind = self.rng.random()
            tx, ty = self.rand_point()
            if kind < 0.45:
                # plain dash
                fence += self.drive_to(tx, ty, power, 2.5)
            elif kind < 0.65:
                # dash with rotation mixed in (arc)
                fence += self.drive_to(tx, ty, power, 2.5,
                                       turn=self.rng.choice([-0.35, 0.35]))
            elif kind < 0.85:
                # abrupt reversal: dash, then immediately dash back the way we came
                x, y, _, _ = self.pose()
                fence += self.drive_to(tx, ty, power, 1.2)
                fence += self.drive_to(x, y, power, 1.2)
            else:
                # stop transition: full stop long enough for the X handover, then resume
                self.cmd(0, 0, 0)
                time.sleep(0.8)
        self.cmd(0, 0, 0)
        return fence


def score_chunk(csv_text):
    tr = parse_csv(csv_text)
    t = tr["t"]
    dts = [x for x in tr["dt"] if x == x and x > 0]
    rows = []
    for i in range(4):
        wheel = tr[f"p{i}_wheel"]
        pwr = tr[f"p{i}_pwr"]
        err = [abs(x) for x in tr[f"p{i}_err"] if x == x]
        span = 0.0
        rev = 0
        d = 0
        ext = None
        prev = None
        for k in range(len(wheel)):
            if wheel[k] != wheel[k] or pwr[k] != pwr[k]:
                continue
            engaged = abs(pwr[k]) > 0.05
            if prev is not None and engaged:
                span += t[k] - t[prev]
                x = wheel[k]
                if ext is None:
                    ext = x
                dxw = wrap180(x - ext)
                if d >= 0 and dxw > 0:
                    ext = x
                    d = 1
                elif d <= 0 and dxw < 0:
                    ext = x
                    d = -1
                elif d == 1 and dxw < -0.5:
                    rev += 1
                    d = -1
                    ext = x
                elif d == -1 and dxw > 0.5:
                    rev += 1
                    d = 1
                    ext = x
            if not engaged:
                ext = None
                d = 0
            prev = k
        rows.append({
            "pod": i,
            "eng_rev_s": round(rev / max(0.1, span), 2),
            "eng_s": round(span, 1),
            "err_rms": round(math.sqrt(sum(x * x for x in err) / len(err)), 2) if err else None,
        })
    return rows, (1.0 / _mean(dts)) if dts else float("nan")


def run_burst(tag, power, seconds):
    b = Bench()
    d = Driver(b)
    st = b.state()
    gains = [{k: p.get(k) for k in ("kp", "kd", "ks", "ksband")} for p in st["pods"]]
    print(f"{tag}: power {power}, {seconds}s, V {st['voltage']:.2f}, "
          f"box x {d.x0:.0f}..{d.x1:.0f} y {d.y0:.0f}..{d.y1:.0f} (inset)")
    b.cmd("setPublishHz", value=10)
    b.cmd("recStart", label=f"boxdrive-{tag}")
    time.sleep(0.2)
    fence = d.burst(power, seconds)
    time.sleep(0.3)
    b.cmd("recStop")
    time.sleep(0.3)
    b.cmd("setPublishHz", value=20)
    b.cmd("stop")

    csv_text = b.rec_csv()
    rows, hz = score_chunk(csv_text)
    out = {"tag": tag, "power": power, "t": time.strftime("%H:%M:%S"),
           "volts": st["voltage"], "hz": round(hz, 1), "fence_hits": fence,
           "gains": gains, "pods": rows}
    out["trace_file"] = _archive(csv_text, {"label": f"boxdrive-{tag}"})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(out) + "\n")

    print(f"  loop {hz:.0f} Hz true, fence hits {fence}")
    for r in rows:
        print(f"  pod {r['pod']}: engaged {r['eng_s']}s  rev/s {r['eng_rev_s']}  "
              f"|err| rms {r['err_rms']}")
    return out


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "report"
    if mode == "burst":
        tag = sys.argv[2]
        power = float(sys.argv[3])
        seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0
        run_burst(tag, power, seconds)
    else:
        for line in open(OUT, encoding="utf-8"):
            r = json.loads(line)
            worst = max(p["eng_rev_s"] for p in r["pods"])
            errs = [p["err_rms"] for p in r["pods"] if p["err_rms"] is not None]
            print(f"{r['tag']:<22} pow {r['power']:.2f} V {r['volts']:.2f} "
                  f"hz {r['hz']} fence {r['fence_hits']} worst rev/s {worst} "
                  f"err rms {max(errs) if errs else '--'}")
