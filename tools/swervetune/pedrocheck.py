"""Validate tuning THROUGH PEDRO - the follower that actually drives autos - on the bring-up bench.

The bring-up tool's own checks (pidStep, headingGoto, stick driving) run the tool's control laws:
its own heading hold (kD, sign feed-forward), its own mixer config, no Foresight. They are good
for SEARCHING gains and say nothing reliable about how the robot follows a path. Every gain is
accepted only on what this script measures. See TUNING_TASK.md.

    python pedrocheck.py suite --pods live|shipped --surface tiles --confirmed-floor --label NAME
                         [--n 3] [--arms arms.json] [--only hold,turn,line,curve] [--power 0.5]
    python pedrocheck.py path FILE.pp --line NAME|INDEX --pods shipped --surface tiles
                         --confirmed-floor --label NAME [--power 0.5] [--n 3]

--surface is the label on every result; the follower moves the robot, so the operator's
confirmation must also be passed explicitly: --confirmed-floor with tiles, --on-blocks with blocks.
    python pedrocheck.py report [--label NAME]

--pods live    the follower runs the TOOL's live pod calibration (validate before exporting)
--pods shipped the follower runs Constants.java as installed (acceptance; needs the deploy, and
               /state pedro.gainsShipped + staticsShipped true, or it is not an acceptance run)

Arms (--arms FILE): {"name": [["action", {params}], ...], ...}. Each arm's commands are sent
before its pedroStart, arms are randomised and interleaved per block (battery drifts), so an A/B
of follower gains is e.g. {"hp110": [["pedroPidf", {"hp": 1.10}]], "hp130": [["pedroPidf",
{"hp": 1.30}]]}. Pod gains: [["setPidf", {...}]] with --pods live.

Diagnostic tooling. It only drives through the bench's own validated commands; every target is
also checked robot-side against the fence and keep-outs.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import statistics
import subprocess
import sys
import time

from robot import EXIT_OK, EXIT_REFUSED, gates, outcome, send, state
from swervebench import BenchError, _archive, _get, parse_csv

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "current_runs", "pedrocheck.jsonl")

# Acceptance thresholds. GUESSES except where noted - TUNING_TASK.md puts each in Survey 0.
THRESHOLDS = {
    "hold_settle_s": 1.5,        # to within 1.0 in and staying there (guess)
    "hold_residual_in": 0.5,     # mean |error| over the last 0.5 s (guess)
    "hold_overshoot_in": 1.0,    # past the target along the step (guess)
    "turn_settle_s": 1.0,        # to within 2 deg (guess)
    "turn_overshoot_deg": 5.0,   # (guess; the tool-hold measured 2.3 at kD 0.08)
    "turn_residual_deg": 1.0,    # SWERVE_TASK criterion 8 (guess there too)
    "path_max_terr_in": 2.0,     # SWERVE_TASK criterion 9 (guess there too)
    "path_max_herr_deg": 3.0,    # SWERVE_TASK criterion 7 (guess there too)
    "path_end_err_in": 1.0,      # (guess)
}

SETTLE_AFTER_IDLE_S = 1.5
TRIAL_TIMEOUT_S = 8.0


# ---- small helpers ------------------------------------------------------------------------

def wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def finite(xs):
    return [x for x in xs if x == x and not math.isinf(x)]


def ci95(xs: list[float]) -> tuple[float, float]:
    xs = finite(xs)
    if not xs:
        return float("nan"), float("nan")
    if len(xs) == 1:
        return xs[0], float("nan")
    # t critical, two-sided 95%, small n
    t = {2: 12.71, 3: 4.30, 4: 3.18, 5: 2.78, 6: 2.57, 7: 2.45, 8: 2.36, 9: 2.31,
         10: 2.26}.get(len(xs), 2.0)
    return statistics.mean(xs), t * statistics.stdev(xs) / math.sqrt(len(xs))


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=HERE,
                                       text=True).strip()
    except Exception:  # noqa: BLE001
        return "UNKNOWN"


def cmd(action: str, **params) -> tuple[bool, str]:
    before = state().get("message", "")
    send(action, **params)
    return outcome(action, before, window_s=3.0)


def wait_idle(timeout_s: float = TRIAL_TIMEOUT_S) -> dict:
    """Polls until the follower reports not busy, then holds SETTLE_AFTER_IDLE_S."""
    t_end = time.time() + timeout_s
    st = state()
    time.sleep(0.25)
    while time.time() < t_end:
        st = state(retries=2, timeout=1.0)
        if not st.get("pedro", {}).get("active"):
            raise BenchError(f"follower dropped out: {st.get('message')}")
        if not st.get("pedro", {}).get("busy"):
            break
        time.sleep(0.1)
    time.sleep(SETTLE_AFTER_IDLE_S)
    return state()


def pose(st: dict) -> tuple[float, float, float]:
    p, h = st["pose"], st["heading"]
    return float(p["x"]), float(p["y"]), float(h["deg"])


# ---- scoring ------------------------------------------------------------------------------

def score_common(tr: dict) -> dict:
    dts = finite(tr.get("dt", []))
    out = {
        "loop_hz_true": 1.0 / statistics.mean(dts) if dts else float("nan"),
        "loop_dt_mean_ms": 1000 * statistics.mean(dts) if dts else float("nan"),
        "loop_dt_p90_ms": 1000 * sorted(dts)[int(0.9 * (len(dts) - 1))] if dts else float("nan"),
        "volts_mean": statistics.mean(finite(tr.get("volts", []))) if finite(tr.get("volts", []))
        else float("nan"),
        "samples": len(tr.get("t", [])),
    }
    # Pod tracking WHILE PEDRO DRIVES - the steering numbers that matter for a match.
    for i in range(4):
        e = [abs(x) for x in finite(tr.get(f"p{i}_err", []))]
        fl = finite(tr.get(f"p{i}_flip", []))
        if e:
            out[f"pod{i}_err_mean_deg"] = statistics.mean(e)
            out[f"pod{i}_err_p95_deg"] = sorted(e)[int(0.95 * (len(e) - 1))]
        out[f"pod{i}_flips"] = int(sum(1 for k in range(1, len(fl)) if fl[k] > 0.5 >= fl[k - 1]))
    return out


def settle_time(t: list[float], err: list[float], band: float) -> float:
    """First time after which |err| stays inside band for the rest of the record."""
    last_out = None
    for ti, e in zip(t, err):
        if e != e:
            continue
        if abs(e) > band:
            last_out = ti
    if last_out is None:
        return 0.0
    if last_out >= t[-1]:
        return float("nan")
    return last_out - t[0]


def score_hold(tr: dict, x0, y0, h0, dx, dy, dh) -> dict:
    t = tr["t"]
    tx, ty, th = x0 + dx, y0 + dy, h0 + dh
    err = [math.hypot(x - tx, y - ty) for x, y in zip(tr["px"], tr["py"])]
    herr = [wrap180(h - th) for h in tr["heading"]]
    out = score_common(tr)
    tail = [e for ti, e in zip(t, err) if ti >= t[-1] - 0.5 and e == e]
    htail = [abs(e) for ti, e in zip(t, herr) if ti >= t[-1] - 0.5 and e == e]
    out["settle_1in_s"] = settle_time(t, err, 1.0)
    out["residual_in"] = statistics.mean(tail) if tail else float("nan")
    out["heading_settle_2deg_s"] = settle_time(t, herr, 2.0)
    out["heading_residual_deg"] = statistics.mean(htail) if htail else float("nan")
    if dx or dy:
        ux, uy = dx / math.hypot(dx, dy), dy / math.hypot(dx, dy)
        along = [((x - tx) * ux + (y - ty) * uy) for x, y in zip(tr["px"], tr["py"])]
        out["overshoot_in"] = max(0.0, max(finite(along), default=0.0))
    if dh:
        s = 1 if dh > 0 else -1
        out["heading_overshoot_deg"] = max(0.0, max((s * e for e in finite(herr)), default=0.0))
    return out


def score_path(tr: dict, end_x, end_y, end_h) -> dict:
    out = score_common(tr)
    busy = [b for b in tr.get("fbusy", [])]
    t = tr["t"]
    following = [i for i, b in enumerate(busy) if b == 1]
    terr = [abs(tr["fterr"][i]) for i in following if tr["fterr"][i] == tr["fterr"][i]]
    herr = [abs(tr["fherr"][i]) for i in following if tr["fherr"][i] == tr["fherr"][i]]
    out["max_terr_in"] = max(terr) if terr else float("nan")
    out["mean_terr_in"] = statistics.mean(terr) if terr else float("nan")
    out["max_herr_deg"] = max(herr) if herr else float("nan")
    out["duration_s"] = (t[following[-1]] - t[following[0]]) if following else float("nan")
    tail = [i for i, ti in enumerate(t) if ti >= t[-1] - 0.5]
    ex = statistics.mean(finite([tr["px"][i] for i in tail]) or [float("nan")])
    ey = statistics.mean(finite([tr["py"][i] for i in tail]) or [float("nan")])
    eh = statistics.mean(finite([tr["heading"][i] for i in tail]) or [float("nan")])
    out["end_err_in"] = math.hypot(ex - end_x, ey - end_y)
    out["end_heading_err_deg"] = abs(wrap180(eh - end_h)) if end_h is not None else float("nan")
    return out


def verdict(kind: str, m: dict) -> list[str]:
    T, fails = THRESHOLDS, []

    def chk(key, limit):
        v = m.get(key)
        if v is None:
            return
        if v != v or v > limit:
            fails.append(f"{key}={v:.3g} > {limit}" if v == v else f"{key}=NaN (never settled)")

    if kind.startswith("hold"):
        chk("settle_1in_s", T["hold_settle_s"])
        chk("residual_in", T["hold_residual_in"])
        chk("overshoot_in", T["hold_overshoot_in"])
    elif kind.startswith("turn"):
        chk("heading_settle_2deg_s", T["turn_settle_s"])
        chk("heading_overshoot_deg", T["turn_overshoot_deg"])
        chk("heading_residual_deg", T["turn_residual_deg"])
    else:
        chk("max_terr_in", T["path_max_terr_in"])
        chk("max_herr_deg", T["path_max_herr_deg"])
        chk("end_err_in", T["path_end_err_in"])
    return fails


# ---- running trials -----------------------------------------------------------------------

def stop_follower():
    """pedroStop straight to /cmd, retried. It used to go through cmd(), whose state() read comes
    first - if that read failed the stop was never sent at all, from a finally block whose whole
    job is to stop the robot. pedroStop is idempotent, so retries are safe."""
    try:
        _get("/cmd", {"action": "pedroStop"}, timeout=2.0, retries=4)
    except BenchError as e:
        print(f"could not send pedroStop ({e}) - operator: STOP on the Driver Station.")


class Session:
    def __init__(self, a):
        self.a = a
        # --surface is a label for the record; the follower drives the robot, so the operator's
        # confirmation of where it is must be asserted explicitly, as robot.py drive requires
        # (CLAUDE.md rule 8), and it must agree with the label.
        if a.confirmed_floor == a.on_blocks:
            raise SystemExit("REFUSED: pass exactly one of --confirmed-floor (operator confirmed "
                             "THIS session: on the floor, inside the fence, area clear) or "
                             "--on-blocks (operator confirmed: on blocks, strapped, hands clear).")
        if (a.surface == "tiles") != bool(a.confirmed_floor):
            raise SystemExit(f"REFUSED: --surface {a.surface} contradicts "
                             f"{'--confirmed-floor' if a.confirmed_floor else '--on-blocks'}.")
        code, st, reason = gates(need_box=True)
        if code != EXIT_OK:
            raise SystemExit(f"NOT READY: {reason}")
        self.st = st
        self.box = st["box"]
        self.home = None

    def start(self, arm_cmds, reset=False):
        if reset:
            ok, msg = cmd("pedroReset")
            if not ok:
                raise BenchError(f"pedroReset failed: {msg}")
        for action, params in arm_cmds:
            ok, msg = cmd(action, **params)
            if not ok:
                raise BenchError(f"arm command {action} {params} failed: {msg}")
        ok, msg = cmd("pedroStart", activate="all", pods=self.a.pods)
        if not ok:
            raise BenchError(f"pedroStart failed: {msg}")
        st = state()
        pe = st.get("pedro", {})
        if self.a.pods == "shipped" and not (pe.get("gainsShipped") and pe.get("staticsShipped")):
            print("WARNING: pods=shipped but gainsShipped/staticsShipped is false - the bench is "
                  "NOT running what is installed. This is not an acceptance run.")
        if self.home is None:
            self.home = pose(st)

    def go_home(self):
        x, y, h = pose(state())
        hx, hy, hh = self.home
        ok, msg = cmd("pedroHold", dx=round(hx - x, 3), dy=round(hy - y, 3),
                      dh=round(wrap180(hh - h), 3))
        if not ok:
            raise BenchError(f"re-centre refused: {msg}")
        wait_idle()

    def trial(self, label, kind, action, params, scorer):
        st0 = state()
        x0, y0, h0 = pose(st0)
        send("recStart", label=label)
        time.sleep(0.2)
        ok, msg = cmd(action, **params)
        if not ok:
            send("recStop")
            return {"kind": kind, "refused": msg}
        try:
            wait_idle()
        finally:
            send("recStop")
            time.sleep(0.2)
        csv_text = _get("/rec.csv", timeout=30.0, retries=3)
        path = _archive(csv_text, {"label": label})
        tr = parse_csv(csv_text)
        m = scorer(tr, x0, y0, h0)
        return {"kind": kind, "params": params, "csv": path, "metrics": m,
                "fails": verdict(kind, m)}


def suite_trials(a):
    S, L, D, T, P = a.step, a.line, a.curve, a.turn, a.power
    only = set(a.only.split(",")) if a.only else {"hold", "turn", "line", "curve"}
    trials = []
    if "hold" in only:
        for name, dx, dy in (("hold+x", S, 0), ("hold-x", -S, 0), ("hold+y", 0, S),
                             ("hold-y", 0, -S)):
            trials.append((name, "pedroHold", {"dx": dx, "dy": dy},
                           lambda tr, x0, y0, h0, dx=dx, dy=dy:
                           score_hold(tr, x0, y0, h0, dx, dy, 0)))
    if "turn" in only:
        for name, dh in (("turn+", T), ("turn-", -T)):
            trials.append((name, "pedroHold", {"dh": dh},
                           lambda tr, x0, y0, h0, dh=dh: score_hold(tr, x0, y0, h0, 0, 0, dh)))
    if "line" in only:
        for name, dx, dy in (("line+x", L, 0), ("line+y", 0, L)):
            trials.append((name, "pedroLine", {"dx": dx, "dy": dy, "power": P},
                           lambda tr, x0, y0, h0, dx=dx, dy=dy:
                           score_path(tr, x0 + dx, y0 + dy, h0)))
    if "curve" in only:
        for name, d in (("curve+", D), ("curve-", -D)):
            def sc(tr, x0, y0, h0, d=d):
                hr = math.radians(h0)
                ad = abs(d)
                ex = x0 + ad * math.cos(hr) - d * math.sin(hr)
                ey = y0 + ad * math.sin(hr) + d * math.cos(hr)
                return score_path(tr, ex, ey, h0)
            trials.append((name, "pedroCurve", {"d": d, "power": P}, sc))
    return trials


def log(rec: dict):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec) + "\n")


def run_blocks(a, trials, arms):
    s = Session(a)
    results = []
    try:
        for block in range(a.n):
            arm_order = list(arms.items())
            random.shuffle(arm_order)
            for arm_name, arm_cmds in arm_order:
                # A/B: every arm starts from the installed gains. Single run: whatever is live.
                s.start(arm_cmds, reset=bool(a.arms))
                order = list(trials)
                random.shuffle(order)
                for name, action, params, scorer in order:
                    s.go_home()
                    label = f"{a.label}:{arm_name}:{name}:b{block}"
                    r = s.trial(label, name, action, params, scorer)
                    st = state()
                    rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label,
                           "arm": arm_name, "block": block, "pods": a.pods,
                           "surface": a.surface, "volts": st.get("voltage"),
                           "fence": st.get("box", {}).get("kind"),
                           "gainsShipped": st.get("pedro", {}).get("gainsShipped"),
                           "staticsShipped": st.get("pedro", {}).get("staticsShipped"),
                           "host_head": git_head(), **r}
                    log(rec)
                    results.append(rec)
                    tag = "REFUSED" if "refused" in r else ("PASS" if not r["fails"] else "FAIL")
                    print(f"[{tag}] {label}  " + (r.get("refused") or "; ".join(r["fails"])
                                                    or "all thresholds met"))
    finally:
        stop_follower()
    summarize(results)
    return EXIT_OK if all(not r.get("fails") and "refused" not in r for r in results) \
        else EXIT_REFUSED


def summarize(results):
    groups = {}
    for r in results:
        if "metrics" in r:
            groups.setdefault((r["arm"], r["kind"]), []).append(r["metrics"])
    keys = ["settle_1in_s", "residual_in", "overshoot_in", "heading_settle_2deg_s",
            "heading_overshoot_deg", "heading_residual_deg", "max_terr_in", "max_herr_deg",
            "end_err_in", "duration_s", "loop_hz_true"]
    print("\narm / trial                       n  metric = mean +/- 95% CI half-width")
    for (arm, kind), ms in sorted(groups.items()):
        parts = []
        for k in keys:
            vals = [m[k] for m in ms if k in m]
            if vals:
                mu, hw = ci95(vals)
                parts.append(f"{k}={mu:.3g}+/-{hw:.2g}")
        pods = [statistics.mean(finite([m.get(f'pod{i}_err_p95_deg', float('nan'))
                                        for m in ms]) or [float('nan')]) for i in range(4)]
        parts.append("pod_err_p95=" + "/".join(f"{p:.2g}" for p in pods))
        print(f"{arm + ' / ' + kind:<32} {len(ms):>2}  " + "  ".join(parts))


# ---- .pp paths ----------------------------------------------------------------------------

def pp_line(doc: dict, sel: str, start_override: str | None = None):
    lines = doc["lines"]
    idx = None
    if sel.isdigit():
        idx = int(sel)
    else:
        for i, ln in enumerate(lines):
            if ln.get("name") == sel or ln.get("id") == sel:
                idx = i
    if idx is None or not 0 <= idx < len(lines):
        names = ", ".join(f"{i}:{ln.get('name')}" for i, ln in enumerate(lines))
        raise SystemExit(f"no line {sel!r}. Lines: {names}")
    ln = lines[idx]
    # The visualizer draws line i from line i-1's end. A line inside a conditional branch (e.g.
    # RedPollenAuto "6b - Park from the flower") really starts somewhere else: pass --start.
    start = doc["startPoint"] if idx == 0 else lines[idx - 1]["endPoint"]
    if start_override:
        sx, sy, *_ = (float(v) for v in start_override.split(","))
        start = {"x": sx, "y": sy}
    pts = [(start["x"], start["y"])] + [(c["x"], c["y"]) for c in ln.get("controlPoints", [])] \
        + [(ln["endPoint"]["x"], ln["endPoint"]["y"])]
    end = ln["endPoint"]
    mode = end.get("heading", "tangential")
    if mode == "linear":
        h0, h1 = float(end["startDeg"]), float(end["endDeg"])
        head = f"param:{h0}:{h1}:{float(end.get('headingCurve', 1) or 1)}"
    elif mode == "constant":
        h0 = h1 = float(end.get("degrees", 0))
        head = f"constant:{h0}"
    else:
        rev = bool(end.get("reverse"))
        tx, ty = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        h0 = math.degrees(math.atan2(ty, tx)) + (180 if rev else 0)
        n = len(pts)
        ex, ey = pts[n - 1][0] - pts[n - 2][0], pts[n - 1][1] - pts[n - 2][1]
        h1 = math.degrees(math.atan2(ey, ex)) + (180 if rev else 0)
        head = "reverseTangent" if rev else "tangent"
    return ln.get("name", f"line{idx}"), pts, head, wrap180(h0), wrap180(h1)


def cmd_path(a) -> int:
    with open(a.file, encoding="utf-8") as fh:
        doc = json.load(fh)
    name, pts, head, h0, h1 = pp_line(doc, a.line, a.start)
    if a.start and len(a.start.split(",")) > 2:
        h0 = float(a.start.split(",")[2])
    print(f"{name}: start ({pts[0][0]:.2f}, {pts[0][1]:.2f}) heading {h0:.1f}, "
          f"{len(pts)} control points, heading mode {head}, end ({pts[-1][0]:.2f}, "
          f"{pts[-1][1]:.2f}) heading {h1:.1f}")
    st = state()
    if st.get("box", {}).get("kind") != "field":
        print("REFUSED: .pp paths are in FIELD coordinates. Arm the field fence first "
              "(ROBOT_CONTROL.md 3.4.1) so the pose frame is the visualizer's.")
        return EXIT_REFUSED
    s = Session(a)
    results = []
    try:
        for block in range(a.n):
            s.start([])
            s.home = (pts[0][0], pts[0][1], h0)
            s.go_home()
            x, y, h = pose(state())
            if math.hypot(x - pts[0][0], y - pts[0][1]) > 2.0 or abs(wrap180(h - h0)) > 5:
                raise BenchError(f"could not reach the path start ({x:.1f}, {y:.1f}, {h:.1f}) vs "
                                 f"({pts[0][0]:.1f}, {pts[0][1]:.1f}, {h0:.1f})")
            label = f"{a.label}:{name}:b{block}"
            ptxt = ";".join(f"{px:.3f},{py:.3f}" for px, py in pts)
            r = s.trial(label, "path", "pedroChain", {"pts": ptxt, "head": head,
                                                       "power": a.power},
                        lambda tr, x0, y0, hh: score_path(tr, pts[-1][0], pts[-1][1], h1))
            st = state()
            rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "label": a.label, "arm": name,
                   "block": block, "pods": a.pods, "surface": a.surface,
                   "volts": st.get("voltage"), "file": os.path.basename(a.file),
                   "gainsShipped": st.get("pedro", {}).get("gainsShipped"),
                   "staticsShipped": st.get("pedro", {}).get("staticsShipped"),
                   "host_head": git_head(), **r}
            log(rec)
            results.append(rec)
            print(f"[{'PASS' if not r.get('fails') and 'refused' not in r else 'FAIL'}] {label} "
                  + (r.get("refused") or "; ".join(r.get("fails", [])) or "all thresholds met"))
    finally:
        stop_follower()
    summarize(results)
    return EXIT_OK if all(not r.get("fails") and "refused" not in r for r in results) \
        else EXIT_REFUSED


def cmd_suite(a) -> int:
    arms = {"base": []}
    if a.arms:
        with open(a.arms, encoding="utf-8") as fh:
            arms = json.load(fh)
    return run_blocks(a, suite_trials(a), arms)


def cmd_report(a) -> int:
    if not os.path.exists(LOG):
        print("no pedrocheck runs yet")
        return EXIT_OK
    with open(LOG, encoding="utf-8") as fh:
        recs = [json.loads(ln) for ln in fh if ln.strip()]
    if a.label:
        recs = [r for r in recs if r.get("label") == a.label]
    summarize(recs)
    return EXIT_OK


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="sub", required=True)

    def common(p):
        p.add_argument("--pods", choices=["live", "shipped"], required=True)
        p.add_argument("--surface", choices=["tiles", "blocks"], required=True,
                       help="CLAUDE.md rule 7 - every result carries it")
        p.add_argument("--confirmed-floor", action="store_true",
                       help="operator confirmed THIS session: on the floor, in the fence, clear "
                            "(required with --surface tiles; CLAUDE.md rule 8)")
        p.add_argument("--on-blocks", action="store_true",
                       help="operator confirmed: on blocks, strapped (required with --surface "
                            "blocks)")
        p.add_argument("--label", required=True)
        p.add_argument("--n", type=int, default=3, help="blocks (repeats of every trial)")
        p.add_argument("--power", type=float, default=0.5,
                       help="Foresight maxPathSpeed fraction for lines/curves/paths")

    p = sub.add_parser("suite")
    common(p)
    p.add_argument("--arms", default=None)
    p.add_argument("--only", default=None, help="comma list of hold,turn,line,curve")
    p.add_argument("--step", type=float, default=12.0, help="hold step, in")
    p.add_argument("--turn", type=float, default=90.0, help="turn step, deg")
    p.add_argument("--line", type=float, default=24.0, help="line length, in")
    p.add_argument("--curve", type=float, default=18.0, help="curve size, in")
    p.set_defaults(fn=cmd_suite)

    p = sub.add_parser("path")
    common(p)
    p.add_argument("file")
    p.add_argument("--line", required=True, help="line name, id or index in the .pp")
    p.add_argument("--start", default=None,
                   help="x,y[,headingDeg] - the path's real start when it is not the previous "
                        "line's end (branches)")
    p.set_defaults(fn=cmd_path)

    p = sub.add_parser("report")
    p.add_argument("--label", default=None)
    p.set_defaults(fn=cmd_report)

    a = ap.parse_args()
    if getattr(a, "power", 0.5) > 0.8:
        print("REFUSED: --power above 0.8 needs a reason in the report; edit the cap if you have "
              "one.")
        return EXIT_REFUSED
    try:
        return a.fn(a)
    except BenchError as e:
        print(f"ABORTED: {e}. Sending stop.")
        try:
            _get("/cmd", {"action": "pedroStop"}, timeout=2.0, retries=4)
        except BenchError:
            print("could not send pedroStop - operator: STOP on the Driver Station.")
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
