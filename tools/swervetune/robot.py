"""One front door for driving the robot from a host session. See ROBOT_CONTROL.md at repo root.

Every ad-hoc ``urllib`` snippet a session writes is a fresh chance to forget the watchdog, retry a
drive command, skip the box check or miss an UNKNOWN COMMAND. This wraps the Swerve Bring-Up HTTP
API with those rules built in, so the same checks run every time.

    python robot.py check                 gates: reachable / live / started / pose / box; exit code says which failed
    python robot.py state [--json]        compact snapshot (or the raw /state JSON)
    python robot.py cmd ACTION [k=v ...]  send one command, then report what the OpMode said about it
    python robot.py drive --f F [--s S] [--t T] --sec SEC (--confirmed-floor | --on-blocks) [--cap 0.30] [--rec LABEL]
    python robot.py pull LABEL            fetch the recorder CSV into runs/ and print true loop rate
    python robot.py stop                  zero the drive and put the OpMode in IDLE (OpMode keeps running)
    python robot.py estop                 STOP the OpMode through FTC Dashboard (kills it; operator restarts)

Diagnostic tooling (tools/swervetune). It reads and commands the existing API only and changes
nothing the OpMode measures.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

from swervebench import BASE, BenchError, _archive, _get, _mean, parse_csv

# Actions the OpMode itself treats as motion (SwerveBringUp.isMotionCommand), plus pedroChain,
# which starts the follower on a path. These are sent exactly once: a retried drive or rawServo
# lands late and restarts a timer, and a duplicate is worse than a dropped packet.
MOTION = {
    "wireScan", "sweep", "pulseMotor", "spinServo", "nudge", "pidStep", "pidStepAll", "rawServo",
    "autoTune", "headingStep", "headingGoto", "drive", "calGoto", "calHome", "calPositional",
    "pedroStart", "pedroLine", "pedroHold", "pedroCurve", "pedroChain",
}

# Actions that move the pose frame and so destroy the safe-area box (CLAUDE.md rule 6).
CLEARS_BOX = {"resetImu", "odoConfig", "boxClear"}

# The OpMode's drive watchdog is 400 ms (SwerveBringUp.DRIVE_WATCHDOG_MS). 75 ms matches
# boxdrive.py and leaves room for five missed packets before it trips.
DRIVE_PERIOD_S = 0.075

# SWERVE_TASK.md speed cap until steering criteria hold. Above HARD_CAP needs --allow-fast.
DEFAULT_CAP = 0.30
HARD_CAP = 0.50

EXIT_OK, EXIT_UNREACHABLE, EXIT_NOT_LIVE, EXIT_NOT_STARTED, EXIT_NO_POSE, EXIT_NO_BOX = 0, 2, 3, 4, 5, 6
EXIT_REFUSED = 7


def state(retries: int = 2, timeout: float = 3.0) -> dict:
    return json.loads(_get("/state", timeout=timeout, retries=retries))


def send(action: str, **params) -> None:
    retries = 1 if action in MOTION else 4
    _get("/cmd", {"action": action, **params}, timeout=3.0, retries=retries)


def outcome(action: str, before_msg: str, window_s: float = 2.0) -> tuple[bool, str]:
    """Reads back what the OpMode said about a command.

    /swerve/cmd always answers {"ok":true} - it only queues. The verdict is in a later published
    snapshot's ``message`` (or an errors entry for an unknown action). /state is the LAST
    published snapshot, so a read straight after the command still shows the previous one - that
    is the bug that ended every drivecapture chunk after one sample. Poll for the message to
    change (identity), not for a fixed delay (timing).
    """
    t_end = time.time() + window_s
    st = state()
    while st.get("message", "") == before_msg and time.time() < t_end:
        time.sleep(0.1)
        st = state()
    msg = st.get("message", "")
    errs = " | ".join(str(e) for e in st.get("errors", []))
    bad_markers = ("UNKNOWN COMMAND", "REFUSED", "Press START", "first.", "needs ", "Cannot ",
                   "failed", "No pinpoint", "No pose", "No box", "not available")
    bad = any(m in msg for m in bad_markers) or f'UNKNOWN COMMAND "{action}"' in errs
    if msg == before_msg:
        msg += ("   (UNCONFIRMED: message did not change in 2 s - either the command repeats the "
                "last message or it has not been drained. Verify the effect in /state.)")
    return (not bad), msg


def summarize(st: dict) -> str:
    h, p, b, r = st.get("heading", {}), st.get("pose", {}), st.get("box", {}), st.get("rec", {})
    lines = [
        f"live={st.get('live')} started={st.get('started')} mode={st.get('mode')} "
        f"busy={st.get('busy')} volts={st.get('voltage')} publishHz={st.get('timing', {}).get('publishHz')}",
        f"pose ok={p.get('ok')} x={p.get('x')} y={p.get('y')} vx={p.get('vx')} vy={p.get('vy')}   "
        f"heading ok={h.get('ok')} deg={h.get('deg')} hold={h.get('hold')} xLock={st.get('xLock')}",
    ]
    if b.get("valid"):
        lines.append(
            f"box ARMED x {b.get('minX')}..{b.get('maxX')} y {b.get('minY')}..{b.get('maxY')} "
            f"({_span(b, 'X')} x {_span(b, 'Y')} in) clamped={b.get('clamped')}"
        )
    else:
        lines.append(f"box NOT ARMED (marked0={b.get('marked0')})")
    if p.get("ok") and b.get("valid"):
        lines.append(f"inside box: {_inside(p, b)}")
    lines.append(
        f"rec recording={r.get('recording')} runId={r.get('runId')} samples={r.get('samples')} "
        f"overflowed={r.get('overflowed')} label={r.get('label')!r}"
    )
    lines.append(f"message: {st.get('message', '')}")
    for e in st.get("errors", []):
        lines.append(f"ERROR: {e}")
    for pod in st.get("pods", []):
        lines.append(
            f"  pod {pod.get('i')} {pod.get('label', ''):<2} enc={pod.get('hasEnc')} "
            f"v={pod.get('volts')} wheel={pod.get('wheelDeg')} tgt={pod.get('tgtDeg')} "
            f"pwr={pod.get('cmdPower')}"
        )
    return "\n".join(lines)


def _span(b: dict, axis: str) -> str:
    try:
        return f"{float(b['max' + axis]) - float(b['min' + axis]):.1f}"
    except (KeyError, TypeError, ValueError):
        return "?"


def _inside(p: dict, b: dict) -> bool:
    try:
        return (float(b["minX"]) <= float(p["x"]) <= float(b["maxX"])
                and float(b["minY"]) <= float(p["y"]) <= float(b["maxY"]))
    except (KeyError, TypeError, ValueError):
        return False


def gates(need_box: bool) -> tuple[int, dict | None, str]:
    """The checks every motion command needs. Returns (exit code, state, reason)."""
    try:
        st = state()
    except BenchError as e:
        return EXIT_UNREACHABLE, None, (
            f"Robot unreachable at {BASE}: {e}. Is this laptop on the Control Hub WiFi? "
            f"Is the Robot Controller app running?")
    if not st.get("live"):
        return EXIT_NOT_LIVE, st, "Web server up but Swerve Bring-Up is not running (live=false)."
    if not st.get("started"):
        return EXIT_NOT_STARTED, st, "OpMode is in INIT. Motion commands are refused until START."
    if need_box and not st.get("pose", {}).get("ok"):
        return EXIT_NO_POSE, st, "No pose from the Pinpoint - translation is refused with a box armed."
    if need_box and not st.get("box", {}).get("valid"):
        return EXIT_NO_BOX, st, "Safe-area box is NOT armed. Operator must re-mark corners A and B."
    if need_box and not _inside(st["pose"], st["box"]):
        return EXIT_NO_BOX, st, "Pose is OUTSIDE the armed box - the frame or the box is wrong."
    return EXIT_OK, st, "all gates pass"


# ---- subcommands --------------------------------------------------------------------


def cmd_check(_a) -> int:
    code, st, reason = gates(need_box=True)
    if st is not None:
        print(summarize(st))
    print(("READY: " if code == EXIT_OK else "NOT READY: ") + reason)
    return code


def cmd_state(a) -> int:
    try:
        st = state()
    except BenchError as e:
        print(f"unreachable: {e}")
        return EXIT_UNREACHABLE
    print(json.dumps(st, indent=1) if a.json else summarize(st))
    return EXIT_OK


def cmd_cmd(a) -> int:
    params = {}
    for kv in a.params:
        if "=" not in kv:
            print(f"bad parameter {kv!r}: use key=value")
            return EXIT_REFUSED
        k, v = kv.split("=", 1)
        params[k] = v
    if a.action == "drive":
        print("Use `robot.py drive` - a one-shot drive command trips the 400 ms watchdog and "
              "leaves nobody holding the stop.")
        return EXIT_REFUSED
    if a.action in CLEARS_BOX and not a.clears_box_ok:
        print(f"{a.action} clears the safe-area box (new pose frame). Re-run with --clears-box-ok "
              "and put 're-mark corners A and B' in the next OPS REQUEST.")
        return EXIT_REFUSED
    if a.action in MOTION:
        code, _, reason = gates(need_box=a.action.startswith("pedro") or a.action == "headingGoto")
        if code != EXIT_OK:
            print(f"REFUSED {a.action}: {reason}")
            return code
    try:
        before = state().get("message", "")
        send(a.action, **params)
        ok, msg = outcome(a.action, before)
    except BenchError as e:
        print(f"transport failure: {e}")
        return EXIT_UNREACHABLE
    print(("OK   " if ok else "FAIL ") + msg)
    return EXIT_OK if ok else EXIT_REFUSED


def cmd_drive(a) -> int:
    if a.confirmed_floor == a.on_blocks:
        print("REFUSED: pass exactly one of --confirmed-floor (operator confirmed THIS session: on "
              "the floor, inside the taped box, area clear) or --on-blocks (operator confirmed: on "
              "blocks, chassis strapped, hands and leads clear of the wheels).")
        return EXIT_REFUSED
    cap = a.cap
    if cap > HARD_CAP and not a.allow_fast:
        print(f"REFUSED: cap {cap} > {HARD_CAP} needs --allow-fast (and a reason in the report).")
        return EXIT_REFUSED
    translating = abs(a.f) > 0 or abs(a.s) > 0
    # On blocks the pose cannot move, so the fence is irrelevant; on the floor it is mandatory.
    code, st, reason = gates(need_box=not a.on_blocks)
    if code != EXIT_OK:
        print(f"REFUSED drive: {reason}")
        return code
    f = max(-cap, min(cap, a.f))
    s = max(-cap, min(cap, a.s))
    t = max(-cap, min(cap, a.t))
    if (f, s, t) != (a.f, a.s, a.t):
        print(f"capped to f={f} s={s} t={t} (cap {cap})")
    if a.sec > 10:
        print("REFUSED: one drive call is at most 10 s. Chain calls, re-checking /state between.")
        return EXIT_REFUSED

    p0, h0, v0 = st["pose"], st["heading"].get("deg"), st.get("voltage")
    surface = "blocks" if a.on_blocks else "floor"
    print(f"drive f={f} s={s} t={t} for {a.sec}s  surface={surface}  volts={v0}  "
          f"start x={p0.get('x')} y={p0.get('y')} h={h0}")
    if a.rec:
        send("recStart", label=a.rec)
        time.sleep(0.15)

    clamped = 0
    fails = 0
    ticks = 0
    last = st
    t_end = time.time() + a.sec
    try:
        while time.time() < t_end:
            try:
                send("drive", f=round(f, 3), s=round(s, 3), t=round(t, 3))
                last = state(retries=1, timeout=0.5)
                fails = 0
                clamped += 1 if last.get("box", {}).get("clamped") else 0
                if translating and not a.on_blocks and not last.get("box", {}).get("valid"):
                    print("ABORT: box disarmed mid-drive.")
                    break
            except BenchError as e:
                fails += 1
                if fails >= 2:
                    print(f"ABORT: two consecutive transport failures mid-drive ({e}). The "
                          f"watchdog stops the robot within 400 ms of the last command. "
                          f"If it is still moving: operator presses STOP.")
                    break
            ticks += 1
            time.sleep(DRIVE_PERIOD_S)
    finally:
        # Always leave the robot commanded to zero, then idle. Send both even if one fails.
        for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
            try:
                _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
            except BenchError as e:
                print(f"WARNING: could not send {action} after the drive: {e}")
        if a.rec:
            time.sleep(0.2)
            try:
                _get("/cmd", {"action": "recStop"}, timeout=2.0, retries=4)
            except BenchError as e:
                print(f"WARNING: recStop failed: {e}")

    time.sleep(0.4)
    try:
        end = state()
        p1 = end["pose"]
        dx = float(p1["x"]) - float(p0["x"])
        dy = float(p1["y"]) - float(p0["y"])
        print(f"end x={p1['x']} y={p1['y']} h={end['heading'].get('deg')}  moved "
              f"{math.hypot(dx, dy):.1f} in (dx {dx:+.1f}, dy {dy:+.1f})  mode={end.get('mode')}  "
              f"volts={end.get('voltage')}")
    except (BenchError, KeyError, TypeError, ValueError) as e:
        print(f"could not read end state: {e}")
    print(f"ticks={ticks}  clamped_ticks={clamped}  "
          f"(a clamped tick means the fence modified the command - azimuth near an edge is suspect)")
    if a.rec:
        return _pull(a.rec)
    return EXIT_OK


def _pull(label: str) -> int:
    try:
        csv_text = _get("/rec.csv", timeout=30.0, retries=3)
        tr = parse_csv(csv_text)
    except BenchError as e:
        print(f"pull failed: {e}")
        return EXIT_UNREACHABLE
    path = _archive(csv_text, {"label": label})
    dts = [x for x in tr.get("dt", []) if x == x and x > 0]
    st = sorted(dts)
    if dts:
        print(f"saved tools/swervetune/{path}  n={len(dts)}  loop_hz_true={1 / _mean(dts):.1f}  "
              f"loop_dt_mean_ms={1000 * _mean(dts):.2f}  "
              f"loop_dt_p90_ms={1000 * st[int(0.9 * (len(st) - 1))]:.2f}")
    else:
        print(f"saved tools/swervetune/{path}  (no dt samples)")
    if "overflowed=true" in csv_text.split("\n", 1)[0]:
        print("NOTE: recorder hit 3000 samples and stopped - the tail of the run is missing.")
    return EXIT_OK


def cmd_pull(a) -> int:
    return _pull(a.label)


def cmd_stop(_a) -> int:
    rc = EXIT_OK
    for action, params in (("drive", {"f": 0, "s": 0, "t": 0}), ("stop", {})):
        try:
            _get("/cmd", {"action": action, **params}, timeout=2.0, retries=4)
        except BenchError as e:
            print(f"could not send {action}: {e}")
            rc = EXIT_UNREACHABLE
    if rc == EXIT_OK:
        time.sleep(0.3)
        try:
            print(f"mode={state().get('mode')}  message: {state().get('message')}")
        except BenchError:
            pass
    else:
        print("If the robot is moving: the operator presses STOP on the Driver Station, or run "
              "`python robot.py estop`.")
    return rc


def cmd_estop(_a) -> int:
    try:
        from ftcdash import stop_op_mode
        print(json.dumps(stop_op_mode(), indent=1)[:800])
        print("OpMode STOPPED. The HTTP server stays up but /state now reads live=false; the "
              "operator must re-start Swerve Bring-Up (and re-check the box) before any motion.")
        return EXIT_OK
    except Exception as e:  # noqa: BLE001 - report whatever went wrong, then escalate to a human
        print(f"estop via FTC Dashboard FAILED: {e}. Operator: press STOP on the Driver Station.")
        return EXIT_UNREACHABLE


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="sub", required=True)
    sub.add_parser("check").set_defaults(fn=cmd_check)
    p = sub.add_parser("state")
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_state)
    p = sub.add_parser("cmd")
    p.add_argument("action")
    p.add_argument("params", nargs="*")
    p.add_argument("--clears-box-ok", action="store_true")
    p.set_defaults(fn=cmd_cmd)
    p = sub.add_parser("drive")
    p.add_argument("--f", type=float, default=0.0, help="forward, robot frame, [-1, 1]")
    p.add_argument("--s", type=float, default=0.0, help="strafe, robot frame, [-1, 1]")
    p.add_argument("--t", type=float, default=0.0, help="turn, [-1, 1]")
    p.add_argument("--sec", type=float, required=True)
    p.add_argument("--cap", type=float, default=DEFAULT_CAP)
    p.add_argument("--allow-fast", action="store_true")
    p.add_argument("--confirmed-floor", action="store_true")
    p.add_argument("--on-blocks", action="store_true")
    p.add_argument("--rec", default=None, help="record the drive under this label and pull it")
    p.set_defaults(fn=cmd_drive)
    p = sub.add_parser("pull")
    p.add_argument("label")
    p.set_defaults(fn=cmd_pull)
    sub.add_parser("stop").set_defaults(fn=cmd_stop)
    sub.add_parser("estop").set_defaults(fn=cmd_estop)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
