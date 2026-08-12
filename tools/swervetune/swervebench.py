"""Client and scorer for the Swerve Bring-Up dashboard.

Talks to the Robot Controller over its own web server, drives pod-rotation experiments, pulls the
loop-rate recording back as CSV and scores it against the pod rotation acceptance criteria.

Standard library only, deliberately: this has to run on whatever laptop is on the robot's WiFi at
a competition, with no pip install.

Usage as a library::

    from swervebench import Bench
    b = Bench()
    r = b.step_trial(step_deg=90, label="baseline")
    print(r["pods"][0]["settle_1_0"])

The units throughout are degrees, seconds and normalised servo power in [-1, 1].
"""

from __future__ import annotations

import gzip
import json
import math
import os
import time
import urllib.parse
import urllib.request
from typing import Any

HOST = os.environ.get("SWERVE_HOST", "192.168.43.1:8080")
BASE = f"http://{HOST}/swerve"

RUN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")
TRIAL_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trials.jsonl")

POD_COUNT = 4

# Encoder read granularity: the Lynx reports analog in whole millivolts, and the configured spans
# are about 3.20 V over 360 degrees. Anything below this is not a measurement.
ENCODER_LSB_DEG = 1.0 / (3200.0 / 360.0)


class BenchError(RuntimeError):
    pass


def _get(path: str, params: dict[str, Any] | None = None, timeout: float = 5.0) -> str:
    url = BASE + path
    if params:
        url += "?" + urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.read().decode("utf-8", "replace")


class Bench:
    """A live connection to the bring-up OpMode."""

    def __init__(self, require_live: bool = True):
        if require_live:
            st = self.state()
            if not st.get("live"):
                raise BenchError(
                    "OpMode is not publishing. Start 'Swerve Bring-Up' (Diagnostics group), "
                    "e.g. from FTC Dashboard at http://192.168.43.1:8080/dash"
                )
            if not st.get("started"):
                raise BenchError("OpMode is in INIT. Press START before running motion commands.")

    # ---- transport ---------------------------------------------------------------

    def state(self) -> dict:
        return json.loads(_get("/state"))

    def cmd(self, action: str, **params) -> None:
        _get("/cmd", {"action": action, **params})

    def rec_csv(self) -> str:
        # Big payload; the hub takes a moment to render 3000 rows.
        return _get("/rec.csv", timeout=30.0)

    # ---- convenience -------------------------------------------------------------

    def voltage(self) -> float:
        return float(self.state()["voltage"])

    def wheel_degs(self) -> list[float]:
        return [p["wheelDeg"] for p in self.state()["pods"]]

    def set_pidf(
        self, kp=None, ki=None, kd=None, kf=None, cache=None, ks=None, ksband=None,
        dom=None, kilimit=None, kiband=None, kireset=None,
        pulsed=None, pband=None, ptol=None, ppow=None, pms=None, pcoast=None, scope="all"
    ) -> dict:
        """Sets gains and returns the state afterwards, so the caller can verify they took."""
        self.cmd(
            "setPidf", kp=kp, ki=ki, kd=kd, kf=kf, cache=cache, ks=ks, ksband=ksband,
            dom=None if dom is None else str(bool(dom)).lower(),
            kilimit=kilimit, kiband=kiband, kireset=kireset,
            pulsed=None if pulsed is None else str(bool(pulsed)).lower(),
            pband=pband, ptol=ptol, ppow=ppow, pms=pms, pcoast=pcoast, scope=scope,
        )
        time.sleep(0.25)
        return self.state()

    def stop(self) -> None:
        self.cmd("stop")

    def settle_at(self, deg: float, seconds: float = 1.5) -> None:
        """Parks every pod at one absolute wheel heading so a step starts from a known state."""
        self.cmd("pidStepAll", deg=deg)
        time.sleep(seconds)

    # ---- experiments -------------------------------------------------------------

    def step_trial(
        self,
        step_deg: float,
        base_deg: float = 0.0,
        hold_s: float = 5.0,
        pre_settle_s: float = 1.5,
        label: str = "",
        notes: dict | None = None,
        save: bool = True,
    ) -> dict:
        """Parks the pods, steps them by ``step_deg``, records at loop rate and scores the result.

        ``step_deg`` is a *relative* step from ``base_deg``; the underlying command takes an
        absolute wheel heading, which is an easy way to accidentally measure a 4 degree move and
        call it a 90 degree one.
        """
        st = self.state()
        gains = _gains_of(st)
        volts_before = st["voltage"]

        self.settle_at(base_deg, pre_settle_s)

        self.cmd("recStart", label=label or f"step{step_deg:g}")
        time.sleep(0.15)  # a few samples of the pre-step hold, so onset detection has a baseline
        self.cmd("pidStepAll", deg=base_deg + step_deg)
        time.sleep(hold_s)
        self.cmd("recStop")
        time.sleep(0.2)
        self.cmd("stop")

        csv_text = self.rec_csv()
        trace = parse_csv(csv_text)
        result = score_step(trace, expected_step_deg=abs(step_deg))

        result["label"] = label
        result["step_deg_commanded"] = step_deg
        result["base_deg"] = base_deg
        result["gains"] = gains
        result["voltage_before"] = volts_before
        result["notes"] = notes or {}
        result["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S")

        if save:
            result["trace_file"] = _archive(csv_text, result)
            _append_trial(result)
        return result

    def raw_staircase(
        self,
        pod: int,
        powers: list[float],
        dwell_s: float = 0.45,
        gap_s: float = 0.35,
        label: str = "",
    ) -> dict:
        """Applies a rising sequence of open-loop powers to one pod, recording throughout.

        This is how breakaway and deadband get measured. Each power is applied from rest, with the
        servo released in between, so every point is a genuine static-friction test rather than a
        continuation of the previous one. Scoring is left to the caller because "did it move"
        depends on the noise floor, which is measured separately.
        """
        self.cmd("select", pod=pod)
        self.cmd("recStart", label=label or f"staircase_p{pod}")
        time.sleep(0.2)
        for p in powers:
            self.cmd("rawServo", pow=p, sec=dwell_s, pod=pod)
            time.sleep(dwell_s + gap_s)
        self.cmd("recStop")
        time.sleep(0.2)

        csv_text = self.rec_csv()
        trace = parse_csv(csv_text)
        out = {
            "kind": "staircase",
            "pod": pod,
            "powers": powers,
            "dwell_s": dwell_s,
            "label": label,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "samples": len(trace["t"]),
        }
        out["trace_file"] = _archive(csv_text, out)
        return out

    def quiet_record(self, seconds: float = 10.0, label: str = "noise") -> dict:
        """Records with every servo released, for the encoder noise floor.

        The OpMode releases servos in IDLE, so the pods are mechanically free. That is the honest
        condition for a noise measurement: any motion seen here is noise plus whatever the pod
        drifts on its own, which is exactly what the controller has to live with.
        """
        self.cmd("stop")
        time.sleep(0.3)
        self.cmd("recStart", label=label)
        time.sleep(seconds)
        self.cmd("recStop")
        time.sleep(0.2)

        csv_text = self.rec_csv()
        trace = parse_csv(csv_text)
        out = {"kind": "noise", "label": label, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")}
        out["pods"] = []
        for i in range(POD_COUNT):
            v = [x for x in trace[f"p{i}_v"] if not math.isnan(x)]
            w = [x for x in trace[f"p{i}_wheel"] if not math.isnan(x)]
            out["pods"].append(
                {
                    "pod": i,
                    "n": len(v),
                    "volts_mean": _mean(v),
                    "volts_sigma": _stdev(v),
                    "deg_sigma": _stdev(w),
                    "deg_peak_to_peak": (max(w) - min(w)) if w else float("nan"),
                }
            )
        out["loopHz_mean"] = _mean([x for x in trace["loopHz"] if not math.isnan(x)])
        out["trace_file"] = _archive(csv_text, out)
        _append_trial(out)
        return out


# ---- CSV ---------------------------------------------------------------------------


def parse_csv(text: str) -> dict[str, list[float]]:
    """Parses the recorder's CSV into columns. Empty fields become NaN."""
    lines = [ln for ln in text.splitlines() if ln and not ln.startswith("#")]
    if not lines:
        raise BenchError("recording is empty - was recStart issued?")
    header = lines[0].split(",")
    cols: dict[str, list[float]] = {h: [] for h in header}
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        for h, raw in zip(header, parts):
            cols[h].append(float(raw) if raw else float("nan"))
    return cols


# ---- scoring -----------------------------------------------------------------------


def score_step(
    trace: dict[str, list[float]],
    expected_step_deg: float | None = None,
    settle_bands: tuple[float, ...] = (0.5, 1.0, 1.5),
    ring_threshold_deg: float = 0.30,
    post_settle_window_s: float = 3.0,
) -> dict:
    """Scores a recorded step response for all four pods.

    Scored against the pod's own post-flip error rather than a reconstructed target, so the
    plus-or-minus 180 degree equivalence and the wrap are already resolved by the pod itself.

    ``ring_threshold_deg`` is the excursion a sign change must reach on both sides before it counts
    as a ring. Without it, encoder noise around zero error registers dozens of crossings per second
    and the ring count measures the ADC rather than the controller.
    """
    t = trace["t"]
    out: dict[str, Any] = {"kind": "step", "samples": len(t), "pods": []}

    loop = [x for x in trace["loopHz"] if not math.isnan(x)]
    volts = [x for x in trace["volts"] if not math.isnan(x)]
    out["loopHz_mean"] = _mean(loop)
    out["loopHz_min"] = min(loop) if loop else float("nan")
    out["voltage_mean"] = _mean(volts)
    out["voltage_min"] = min(volts) if volts else float("nan")

    for i in range(POD_COUNT):
        err = trace[f"p{i}_err"]
        pwr = trace[f"p{i}_pwr"]
        flip = trace[f"p{i}_flip"]
        tgt = trace[f"p{i}_tgt"]
        wheel = trace[f"p{i}_wheel"]

        onset, pre_step_err = _step_onset(err, tgt)
        if onset is None:
            out["pods"].append({"pod": i, "ok": False, "reason": "no step found in this recording"})
            continue

        t0 = t[onset]
        rel = [t[k] - t0 for k in range(onset, len(t))]
        e = [err[k] for k in range(onset, len(err))]
        p = [pwr[k] for k in range(onset, len(pwr))]
        f = [flip[k] for k in range(onset, len(flip))]

        step = abs(e[0])
        sign = 1.0 if e[0] >= 0 else -1.0
        end_t = rel[-1] if rel else 0.0

        pod: dict[str, Any] = {
            "pod": i,
            "ok": True,
            "step_deg_measured": step,
            "duration_s": end_t,
            # What the pod was holding just before the step. This is the residual the controller
            # settles to from the *previous* move, measured under identical conditions, so it is a
            # free extra sample of steady-state error on every trial.
            "pre_step_err_deg": pre_step_err,
        }
        if expected_step_deg is not None:
            # A pod that flipped sees half the commanded move, and a mis-zeroed pod sees the wrong
            # move entirely. Recording the discrepancy stops a bad setup scoring as a good result.
            pod["step_deg_expected"] = expected_step_deg
            pod["step_matches_command"] = abs(step - expected_step_deg) <= max(
                2.0, 0.05 * expected_step_deg
            )

        pod["rise_10_90_s"] = _rise_time(rel, e, step)

        # Overshoot and ringing are measured from the physical wheel angle, not from the pod's
        # internal error.
        #
        # A pod treats a heading and that heading plus 180 as the same demand, and a 90 degree step
        # sits exactly on that boundary. When move() takes the flip, errorRad jumps discontinuously
        # from +90 to -90 while the wheel does not move at all - which scored as a 100% overshoot
        # and a fistful of rings for a pod that was behaving perfectly. Unwrapped wheel angle
        # against where the pod actually came to rest has no such discontinuity.
        wheel_un = _unwrap([wheel[k] for k in range(onset, len(wheel))])
        final = _mean([wheel_un[k] for k in range(len(rel)) if rel[k] >= end_t - 0.5])
        travelled = final - wheel_un[0]
        pod["travel_deg"] = travelled

        if abs(travelled) > 1e-6:
            # Normalised response: 0 at the step, 1 where the pod ended up.
            y = [(wheel_un[k] - wheel_un[0]) / travelled for k in range(len(wheel_un))]
            overshoot_frac = max(y) - 1.0
            pod["overshoot_deg"] = max(0.0, overshoot_frac * abs(travelled))
            pod["overshoot_pct"] = 100.0 * max(0.0, overshoot_frac)
            residual = [(v - 1.0) * abs(travelled) for v in y]
            pod["rings"] = _ring_count(residual, ring_threshold_deg)
            # The same count against a threshold scaled to the step. Which one is "the" ring count
            # is a real choice, not a detail: at a fixed 0.3 deg, a 0.4 deg wobble on a 90 deg move
            # counts as a ring, and that is 0.4% of the travel - far below anything visible on a
            # chart, which is how the ring counts in the repo's existing notes were arrived at.
            # Both are reported so the comparison is explicit rather than implied.
            pod["rings_1pct"] = _ring_count(residual, max(0.30, 0.01 * abs(travelled)))
        else:
            pod["overshoot_deg"] = 0.0
            pod["overshoot_pct"] = 0.0
            pod["rings"] = 0
            pod["rings_1pct"] = 0

        for band in settle_bands:
            # One decimal always, so 1.0 keys as settle_1_0 rather than settle_1 and the column
            # names stay stable whatever bands are asked for.
            pod[f"settle_{band:.1f}".replace(".", "_")] = _settle_time(rel, e, band)

        tail = [x for k, x in enumerate(e) if rel[k] >= end_t - 0.5]
        pod["steady_state_deg"] = _mean(tail)
        pod["steady_state_abs_deg"] = abs(_mean(tail))

        # A flip on the first samples of a 90 degree step is expected and harmless: the two
        # directions reach the same physical heading. Only a flip once the pod is already moving
        # means the decision is chattering, which is the failure worth counting.
        moving = [k for k in range(len(wheel_un)) if abs(wheel_un[k] - wheel_un[0]) > 5.0]
        after = moving[0] if moving else len(f)
        pod["flip_events"] = sum(1 for k in range(max(1, after), len(f)) if f[k] != f[k - 1])
        pod["flip_at_onset"] = bool(f and f[0] != f[min(len(f) - 1, max(0, after - 1))])

        # Post-settle behaviour, over the last window of the hold. The hold has to be long enough
        # that this window sits entirely after the transient; otherwise the peak-to-peak figure is
        # just the step height wearing a different name, which is exactly how a 3 s window on a 3 s
        # hold reported 90 degrees of "limit cycle".
        w0 = max(0.0, end_t - post_settle_window_s)
        idx = [k for k in range(len(rel)) if rel[k] >= w0]
        we = [e[k] for k in idx]
        wp = [p[k] for k in idx]
        pod["post_settle_window_s"] = end_t - w0
        pod["post_settle_window_start_s"] = w0
        # True when the window provably excludes the transient.
        s10 = pod.get("settle_1_0")
        pod["post_settle_clean"] = bool(
            s10 is not None and not math.isnan(s10) and w0 >= s10
        )
        pod["post_settle_pp_deg"] = (max(we) - min(we)) if we else float("nan")
        pod["rest_power_rms"] = _rms(wp)
        pod["rest_power_mean_abs"] = _mean([abs(x) for x in wp])
        pod["peak_power"] = max((abs(x) for x in p), default=0.0)

        # Discrete corrections in the post-settle window: every transition from no output to some
        # output. For the pulsed controller this counts pulses, and a settled pod should need
        # none - repeated pulses at rest are the stacking failure, not normal operation.
        pod["pulses_post"] = sum(
            1 for k in range(1, len(wp)) if abs(wp[k]) > 1e-3 and abs(wp[k - 1]) <= 1e-3
        )
        # The composite the pulsed work is aimed at: parked tight, quiet, and not still correcting.
        pod["parked"] = bool(
            abs(pod["steady_state_deg"]) <= 0.34
            and pod["post_settle_pp_deg"] <= 0.34
            and pod["pulses_post"] == 0
        )

        out["pods"].append(pod)

    return out


def _wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def _unwrap(deg: list[float]) -> list[float]:
    """Removes 360 degree jumps so travel can be measured across the encoder wrap."""
    out, acc, prev = [], 0.0, None
    for d in deg:
        if math.isnan(d):
            out.append(float("nan"))
            continue
        if prev is not None:
            step = d - prev
            if step > 180:
                acc -= 360
            elif step < -180:
                acc += 360
        out.append(d + acc)
        prev = d
    return out


def _step_onset(
    err: list[float], tgt: list[float], jump_deg: float = 5.0
) -> tuple[int | None, float]:
    """Finds the sample where the setpoint moved, plus the error held just before it.

    Detecting the step from the *target* rather than from the error matters: a trial parks the pods
    under closed-loop control before stepping them, so the error is already non-NaN and small when
    recording starts. Triggering on the first non-NaN error measures the pre-step hold and reports
    the residual as if it were the step size.
    """
    first = next((k for k, v in enumerate(tgt) if not math.isnan(v)), None)
    if first is None:
        # No commanded target was ever recorded; fall back to the first closed-loop sample.
        k = next((k for k, v in enumerate(err) if not math.isnan(v)), None)
        return k, float("nan")

    base = tgt[first]
    for k in range(first, len(tgt)):
        if math.isnan(tgt[k]):
            continue
        if abs(_wrap180(tgt[k] - base)) > jump_deg:
            pre = [err[j] for j in range(first, k) if not math.isnan(err[j])]
            return k, (_mean(pre[-5:]) if pre else float("nan"))
    return None, float("nan")


def _rise_time(rel: list[float], e: list[float], step: float) -> float:
    """Time for |error| to fall from 90% to 10% of the step."""
    if step <= 1e-9:
        return float("nan")
    hi, lo = 0.9 * step, 0.1 * step
    t_hi = next((rel[k] for k in range(len(e)) if abs(e[k]) <= hi), None)
    t_lo = next((rel[k] for k in range(len(e)) if abs(e[k]) <= lo), None)
    if t_hi is None or t_lo is None:
        return float("nan")
    return t_lo - t_hi


def _settle_time(rel: list[float], e: list[float], band: float) -> float:
    """Earliest time after which |error| stays within ``band`` for the rest of the record.

    NaN when it never settles, which is a result, not a missing value - the caller must not treat
    it as zero.
    """
    last_out = None
    for k in range(len(e)):
        if abs(e[k]) > band:
            last_out = k
    if last_out is None:
        return 0.0
    if last_out >= len(e) - 1:
        return float("nan")
    return rel[last_out + 1]


def _ring_count(e: list[float], threshold: float) -> int:
    """Sign changes of the error that actually swing past ``threshold`` on both sides."""
    rings = 0
    armed_sign = 0
    for x in e:
        if abs(x) < threshold:
            continue
        s = 1 if x > 0 else -1
        if armed_sign == 0:
            armed_sign = s
        elif s != armed_sign:
            rings += 1
            armed_sign = s
    return rings


# ---- small stats (no numpy) ---------------------------------------------------------


def _mean(xs: list[float]) -> float:
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def _stdev(xs: list[float]) -> float:
    xs = [x for x in xs if not math.isnan(x)]
    if len(xs) < 2:
        return float("nan")
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _rms(xs: list[float]) -> float:
    xs = [x for x in xs if not math.isnan(x)]
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


# ---- persistence --------------------------------------------------------------------


def _gains_of(state: dict) -> list[dict]:
    return [
        # Tolerant of missing keys: the host is often a build ahead of what is flashed on the hub,
        # and a KeyError here would abort a run that is otherwise perfectly valid.
        {
            k: p[k]
            for k in ("i", "label", "kp", "ki", "kd", "kf", "cache", "ks", "ksband", "dom",
                      "pulsed", "pband", "ptol", "ppow", "pms", "pcoast")
            if k in p
        }
        for p in state.get("pods", [])
    ]


def _archive(csv_text: str, meta: dict) -> str:
    """Stores the raw trace gzipped, and returns its path relative to this tool."""
    os.makedirs(RUN_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(meta.get("label", "run")))
    name = f"{stamp}_{safe or 'run'}.csv.gz"
    path = os.path.join(RUN_DIR, name)
    with gzip.open(path, "wt", encoding="utf-8") as fh:
        fh.write(csv_text)
    return os.path.join("runs", name)


def _append_trial(result: dict) -> None:
    with open(TRIAL_LOG, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(result, default=str) + "\n")


# ---- reporting ----------------------------------------------------------------------


def format_step(result: dict) -> str:
    """One compact table per step trial, for reading in a terminal."""
    lines = []
    lines.append(
        f"{result.get('label','')}  step={result.get('step_deg_commanded')}deg  "
        f"loop={result['loopHz_mean']:.0f}Hz (min {result['loopHz_min']:.0f})  "
        f"V={result['voltage_mean']:.2f}"
    )
    g = result.get("gains") or [{}]
    lines.append(
        f"  gains kP={g[0].get('kp')} kI={g[0].get('ki')} kD={g[0].get('kd')} "
        f"kF={g[0].get('kf')} cache={g[0].get('cache')}"
    )
    hdr = (
        f"  {'pod':>3} {'step':>6} {'rise':>6} {'over%':>6} {'s1.5':>6} {'s1.0':>6} {'s0.5':>6} "
        f"{'ss':>6} {'pre':>6} {'ring':>4} {'flip':>4} {'pp':>6} {'Prms':>6}"
    )
    lines.append(hdr)
    for p in result["pods"]:
        if not p.get("ok"):
            lines.append(f"  {p['pod']:>3}  {p.get('reason','')}")
            continue
        lines.append(
            f"  {p['pod']:>3} {p['step_deg_measured']:>6.1f} {_f(p['rise_10_90_s']):>6} "
            f"{p['overshoot_pct']:>6.1f} {_f(p['settle_1_5']):>6} {_f(p['settle_1_0']):>6} "
            f"{_f(p['settle_0_5']):>6} {p['steady_state_deg']:>6.2f} "
            f"{p['pre_step_err_deg']:>6.2f} {p['rings']:>4} {p['flip_events']:>4} "
            f"{p['post_settle_pp_deg']:>6.2f} {p['rest_power_rms']:>6.3f}"
        )
    return "\n".join(lines)


def _f(v: float) -> str:
    return "  --  " if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}"
