"""Steering-quality scorer: one pass over a recorded run, all eleven criteria.

DIAGNOSTIC TOOL. Reads the recorder CSV that SwerveBringUp already produces (columns
t,dt,volts,loopHz,mode,servoMa,batteryMa,heading,htgt,px,py,cf,cs,ct,p{i}_{v,wheel,tgt,err,pwr,flip})
and emits the numbers and graphs the swerve-steering task asks for.

Two rules this file exists to enforce:

  * Loop rate is ``1 / mean(dt)``. ``mean(loopHz)`` is the Jensen-inflated statistic that had
    every rate in this project reading ~1.8x optimistic; it is computed here ONLY so the
    inflation factor can be reported and dismissed.
  * Reversals and path length use the same definitions as drivetune.py - unwrapped degrees,
    0.5 deg hysteresis - so numbers are comparable with everything already in trials.jsonl.

    python steerqual.py runs/<file>.csv.gz [more.csv.gz ...] [--out DIR] [--tag NAME]
                        [--path PATHFILE.json] [--quiet-graphs]

``--path`` takes a JSON file of {"x": [...], "y": [...]} sampled densely along the planned
path, in the same inches-and-pose-frame the recorder logs px/py in; cross-track error is then
the distance from each pose sample to the nearest point on that polyline. Without it, graph 4
is skipped and criterion 9 reports "no reference path".
"""

from __future__ import annotations

import argparse
import glob
import gzip
import json
import math
import os
import sys

import numpy as np

POD_COUNT = 4
POD_NAMES = {0: "RB", 1: "RF", 2: "LF", 3: "LB"}

# A wheel-direction change only counts once the wheel has moved this far the other way; below
# it, encoder noise reads as reversals. Same value as drivetune.py so the numbers compare.
REV_HYST_DEG = 0.5

# Mode ordinals from SwerveBringUp.Mode.
MODE_DRIVE = 9
MODE_FOLLOW = 10

# Applied-command magnitude above which the robot counts as "being driven".
DRIVE_ACTIVE = 0.02

# A consecutive-loop setpoint jump larger than this is a criterion-1 violation unless the pod
# also flipped on that loop.
JUMP_LIMIT_DEG = 15.0

# Half-width of the window around each 45 deg multiple used for the clustering test.
CLUSTER_HALF_WIDTH_DEG = 5.0


# ---------------------------------------------------------------- loading


def load(path: str) -> dict[str, np.ndarray]:
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as fh:
        text = fh.read()
    lines = [ln for ln in text.splitlines() if ln and not ln.startswith("#")]
    if not lines:
        raise SystemExit(f"{path}: empty recording")
    header = lines[0].split(",")
    rows = []
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) != len(header):
            continue
        rows.append([float(p) if p else math.nan for p in parts])
    arr = np.array(rows, dtype=float)
    cols = {h: arr[:, i] for i, h in enumerate(header)}
    cols["_file"] = os.path.basename(path)
    return cols


def concat(chunks: list[dict]) -> dict:
    """Joins chunks end to end, rebasing t so the time axis is continuous.

    The gap between chunks (the hub rendering 3000 rows of CSV) is NOT bridged with a fake
    sample: the first dt of each chunk is dropped so a ~1 s hole cannot masquerade as a slow
    loop, and every rate is computed against summed dt rather than elapsed wall time.
    """
    keys = [k for k in chunks[0] if not k.startswith("_")]
    out = {k: [] for k in keys}
    out["_chunk"] = []
    t_base = 0.0
    for n, c in enumerate(chunks):
        span = float(np.nansum(c["dt"][1:])) if len(c["dt"]) > 1 else 0.0
        for k in keys:
            v = np.array(c[k], dtype=float)
            if k == "t":
                v = v - (v[0] if len(v) else 0.0) + t_base
            if k == "dt" and len(v):
                v = v.copy()
                v[0] = math.nan  # first dt of a chunk spans the inter-chunk hole
            out[k].append(v)
        out["_chunk"].append(np.full(len(c["dt"]), n, dtype=float))
        t_base += span
    return {k: np.concatenate(v) for k, v in out.items()}


# ---------------------------------------------------------------- primitives


def unwrap_deg(a: np.ndarray) -> np.ndarray:
    """Unwraps a degree series, holding NaN gaps rather than bridging them."""
    out = np.full(len(a), math.nan)
    acc = 0.0
    prev = math.nan
    for i, d in enumerate(a):
        if math.isnan(d):
            continue
        if not math.isnan(prev):
            step = d - prev
            if step > 180:
                acc -= 360
            elif step < -180:
                acc += 360
        out[i] = d + acc
        prev = d
    return out


def wrap180(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def wrap90(a: np.ndarray) -> np.ndarray:
    """Wraps to (-90, 90] - the PHYSICAL azimuth change a pod would perform.

    A pod treats theta and theta+180 as the same azimuth: past a quarter turn it flips and
    reverses the drive instead of rotating. So a 179 degree change in the demand costs one
    degree of travel, not 179. Criterion 1 excludes deliberate flips, which is exactly this
    wrap; the flips themselves are counted separately from the pod's own flip flag.
    """
    return (np.asarray(a) + 90.0) % 180.0 - 90.0


def reversals(series: np.ndarray, hyst: float = REV_HYST_DEG) -> int:
    """Direction changes with hysteresis. Identical algorithm to drivetune.reversals."""
    xs = series[~np.isnan(series)]
    if len(xs) < 3:
        return 0
    count = 0
    direction = 0
    extreme = xs[0]
    for x in xs[1:]:
        if direction >= 0 and x > extreme:
            extreme, direction = x, 1
        elif direction <= 0 and x < extreme:
            extreme, direction = x, -1
        elif direction == 1 and x < extreme - hyst:
            count += 1
            direction, extreme = -1, x
        elif direction == -1 and x > extreme + hyst:
            count += 1
            direction, extreme = 1, x
    return count


def pct(a: np.ndarray, q: float) -> float:
    a = a[~np.isnan(a)]
    return float(np.percentile(a, q)) if len(a) else math.nan


def stats(a: np.ndarray) -> dict:
    a = np.asarray(a, dtype=float)
    a = a[~np.isnan(a)]
    if not len(a):
        return {"n": 0}
    return {
        "n": int(len(a)),
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p90": float(np.percentile(a, 90)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "min": float(a.min()),
        "max": float(a.max()),
    }


# ---------------------------------------------------------------- metrics


def loop_metrics(d: dict) -> dict:
    dt = d["dt"]
    good = dt[(~np.isnan(dt)) & (dt > 0)]
    ms = good * 1000.0
    out = {
        "samples": int(len(good)),
        "span_s": float(good.sum()),
        "loop_hz_true": float(1.0 / good.mean()) if len(good) else math.nan,
        "loop_dt_mean_ms": float(ms.mean()) if len(good) else math.nan,
        "loop_dt_p50_ms": pct(ms, 50),
        "loop_dt_p90_ms": pct(ms, 90),
        "loop_dt_p99_ms": pct(ms, 99),
        "loop_dt_min_ms": float(ms.min()) if len(good) else math.nan,
        "loop_dt_max_ms": float(ms.max()) if len(good) else math.nan,
    }
    lh = d.get("loopHz")
    if lh is not None:
        lh = lh[~np.isnan(lh)]
        if len(lh):
            # Reported only to name the inflation, never as the loop rate.
            out["loopHz_col_mean_INFLATED"] = float(lh.mean())
            out["inflation_factor"] = float(lh.mean() / out["loop_hz_true"])
    return out


def drive_mask(d: dict) -> np.ndarray:
    """True where the robot was being commanded to move."""
    cf, cs, ct = d.get("cf"), d.get("cs"), d.get("ct")
    if cf is None:
        return np.ones(len(d["dt"]), dtype=bool)
    mag = np.nan_to_num(np.hypot(cf, cs)) + np.abs(np.nan_to_num(ct))
    return mag > DRIVE_ACTIVE


def pod_target(d: dict, i: int) -> tuple[np.ndarray, str]:
    """The demand column, preferring the pod's own report over the host-side mirror.

    ``ctgt`` is read out of CoaxialPod and cannot disagree with what the pod acted on; ``tgt`` is
    SwerveBringUp.computeTargets, a mirror that has drifted from the mixer before. Runs recorded
    before 2026-08-16 have no ctgt column, and DriveTeleOp captures have no tgt.
    """
    c = d.get(f"p{i}_ctgt")
    if c is not None and not np.all(np.isnan(c)):
        return c, "ctgt"
    return d[f"p{i}_tgt"], "tgt"


def pod_metrics(d: dict, i: int, active: np.ndarray) -> dict:
    tgt, tgt_src = pod_target(d, i)
    wheel = d[f"p{i}_wheel"]
    err = d[f"p{i}_err"]
    flip = d.get(f"p{i}_flip")
    dt = d["dt"]

    tgt_un = unwrap_deg(tgt)
    wheel_un = unwrap_deg(wheel)
    span = float(np.nansum(dt[1:]))
    span_active = float(np.nansum(np.where(active, dt, 0.0)))

    # ---- criterion 1: consecutive-loop setpoint jumps, deliberate flips excluded
    dtgt = np.abs(wrap90(np.diff(tgt)))
    flip_change = None
    if flip is not None:
        f = np.nan_to_num(flip) > 0.5
        flip_change = f[1:] != f[:-1]
    jump_mask = active[1:] & ~np.isnan(dtgt)
    if flip_change is not None:
        jump_mask &= ~flip_change
    jumps = dtgt[jump_mask]
    n_flip_events = int(flip_change[active[1:]].sum()) if flip_change is not None else 0

    # ---- 45 degree clustering of the setpoint itself
    t_active = tgt[active]
    t_active = t_active[~np.isnan(t_active)]
    if len(t_active):
        near45 = np.abs(wrap180(t_active[:, None] - np.arange(0, 360, 45)[None, :]))
        frac45 = float((near45.min(axis=1) <= CLUSTER_HALF_WIDTH_DEG).mean())
    else:
        frac45 = math.nan
    # Under a uniform setpoint distribution, 8 windows of +-5 deg cover 80/360 of the circle.
    uniform45 = 8 * 2 * CLUSTER_HALF_WIDTH_DEG / 360.0

    # ---- criteria 5/6: path inflation and reversals
    wheel_path = float(np.nansum(np.abs(np.diff(wheel_un[active]))))
    tgt_path = float(np.nansum(np.abs(np.diff(tgt_un[active]))))
    wrev = reversals(wheel_un[active])
    trev = reversals(tgt_un[active])

    return {
        "pod": i,
        "label": POD_NAMES.get(i, str(i)),
        "target_column": tgt_src,
        "span_s": span,
        "span_active_s": span_active,
        "jump_deg": stats(jumps),
        "jumps_over_limit": int((jumps > JUMP_LIMIT_DEG).sum()),
        "jumps_over_limit_per_s": float((jumps > JUMP_LIMIT_DEG).sum() / span_active)
        if span_active else math.nan,
        "flip_events": n_flip_events,
        "flip_events_per_s": float(n_flip_events / span_active) if span_active else math.nan,
        "frac_within_5deg_of_45mult": frac45,
        "frac_expected_if_uniform": uniform45,
        "wheel_path_deg": wheel_path,
        "tgt_path_deg": tgt_path,
        "path_ratio": wheel_path / tgt_path if tgt_path > 1e-6 else math.nan,
        "wheel_rev_per_s": float(wrev / span_active) if span_active else math.nan,
        "tgt_rev_per_s": float(trev / span_active) if span_active else math.nan,
        "excess_rev_per_s": float((wrev - trev) / span_active) if span_active else math.nan,
        "err_driving_abs": stats(np.abs(err[active])),
        "err_rest_abs": stats(np.abs(err[~active])),
    }


def heading_metrics(d: dict, active: np.ndarray) -> dict:
    h, ht = d.get("heading"), d.get("htgt")
    if h is None or ht is None:
        return {"available": False}
    err = wrap180(ht - h)
    have = ~np.isnan(err)
    speed = chassis_speed(d)
    moving = have & active & (speed > 2.0)
    resting = have & ~active
    out = {
        "available": True,
        "samples_with_target": int(have.sum()),
        "frac_of_run_with_target": float(have.mean()),
        "err_abs_translating": stats(np.abs(err[moving])),
        "err_abs_at_rest": stats(np.abs(err[resting])),
        "err_abs_all": stats(np.abs(err[have])),
    }
    # Does error scale with translation speed? Tuning-vs-geometry discriminator.
    m = have & (speed > 0)
    if m.sum() > 30:
        sp, ae = speed[m], np.abs(err[m])
        out["speed_corr_r"] = float(np.corrcoef(sp, ae)[0, 1])
        slope, intercept = np.polyfit(sp, ae, 1)
        out["speed_fit_deg_per_in_s"] = float(slope)
        out["speed_fit_intercept_deg"] = float(intercept)
        bins = [(0, 5), (5, 15), (15, 30), (30, 1e9)]
        out["err_by_speed_bin"] = [
            {"lo": lo, "hi": hi if hi < 1e9 else None,
             "n": int(((sp >= lo) & (sp < hi)).sum()),
             "mean_abs_err_deg": float(ae[(sp >= lo) & (sp < hi)].mean())
             if ((sp >= lo) & (sp < hi)).sum() else math.nan}
            for lo, hi in bins
        ]
    # Rotation-command correlation, to separate "error grows when turning" from
    # "error grows when translating".
    ct = d.get("ct")
    if ct is not None and m.sum() > 30:
        turn = np.abs(np.nan_to_num(ct))[m]
        if turn.std() > 1e-9:
            out["turn_corr_r"] = float(np.corrcoef(turn, np.abs(err[m]))[0, 1])
    return out


def chassis_speed(d: dict) -> np.ndarray:
    """Inches/second from the logged pose, NaN where pose is missing."""
    px, py, dt = d.get("px"), d.get("py"), d["dt"]
    if px is None:
        return np.full(len(dt), math.nan)
    vx = np.full(len(px), math.nan)
    vy = np.full(len(px), math.nan)
    dpx, dpy = np.diff(px), np.diff(py)
    step = dt[1:]
    ok = (step > 0) & ~np.isnan(dpx) & ~np.isnan(dpy)
    vx[1:][ok] = dpx[ok] / step[ok]
    vy[1:][ok] = dpy[ok] / step[ok]
    return np.hypot(vx, vy)


def chassis_kinematics(d: dict) -> dict:
    """Speed, acceleration and jerk from the pose column, lightly smoothed.

    Differentiating a 20-90 Hz pose stream three times amplifies quantisation badly, so each
    derivative is taken on a 5-sample moving average. The jerk figure is therefore an
    order-of-magnitude number, not a precise one - it is labelled as such wherever it is used.
    """
    dt = d["dt"]
    speed = chassis_speed(d)
    t = np.nancumsum(np.nan_to_num(dt))

    def smooth(a, w=5):
        out = np.full(len(a), math.nan)
        good = ~np.isnan(a)
        if good.sum() < w:
            return out
        idx = np.where(good)[0]
        vals = np.convolve(a[good], np.ones(w) / w, mode="same")
        out[idx] = vals
        return out

    sp = smooth(speed)
    acc = np.full(len(sp), math.nan)
    jerk = np.full(len(sp), math.nan)
    step = dt.copy()
    step[np.isnan(step) | (step <= 0)] = math.nan
    acc[1:] = np.diff(sp) / step[1:]
    acc = smooth(acc)
    jerk[1:] = np.diff(acc) / step[1:]
    return {"t": t, "speed": sp, "acc": acc, "jerk": smooth(jerk)}


def cross_track(d: dict, path: dict) -> np.ndarray:
    px, py = d.get("px"), d.get("py")
    if px is None or path is None:
        return None
    xs, ys = np.array(path["x"], dtype=float), np.array(path["y"], dtype=float)
    out = np.full(len(px), math.nan)
    for k in range(len(px)):
        if math.isnan(px[k]) or math.isnan(py[k]):
            continue
        out[k] = float(np.min(np.hypot(xs - px[k], ys - py[k])))
    return out


# ---------------------------------------------------------------- graphs


def graphs(d: dict, m: dict, outdir: str, tag: str, path: dict | None) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(outdir, exist_ok=True)
    files = []
    t = np.nancumsum(np.nan_to_num(d["dt"]))
    active = drive_mask(d)

    # 1 - per-module commanded vs measured azimuth
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for i in range(POD_COUNT):
        ax = axes[i]
        tv, src = pod_target(d, i)
        ax.plot(t, tv, lw=0.8, label=f"commanded ({src})")
        ax.plot(t, d[f"p{i}_wheel"], lw=0.8, label="measured (wheel)")
        for mult in range(0, 360, 45):
            ax.axhline(mult, color="0.85", lw=0.5, zorder=0)
        ax.set_ylabel(f"pod {i} {POD_NAMES[i]}\ndeg")
        ax.set_ylim(0, 360)
        if i == 0:
            ax.legend(loc="upper right", fontsize=8, ncol=2)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle(f"1. Commanded vs measured azimuth - {tag}\n"
                 "grey lines are 45 deg multiples (the snapping family)")
    fig.tight_layout()
    f = os.path.join(outdir, f"{tag}_1_azimuth.png")
    fig.savefig(f, dpi=110)
    plt.close(fig)
    files.append(f)

    # 2 - heading target vs measured
    if m["heading"].get("available") and m["heading"]["samples_with_target"]:
        fig, ax = plt.subplots(2, 1, figsize=(13, 6), sharex=True,
                               gridspec_kw={"height_ratios": [2, 1]})
        ax[0].plot(t, d["heading"], lw=0.9, label="measured")
        ax[0].plot(t, d["htgt"], lw=0.9, label="target (latched)")
        ax[0].set_ylabel("heading (deg)")
        ax[0].legend(loc="upper right", fontsize=8)
        ax[1].plot(t, wrap180(d["htgt"] - d["heading"]), lw=0.8, color="crimson")
        ax[1].axhline(0, color="0.7", lw=0.5)
        ax[1].set_ylabel("error (deg)")
        ax[1].set_xlabel("t (s)")
        fig.suptitle(f"2. Robot heading: target vs measured - {tag}")
        fig.tight_layout()
        f = os.path.join(outdir, f"{tag}_2_heading.png")
        fig.savefig(f, dpi=110)
        plt.close(fig)
        files.append(f)

    # 3 - chassis velocity / acceleration / jerk
    k = chassis_kinematics(d)
    fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    ax[0].plot(k["t"], k["speed"], lw=0.9)
    ax[0].set_ylabel("speed (in/s)")
    ax[1].plot(k["t"], k["acc"], lw=0.8, color="darkorange")
    ax[1].set_ylabel("accel (in/s^2)")
    ax[2].plot(k["t"], k["jerk"], lw=0.6, color="purple")
    ax[2].set_ylabel("jerk (in/s^3)\n(order of magnitude)")
    ax[2].set_xlabel("t (s)")
    fig.suptitle(f"3. Chassis velocity, acceleration, jerk - {tag}\n"
                 "differentiated from logged pose; jerk is indicative only")
    fig.tight_layout()
    f = os.path.join(outdir, f"{tag}_3_kinematics.png")
    fig.savefig(f, dpi=110)
    plt.close(fig)
    files.append(f)

    # 4 - cross-track error vs distance along path
    xt = cross_track(d, path) if path else None
    if xt is not None:
        px, py = d["px"], d["py"]
        dist = np.nancumsum(np.concatenate([[0], np.hypot(np.diff(px), np.diff(py))]))
        fig, ax = plt.subplots(figsize=(13, 4))
        ax.plot(dist, xt, lw=0.9)
        ax.set_xlabel("distance along path (in)")
        ax.set_ylabel("cross-track error (in)")
        fig.suptitle(f"4. Cross-track error - {tag}")
        fig.tight_layout()
        f = os.path.join(outdir, f"{tag}_4_crosstrack.png")
        fig.savefig(f, dpi=110)
        plt.close(fig)
        files.append(f)

    # 5 - loop dt vs time
    fig, ax = plt.subplots(2, 1, figsize=(13, 6),
                           gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(t, d["dt"] * 1000.0, lw=0.6)
    ax[0].axhline(m["loop"]["loop_dt_mean_ms"], color="crimson", lw=0.8,
                  label=f"mean {m['loop']['loop_dt_mean_ms']:.1f} ms "
                        f"({m['loop']['loop_hz_true']:.1f} Hz true)")
    ax[0].axhline(25, color="green", ls="--", lw=0.8, label="p90 target 25 ms")
    ax[0].set_ylabel("dt (ms)")
    ax[0].legend(loc="upper right", fontsize=8)
    good = d["dt"][(~np.isnan(d["dt"])) & (d["dt"] > 0)] * 1000.0
    ax[1].hist(good, bins=60)
    ax[1].set_xlabel("dt (ms)")
    ax[1].set_ylabel("count")
    fig.suptitle(f"5. Loop period - {tag}")
    fig.tight_layout()
    f = os.path.join(outdir, f"{tag}_5_loopdt.png")
    fig.savefig(f, dpi=110)
    plt.close(fig)
    files.append(f)

    # 6 - path ratio and reversals per pod
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    idx = np.arange(POD_COUNT)
    ratios = [m["pods"][i]["path_ratio"] for i in range(POD_COUNT)]
    ax[0].bar(idx, ratios, color="steelblue")
    ax[0].axhline(1.3, color="crimson", ls="--", lw=1, label="criterion 5: < 1.3x")
    ax[0].set_xticks(idx, [f"{i} {POD_NAMES[i]}" for i in idx])
    ax[0].set_ylabel("wheel path / commanded path")
    ax[0].legend(fontsize=8)
    w = 0.38
    ax[1].bar(idx - w / 2, [m["pods"][i]["wheel_rev_per_s"] for i in idx], w, label="wheel")
    ax[1].bar(idx + w / 2, [m["pods"][i]["tgt_rev_per_s"] for i in idx], w, label="commanded")
    ax[1].axhline(1.0, color="crimson", ls="--", lw=1, label="criterion 6: < 1.0/s")
    ax[1].set_xticks(idx, [f"{i} {POD_NAMES[i]}" for i in idx])
    ax[1].set_ylabel("reversals / s")
    ax[1].legend(fontsize=8)
    fig.suptitle(f"6. Wheel path inflation and reversals, while driving - {tag}")
    fig.tight_layout()
    f = os.path.join(outdir, f"{tag}_6_shake.png")
    fig.savefig(f, dpi=110)
    plt.close(fig)
    files.append(f)

    # extra - setpoint jump histogram, the criterion-1 evidence
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    for i in range(POD_COUNT):
        dtg = np.abs(wrap180(np.diff(pod_target(d, i)[0])))[active[1:]]
        dtg = dtg[~np.isnan(dtg)]
        ax[0].hist(dtg, bins=np.arange(0, 91, 1.5), histtype="step",
                   label=f"pod {i}", log=True)
    ax[0].axvline(JUMP_LIMIT_DEG, color="crimson", ls="--", lw=1, label="15 deg limit")
    ax[0].set_xlabel("|setpoint change| between consecutive loops (deg)")
    ax[0].set_ylabel("count (log)")
    ax[0].legend(fontsize=8)
    for i in range(POD_COUNT):
        tv = pod_target(d, i)[0][active]
        tv = tv[~np.isnan(tv)]
        ax[1].hist(tv % 45.0, bins=45, histtype="step", label=f"pod {i}")
    ax[1].set_xlabel("setpoint mod 45 deg  (a spike at 0/45 is snapping)")
    ax[1].set_ylabel("count")
    ax[1].legend(fontsize=8)
    fig.suptitle(f"1b. Setpoint continuity and 45 deg clustering - {tag}")
    fig.tight_layout()
    f = os.path.join(outdir, f"{tag}_1b_setpoint.png")
    fig.savefig(f, dpi=110)
    plt.close(fig)
    files.append(f)
    return files


# ---------------------------------------------------------------- report


def analyse(d: dict, tag: str, path: dict | None = None) -> dict:
    active = drive_mask(d)
    modes = d.get("mode")
    m = {
        "tag": tag,
        "loop": loop_metrics(d),
        "pods": [pod_metrics(d, i, active) for i in range(POD_COUNT)],
        "heading": heading_metrics(d, active),
        "active_frac": float(active.mean()),
        "volts": stats(d.get("volts", np.array([math.nan]))),
    }
    if modes is not None:
        vals, counts = np.unique(modes[~np.isnan(modes)], return_counts=True)
        m["mode_histogram"] = {str(int(v)): int(c) for v, c in zip(vals, counts)}
        m["mode_is_drive_frac"] = float(np.mean(np.isin(modes, [MODE_DRIVE, MODE_FOLLOW])))
    sp = chassis_speed(d)
    m["speed_in_s"] = stats(sp[active])
    if path:
        xt = cross_track(d, path)
        m["cross_track_in"] = stats(xt[active]) if xt is not None else {"n": 0}
    return m


def report(m: dict) -> str:
    L = m["loop"]
    out = [f"=== {m['tag']} ===",
           f"samples {L['samples']}  span {L['span_s']:.1f} s  "
           f"battery {m['volts'].get('mean', float('nan')):.2f} V "
           f"({m['volts'].get('min', float('nan')):.2f}-{m['volts'].get('max', float('nan')):.2f})",
           "",
           "LOOP",
           f"  loop_hz_true      {L['loop_hz_true']:.1f} Hz      (1/mean(dt))",
           f"  loop_dt_mean_ms   {L['loop_dt_mean_ms']:.2f}",
           f"  loop_dt_p50_ms    {L['loop_dt_p50_ms']:.2f}",
           f"  loop_dt_p90_ms    {L['loop_dt_p90_ms']:.2f}",
           f"  loop_dt_p99_ms    {L['loop_dt_p99_ms']:.2f}",
           f"  loop_dt_min/max   {L['loop_dt_min_ms']:.2f} / {L['loop_dt_max_ms']:.2f}"]
    if "inflation_factor" in L:
        out.append(f"  [mean(loopHz) would read {L['loopHz_col_mean_INFLATED']:.1f} Hz - "
                   f"INFLATED {L['inflation_factor']:.2f}x, not a loop rate]")
    out += ["", f"DRIVING {m['active_frac'] * 100:.0f}% of samples   "
                f"speed mean {m['speed_in_s'].get('mean', float('nan')):.1f} "
                f"p95 {m['speed_in_s'].get('p95', float('nan')):.1f} in/s", ""]

    out.append("PER POD (while driving)")
    out.append(f"  {'pod':>5} {'jumpP90':>8} {'jumpMax':>8} {'>15deg':>7} {'>15/s':>7} "
               f"{'flip/s':>7} {'ratio':>6} {'whlRev/s':>9} {'tgtRev/s':>9} "
               f"{'|err|':>7} {'errP95':>7} {'45%':>6}")
    for p in m["pods"]:
        j = p["jump_deg"]
        e = p["err_driving_abs"]
        out.append(
            f"  {p['pod']} {p['label']:>3} {j.get('p90', float('nan')):>8.2f} "
            f"{j.get('max', float('nan')):>8.2f} {p['jumps_over_limit']:>7} "
            f"{p['jumps_over_limit_per_s']:>7.2f} {p['flip_events_per_s']:>7.2f} "
            f"{p['path_ratio']:>6.2f} {p['wheel_rev_per_s']:>9.2f} {p['tgt_rev_per_s']:>9.2f} "
            f"{e.get('mean', float('nan')):>7.2f} {e.get('p95', float('nan')):>7.2f} "
            f"{p['frac_within_5deg_of_45mult'] * 100:>5.1f}%")
    out.append(f"  (45% column: fraction of setpoints within {CLUSTER_HALF_WIDTH_DEG:.0f} deg of a "
               f"45 deg multiple; {m['pods'][0]['frac_expected_if_uniform'] * 100:.1f}% if uniform)")

    out.append("")
    out.append("PER POD (at rest)")
    for p in m["pods"]:
        e = p["err_rest_abs"]
        out.append(f"  {p['pod']} {p['label']:>3}  |err| mean {e.get('mean', float('nan')):>6.2f}  "
                   f"p95 {e.get('p95', float('nan')):>6.2f}  max {e.get('max', float('nan')):>6.2f}  "
                   f"n={e.get('n', 0)}")

    h = m["heading"]
    out.append("")
    if not h.get("available"):
        out.append("HEADING: columns absent")
    elif not h["samples_with_target"]:
        out.append("HEADING: no target logged in this run - htgt is NaN throughout, "
                   "so no closed heading loop was active (or hold was off)")
    else:
        out.append(f"HEADING (target present in {h['frac_of_run_with_target'] * 100:.0f}% of samples)")
        for k in ("err_abs_translating", "err_abs_at_rest", "err_abs_all"):
            s = h[k]
            if s.get("n"):
                out.append(f"  {k:<22} mean {s['mean']:.2f}  p95 {s['p95']:.2f}  "
                           f"max {s['max']:.2f}  n={s['n']}")
        if "speed_corr_r" in h:
            out.append(f"  error vs speed        r={h['speed_corr_r']:+.3f}  "
                       f"slope {h['speed_fit_deg_per_in_s']:+.3f} deg per in/s  "
                       f"intercept {h['speed_fit_intercept_deg']:.2f} deg")
            for b in h.get("err_by_speed_bin", []):
                hi = "inf" if b["hi"] is None else f"{b['hi']:g}"
                out.append(f"    {b['lo']:>3}-{hi:>3} in/s: mean |err| "
                           f"{b['mean_abs_err_deg']:.2f} deg  n={b['n']}")
        if "turn_corr_r" in h:
            out.append(f"  error vs |turn cmd|   r={h['turn_corr_r']:+.3f}")
    if "cross_track_in" in m and m["cross_track_in"].get("n"):
        c = m["cross_track_in"]
        out.append("")
        out.append(f"CROSS-TRACK  mean {c['mean']:.2f}  p95 {c['p95']:.2f}  max {c['max']:.2f} in")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "graphs"))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--path", default=None)
    ap.add_argument("--quiet-graphs", action="store_true")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    files = []
    for r in a.runs:
        files.extend(sorted(glob.glob(r)) or [r])
    chunks = [load(f) for f in files]
    d = concat(chunks) if len(chunks) > 1 else chunks[0]
    tag = a.tag or os.path.basename(files[0]).replace(".csv.gz", "").replace(".csv", "")
    path = json.load(open(a.path, encoding="utf-8")) if a.path else None

    m = analyse(d, tag, path)
    print(report(m))
    if not a.quiet_graphs:
        made = graphs(d, m, a.out, tag, path)
        print("\ngraphs:")
        for f in made:
            print(f"  {f}")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump(m, fh, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
