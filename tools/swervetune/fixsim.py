"""Replays a recorded drive through the old and new mixers and compares the demand.

DIAGNOSTIC TOOL. The recorder logs the applied robot-frame command (cf, cs, ct) - which is
exactly the input Swerve.arcadeDrive received - so both versions of the mixer can be run over a
real driving session host-side and the effect of each fix measured before any robot time is
spent on it.

What is simulated exactly:
  * epsilon taper      (Swerve.epsilonTaper)      - operates on cf/cs/ct, so this is exact
  * demand slew limit  (Swerve.demandSlewDegPerSec) - same

What is NOT simulated, and why:
  * the per-axis deadband fix (DriveTeleOp.applyDeadband, dashboard padAxis) acts UPSTREAM of
    what was recorded - the logged cf/cs are already deadbanded, and the raw stick values are
    gone. Its effect is counted structurally instead: every jump jumpcause.py attributes to an
    axis-deadband transition disappears by construction.
  * clampReversePower is not in the bring-up path at all, so no archived run contains it.

    python fixsim.py runs/<file>.csv.gz [--slew 300] [--limit 15]
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from steerqual import (load, concat, wrap90, drive_mask, reversals, unwrap_deg,
                       POD_NAMES)
from jumpcause import POD_XY, SWERVE_EPSILON, ROTATION_EPSILON, X_LOCK_DELAY_S

POD_COUNT = 4


def taper(mag: float, band: float) -> float:
    if band <= 0 or mag >= band:
        return 1.0
    u = mag / band
    return u * u * (3.0 - 2.0 * u)


def mixer(cf, cs, ct, dt, use_taper: bool, slew_deg_s: float):
    """Runs the whole series through one configuration of the mixer. Returns demand in degrees."""
    n = len(cf)
    out = np.full((n, POD_COUNT), math.nan)
    last = [math.nan] * POD_COUNT
    quiet_for = 0.0
    max_delta = math.pi / 2.0
    for k in range(n):
        if math.isnan(cf[k]) or math.isnan(cs[k]) or math.isnan(ct[k]):
            continue
        forward, strafe, rotation = cf[k], -cs[k], ct[k]
        trans_mag = min(1.0, math.hypot(strafe, forward))
        trans_theta = math.atan2(forward, strafe)
        zero_trans = trans_mag < SWERVE_EPSILON
        zero_rot = abs(rotation) < ROTATION_EPSILON

        step = dt[k] if (not math.isnan(dt[k]) and 0 < dt[k] < 0.5) else 0.0
        quiet_for = quiet_for + step if (zero_trans and zero_rot) else 0.0
        xlock_ripe = zero_trans and zero_rot and quiet_for >= X_LOCK_DELAY_S

        t_scale = taper(trans_mag, SWERVE_EPSILON) if use_taper else 0.0
        r_scale = taper(abs(rotation), ROTATION_EPSILON) if use_taper else 0.0
        rot_scalar = rotation * r_scale if zero_rot else rotation
        eff_trans = trans_mag * t_scale if zero_trans else trans_mag

        max_step = math.radians(slew_deg_s) * step if (slew_deg_s > 0 and step > 0) else math.inf

        for i in range(POD_COUNT):
            if xlock_ripe:
                theta = math.atan2(POD_XY[i][0], -POD_XY[i][1])
                if not math.isnan(last[i]) and max_step != math.inf:
                    delta = (theta - last[i] + math.pi) % (2 * math.pi) - math.pi
                    if abs(delta) <= max_delta:
                        theta = last[i] + max(-max_step, min(max_step, delta))
                last[i] = theta
                out[k, i] = math.degrees(theta) % 360.0
                continue
            tx = eff_trans * math.cos(trans_theta)
            ty = eff_trans * math.sin(trans_theta)
            rot_theta = math.atan2(POD_XY[i][0], -POD_XY[i][1]) + math.pi / 2.0
            px = tx + rot_scalar * math.cos(rot_theta)
            py = ty + rot_scalar * math.sin(rot_theta)
            theta = math.atan2(py, px)
            if not math.isnan(last[i]) and max_step != math.inf:
                delta = (theta - last[i] + math.pi) % (2 * math.pi) - math.pi
                if abs(delta) <= max_delta:
                    if delta > max_step:
                        theta = last[i] + max_step
                    elif delta < -max_step:
                        theta = last[i] - max_step
            last[i] = theta
            out[k, i] = math.degrees(theta) % 360.0
    return out


def score(demand: np.ndarray, active: np.ndarray, dt: np.ndarray, limit: float) -> dict:
    span = float(np.nansum(np.where(active, dt, 0.0)))
    rows = []
    for i in range(POD_COUNT):
        col = demand[:, i]
        # wrap90: the physical azimuth change. A near-180 demand change costs the pod no
        # travel - it flips and reverses the drive - so it is not a jump.
        dtg = np.abs(wrap90(np.diff(col)))
        m = active[1:] & ~np.isnan(dtg)
        ordinary = dtg[m]
        j = ordinary
        rows.append({
            "pod": i,
            "p90": float(np.percentile(ordinary, 90)) if len(ordinary) else math.nan,
            "p99": float(np.percentile(ordinary, 99)) if len(ordinary) else math.nan,
            "max": float(ordinary.max()) if len(ordinary) else math.nan,
            "over": int((ordinary > limit).sum()),
            "over_per_s": float((ordinary > limit).sum() / span) if span else math.nan,
            "near90": int((j >= 85.0).sum()),
            "rev_per_s": reversals(unwrap_deg(col[active])) / span if span else math.nan,
            "path_deg": float(np.nansum(np.abs(np.diff(unwrap_deg(col[active]))))),
        })
    return {"span_s": span, "pods": rows}


def show(name: str, s: dict) -> None:
    print(f"\n{name}")
    print(f"  {'pod':>5} {'jumpP90':>8} {'jumpP99':>8} {'jumpMax':>8} {'>lim':>6} {'>lim/s':>7} "
          f"{'>=85':>6} {'rev/s':>7} {'pathDeg':>9}")
    for r in s["pods"]:
        print(f"  {r['pod']} {POD_NAMES[r['pod']]:>3} {r['p90']:>8.2f} {r['p99']:>8.2f} "
              f"{r['max']:>8.2f} {r['over']:>6} {r['over_per_s']:>7.2f} {r['near90']:>6} "
              f"{r['rev_per_s']:>7.2f} {r['path_deg']:>9.0f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--slew", type=float, default=300.0)
    ap.add_argument("--limit", type=float, default=15.0)
    a = ap.parse_args()

    chunks = [load(f) for f in a.runs]
    d = concat(chunks) if len(chunks) > 1 else chunks[0]
    active = drive_mask(d)
    cf, cs, ct, dt = d["cf"], d["cs"], d["ct"], d["dt"]

    print(f"Replaying {len(cf)} samples, {float(np.nansum(np.where(active, dt, 0))):.1f} s of "
          f"driving, criterion-1 limit {a.limit:g} deg")

    configs = [
        ("A  as shipped 2026-08-15 (hard walls, no slew limit)", False, 0.0),
        ("B  epsilon taper only", True, 0.0),
        (f"C  demand slew {a.slew:g} deg/s only", False, a.slew),
        (f"D  taper + slew {a.slew:g} deg/s  (the deployed default)", True, a.slew),
    ]
    for name, tp, sl in configs:
        show(name, score(mixer(cf, cs, ct, dt, tp, sl), active, dt, a.limit))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
