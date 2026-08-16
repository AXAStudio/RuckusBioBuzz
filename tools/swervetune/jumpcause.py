"""Attributes every large azimuth-setpoint jump to the branch of the mixer that caused it.

DIAGNOSTIC TOOL. The recorder logs the applied robot-frame command (cf, cs, ct) alongside
every pod's target, so the whole demand chain can be replayed host-side and each discontinuity
blamed on a specific line of Java rather than guessed at.

Replayed here, in the order SwerveBringUp/Swerve apply them:

  1. box fence      applyBoxLimit zeroes a FIELD axis (vx or vy) independently - detectable
                    after the fact because the applied command then has an exactly-zero field
                    component while the other is live
  2. epsilon walls  |trans| < 0.05 drops the translation term entirely; |rot| < 0.015 (mixer)
                    or < 0.05 (computeTargets' mirror) drops the rotation term
  3. X-lock         zero translation AND zero rotation for 0.35 s parks the pods on their own
                    radii, the +-43.5 / +-136.5 deg family
  4. flip           the pod's own shortest-path 180 deg decision, logged per pod

Usage:
    python jumpcause.py runs/<file>.csv.gz [--limit 15]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

from steerqual import load, concat, wrap180, drive_mask, POD_NAMES

POD_COUNT = 4

# Geometry from SwerveBringUp.init: index 0=RB, 1=RF, 2=LF, 3=LB.
DT_LENGTH = 146.420
DT_WIDTH = 154.240
POD_XY = {
    0: (-DT_LENGTH, -DT_WIDTH),
    1: (DT_LENGTH, -DT_WIDTH),
    2: (DT_LENGTH, DT_WIDTH),
    3: (-DT_LENGTH, DT_WIDTH),
}

SWERVE_EPSILON = 0.05        # SwerveBringUp.SWERVE_EPSILON, and Swerve's translation epsilon
ROTATION_EPSILON = 0.015     # Swerve.ROTATION_EPSILON (the mixer's real rotation wall)
X_LOCK_DELAY_S = 0.35        # Swerve.X_LOCK_ENGAGE_DELAY_S


def compute_targets(cf, cs, ct, rot_eps):
    """Replays SwerveBringUp.computeTargets for one sample. Returns (theta_deg[4], flags)."""
    forward = cf
    strafe = -cs
    rotation = ct
    trans_mag = min(1.0, math.hypot(strafe, forward))
    trans_theta = math.atan2(forward, strafe)
    zero_trans = trans_mag < SWERVE_EPSILON
    zero_rot = abs(rotation) < rot_eps

    if zero_trans and zero_rot:
        return ([math.degrees(math.atan2(POD_XY[i][0], -POD_XY[i][1])) % 360.0
                 for i in range(POD_COUNT)], {"xlock": True, "zt": True, "zr": True})

    rot_scalar = 0.0 if zero_rot else rotation
    out = []
    for i in range(POD_COUNT):
        tx = 0.0 if zero_trans else trans_mag * math.cos(trans_theta)
        ty = 0.0 if zero_trans else trans_mag * math.sin(trans_theta)
        rot_theta = math.atan2(POD_XY[i][0], -POD_XY[i][1]) + math.pi / 2.0
        px = tx + rot_scalar * math.cos(rot_theta)
        py = ty + rot_scalar * math.sin(rot_theta)
        out.append(math.degrees(math.atan2(py, px)) % 360.0)
    return out, {"xlock": False, "zt": zero_trans, "zr": zero_rot}


def field_components(cf, cs, heading_deg):
    h = math.radians(heading_deg)
    return cf * math.cos(h) - cs * math.sin(h), cf * math.sin(h) + cs * math.cos(h)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--limit", type=float, default=15.0)
    a = ap.parse_args()

    chunks = [load(f) for f in a.runs]
    d = concat(chunks) if len(chunks) > 1 else chunks[0]
    n = len(d["dt"])
    active = drive_mask(d)

    cf, cs, ct = d["cf"], d["cs"], d["ct"]
    heading = d["heading"]

    # ---- 1. validate the replay against the logged targets
    rep = np.full((n, POD_COUNT), math.nan)
    rep_mixer = np.full((n, POD_COUNT), math.nan)
    zt = np.zeros(n, dtype=bool)
    zr = np.zeros(n, dtype=bool)
    xl = np.zeros(n, dtype=bool)
    for k in range(n):
        if math.isnan(cf[k]) or math.isnan(cs[k]) or math.isnan(ct[k]):
            continue
        t, f = compute_targets(cf[k], cs[k], ct[k], SWERVE_EPSILON)
        rep[k] = t
        zt[k], zr[k], xl[k] = f["zt"], f["zr"], f["xlock"]
        rep_mixer[k] = compute_targets(cf[k], cs[k], ct[k], ROTATION_EPSILON)[0]

    resid = []
    for i in range(POD_COUNT):
        r = np.abs(wrap180(rep[:, i] - d[f"p{i}_tgt"]))
        resid.append(r)
    resid = np.array(resid)
    ok = ~np.isnan(resid)
    print("REPLAY VALIDATION (host replay of computeTargets vs logged tgt)")
    print(f"  samples matched   {int(ok[0].sum())} / {n}")
    print(f"  |residual| mean   {np.nanmean(resid):.4f} deg")
    print(f"  |residual| p99    {np.nanpercentile(resid, 99):.4f} deg")
    print(f"  |residual| max    {np.nanmax(resid):.4f} deg")

    # ---- 2. how much of the demand differs between the mirror and the real mixer
    mism = np.abs(wrap180(rep - rep_mixer))
    mism_any = np.nanmax(mism, axis=1)
    n_mism = int(np.nansum(mism_any > 1.0))
    print("\nMIRROR DIVERGENCE (computeTargets uses rot epsilon 0.05, the mixer uses 0.015)")
    print(f"  samples where the logged target is not what the pods were commanded: "
          f"{n_mism} ({100.0 * n_mism / n:.1f}%)")
    if n_mism:
        print(f"  worst disagreement {np.nanmax(mism_any):.1f} deg")

    # ---- 3. box-fence detection: an exactly-zero field axis with the other live
    vx = np.full(n, math.nan)
    vy = np.full(n, math.nan)
    for k in range(n):
        if math.isnan(cf[k]) or math.isnan(cs[k]) or math.isnan(heading[k]):
            continue
        vx[k], vy[k] = field_components(cf[k], cs[k], heading[k])
    fenced = ((np.abs(vx) < 1e-9) & (np.abs(vy) > 1e-6)) | \
             ((np.abs(vy) < 1e-9) & (np.abs(vx) > 1e-6))
    fenced = fenced & active
    print("\nBOX FENCE (applyBoxLimit zeroes one FIELD axis, so a clamped sample has an "
          "exactly-zero component)")
    print(f"  clamped samples   {int(fenced.sum())} ({100.0 * fenced.sum() / max(1, active.sum()):.2f}% of driving)")
    if fenced.sum():
        edges = np.diff(fenced.astype(int))
        print(f"  clamp engagements {int((edges > 0).sum())}  releases {int((edges < 0).sum())}")

    # ---- 4. attribute each jump
    # An axis-deadband event: one translation axis is EXACTLY zero on one side of the step and
    # live on the other, while the other axis stays live. That is the signature of a per-axis
    # deadband (dashboard.html padAxis 0.06, DriveTeleOp.applyDeadband 0.05) rotating the
    # commanded direction rather than shortening it.
    fz = np.abs(np.nan_to_num(cf, nan=1.0)) < 1e-12
    sz = np.abs(np.nan_to_num(cs, nan=1.0)) < 1e-12
    axis_dead = (fz & ~sz) | (sz & ~fz)

    for which, series in (("logged tgt (recorder mirror)", None),
                          ("mixer demand (rot epsilon 0.015 - what the pods chased)", rep_mixer)):
        print(f"\nJUMP ATTRIBUTION - {which}")
        print(f"  consecutive-loop |d target| > {a.limit:g} deg, while driving")
        print(f"  {'pod':>5} {'jumps':>6} {'/s':>6} {'xlock':>6} {'transEps':>9} {'rotEps':>7} "
              f"{'axisDB':>7} {'fence':>6} {'flip':>5} {'other':>6}")
        totals = dict(jumps=0, xlock=0, teps=0, reps=0, adb=0, fence=0, flip=0, other=0)
        other_examples = []
        span_active = float(np.nansum(np.where(active, d["dt"], 0.0)))
        for i in range(POD_COUNT):
            tgt = d[f"p{i}_tgt"] if series is None else series[:, i]
            flip = np.nan_to_num(d[f"p{i}_flip"]) > 0.5
            dtg = np.abs(wrap180(np.diff(tgt)))
            idx = np.where((dtg > a.limit) & active[1:] & ~np.isnan(dtg))[0] + 1
            c = dict(xlock=0, teps=0, reps=0, adb=0, fence=0, flip=0, other=0)
            for k in idx:
                if xl[k] != xl[k - 1]:
                    c["xlock"] += 1
                elif zt[k] != zt[k - 1]:
                    c["teps"] += 1
                elif zr[k] != zr[k - 1]:
                    c["reps"] += 1
                elif axis_dead[k] != axis_dead[k - 1]:
                    c["adb"] += 1
                elif fenced[k] != fenced[k - 1]:
                    c["fence"] += 1
                elif flip[k] != flip[k - 1]:
                    c["flip"] += 1
                else:
                    c["other"] += 1
                    if len(other_examples) < 10:
                        other_examples.append((i, k, dtg[k - 1], cf[k], cs[k], ct[k],
                                               cf[k - 1], cs[k - 1], ct[k - 1]))
            print(f"  {i} {POD_NAMES[i]:>3} {len(idx):>6} {len(idx) / span_active:>6.2f} "
                  f"{c['xlock']:>6} {c['teps']:>9} {c['reps']:>7} {c['adb']:>7} "
                  f"{c['fence']:>6} {c['flip']:>5} {c['other']:>6}")
            totals["jumps"] += len(idx)
            for k2 in c:
                totals[k2] += c[k2]
        print(f"  {'ALL':>5} {totals['jumps']:>6} {'':>6} {totals['xlock']:>6} "
              f"{totals['teps']:>9} {totals['reps']:>7} {totals['adb']:>7} {totals['fence']:>6} "
              f"{totals['flip']:>5} {totals['other']:>6}")
        if other_examples:
            print("  unattributed examples (pod, sample, jump deg, cmd now <- cmd prev):")
            for (i, k, j, f1, s1, t1, f0, s0, t0) in other_examples:
                print(f"    pod{i} @{k:5d}  {j:6.1f} deg   "
                      f"f/s/t {f1:+.3f}/{s1:+.3f}/{t1:+.3f}  <-  "
                      f"{f0:+.3f}/{s0:+.3f}/{t0:+.3f}")

    # ---- 4b. how often does an axis get zeroed at all
    print("\nPER-AXIS DEADBAND EXPOSURE")
    n_act = max(1, int(active.sum()))
    print(f"  samples with exactly one translation axis zeroed: "
          f"{int((axis_dead & active).sum())} ({100.0 * (axis_dead & active).sum() / n_act:.1f}% of driving)")
    edges = np.diff((axis_dead & active).astype(int))
    print(f"  transitions in/out of that state: {int((edges != 0).sum())} "
          f"({(edges != 0).sum() / float(np.nansum(np.where(active, d['dt'], 0.0))):.2f}/s)")

    # ---- 5. how large is the demand step at each epsilon crossing
    print("\nEPSILON-WALL STEP SIZE (demand angle change across a translation-epsilon crossing)")
    for i in range(POD_COUNT):
        tgt = d[f"p{i}_tgt"]
        dtg = np.abs(wrap180(np.diff(tgt)))
        cross = (zt[1:] != zt[:-1]) & active[1:]
        vals = dtg[cross]
        vals = vals[~np.isnan(vals)]
        if len(vals):
            print(f"  pod {i} {POD_NAMES[i]}: n={len(vals):4d}  mean {vals.mean():6.1f}  "
                  f"p90 {np.percentile(vals, 90):6.1f}  max {vals.max():6.1f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
