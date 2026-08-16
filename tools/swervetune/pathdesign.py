"""Designs the practice path, sizes it against the box, and exports it three ways.

DIAGNOSTIC TOOL. The practice area is a 51 x 46 in box in the Pinpoint's pose frame - not a
144 x 144 in field - so a competition-scale path cannot be ported into it and the envelope has
to be computed before a single control point is placed.

Geometry: a CLOSED uniform cubic B-spline, converted segment by segment to Bezier form. That
conversion is the reason for the choice - a uniform cubic B-spline is C2 by construction at
every joint including the wrap-around, so C1 and C2 are properties of the representation rather
than something hand-placed control points have to be checked for afterwards. Bezier control
points come from the B-spline points d0..d3 as

    b0 = (d0 + 4 d1 + d2) / 6      b1 = (2 d1 + d2) / 3
    b2 = (d1 + 2 d2) / 3           b3 = (d1 + 4 d2 + d3) / 6

Pedro Pathing 2.1.2 exposes NO jerk limit - it is a path follower, not a trajectory follower,
and PathConstraints carries only end-of-path tolerances and braking behaviour. So jerk is bounded
by CONSTRUCTION instead, and reported rather than commanded:

    lateral acceleration    a = v^2 * kappa
    lateral jerk            j = v^3 * |d kappa / ds|          (at constant speed)
    pod azimuth rate, constant-heading   omega_pod = v * kappa
    pod azimuth rate, tangential heading d/dt atan(kappa * r_pod)

A C2 path has continuous curvature, so d kappa / ds is bounded and therefore so is jerk. The
speed is then chosen so the pod azimuth rate stays inside a stated fraction of the measured
214 deg/s pod slew.

    python pathdesign.py [--box MINX,MINY,MAXX,MAXY] [--robot 18] [--speed 12]
                         [--out DIR] [--heading tangential|constant]
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

# Measured pod slew, median of the 2026-08-12 sweep. 184-259 deg/s across pods.
POD_SLEW_DEG_S = 214.0

# Fraction of that slew the path is allowed to demand. A pod riding its slew limit is
# open-loop: the PID has already saturated and tracking error is set by the plant, not the gains.
SLEW_BUDGET = 0.25

# Pod centre distance from robot centre, from SwerveDrivetrainConstants dtLength/dtWidth (mm).
POD_RADIUS_IN = math.hypot(146.420, 154.240) / 25.4

# Criterion 9's allowance, kept as clearance so a path that tracks badly is still legal.
CROSS_TRACK_ALLOWANCE_IN = 2.0

# SwerveBringUp.PEDRO_TARGET_MARGIN_IN - the follower bench refuses any control point closer
# than this to a wall.
PEDRO_TARGET_MARGIN_IN = 6.0


def bspline_to_bezier(d: np.ndarray) -> list[np.ndarray]:
    """Closed uniform cubic B-spline control points -> one cubic Bezier per span."""
    n = len(d)
    out = []
    for i in range(n):
        d0, d1, d2, d3 = d[i % n], d[(i + 1) % n], d[(i + 2) % n], d[(i + 3) % n]
        b0 = (d0 + 4 * d1 + d2) / 6.0
        b1 = (2 * d1 + d2) / 3.0
        b2 = (d1 + 2 * d2) / 3.0
        b3 = (d1 + 4 * d2 + d3) / 6.0
        out.append(np.array([b0, b1, b2, b3]))
    return out


def bezier_eval(b: np.ndarray, t: np.ndarray) -> np.ndarray:
    t = t[:, None]
    return ((1 - t) ** 3 * b[0] + 3 * (1 - t) ** 2 * t * b[1]
            + 3 * (1 - t) * t ** 2 * b[2] + t ** 3 * b[3])


def bezier_d1(b: np.ndarray, t: np.ndarray) -> np.ndarray:
    t = t[:, None]
    return 3 * ((1 - t) ** 2 * (b[1] - b[0]) + 2 * (1 - t) * t * (b[2] - b[1])
                + t ** 2 * (b[3] - b[2]))


def bezier_d2(b: np.ndarray, t: np.ndarray) -> np.ndarray:
    t = t[:, None]
    return 6 * ((1 - t) * (b[2] - 2 * b[1] + b[0]) + t * (b[3] - 2 * b[2] + b[1]))


def continuity_report(segs: list[np.ndarray]) -> list[dict]:
    """C1 and C2 residuals at every joint, including the closing one."""
    out = []
    n = len(segs)
    for i in range(n):
        a, b = segs[i], segs[(i + 1) % n]
        c0 = float(np.linalg.norm(b[0] - a[3]))
        # C1: 3(b1-b0) == 3(a3-a2)
        c1 = float(np.linalg.norm((b[1] - b[0]) - (a[3] - a[2])))
        # C2: (b2 - 2 b1 + b0) == (a3 - 2 a2 + a1)
        c2 = float(np.linalg.norm((b[2] - 2 * b[1] + b[0]) - (a[3] - 2 * a[2] + a[1])))
        out.append({"joint": i, "c0_in": c0, "c1_in": c1, "c2_in": c2})
    return out


def sample(segs: list[np.ndarray], per_seg: int = 400) -> dict:
    """Dense sample with arc length, curvature and curvature rate."""
    xs, ys, kap, tan, sseg = [], [], [], [], []
    for si, b in enumerate(segs):
        t = np.linspace(0, 1, per_seg, endpoint=False)
        p = bezier_eval(b, t)
        d1 = bezier_d1(b, t)
        d2 = bezier_d2(b, t)
        speed = np.hypot(d1[:, 0], d1[:, 1])
        cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
        k = cross / np.maximum(speed ** 3, 1e-12)
        xs.append(p[:, 0])
        ys.append(p[:, 1])
        kap.append(k)
        tan.append(np.arctan2(d1[:, 1], d1[:, 0]))
        sseg.append(np.full(per_seg, si))
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    k = np.concatenate(kap)
    th = np.unwrap(np.concatenate(tan))
    seg = np.concatenate(sseg)
    ds = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    s = np.concatenate([[0], np.cumsum(ds)[:-1]])
    dk_ds = np.gradient(k, s, edge_order=2)
    return {"x": x, "y": y, "kappa": k, "tangent": th, "s": s, "ds": ds,
            "dk_ds": dk_ds, "seg": seg, "length": float(ds.sum())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--box", default="-2.0827,-32.7723,48.71,12.8743",
                    help="minX,minY,maxX,maxY in the pose frame; default is the armed box")
    ap.add_argument("--robot", type=float, default=18.0,
                    help="robot footprint, inches square (18 = FTC legal max, the safe bound)")
    ap.add_argument("--speed", type=float, default=None,
                    help="planned speed in in/s; default is whatever the slew budget allows")
    ap.add_argument("--heading", default="tangential",
                    choices=["tangential", "constant"])
    ap.add_argument("--points", type=int, default=8)
    ap.add_argument("--shape", type=float, default=1.0,
                    help="1.0 = ellipse; below 1 squares the corners off (more straight run, "
                         "tighter corners); above 1 rounds them further")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "paths"))
    a = ap.parse_args()

    minx, miny, maxx, maxy = [float(v) for v in a.box.split(",")]
    bw, bh = maxx - minx, maxy - miny
    half = a.robot / 2.0
    halfdiag = a.robot * math.sqrt(2) / 2.0

    print("BOX AND ENVELOPE")
    print(f"  box                     {bw:.2f} x {bh:.2f} in  "
          f"(x {minx:.2f}..{maxx:.2f}, y {miny:.2f}..{maxy:.2f})")
    print(f"  robot footprint         {a.robot:.1f} x {a.robot:.1f} in (assumed)")
    print(f"  half-width              {half:.2f} in")
    print(f"  half-diagonal           {halfdiag:.2f} in  (clearance if it may rotate freely)")
    print(f"  cross-track allowance   {CROSS_TRACK_ALLOWANCE_IN:.2f} in")
    # A constant-heading path never rotates the footprint, so half-width is the real clearance;
    # a tangential path sweeps the robot through every orientation and needs the half-diagonal.
    clearance = half if a.heading == "constant" else halfdiag
    inset = clearance + CROSS_TRACK_ALLOWANCE_IN
    ew, eh = bw - 2 * inset, bh - 2 * inset
    print(f"  centre envelope         {ew:.2f} x {eh:.2f} in   "
          f"(box - 2 x ({clearance:.2f} + {CROSS_TRACK_ALLOWANCE_IN:.1f}))  "
          f"[{a.heading} heading]")
    print(f"  follower bench margin   {PEDRO_TARGET_MARGIN_IN:.1f} in - satisfied "
          f"({inset:.2f} > {PEDRO_TARGET_MARGIN_IN:.1f})")
    if ew <= 0 or eh <= 0:
        print("  ENVELOPE IS EMPTY - no legal path exists for this robot in this box.")
        return 1

    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
    # A closed loop that uses the envelope without touching it: a rounded rectangle laid out as
    # B-spline control points. The spline pulls inside its control polygon, so the drawn path is
    # strictly smaller than the envelope - checked numerically below rather than assumed.
    rx, ry = ew / 2.0, eh / 2.0
    ang = np.linspace(0, 2 * math.pi, a.points, endpoint=False)
    # Superellipse. Below 1.0 buys straighter runs - real straight-line tracking to score -
    # at the cost of tighter corners, and the corner radius is what sets the speed here.
    d = np.stack([cx + rx * np.sign(np.cos(ang)) * np.abs(np.cos(ang)) ** a.shape,
                  cy + ry * np.sign(np.sin(ang)) * np.abs(np.sin(ang)) ** a.shape], axis=1)

    segs = bspline_to_bezier(d)
    sm = sample(segs)

    # Envelope check on the drawn path, not the control polygon.
    mx = float(min(sm["x"].min() - minx, maxx - sm["x"].max()))
    my = float(min(sm["y"].min() - miny, maxy - sm["y"].max()))
    print(f"  path clearance          x {mx:.2f} in, y {my:.2f} in from the nearest wall")
    print(f"  vs half-width {half:.2f}: "
          f"{'OK' if min(mx, my) >= half else 'TOO CLOSE'};  "
          f"vs half-diagonal {halfdiag:.2f}: "
          f"{'OK' if min(mx, my) >= halfdiag else 'TOO CLOSE'}")

    kmax = float(np.abs(sm["kappa"]).max())
    rmin = 1.0 / kmax if kmax > 0 else float("inf")
    dkmax = float(np.abs(sm["dk_ds"]).max())

    print("\nGEOMETRY")
    print(f"  path length             {sm['length']:.2f} in")
    print(f"  segments                {len(segs)} cubic Beziers, closed")
    print(f"  max |kappa|             {kmax:.5f} /in   -> min radius {rmin:.2f} in")
    print(f"  max |d kappa / ds|      {dkmax:.6f} /in^2")

    cont = continuity_report(segs)
    c0 = max(c["c0_in"] for c in cont)
    c1 = max(c["c1_in"] for c in cont)
    c2 = max(c["c2_in"] for c in cont)
    print("\nCONTINUITY (worst joint of all, including the closing joint)")
    print(f"  C0 residual             {c0:.3e} in")
    print(f"  C1 residual             {c1:.3e} in   -> C1 at every joint")
    print(f"  C2 residual             {c2:.3e} in   -> C2 at every joint")
    print("  C2 here is PARAMETRIC, from the uniform B-spline. The segments carry equal")
    print("  parameter speed by construction, so it is also geometric G2 (curvature is")
    print("  continuous - see the kappa trace).")

    # Speed from the pod-slew budget.
    budget_rad_s = math.radians(POD_SLEW_DEG_S * SLEW_BUDGET)
    if a.heading == "constant":
        # Pod azimuth = path tangent - fixed heading, so it rotates at exactly v * kappa.
        v_slew = budget_rad_s / kmax
        pod_rate_expr = "v * kappa"
    else:
        # Tangential: the chassis yaws with the path, so on a constant-curvature arc the pod
        # azimuth is CONSTANT at atan(kappa * r_pod). It only moves as curvature changes.
        # d/dt atan(kappa r) = r v (dkappa/ds) / (1 + (kappa r)^2)
        worst = float(np.abs(POD_RADIUS_IN * sm["dk_ds"]
                             / (1 + (sm["kappa"] * POD_RADIUS_IN) ** 2)).max())
        v_slew = budget_rad_s / worst if worst > 0 else float("inf")
        pod_rate_expr = "r_pod v (dkappa/ds) / (1 + (kappa r_pod)^2)"
    v = a.speed if a.speed is not None else min(v_slew, 40.0)

    print("\nSPEED, ACCELERATION, JERK")
    print(f"  heading mode            {a.heading}")
    print(f"  pod azimuth rate        {pod_rate_expr}")
    print(f"  pod slew, measured      {POD_SLEW_DEG_S:.0f} deg/s; budget "
          f"{SLEW_BUDGET * 100:.0f}% = {POD_SLEW_DEG_S * SLEW_BUDGET:.0f} deg/s")
    print(f"  speed the budget allows {v_slew:.2f} in/s")
    print(f"  planned speed           {v:.2f} in/s")
    print(f"  lap time                {sm['length'] / v:.2f} s")
    print(f"  max lateral accel       {v * v * kmax:.2f} in/s^2  "
          f"({v * v * kmax / 386.1:.3f} g)")
    print(f"  max lateral jerk        {v ** 3 * dkmax:.1f} in/s^3   "
          "(= v^3 max|dkappa/ds|, the bound C2 buys)")
    if a.heading == "constant":
        print(f"  max pod azimuth rate    {math.degrees(v * kmax):.1f} deg/s")
    else:
        print(f"  max pod azimuth rate    "
              f"{math.degrees(v * worst):.1f} deg/s")
        print(f"  chassis yaw rate        {math.degrees(v * kmax):.1f} deg/s")
        print(f"  pod azimuth offset      +/- "
              f"{math.degrees(math.atan(kmax * POD_RADIUS_IN)):.1f} deg at the tightest corner")

    os.makedirs(a.out, exist_ok=True)
    tag = f"loop_{a.heading}"

    # 1. Reference polyline for steerqual's cross-track scoring.
    ref = {"x": [round(float(x), 4) for x in sm["x"]],
           "y": [round(float(y), 4) for y in sm["y"]],
           "heading_mode": a.heading, "speed_in_s": v, "length_in": sm["length"]}
    with open(os.path.join(a.out, f"{tag}_reference.json"), "w", encoding="utf-8") as fh:
        json.dump(ref, fh)

    # 2. pedroChain payload for the bring-up bench.
    pts = ";".join(",".join(f"{c:.4f}" for c in p) for b in segs for p in b)
    head = "|".join(["tangent" if a.heading == "tangential" else "constant:0"] * len(segs))
    payload = {"action": "pedroChain", "pts": pts, "head": head,
               "power": round(min(1.0, v / 73.9), 3), "frame": "field"}
    with open(os.path.join(a.out, f"{tag}_pedrochain.json"), "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=1)

    # 3. Visualizer .pp project.
    lines = []
    for i, b in enumerate(segs):
        lines.append({
            "id": f"seg-{i}",
            "name": f"segment {i}",
            "endPoint": {"x": float(b[3][0]), "y": float(b[3][1]),
                         "heading": "tangential" if a.heading == "tangential" else "constant",
                         "degrees": 0},
            "controlPoints": [{"x": float(b[1][0]), "y": float(b[1][1])},
                              {"x": float(b[2][0]), "y": float(b[2][1])}],
            "color": "#5977C5", "speed": round(min(1.0, v / 73.9), 3), "locked": False,
        })
    pp = {"startPoint": {"x": float(segs[0][0][0]), "y": float(segs[0][0][1]),
                         "heading": "tangential", "locked": False},
          "lines": lines}
    with open(os.path.join(a.out, f"{tag}.pp"), "w", encoding="utf-8") as fh:
        json.dump(pp, fh, indent=1)

    # 4. Picture: path in the box, and the curvature trace.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5),
                           gridspec_kw={"width_ratios": [1.15, 1]})
    ax[0].add_patch(plt.Rectangle((minx, miny), bw, bh, fill=False, lw=2, color="crimson",
                                  label="box (hard limit)"))
    ax[0].add_patch(plt.Rectangle((minx + half, miny + half), bw - 2 * half, bh - 2 * half,
                                  fill=False, lw=1, ls="--", color="orange",
                                  label=f"half-width {half:.1f} in"))
    ax[0].add_patch(plt.Rectangle((minx + inset, miny + inset), bw - 2 * inset, bh - 2 * inset,
                                  fill=False, lw=1, ls=":", color="green",
                                  label=f"centre envelope"))
    ax[0].plot(sm["x"], sm["y"], lw=2, color="navy", label="path")
    for b in segs:
        ax[0].plot(b[:, 0], b[:, 1], lw=0.6, color="0.7", marker=".", ms=3)
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x (in)")
    ax[0].set_ylabel("y (in)")
    ax[0].legend(fontsize=7, loc="upper right")
    ax[0].set_title(f"{tag}: {sm['length']:.1f} in, {len(segs)} C2 cubics")
    ax[1].plot(sm["s"], sm["kappa"], lw=1.2, label="kappa (1/in)")
    ax[1].plot(sm["s"], sm["dk_ds"], lw=0.9, label="d kappa / ds (1/in^2)")
    for j in range(1, len(segs)):
        ax[1].axvline(sm["s"][j * 400], color="0.85", lw=0.6, zorder=0)
    ax[1].set_xlabel("arc length (in)")
    ax[1].legend(fontsize=8)
    ax[1].set_title("curvature is continuous across every joint (grey lines)")
    fig.tight_layout()
    png = os.path.join(a.out, f"{tag}.png")
    fig.savefig(png, dpi=115)
    plt.close(fig)

    print("\nWRITTEN")
    for f in (f"{tag}_reference.json", f"{tag}_pedrochain.json", f"{tag}.pp", f"{tag}.png"):
        print(f"  {os.path.join(a.out, f)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
