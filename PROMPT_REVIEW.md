## Verdict on ultracode

Yes — but in three short bursts, not as a persistent swarm, and **not at all during Tasks 0–2 on-robot work.**

The evidence is this review itself. Five independent lenses over the same two files produced findings a single linear pass would not have: that criterion 4 is arithmetically unreachable, that `cache = 0.01` is a servo *write* deadband and not a read cache, that `CoaxialPod` lives in the vendored Pedro tree and not in `TeamCode/`, that `PathConstraints.setVelocityConstraint()` is an end-of-path tolerance and not a speed cap, that the 30.9 Hz / 29% / 71% baseline triple cannot all be true. Every one of those is *wide independent read work* — grep a 193 KB OpMode, clone a dependency at a tag, redo someone else's arithmetic. That is exactly what fan-out buys, and it is the only thing that is going to keep the primary agent from spending its whole context window reading `SwerveBringUp.java`.

**Fan out here:**

- **Phase A — source audit, before the first OPS REQUEST. ~6 agents, all read-only.** One per Task 2 hypothesis cluster (clamp path; X-lock + input paths; encoder conversion + rounding grep; flip/optimize step), one on `publish()` and bulk caching for Task 0, one that reads the *vendored* `third_party/PedroPathing` tree and reports the real API surface + how it diverges from upstream 2.1.2. This is the single highest-value fan-out in the run: it is context-expensive, embarrassingly parallel, and it happens while the operator is not even in the room.
- **Phase B — archive reanalysis. 1–2 agents.** Signed residual by approach direction, dt-weighted restatement of every baseline number, TV noise floor, and the 30.9 Hz reconciliation — all from `trials.jsonl` and `runs/` with zero robot time. May close criterion 3 outright before the robot is touched.
- **Phase C — Task 3 offline + verification tooling. 2 agents, running during OPS waits.** One builds the path and the numeric envelope/continuity checker; one builds the six plotting scripts. These are the deliverables that get dropped when the run runs out of clock.
- **Phase E — adversarial verification. 1 short-lived agent per claimed root cause**, before that claim is allowed to spend robot time. This repo has three retracted statistical claims and one fabricated one. A cheap red-team pass on "I think criterion 3 is X" is worth more than another twenty pod-runs.

**Do not fan out Tasks 0–2 on-robot execution.** One robot, one recorder that stops at 3000 samples and does not wrap, one human doing reflashes, and A/B arms that must be randomized and interleaved *within a battery*. Parallel agents there do not go faster; they queue behind the same operator, contend for the same `/state`, and corrupt each other's arms. One agent, one queue.

Rough shape: ~10–12 agent-runs total, front-loaded, ~70% of the value in Phase A. Anything beyond that is theatre.

---

## Critical patches

### 1. Criterion 4 is arithmetically impossible in software

**Problem:** 90° at the measured 214 °/s median slew is 421 ms of pure saturation — 71 ms more than the entire 350 ms budget — before any settling. No gain value is in the equation during rate saturation.

Replace row 4 of the criteria table (SWERVE_TASK.md:335):

```
| 4 | 90° azimuth step settle to ±2° | **≤ 500 ms software-only**; < 350 ms needs ~1.6× pod slew, i.e. hardware | ~647 ms = 421 ms saturated + 226 ms settle |
```

Add immediately below the table:

```
**Criterion 4 is slew-bound, not gain-bound.** CLAUDE.md's 421 ms of gross travel at
214 °/s already exceeds the old 350 ms target, and the pod is command-saturated for all
of it — gains act only on the remaining ~226 ms. Tune against 480–500 ms. Report
`t_sat` (|θ̇| > 0.9 × 214 °/s) and `t_settle` separately for every step trial; if only
`t_settle` is moving, gain work is finished and the remaining lever is the 2× overdrive
(estimated ~2× slew, 540° coverage — unmeasured, verify on the robot). Do not raise kP
to chase the step: it worsens relay chatter and fights criterion 3.
```

### 2. Criterion 3's cause is not "unknown" — the archive already localizes it, and the search order is wrong

**Problem:** the prompt says the cause is unknown and lists four undifferentiated candidates, while CLAUDE.md §5 records a 1.29× change in kS moving the residual 1.97× — a friction-deadband signature no authority-independent mechanism can produce. Meanwhile every residual figure is `|ss|`, which destroys the bias-vs-limit-cycle discriminator for free; a write-deadband hypothesis with matching arithmetic is nowhere in the file; and the plan is to grid-search gains on a step that is 65% saturated, where the gains are not identifiable.

Replace the criterion-3 paragraph (SWERVE_TASK.md:344–352) entirely:

```
Criterion 3 is the known blocker, but its cause is **not unknown — the archive already
localizes it.** CLAUDE.md §5: |ss| = **2.92° at kS = 0.035** vs **1.48° at kS = 0.045** —
a 1.29× change in kS moved the residual 1.97×. Loop rate (REFUTED, p = 0.70) and creep
quantum (REFUTED) are both independent of command authority and cannot produce that.
Work these in order. Do not start at step 4.

1. **Zero-robot-time reanalysis. Do this before requesting robot time for criterion 3.**
   Every residual in this repo is `|ss|`; the abs() hides the discriminator. Recompute
   signed `wrap(tgt − wheel)` from `runs/` and `tools/swervetune/trials.jsonl`: mean ±
   95% CI per pod, **split by step direction** (sign of the commanded step, recoverable
   from `tgt`). Both directions parking short ⇒ stiction/hysteresis: the pending
   lubrication pass, `kS`, and a `kI` band are the levers, and the gap between directions
   is the hysteresis width. Signed mean ≈ 0 with 2.7° RMS ⇒ symmetric relay limit cycle:
   `kD` and slew are the levers. If it is a limit cycle, report its dominant frequency
   separately in the **47.8 Hz and 92.1 Hz** arms already in `trials.jsonl` — frequency
   scaling with loop rate ⇒ discrete-time relay limit cycle; fixed frequency ⇒
   mechanical. This re-reads the refuted loop-rate A/B; it does not re-run it.

2. **The `servoCachingThreshold` write deadband — untested and the arithmetic lands
   inside the measured band.** `CoaxialPod.move()` calls `turnServo.setPower()` only when
   `|turnPower − lastTurnPower| > 0.01`, and it feeds the PID error in **radians**. With
   `turnKP = 0.200`, a 0.01 command corresponds to `0.05 rad = 2.86°`. Inside
   `turnKSBandDeg = 2.0` the kS relay is forced to zero, so output is `kP·err ≤ 0.0070` —
   under the threshold, so the command goes **stale** rather than updating. 2.86° sits
   inside the measured 2.65–3.01° residual. Caveat: kS = 0.045 measured 1.48°, also below
   threshold, so this is **not** a hard floor — a stale CRServo command is a latched
   *speed*. Treat it as a limit-cycle hypothesis. Test: sweep `servoCachingThreshold` over
   {0.01, 0.003, 0.001, 0.0} at fixed kP/kD, randomized and interleaved, reading |ss| off
   the `wheel` column. Lower values raise the write rate toward the 20 ms / 50 Hz PWM
   frame — report loop dt per arm.

3. **Mechanical backlash and per-direction dead zone.** The encoder is 1:1 with the pod,
   i.e. downstream of any lash, so lash is a plant dead zone the controller cannot see
   through. With kI = 0 and kS acting as a relay, that puts a floor under settled error
   that no gain change can move — which is what the residual's indifference to loop rate,
   creep quantum, and every gain tried so far looks like. **Test (~10 min, no code change,
   batch into an existing on-tiles OPS REQUEST):** command one absolute azimuth target 20
   times, 10 approaches from CW and 10 from CCW, randomized order, volts logged. Report
   the mean settled position per direction. Half the CW/CCW gap is the loop's one-sided
   directional hysteresis (lash + stiction) and bounds what any gain set can achieve. If
   that half-gap reaches the measured 2.65–3.01°, criterion 3 is mechanically floored:
   report it under "Done means" (2) with the number and stop tuning gains against it.

4. **kS sweep on tiles.** kS ∈ {0.035, 0.040, 0.045, 0.050, 0.055}. Fit err*(kS) and
   report the 95% CI on the slope. The two existing points extrapolate to **kS ≈ 0.048 at
   the 1.0° criterion** — that is a prediction, not a measurement; falsify it. The pending
   lubrication pass invalidates this fit, so state which side of it your numbers were
   taken on. Fix the off-ground failure (13/40 loose at 0.045) with a **clamped
   integrator, not a larger constant kS**: |∫| ≤ 0.015, leak to zero with τ ≈ 0.5 s, and
   **hold the integrator whenever the pod is rate-saturated** — the 90° step is 65%
   slew-limited, so anti-windup is mandatory.

5. **Only then touch kP/kD, and not on the 90° step.** 421 ms of that 647 ms settle is
   rate saturation, where output is the servo's max slew regardless of gains. Instead:
   (a) characterize the actuator open-loop — steering is a CRServo, a velocity actuator,
   so θ̇ ≈ g·(u − dz); sweep u ∈ {0.05, 0.10, 0.20, 0.40, 0.70, 1.0}, n = 5 per point,
   both directions, on blocks and on tiles, fit g and dz, and report the CW/CCW asymmetry
   (dz should land near the measured breakaway kS = 0.035 — say so if it does not);
   (b) add a real rate feedforward to `CoaxialPod`: `u = θ̇_ref/g + kP·err + kS·sign(err)`
   — this is the feedforward the ±kF relay is not, and it makes kF moot (**shipped**
   change); (c) fit kP against a **rate-limited reference**, ramped at 0.8 × measured slew
   = 171 °/s, so the loop stays out of saturation and kP is identifiable (this ramp alone
   cannot meet criterion 4 — 90° at 171 °/s = 526 ms — use it for identification and for
   criteria 5 and 6); (d) test **kD = 0 as an explicit arm**: on an integrator plant, P
   alone gives a first-order closed loop with nothing to damp, while D differentiates an
   encoder carrying ~120° of at-rest noise at a dt varying 8.9 → 53.6 ms (6×
   loop-to-loop). Randomized, interleaved, n ≥ 10.

Separately, 2× servo **overdrive** gives 540° of pod travel, passes `verifyCoverage`, and
attacks the 65% slew-limited settle (criterion 4) without the residual penalty that was
previously and wrongly claimed.
```

Also amend Task 2's "not applicable" paragraph (SWERVE_TASK.md:234–237) — it contains a false statement:

```
Note what is **not** applicable: `setPosition()` resolution is not in the CR path — skip
it unless the shelved positional variant (`swerve_positional_p0.xml`,
`POSITIONAL_SHELVED.md`) turns out to be live.

**Servo deadband is a different matter, and the earlier version of this prompt was wrong
to exclude it.** A CR servo has a pulse-width dead zone around neutral (1500 µs) that
applies to `setPower()` exactly as it does to `setPosition()`. kS = 0.035 is ~17 µs of a
500 µs half-range, and CLAUDE.md's "0.035 = measured breakaway" lumps that electrical dead
zone together with mechanical friction into one number. Do not re-measure breakaway;
measure its **per-direction split** (see criterion 3 step 3) — an asymmetric dead zone
produces a *signed* azimuth residual.
```

### 3. CLAUDE.md contains three facts that are wrong or unusable

**Problem:** an agent that trusts these will grep the wrong tree, chase a read cache that does not exist, and benchmark against a loop rate nobody can reproduce.

**(a)** Replace the `cache = 0.01` bullet, CLAUDE.md:158–159:

```
- **`cache = 0.01` is a servo *write* deadband — not a read cache, not a time.** In
  `CoaxialPod` it is `servoCachingThreshold` / `motorCachingThreshold`: `move()` calls
  `turnServo.setPower()` only when `|turnPower − lastTurnPower| > 0.01` (plus a forced
  write when turnPower hits 0). Dimensionless power units. It is **not** a pod
  encoder-read interval and **not** LynxModule bulk caching. Confirm against the vendored
  `third_party/PedroPathing` source; Task 0 item 2's premise is wrong and must be
  corrected in the same commit.
```

And fix SWERVE_TASK.md:111–113 to stop asserting the read-cache reading.

**(b)** Add to CLAUDE.md §3, right after the code tree (line 75):

```
**`CoaxialPod` is not in TeamCode.** It is `com.pedropathing.ftc.drivetrains.CoaxialPod`,
vendored at `third_party/PedroPathing/ftc/src/main/java/com/pedropathing/ftc/drivetrains/`
(Pedro gained native swerve in 2.1.0; `FollowerBuilder.swerveDrivetrain(SwerveConstants,
pods...)` wires it). Greps scoped to `TeamCode/` will miss the pod control loop, the
encoder→angle map, and the shortest-path flip. Read the vendored source, not upstream
docs — `turnKS`, `turnKSBandDeg`, `cache` and `PositionalPod.java` are not stock 2.1.2, so
this tree is forked or wrapped; say which.

**Third bucket for the shipped/diagnostic rule: `vendored` — `third_party/PedroPathing/**`.**
Editing it forks an upstream dependency and `includeBuild` rebuilds it into every module
with no version bump. Flag any such change and name the upstream 2.1.2 behaviour it
changes.
```

Mirror the `vendored` bucket into SWERVE_TASK.md Standing rules (line 82–84).

**(c)** Amend CLAUDE.md §6 (lines 165–166):

```
| Loop, DRIVE, after batteryVolts fix | **30.9 Hz true — DISPUTED, see note** (was 19.4–27.4) |
| Loop, DRIVE, bimodal | 29% @ 8.9 ms (publish skipped) / 71% @ 53.6 ms (publish runs) |

> **These two rows do not reconcile.** 0.29·8.9 + 0.71·53.6 = 40.6 ms = **24.6 Hz true**.
> 30.9 Hz would need a 52.5% slow fraction, not 71%. mean(1/dt) on the 29/71 split =
> 45.8 Hz, and 45.8/24.6 = 1.86× — inside the documented inflation band — so the split is
> self-consistent and 30.9 is the odd number out. Treat 30.9 Hz as unverified, and with it
> the "19.4–27.4 → 30.9" delta in §7.
```

And insert as Task 0 item 0 (before SWERVE_TASK.md:105):

```
0. **Reconcile the baseline before improving it.** From ONE SwerveBringUp DRIVE trace,
   report together: `1/mean(dt)`, the dt histogram with both mode locations and their
   sample fractions, and `mean(loopHz)` labelled inflated. The fraction-weighted mean dt
   must match `1/mean(dt)` within 5%. CLAUDE.md's triple fails this — it implies 24.6 Hz.
   Amend CLAUDE.md §6 and §7 in the same commit, and do not claim any DRIVE loop
   improvement against 30.9 Hz.
```

### 4. Task 2's hypothesis list misses two Pedro-side causes and will actively steer the agent away from the right family

**Problem:** hypothesis 0 ends with "if no edge flashed, cross this off and move to 1" — but `CustomDrivetrain.runDrive()` applies the identical per-axis clamp one layer down, unconditionally, on every deceleration. And X-lock is a stock Pedro default with no hysteresis that also fires mid-path in autonomous, not a hand-written `SwerveBringUp` quirk.

Insert as hypothesis 0b, after SWERVE_TASK.md:198:

```
0b. **The same defect one layer down, in Pedro itself.** `CustomDrivetrain.runDrive()`
   (`third_party/PedroPathing/core/src/main/java/com/pedropathing/drivetrain/CustomDrivetrain.java`)
   calls `clampReversePower()` **separately on the robot-frame X and Y components**,
   capping each to ±0.2 whenever that component opposes the measured velocity on the same
   axis, then passes the pair to `arcadeDrive(forward, strafe, rot)`. Clamping one axis
   and not the other **rotates the commanded translation vector**; the swerve
   `arcadeDrive` then recomputes `atan2(forward, strafe)` and pushes the rotated angle to
   all four pods. No enable flag, fires on every deceleration and every reversal.
   **Crossing off hypothesis 0 because no box edge flashed does not cross this off.**
   - **Applicability, one grep, do it first:** does the competition path reach
     `runDrive()` in teleop (`setTeleOpDrive` / `setTeleOpMovementVectors` →
     `follower.update()`), or go straight to `CoaxialPod`? If straight to `CoaxialPod`,
     this cannot cause the stick symptom — record it as a Task 3 finding and move on.
   - Falsifying column: log `translationalVector` X/Y **before and after** the clamp,
     `robotVelocity` X/Y, and the resulting `atan2`. Smooth pre-clamp angle with a stepped
     post-clamp angle confirms it.
   - Fix: clamp the scalar projection of the command onto the velocity direction and
     rebuild the vector at the original angle. **Vendored, shipped** change — label it a
     fork of upstream 2.1.2 and report the diff.
   - It is live during the Task 3 path run either way, so it can contaminate verification
     graphs 1 and 4.
```

Replace hypothesis 1 (SWERVE_TASK.md:200–204):

```
1. **X-lock — it is a stock Pedro default, not team code, and it fires in autonomous
   too.** `com.pedropathing.ftc.drivetrains.SwerveConstants` defines
   `enum ZeroPowerBehavior { X_LOCK, IGNORE_ANGLE_CHANGES }` (only those two — there is no
   "hold last angle") and `defaults()` sets `zeroPowerBehavior = X_LOCK`, `epsilon = 0.05`.
   `Swerve.arcadeDrive()` computes `zeroTrans = |rawTrans| < epsilon` and
   `zeroRotation = |rotation| < epsilon`; when both hold under `X_LOCK` it overwrites every
   pod vector with the pure rotation vector — the ±45° X pattern, i.e. exactly the symptom.
   **There is no hysteresis on `epsilon` anywhere in the library**, so a stick resting at
   the edge toggles all four pods between X and the stick target every loop.
   - First action is a read: report this robot's configured `zeroPowerBehavior` and
     `epsilon`. `SwerveBringUp.java:2532` may be a *separate* hand-written park — check
     both, and say whether the teleop path actually reaches `Swerve.arcadeDrive()` or goes
     straight to `CoaxialPod`.
   - Falsifier on `tgt`: under X-lock chatter `tgt` alternates between the stick angle and
     a fixed X angle on consecutive loops while stick magnitude sits within ~0.01 of
     `epsilon`. Log stick magnitude alongside `tgt`.
   - **This also runs in autonomous**: `Follower.update()` → `CustomDrivetrain.runDrive()`
     → `Swerve.arcadeDrive()`, so any low-power stretch of a Task 3 path (end-of-path creep
     especially) can snap all four pods mid-path. Expect it in verification graph 1; do not
     blame follower gains.
   - Fixes: (i) `zeroPowerBehavior(IGNORE_ANGLE_CHANGES)` — note it suppresses turn power
     rather than holding an angle; (ii) add magnitude hysteresis on `epsilon` (enter below,
     leave above ~1.5×). Both are **vendored** changes — report the diff.
```

### 5. Task 3 is unbuildable as written: Pedro 2.1.2 has no profiler, the API the agent will pattern-match was deleted, and the continuity spec manufactures the defect criterion 1 forbids

**Problem:** four separate traps. There is no jerk limit, acceleration limit, or motion profile anywhere in Pedro 2.1.2, and `PathConstraints.setVelocityConstraint()` — which reads exactly like a speed cap — is an *end-of-path completion tolerance* that will silently break the path's end condition while leaving speed uncapped. The `Point` class every web example uses was deleted in 2.0.0. `HeadingInterpolator`'s own javadoc example passes degrees to a radians API. `PathChain` does not check C0, let alone C1. And on a swerve, module azimuth rate is `θ̇ = κ·v`, so a C1-only joint is a step in commanded azimuth rate — a jerk limit bounds none of it.

Replace SWERVE_TASK.md:254–269 (the first four Task 3 bullets):

```
- **Pedro 2.1.2 API — the `Point` class was DELETED in 2.0.0.** Every Pedro example on the
  web is 1.x (`new BezierCurve(new Point(x, y, Point.CARTESIAN), ...)`) and **will not
  compile**. Do not search for examples. Pedro is vendored at `third_party/PedroPathing`
  via `includeBuild` — read that tree, do not hand-roll a spline library, and use these
  v2.1.2 signatures:
  - `com.pedropathing.geometry.Pose(double x, double y, double heading)` — inches,
    **heading in radians**.
  - `com.pedropathing.geometry.BezierCurve(List<Pose>)` — any degree; a cubic is 4 poses.
    Also `BezierLine(Pose, Pose)` and `BezierPoint(Pose)` for a hold point.
  - `com.pedropathing.paths.Path(Curve)`; `follower.pathBuilder().addPath(...).build()` →
    `com.pedropathing.paths.PathChain`, or `new PathChain(Path...)` directly.
  - `follower.followPath(chain[, maxPower][, holdEnd])`; `follower.holdPoint(Pose)`.
  - For the continuity checks below: `Curve.getPose(t)`, `getDerivative(t)`,
    `getSecondDerivative(t)`, `getEndTangent()`, `getCurvature(t)`, `length()`.
  If a symbol above does not resolve, the vendored tree diverges from upstream 2.1.2 —
  diff it and report that; do not go debugging the build.

- **C2 (curvature-continuous) at every joint — not C1.** Module azimuth rate is θ̇ = κ·v,
  so a curvature step is a step in commanded azimuth *rate*. Build by the reflection rule,
  but **the construction rule is not the evidence.** `PathChain(Path...)` only appends paths
  and sums lengths — it checks nothing, not even C0, so a chain with a 6 in gap builds and
  runs and the follower just yanks toward the new curve. Before the path runs, print a
  per-joint table: **C0** `|end(i) − start(i+1)|` in inches (above 0.01 in is a
  transcription bug); **tangent direction**, the angle between `getEndTangent()` of segment
  i and `getDerivative(0)` of segment i+1, in degrees — each `Path` carries its own t and t
  is not arc length, so compare directions, not magnitudes; **curvature**, `getCurvature(1)`
  vs `getCurvature(0)`, reported as a step in 1/in. State |Δκ| at every joint; it must be 0.
  Where geometry forces a corner, dwell (v → 0) through it. Report the measured number per
  joint — "C1 achieved" without the table does not count.

- **Speed/accel: Pedro 2.1.2 has no motion profile — no jerk limit, no acceleration limit,
  no time parameterization anywhere in `core/` or `ftc/`. Do not go looking for one.** It is
  a reactive vector follower. **`PathConstraints` is not a motion-constraint class.**
  `velocityConstraint` (default 0.1 in/s), `translationalConstraint`, `headingConstraint`,
  `tValueConstraint`, `timeoutConstraint` are all *end-of-path completion tolerances* —
  `setVelocityConstraint(30)` does **not** cap speed at 30 in/s, it ends the path as soon as
  speed drops below 30 in/s. The knobs that actually exist, and all you may use:
  `follower.setMaxPower(p)` / `followPath(chain, maxPower, holdEnd)`; `brakingStrength` /
  `brakingStart` / `usePredictiveBraking` (end-of-path deceleration only);
  **`FollowerConstants.centripetalScaling` (default 0.0005) — this plus curve geometry is
  your actual smoothness control**. `forwardZeroPowerAcceleration` /
  `lateralZeroPowerAcceleration` are *measured coast decelerations* feeding the braking
  model, not limits you set — do not present them as an acceleration limit.
  So bound acceleration **geometrically, not temporally**, and report peak lateral
  acceleration as `v²·κ_max` with `v` from `maxPower`, labelled **estimated**. Report a jerk
  number only from logged velocity differentiated twice with a stated filter — at 30–50 Hz
  with the known bimodal dt, an unfiltered double difference is noise. Otherwise write
  **"jerk not measured"**. Do not report a limit you never enforced.

- **Azimuth rate caps speed before accel or jerk does.** Solve v(s) subject to
  κ(s)·v(s) ≤ 171 °/s = 2.99 rad/s first (0.8 × the 214 °/s median slew; also below the
  184 °/s slowest measured pod), then accel, then bounded jerk. At R = 10 in (κ = 3.94 /m)
  that caps v at 0.76 m/s = 30 in/s. Uncapped, a ±10 in S-curve at 0.5 m/s demands a
  226 °/s azimuth-rate step — above the pod's slew ceiling, so the pods fall behind and
  cross-track error grows through the joint. State the binding constraint per arc-length
  station.

- **State how heading is interpolated per segment and name the method. Pedro takes RADIANS
  everywhere — write `Math.toRadians(...)`, never a bare degree literal.** Per-path on
  `PathBuilder`: `setTangentHeadingInterpolation()`, `setConstantHeadingInterpolation(h)`,
  `setLinearHeadingInterpolation(start, end[, endT[, startT]])`, `setReversed()`,
  `setHeadingInterpolation(HeadingInterpolator)`. Whole-chain: the same names with a
  `Global` prefix. In `com.pedropathing.paths.HeadingInterpolator`: `tangent`,
  `tangent.offset(rad)`, `constant(rad)`, `linear(startRad, endRad[, endT])`, and
  **`facingPoint(x, y)` / `facingPoint(Pose)`** — hold a target in view; usually better than
  `constant` on a scoring approach because it self-corrects. Add `facingPoint` as option (d)
  in the Survey 0 heading question. **Two traps, both in the vendored source:** (1) the class
  javadoc says "these methods all use radians" and its own example block directly below
  writes `constant(45)` labelled as degrees — that is 45 radians; do not copy a literal out
  of Pedro's docs. (2) `PathChain.getHeadingGoal` feeds a `setGlobal*` interpolator
  `chainT` = arc length travelled ÷ total chain length, while a per-path interpolator gets
  that path's raw **Bezier t**. These diverge on a curve — pick one family per chain and say
  which. Heading interpolation is a feasibility constraint, not a style choice: chassis
  rotation adds ω to θ̇, tangential gives ω = κ·v (continuous if the path is C2), constant or
  linear gives piecewise-constant ω that steps at every joint. If not tangential, smooth ω
  across the joint and report the resulting azimuth-rate step.
```

Replace the visualizer bullet (SWERVE_TASK.md:298–299):

```
- **The visualizer is a sanity picture, not the clearance check.** It draws an absolute
  field frame; the box lives in the pose frame with its origin wherever pose was last reset,
  so the visualizer cannot show box clearance at all. Before opening it, write an offline
  check that samples every `Path` in the `PathChain` at ≥200 points per segment and prints:
  min clearance to the envelope from consequence 1, in inches, with the `t` where it occurs;
  pass/fail against consequence 4's robot half-width; max |curvature| and the `t` where it
  occurs; and per-module **commanded azimuth rate** vs arc length with the 214 °/s slew
  ceiling and 171 °/s design limit drawn — any pod crossing 171 °/s means reshape the path,
  do not run it. Commit it as a test so it reruns on every control-point edit. **Then** open
  the visualizer to eyeball gross shape. Label the handoff "validated numerically against
  the envelope; untested on the robot" — that supersedes "validated in the visualizer"
  above.
```

### 6. CLAUDE.md tells the agent to stop; SWERVE_TASK.md tells it not to. CLAUDE.md is re-read on every compaction, so CLAUDE.md wins

**Problem:** §8 rules 5, 7, 9, 10 are halt instructions. Rule 9 in particular forbids the whole of Task 0 (instrument `publish()`, add timing to `DriveTeleOp`) and Task 1's recorder extension.

Replace CLAUDE.md §8 items 5, 7, 9, 10:

```
5. **You cannot touch the robot.** Deploying restarts the app, killing the OpMode and the
   HTTP server. Batch it: queue changes, post one OPS REQUEST (build green → operator
   installs → starts `Swerve Bring-Up` → replies "ready" + volts), and work an offline
   thread while you wait. Never one deploy per change; never idle waiting.

7. **Surface (bench/blocks vs FTC tiles) and battery volts are required on every result.**
   Take them from the OPS REQUEST reply. If you do not have them, record `surface=UNKNOWN` /
   `volts=UNKNOWN` and ask in the next batched survey — do not stop.

9. **Declare diagnostic-tooling changes that alter the measurement; do not ask.** Before
   taking data with it, log the change and its expected effect in `FINDINGS.md`, and keep
   the pre-change numbers there for comparison. Ask only if it would make an
   already-collected dataset unreadable.

10. **Present options as a survey item with a recommendation and an explicit default, then
    work against the default.** Never stop the run for the operator to choose an order.
```

Add to the §8 preamble:

```
> **Precedence:** during an autonomous run, the task prompt's run-control rules (batching,
> defaults, do-not-stop) override anything below that reads as "ask and wait". The evidence
> and safety rules (1–4, 6, 8, 11) are never overridden.
```

### 7. "BLOCKING" is undefined, so the agent can end the run by labelling its own question

Replace the second bullet under "Rules:" (SWERVE_TASK.md:42–45):

```
- Mark each item **BLOCKING** or **NON-BLOCKING**. An item is BLOCKING **only** if taking
  the default could damage hardware, drive the robot into a wall or a person, or destroy
  measured data you cannot recapture. Nothing else. Thresholds, statistical choices, which
  experiment to run first, shipped-vs-diagnostic scope, naming, and file layout are **always
  NON-BLOCKING** — take the default, log it in `ASSUMPTIONS.md`, keep working. If you cannot
  write a default for an item, the item is malformed: split it until you can. When a late
  answer contradicts a default you took, apply it going forward and re-run only the work
  whose result it changes.
```

### 8. "Batch your deploys" and "one change, one test" are irreconcilable — the fix is never stated

Replace the paragraph beginning "So do not ask for a deploy per change" (SWERVE_TASK.md:58):

```
**Deploys are the scarcest resource — make each one carry many experiments.** Ship every
experimental change behind a **runtime toggle** drained through the existing queued-command
path (`setPidf`, `setPublishHz`, `pidStep` prove it works), so one install yields A/B/C arms
with no reflash. Note `config.jsonc` is compiled in and read at OpMode init — editing it
still costs a deploy. A knob that can only be exercised by reflashing is a last resort; say
why.

**"One change, one test" binds at trial granularity, not deploy granularity.** One deploy
may carry ten toggles; each trial varies exactly one, and each trial record states the full
toggle vector. If a build unlocks only one experiment, you have not batched — find the
others first.

So do not ask for a deploy per change. Batch them, then post one **OPS REQUEST** block:
```

### 9. Task 0 reads as a gate on the repo's hardest unsolved problem

Replace Task 0's "Target before moving on" block (SWERVE_TASK.md:131–134):

```
**Target (not a gate):** `DriveTeleOp` at ≥50 Hz true, p90 dt < 25 ms. The servo PWM frame
is 20 ms ≈ 50 Hz, so past that nothing improves at the steering hardware. If it already
meets this, say so and move on — do not optimize the diagnostic tool for its own sake. **If
it misses, record `loop_hz_true`, mean/p90/min/max dt, and your best hypothesis, then go to
Task 1 anyway.** Loop rate is already refuted as the cause of the azimuth residual
(CLAUDE.md §7), so Tasks 1–3 are valid at 40 Hz with the rate recorded as a covariate.

**Budget:** the 37.4 ms `publish()` residual is a known-open problem — **one instrumented
attempt**. If you have not localized it, report the breakdown you did get, mark it `open`,
and move on. Task 0 gets at most **2 deploy cycles**.

**The loop-rate refutation does not cover this regime — read before skipping.** It ran 47.8
vs 92.1 Hz: both arms at or above the 20 ms / 50 Hz servo frame, so both got a fresh command
every frame. DRIVE runs below that, where a large fraction of frames get no new command and
a 53.6 ms dt holds one across 2.7 frames. So the ≥50 Hz target is a mechanism, not hygiene,
and the A/B compared *mean* rate, never jitter — **p90 dt < 25 ms is the real gate and the
mean is secondary.** Deliverable: state `DriveTeleOp` vs `SwerveBringUp` true rate against
the 50 Hz line. If bring-up is below it and `DriveTeleOp` above, every azimuth trace in the
archive was taken in a regime the shipped code never enters — say so. **Only in that case**,
a randomized interleaved **25 vs 50 Hz** A/B (n ≥ 20 pod-runs/arm, tiles, volts logged) is
**permitted and is not a re-run** — it tests the sub-frame regime CLAUDE.md lists as open
("a knee in loop rate between 20–48 Hz in DRIVE") and overrides "do not re-run those" for
loop rate only. If both OpModes already ship above 50 Hz, loop rate is closed: stop
instrumenting it.
```

Add to Standing rules:

```
- **Breadth before depth.** Attempt every criterion at least once before iterating a second
  time on any one, and cap first-pass on-robot iterations at **3 per criterion**. When you
  hit the cap, write the number you hit, the cause, and the next experiment, and move on.
  Note iteration counts in `FINDINGS.md`.
```

### 10. Every per-sample statistic has the `mean(loopHz)` defect, and "steady state" is never defined

**Problem:** the recorder samples once per loop and the loop is bimodal, so fast loops are 29% of samples but 6.4% of elapsed time — no unweighted per-sample statistic is a time-average. Separately, criterion 3's statistic, window, and aggregation are undefined, so the same trace supports any answer between 0.1° and 5°.

Add to Standing rules, right after the loop-rate line (SWERVE_TASK.md:80):

```
- **The recorder writes one sample per loop, and the loop is bimodal — samples are not
  evenly spaced in time, so no unweighted per-sample statistic is a time-average.** At the
  measured 29%/71% split of 8.9/53.6 ms, fast loops are 29% of samples but 6.4% of elapsed
  time: a 4.6× over-weight. `mean(loopHz)` is one case of this, not the whole of it.
  Time-averages: `sum(x*dt)/sum(dt)`, not `mean(x)`. Quantiles: sort by x, accumulate dt,
  take the value at the target fraction of total dt — not `numpy.percentile` on the raw
  sample array. Applies to mean and p95 |azimuth err| and heading error (criteria 3, 7, 8)
  and every histogram in Task 2 and Verification. It does **not** apply to criterion 5 (a
  ratio of integrals) or criteria 2 and 6 — but those must be pooled
  `total_count / total_elapsed_s`, never a mean of per-chunk rates.
- **Loop samples within a run are autocorrelated — 3000 samples are not n = 3000.** For any
  interval or test over a within-run series, report lag-1 `rho` and
  `n_eff = n(1−rho)/(1+rho)`, and get the interval from a moving-block bootstrap with blocks
  ≥ 1.1 s (3× the 0.37 s rise), never the iid formula. At rho = 0.9 a 3000-sample chunk
  carries n_eff ≈ 158 and the naive SE is 4.4× too small. No p-value on a within-run series
  unless it came from that bootstrap. For the 45° clustering check, report the **resultant
  length of 8× azimuth** (0 = uniform, 1 = fully locked to 45° multiples) with a
  block-bootstrap interval, not a uniformity p-value.
- **A robot-level treatment is replicated at the RUN, not the pod.** Loop rate, gains, build
  and publish path apply to all four pods at once, and pods within a run share battery, Lynx
  bus, timing and surface. Average the four pods to one value per run and do the A/B on
  n = runs. Write "n = 10 runs (40 pod-observations)", never "n = 40 pod-runs". Report per
  pod as well as pooled — a pooled mean hides one bad pod inside three good ones, which is
  the likeliest shape of the criterion-3 residual. The loop-rate refutation was analysed at
  n = 40 pod-runs, so its CI [−0.51, +0.71] is up to 2× too narrow; run-level analysis does
  not move the point estimate (+0.12°), so it **stays refuted, do not re-run it** — correct
  the CI in CLAUDE.md §7 if you can recompute from `trials.jsonl`.
```

Insert directly above the criteria table (SWERVE_TASK.md:330):

```
**Fix these measurement definitions before taking data. Do not change them after seeing a
trace.**

- **t0** = the loop where the commanded azimuth setpoint steps by > 45°. Hold each step
  ≥ 2.5 s.
- **Criterion 4 settle time** = first time after t0 that |err| stays inside ±2.0°
  continuously for 250 ms.
- **Criterion 3 window** = [t0 + 1.5 s, t0 + 2.5 s]. That is > 2× the measured 0.647 s
  settle, so it is past the transient whether or not the step settled.
- **Criterion 3 statistic** = **dt-weighted median |err|** over that window. Report the
  dt-weighted **signed mean** (bias) and **peak-to-peak** (limit-cycle amplitude) alongside
  it, always: 0.4° bias with 5° p-p and 2.8° bias with 0.2° p-p are different failures and
  must not collapse to one number.
- **Aggregation**: per pod, ≥ 10 steps, median and IQR across steps. Do not pool the four
  pods into one number.
- The **2.65–3.01° baseline was computed by an unknown statistic**. Re-derive it under this
  definition before claiming any improvement against it.
- Criterion 4's ±2° band is deliberately looser than criterion 3's 1.0°: 4 measures
  transient speed, 3 measures the floor it settles onto. Both must hold.
```

Amend criterion 1 and criterion 3 rows:

```
| 1 | Unintended azimuth setpoint **rate** `\|Δtgt\|/dt` between consecutive loops | **≤ 170 °/s** (0.8 × measured 214 °/s pod slew) — report max **and p99** — and zero clustering at 45° multiples. Deliberate shortest-path flips excluded but **counted and reported** | 45° snapping observed |
| 3 | Per-module azimuth steady-state error — dt-weighted median \|err\| over [t0+1.5 s, t0+2.5 s], median across ≥10 steps, per pod | **< 1.0°**, with signed bias and p-p reported alongside | 2.65–3.01° (statistic **unknown** — re-derive) |
```

with this note under the table:

```
Criterion 1 is a rate because Task 0 changes dt. The old "≤ 15° per loop" is 463 °/s at
30.9 Hz — 2.2× the pod slew, so a setpoint at the threshold is untrackable yet passes — and
750 °/s at 50 Hz, i.e. looser precisely when the loop gets better. 170 °/s is 5.5° per loop
at 30.9 Hz, 3.4° at 50 Hz.
```

### 11. Criterion 5 is a total-variation estimator, so Task 0's own loop-rate fix inflates it with zero change in robot behaviour

Add as a note on criterion 5 and extend Verification item 6:

```
**Criterion 5 is Σ|Δθ| — total variation — so its noise term grows linearly with sample
count while its signal term does not. Task 0's publish() fix raises the DRIVE loop rate the
recorder samples at, which inflates criterion 5 with zero change in robot behaviour. Do not
read a post-Task-0 rise as a regression.** Compute it as:

- Clip each sample delta at `259 °/s × dt_i` before summing. That is the measured pod slew
  max; anything above it is provably noise, not motion. Physical bound, not tuned.
- Pool, never average ratios: `Σ wheel_TV / Σ commanded_TV` per pod across all chunks.
  Exclude any interval below the magnitude gate (Task 2 hypothesis 5) — a stopped robot
  contributes a zero denominator.
- Measure the floor in the same session: robot commanded to hold still, same surface, 60 s.
  Report criterion 5 both raw and minus floor. A criterion-5 number without its same-session
  floor is not admissible.
- Any criterion 5 or 6 comparison across a loop-rate change must be at matched sample rate —
  decimate the faster trace to the slower one. Say that you did.
- Criterion 6 reads 0.00 reversals/s at rest against ~120° of encoder noise, so it already
  deadbands and criterion 5 does not. State criterion 6's existing deadband numerically and
  apply the same one to criterion 5.
```

And in CLAUDE.md §6, replace the at-rest row (line 176):

```
| At-rest baseline | ~120° — **summed |Δθ| path over a chunk, NOT an amplitude** | 0.00 reversals/s |
```

with this below the table:

```
**Metric definitions.** `wheel_path` = Σ|Δθ| over the chunk, in degrees. 120° as a
peak-to-peak amplitude is impossible (1.1 V of noise on a 3.3 V ratiometric 1:1 encoder) and
contradicts 0.00 reversals/s. It is either a path sum (≈1.2–4 °/s over a 30–97 s chunk) or a
0/360 wrap-seam artifact: a pod parked on the seam adds ~360° per sample. Report
`wheel_p2p_deg`, `wheel_rms_deg`, `wheel_path_deg_per_s`, chunk duration, and the fraction of
at-rest samples within 5° of the seam — **before** trusting criterion 5's ratio, which
carries this same numerator and is the likely source of its 1.7–3.0× spread. A `reversal` is
undefined without a hysteresis threshold; state it in degrees next to every reversals/s
figure or criterion 6 is arbitrary. **ADC quantization is ruled out by arithmetic — do not
experiment on it.** 12-bit over 3.3 V = 0.806 mV/LSB; at 360°/3.3 V = 109.1 °/V that is
0.088 °/LSB (≤0.14° through a 5 V divider) — 20–30× below the 2.7° residual. This supersedes
Task 2 hypothesis 4's "sensor noise or a conversion artifact" framing for the 120° figure;
the rest of that hypothesis still stands.
```

### 12. Ops: no abort word, no pre-flight, no speed cap, and "box armed" is not "box correct"

Add as a new section before Task 0:

```
## ABORT

"ABORT" from you means: I stop sending to `/command` and the bench client immediately, post
the last command I sent and when I sent it, and send nothing motion-capable until you reply
"re-arm". A survey answer, a "ready", or silence is not a re-arm. Any recorder chunk spanning
an abort is discarded, not analysed — say how many.

Your fast stop is **STOP on the Driver Station**: it kills the OpMode, so I must then assume
the HTTP server is gone too. The dashboard's 400 ms watchdog only cuts the browser drive path
— backgrounding the tab is not an abort.

I stop on my own and ask you to hit STOP if a pod's measured azimuth stops responding while
I am commanding it to move (a CRServo stalled against a hard stop), or if `/state` fails
twice in a row while a drive command is outstanding.

## Pre-flight — first OPS REQUEST of the session, before any command that can move the robot

Starred items repeat after every deploy or battery swap. One reply, not drip-fed.

1. 3 ft clear around the 51 × 46 in box. Tape the box on the floor plus a second line 6 in
   inside it. The tape is the barrier — you are about to test the software clamp for a
   direction-rotating bug (Task 2, hyp 0), so do not rely on it to keep the robot in.
2. Battery strapped in, leads clear of the wheels, resting volts reported (≥ 12.5 V to start;
   the baseline runs were 12.2–12.5 V).
3. *Turn each pod by hand through its full 270° travel. Report any bind, grind, play, or
   debris in a wheel — a binding pod invalidates every kS/friction result before you measure
   it.
4. *Encoder liveness: OpMode running, nothing commanded, operator turns each pod by hand
   while you watch `/state`. All four azimuth angles must move. One that holds still is a
   dead encoder: the CR loop sees a constant error and holds full command on that servo
   indefinitely, and the 5 Hz four-servo rail total will not show it. STOP — do not command
   steering.
5. Sticks read ~0 at rest. Drift here fakes the deadband-0.06 chatter in Task 2 hyp 1.
6. Driver Station within reach, STOP visible.

## "Armed" is not "correct" — the box can drift

The box is stored in the Pinpoint's dead-reckoned pose frame, so odometry drift slides and
rotates it relative to the floor while `/state` still reports armed. 3° of yaw error moves the
far corner of a 51 × 46 in box (68.7 in diagonal) by 3.6 in. Drift over a run is **unmeasured**
— Task 1 measures it. So "confirm the box is armed" is necessary and not sufficient. In the
same OPS REQUEST that re-marks corners A and B, add: put the robot back on the corner-A
position, square to the box — I read x, y from `/state`; repeat at corner B. PASS if both land
within **2 in** *(guess — Survey 0)* of the marked coordinates and the implied box is
51 × 46 ± 2 in. On fail: reset pose, re-mark, redo — I do not command motion on a box that
failed. Log each pair in `FINDINGS.md` as the drift measurement Task 1 wants.

**Re-mark procedure (this is the order, and the order matters):** (1) place the robot on the
corner-A mark, square to the box, +x along the 51 in side, using the FIELD panel's footprint
indicator centre as the anchor point at both corners; (2) press **Reset pose** — this must come
BEFORE marking, because it clears the box, so pressing it after Mark B destroys the box you
just made; (3) **Mark corner A**; (4) drive to the diagonally opposite mark (+51 in x, +46 in y)
holding the same heading, press **Mark corner B**; (5) reply with the x, y the FIELD panel reads
at corner B.

## Speed cap

Baseline is p95 driving azimuth error ~42° with 97 flips in one chunk — pods fighting at speed
scrub and lurch, and a 180° flip reverses a loaded wheel.

- Cap every diagnostic drive in Tasks 0–2 at 30% of full commanded translation and rotation
  until criteria 1, 2, 5 and 6 hold. Convert it to in/s off the FIELD panel `v` readout, record
  it with every result, Survey 0 default 30%.
- Task 2 hypothesis 0's edge-proximity check is a deliberate drive into the hard limit, and the
  hypothesis itself predicts it will chatter. Run it at 15%, one axis at a time, maximum 3 clamp
  engagements per run, then stop and disarm. Say in the OPS REQUEST that this run drives into
  the clamp on purpose.
- The clamp acts on commanded velocity, not position, so it does not kill momentum — the robot
  coasts past the edge. Measure that overshoot once from the pose trail at the capped speed. If
  it exceeds the Task 3 item 4 half-width margin, lower the cap until it does not.

## On blocks is not the safe state

Rule 8 covers driving off the floor; it does not cover on-blocks work, which is where the pod
step tests and the kS comparison happen. Every OPS REQUEST that puts the robot on blocks gets
these as numbered steps: clamp or strap the chassis to the blocks (`pidStepAll` slews four pods
at once at 214 °/s; the reaction can walk an unsecured chassis off them); hands, cables and the
gamepad lead clear of all four wheels before you reply "ready".
```

### 13. No resumption state — a compaction or a battery swap loses the run

Add after "Robot operations — also batched":

```
### `RUN_STATE.md` — so an interruption does not cost robot time

Keep `RUN_STATE.md` at repo root. Rewrite it in full after each task's result block, before
each OPS REQUEST, and after each operator reply. A compaction or a new session loses
everything else.

```
updated:          <ISO timestamp>
phase:            Task 0|1|2|3 | Verification | wrapping up
last_commit:      <sha>  (build green? y/n)
on_robot_build:   <sha or UNKNOWN>   # what the operator last installed
opmode_running:   SwerveBringUp | DriveTeleOp | none | UNKNOWN
surface:          tiles | blocks | UNKNOWN
battery_v:        <v> @ <time>
box_armed:        yes | no | UNKNOWN
open_ops_request: <n + its numbered steps, or none>
open_surveys:     <ids awaiting answer, or none>
criteria:         1..11, each: met <value> | missed <value> | unmeasured
next_3_actions:   1. 2. 3.
```

**On resume (new session or after compaction): read `CLAUDE.md`, `RUN_STATE.md`,
`ASSUMPTIONS.md`, then the tail of `FINDINGS.md`. `RUN_STATE.md` is a hypothesis, not a fact —
before any command that moves the robot, re-read `/state` and refresh `opmode_running` and
`box_armed`; before attributing any measurement to a code change, confirm
`on_robot_build == last_commit`.**
```

Add to CLAUDE.md as §10:

```
> **§10 Autonomous runs.** If `RUN_STATE.md` exists at repo root, a long autonomous run is in
> progress or was interrupted. Read it before anything else and keep it current per its schema.
```

---

## Worthwhile patches

### 14. Verification assumes the path ran; Task 3 explicitly allows it not to

Insert at the top of Verification, before item 1:

```
**If the path never runs on the robot** (Tasks 0–2 did not converge, or you ran out of deploy
cycles), verification is still required. Produce graphs **1, 2, 3, 5 and 6** from your best
free-driving `drivecapture.py` capture; mark graph **4 `N/A — no path run`** and criterion 9
unmeasured. Put the capture label and `freedrive` vs `pathrun` in every graph title and
filename. Never present a free-driving capture as a path run.
```

Replace verification item 3:

```
3. Chassis velocity and acceleration vs. time.
   - **Velocity is a Pinpoint channel, not a pose difference.** The Pinpoint reports it
     directly (`PinpointLocalizer.getVelocity()` → `odo.getVelX/getVelY/getHeadingVelocity`).
     Add `vx`, `vy`, `omega` recorder columns and say you did; do not difference pose.
   - Acceleration is a host-side first difference of that velocity. State the filter, or state
     "unfiltered".
   - **Jerk:** a second difference at 30.9 Hz true with dt bimodal at 8.9/53.6 ms is noise.
     Report it only with a stated filter and stated effective bandwidth, both on the plot
     label — otherwise write "jerk not measured" and move on.
```

Add verification item 7:

```
7. Commanded azimuth rate vs. arc length, all four pods on one plot, with the 214 °/s slew
   ceiling and 171 °/s design limit drawn. Offline — produce it before any robot time.
```

### 15. Human sticks are the stimulus for criteria 1, 2, 5, 6, 7 — before and after are different experiments

Add to Task 2 and Verification:

```
**Standardized stimulus (criteria 1, 2, 5, 6, 7 and all of Verification).** Human stick input
is not repeatable and may not supply any before/after number. Add to the bring-up OpMode a
**diagnostic** scripted-command mode replaying a fixed (vx, vy, omega) sequence: short traverse
at three speeds, pure rotation, translation + rotation, stick-through-centre reversal, and one
slow approach to a single box edge for Task 2 hypothesis 0 — that is the only segment allowed
to reach the clamp; every other segment stays a robot half-width clear of every edge. Size it
to the 51 × 46 in box: rotation in place and fast direction sweeps, not long straight legs.
Under 25 s total so one run fits the 3000-sample recorder without chunking, then auto-disarm.
Log the profile sample index in the recorder so runs align, and report each criterion **per
segment, not pooled**. Do not duplicate `pidStepAll` — 90° azimuth steps already run scripted.

This replaces the bring-up-side hand-driven runs, not the `DriveTeleOp` gamepad runs — Task 2
hypothesis 2 still needs a human on that path to separate a dashboard artifact from a robot
bug. The 1.7–3.0× / 2.58–4.18 /s baseline is human-driven and cannot serve as the "before" —
capture a pre-fix baseline with the same profile in the same OPS REQUEST batch. Put "build it
or hand-drive everything" in Survey 0, default build.
```

### 16. Criteria 5 and 6 are the wrong observables — add three that work

Add to Standing rules:

```
- **Criteria 5 and 6 are provisional — report them, but not alone.** Report these three
  alongside, thresholds routed through Survey 0 as guesses:
  1. **Excess azimuth rate, °/s per pod:** `mean(|θ̇_wheel|) − mean(|θ̇_tgt|)`, minus the
     measured at-rest floor. Additive, no denominator, comparable across chunks. *(guess:
     < 15 °/s — Survey 0)*
  2. **Azimuth-error spectrum per pod.** Resample to a uniform grid first — `dt` is bimodal
     and an FFT on raw samples is invalid. Report peak frequency and amplitude from 2 Hz to
     Nyquist, and state Nyquist rather than reporting a band you cannot see. A sharp peak =
     relay limit cycle from the ±kS band (check against the 20 ms/50 Hz PWM frame and
     loop_rate/2 for aliasing); a broad low-frequency lump = stick-slip, which the pending
     lubrication pass should move.
  3. **Command-side chatter:** mean |Δu| per second and the fraction of loops where u changes
     sign. Computed on the command, so it carries no encoder noise; also the actuator-wear and
     rail-current proxy.
```

### 17. The safe-area clamp can contaminate every on-floor trial and there is no way to detect it after the fact

Add to Standing rules:

```
- **The safe-area clamp is inside the steering control path — instrument it before the first
  on-floor trial, not at Task 3.** Add per-loop `clamped` (bool), `clampAxis`
  (`none`/`x`/`y`/`both`) and `edgeDistIn` to the recorder — **diagnostic**, `SwerveBringUp` +
  `PodRecorder`. `clampAxis` is the log-side discriminator for Task 2 hypothesis 0: `x` or `y`
  alone snaps the commanded direction toward ±90°, `both` toward ±45°.
- **Reject clamped samples, do not average them.** Exclude every `clamped = true` sample from
  all azimuth, path-ratio and reversals figures, and report the excluded fraction alongside
  surface / volts / n / spread. Above 5% the run is contaminated — re-drive further from the
  walls rather than reporting it.
- **Step tests and A/B trials: park ≥ (robot half-width + 6 in) from every edge** and confirm
  `clamped` stayed false for the whole trial. Never disarm the box.
```

### 18. Analysis plan pre-registration — the flip classifier in particular

Add as a new section before Task 0:

```
## Analysis plan — commit this before the first robot measurement

Commit `ANALYSIS_PLAN.md` at repo root before any robot data is taken. After the first capture
it is append-only: later changes go in as dated `DEVIATION:` entries saying what changed and why.

1. **One line per criterion, all eleven:** the exact statistic, the time window, any gate, the
   aggregation unit, and the planned n. If you cannot write one down before you have the data,
   it is not measurable yet — say so in Survey 0.
2. **Flip classifier — mechanical, fixed in advance.** Add a logged boolean `flipCmd`, set on
   the loop where the optimizer takes its shortest-path branch (diagnostic, `PodRecorder`).
   Deliberate = `flipCmd` true on that loop; every 180° azimuth transition with `flipCmd` false
   is unintended. **Do not classify flips by inspecting the trace afterwards.** If you cannot
   add the column, report criteria 1 and 2 as "not separable" rather than estimating the split.
3. **Run exclusion rule, stated up front** — e.g. start battery below your floor, the 400 ms
   gamepad watchdog cutting, or the safe-area clamp engaging inside the window. Report how many
   runs were excluded and under which clause, every time. No post-hoc exclusions.
4. **Freeze before the confirming capture.** Tune on tuning runs. When you stop tuning, freeze
   and commit the code and the analysis script, then take the final capture. **Fill the criteria
   table from that frozen run only**; report tuning-run numbers separately, labelled "tuning,
   not confirmatory". If you never reach a freeze, say so and label the whole table as tuning
   data.
```

### 19. Provenance tags and a lint, because the exhortations have already failed here three times

Replace the second and third Standing rules bullets:

```
- **Every result gets surface, battery volts, n, spread — and a provenance tag.** In
  `FINDINGS.md`, every line containing a unit-bearing number (`Hz ms ° in V × /s %`) ends with
  exactly one of: `[M <file> n=<n> surface=<tiles|blocks> v=<volts>]` measured this session,
  `[B]` quoted from `CLAUDE.md`, `[EST <basis>]`, `[UNK]`. First offline work in Task 0: write
  `tools/swervetune/lint_findings.py` — exit non-zero on an untagged unit-bearing number, and on
  `mean(loopHz)`, `meanLoopHz`, or `loopHz.mean()` appearing on a line without the word
  "inflated". Run it before every commit and say in the message that it passed.
- **Loop rate is always `1/mean(dt)`, and comes from one function.** `loop_stats(dt_series)` in
  `tools/swervetune/stats.py` returns `loop_hz_true`, `loop_dt_mean_ms`, `loop_dt_p90_ms`,
  `dt_min_ms`, `dt_max_ms`, `n`. Every loop-rate figure in every report and graph label comes
  from it. Never compute a rate from the `loopHz` column.
- **A/B arms are randomized, interleaved, and recorded.** Every trial row carries `arm`,
  `trial_index`, `rng_seed`, `volts`; report the realized arm sequence. No recorded sequence
  means it is not an A/B result — label it `[EST]`.
```

### 20. Context budget — the three largest files in the repo will otherwise be read whole

Add to Standing rules:

```
- **Never read `SwerveBringUp.java` (~193 KB), `dashboard.html` (~72 KB), or `Tuning.java`
  (68 KB) whole** — `Grep` for line numbers, then `Read` with `offset`/`limit` in windows of
  ~120 lines. Same for any file over ~1500 lines. Read a script's printed summary, never a raw
  capture file, into context.
- **Deliverables beat experiments.** At ~70% context used, stop starting new experiments: finish
  the one in flight, then write `FINDINGS.md`, `ASSUMPTIONS.md`, the six graphs, and the refilled
  criteria table, and commit. At ~85%, write the final report against whichever "Done means"
  clause is true and stop. A session that ends with good measurements and no written deliverables
  is a failed session.
```

### 21. Task 1: name the swerve-specific geometry cause and the free Pinpoint facts

Replace the "Does error scale with translation speed?" bullet:

```
- **Does error scale with translation speed?** This is the tuning-vs-geometry discriminator. On
  a coaxial swerve the leading geometric cause is **mismatched per-pod azimuth offsets in
  `PodCal`**: pods commanded to θ but sitting at θ + δᵢ that do not cancel give a net yaw rate
  **linear in wheel speed**, so heading error grows with translation and not with rotation.
  - **Test:** heading hold **OFF**, drive straight at 10, 20, 30 in/s, n = 3 each, randomized,
    tiles, volts logged, in the clear middle of the box — at 30 in/s a 51 × 46 in box is ~1 s of
    travel, and an engaging wall clamp rotates the velocity vector (Task 2 hyp 0) and corrupts
    the measurement. Fit open-loop Pinpoint yaw rate vs. speed. **Nonzero slope ⇒ azimuth offset
    mismatch. Speed-independent yaw offset ⇒ Pinpoint bias or wheelbase error. Neither ⇒ gains.**
  - If the slope is nonzero, recover the δᵢ rather than tuning around them: least squares on yaw
    rate over ~8 commanded directions at one speed, then write the correction via
    `saveCalibration`. Say which side of the shipped/diagnostic line that write lands on.
  - Then compare the δᵢ against the **signed** per-pod azimuth residual behind criterion 3. If
    the signs and magnitudes agree, criteria 3 and 7 have one cause; report it as one finding.
```

Replace the Pinpoint bullet:

```
- **Heading comes from the goBILDA Pinpoint, not a Control Hub IMU.** Do not go looking for
  `getRobotYawPitchRollAngles()` unless you actually find an IMU in the path. Read cost is
  settled — `msHeading` 1.81 ms is one 40-byte I2C `BULK_READ` inside `odo.update()`, and I2C is
  **not** covered by LynxModule bulk caching, so it is irreducible. Do not try to remove it.
- **Every Pinpoint getter returns a value cached at the last `update()`**; Pedro's
  `PinpointLocalizer` calls `update()` once. Do not design an experiment to discover this. The
  only check is in code: `update()` runs exactly once per loop and nothing reads heading before it.
- **Log `getDeviceStatus()`, `getLoopTime()` (µs) and `getFrequency()` (Hz) as trace columns.** On
  a bad I2C read the driver keeps the previous values and reports `FAULT_BAD_READ` — that reads in
  a graph as a frozen or suspiciously perfect heading and will be misdiagnosed as a control result.
  **Any heading conclusion from a run containing a non-READY sample is void.** Healthy device side
  is 500–1100 µs / 900–2000 Hz.
- Yaw drift over the run is genuinely empirical — measure that one.
```

### 22. Free Task 0 finding: uncached `getVoltage()` inside Pedro's per-pod loop

Add as Task 0 item 3b:

```
3b. **Pedro's `Swerve.arcadeDrive()` has an uncached `getVoltage()` inside the per-pod loop.**
   `LynxVoltageSensor.getVoltage()` issues `LynxGetADCCommand(BATTERY_MONITOR, ENGINEERING)` via a
   bare `sendReceive()` with no bulk-cache branch — unlike `LynxAnalogInputController`, which has
   one. `Swerve.arcadeDrive()` calls `getVoltageNormalized()` → `getVoltage()` inside the per-pod
   loop when the flag is on — **four hub round-trips per call** — and `CustomDrivetrain.runDrive()`
   tail-calls `arcadeDrive()`, so the Follower hits it every update. Grep `useVoltageCompensation`
   (library default `false`) and report the configured value; if `true`, hoist the call out of the
   loop and re-measure as a one-change-one-test. This is the `DriveTeleOp`/follower path, **not**
   `publish()` — do not expect it to explain the 37.4 ms.
   Report upstream regardless: in that loop `podVector.times(voltageNormalized);` **discards its
   return** — `Vector.times(double)` returns a new Vector — so voltage compensation costs four ADC
   round-trips and does nothing. The flag is inert; do not enable it to explain a voltage-dependent
   result.
```

### 23. Criterion 9 is measured with the same localizer that closes the loop

Add to Verification, before item 4:

```
**Localizer ground truth — do this before reporting any cross-track number.** Cross-track error
from the Pinpoint pose measures the follower tracking its own estimate, not physical path error: a
wrong wheelbase constant or pod scale gives a badly wrong physical path with an excellent
cross-track number. Bound the localizer independently, batched into an OPS REQUEST you already
need:
- **Closure:** tape two floor reference points, drive a closed path back to start, operator
  measures physical `dx`, `dy` (in) and heading offset (deg). n ≥ 3; report median and range as
  `localizer_closure_error`.
- **Scale:** command a 36 in straight traverse, operator measures actual distance. n ≥ 3; report
  the ratio. An out-and-back cancels scale error, so closure alone will not catch this.
```

Amend criterion 9:

```
| 9 | Cross-track error (Pinpoint pose), reported jointly with `localizer_closure_error` and the scale ratio | **< 2.0 in** *(guess — Survey 0)*, **admissible only if closure error < 2.0 in** — otherwise report criterion 9 as not measurable, not as a pass | unmeasured |
```

### 24. Same-session baseline, battery floor, and the watchdog

Add to Standing rules:

```
- **Baseline the criteria in this session; 2026-08-13 is not a control arm.** The `Current` column
  is 08-13 data, and a mechanical lubrication pass has been pending since then — criteria 1–6 can
  move with no code change. Before your first tuning or code change, capture criteria 1–6 on
  today's battery and surface (the Task 1/2 characterization capture serves — label it the
  control), and report `Current` from that with the 08-13 figure beside it. Every improvement claim
  is against the same-session number.
- Keep baseline and post-change captures on the **same battery charge**; re-baseline after a
  battery swap or the lubrication pass. For runtime-settable gains (`setPidf`, no deploy) do A-B-A.
  Where a change needs a reflash, label the result **uncontrolled before/after**. Before crediting
  any change, plot the criterion against the recorder's `volts` column across the session.
- **Every voltage says resting or loaded.** Floor: **11.5 V loaded** — finish the chunk, disarm,
  post a battery-swap OPS REQUEST. Do not start a new chunk below 11.8 V loaded, or a campaign
  below 12.5 V resting. Sub-floor data is not evidence about gains — discard it, do not tune on it.
  **Never straddle a battery swap inside one A/B arm**; treat battery as a block and report
  per-battery means alongside the pooled one.
- **Browser drive cuts on tab focus loss** (`dashboard.html:556-628`, 60 ms poll, **400 ms
  watchdog**). Every OPS REQUEST for a dashboard drive says "keep the tab focused and in front".
  If drive stops with no command from me, ask whether the tab had focus before you call it a robot
  fault. Discard — do not analyse — any chunk in which it fired: a watchdog cut is zero input, and
  zero input trips X-lock into a ±45° park, so it manufactures the Task 2 symptom and corrupts
  criteria 5 and 6. Report the discard count.
```

### 25. Hardware-gated criteria, and a rule for when your data contradicts CLAUDE.md

Replace the closing paragraph of Success criteria:

```
**Bounded iteration.** At most 3 on-robot iterations per criterion. When a criterion's remaining
candidates all need physical work you cannot do, it is **hardware-gated**: name the intervention
and stop spending robot time on it. Two candidates above are hardware-gated — the pending
lubrication pass (criterion 3's kinetic-friction candidate; its kD and kI-band candidates are
still yours to test) and 2× servo overdrive (criterion 4). Ask both in Survey 0:

- `S0.Qx` Can you do the pending lubrication pass this session? a) now b) later this session
  c) no. DEFAULT: (c) — I treat the friction candidate for criterion 3 as hardware-gated and spend
  my iterations on kD/kI. NON-BLOCKING.
- `S0.Qy` Can you install 2× servo overdrive gearing this session? a) yes b) before the next
  session c) no. DEFAULT: (c) — criterion 4 is reported as slew-limited with the measured slew
  rate. NON-BLOCKING.

**If a criterion cannot be met, say so with the number you actually hit and why. Do not quietly
lower the bar and do not report an estimate as a measurement.**
```

Add after the opening "Read CLAUDE.md first" paragraph:

```
**When your data contradicts `CLAUDE.md`, which wins depends on the kind of fact:**

- **Source facts** (gains, file paths, port names, what a function does): the code wins. Fix the
  file in the same commit and say in the message what was wrong.
- **Measured facts** (§6 baseline, §7 refuted hypotheses): overturning one takes evidence at least
  as strong as the result it replaces — same or greater n, randomized interleaved arms, CI
  reported. Meet that bar and you edit the row, keeping the old value struck through with its date
  and your run file. Miss it and you **do not touch the row**: append a `CONTESTED:` line under §7
  with your observation, your n, and what a properly powered check would take, then keep working
  and raise it in the next survey. **Never delete a REFUTED row** — a refutation is data.
- If a `CLAUDE.md` fact you were relying on turns out wrong mid-experiment, mark the results taken
  under it suspect in `FINDINGS.md` rather than reporting them straight.
```

---

## Rejected / already covered

- **Chassis pod-to-pod coupling as a criterion-3 cause** — the mechanism cannot operate in the single-pod `pidStep` experiment that produced the 2.65–3.01° figure, and the three-arm test would need ~30 re-blockings by the operator.
- **Encoder magnet eccentricity / PodCal offset as the residual** — the loop closes on the same encoder, so a measurement offset cancels out of `tgt − wheel`.
- **FFT hunting-frequency per step** — a 1.0 s window at ~30 Hz with bimodal dt gives ~30 uneven samples; peak-to-peak already captures limit-cycle amplitude.
- **Demoting eight of eleven criteria to "descriptive"** — contradicts the user's explicit "all eleven hold"; most are deterministic acceptance thresholds, not noisy tests.
- **Power calculation before the run** — already covered by CLAUDE.md rule 2, and the existing A/B implies MDE ≈ 0.9° against a 2.0° gap, so it would print "yes, detectable".
- **Unique survey IDs (`S0.Q1`)** — items are already numbered and partial answers already have a rule.
- **"Criterion 3 and 4 are mutually inconsistent"** — they are not; 4 is transient speed, 3 is the floor. Kept only as a one-line clarification.
- **Task 0 item reorder / demoting the `publish()` hunt** — contradicts Task 0's own title and CLAUDE.md §7's open list. Replaced with a budget instead.
- **`loop_dt_cv` / `loop_dt_p99` report fields** — extra fields, no decision changes.
- **A separate deadbanded turning-point path metric (5b)** — the slew clip plus the decimation rule already give comparability.
- **Pull-the-battery as a listed abort** — changes no agent behaviour.
- **Re-stating randomization / n / volts / surface inside individual experiments** — already a standing rule.
- **`config.jsonc` as a redeploy-free toggle path** — it is compiled in and read at OpMode init; editing it still costs a deploy. Corrected inline rather than adopted.

---

## The three things most likely to still go wrong

**1. The agent never finds out which code is actually running, and everything downstream is attributed to the wrong layer.** This is the real risk, and it is bigger than any single patch above. There are now at least three plausible steering paths — `SwerveBringUp`'s own kinematics at `:2562`, `DriveTeleOp`'s, and vendored Pedro's `Swerve.arcadeDrive()` reached through `CustomDrivetrain.runDrive()` — and CLAUDE.md asserts that `CoaxialPod` is what both OpModes drive without saying how they reach it. Every Pedro-side finding in this review (per-axis reverse-power clamp, X-lock default, `servoCachingThreshold`, voltage-compensation no-op) is conditional on that routing question, and so is half of Task 2. If I could add only one instruction it would be: **before any hypothesis testing, trace one stick deflection from `gamepad` to `turnServo.setPower()` and write the call chain into `FINDINGS.md`.** Nothing in the prompt asks for that, and without it the agent will test hypotheses against code that never executes.

**2. Criterion 3 gets closed with the wrong cause, confidently.** There are now four live candidates with overlapping signatures — stiction, backlash, the 2.86° write deadband, and a relay limit cycle — and at least three of them predict a residual in the 2.5–3° band. The signed-residual and CW/CCW reanalyses separate stiction from limit cycle, but they do **not** separate mechanical lash from the write deadband: both produce a direction-dependent park short of target. The only clean discriminator is the `servoCachingThreshold = 0` arm, and it needs a deploy. If the operator's time runs out before that deploy lands, the run will report "friction-limited, lubrication pass pending" — which is plausible, matches the kS data, and may be wrong. Watch for the agent asserting a cause when two candidates are still degenerate.

**3. Robot time runs out during Task 2 and the deliverables never get written.** The prompt front-loads the two hardest open problems (37.4 ms `publish()`, criterion 3) and puts the only concrete artifacts — six graphs, a numerically validated path, a refilled table — at the end. Every patch above adds work: a scripted demand mode, clamp columns, `flipCmd`, `vx/vy/omega` columns, a lint, a per-joint continuity table, a numeric envelope checker, `RUN_STATE.md`. That is a lot of build for a run also doing physics. The Phase C fan-out is the mitigation and it is why I would not skip it: get the plotting scripts and the offline path validator built by separate agents during the first OPS wait, so that when the clock dies at 70% context the deliverables already exist and only need data poured into them.