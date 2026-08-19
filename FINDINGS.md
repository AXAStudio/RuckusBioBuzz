# Swerve steering quality — findings

Session of 2026-08-16. Every number here is measured unless it says "estimated" or "predicted".
Loop rates are `1 / mean(dt)`. `mean(loopHz)` is never quoted as a rate.

Data sources used: the 31 archived runs in `tools/swervetune/runs/`, and live `/state` reads
from the robot (Swerve Bring-Up running, DRIVE mode, at rest, 12.76 V).

Tools written this session (both diagnostic):
- `tools/swervetune/steerqual.py` — all eleven criteria and the six verification graphs from one
  recorder CSV.
- `tools/swervetune/jumpcause.py` — replays the demand chain host-side and attributes every
  azimuth-setpoint discontinuity to a specific branch of the mixer.

---

## Task 0 — Loop hygiene and the publish() question

### 0.1 The remaining 34–37 ms of `publish()` is CPU, not the Lynx bus — REFUTES the standing hypothesis

Measured live, **robot at rest in DRIVE, zero drive command, pods X-locked, no servo or motor
writes happening**, 60 consecutive `/state` samples over 25 s at 12.76 V:

| field | mean | min | max |
|---|---|---|---|
| `msPublish` | **36.67 ms** | 32.31 | 39.58 |
| `msEncoders` | 2.00 | 1.94 | 2.20 |
| `msHeading` (incl. idle-path current + battery) | 5.42 | 5.00 | 6.00 |
| `msTelemetry` | 0.65 | 0.51 | 0.82 |
| `msMode` | 0.19 | 0.17 | 0.26 |

The task's standing hypothesis was that publish blocks on the Lynx bus behind four servo writes
plus four motor writes. **It cannot be:** there are no actuator writes at all in this sample and
publish still costs 36.7 ms. The IDLE/DRIVE asymmetry recorded on 2026-08-13 is explained by
what `appendPod` used to do — eight live `getPower()` transactions per publish — and that was
already fixed. What remains is pure computation.

The candidate that survives: **`fmt()` is `String.format(Locale.US, "%.4f", v)`**, and one
publish makes roughly a thousand calls to it — about **780 of them from the 260-sample trace
alone** (`appendTraceSeries` emits `t`, `tgt` and `act`), plus ~160 from the four pods and ~60
from the header. At 35 µs per call that is 35 ms, which is the whole measurement.

This also explains the loop-rate spread across the archive: publish runs on a 50 ms timer, so
once the loop period exceeds 50 ms **every** loop pays the publish cost and the slow mode
sustains itself. Human-driven runs (heading hold on → trace full → ~780 extra formats) sit at
22–24 Hz; a scripted box drive with the same code sits at 51 Hz.

| run | loop_hz_true | dt mean | dt p50 | dt p90 | dt p99 |
|---|---|---|---|---|---|
| `mydrive-001` (human, dashboard) | 24.1 | 41.6 | 48.4 | 64.1 | 77.1 |
| `drift-look-001` (human, dashboard) | 22.5 | 44.5 | 51.1 | 66.1 | 79.4 |
| `boxdrive-ebburst-pulsed` (scripted) | 51.1 | 19.6 | 12.1 | 52.9 | 68.6 |
| `slowfix` (scripted crawl) | 34.6 | 28.9 | 32.9 | 51.4 | 58.7 |

**Not yet proven**, and the reason for the deployed instrumentation: per-section timers inside
`publish()` plus a formatter A/B (`setFastFmt`) that can be interleaved inside one session.

### 0.2 Bulk caching — audited, correct on both OpModes

| | mode | cleared |
|---|---|---|
| `DriveTeleOp` | `MANUAL` on every `LynxModule` | once per loop, top of `loop()` |
| `SwerveBringUp` | `MANUAL` on every `LynxModule` | once per loop, top of `serviceLoop()` |

`MANUAL` rather than `AUTO` is deliberate and correct here: DRIVE reads every pod encoder three
times per loop (`readEncoders`, `Swerve`'s avgScaling pass, `CoaxialPod.move`) and `AUTO`
re-fetches a whole bulk packet on a repeated same-channel read.

`cache = 0.01` in the gain set is **not** this. It is `CoaxialPod.servoCachingThreshold` — the
minimum change in servo power before the pod actually writes it. Two unrelated things with one
nickname; both are correct as configured.

Reads that bulk caching does **not** cover, and where they now live:
- `VoltageSensor.getVoltage()` — Lynx ADC transaction. `SwerveBringUp`: idle path only (5 Hz).
  `Swerve.getVoltageNormalized()`: **in the shipped mixer, but dead** — `useVoltageCompensation`
  is `false` (SwerveConstants default, not overridden), so it never runs. Verified, not assumed.
- `LynxModule.getCurrent()` — idle path only.
- Pinpoint (I2C) — gated on `headingInUse() || refreshIdleSensors`.

No blocking sleeps in either loop body. No `hardwareMap.get()` inside either loop.
`telemetry.update()` is called exactly once per telemetry tick in both.

### 0.3 `DriveTeleOp` loop rate — instrumented, not yet measured

Never measured honestly; the logged 75.8 Hz was `mean(1/dt)`. `DriveTeleOp` has no `publish()`
and no recorder, so nothing existed to measure it with. Added `TeleLoopProbe` (diagnostic class,
three lines in the OpMode): 1 ms-bin dt histogram → `1/mean(dt)`, p50/p90/p99/min/max on the
Driver Station, plus the same `PodRecorder` and a `/swerve/state` snapshot so
`drivecapture.py` and every host scorer work against the competition OpMode unchanged.

**Awaiting OPS REQUEST 1.**

---

## Task 1 — Heading lock

### 1.1 There is no closed heading loop in the competition path. At all.

Traced end to end:

```
DriveTeleOp.loop()
  → follower.setTeleOpDrive(forward, strafe, turn, true)
      → VectorCalculator.setTeleOpMovementVectors(...)
          teleopHeadingVector = Vector(magnitude = turn stick, theta = current heading)
  → follower.update()  [manualDrive branch]
      → CustomDrivetrain.runDrive(centripetal, heading, pathing, robotHeading, velocity)
          → calculateDrive(...)  returns  headingPower.dot(Vector(1, robotHeading)) = turn stick
      → Swerve.arcadeDrive(forward, strafe, rotation = turn stick)
```

The right stick is a **rate command straight through the mixer**. There is no setpoint, no
error, no latch, no wrap handling — because there is nothing to wrap. Releasing the stick
commands zero rotation and the robot keeps whatever heading momentum left it with.

The heading hold the prior sessions tuned lives **only in `SwerveBringUp.runDriveMode()`**
(diagnostic): setpoint integrated from the stick at 7 rad/s, ±60° lead cap, a three-phase
release machine (ACTIVE → STOPPING → RESTING), epsilon-bypass trim with 1.2°/0.5° hysteresis,
`headingKp` 1.20 / `headingKd` 0.080, its own copy of the gains. None of it is in `tele/`.

So Task 1's answer is: **build one, then characterise it.** Every heading number below is from
the diagnostic path and describes the bring-up tool, not the robot as it competes.

### 1.2 Heading hold as measured in the bring-up path (`mydrive-001`, 71.9 s, 12.37 V, tiles)

| metric | value |
|---|---|
| \|error\| while translating (>2 in/s) | mean **7.83°**, p95 60.0°, max 60.0°, n=1005 |
| \|error\| at rest | mean **3.65°**, p95 15.07°, max 44.04°, n=608 |
| error vs translation speed | r = **−0.168**, slope −0.126°/(in/s), intercept 9.75° |
| error vs \|turn command\| | r = **+0.903** |

Read the two correlations together: **heading error does not grow with translation speed — it
grows with rotation command.** Error is *worst at crawl* (0–5 in/s: 12.17° mean; >30 in/s:
4.07° mean), which is the opposite of a geometry or wheelbase error and consistent with a
setpoint that is being swept by the stick faster than the chassis can follow.

The p95 and max of exactly **60.00°** are not a coincidence and not a controller property: that
is `HEADING_MAX_LEAD = 60°`, the deliberate cap on how far the setpoint may lead the robot. The
error saturates against the cap during any sustained turn. **Any "steady-state heading error"
number taken while the turn stick is held is measuring the lead cap, not the loop.**

Not yet answered, and needing a purpose-built capture rather than a driving session: overshoot,
oscillation frequency/amplitude, and settling time after the stick returns to zero. The
`headingStep` routine already exists in the bring-up tool for exactly this.

### 1.3 Heading source

goBILDA Pinpoint, read fresh every loop that needs it (`readHeading` → `pinpoint.update()`),
gated so IDLE pays 5 Hz instead of loop rate. Confirmed: **no Control Hub IMU anywhere in the
control path**. Read cost is inside the 5.42 ms `msHeading` figure above, which also carries the
idle-path current and battery reads — the isolated 1.81 ms in CLAUDE.md still looks right.
Yaw drift over a run: not yet isolated (needs a stationary run with the recorder going).

---

## Task 2 — Quantised steering

### 2.1 It is real, it is in the demand, and the pods are innocent

`mydrive-001`, while driving, per pod:

| pod | jump p90 | jump max | jumps >15° | /s | flips/s | wheel path ÷ commanded | wheel rev/s | **commanded rev/s** |
|---|---|---|---|---|---|---|---|---|
| 0 RB | 17.8° | 158.5° | 139 | 2.71 | 0.66 | 1.01 | 4.17 | 4.17 |
| 1 RF | 14.4° | 179.3° | 106 | 2.07 | 0.70 | 0.89 | 4.56 | 3.78 |
| 2 LF | 17.8° | 149.9° | 133 | 2.59 | 0.47 | 1.01 | 4.91 | 3.90 |
| 3 LB | 15.6° | 168.3° | 112 | 2.18 | 0.64 | 1.01 | 4.44 | 3.47 |

**Path ratio ≈ 1.0 and commanded reversals ≈ wheel reversals.** The wheels are faithfully
tracking a demand that is itself reversing 3.5–4.2 times per second. This inverts the
2026-08-13 diagnosis, which had the demand reversing 0.41–0.53/s and blamed a hunting closed
loop. Chasing pod gains cannot fix a shaking setpoint.

### 2.2 Every discontinuity attributed to a line of Java

`jumpcause.py` replays `computeTargets` host-side from the logged applied command. Replay
residual against the logged targets: **mean 0.0001°, max 0.0006°** over 1731 samples — the model
is exact, so the attribution below is arithmetic, not inference.

601 jumps >15° while driving, n=1731 samples, 51.4 s of driving:

| cause | jumps | share | typical size |
|---|---|---|---|
| **rotation epsilon wall** (rotation term switched on/off wholesale) | 252 | 42% | — |
| **X-lock engage/release** (pods park on their own radii: the ±43.5°/±136.5° family) | 113 | 19% | — |
| **unattributed** — heading-PID rotation jitter at low translation | 126 | 21% | — |
| **per-axis deadband** (one translation axis exactly zeroed) | 63 | 10% | — |
| **flip** (deliberate 180°, hysteresis working) | 30 | 5% | 180° |
| **translation epsilon wall** | 17 | 3% | **mean 74–95°, max 179°** |
| **box fence** | **0** | 0% | — |

Ranked against the task's priors:

0. **Box clamp — not the cause here.** Zero clamped samples in this run (detector: after
   `applyBoxLimit` a clamped sample has an exactly-zero *field* axis with the other live).
   But the mechanism the task described is real and the code does clamp **per-axis**:
   `applyBoxLimit` zeroes `vx` or `vy` independently, with a speed-dependent margin
   (4 in + 0.3 s × closing speed) that shrinks as the robot slows — so engaging it both rotates
   the command and chatters. For an axis-aligned box, zeroing the outward normal *is* the right
   sliding behaviour; the defect is that it is a **step**, not a taper. Diagnostic code only.
1. **X-lock — confirmed, 19%.** Present in shipped constants (`ZeroPowerBehavior.X_LOCK`) and
   already carries a 0.35 s engage delay. `DriveTeleOp` inherits exactly the same behaviour.
   No magnitude hysteresis on the epsilon itself.
2. **Input path — the symptom is not a dashboard artifact, but the dashboard has its own copy of
   the bug.** `dashboard.html` `padAxis()` deadbands each axis at 0.06;
   `DriveTeleOp.applyDeadband` deadbands each axis at 0.05. Both **rotate the commanded
   direction** instead of shortening it: 35.5% of driving samples had exactly one translation
   axis zeroed, transitioning 1.44 times per second.
3. **Explicit snapping — none.** No rounding, cardinal-lock or `Math.round` anywhere in the
   demand chain. The 45° family comes from geometry: the chassis is 146.42 × 154.24 mm from
   centre, so the pure-rotation azimuths are ±43.5° / ±136.5°, and pure-translation cardinals
   are 0/90/180/270. Every wholesale switch between "translation" and "rotation" therefore
   lands on the 45° family — which is why the symptom looks like snapping to 45°.
4. **Encoder conversion — not implicated.** Quantisation would appear in `wheel`, and `wheel`
   tracks `tgt` at ratio ≈ 1.0.
5. **`atan2` with no magnitude gate — this is the 21% "unattributed" bucket.** At low
   translation the pod demand direction is `atan2` of (translation + rotation), so its
   sensitivity to the rotation term is ≈ trans/(trans²+rot²). Worked example from the log,
   consecutive loops: f/s/t `−0.077/+0.168/+0.204` → `−0.085/+0.187/+0.121`; translation
   essentially unchanged, rotation moves 0.08, demand jumps **67.4°**. The rotation term is the
   heading PID's output, so at crawl the heading controller's own jitter is what swings the
   pods.
6. **Flip rounding — working correctly.** 30 of 601 jumps, ±10° hysteresis band, compared
   against the continuous target. Flip rate 0.47–0.70/s against a 0.2/s criterion, i.e. still
   over, but these are legitimately-decided flips, not rounding artifacts.

### 2.3 A second per-axis clamp, this one only in the shipped path

`CustomDrivetrain.runDrive` (vendored Pedro, shipped) ends with:

```java
double clampedForward = clampReversePower(translationalVector.getXComponent(), robotVelocity.getXComponent());
double clampedStrafe  = clampReversePower(translationalVector.getYComponent(), robotVelocity.getYComponent());
```

`clampReversePower` caps a component at ±0.2 when it opposes the measured velocity on **that
axis alone**. Clamping one component and not the other rotates the commanded direction — the
exact mechanism the task predicted for the box fence, sitting in the shipped teleop path. It
engages on every deceleration and stick reversal, and releases the moment the velocity component
crosses zero, so it also chatters. **`SwerveBringUp` bypasses it entirely** (it calls
`Swerve.arcadeDrive` directly), which means every measurement in this repo's archive was taken
without it and `DriveTeleOp` may be worse than anything measured so far.

### 2.4 The recorder's `tgt` column had drifted from the mixer

`SwerveBringUp.computeTargets` is a host-side mirror of `Swerve.arcadeDrive`. The mixer's
rotation epsilon moved to 0.015 on 2026-08-15 (commit ced6c13); the mirror kept 0.05. For
`|rotation|` in [0.015, 0.05) the recorder logged a translation-only demand while the pods were
given translation plus rotation: **4.0% of samples in mydrive-001, worst disagreement 15.0°** —
in the one column that is supposed to discriminate "the demand is shaking" from "the response is
shaking". The mirror also lacked the X-lock 0.35 s engage delay.

Fixed both, and added `p{i}_ctgt` — the demand read back out of the pod
(`CoaxialPod.getLastTargetWheelRad()`), which cannot disagree with what the pod acted on. `tgt`
is kept so the archive stays comparable.

### 2.5 Predicted-but-unverified: teleop centripetal correction is dead code

`VectorCalculator.teleopUpdate()` does `velocities.add(v); velocities.remove(velocities.get(velocities.size()-1))`
— which removes the element just added (`Vector` does not override `equals`, and
`PoseTracker.getVelocity()` returns a fresh object each call). So `averageVelocity` stays zero
forever, `curvature` is always NaN, and `getCentripetalForceCorrection()` always returns an empty
vector in teleop. Stock Pedro bug, not a Ruckus patch.

This is load-bearing: if the list ever *did* update, teleop curvature would be finite and
`getCentripetalForceCorrection()` would dereference `currentPath`, which is **null** in
`DriveTeleOp` — an NPE on first movement. Flagged as a risk for the first `DriveTeleOp` run.
Predicted safe; **not verified on hardware.**

### 2.6 The fixes, and what simulation says they buy

Four distortions fixed. Each has its own measurable signature, so one capture can attribute
them separately even though they deploy together.

| # | Where | Side | Fix |
|---|---|---|---|
| 1 | `DriveTeleOp.applyDeadband` | **shipped** | deadband the translation **vector**, rescaled from the band edge |
| 2 | `CustomDrivetrain.clampReversePower` | **shipped** | project onto the direction of travel, scale the whole vector |
| 3 | `Swerve.arcadeDrive` epsilon walls | **shipped** | smoothstep taper across each band instead of a step |
| 4 | `Swerve.arcadeDrive` demand rate | **shipped** | slew limit at 214 °/s, the measured pod slew |
| 5 | `SwerveBringUp.applyBoxLimit` | diagnostic | taper the outward component over 6 in |
| 6 | `dashboard.html padAxis` | diagnostic | same vector deadband as fix 1 |

Simulated over `mydrive-001`'s recorded commands — 51.3 s of real driving replayed through both
mixers. **Physical** (mod-180, so deliberate flips are excluded) consecutive-loop demand change:

| configuration | jump p90 | jumps >15°/s | demand reversals/s |
|---|---|---|---|
| as shipped 2026-08-15 | 19.8° | 2.9 | 4.25 |
| epsilon taper only | 19.8° | 2.9 | 4.37 |
| demand slew 214 °/s only | 13.2° | 0.9 | 3.47 |
| taper + slew 214 °/s | **13.2°** | **0.9** | **3.47** |

Two results worth stating plainly because they are negative:

- **The taper alone does almost nothing.** Epsilon crossings are 3% of jumps and the input
  usually crosses the wall in a single loop, which no taper can smooth. It is kept because it
  removes a genuine discontinuity, not because it moved the number.
- **300 °/s, tried first, is worse than 214 at this loop rate.** It spreads one big jump into
  several 19° steps and the count of violations goes *up* (151 → 174 per pod). Rate limits
  interact with loop period; they are not free.

And the consequence that reorders the work: **criterion 1 is a loop-rate criterion.** A slew
limit is a rate, so a slow loop turns any rate into a big step. 53% of the jumps that survive
the 214 °/s limit in simulation land on loops longer than 70 ms (= 15° at 214 °/s). At 50 Hz
true with a 25 ms p99, the same limit permits 5.4° per loop. **Task 0 must land before Task 2's
criterion can be judged.**

---

## Task 3 — The path

Designed offline, validated offline, **not yet run**.

### 3.1 The envelope, computed before any control point was placed

Box read live from `/state`: x ∈ [−2.08, 48.71], y ∈ [−32.77, 12.87] → **50.79 × 45.65 in**.
Robot footprint is **assumed 18 × 18 in** (FTC legal maximum — the safe bound; nothing in the
codebase records the real footprint, and the dashboard's outline is drawn from pod extents plus
a fixed pixel margin, not a measurement). See ASSUMPTIONS.

| heading mode | clearance needed | centre envelope |
|---|---|---|
| tangential (robot sweeps every orientation) | half-diagonal 12.73 + 2.0 cross-track | **21.34 × 16.19 in** |
| constant (footprint never rotates) | half-width 9.00 + 2.0 cross-track | **28.79 × 23.65 in** |

The follower bench's own 6 in waypoint margin is satisfied with room to spare in both.

### 3.2 Geometry: C2 by construction, not by inspection

A **closed uniform cubic B-spline**, converted span by span to Bézier form
(`b0 = (d0+4d1+d2)/6`, `b1 = (2d1+d2)/3`, `b2 = (d1+2d2)/3`, `b3 = (d1+4d2+d3)/6`). C1 and C2 are
then properties of the representation rather than something hand-placed control points must be
checked for — including at the closing joint, which is where a hand-built loop usually fails.

Measured residuals at the **worst joint of all**, both variants: C0 = 0, C1 ≤ 1.5e-14 in,
C2 ≤ 1.4e-14 in. So: **C2 at every joint, including the wrap-around.** It is parametric C2 from
a uniform spline, and because the spans carry equal parameter speed it is also geometric G2 —
curvature is continuous, which the κ trace shows directly.

| variant | segments | length | min radius | max \|dκ/ds\| |
|---|---|---|---|---|
| tangential | 8 cubics, closed | 53.41 in | 5.24 in | 0.0171 in⁻² |
| constant | 8 cubics, closed | 74.46 in | 8.29 in | 0.0073 in⁻² |

### 3.3 Jerk: Pedro 2.1.2 has no jerk limit, so it is bounded by construction and reported

`PathConstraints` carries end-of-path tolerances and braking behaviour only — Pedro is a path
follower, not a trajectory follower, and there is no jerk parameter to set. Saying otherwise
would be inventing a feature. What was done instead:

- C2 geometry bounds `dκ/ds`, and at constant speed lateral jerk is `v³·|dκ/ds|`.
- Speed is chosen from a **pod-slew budget**: 25% of the measured 214 °/s median pod slew. A pod
  riding its slew limit is open-loop — the PID has already saturated — so tracking error there is
  set by the plant, not by gains.

| variant | speed | lap | max lateral accel | max lateral jerk | max pod azimuth rate |
|---|---|---|---|---|---|
| tangential | 13.00 in/s | 4.11 s | 32.2 in/s² (0.083 g) | 37.6 in/s³ | 53.5 °/s (chassis yaw 142 °/s) |
| constant | 7.74 in/s | 9.62 s | 7.2 in/s² (0.019 g) | 3.4 in/s³ | 53.5 °/s |

Worth noting because it is not obvious: on a constant-curvature arc a **tangential** heading
keeps each pod azimuth *fixed* (at `atan(κ·r_pod)`, ±57.9° at the tightest corner) because the
chassis yaws with the path — the pods only move as curvature changes. A **constant** heading
makes the pod azimuth rotate at exactly `v·κ` the whole way round. That is why the two variants
have such different speed allowances for the same slew budget.

### 3.4 Heading interpolation, per segment, with the reason

- **Tangential variant** — `HeadingInterpolator.tangent` on all 8 segments. A closed loop has no
  "approach", and tangential is what makes a traverse look driven rather than dragged. Linear
  interpolation across a curve fights the translation the whole way; it is not used anywhere here.
- **Constant variant** — `constant(0°)` on all 8 segments. This one exists as the *experiment*:
  with heading fixed, every pod azimuth change comes from the path alone, which isolates Task 2's
  question from the heading loop entirely.

### 3.5 Clamp clearance

The path never approaches the fence. Minimum wall clearance: **12.41 in** (constant variant),
**15.52 in** (tangential) — against a 9.00 in half-width and a 12.73 in half-diagonal. The
tangential variant clears the half-diagonal everywhere; the constant variant clears its own
relevant bound (half-width) with 3.4 in to spare and does not need the half-diagonal because it
never rotates. And the follower path does **not** run through `applyBoxLimit` at all — the bench
validates every control point before anything moves, and a Bézier stays inside its control
points' convex hull, so that check bounds the whole curve.

### 3.6 Visualizer validation

`.pp` loaded into the local Pedro visualizer (translated to the field centre; the visualizer
knows only the 144 × 144 field, so the *envelope* check stays with `pathdesign.py` against the
real box). Result: **8 segments, 74 in, zero wall or obstacle collisions** against the
visualizer's own footprint check. Length agrees with the computed 74.46 in.

One thing the visualizer caught: its time estimate was 19.1 s against the computed 9.6 s,
because the exported `.pp` did not mark the segments as chained, so it profiled eight separate
stop-at-end paths. The `pedroChain` command builds a single `PathChain` and is unaffected — but
the exporter should carry the chaining flag, and until it does the visualizer's time estimate
for these files reads roughly double.

Pedro's `BezierCurve` uses the standard characteristic matrix over the control points, so a
4-point curve is a plain cubic Bernstein curve — identical to the Python that designed it. Read
from `generateBezierCurve`, not run.

---

## Corrections to CLAUDE.md forced by this session

1. §5 gains are stale. Shipped now: `turnKPPerPod` 0.380, `turnKD` 0.022, `turnKSPerPod` **0.022**
   (not 0.035 — that is still the scalar `turnKS`, which the factories no longer use), pulsed
   final approach enabled, and `CoaxialPod.TURN_GAIN_SCHEDULING` on.
2. §5's "a mechanical lubrication pass is pending" is wrong — it happened before 2026-08-13 and
   the gains were re-fitted after it.
3. §7's "the remaining 37.4 ms of publish is unfound" — the Lynx-bus explanation is refuted
   above; the surviving candidate is `String.format`.
4. §4's recorder description (7 global columns) predates the driver-session columns; it is
   14 global + 7 per pod now.
5. §6's "wheel reversals 2.58–4.18/s vs 0.41–0.53/s demand" no longer describes the robot: the
   demand itself now reverses 3.5–4.2/s.
6. The line numbers quoted in the task prompt (`SwerveBringUp.java:2532`, `:2562`, `:2384`) are
   stale; the file has grown to 4060 lines. Current: `runDriveMode` 2234, `computeTargets` 3593,
   `handleGamepad` 3471.

---

## Criteria table, as it stands

"sim" = replayed host-side through the new mixer over recorded commands, not measured on the
robot. "—" = needs a robot run that has not happened.

| # | Criterion | Threshold | Measured baseline | After fixes | Status |
|---|---|---|---|---|---|
| 1 | Setpoint jump between loops | ≤ 15°, no 45° clustering | p90 14.4–17.8°, 2.07–2.71/s over 15°, **55–57% within 5° of a 45° multiple** (22% if uniform) | p90 13.2°, 0.9/s over (sim) | **blocked on Task 0** — a rate limit cannot beat a 77 ms p99 loop |
| 2 | Unintended 180° flips | < 0.2 /s | 0.47–0.70 /s | — | open |
| 3 | Azimuth steady-state error | < 1.0° | 2.65–3.01° | — | open, cause still unknown |
| 4 | 90° step settle to ±2° | < 350 ms | ~647 ms, 65% slew-limited | — | open |
| 5 | Wheel path ÷ commanded path | < 1.3× | **0.89–1.01×** | — | **already met** — and the old 1.7–3.0× figure was wrong |
| 6 | Wheel reversals/s | < 1.0 /s | 4.17–4.91 /s, against a **3.47–4.17 /s demand** | demand 3.47 (sim) | open — but the demand is the target, not the pod loop |
| 7 | Heading error translating | < 3.0° | 7.83° mean (bring-up hold; p95 saturates at the 60° lead cap) | — | open; competition path had no loop until today |
| 8 | Heading error at rest | < 1.0° | 3.65° mean (bring-up hold) | — | open |
| 9 | Cross-track | < 2.0 in | never measured | — | needs the path run |
| 10 | `DriveTeleOp` loop | ≥ 50 Hz true, p90 < 25 ms | **never measured** — instrumented today | — | needs a run |
| 11 | No visible heading oscillation | qualitative + p-p | — | — | needs a run |

## What is left, and what it needs

Everything below is blocked on robot time, in this order:

1. **`setFastFmt` A/B, robot stationary** (~3 min, no motion). Settles the `String.format`
   theory of the 36.7 ms publish and, with it, criterion 10's prerequisite.
2. **`DriveTeleOp` loop histogram** — start the OpMode, drive briefly, read the Driver Station.
   Criterion 10 directly, and it decides whether the loop work is done or has just begun.
3. **One capture per path, before and after**, through `drivecapture.py` against both OpModes.
   Criteria 1–6 and the graphs.
4. **`HeadingHold`'s first run.** It is new, in shipped code, and untested. Criteria 7, 8, 11.
5. **The path run**, via `pedroStart` + `pedroChain`. Criterion 9 and the last graph.
