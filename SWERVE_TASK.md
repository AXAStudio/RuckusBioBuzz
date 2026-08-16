# Task: swerve steering quality — loop, heading, azimuth continuity, path smoothness

Read `CLAUDE.md` first. It has the hardware, the code map, the measured
baseline, and the list of hypotheses already refuted. **Do not re-derive
anything that file already establishes, and do not re-run a refuted
experiment.** If you find the file is wrong, fix it in the same commit.

---

## Run this end to end in one go

**Work Tasks 0 → 1 → 2 → 3 → Verification in a single continuous session. Do
not stop between tasks to ask whether to continue, and do not wait for me to
approve a plan before you start.** Post a numbered result block at the end of
each task and keep moving. The only things that legitimately stop you are
physical robot operations and a genuinely blocking unknown — both are handled
by the batching rules below.

"One change, one test" still applies to **experiments** — do not stack three
code changes and run one trial, because then you cannot attribute the result.
It does **not** mean pausing for my permission between changes.

### Question protocol — batch into surveys, never drip-feed

When you have questions, **collect them and ask them as one numbered survey.**
Never send a single question and wait. Every survey item must be:

```
Q3. Heading interpolation for the scoring approach segment
    Why I'm asking: linear across a curve is the usual source of robotic-
    looking swerve paths; I can't tell from Full18Auto.java which you want.
    a) Tangential — heading follows path tangent            [my recommendation]
    b) Constant — hold the approach heading through the arc
    c) Linear — interpolate start→end over the segment
    DEFAULT IF YOU DON'T ANSWER: (a)
    BLOCKING? No — I'll proceed on (a) and flag it in the report.
```

Rules:

- Multiple choice with a recommended option and an explicit **default**.
- Mark each item **BLOCKING** or **NON-BLOCKING**. An item is BLOCKING **only**
  if taking the default could damage hardware, drive the robot into a wall or a
  person, or destroy measured data you cannot recapture. Nothing else.
  Thresholds, statistical choices, which experiment to run first,
  shipped-vs-diagnostic scope, naming, and file layout are **always
  NON-BLOCKING** — take the default, log it in `ASSUMPTIONS.md`, keep working.
  If you cannot write a default for an item, the item is malformed: split it
  until you can. When a late answer contradicts a default you took, apply it
  going forward and re-run only the work whose result it changes. Even on a
  genuinely BLOCKING item, keep working any thread it does not block.
- Ask **Survey 0 first, before any work**, covering everything you can already
  foresee — field frame, thresholds, surface, how much robot time I have. Then
  at most one more survey per task. If you find yourself writing a third survey
  in one task, you are drip-feeding.
- If I answer only some items, take the defaults on the rest and carry on.

### Robot operations — also batched

I am the hands. You cannot deploy, power the robot, put it on blocks, take it
off blocks, or start an OpMode. Deploying restarts the app, kills the OpMode and
the HTTP server, and needs me to restart `Swerve Bring-Up` on the Driver Station.

**Deploys are the scarcest resource — make each one carry many experiments.**
Ship every experimental change behind a **runtime toggle** drained through the
existing queued-command path (`setPidf`, `setPublishHz`, `pidStep` prove the
mechanism works), so one install yields A/B/C arms with no reflash. Note
`config.jsonc` is compiled in and read at OpMode init, so editing it still
costs a deploy. A knob that can only be exercised by reflashing is a last
resort; say why.

**"One change, one test" binds at trial granularity, not deploy granularity.**
One deploy may carry ten toggles; each trial varies exactly one, and each trial
record states the full toggle vector. If a build unlocks only one experiment,
you have not batched — find the others first.

**So do not ask for a deploy per change.** Queue code changes until you have a
coherent batch, then post one **OPS REQUEST** block:

```
OPS REQUEST 2
1. adb install (build is green, commit 4f21a09)
2. Put the robot on the FTC tiles
3. Start Swerve Bring-Up on the Driver Station
4. Reply "ready" + battery voltage
While you do that I'll be: writing the cross-track plotting script.
```

Always say what you will work on while you wait, and actually do it. Before any
command that drives the robot across the floor, the OPS REQUEST must have
confirmed it is on the floor and that bring-up is not fighting `DriveTeleOp`
for hardware. Assume nothing about robot state across a gap in the conversation
— re-confirm via `/state` when you resume.

### `RUN_STATE.md` — so an interruption does not cost robot time

Keep `RUN_STATE.md` at repo root. Rewrite it in full after each task's result
block, before each OPS REQUEST, and after each operator reply. A compaction or a
new session loses everything else.

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

**On resume (new session or after compaction): read `CLAUDE.md`,
`RUN_STATE.md`, `ASSUMPTIONS.md`, then the tail of `FINDINGS.md`.
`RUN_STATE.md` is a hypothesis, not a fact — before any command that moves the
robot, re-read `/state` and refresh `opmode_running` and `box_armed`; before
attributing any measurement to a code change, confirm
`on_robot_build == last_commit`.**

### Standing rules

- Every result gets: **surface** (bench/blocks vs FTC tiles), **battery volts**,
  **n**, and **spread**. A/B comparisons are randomized and interleaved.
- Loop rate is always `1/mean(dt)`. `mean(loopHz)` is inflated ~1.8× and is
  banned from your reports.
- Say for every change whether it lands in **shipped** code (`tele/`, `auto/`,
  `pedroPathing/`) or **diagnostic** code (`diagnostics/swerve/`,
  `tools/swervetune/`).
- Prefer measuring over reading source. Where the log and the source disagree,
  the log is the evidence and the source is the hypothesis.
- Keep a running `ASSUMPTIONS.md` and a running `FINDINGS.md` at repo root so I
  can catch up without rereading the transcript. Commit per finding.

**Begin now: post Survey 0, then start Task 0's source audit and build-only
work immediately without waiting for my answers.**

---

## Safety — read before Task 0

### ABORT

**"ABORT" from the operator means:** I stop sending to `/command` and to the
bench client immediately, post the last command I sent and when I sent it, and
send nothing motion-capable until you reply **"re-arm"**. A survey answer, a
"ready", or silence is not a re-arm. Any recorder chunk spanning an abort is
discarded, not analysed — I say how many.

**Your fast stop is STOP on the Driver Station.** It kills the OpMode, so I must
then assume the HTTP server is gone too. The dashboard's 400 ms watchdog only
cuts the browser drive path — backgrounding the tab is not an abort.

**I stop on my own** and ask you to hit STOP if a pod's measured azimuth stops
responding while I am commanding it to move (a CRServo stalled against a hard
stop), or if `/state` fails twice in a row while a drive command is outstanding.

### Pre-flight — first OPS REQUEST of the session, before any command that can move the robot

Starred items repeat after every deploy or battery swap. One reply, not
drip-fed.

1. 3 ft clear around the 51 × 46 in box. Tape the box on the floor plus a second
   line 6 in inside it. **The tape is the barrier** — Task 2 hypothesis 0 is
   about to test the software clamp for a direction-rotating bug, so do not rely
   on the clamp to keep the robot in.
2. Battery strapped in, leads clear of the wheels, resting volts reported
   (≥ 12.5 V to start; the baseline runs were 12.2–12.5 V).
3. \* Turn each pod by hand through its full 270° travel. Report any bind,
   grind, play, or debris in a wheel — a binding pod invalidates every
   kS/friction result before you measure it.
4. \* **Encoder liveness.** OpMode running, nothing commanded, operator turns
   each pod by hand while I watch `/state`. All four azimuth angles must move.
   One that holds still is a dead encoder: the CR loop then sees a constant
   error and holds full command on that servo indefinitely, and the 5 Hz
   four-servo rail total will not show it. STOP — do not command steering.
5. Sticks read ~0 at rest. Drift here fakes the deadband-0.06 chatter in Task 2
   hypothesis 1.
6. Driver Station within reach, STOP visible.

### "Armed" is not "correct" — the box can drift

The box is stored in the Pinpoint's **dead-reckoned pose frame**, so odometry
drift slides and rotates it relative to the floor while `/state` still reports
armed. 3° of yaw error moves the far corner of a 51 × 46 in box (68.7 in
diagonal) by **3.6 in**. Drift over a run is **unmeasured** — Task 1 measures
it. So "confirm the box is armed" is necessary and not sufficient.

In the same OPS REQUEST that re-marks corners A and B, add a **box check**: put
the robot back on the corner-A position, square to the box — I read x, y from
`/state`; repeat at corner B. PASS if both land within **2 in** *(guess —
Survey 0)* of the marked coordinates and the implied box is 51 × 46 ± 2 in. On
fail: reset pose, re-mark, redo — I do not command motion on a box that failed.
Log each pair in `FINDINGS.md` as the drift measurement Task 1 wants.

**Re-mark procedure — the order matters:** (1) place the robot on the corner-A
mark, square to the box, +x along the 51 in side, using the FIELD panel's
footprint-indicator centre as the anchor at both corners; (2) press **Reset
pose** — this must come BEFORE marking, because it clears the box, so pressing
it after Mark B destroys the box you just made; (3) **Mark corner A**; (4) drive
to the diagonally opposite mark (+51 in x, +46 in y) holding the same heading,
press **Mark corner B**; (5) reply with the x, y the FIELD panel reads at
corner B.

### Speed cap

Baseline is p95 driving azimuth error ~42° with 97 flips in one chunk — pods
fighting at speed scrub and lurch, and a 180° flip reverses a loaded wheel.

- Cap every diagnostic drive in Tasks 0–2 at **30% of full commanded
  translation and rotation** until criteria 1, 2, 5 and 6 hold. Convert it to
  in/s off the FIELD panel `v` readout and record it with every result. Survey 0
  default 30%.
- Task 2 hypothesis 0's edge-proximity check is a **deliberate drive into the
  hard limit**, and the hypothesis predicts it will chatter. Run it at **15%,
  one axis at a time, maximum 3 clamp engagements per run**, then stop and
  disarm. Say in the OPS REQUEST that this run drives into the clamp on purpose.
- The clamp acts on commanded **velocity, not position**, so it does not kill
  momentum — the robot coasts past the edge. Measure that overshoot once from
  the pose trail at the capped speed. If it exceeds the Task 3 clearance margin,
  lower the cap until it does not.

### On blocks is not the safe state

The standing rule covers driving off the floor; it does not cover on-blocks
work, which is where the pod step tests and the kS comparison happen. Every OPS
REQUEST that puts the robot on blocks gets these as numbered steps: clamp or
strap the chassis to the blocks (`pidStepAll` slews four pods at once at
214 °/s and the reaction can walk an unsecured chassis off them); hands, cables
and the gamepad lead clear of all four wheels before you reply "ready".

---

## Task 0 — Loop hygiene, and close the open `publish()` question

The known state: DRIVE is **30.9 Hz true** with a bimodal dt (29% @ 8.9 ms when
publish is skipped, 71% @ 53.6 ms when it runs). `msPublish` is still **37.4 ms
in DRIVE vs 13.4 ms in IDLE** after the `batteryVolts()` caching fix, and that
gap is unexplained. That is the loop problem. Everything else in the loop
(`msHeading` 1.81, encoders 2.6, `msMode` 5–6, `msTelemetry` 2.05) is small.

Do this:

0. **Trace one stick deflection from `gamepad` to `turnServo.setPower()` and
   write the call chain into `FINDINGS.md` before anything else.** This is the
   highest-value hour in the run. There are at least three plausible steering
   paths — `SwerveBringUp`'s own kinematics near `:2562`, `DriveTeleOp`'s, and
   vendored Pedro's swerve drivetrain reached through the follower — and
   CLAUDE.md asserts both OpModes drive `CoaxialPod` **without saying how they
   reach it**. Half of Task 2's hypotheses are conditional on that routing.
   Name every file, class and line the command passes through, for the
   dashboard path and the `DriveTeleOp` path separately, and say where they
   diverge. **Test no hypothesis against code you have not confirmed
   executes.**

0b. **Reconcile the loop baseline before improving it.** From ONE post-fix
   DRIVE trace report together: `1/mean(dt)`, the dt histogram with both mode
   locations and their sample fractions, and `mean(loopHz)` labelled inflated.
   The fraction-weighted mean dt must match `1/mean(dt)` within 5%. See the
   note in CLAUDE.md §6 — the 30.9 Hz and the 29/71 split are different traces
   and do not reconcile. Amend CLAUDE.md in the same commit.

1. **Find the remaining 37.4 ms.** The IDLE/DRIVE asymmetry says it is blocking
   on the Lynx bus behind the four servo writes plus four motor writes.
   Instrument `publish()` internally — per-field or per-section timers — rather
   than guessing. Report the breakdown.
2. **Audit bulk caching properly.** Report the actual `LynxModule`
   `BulkCachingMode` on every hub, where `clearBulkCache()` is called, and
   whether it is called exactly once per loop per hub. **`cache = 0.01` in the
   gain set is NOT this and is NOT a read cache** — CLAUDE.md §5 has the
   corrected reading (a servo *write* deadband of 0.01 power units). Confirm it
   against the vendored `CoaxialPod` source and fix CLAUDE.md if it is wrong.
3. **Audit the loop body** for: blocking sleeps, `hardwareMap.get()` inside the
   loop, `telemetry.update()` called more than once, and any hardware read that
   happens on a path that does not need it. `getVoltage()` is a Lynx ADC
   transaction that bulk caching does not cover — verify no other such call
   snuck back into the hot path.
4. **Measure `DriveTeleOp`'s own loop rate honestly.** It has never been
   measured with the correct statistic; the logged 75.8 Hz is inflated and the
   true figure is probably ~40 Hz. This is the loop that actually matters for
   Tasks 1 and 2, and `DriveTeleOp` has no `publish()` at all — so it may
   already be fine and the bimodality may be a pure tooling artifact. **Find
   out before you fix anything.** Add whatever minimal timing instrumentation
   `DriveTeleOp` needs; keep it cheap.

**Report:** `loop_hz_true`, `loop_dt_mean_ms`, `loop_dt_p90_ms`, and the min/max
dt, separately for `DriveTeleOp` and for `SwerveBringUp` in DRIVE; the publish
breakdown; the bulk-caching finding.

**Target before moving on:** `DriveTeleOp` at ≥50 Hz true with p90 dt < 25 ms.
Note the ceiling: the servo PWM frame is 20 ms ≈ 50 Hz, so going far past 50 Hz
buys nothing at the steering hardware. If `DriveTeleOp` already meets this,
say so and move on — do not optimize the diagnostic tool for its own sake.

---

## Task 1 — Diagnose heading lock

**First, establish that heading lock exists.** Prior logs show bring-up running
with `headingHold off` — right stick as raw rotation, described as "matching
`DriveTeleOp`". So the first question is not "why is heading lock bad", it is
**"is there a closed heading loop in the competition path at all, and where is
its setpoint?"** Read `DriveTeleOp`, `CoaxialPod`, and the Pedro follower
wiring in `pedroPathing/Constants.java`. Write what you find into `FINDINGS.md`
and keep going — if there is no closed heading loop in the competition path,
that *is* the answer to Task 1, and the rest of this task becomes "build one,
then characterize it."

Then characterize from a log, not from source:

- Capture a driving run with `tools/swervetune/drivecapture.py` (or extend the
  recorder if heading target/measured are not currently logged — say so if you
  need to add columns). Remember the recorder is 3000 samples and **stops when
  full**, so chunk it.
- Graph target heading vs. measured heading over time.
- Report: steady-state error, overshoot, oscillation frequency and amplitude,
  settling time after the turn stick returns to zero.
- **Does error scale with translation speed?** This is the tuning-vs-geometry
  discriminator. If heading error grows with translation but not with rotation,
  suspect module skew, wheelbase constants, or odometry — not gains.
- Verify heading error is wrapped to (−180, 180] and that the setpoint is
  **latched once** when the turn stick returns inside the deadband, not
  re-latched every loop. Deadband is 0.06 — check the latch does not chatter
  across the deadband edge.
- **Heading comes from the goBILDA Pinpoint, not a Control Hub IMU.** Report
  the Pinpoint read cost and rate, whether heading is read fresh each loop or
  from a stale cached value, and yaw drift over the run. Do not go looking for
  `getRobotYawPitchRollAngles()` unless you actually find an IMU in the path.

**State a root cause with log evidence before changing any code.** If the
evidence points at the localizer or the drivetrain geometry rather than the
heading PID, say that — do not tune your way around a geometry error.

---

## Task 2 — Fix quantized steering (azimuth snapping to ~45°)

Symptom: module azimuth appears to snap to 45° increments instead of tracking
the stick continuously.

**Rank your hypotheses before testing them, and say which log column would
falsify each.** My priors, strongest first — argue with them if the data
disagrees:

0. **The field bounding-box clamp.** The FIELD panel's hard limit clamps drive
   commands at the wall (51 × 46 in box, armed, persisted to the hub). **If that
   clamp is applied per-axis rather than along the commanded direction, it
   rotates the velocity vector every time it engages** — clamp `vx` alone and
   the commanded direction snaps toward ±90°; clamp both and it snaps toward a
   ±45° diagonal. Per-pod `atan2(py, px)` then follows, which is *exactly* the
   45° family in the symptom. It also chatters: clamp → robot stops advancing →
   unclamp → clamp. Check first whether the snapping correlates with proximity
   to a box edge or with the red edge-flash. **If it does, the fix is to clamp
   the magnitude along the commanded direction (or project onto the wall
   tangent), never per-axis** — and this is a real bug in shipped behaviour, not
   a tooling artifact. If the snapping happens dead centre of the box with the
   edges never flashing, cross this off and move to 1.

1. **X-lock.** `SwerveBringUp.java:2532` — zero input snaps the pods to an X
   park, which *is* a ±45° pattern. With deadband 0.06, flickering across the
   deadband edge toggles between the X-lock target and the stick target. That
   would look exactly like 45° snapping. **Check whether `DriveTeleOp` has the
   same zero-input park, and whether either applies a magnitude hysteresis.**
2. **Which input path is actually live.** There are three: the Driver Station
   pad (`SwerveBringUp.java:2384` — pod selection and test modes only, no drive
   sticks), the browser Gamepad API path (`dashboard.html:556-628`, 60 ms poll,
   400 ms watchdog), and `DriveTeleOp`'s own gamepad. Confirm which one you are
   observing. **If the symptom only appears when driving through the dashboard,
   it is a tooling artifact, not a robot bug** — that possibility is already
   flagged in prior logs and it must be ruled out first. Also rule out a
   D-pad/POV read standing in for the analog stick.
3. **Explicit snapping / cardinal-lock / rounding** in the drive OpMode. Grep
   for `Math.round`, `/ 45`, `* 45`, `Math.PI / 4`, `signum`, and any
   "cardinal", "snap", or "lock" identifiers across `tele/`, `auto/`, and
   `diagnostics/swerve/`.
4. **Analog encoder conversion.** `wheelThetaFromEncoder` is documented as a
   straight 1:1 offset+reversal map. Verify the voltage→angle math, the ADC
   resolution and reference (is it `getVoltage()` scaled against the actual
   max, or a hardcoded 3.3 V?), and wrap handling at the 0/360 seam. Note the
   ~120° of at-rest encoder noise already observed — check whether that is
   sensor noise or a conversion artifact. **Quantization on this path would
   show up in `wheel`, not in `tgt` — use that to separate command
   quantization from measurement quantization.**
5. **`atan2(py, px)` at `SwerveBringUp.java:2562`** — recomputed every loop with
   no magnitude gate, so when a pod's own `hypot(px, py)` is small the demand is
   noise. Prior logs say the targets looked smooth, so this is low priority, but
   a magnitude gate + last-good-angle hold is cheap insurance either way.
6. **Shortest-path flip rounding.** Confirm the optimize step compares against
   the *continuous* target, not a rounded one, and that flips are counted. One
   prior chunk logged **97 flips**; unintended flips are 180° jumps that will
   also read as "snapping".

Note what is **not** applicable: steering is CRServo, so `setPosition()`
resolution and servo deadband are not in the CR path. Only investigate those if
you find the shelved positional variant is actually live —
`swerve_positional_p0.xml`, `POSITIONAL_SHELVED.md`.

**Fix so azimuth setpoints are continuous with stick input.** Then prove it
with a `tgt`-column trace: consecutive-loop setpoint deltas, and a histogram of
setpoint values showing no clustering at 45° multiples.

---

## Task 3 — Smooth path

A smooth path over a hunting azimuth loop is wasted work, so this task depends
on 0–2. Do not let that stall the run: **the design, the profile, and the
visualizer validation are all offline work — do them regardless.** Defer only
the on-robot path run until 0–2 converge, and if they do not converge, say so
and hand me a path that is validated in the visualizer but untested on the
robot. Label it that way.

- **Pedro 2.1.2 API — the `Point` class was DELETED in 2.0.0.** Every Pedro
  example on the web is 1.x (`new BezierCurve(new Point(x, y, Point.CARTESIAN),
  ...)`) and **will not compile**. Do not search for examples. Pedro is vendored
  at `third_party/PedroPathing` via `includeBuild` — read that tree, do not
  hand-roll a spline library, and expect these 2.1.2 signatures:
  - `com.pedropathing.geometry.Pose(double x, double y, double heading)` —
    inches, **heading in radians**.
  - `com.pedropathing.geometry.BezierCurve(List<Pose>)` — any degree; a cubic
    is 4 poses. Also `BezierLine(Pose, Pose)`, `BezierPoint(Pose)` for a hold.
  - `com.pedropathing.paths.Path(Curve)`;
    `follower.pathBuilder().addPath(...).build()` → `PathChain`.
  - `follower.followPath(chain[, maxPower][, holdEnd])`,
    `follower.holdPoint(Pose)`.
  - For the continuity check: `Curve.getPose(t)`, `getDerivative(t)`,
    `getSecondDerivative(t)`, `getEndTangent()`, `getCurvature(t)`, `length()`.

  If a symbol above does not resolve, the vendored tree diverges from upstream
  — diff it and report that; do not go debugging the build.

- **C2 (curvature-continuous) at every joint — not C1.** On a swerve, module
  azimuth rate is **θ̇ = κ·v**, so a curvature step is a step in commanded
  azimuth *rate*, and a jerk limit on translation bounds none of it. That is the
  actual mechanism behind "robotic-looking". Build by the reflection rule, but
  **the construction rule is not the evidence:** `PathChain` only appends paths
  and sums lengths — it checks nothing, not even C0, so a chain with a 6 in gap
  builds and runs and the follower just yanks toward the new curve. Print a
  per-joint table before the path runs:
  **C0** `|end(i) − start(i+1)|` in inches (>0.01 in is a transcription bug);
  **tangent direction**, the angle between `getEndTangent()` of segment i and
  `getDerivative(0)` of segment i+1, in degrees (each `Path` carries its own t
  and t is not arc length — compare directions, not magnitudes);
  **curvature**, `getCurvature(1)` vs `getCurvature(0)`, as a step in 1/in.
  |Δκ| must be 0 at every joint. Where geometry forces a corner, dwell (v → 0)
  through it. "C1 achieved" without the table does not count.

- **Pedro 2.1.2 has no motion profile — no jerk limit, no acceleration limit,
  no time parameterization. Do not go looking for one.** It is a reactive
  vector follower. **`PathConstraints` is not a motion-constraint class:**
  `velocityConstraint` (default 0.1 in/s), `translationalConstraint`,
  `headingConstraint`, `tValueConstraint`, `timeoutConstraint` are all
  *end-of-path completion tolerances*. `setVelocityConstraint(30)` does **not**
  cap speed at 30 in/s — it ends the path as soon as speed drops below 30 in/s,
  silently breaking the end condition while leaving speed uncapped. The knobs
  that exist, and all you may use: `follower.setMaxPower(p)` /
  `followPath(chain, maxPower, holdEnd)`; `brakingStrength` / `brakingStart` /
  `usePredictiveBraking` (end-of-path deceleration only); and
  **`FollowerConstants.centripetalScaling` (default 0.0005), which together
  with curve geometry is your actual smoothness control.**
  `forwardZeroPowerAcceleration` / `lateralZeroPowerAcceleration` are *measured
  coast decelerations* feeding the braking model — not limits you set; do not
  present them as an acceleration limit.
  So bound acceleration **geometrically, not temporally.** Report peak lateral
  acceleration as `v²·κ_max` with v from `maxPower`, labelled **estimated**.
  Report a jerk number only from logged velocity differentiated twice with a
  stated filter — at 30–50 Hz with the known bimodal dt an unfiltered double
  difference is noise. Otherwise write **"jerk not measured"**. Do not report a
  limit you never enforced.

- **Azimuth rate caps speed before acceleration or jerk does.** Solve v(s)
  subject to `κ(s)·v(s) ≤ 171 °/s` (0.8 × the 214 °/s median slew, and below
  the 184 °/s slowest measured pod) first, then acceleration. At R = 10 in that
  caps v ≈ 30 in/s. Uncapped, a ±10 in S-curve at ~20 in/s demands a ~226 °/s
  azimuth-rate step at the joint — above the pod's slew ceiling, so the pods
  fall behind and cross-track error grows through the joint. State the binding
  constraint per arc-length station.

- **State how heading is interpolated per segment and name the method. Pedro
  takes RADIANS everywhere — write `Math.toRadians(...)`, never a bare degree
  literal.** Per-path on `PathBuilder`: `setTangentHeadingInterpolation()`,
  `setConstantHeadingInterpolation(h)`, `setLinearHeadingInterpolation(start,
  end[, endT[, startT]])`, `setReversed()`,
  `setHeadingInterpolation(HeadingInterpolator)`. Whole-chain: the same names
  with a `Global` prefix. In `com.pedropathing.paths.HeadingInterpolator`:
  `tangent`, `tangent.offset(rad)`, `constant(rad)`, `linear(startRad,
  endRad[, endT])`, and **`facingPoint(x, y)` / `facingPoint(Pose)`** — hold a
  target in view, usually better than `constant` on a scoring approach because
  it self-corrects. Add `facingPoint` as option (d) in the Survey 0 heading
  question.
  **Two traps, both in the vendored source:** (1) the `HeadingInterpolator`
  class javadoc says "these methods all use radians" and its own example
  directly below writes `constant(45)` labelled as degrees — that is 45
  radians; do not copy a literal out of Pedro's docs. (2)
  `PathChain.getHeadingGoal` feeds a `setGlobal*` interpolator `chainT` = arc
  length travelled ÷ total chain length, while a per-path interpolator gets
  that path's raw Bezier t. These diverge on a curve — pick one family per
  chain and say which.
  Heading interpolation is a feasibility constraint, not a style choice:
  chassis rotation adds ω to θ̇. Tangential gives ω = κ·v (continuous if the
  path is C2); constant or linear gives piecewise-constant ω that **steps at
  every joint**. If not tangential, smooth ω across the joint and report the
  resulting azimuth-rate step.
- **Bounding box is already set — do not ask about it.** The FIELD panel in
  `dashboard.html` has it marked and armed:

  - **Box: 51 × 46 in**, corners captured via Mark corner A / Mark corner B,
    persisted **to the hub**.
  - It is a **hard limit**: any drive command that would carry the robot out is
    **clamped at the wall**, and the edges flash red while clamping.
  - It lives in the **pose frame**, not an absolute field frame. **Reset pose
    clears the box.** Origin is wherever the pose was last reset; units are
    inches; the panel reads out `x`, `y` in inches and `v` in in/s.

  Consequences you must design around:

  1. This is a **51 × 46 in practice area, not a 144 × 144 in field.** Do not
     port a competition-scale path into it. Compute the usable envelope as the
     box minus the robot footprint and half-diagonal for rotation, and state
     that number before you place a single control point.
  2. **Any deploy or pose reset invalidates the box.** Every OPS REQUEST that
     includes a reflash or a pose reset must also include "re-mark corners A
     and B" as a numbered step, and you must re-read `/state` to confirm the
     box is armed before commanding motion.
  3. Read the actual clamp implementation in `SwerveBringUp` before you trust
     it, and say whether it clamps **position** or **velocity**, and whether it
     clamps **per-axis** or along the commanded direction — see Task 2
     hypothesis 0, this matters there too.
  4. Design the path so it never touches the clamp. A path that relies on the
     hard limit to stay legal is not a smooth path; report any point where the
     planned trajectory comes within the robot half-width of an edge.
- **The visualizer is a sanity picture, not the clearance check.** It draws an
  absolute field frame; the box lives in the **pose frame** with its origin
  wherever pose was last reset, so the visualizer cannot show box clearance at
  all. First write an offline checker that samples every `Path` in the
  `PathChain` at ≥200 points per segment and prints: min clearance to the
  envelope in inches with the `t` where it occurs; pass/fail against the robot
  half-width; max |curvature| and its `t`; and per-module **commanded azimuth
  rate** vs arc length with the 214 °/s slew ceiling and the 171 °/s design
  limit drawn. Any pod crossing 171 °/s means reshape the path — do not run it.
  Commit the checker as a test so it reruns on every control-point edit.
  **Then** open the visualizer (`./gradlew visualizer`,
  http://127.0.0.1:5173/) to eyeball gross shape. Label the handoff "validated
  numerically against the envelope; untested on the robot".

---

## Verification (required)

Run the path, capture through the existing harness
(`tools/swervetune/drivecapture.py` → `runs/`), and produce these graphs. Say
which log columns feed each; if a column does not exist yet, add it and say so.

1. Per-module commanded azimuth vs. measured azimuth — all four pods, one graph.
2. Robot heading: target vs. measured.
3. Chassis velocity and acceleration vs. time (and jerk if you can get it).
4. Cross-track error vs. distance along path.
5. Loop dt vs. time — `dt`, not `loopHz` — to confirm nothing regressed.
6. Wheel-path-length ÷ commanded-path-length, and reversals/s, per pod. These
   are the two numbers that quantify the shake; the current baseline is 1.7–3.0×
   and 2.58–4.18 reversals/s against a 0.41–0.53/s demand.

Deliver the graphs as image files plus the raw numbers in text.

---

## Success criteria — iterate until all hold

Thresholds below are grounded in the measured baseline in `CLAUDE.md`. Where
one is a guess, it is marked — **put every guessed threshold in Survey 0 as a
multiple choice with your recommendation as the default, and start working
against the default.** If you think a non-guessed threshold is wrong, say so in
Survey 0 too, not after you miss it.

| # | Criterion | Threshold | Current |
|---|---|---|---|
| 1 | Unintended azimuth setpoint jump between consecutive loops | **≤ 15°**, and zero clustering at 45° multiples. Deliberate shortest-path flips excluded but **counted and reported** | 45° snapping observed |
| 2 | Unintended 180° flips per pod | **< 0.2 /s** while driving | 97 flips in one chunk |
| 3 | Per-module azimuth steady-state tracking error | **< 1.0°** (existing criterion 5) | 2.65–3.01° — **2.9× over** |
| 4 | 90° azimuth step settle to ±2° | **≤ 500 ms software-only**; < 350 ms needs ~1.6× pod slew, i.e. hardware | ~647 ms = 421 ms saturated + ~226 ms settle |
| 5 | Wheel path ÷ commanded path while driving | **< 1.3×** | 1.7–3.0× |
| 6 | Wheel reversals/s while driving | **< 1.0 /s** | 2.58–4.18 /s |
| 7 | Robot heading error while translating | **< 3.0°** *(guess — Survey 0)* | unmeasured |
| 8 | Robot heading error at rest | **< 1.0°** *(guess — Survey 0)* | unmeasured |
| 9 | Cross-track error | **< 2.0 in** *(guess — Survey 0)* | unmeasured |
| 10 | `DriveTeleOp` loop | **≥ 50 Hz true**, p90 dt **< 25 ms** | unmeasured (~40 Hz est.) |
| 11 | No visible oscillation in the heading graph after settling | qualitative + p-p amplitude in the report | — |

**Criterion 4 is slew-bound, not gain-bound.** 421 ms of gross travel at
214 °/s already exceeds the old 350 ms target, and the pod is command-saturated
for all of it — gains act only on the remaining ~226 ms. Tune against 480–500 ms.
Report `t_sat` (|θ̇| > 0.9 × 214 °/s) and `t_settle` separately for every step
trial; if only `t_settle` is moving, gain work is finished and the remaining
lever is hardware — the 2× **overdrive** (est. ~2× slew, 540° coverage, passes
`verifyCoverage`; unmeasured, verify on the robot). Do not raise kP to chase the
step: it worsens relay chatter and fights criterion 3.

**Criterion 3's cause is not "unknown" — the archive already localizes it.**
CLAUDE.md §5: |ss| = **2.92° at kS = 0.035** vs **1.48° at kS = 0.045**. A 1.29×
change in kS moved the residual 1.97×. Loop rate (REFUTED, p = 0.70) and creep
quantum (REFUTED) are both independent of command authority and cannot produce
that. **Work these in order. Do not start at step 4.**

1. **Zero-robot-time reanalysis — do this before requesting robot time for
   criterion 3.** Every residual in this repo is `|ss|`; the abs() destroys the
   discriminator. Recompute signed `wrap(tgt − wheel)` from `runs/` and
   `trials.jsonl`: mean ± 95% CI per pod, **split by step direction** (sign of
   the commanded step, recoverable from `tgt`).
   Both directions parking short ⇒ stiction/hysteresis — the lubrication pass,
   `kS`, and a `kI` band are the levers, and the gap between directions is the
   hysteresis width.
   Signed mean ≈ 0 with 2.7° RMS ⇒ symmetric relay limit cycle — `kD` and slew
   are the levers. If it is a limit cycle, report its dominant frequency in the
   **47.8 Hz and 92.1 Hz** arms already sitting in `trials.jsonl`: frequency
   scaling with loop rate ⇒ discrete-time relay limit cycle; fixed frequency ⇒
   mechanical. This *re-reads* the refuted loop-rate A/B; it does not re-run it.

2. **The `servoCachingThreshold` write deadband — untested, and the arithmetic
   lands inside the measured band.** See CLAUDE.md §5: a 0.01 command step ≈
   2.86° of error at `turnKP = 0.200` with radian error units, and inside
   `turnKSBandDeg = 2.0` the relay is zeroed so the command goes **stale**
   rather than updating. Test: sweep the threshold over {0.01, 0.003, 0.001,
   0.0} at fixed kP/kD, randomized and interleaved, reading |ss| off the
   `wheel` column. Lower values raise the write rate toward the 20 ms / 50 Hz
   PWM frame — report loop dt per arm.

3. **Mechanical backlash / per-direction dead zone.** The encoder is 1:1 with
   the pod, i.e. *downstream* of any lash, so lash is a plant dead zone the
   controller cannot see through. With kI = 0 and kS acting as a relay, that
   puts a floor under settled error that no gain change can move — which is
   exactly what a residual indifferent to loop rate, creep quantum, and every
   gain tried so far looks like. **Test (~10 min, no code change, batch into an
   existing on-tiles OPS REQUEST):** command one absolute azimuth target 20
   times — 10 approaches from CW, 10 from CCW, randomized order, volts logged.
   Report mean settled position per direction. Half the CW/CCW gap is the
   loop's one-sided directional hysteresis and **bounds what any gain set can
   achieve.**

4. Only then: `kD` on measurement vs. on error, and the `kI` band / reset
   (kI is still 0 and the old "30–45° of hunting" result predates both the
   caching fix and the integral band).

**Warning — steps 2 and 3 are degenerate on the signed-residual test.** Both
mechanical lash and the write deadband produce a direction-dependent park short
of target. The only clean discriminator is the threshold = 0 arm, and it needs a
deploy. If you run out of robot time before that deploy lands, **do not report
"friction-limited, lubrication pass pending" as the cause** — it is plausible,
it matches the kS data, and it may be wrong. Report two live candidates.

**If a criterion cannot be met, say so with the number you actually hit and
why. Do not quietly lower the bar and do not report an estimate as a
measurement.** Keep iterating until the graphs are clean or you can tell me
precisely what is stopping them.

---

## Done means

You stop when one of these is true, and you say which:

1. All eleven criteria hold, with graphs and numbers.
2. Every remaining miss has a stated cause, a stated reason it is not fixable in
   software, and a proposed next experiment.
3. You are blocked on a BLOCKING survey item or an OPS REQUEST, and there is no
   unblocked work left. List exactly what you need.

Final deliverable, all at once: `FINDINGS.md` (per-task results with numbers),
`ASSUMPTIONS.md` (every default you took), the six verification graphs as image
files, the criteria table refilled with measured values, and a commit log where
each commit carries its evidence. Amend any comment in the codebase that this
run proved wrong, in the same commit that proves it.
