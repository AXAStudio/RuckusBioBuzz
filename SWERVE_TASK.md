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
- Mark each item **BLOCKING** or **NON-BLOCKING**. Non-blocking means you take
  the default, log the assumption in an `ASSUMPTIONS.md` at repo root, and keep
  working. **Only stop for genuinely blocking items**, and even then keep
  working on any parallel thread that is not blocked by it.
- Ask **Survey 0 first, before any work**, covering everything you can already
  foresee — field frame, thresholds, surface, how much robot time I have. Then
  at most one more survey per task. If you find yourself writing a third survey
  in one task, you are drip-feeding.
- If I answer only some items, take the defaults on the rest and carry on.

### Robot operations — also batched

I am the hands. You cannot deploy, power the robot, put it on blocks, take it
off blocks, or start an OpMode. Deploying restarts the app, kills the OpMode and
the HTTP server, and needs me to restart `Swerve Bring-Up` on the Driver Station.

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

## Task 0 — Loop hygiene, and close the open `publish()` question

The known state: DRIVE is **30.9 Hz true** with a bimodal dt (29% @ 8.9 ms when
publish is skipped, 71% @ 53.6 ms when it runs). `msPublish` is still **37.4 ms
in DRIVE vs 13.4 ms in IDLE** after the `batteryVolts()` caching fix, and that
gap is unexplained. That is the loop problem. Everything else in the loop
(`msHeading` 1.81, encoders 2.6, `msMode` 5–6, `msTelemetry` 2.05) is small.

Do this:

1. **Find the remaining 37.4 ms.** The IDLE/DRIVE asymmetry says it is blocking
   on the Lynx bus behind the four servo writes plus four motor writes.
   Instrument `publish()` internally — per-field or per-section timers — rather
   than guessing. Report the breakdown.
2. **Audit bulk caching properly.** Report the actual `LynxModule`
   `BulkCachingMode` on every hub, where `clearBulkCache()` is called, and
   whether it is called exactly once per loop per hub. `cache = 0.01` in the
   gain set is the pod encoder cache interval and is **not** this — check both
   and say which is which.
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

- Use **Pedro Pathing 2.1.2's own `BezierCurve` / `PathChain`** — it is
  vendored at `third_party/PedroPathing` via `includeBuild`, so do not
  hand-roll a spline library.
- Cubic Bezier segments with **C1 continuity at every joint minimum, C2 where
  the geometry allows** — for C1, the outgoing control point must be the
  reflection of the incoming one through the joint. State which you achieved
  per joint.
- **Velocity profile with bounded jerk, not just bounded acceleration.** State
  the jerk limit you chose and how you derived it. If Pedro 2.1.2 does not
  expose a jerk limit, say so and describe what you did instead rather than
  implying you got one.
- **State explicitly how heading is interpolated** — constant / linear /
  tangential — and justify it per segment. This is usually what makes FTC
  swerve paths look robotic: linear heading interpolation across a curve fights
  the translation the whole way. Tangential is usually right for a traverse,
  constant for a scoring approach.
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
- Validate the path in the Pedro visualizer (`./gradlew visualizer`,
  http://127.0.0.1:5173/) before it ever runs on the robot.

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
| 4 | 90° azimuth step settle to ±2° | **< 350 ms** | ~647 ms, 65% slew-limited |
| 5 | Wheel path ÷ commanded path while driving | **< 1.3×** | 1.7–3.0× |
| 6 | Wheel reversals/s while driving | **< 1.0 /s** | 2.58–4.18 /s |
| 7 | Robot heading error while translating | **< 3.0°** *(guess — Survey 0)* | unmeasured |
| 8 | Robot heading error at rest | **< 1.0°** *(guess — Survey 0)* | unmeasured |
| 9 | Cross-track error | **< 2.0 in** *(guess — Survey 0)* | unmeasured |
| 10 | `DriveTeleOp` loop | **≥ 50 Hz true**, p90 dt **< 25 ms** | unmeasured (~40 Hz est.) |
| 11 | No visible oscillation in the heading graph after settling | qualitative + p-p amplitude in the report | — |

Criterion 3 is the known blocker and its cause is **unknown** — loop rate and
"creep quantum" are both eliminated. Do not re-run those. Candidates worth
attacking: kinetic-friction drop after breakaway (the pending mechanical
lubrication pass changes this), kD on measurement vs. on error, the kI band /
reset behaviour (kI is still 0 and the old "30–45° of hunting" result predates
both the caching fix and the integral band), and 2× servo **overdrive** — which
gives 540° of pod travel, passes `verifyCoverage`, and attacks the 65%
slew-limited settle without the residual penalty that was previously and
wrongly claimed.

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
