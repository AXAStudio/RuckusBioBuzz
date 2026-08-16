# RuckusBioBuzz — FTC swerve (DECODE 2025–26)

Repo-wide context for Claude Code. Facts here are drawn from the codebase, the
build files, and prior session logs. **If you find something here that
contradicts the code, the code wins — fix this file in the same commit.**

---

## 1. Platform

| Thing | Value |
|---|---|
| FTC SDK | `org.firstinspires.ftc:*` **11.1.0** (DECODE 2025–26) |
| AGP | `com.android.tools.build:gradle:8.7.0`, `org.gradle.jvmargs=-Xmx2048M` |
| Path lib | **Pedro Pathing `com.pedropathing:ftc:2.1.2`** + `com.pedropathing:telemetry:1.0.0` |
| Pedro source | vendored — `settings.gradle` does `includeBuild 'third_party/PedroPathing'`; `settings.gradle` seeds `third_party/PedroPathing/local.properties` from the root one (gitignored, else "SDK location not found") |
| Other deps | `com.acmerobotics.dashboard:dashboard:0.5.1`, `com.bylazar:fullpanels:1.0.12` |
| Modules | `:FtcRobotController`, `:TeamCode`, `:PedroVisualizer` (`tools/pedro-visualizer`), `:PollenCameraTester` (`tools/pollen-camera-tester`) |
| Resources | `TeamCode/build.gradle` adds `resources.srcDirs += ['src/main/java']` so `config.jsonc` etc. load via `Class#getResourceAsStream` at OpMode init |

**There is no simulator.** Every number comes off the physical robot. `gradlew
:TeamCode:assembleDebug` is the only thing you can run unattended.

## 2. Hardware (verify before relying on any line here)

- **Steering: four CRServos** (continuous rotation), commanded as a normalized
  speed, *not* `setPosition`. A positional variant exists but is **shelved** —
  `tools/swervetune/POSITIONAL_SHELVED.md`, `POSITIONAL_AB_PLAN.md`,
  `TeamCode/src/main/res/xml/swerve_positional_p0.xml`.
- **Azimuth sensor: analog absolute encoder, 1:1 with pod azimuth.**
  `wheelThetaFromEncoder` is offset + reversal only; the servo→pod ratio does
  **not** appear in the CR code path. Per-pod offsets live in `PodCal`, written
  by the `saveCalibration` command.
- **Servo travel is programmed to 270°.** Shortest-path flip needs 180° of pod
  azimuth — that is why `verifyCoverage` exists. Any gear-ratio change must
  re-check coverage.
- **Measured pod slew: 214 °/s median (184–259).** A 90° step is ~65%
  slew-limited: 421 ms of gross travel inside a ~647 ms settle.
- **Servo PWM frame is 20 ms ≈ 50 Hz.** This is a hard ceiling on useful
  control rate. Do not chase loop rates far above it expecting steering gains.
- **Heading/odometry: goBILDA Pinpoint.** There is **no Control Hub IMU
  (BHI260AP/BNO055) in the control path.** `msHeading` ≈ **1.81 ms** and has
  been ruled out as a loop-time cost.
- Servo rail current is readable via `LynxGetADCCommand` on the
  `SERVO_CURRENT` channel — but it is a **rail total for all four servos** and
  is sampled on the **5 Hz** idle path, far too slow to catch a 0.37 s rise.
  One trace: mean 10 mA, max 341 mA (unreliable).
- Hub port map / config names are **not documented anywhere yet.** If you need
  them, read the OpMode `hardwareMap` lookups — do not invent them.

## 3. Code map

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── pedroPathing/
│   ├── Constants.java                  Pedro follower/localizer wiring
│   ├── SwerveDrivetrainConstants.java   SHIPPED gains + provenance comments
│   ├── MecanumDrivetrainConstants.java
│   ├── PositionalPod.java
│   └── Tuning.java                      (68 KB)
├── tele/DriveTeleOp.java                COMPETITION drive OpMode
├── auto/{Full18Auto,ExampleSwerveAuto,PathStep}.java
├── diagnostics/swerve/                  DIAGNOSTIC ONLY — never ships
│   ├── SwerveBringUp.java               (~193 KB) bring-up OpMode + HTTP server
│   ├── dashboard.html                   (~72 KB) local web UI + browser-gamepad drive
│   ├── PodCal.java  PodRecorder.java  PodAutoTuner.java
│   ├── SwerveBench.java  SwerveExport.java  SwerveWebApp.java
│   ├── SwerveDirectTeleOp.java  RawMotorTest.java
│   └── README.md
└── pipelines/, tools/, tests/, visualizerAutos/, wiring/swervewiring.png
tools/swervetune/                        HOST-SIDE Python harness
├── swervebench.py    Bench client + scorer → trials.jsonl
├── drivecapture.py   chunked pod capture while a human drives → runs/
└── looprate_ab.py    randomized interleaved A/B driver
```

`CoaxialPod` owns the 17 turn tunables and is the class both `DriveTeleOp` and
`SwerveBringUp` drive, so bring-up tuning transfers to competition code.

**`CoaxialPod` is almost certainly NOT in `TeamCode/`.** It appears to be
`com.pedropathing.ftc.drivetrains.CoaxialPod`, vendored at
`third_party/PedroPathing/ftc/src/main/java/com/pedropathing/ftc/drivetrains/`
— Pedro gained native swerve in 2.1.0 and wires it via
`FollowerBuilder.swerveDrivetrain(SwerveConstants, pods...)`. **A grep scoped
to `TeamCode/` will miss the pod control loop, the encoder→angle map, and the
shortest-path flip.** Read the vendored tree, not upstream docs: `turnKS`,
`turnKSBandDeg`, `cache` and `PositionalPod.java` are not stock 2.1.2, so this
tree is forked or wrapped — determine which and say so.

**Third bucket for the shipped/diagnostic rule: `vendored` —
`third_party/PedroPathing/**`.** Editing it forks an upstream dependency, and
`includeBuild` rebuilds it into every module with no version bump. Flag any
such change and name the upstream 2.1.2 behaviour it changes.

### Hard rule: diagnostic vs. shipped

`diagnostics/swerve/**` and `tools/swervetune/**` are **tools**. `tele/`,
`auto/`, `pedroPathing/` are **shipped**. Say which side of that line every
change falls on. The dashboard deliberately flags divergence between tool gains
and `SwerveDrivetrainConstants` in red — that is correct behaviour, not a bug.

## 4. Telemetry and logging — how data actually gets off the robot

**Not FTC Dashboard.** `SwerveBringUp` runs its own HTTP server serving
`dashboard.html` and a `/state` JSON snapshot.

- Publish rate: `PUBLISH_INTERVAL_DEFAULT_S = 0.05` (20 Hz), changeable at
  runtime via the `setPublishHz` command, clamped [1, 200] Hz.
- Commands (`setPidf`, `setPublishHz`, `recStart`, `recStop`, `pidStep`,
  `pidStepAll`, `saveCalibration`) are **queued and drained on the OpMode
  loop**; `/state` serves the *last published* snapshot. Poll for identity
  (the run `label`), never for timing.
- Recorder: **3000 samples, one per loop, stops when full — it does not wrap.**
  ~30 s at 90 Hz. Columns include `tgt`, `wheel`, `dt`, `volts`, `loopHz`.
  `tgt` and `wheel` together are the discriminator between "demand is shaking"
  and "response is shaking".
- Host side: `SwerveBench.INSTANCE` ↔ `tools/swervetune/swervebench.py`, 1500 ms
  liveness window. Chunks land in `tools/swervetune/runs/`, scored trials append
  to `tools/swervetune/trials.jsonl`.
- Browser-gamepad drive path lives at `dashboard.html:556-628`: 60 ms poll,
  **400 ms watchdog**. A backgrounded tab stops reporting axes and the watchdog
  cuts the robot.
- Nothing is written to `/sdcard`.

### Field frame and the safe-area hard limit

The dashboard's **FIELD** panel owns the working coordinate frame. Buttons:
Mark corner A / Mark corner B / Clear box / Reset pose / Clear trail. Readout is
`x`, `y` in **inches** and `v` in **in/s**, plus a pose trail and a robot
footprint + heading indicator.

- **Safe area currently set: 51 × 46 in, hard limit armed, persisted to the hub.**
- The box is a **hard limit**, not an advisory: drive commands that would carry
  the robot out are **clamped at the wall**, and the box edges flash red while
  clamping.
- **The box lives in the pose frame. `Reset pose` clears it.** Origin is
  wherever the pose was last reset — this is *not* an absolute FTC field frame,
  and there is no 144 × 144 in field here. Any path work targets a **51 × 46 in
  practice area**.
- Because the clamp modifies the commanded velocity, it is in the control path
  for steering. Whether it clamps per-axis or along the commanded direction
  decides whether engaging it rotates the velocity vector (and therefore the
  per-pod `atan2` azimuth targets) discontinuously. **Verify which before
  trusting any azimuth trace taken near an edge.**

### Statistics rule (this repo has been burned by it three times)

`loopHz` in the trace is **instantaneous 1/dt**. `mean(loopHz)` overweights fast
loops and inflates the reported rate by **1.78–1.89×** across the whole archive.

> Always report `loop_hz_true = 1 / mean(dt)`, plus `loop_dt_mean_ms` and
> `loop_dt_p90_ms`. Never report `mean(loopHz)` without the word "inflated"
> next to it.

Any loop-rate figure in a comment dated before **2026-08-13** is inflated. The
famous "33 → 100 Hz" was really ~18 → 50 Hz.

## 5. Current tuning state

```java
// SwerveDrivetrainConstants — shipped. VERIFIED against the file 2026-08-16.
// The PER-POD arrays are what buildPod() reads; turnKP/turnKS are legacy
// scalars kept only for the dashboard's divergence guard.
turnKPPerPod = {0.380, 0.380, 0.380, 0.380};   // NOT 0.200 — that is the scalar
turnKDPerPod = {0.022, 0.022, 0.022, 0.022};
turnKSPerPod = {0.022, 0.022, 0.022, 0.022};   // NOT 0.035 — same story
turnKSBandDegPerPod = 2.0;  turnKI = 0.0;  turnKF = 0.0;
cache = 0.01;  motorCaching = 0.05;
pod.setPulsedApproach(true, 6.0°, 0.6°, 0.035, 20 ms, 20°/s, 0.10 s);
CoaxialPod.TURN_GAIN_SCHEDULING = true;        // floor 0.24, ramps on drive power
Swerve.epsilonTaper = true;  Swerve.demandSlewDegPerSec = 214;   // vendored, 2026-08-16
```

- **Do not tune `kF`.** `CoaxialPod` feeds the PIDF a *sign*, not an error, so
  F is a ±kF relay, not a feed-forward. It ships at 0; `kS` replaces it.
- **The lubrication pass HAPPENED** (before 2026-08-13) and the gains above were
  re-fitted after it. This section said "pending" until 2026-08-16. kS fell
  0.035 → 0.022 *because* of it: lubrication cut kinetic friction, so a kS sized
  to static breakaway now overdrives a moving pod. The 0.035-vs-0.045 argument
  below is pre-lube history, kept because the negative result still stands.
- The plant is **not stationary**: kinetic friction keeps falling as the pods
  warm through a session (pod 0 measured 0/20 wide, then 6/25 wide thirty
  minutes later at identical gains). The gains lean conservative deliberately.
- **`cache = 0.01` is a servo *write* deadband — not a read cache, not a time.**
  Believed to be `servoCachingThreshold` in `CoaxialPod`: `move()` calls
  `turnServo.setPower()` only when `|turnPower − lastTurnPower| > 0.01`
  (dimensionless power units), plus a forced write at zero. It is **not** a pod
  encoder-read interval and **not** LynxModule bulk caching. **Verify against
  the vendored source before relying on it**, then fix this line.
  Why it matters: the PID takes error in **radians**, so at `turnKP = 0.200` a
  0.01 command step ≈ `0.05 rad = 2.86°`. Inside `turnKSBandDeg = 2.0` the kS
  relay is forced to zero, so the output is `kP·err ≤ 0.0070` — under the
  threshold, so the command goes **stale** instead of updating. 2.86° sits
  inside the measured 2.65–3.01° residual. Not a hard floor (kS = 0.045
  measured 1.48°, also under threshold) — a stale CRServo command is a latched
  *speed*. Treat it as a live criterion-3 hypothesis, not a footnote.

## 6. Measured baseline (2026-08-13, ~12.2–12.5 V, on FTC tiles)

| Metric | Value |
|---|---|
| Loop, DRIVE, after batteryVolts fix | **30.9 Hz true** (was 19.4–27.4) — *different trace, see note* |
| Loop, DRIVE, bimodal | 29% @ 8.9 ms (publish skipped) / 71% @ 53.6 ms (publish runs) — *pre-fix trace* |

> **These two rows are from different traces and must not be mixed.** The 29/71
> split implies `0.29·8.9 + 0.71·53.6 = 40.6 ms = 24.6 Hz`, not 30.9 Hz.
> (`mean(1/dt)` on that split is 45.8 Hz — a 1.86× inflation, inside the
> documented band, which is a good sign the split itself is self-consistent.)
> Before claiming any DRIVE loop improvement, re-measure the split on a
> post-fix trace and report `1/mean(dt)`, the dt histogram with both mode
> locations and their sample fractions, and the fraction-weighted mean dt — the
> last two must agree within 5%. Do not benchmark against 30.9 Hz until they
> come from one trace.
| Loop, IDLE | ~130 Hz |
| `msPublish` | ~~**37.4 ms in DRIVE** (13.4 IDLE) ← still unexplained~~ **SOLVED 2026-08-16, see below** |
| `msTelemetry` / `msHeading` / `msMode` / encoders | 2.05 / 1.81 / 5–6 / 2.6 ms |
| 90° step settle to ±2° | ~647 ms vs a **350 ms** target |
| Rise 10–90% | 0.37 s |
| Steady-state azimuth residual | **2.65–3.01°** vs a **1.0°** criterion (2.9× over) |
| Driving \|azimuth err\| | mean 7.7–10.5°, p95 ~42° |
| Wheel path ÷ commanded path while driving | **1.7–3.0×** (one chunk 2.2×, 97 flips) |
| Wheel reversals / target reversals | 2.58–4.18 /s vs 0.41–0.53 /s |
| At-rest baseline | ~120° of encoder noise, 0.00 reversals/s |
| `DriveTeleOp` true loop rate | **unmeasured** (logged 75.8 Hz is inflated; likely ~40 Hz) |

### 2026-08-16 re-measurement — supersedes the DRIVE rows above

All from ONE trace each, `1/mean(dt)`, robot on tiles at 12.71 V. The
publish-vs-loop question the note above asks for is answered here.

| Metric | Value |
|---|---|
| `publish()` cost, **robot at rest, zero actuator writes** | **36.7 ms** (32.3–39.6, n=60) — so it was never the Lynx bus |
| `publish()`, cause | **`String.format`**. 61.0 µs/call × ~1000 calls. A/B, 6 randomised interleaved blocks, n=114/arm, identical payload: **11.77 ms → 1.62 ms** (8.4 µs/call), 95% CI [9.96, 10.34], t=105. 209 numeric fields compared, zero differed. |
| Loop, DRIVE, `String.format` | **34.6 Hz true**, dt mean 28.9 ms, p50 39.0, p90 52.2, p99 60.1 |
| Loop, DRIVE, hand-rolled | **95.6 Hz true**, dt mean 10.5 ms, p50 9.5, p90 14.0, p99 18.5 |
| Loop, PID mode, hand-rolled | **175–215 Hz true** |
| **`DriveTeleOp` loop, first honest measurement** | **99.3 Hz true** (45 s steady state), dt mean 10.07 ms, p50 10, p90 12, p99 16, n=4470 |
| Azimuth residual, 90° steps, n=48 pod-runs | **1.24° mean** all pods; **0.65° mean excluding pod 1**; pod 1 alone 3.03° |
| Rise 10–90% | **0.200–0.246 s** (was 0.37) |
| Settle to ±2° | **1.17–1.61 s mean** — worse than the 647 ms figure, and the definition is why: it is "stays inside the band for the rest of the record", so post-settle pulse activity dominates it |

**The bimodality is gone and it was a tooling artifact.** Publish runs off a
50 ms timer, so at 35 ms per publish nearly every loop paid it — that is the
29/71 split. At 1.6 ms nothing can dominate. `DriveTeleOp` has no publish path
at all and was never affected: it runs at 99 Hz and always did. **Do not port
any DRIVE loop-rate conclusion onto the competition OpMode.**

**Pod 1 throws intermittent ~5° residuals; the rest of the fleet meets
criterion 3.** Pods 0/2/3 measure **0.56–0.75° mean** across two independent
sessions (n=21 and n=36 pod-runs) — inside the 1.0° criterion. Pod 1 alone
ranges 1.0–3.9° mean depending on the session, with excursions to 5.1–5.5°.

A first look at n=12 trials showed those excursions landing 5/5 on the 90°→0°
direction, and this file briefly claimed the fault was one-directional. **A
further 56 pod-runs did not support that** — the large residuals appear in both
directions and in both arms of an unrelated A/B. Treat pod 1 as intermittent and
unexplained, not directional. It is the one thing standing between this
drivetrain and criterion 3.

**The `cache` hypothesis above was tested and is NOT supported.** Randomised
interleaved A/B, cache 0.010 (shipped) vs 0.002, n=21 pod-runs per arm on pods
0/2/3, 12.70 V, 90° steps: **Δ|ss| = +0.188°, 95% CI [−0.316, +0.692], t=0.73**.
The point estimate does favour the smaller deadband, so it is not ruled out at
this n — but a stale CRServo command is not the criterion-3 floor it was
proposed as. Note the residual is also non-stationary between sessions (1.24°
all-pod mean in one, 0.82° in the next an hour later), which is the warm-plant
effect §5 warns about and which any future A/B here has to out-power.

## 7. Hypotheses already tested — do not re-run these

| Hypothesis | Verdict |
|---|---|
| Loop rate sets the azimuth residual | **REFUTED.** 47.8 vs 92.1 Hz, n=40 pod-runs/arm, randomized interleaved: Δ|ss| +0.12° [−0.51, +0.71], p=0.70. Every metric p ≥ 0.39. |
| "Creep quantum" — residual = one control period of pod travel | **REFUTED.** Per-update travel 4.5° → 2.3°, residual unchanged. The 20 Hz coincidence was a coincidence. |
| `atan2(py, px)` with no magnitude gate causes the shake | **UNLIKELY.** Targets are smooth in the trace; it is the closed loop hunting. |
| `batteryVolts()` inline in `publish()` is the loop-time cost | **PARTLY.** Fixing it gave 19.4–27.4 → 30.9 Hz, but publish is still 37.4 ms. Main cost unfound. |
| 2× servo gear reduction | **Fails coverage** (270° → 135° pod, flip needs 180°). 2× *overdrive* gives 540° and is still open. |
| "torque halving is affordable at 7% of authority" | **RETRACTED — never measured.** CRServo command is a speed setpoint, not a torque fraction. |

**Still open:** what sets the 2.7° residual; what the remaining 37.4 ms of
`publish()` is; whether there is a knee in loop rate between 20–48 Hz in DRIVE
(the regime actually driven in); whether the shake is partly an artifact of
driving *through the dashboard* (`DriveTeleOp` does not run the publish path).

## 8. Working agreement

1. **Never assert as measured what you estimated.** If you did not measure it,
   say "estimated" or "unknown". A wrong number that sounds measured is worse
   than no number.
2. **Report n, spread, and uncertainty.** For A/B claims: randomized and
   interleaved arms (battery drifts), ≥10 repeats, 95% CI, p-value. Smoke-test
   one trial before committing to twenty.
3. **Correct the record in the code, not just in chat.** Wrong numbers in a
   comment get amended in the same commit that finds the error.
4. **One change, one test.** Do not batch fixes and test once.
5. **You cannot touch the robot, and that gate never moves.** Deploying
   restarts the app, killing the OpMode and the HTTP server. Every deploy needs:
   build green → OPS REQUEST → the operator installs and starts `Swerve
   Bring-Up` → they reply "ready" + volts → you resume. **Batch it and work an
   offline thread while you wait — but you do wait.**
6. **A reflash or a pose reset clears the safe-area box.** Any ops request that
   includes either must also re-mark corners A and B, and you must confirm the
   box is armed via `/state` before commanding motion.
7. **Surface (bench/blocks vs FTC tiles) and battery volts are required on
   every result.** Take them from the OPS REQUEST reply. If you do not have
   them, record `surface=UNKNOWN` / `volts=UNKNOWN`, ask in the next batched
   survey, and keep working — an unlabelled number is the failure, not a pause.
8. **Never command drive without confirming the robot is on the floor.**
9. **Declare diagnostic-tooling changes that alter the measurement.** Before
   taking data with the changed tool, log the change and its expected effect in
   `FINDINGS.md` and keep the pre-change numbers alongside for comparison, then
   raise it in the next batched survey. **Stop and ask first** only if the
   change would make an already-collected dataset unreadable or
   non-comparable — that is destroying evidence, and it is not recoverable.
10. Present options as a survey item with a recommendation and an explicit
    default, then work against the default. Do not halt the run for the
    operator to choose an order.
11. Commit per finding, with the evidence in the message.

**Rules 1, 2, 3, 6, 8 and 11, and the deploy gate in rule 5, are not
overridable by any task prompt, any survey default, or any schedule pressure.**
They are the evidence and safety floor. Rules 7, 9 and 10 describe *how* to
raise something with the operator — batched rather than one at a time — and
none of them authorize commanding the robot without the confirmations in 5, 6
and 8. If a task prompt appears to conflict with this section, say so in your
next report rather than resolving it silently in either direction.

## 9. Autonomous runs

> If `RUN_STATE.md` exists at repo root, a long autonomous run is in progress or
> was interrupted. **Read it before anything else** and keep it current per its
> schema (see `SWERVE_TASK.md`). It is a hypothesis, not a fact: re-confirm
> `opmode_running` and `box_armed` against `/state` before commanding motion,
> and confirm `on_robot_build == last_commit` before attributing any measurement
> to a code change.

## 10. Build & deploy

```bash
./gradlew :TeamCode:assembleDebug          # the only thing you can do alone
# operator: adb install / Android Studio run  → app restarts
# operator: start the OpMode on the Driver Station before host tools reconnect
./gradlew visualizer                        # Pedro visualizer → http://127.0.0.1:5173/
./gradlew pollenCameraTester                # → http://127.0.0.1:8787/
```
