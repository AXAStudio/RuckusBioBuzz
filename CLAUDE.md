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
// SwerveDrivetrainConstants — shipped
turnKP = 0.200;  turnKI = 0.0;  turnKD = 0.022;
turnKF = 0.0;    turnKS = 0.035;  turnKSBandDeg = 2.0;  cache = 0.01;
```

- **Do not tune `kF`.** `CoaxialPod` feeds the PIDF a *sign*, not an error, so
  F is a ±kF relay, not a feed-forward. It ships at 0; `kS` replaces it.
- `kS = 0.035` is the measured breakaway command. `0.045` fits today's friction
  better on tiles (|ss| 1.48° vs 2.92°) but falls apart off-ground (13/40 loose
  runs vs 1/40). **0.035 ships because it holds across the whole measured
  friction range.** A mechanical lubrication pass is pending; after it, re-fit
  kP/kD as a small grid, not just kS.
- `cache = 0.01` is the **pod encoder-read cache interval in seconds** — it is
  *not* LynxModule bulk caching.

## 6. Measured baseline (2026-08-13, ~12.2–12.5 V, on FTC tiles)

| Metric | Value |
|---|---|
| Loop, DRIVE, after batteryVolts fix | **30.9 Hz true** (was 19.4–27.4) |
| Loop, DRIVE, bimodal | 29% @ 8.9 ms (publish skipped) / 71% @ 53.6 ms (publish runs) |
| Loop, IDLE | ~130 Hz |
| `msPublish` | **37.4 ms in DRIVE** (13.4 IDLE) ← still unexplained |
| `msTelemetry` / `msHeading` / `msMode` / encoders | 2.05 / 1.81 / 5–6 / 2.6 ms |
| 90° step settle to ±2° | ~647 ms vs a **350 ms** target |
| Rise 10–90% | 0.37 s |
| Steady-state azimuth residual | **2.65–3.01°** vs a **1.0°** criterion (2.9× over) |
| Driving \|azimuth err\| | mean 7.7–10.5°, p95 ~42° |
| Wheel path ÷ commanded path while driving | **1.7–3.0×** (one chunk 2.2×, 97 flips) |
| Wheel reversals / target reversals | 2.58–4.18 /s vs 0.41–0.53 /s |
| At-rest baseline | ~120° of encoder noise, 0.00 reversals/s |
| `DriveTeleOp` true loop rate | **unmeasured** (logged 75.8 Hz is inflated; likely ~40 Hz) |

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
5. **You cannot touch the robot.** Deploying restarts the app, which kills the
   OpMode and the HTTP server. Every deploy needs: build → tell the operator →
   they install and start `Swerve Bring-Up` on the Driver Station → they say
   "restarted" → you resume. **Ask, then wait.**
6. **A reflash or a pose reset clears the safe-area box.** Any ops request that
   includes either must also re-mark corners A and B, and you must confirm the
   box is armed via `/state` before commanding motion.
7. **Always ask which surface the robot is on** — bench/blocks (low friction)
   or FTC tiles (the real operating condition) — and record it with every
   result. Also record battery voltage per trial.
8. **Never command drive without confirming the robot is on the floor.**
9. Ask before changing the diagnostic tooling in a way that changes what is
   being measured.
10. Present options with trade-offs and let the operator choose the order.
11. Commit per finding, with the evidence in the message.

## 9. Build & deploy

```bash
./gradlew :TeamCode:assembleDebug          # the only thing you can do alone
# operator: adb install / Android Studio run  → app restarts
# operator: start the OpMode on the Driver Station before host tools reconnect
./gradlew visualizer                        # Pedro visualizer → http://127.0.0.1:5173/
./gradlew pollenCameraTester                # → http://127.0.0.1:8787/
```