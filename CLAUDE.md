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
  ~30 s at 90 Hz. 14 global columns (`t,dt,volts,loopHz,mode,servoMa,batteryMa,
  heading,htgt,px,py,cf,cs,ct`) plus 7 per pod (`v,wheel,tgt,err,pwr,flip,ctgt`).
  `ctgt` and `wheel` together are the discriminator between "demand is shaking"
  and "response is shaking". **Use `ctgt`, not `tgt`:** `tgt` is
  `computeTargets`, a host-side mirror of the mixer, and it has drifted from the
  mixer before (2026-08-16: 4.0% of samples wrong, worst 15.0°). `ctgt` is read
  out of `CoaxialPod` itself and cannot disagree with what the pod acted on.
- **`DriveTeleOp` publishes to the same `/state` and drives the same recorder**
  (`TeleLoopProbe`, 2026-08-16). The web routes come from a
  `@WebHandlerRegistrar` at app start and serve whatever OpMode published last,
  so `drivecapture.py` works against the competition OpMode too. That is the
  only way to measure the shipped path: bring-up builds its own drivetrain and
  bypasses `CustomDrivetrain.runDrive` entirely.
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
// SwerveDrivetrainConstants — shipped, and it is the PER-POD arrays the
// factories read. turnKP / turnKS are legacy scalars kept for the dashboard's
// divergence guard; nothing builds a pod from them.
turnKPPerPod = {0.380, 0.380, 0.380, 0.380};
turnKDPerPod = {0.022, 0.022, 0.022, 0.022};
turnKSPerPod = {0.022, 0.022, 0.022, 0.022};   turnKSBandDegPerPod = 2.0;
turnKI = 0.0;  turnKF = 0.0;  cache = 0.01;  motorCaching = 0.05;
pod.setPulsedApproach(true, 6.0°, 0.6°, 0.035, 20 ms, 20°/s, 0.10 s);
CoaxialPod.TURN_GAIN_SCHEDULING = true;   // floor 0.24, ramps on drive power
Swerve.epsilonTaper = true;  Swerve.demandSlewDegPerSec = 214;  // 2026-08-16
```

- **Do not tune `kF`.** `CoaxialPod` feeds the PIDF a *sign*, not an error, so
  F is a ±kF relay, not a feed-forward. It ships at 0; `kS` replaces it.
- **The lubrication pass HAPPENED** (before 2026-08-13) and everything above was
  re-fitted after it. This section used to say it was pending — it is not.
  kS fell 0.035 → 0.022 as a result: lubrication cut kinetic friction, so a kS
  sized to static breakaway now overdrives a moving pod.
- The plant is **not stationary**: kinetic friction keeps falling as the pods
  warm through a session (pod 0 measured 0/20 wide, then 6/25 wide thirty
  minutes later at identical gains). Gains lean conservative deliberately — too
  little feed-forward parks a cold pod short, too much feeds the wide mode.
- `cache = 0.01` is the **pod servo-output caching threshold** — the minimum
  change in servo power before `CoaxialPod` writes it. It is *not* LynxModule
  bulk caching, and it is not an encoder-read interval either (that older
  description was wrong).

## 6. Measured baseline

Re-measured 2026-08-16 from `mydrive-001` (71.9 s of human driving through the
dashboard, FTC tiles, 12.37 V) and from live `/state` reads. **Several 2026-08-13
rows below were wrong, not merely stale** — see §7.

| Metric | Value |
|---|---|
| Loop, DRIVE, human driving through the dashboard | **22.5–24.1 Hz true**, dt mean 41.6–44.5 ms, p90 64–66, p99 77–79 |
| Loop, DRIVE, scripted box drive (same code, trace shorter) | **51.1 Hz true**, dt p50 12.1 ms, p90 52.9 |
| `msPublish`, **robot at rest, no actuator writes at all** | **36.7 ms** (32.3–39.6, n=60) |
| `msEncoders` / `msHeading`+idle sensors / `msTelemetry` / `msMode` | 2.00 / 5.42 / 0.65 / 0.19 ms |
| Steady-state azimuth residual | **2.65–3.01°** vs a **1.0°** criterion (2.9× over) — unchanged, cause still unknown |
| Driving \|azimuth err\| | mean 16.7–21.5°, p95 57–66° |
| Wheel path ÷ **commanded** path while driving | **0.89–1.01×** — the pods track faithfully |
| Wheel reversals /s | 4.17–4.91 |
| **Commanded** reversals /s | **3.47–4.17** — the demand itself is what shakes |
| Azimuth setpoint jumps > 15° while driving | 2.07–2.71 /s, p90 14.4–17.8°, max 149–179° |
| Setpoints within 5° of a 45° multiple | **55–57%** against 22% if uniform (~20× spike in the histogram) |
| Flip events /s | 0.47–0.70 vs a 0.2 /s criterion |
| Heading \|err\| translating / at rest (bring-up hold only) | 7.83° / 3.65° mean; p95 saturates at the 60° lead cap |
| `DriveTeleOp` true loop rate | **still unmeasured** — instrumented 2026-08-16, awaiting a run |

**The loop is bistable, and that is the whole loop story.** Publish runs off a
50 ms timer, so once the loop period exceeds 50 ms *every* loop pays the publish
cost and the slow mode sustains itself. Heading hold fills the 260-sample trace,
which adds ~780 `String.format` calls per publish, which is what pushes it over.

## 7. Hypotheses already tested — do not re-run these

| Hypothesis | Verdict |
|---|---|
| Loop rate sets the azimuth residual | **REFUTED.** 47.8 vs 92.1 Hz, n=40 pod-runs/arm, randomized interleaved: Δ|ss| +0.12° [−0.51, +0.71], p=0.70. Every metric p ≥ 0.39. |
| "Creep quantum" — residual = one control period of pod travel | **REFUTED.** Per-update travel 4.5° → 2.3°, residual unchanged. The 20 Hz coincidence was a coincidence. |
| `atan2(py, px)` with no magnitude gate causes the shake | **CONFIRMED as a contributor, 2026-08-16 — this row used to say UNLIKELY and it was wrong.** The targets are not smooth: they reverse 3.47–4.17 /s. At low translation the demand direction is `atan2(translation + rotation)`, so its sensitivity to the rotation term is ≈ trans/(trans²+rot²) — one logged pair of consecutive loops moved rotation by 0.08 with translation unchanged and swung the demand **67.4°**. 21% of large jumps are this. |
| "the closed loop is hunting" | **REFUTED, 2026-08-16.** Wheel path ÷ **commanded** path is 0.89–1.01. The pods follow the demand faithfully; the demand is what shakes. Every conclusion that assumed a hunting pod loop needs re-reading. |
| `batteryVolts()` inline in `publish()` is the loop-time cost | **PARTLY.** Fixing it gave 19.4–27.4 → 30.9 Hz. |
| publish() is slow because it blocks on the Lynx bus behind the actuator writes | **REFUTED, 2026-08-16.** Sampled with the robot at REST in DRIVE — pods X-locked, zero servo and motor writes — publish still cost 36.7 ms (32.3–39.6, n=60). It is CPU. The `getPower()` fix removed the bus component; what is left is ~1000 `String.format` calls per publish, ~780 of them from the trace. |
| 2× servo gear reduction | **Fails coverage** (270° → 135° pod, flip needs 180°). 2× *overdrive* gives 540° and is still open. |
| "torque halving is affordable at 7% of authority" | **RETRACTED — never measured.** CRServo command is a speed setpoint, not a torque fraction. |
| The 45° snapping is a dashboard artifact | **PARTLY, and the dashboard is not the only culprit.** Per-axis deadbanding is in BOTH paths (dashboard 0.06, `DriveTeleOp` 0.05) and rotates the commanded direction rather than shortening it. But 42% of the jumps are the mixer's rotation-epsilon wall and 19% are X-lock, and both are in the shipped path. Fixed 2026-08-16. |
| The field box clamp causes the snapping | **NOT the cause in the archive** — 0 clamped samples in `mydrive-001`. The mechanism is real though: `applyBoxLimit` zeroed a field axis as a step. Tapered 2026-08-16. |

**Still open:** what sets the 2.7° azimuth residual (loop rate and "creep
quantum" are both eliminated — do not re-run them); whether `DriveTeleOp`'s own
loop is fast enough (instrumented, not yet run); whether the `String.format`
theory of `publish()` is right (instrumented with per-section timers and a
runtime `setFastFmt` A/B, not yet run).

**Criterion 1 (≤15° setpoint change per loop) is a loop-rate criterion.** A
demand slew limit is a *rate*, so a slow loop turns any rate into a big step: in
simulation over `mydrive-001`, 53% of the jumps that survive a 214 °/s limit
happen on loops longer than 70 ms. Fix the loop before judging the mixer.

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