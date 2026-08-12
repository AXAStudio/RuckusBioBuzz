# Ruckus PedroPathing Patch Notes

This folder is vendored into the main `RuckusBioBuzz` git repository. It is not a nested git checkout or submodule.

## Upstream

Official source:

```bash
https://github.com/Pedro-Pathing/PedroPathing.git
```

The main repository should have a remote named `pedro-upstream`:

```bash
git remote add pedro-upstream https://github.com/Pedro-Pathing/PedroPathing.git
git remote set-url --push pedro-upstream DISABLED
```

The vendored Gradle metadata currently reports PedroPathing version `2.1.2`.

## Local Patches

### 2026-08-12 — Derivative on measurement, and a pulsed final approach, on `CoaxialPod`

- **Files changed:** `ftc/src/main/java/com/pedropathing/ftc/drivetrains/CoaxialPod.java`,
  `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** two attempts at the residual that a continuous loop cannot reach. Both are
  implemented, both default to off, and **both are currently negative results** — kept because they
  are cheap to re-test and expensive to re-derive.

  **`setDerivativeOnMeasurement(boolean)`** plus `PIDFController.updateErrorWithDerivative`.
  Differencing the error assumes a stationary setpoint; a pod's steps by up to 90° and jumps
  discontinuously at the ±180° flip, and at 120 Hz a 90° step alone drives `kD = 0.010` to 1.96 of
  output, saturating the clamp. Derivative-on-measurement removes that kick and is identical while
  the setpoint holds. It did **not** permit more damping: kD 0.040 rings 16×, 0.070 and 0.110 are
  unusable. The reason is structural — for a plant `1/(s(1+sT))` the PD zero cancels the lag pole at
  `kD = kP·T`, so with the measured T = 42 ms the useful range is already covered. A randomised grid
  confirmed the optimum sits at 1.7–2.4× that value, above it rather than below, because the 39 ms
  of *pure transport delay* needs additional lead that pole cancellation does not account for.

  **`setPulsedApproach(...)`.** The pod has no creep regime: static friction breaks to kinetic and it
  goes from stopped to tens of degrees per second with nothing between, so any continuous controller
  with the authority to correct a small error commits to several degrees of travel before feedback
  (39 ms transport + 42 ms velocity lag) can act. A bounded open-loop pulse sidesteps the stability
  question entirely. Where it works it is decisive — 11 of 16 pod-runs parked at ≤0.34° with
  peak-to-peak ≤0.34° and *exactly zero* holding power. It is blocked by the pulse quantum: at the
  SDK's default 20 ms servo PWM frame the shortest possible pulse travels 0.06–2.85° at 0.040 power,
  already more than the whole 1.0° error budget, and travel is not a repeatable function of duration
  (pod 0 moves 2.85° at 20 ms and 1.80° at 45 ms), so the scatter is intrinsic breakaway variability
  rather than something a calibration can remove. Halving the frame period halves the quantum but
  not necessarily the scatter, which is the thing to judge a retest on.

- **Upstream issue:** none filed.
- **How to remove:** search for `RUCKUS PATCH`. Both are inert at their defaults, so removal is
  deleting the setters, the two `if` blocks in `move()` and `updateErrorWithDerivative`.

### 2026-08-12 — Continuous static-friction term and instrumentation on `CoaxialPod`

- **Files changed:** `ftc/src/main/java/com/pedropathing/ftc/drivetrains/CoaxialPod.java`
- **Reason:** `kF` is not a feed-forward on a swerve pod. `move()` feeds
  `turnPID.updateFeedForwardInput(getTurnDirection(...))`, which is exactly ±1, so the F term is
  `±kF` — a relay, constant in magnitude and switching sign at the target, zeroed inside 2°.

  Measured on this drivetrain (robot on blocks, 12.9–13.3 V, `pidStepAll` 90°, n≥12 pod-runs each):

  - Open-loop breakaway power is **0.025–0.050** depending on pod and direction (measured by
    applying each power from rest on a staircase; see `tools/swervetune/phase2_plant.py`).
  - Shipped `kF = 0.005` is therefore **5–10× below breakaway** — the relay does nothing at all,
    and the pod parks where `kP·e` falls under breakaway, i.e. `0.035/0.300 = 0.117 rad ≈ 6.7°`.
    Measured residual was 3–6°, matching.
  - Raising `kF` to 0.035 (≈ breakaway) *does* fix the residual — |steady-state| fell 3.52° → 1.25°
    — but produced a mean of **15.8 error sign changes per step** and a **42.5° post-settle
    peak-to-peak**. That is the relay limit-cycling, and it is why the previous session's table
    records rings rising monotonically with kF (0.005→2.7, 0.011→4.3, 0.022→7.7, 0.035→13.7).

  So the fix is not more kF, it is a term with the same authority away from the target and no
  switching discontinuity at it. `setStaticFriction(power, bandRadians)` adds
  `power · tanh(error / band)` to the controller output. With `kS = 0.035`, `band = 2°`,
  `kP = 0.20`, `kD = 0.020`, `kF = 0`: |steady-state| **0.51° mean**, post-settle peak-to-peak
  0.34°, resting servo power RMS 0.0115 — below breakaway, so the pod is genuinely at rest.

  Also added, instrumentation only with no control effect: `getLastTurnPower()`,
  `getLastErrorRad()` and `wasLastMoveFlipped()`. Scoring a step needs the error the PID actually
  acted on — after the ±180° flip is resolved — and the power actually *written*, which output
  caching makes different from the controller's output. Reconstructing either outside the pod means
  duplicating `move()`'s wrap and flip logic and watching it drift.

  A no-op at `staticFrictionPower = 0`, which is the default and every stock configuration.

- **Upstream issue:** none filed. Worth proposing: the sign-only feed-forward is a genuine design
  problem for any friction-dominated servo axis, not just this robot.
- **How to remove:** if upstream adds a real static-friction term, drop `setStaticFriction` and use
  theirs. Search the file for `RUCKUS PATCH`. After removing, re-measure breakaway and confirm the
  replacement is continuous through zero error — a relay will limit-cycle here.

### 2026-08-12 — Make the integral term actually usable in `PIDFController`

- **Files changed:** `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** the 2026-08-11 patch below clears `errorIntegral` on any error sign change. Near the
  target the error sign flips on sensor noise every few loops, so the accumulator was being cleared
  continuously in exactly the situation it was added for. The measured encoder noise floor here is
  **σ = 0.042–0.054°** against a 0.1125° read granularity, so zero crossings at rest are constant.

  Two additions, both no-ops at their defaults:

  1. `setIntegralResetThreshold(t)` — a sign change only clears the accumulator if the error was
     larger than `t` on one side of the crossing. Default 0 reproduces the previous behaviour.
  2. `setIntegralBand(b)` — only accumulate while `|error| < b`, holding the accumulator at zero
     outside. Conditional integration: an integral meant to break stiction at the target should not
     wind up across a 90° slew where the proportional term already saturates the output. Default
     infinity reproduces the previous behaviour.

  Both are reachable on a pod through `CoaxialPod.setTurnIntegralSettings(...)`, added for this,
  since the controller is constructed inside that class.

- **Upstream issue:** none filed.
- **How to remove:** search for `RUCKUS PATCH`; the shared `accumulateIntegral()` helper contains
  all of the changed behaviour and reverts to the stock two lines at default settings.

### 2026-08-11 — Make the integral term safe to use in `PIDFController`

- **Files changed:** `core/src/main/java/com/pedropathing/control/PIDFController.java`
- **Reason:** The swerve pods park 2–4° short of target on carpet and stop dead. `CoaxialPod.move()`
  zeroes the feed-forward inside 2°, and P alone there is far below breakaway, so nothing remains to
  close the gap. Raising kF makes it worse — the pod breaks free, overshoots and stops on the far
  side. An integral term is the correct fix, but stock `PIDFController` could not carry one safely:
  `errorIntegral` accumulates without bound, is never clamped, and nothing calls `reset()` during
  normal operation, so any non-zero I eventually saturates the servo.

  Two changes, both no-ops while I is 0 — which is every stock Pedro config:

  1. `run()` clamps the integral **contribution** to `integralLimit` (default 0.25 of normalised
     output). Clamping the contribution rather than the raw sum keeps the bound meaningful for any I.
  2. `updateError()` and `updatePosition()` zero `errorIntegral` when the error changes sign.
     Accumulation from the approach is stale once the target is crossed, and carrying it through a
     swerve pod's 180° flip would drive hard the wrong way.

- **Upstream issue:** none filed. Arguably worth proposing upstream — unbounded integral with no
  reset path is a latent trap for anyone who sets I.
- **How to remove:** if upstream adds its own anti-windup, drop both patches and use theirs. Search
  the file for `RUCKUS PATCH`. Verify afterwards that a pod with I set does not creep or saturate
  when held off-target.

### 2026-08-09 — Remove Dokka from `:core` so its classes reach the APK

- **Files changed:** `core/build.gradle.kts`
- **Reason:** With Dokka applied, `:core` exposes consumable configurations whose attributes are
  `DGP~`-prefixed strings rather than Gradle's typed attributes. When the Android app consumed
  `:core` transitively (via `ftc`'s `api(project(":core"))`) across the composite build, Gradle
  selected the variant `dokkaHtmlPublicationPluginApiOnlyConsumable~internal` instead of
  `runtimeElements`. That variant carries the Dokka documentation-plugin classpath, not the code
  jar, so **no `com.pedropathing` core class was packaged into the APK** — verified with `dexdump`:
  `follower`, `geometry`, `control`, `math` were all absent while `com.pedropathing.ftc.*` was
  present.

  The symptom was a Robot Controller crash loop: Panels' Configurables plugin scans `@Configurable`
  classes at startup, and `getDeclaredFields()` on `Tuning` threw
  `NoClassDefFoundError: Lcom/pedropathing/follower/Follower;`, taking down the whole app process
  every ~40 seconds. That killed FTC Dashboard connections and made every Pedro OpMode unusable.

  Removed the `org.jetbrains.dokka` plugin, its `dokkaPlugin` dependency, the `dokkaJar` task and
  the `docs(dokkaJar)` deployer entry. Generated documentation has no value for a vendored copy.

- **Upstream issue:** none filed; this is a Dokka Gradle Plugin v2 variant-selection interaction
  with composite builds, not a PedroPathing source bug.
- **How to remove:** once DGP no longer publishes those configurations for `java-library` consumers
  (or if Pedro is consumed from Maven instead of `includeBuild`), restore the four removed pieces
  from upstream `core/build.gradle.kts`. After any change here, confirm with:

  ```bash
  ./gradlew :TeamCode:dependencyInsight --configuration debugRuntimeClasspath --dependency core
  ```

  The selected variant must be `runtimeElements`, not a `dokka*` one.

When local Pedro changes are added, list them here with:

- files changed
- reason for the patch
- upstream issue or PR if one exists
- how to remove the patch once upstream includes it

## Upstream Update Workflow

Fetch upstream tags and branches without touching the working tree:

```bash
git fetch pedro-upstream --tags
```

Review upstream changes before applying them:

```bash
git log --oneline v2.1.2..pedro-upstream/main
git diff --stat v2.1.2..pedro-upstream/main
```

Because Pedro is vendored under `third_party/PedroPathing`, do not merge `pedro-upstream/main` directly into the robot repo. Apply upstream changes into the vendored prefix from the recorded base tag, then compile TeamCode before committing.

Recommended update shape, replacing `main` with a newer tag if you want a tagged release instead:

```bash
git diff --binary v2.1.2..pedro-upstream/main -- . \
  | git apply --3way --directory=third_party/PedroPathing
```

If upstream changes conflict with local patches, resolve the files in `third_party/PedroPathing`, update this note, then run the normal TeamCode Gradle compile. After a successful update, replace the base tag in this file with the new upstream tag or commit SHA.
