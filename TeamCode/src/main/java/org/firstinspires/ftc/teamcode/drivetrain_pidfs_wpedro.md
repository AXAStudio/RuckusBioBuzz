# Drivetrain PIDFs with Pedro Pathing 3.0.0

Pedro 3 renamed almost every `Follower` call and replaced the 2.x PIDF follower
with **Foresight**. Everything below is checked against the vendored source in
`third_party/PedroPathing/core/src/main/java/com/pedropathing/` — read that, not
the upstream 2.x docs.

| Pedro 2.1.2 | Pedro 3.0.0 |
|---|---|
| `follower.setTeleOpDrive(f, s, t, true)` + `startTeleopDrive()` | `follower.manual(f, s, t)` (robot-centric only) |
| `follower.holdPoint(pose)` | `follower.hold(pose)` / `follower.hold(pose, useScaling)` |
| `follower.followPath(path)` | `follower.follow(path)` |
| `follower.getPose()` | `follower.pose()` |
| `follower.getVelocity()` | `follower.velocity()` (field frame, `.vx .vy .omega`) |
| `pose.getX() / getY() / getHeading()` | `pose.x() / y() / heading()`, plus `withX/withY/withHeading` |
| `translationalPIDFCoefficients`, `headingPIDFCoefficients`, `drivePIDFCoefficients` | `ForesightConfig`: `forwardTranslational`, `strafeTranslational`, `headingFeedback`, `coast`, `brake` |
| `FollowerBuilder` | `new Follower(localizer, drivetrain, new Foresight(config))` |

All of this is the same for swerve and mecanum — `Constants.createFollower`
picks the drivetrain from `config.jsonc`'s `"drivetrain"` and every OpMode uses
the same `Follower` calls.

Units: inches, radians, **heading CCW-positive**. `DrivePowers.turn` is also
CCW-positive (the `Swerve` mixer carries a `RUCKUS PATCH` so this holds on
swerve too).

---

## 1. Open-loop (raw power, no PIDF)

```java
follower.manual(p, 0, 0);   // forward/back (robot frame)
follower.manual(0, p, 0);   // strafe, positive p = LEFT
follower.manual(0, 0, p);   // rotate, positive p = counterclockwise
follower.update();          // required every loop, manual() only stores the command
```

`manual(...)` sends the powers straight to `drivetrain.drive(powers, true)` —
no Foresight, no PIDF. It is **robot-centric only**; Pedro 3 has no
field-centric flag. `tele/TeleOp.java` drives exactly like this:

```java
follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
follower.update();
```

For field-centric, rotate the stick vector by `-heading` yourself (same math as
§3 below).

## 2. Closed-loop through Foresight (`hold`)

```java
Pose current = follower.pose();

follower.hold(current.withX(current.x() + d));   // forward/back (FIELD x) offset d
follower.hold(current.withY(current.y() + d));   // strafe (FIELD y) offset d
follower.hold(current.withHeading(theta));       // rotate to field heading theta (radians)
```

Note the offsets are **field-frame** — `withX` moves along the field x axis,
which is only "forward" when the heading is 0.

`hold` runs Foresight's `calculateHold`, which uses:

- `forwardTranslational` / `strafeTranslational` — two translational
  controllers, one per robot axis (2.x had one shared 2D translational PIDF).
- `headingFeedback` — heading controller. **P only on purpose**: Foresight
  calls it twice per update with two different errors, which would corrupt a
  D term.
- It also projects the pose forward by the **braking model**
  (`linear/quadraticBrakeCoefficients`) before computing error, so bad brake
  coefficients show up as hold overshoot, not as a bad PID.

`hold(pose, true)` multiplies the correction by `holdPointTranslationalScaling`
(default 0.45) and `holdPointHeadingScaling` (default 0.35) — softer hold.
`hold(pose)` is `useScaling = false`.

Gotchas:
- `follower.update()` must run every loop — `hold` only sets the target.
- **Swerve refuses to drive Foresight on placeholder constants.** Call
  `SwerveDrivetrainConstants.requireForesightMeasured()` before `hold`/`follow`
  (the autos already do). `FORESIGHT_MEASURED` is currently `false`, so on the
  swerve build `hold` is not usable until the Foresight Tuner has been run.
- **Don't poll `isBusy()` for "arrived" after a fresh `hold`.** `busy` is only
  set `true` by `follow()` (via `algorithm.reset()`); `hold()` doesn't reset it,
  so after a manual → hold switch it can already read `false`. Check the error
  yourself: `follower.pose().distance(target)` and
  `Angle.error(follower.pose().heading(), target.heading())`.
- `follower.holding()` / `manual()` / `following()` / `idle()` / `mode()`
  report the current mode.
- To let go: `follower.manual(...)` (back to driver), `follower.follow(path)`
  (back to auto), or `follower.stop()` (idle, drivetrain stopped).

## 3. Your own translational PIDF (drive to a point while in `manual`)

Use this when you want a closed loop on position without Foresight — e.g. a
"snap to shooting spot" button in TeleOp. You own the controllers and feed the
result into `manual(...)`.

```java
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Utils;

// Starting gains = the 2.x translational PD measured on this robot (0.26 / 0.025, power per inch).
// They were tuned for Pedro's follower, not this loop — re-tune before trusting them.
private final PIDFController xPid = new PIDFController(new PIDFCoefficients(0.26, 0, 0.025, 0));
private final PIDFController yPid = new PIDFController(new PIDFCoefficients(0.26, 0, 0.025, 0));

/** Returns {forward, strafe} robot-frame powers that drive toward target (field frame). */
private double[] translationalToward(Pose target) {
    Pose pose = follower.pose();
    Velocity vel = follower.velocity();              // field frame, in/s

    double ex = target.x() - pose.x();               // field-frame error, inches
    double ey = target.y() - pose.y();

    // D on measurement: d(error)/dt = -velocity. No derivative kick when the target moves.
    xPid.updateErrorWithDerivative(ex, -vel.vx);
    yPid.updateErrorWithDerivative(ey, -vel.vy);
    double fx = xPid.run();                          // field-frame power
    double fy = yPid.run();

    // Field -> robot frame (rotate by -heading). manual() is robot-centric.
    double h = pose.heading();
    double forward =  fx * Math.cos(h) + fy * Math.sin(h);
    double strafe  = -fx * Math.sin(h) + fy * Math.cos(h);

    // Clamp the vector, not each axis, so the direction survives saturation.
    double mag = Math.hypot(forward, strafe);
    if (mag > 1.0) { forward /= mag; strafe /= mag; }
    return new double[]{forward, strafe};
}
```

```java
// in loop():
double[] t = translationalToward(shootingSpot);
follower.manual(t[0], t[1], turn);   // turn from §4, or the stick
follower.update();
```

Notes:
- `PIDFController` is the **`RUCKUS PATCH`** class in `com.pedropathing.control`
  (upstream deleted it in v3). It has the integral clamp/band/reset threshold
  patches; with `I = 0` those are no-ops. Pedro's own `Controller.pid(kP, kI, kD)`
  also works — call `pid.calculate(0, error, measuredVelocity)` for the
  D-on-measurement form — but it has no integral clamp and drops D when
  `dt ≤ 1 ms`.
- Call `xPid.reset()` / `yPid.reset()` whenever you switch into this mode, or
  the first D sample uses a stale timestamp.
- Leave `F = 0`. On this codebase F is applied as `feedForwardInput * F`, and
  nothing here sets a meaningful feed-forward input.

## 4. Rotational PIDF for aiming

`modules/aimingSystem` returns `{distance, theta}` where `theta` is the
**field-frame** angle from the (lead-compensated) turret position to the hive.
To point the chassis at it you need a heading loop that **keeps working while
the driver translates** — `hold()` can't do that (it also locks x/y), so the
aiming loop lives outside Foresight and goes into `manual(...)`'s turn slot.

### 4a. Quick version: heading PD on the aim angle

```java
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;

// Starting gains = the heading values measured through the bring-up tool
// (kP 1.20, kD 0.080, 2026-08-13 — see SwerveDrivetrainConstants / HeadingHold).
// Error is in RADIANS, output is turn power.
private final PIDFController aimPid = new PIDFController(new PIDFCoefficients(1.20, 0, 0.080, 0));

/** If the shooter points out the back of the robot, set this to Math.PI. */
private static final double SHOOTER_HEADING_OFFSET = 0.0;

private double aimTurn(double thetaField) {
    double heading = follower.pose().heading();
    double target  = thetaField - SHOOTER_HEADING_OFFSET;

    // Signed shortest-path error in [-pi, pi) — never take the long way round.
    double error = Angle.error(heading, target);

    // D on measurement (omega is CCW-positive rad/s, same sign as heading).
    aimPid.updateErrorWithDerivative(error, -follower.velocity().omega);
    return Utils.clamp(aimPid.run(), -1.0, 1.0);
}
```

```java
// in loop(), aim while the right bumper is held, driver keeps translation:
double fwd    = -gamepad1.left_stick_y;
double strafe = -gamepad1.left_stick_x;
double turn;
if (gamepad1.right_bumper) {
    if (!aiming) { aimPid.reset(); aiming = true; }   // reset on entry
    turn = aimTurn(aimer.aim()[1]);
} else {
    aiming = false;
    turn = -gamepad1.right_stick_x;
}
follower.manual(fwd, strafe, turn);
follower.update();

boolean aimed = Math.abs(Angle.error(follower.pose().heading(), aimer.aim()[1])) < Math.toRadians(2);
```

### 4b. Adding kS for the last few degrees

Heading P alone stalls short when `kP·error` is below what it takes to break
the chassis loose (at kP 1.20 a 2° error is only 0.042 power). The bring-up
history for the 2.x heading loop found kP below ~1.0 left 3–4° standing and
1.40 overshot 22–51°, so raising kP is not the fix. Add a static-friction
kick outside a small band instead:

```java
private static final double AIM_KS = 0.04;                   // ESTIMATED — tune on the robot
private static final double AIM_KS_BAND = Math.toRadians(1); // no kick inside the band

double out = aimPid.run();
if (Math.abs(error) > AIM_KS_BAND) out += Math.signum(error) * AIM_KS;
return Utils.clamp(out, -1.0, 1.0);
```

Do **not** put this in the PIDF's `F` — `F` multiplies the feed-forward input,
it's a relay if you feed it a sign, and 0.06 on the 2.x heading loop produced
23–26 oscillations per step. Pedro 3's equivalent for the path follower is
`ForesightConfig.headingStaticFF` (`Controller.staticFeedforward(kS)`), which
is separate from `headingFeedback` for the same reason.

### 4c. Aim with Pedro's heading controller instead (robot stationary)

If the robot can stop to shoot, `hold` is the least code — Foresight's
`headingFeedback` (P 1.20) does the rotation and the translational controllers
keep it planted:

```java
SwerveDrivetrainConstants.requireForesightMeasured();       // throws until Foresight is tuned
follower.hold(follower.pose().withHeading(aimer.aim()[1]));  // re-issue each loop to track a moving lead
```

Re-issuing `hold` every loop is fine (it only swaps the target), but it also
re-latches x/y to wherever the robot drifted to.

### Aiming gotchas
- **`theta` must be field frame and the pose must be right.** Aim is only as
  good as `follower.pose()`. TeleOp loads the auto's end pose from
  `PoseStorage`; if TeleOp starts without an auto first, the pose is wherever
  the Pinpoint thinks it is.
- **Don't differentiate the aim target.** `aimer.aim()` moves as the robot
  moves (it uses `predictor.leadPose`). D on error would add a kick every time
  the lead jumps — that's why the examples use `-omega`.
- **Existing heading-hold code:** `pedroPathing/HeadingHold.java` already has
  lead-capping, a stuck-sensor guard, and wrap-safe error with the same
  kP 1.20 / kD 0.080. It is **not yet validated on the robot**. If you build aim
  on top of it, reuse its `correction` logic rather than writing a third copy
  (`SwerveBringUp` already has one — see the duplication note in that class).

## 5. Where the gains actually live

- `pedroPathing/SwerveDrivetrainConstants.java` → `foresightConfig`:
  `headingFeedback` = `Controller.proportional(1.20)`,
  `forward/strafeTranslational` = `Controller.pid(0.26, 0, 0.025)` (carried from
  2.x), `coast`/`brake` = `Controller.proportionalFeedforward(1/73.9)`
  (**unmeasured**), plus the braking-model coefficients. The comment block above
  it lists which values are measured, carried, derived, or placeholder.
- `pedroPathing/MecanumDrivetrainConstants.java` → its own `ForesightConfig`
  (heading P 0.87, translational PID 0.093 / 0 / 0.013).
- `pedroPathing/Constants.java` picks between them from `config.jsonc`'s
  `"drivetrain"`, so tuning one doesn't touch the other.
- **There is no `drivePIDFCoefficients` anymore.** Along-path speed is handled
  by Foresight's `coast` / `brake` controllers and the braking model.
- The swerve **pod** turn PIDF (`turnKPPerPod` etc.) is a different loop
  entirely — it steers each module and is unrelated to anything above.
- Tune Foresight with the Foresight Tuner in `pedroPathing/Tuning.java`, paste
  the values, then flip `FORESIGHT_MEASURED = true`.
