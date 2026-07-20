# Drivetrain PIDFs with Pedro Pathing

Two different ways to drive an axis: raw open-loop power (no PIDF involved, just
moves the robot) and closed-loop via `holdPoint` (runs the tuned PIDF from
whichever drivetrain `Constants.java` currently resolves to). Both work the same
regardless of `config.jsonc`'s `"drivetrain"` value — swerve and mecanum go
through the same `Follower` calls.

## Open-loop (raw power, no PIDF)

```java
follower.setTeleOpDrive(p, 0, 0, true); // forward/back
follower.setTeleOpDrive(0, p, 0, true); // strafe
follower.setTeleOpDrive(0, 0, p, true); // rotate, positive p = counterclockwise
```

`setTeleOpDrive` bypasses the translational and heading PIDFs entirely — it
sends `p` straight to the drivetrain. This is what `ForwardVelocityTuner` /
`LateralVelocityTuner` / `ForwardZeroPowerAccelerationTuner` in
`pedroPathing/Tuning.java` use to characterize raw speed and coast-down, not to
test PIDF response.

## Closed-loop (uses the tuned PIDFs, via `holdPoint`)

```java
Pose current = follower.getPose();

follower.holdPoint(new Pose(current.getX() + d, current.getY(), current.getHeading())); // forward/back offset d
follower.holdPoint(new Pose(current.getX(), current.getY() + d, current.getHeading())); // strafe offset d
follower.holdPoint(new Pose(current.getX(), current.getY(), p));                        // rotate to heading p (radians)
```

All three run through the same `translationalPIDFCoefficients` (position) and
`headingPIDFCoefficients` (rotation) — there's no separate "forward PIDF" vs
"strafe PIDF" in code, it's one 2D translational PIDF, just being exercised
from different offset directions. Still worth testing both directions, since
swerve/mecanum kinematics can respond asymmetrically per axis even with one
shared PIDF.

Gotchas:
- `follower.update()` must run every loop for the hold to actually apply —
  `holdPoint` only sets the target, it doesn't block or run to completion.
- `follower.isBusy()` stays `false` while holding (it's an indefinite hold),
  so it can't be polled for "done."
- To let go of the hold: `follower.startTeleopDrive(...)` (back to manual) or
  `follower.followPath(...)` (back to autonomous).

## Where the actual PIDF values live

- `pedroPathing/SwerveDrivetrainConstants.java` / `pedroPathing/MecanumAltConstants.java`
  — `translationalPIDFCoefficients`, `headingPIDFCoefficients`,
  `drivePIDFCoefficients` (along-path speed regulation — only active during
  `follower.followPath(...)`, not during `holdPoint`).
- `pedroPathing/Constants.java` picks between them based on `config.jsonc`'s
  `"drivetrain"` field, so tuning one doesn't touch the other.
