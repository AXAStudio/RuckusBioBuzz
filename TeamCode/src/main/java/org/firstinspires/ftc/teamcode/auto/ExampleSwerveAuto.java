package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrainConstants;

@Autonomous(name = "ExampleSwerveAuto", group = "Auto")
public class ExampleSwerveAuto extends OpMode {
    private static final PathStep POSE_SHOOTING_STEP = new PathStep(56.000, 22.000, 112.000);
    private static final PathStep POSE_INTAKE_1_STEP = new PathStep(8.205, 34.947, 0.000);
    private static final PathStep POSE_HUMAN_PLAYER_STEP = new PathStep(8.199, 8.199, 90.000);
    private static final PathStep POSE_PARK_STEP = new PathStep(56.000, 40.000, 90.000);
    private static final PathStep START_STEP = new PathStep(56.000, 8.000, 90.000);
    private static final PathStep POINT_6 = new PathStep(8.008, 27.616, 0.000);

    private Follower follower;
    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;
    private Path path5;
    private Path path6;
    private Path path7;
    private Path path8;
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;

    @Override
    public void init() {
        // Pedro 3 follows paths with Foresight, whose braking model and gains have not been
        // measured on this robot yet. Refuse to build a path-following OpMode on placeholders.
        SwerveDrivetrainConstants.requireForesightMeasured();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_STEP.toPose());

        buildPaths();
        updateTelemetry("Initialized");
    }

    @Override
    public void init_loop() {
        // Pedro 2's update() with no path only refreshed the pose. Pedro 3's update() in IDLE
        // calls drivetrain.stop(), which writes every servo and zero-power behaviour each loop,
        // so refresh the localizer alone to keep init actuator-silent as before.
        follower.localizer.update();
        updateTelemetry("Ready");
    }

    @Override
    public void start() {
        sequenceIndex = 0;
        stepStarted = false;
        pathFinished = false;

        follower.setPose(START_STEP.toPose());
    }

    @Override
    public void loop() {
        follower.update();

        runSequence();

        updateTelemetry(pathFinished ? "Done" : "Running");
    }

    @Override
    public void stop() {
        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
        PoseStorage.pose = follower.pose();
    }

    private void buildPaths() {
      path1 = Paths.line(
              START_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
          )
          .heading((curve, t) -> interpolateHeading(Math.toRadians(90.000), Math.toRadians(112.000), t, 1.450));

      path2 = Paths.curve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(57.908, 40.376),
              new Pose(32.947, 33.899),
              POSE_INTAKE_1_STEP.toPose()
          )
          .heading((curve, t) -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(0.000), t, 0.250));

      path3 = Paths.line(
              POSE_INTAKE_1_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
          )
          .heading(linearHeading(Math.toRadians(0.000), Math.toRadians(112.000)));

      path4 = Paths.curve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(4.818, 34.811),
              new Pose(7.579, 38.525),
              POSE_HUMAN_PLAYER_STEP.toPose()
          )
          .heading((curve, t) -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(90.000), t, 0.250));

      path5 = Paths.line(
              POSE_HUMAN_PLAYER_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
          )
          .heading(linearHeading(Math.toRadians(90.000), Math.toRadians(112.000)));

      path6 = Paths.curve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(42.526, 29.664),
              POINT_6.toPose()
          )
          .heading((curve, t) -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(0.000), t, 0.250));

      path7 = Paths.line(
              POINT_6.toPose(),
              POSE_SHOOTING_STEP.toPose()
          )
          .heading(linearHeading(Math.toRadians(0.000), Math.toRadians(112.000)));

      path8 = Paths.line(
              POSE_SHOOTING_STEP.toPose(),
              POSE_PARK_STEP.toPose()
          )
          .heading(linearHeading(Math.toRadians(112.000), Math.toRadians(90.000)));
    }

    private void runSequence() {
        if (pathFinished) {
            return;
        }

        switch (sequenceIndex) {
      case 0:
        followPathStep(path1, 1.000);
        break;
      case 1:
        runTimedEventStep("Shoot", 500L);
        break;
      case 2:
        followPathStep(path2, 1.000);
        break;
      case 3:
        followPathStep(path3, 1.000);
        break;
      case 4:
        runTimedEventStep("Shoot", 500L);
        break;
      case 5:
        followPathStep(path4, 1.000);
        break;
      case 6:
        followPathStep(path5, 1.000);
        break;
      case 7:
        runTimedEventStep("Shoot", 500L);
        break;
      case 8:
        followPathStep(path6, 1.000);
        break;
      case 9:
        followPathStep(path7, 1.000);
        break;
      case 10:
        runTimedEventStep("Shoot", 500L);
        break;
      case 11:
        followPathStep(path8, 1.000);
        break;
            default:
                pathFinished = true;
                follower.manual(DrivePowers.zero());
                break;
        }
    }

    private static double interpolateHeading(
        double startHeading,
        double endHeading,
        double tValue,
        double curve
    ) {
        double clampedT = Math.max(0.0, Math.min(1.0, tValue));
        double clampedCurve = Math.max(0.25, Math.min(4.0, curve));
        double shapedT = Math.pow(clampedT, clampedCurve);
        double deltaHeading = normalizeRadians(endHeading - startHeading);
        return normalizeRadians(startHeading + deltaHeading * shapedT);
    }

    /**
     * Pedro 2's setLinearHeadingInterpolation: shortest-way linear in the curve PARAMETER t.
     * Pedro 3's Path.linear() interpolates on curve.pathCompletion(t) - distance fraction -
     * instead, which differs on any Bezier whose speed along t is not uniform. (Upstream 3.0.0's
     * default pathCompletion also ran backwards on a Line; patched in the vendored tree, see
     * RUCKUS_PATCHES.md.) This keeps the Pedro 2 behaviour exactly.
     */
    private static Interpolator linearHeading(double startHeading, double endHeading) {
        double start = Angle.normalize(startHeading);
        double delta = Angle.turnDirection(start, Angle.normalize(endHeading))
                * Angle.smallestDifference(Angle.normalize(endHeading), start);
        return (curve, t) -> Angle.normalize(start + delta * t);
    }

    private static double normalizeRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    private void followPathStep(Path path, double pathSpeed) {
        if (!stepStarted) {
            follower.holdEnd.set(true);
            follower.follow(withPathSpeed(path, clampPathSpeed(pathSpeed)));
            stepStarted = true;
        }

        if (!follower.isBusy()) {
            advanceSequence();
        }
    }

    /**
     * Pedro 2 capped drive POWER per path. Pedro 3 has no power cap; the nearest thing is
     * Foresight's maxPathSpeed, a fraction of max achievable VELOCITY, applied while this path is
     * followed. Every call site passes 1.0, where no modifier is applied at all.
     */
    private Path withPathSpeed(Path path, double pathSpeed) {
        if (pathSpeed >= 1.0 || !(follower.algorithm() instanceof Foresight)) {
            return path;
        }
        return path.with(((Foresight) follower.algorithm()).config.maxPathSpeed.at(pathSpeed));
    }

    private double clampPathSpeed(double pathSpeed) {
        return Math.max(0.05, Math.min(1.0, pathSpeed));
    }

    private void runWaitStep(long durationMs) {
        if (!stepStarted) {
            stepStartTime = System.currentTimeMillis();
            stepStarted = true;
        }

        if (System.currentTimeMillis() - stepStartTime >= durationMs) {
            advanceSequence();
        }
    }

    private void runTimedEventStep(String eventName, long durationMs) {
        if (!stepStarted) {
            stepStartTime = System.currentTimeMillis();
            startEvent(eventName);
            stepStarted = true;
        }

        if (System.currentTimeMillis() - stepStartTime >= durationMs) {
            finishEvent(eventName);
            advanceSequence();
        }
    }

    private void startEvent(String eventName) {
        switch (eventName) {
      case "Shoot":
        startShoot();
        break;
            default:
                break;
        }
    }

    private void finishEvent(String eventName) {
        switch (eventName) {
      case "Shoot":
        finishShoot();
        break;
            default:
                break;
        }
    }

    private void startShoot() {
        // TODO: start Shoot mechanism here.
    }

    private void finishShoot() {
        // TODO: stop Shoot mechanism here.
    }

    private void advanceSequence() {
        sequenceIndex++;
        stepStarted = false;
    }

    private void updateTelemetry(String state) {
        Pose pose = follower.pose();

        telemetry.addData("State", state);
        telemetry.addData("Sequence", sequenceIndex);
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
