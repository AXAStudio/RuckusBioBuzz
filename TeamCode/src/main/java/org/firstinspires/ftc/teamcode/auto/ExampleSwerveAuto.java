package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "ExampleSwerveAuto", group = "Auto")
public class ExampleSwerveAuto extends OpMode {
    private static final PathStep POSE_SHOOTING_STEP = new PathStep(56.000, 22.000, 112.000);
    private static final PathStep POSE_INTAKE_1_STEP = new PathStep(8.205, 34.947, 0.000);
    private static final PathStep POSE_HUMAN_PLAYER_STEP = new PathStep(8.199, 8.199, 90.000);
    private static final PathStep POSE_PARK_STEP = new PathStep(56.000, 40.000, 90.000);
    private static final PathStep START_STEP = new PathStep(56.000, 8.000, 90.000);
    private static final PathStep POINT_6 = new PathStep(8.008, 27.616, 0.000);

    private Follower follower;
    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_STEP.toPose());

        buildPaths();
        updateTelemetry("Initialized");
    }

    @Override
    public void init_loop() {
        follower.update();
        updateTelemetry("Ready");
    }

    @Override
    public void start() {
        sequenceIndex = 0;
        stepStarted = false;
        pathFinished = false;

        follower.setStartingPose(START_STEP.toPose());
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

        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
        follower.update();
    }

    private void buildPaths() {
      path1 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              START_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
            )
          )
          .setHeadingInterpolation(closestPoint -> interpolateHeading(Math.toRadians(90.000), Math.toRadians(112.000), closestPoint.getTValue(), 1.450))
          .build();

      path2 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(57.908, 40.376),
              new Pose(32.947, 33.899),
              POSE_INTAKE_1_STEP.toPose()
            )
          )
          .setHeadingInterpolation(closestPoint -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(0.000), closestPoint.getTValue(), 0.250))
          .build();

      path3 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              POSE_INTAKE_1_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(112.000))
          .build();

      path4 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(4.818, 34.811),
              new Pose(7.579, 38.525),
              POSE_HUMAN_PLAYER_STEP.toPose()
            )
          )
          .setHeadingInterpolation(closestPoint -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(90.000), closestPoint.getTValue(), 0.250))
          .build();

      path5 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              POSE_HUMAN_PLAYER_STEP.toPose(),
              POSE_SHOOTING_STEP.toPose()
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(112.000))
          .build();

      path6 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              POSE_SHOOTING_STEP.toPose(),
              new Pose(42.526, 29.664),
              POINT_6.toPose()
            )
          )
          .setHeadingInterpolation(closestPoint -> interpolateHeading(Math.toRadians(112.000), Math.toRadians(0.000), closestPoint.getTValue(), 0.250))
          .build();

      path7 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              POINT_6.toPose(),
              POSE_SHOOTING_STEP.toPose()
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(112.000))
          .build();

      path8 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              POSE_SHOOTING_STEP.toPose(),
              POSE_PARK_STEP.toPose()
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(112.000), Math.toRadians(90.000))
          .build();
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
                follower.startTeleopDrive(true);
                follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
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

    private static double normalizeRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    private void followPathStep(PathChain path, double pathSpeed) {
        if (!stepStarted) {
            follower.followPath(path, clampPathSpeed(pathSpeed), true);
            stepStarted = true;
        }

        if (!follower.isBusy()) {
            advanceSequence();
        }
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
        Pose pose = follower.getPose();

        telemetry.addData("State", state);
        telemetry.addData("Sequence", sequenceIndex);
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}