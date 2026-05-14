package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "ExampleSwerveAuto", group = "Auto")
public class ExampleSwerveAuto extends OpMode {
    private static final Pose START_POSE = new Pose(72.0, 72.0, 0.0);
    private static final Pose END_POSE = new Pose(96.0, 72.0, 0.0);

    private Follower follower;
    private Path driveForward;
    private boolean pathStarted;
    private boolean pathFinished;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

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
        pathStarted = true;
        pathFinished = false;

        follower.setStartingPose(START_POSE);
        follower.followPath(driveForward, true);
    }

    @Override
    public void loop() {
        follower.update();

        if (pathStarted && !pathFinished && !follower.isBusy()) {
            pathFinished = true;
        }

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
        driveForward = new Path(new BezierLine(START_POSE, END_POSE));
        driveForward.setConstantHeadingInterpolation(START_POSE.getHeading());
    }

    private void updateTelemetry(String state) {
        Pose pose = follower.getPose();

        telemetry.addData("State", state);
        telemetry.update();
    }
}
