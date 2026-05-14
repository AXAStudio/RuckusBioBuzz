package org.firstinspires.ftc.teamcode.tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive TeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_DEADBAND = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        follower.update();
    }

    @Override
    public void loop() {
        double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;

        double forward = applyDeadband(-gamepad1.left_stick_y) * speed;
        double strafe = applyDeadband(-gamepad1.left_stick_x) * speed;
        double turn = applyDeadband(-gamepad1.right_stick_x) * speed;

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();

        Pose pose = follower.getPose(); // if using custom paths in teleop
        telemetry.addData("Drive Mode", gamepad1.left_bumper ? "Slow" : "Normal");
        telemetry.update();
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

    private double applyDeadband(double value) {
        return Math.abs(value) < DRIVE_DEADBAND ? 0.0 : value;
    }
}
