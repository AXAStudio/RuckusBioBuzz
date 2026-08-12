package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive TeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_DEADBAND = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;

    private Follower follower;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopHz;

    /** Telemetry is for a human reading a screen; 10 Hz is already faster than anyone reads. */
    private static final double TELEMETRY_INTERVAL_S = 0.1;
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void init() {
        // The swerve pods' turn PID runs at this OpMode's loop rate, so loop rate is a control
        // parameter here. Measured 2026-08-12: without bulk caching this loop ran at 30.7 Hz, a
        // 32.5 ms period - longer than the servos' own 20 ms PWM frame, so the controller was
        // updating less often than the hardware could accept commands. Pedro does not configure
        // caching itself (nothing in the vendored tree touches LynxModule), so it defaults to off
        // and every encoder read is a separate bus transaction.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        follower.update();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            loopHz = 0.9 * loopHz + 0.1 * (1.0 / dt);
        }

        double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;

        double forward = applyDeadband(-gamepad1.left_stick_y) * speed;
        double strafe = applyDeadband(-gamepad1.left_stick_x) * speed;
        double turn = applyDeadband(-gamepad1.right_stick_x) * speed;

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();

        // Telemetry is throttled and no longer includes the drivetrain dump. debugString() calls
        // getRawAngleRad() twice per pod on top of the two reads move() already does, then builds
        // a multi-line string, and MultipleTelemetry pushed all of it to the Driver Station and to
        // FTC Dashboard every single loop. Per-pod detail lives on the bring-up dashboard at
        // http://192.168.43.1:8080/swerve, which is built for it and does not sit in this loop.
        if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_S) {
            telemetryTimer.reset();
            Pose pose = follower.getPose();
            telemetry.addData("loopHz", loopHz);
            telemetry.addData("Drive Mode", gamepad1.left_bumper ? "Slow" : "Normal");
            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
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
