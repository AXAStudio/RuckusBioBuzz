package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.diagnostics.swerve.TeleLoopProbe;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.HeadingHold;

@TeleOp(name = "Drive TeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private static final double DRIVE_DEADBAND = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;

    private Follower follower;

    private final java.util.List<LynxModule> hubs = new java.util.ArrayList<>();

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopHz;

    /** Smoothed loop period. Smoothing 1/dt instead overweights fast loops (~1.8x optimistic). */
    private double loopDtEma;

    /** Telemetry is for a human reading a screen; 10 Hz is already faster than anyone reads. */
    private static final double TELEMETRY_INTERVAL_S = 0.1;
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    /**
     * Loop-rate histogram, pod capture and a /swerve/state snapshot for this OpMode.
     *
     * <p>Diagnostic, and deliberately three lines here rather than two hundred: see
     * {@link TeleLoopProbe}. Until 2026-08-16 this loop's rate had never been measured with
     * 1/mean(dt) and no capture of the competition path existed, so every steering conclusion in
     * this project came from a different OpMode with a different publish path.
     */
    private final TeleLoopProbe probe = new TeleLoopProbe();

    /** Closed heading loop. See {@link HeadingHold} - this OpMode had none before 2026-08-16. */
    private final HeadingHold headingHold = new HeadingHold();

    @Override
    public void init() {
        // The swerve pods' turn PID runs at this OpMode's loop rate, so loop rate is a control
        // parameter here. Measured 2026-08-12: without bulk caching this loop ran at 30.7 Hz, a
        // 32.5 ms period - longer than the servos' own 20 ms PWM frame, so the controller was
        // updating less often than the hardware could accept commands. Pedro does not configure
        // caching itself (nothing in the vendored tree touches LynxModule), so it defaults to off
        // and every encoder read is a separate bus transaction.
        // MANUAL rather than AUTO, cleared once per loop below. AUTO re-fetches a whole bulk
        // packet whenever the SAME channel is read twice in one loop, and this loop reads every
        // pod encoder at least twice (Swerve's avgScaling pass and CoaxialPod.move) - so AUTO
        // was still paying several bulk transactions per loop. MANUAL pins one snapshot per
        // loop: every repeat is served from cache, and all pods act on the same reading.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hubs.add(module);
        }

        follower = Constants.createFollower(hardwareMap);
        probe.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        follower.update();
        headingHold.latch(follower.getPose().getHeading());
    }

    @Override
    public void loop() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            loopDtEma = loopDtEma == 0 ? dt : 0.9 * loopDtEma + 0.1 * dt;
            loopHz = 1.0 / loopDtEma;
        }

        double speed = gamepad1.left_bumper ? SLOW_SPEED : NORMAL_SPEED;

        // Deadband the translation VECTOR, not each axis. Per-axis deadbanding does not shorten
        // a shallow-diagonal command, it rotates it: the smaller axis is zeroed and the direction
        // snaps to the nearest cardinal. Measured 2026-08-16 on the equivalent 0.06 deadband in
        // the bring-up dashboard - 35.5% of driving samples had exactly one axis zeroed, crossing
        // in and out of that state 1.44 times a second, and 10% of all large azimuth-setpoint
        // jumps traced to it. Every pod's azimuth is atan2 of this vector, so the snap goes
        // straight to the wheels. Rescaling from the deadband edge also means the output ramps
        // from zero instead of stepping to 0.05.
        double rawForward = -gamepad1.left_stick_y;
        double rawStrafe = -gamepad1.left_stick_x;
        double mag = Math.hypot(rawForward, rawStrafe);
        double scale = mag > DRIVE_DEADBAND
                ? (mag - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND) / mag * speed
                : 0.0;
        double forward = rawForward * scale;
        double strafe = rawStrafe * scale;
        // Rotation is one-dimensional: there is no direction for a deadband to distort, so a
        // scalar deadband is correct here. It is still rescaled so the output starts from zero.
        double stick = applyDeadband(-gamepad1.right_stick_x) * speed;

        // Closed heading loop, new 2026-08-16. Until then this OpMode had none: the stick went
        // to the mixer as a raw rate and nothing held a heading, so a shove or a drift was
        // simply accepted. Right bumper falls back to the old open-loop behaviour, which is both
        // the escape hatch and the A/B.
        boolean holdHeading = !gamepad1.right_bumper;
        double heading = follower.getPose().getHeading();
        double turn = holdHeading
                ? headingHold.update(stick, heading, Math.hypot(forward, strafe) > 0,
                        dt > 0 ? dt : 0.02)
                : stick;
        if (!holdHeading) {
            // Keep the setpoint under the robot while it is being flown open-loop, so releasing
            // the bumper does not snap back to wherever the hold was last latched.
            headingHold.latch(heading);
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();

        Pose probePose = follower.getPose();
        probe.update(dt, loopHz, Math.toDegrees(probePose.getHeading()),
                Math.toDegrees(headingHold.targetRad()),
                probePose.getX(), probePose.getY(), forward, strafe, turn);

        // Telemetry is throttled and no longer includes the drivetrain dump. debugString() calls
        // getRawAngleRad() twice per pod on top of the two reads move() already does, then builds
        // a multi-line string, and MultipleTelemetry pushed all of it to the Driver Station and to
        // FTC Dashboard every single loop. Per-pod detail lives on the bring-up dashboard at
        // http://192.168.43.1:8080/swerve, which is built for it and does not sit in this loop.
        if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_S) {
            telemetryTimer.reset();
            Pose pose = follower.getPose();
            // loopHz is a smoothed 1/dt for the driver's benefit. The honest statistic - and the
            // only one that may be quoted - is in probe.summary().
            telemetry.addData("loopHz", loopHz);
            telemetry.addLine(probe.summary());
            telemetry.addLine(probe.recStatus());
            telemetry.addData("Drive Mode", gamepad1.left_bumper ? "Slow" : "Normal");
            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.addData("heading hold", holdHeading
                    ? headingHold.phase() + " err " + String.format(java.util.Locale.US, "%.2f deg",
                            headingHold.errorDeg(pose.getHeading()))
                    : "OFF (right bumper)");
            if (!headingHold.enabled()) {
                telemetry.addData("heading hold DISABLED", headingHold.disabledReason());
            }
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
        double a = Math.abs(value);
        if (a < DRIVE_DEADBAND) {
            return 0.0;
        }
        return Math.signum(value) * (a - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND);
    }
}
