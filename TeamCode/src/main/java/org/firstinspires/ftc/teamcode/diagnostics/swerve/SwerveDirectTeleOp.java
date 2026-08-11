package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Two-stick swerve driving, without the Follower or the localizer in the loop.
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li><b>Left stick</b> - translation. Up drives forward, left strafes left.</li>
 *   <li><b>Right stick X</b> - rotation. Left turns counter-clockwise.</li>
 *   <li><b>Left bumper</b> - hold for 35% speed.</li>
 * </ul>
 *
 * <p>Robot-centric: forward is the robot's nose, not the field. Field-centric needs a working
 * localizer, which is what {@code DriveTeleOp} is for once the odometry pods are installed.
 *
 * <p>Use this while the odometry pods are not installed. {@code Follower.update()} rotates the
 * drive vector by the localizer's heading and runs {@code clampReversePower}, which limits power to
 * 0.2 whenever the commanded direction opposes the <em>measured</em> direction of motion. With no
 * odometry pods - or with the robot on blocks - measured velocity is noise around zero, its sign
 * flips at random, and that clamp fires at random. The result is a drivetrain that stutters and
 * never quite reaches its target angle.
 *
 * <p>This OpMode calls {@link Swerve#arcadeDrive} straight from the sticks instead, which is the
 * same path the bring-up dashboard's kinematics test uses - the one that already behaves correctly.
 * Pods and constants are identical: the drivetrain is taken from a real Follower built by
 * {@link Constants}, so nothing about the calibration changes.
 *
 * <p>Once the odometry pods are installed and verified, go back to {@code DriveTeleOp}, which uses
 * the Follower and gains field-centric driving and path following.
 */
@Config
@TeleOp(name = "Swerve TeleOp", group = "TeleOp")
public class SwerveDirectTeleOp extends OpMode {
    private static final double DEADBAND = 0.05;
    private static final double SLOW_SPEED = 0.35;

    /**
     * Drive from these dashboard values instead of the sticks.
     *
     * <p>FTC Dashboard wipes both gamepads whenever it has not received a gamepad packet for
     * 500 ms, which happens constantly with no driver station attached - the OpMode then reads
     * zero however hard you are holding the stick. These give a stable input source until a
     * driver station is available.
     */
    public static boolean USE_DASHBOARD_INPUT = false;
    public static double FORWARD = 0;
    public static double STRAFE = 0;
    public static double TURN = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private Swerve swerve;
    private double loopHz;

    @Override
    public void init() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drivetrain drivetrain = follower.getDrivetrain();

        if (!(drivetrain instanceof Swerve)) {
            throw new IllegalStateException("Expected a swerve drivetrain, but config.jsonc "
                    + "selected " + drivetrain.getClass().getSimpleName()
                    + ". Set \"drivetrain\" to \"swerve\".");
        }

        swerve = (Swerve) drivetrain;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        swerve.startTeleopDrive(true);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            loopHz = 0.9 * loopHz + 0.1 / dt;
        }

        double forward;
        double strafe;
        double turn;

        if (USE_DASHBOARD_INPUT) {
            forward = FORWARD;
            strafe = STRAFE;
            turn = TURN;
        } else {
            double speed = gamepad1.left_bumper ? SLOW_SPEED : 1.0;
            forward = deadband(-gamepad1.left_stick_y) * speed;
            strafe = deadband(-gamepad1.left_stick_x) * speed;
            turn = deadband(-gamepad1.right_stick_x) * speed;
        }

        swerve.arcadeDrive(forward, strafe, turn);

        telemetry.addData("input", USE_DASHBOARD_INPUT ? "dashboard" : "gamepad");
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);
        telemetry.addData("loopHz", loopHz);
        telemetry.addData("pods", swerve.debugString());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (swerve != null) {
            swerve.arcadeDrive(0, 0, 0);
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}
