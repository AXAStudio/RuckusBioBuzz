package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Runs all four swerve drive motors at POWER, set live from the FTC Dashboard config panel.
 *
 * <p>Driving this from a dashboard variable rather than the gamepad avoids the dashboard's gamepad
 * watchdog, which zeroes the gamepad every 500 ms when no driver station is attached.
 *
 * <p>Starts at 0 so nothing moves until you set it - and POWER is put back to 0 at every init,
 * because it is a static and would otherwise carry the last run's value straight into START.
 * POWER expires {@link #POWER_TIMEOUT_S} after it last changed (nothing reports a dashboard
 * disconnect, so a dropped laptop would otherwise leave the motors running); set it to 0 and back
 * to keep going. Put the robot on blocks.
 */
@Config
@TeleOp(name = "Raw Motor Test", group = "Diagnostics")
public class RawMotorTest extends OpMode {
    public static double POWER = 0;

    private static final double POWER_TIMEOUT_S = 10.0;

    private final DcMotor[] motors = new DcMotor[4];
    private final ElapsedTime powerAge = new ElapsedTime();
    private double lastPower;

    @Override
    public void init() {
        POWER = 0;
        lastPower = 0;
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "sm" + i);
        }

        // sm0 and sm1 spun opposite the other two, so they run reversed from here on.
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        powerAge.reset();
    }

    @Override
    public void loop() {
        if (POWER != lastPower) {
            lastPower = POWER;
            powerAge.reset();
        }
        boolean expired = powerAge.seconds() > POWER_TIMEOUT_S;
        double power = expired || Double.isNaN(POWER) ? 0.0 : Math.max(-1.0, Math.min(1.0, POWER));
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }

        telemetry.addData("power", power);
        if (expired && POWER != 0) {
            telemetry.addData("POWER", "EXPIRED (no change for %.0f s) - set 0 and back to run",
                    POWER_TIMEOUT_S);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
    }
}
