package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Runs all four swerve drive motors at POWER, set live from the FTC Dashboard config panel.
 *
 * <p>Driving this from a dashboard variable rather than the gamepad avoids the dashboard's gamepad
 * watchdog, which zeroes the gamepad every 500 ms when no driver station is attached.
 *
 * <p>Starts at 0 so nothing moves until you set it. Put the robot on blocks.
 */
@Config
@TeleOp(name = "Raw Motor Test", group = "Diagnostics")
public class RawMotorTest extends OpMode {
    public static double POWER = 0;

    private final DcMotor[] motors = new DcMotor[4];

    @Override
    public void init() {
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, "sm" + i);
        }

        // sm0 and sm1 spun opposite the other two, so they run reversed from here on.
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        for (DcMotor motor : motors) {
            motor.setPower(POWER);
        }

        telemetry.addData("power", POWER);
        telemetry.update();
    }
}
