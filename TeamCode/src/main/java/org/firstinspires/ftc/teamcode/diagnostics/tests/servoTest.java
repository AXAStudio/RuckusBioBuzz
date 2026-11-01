package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * CRServo bench test. Edit {@code power} under Configuration -> ServoTest in
 * FTC Dashboard (http://192.168.43.1:8080/dash) and press Enter / Save.
 * The value actually in use is echoed to telemetry, so you can tell
 * "dashboard didn't change the field" apart from "servo ignored the command".
 */
@Config
@TeleOp(name = "servoTest", group = "Test")
public class servoTest extends LinearOpMode {
    public static String SERVO_NAME = "servo";
    public static double power = 0.0;   // -1..1, CR speed

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CRServo test = hardwareMap.get(CRServo.class, SERVO_NAME);
        waitForStart();
        while (opModeIsActive()) {
            double p = Math.max(-1.0, Math.min(1.0, power));
            test.setPower(p);
            telemetry.addData("power (from dashboard)", p);
            telemetry.addData("servo getPower()", test.getPower());
            telemetry.update();
        }
        test.setPower(0);
    }
}
