package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "servoTest", group = "Test")
public class servoTest extends LinearOpMode {
    public static String SERVO_NAME = "servo";
    public static double position = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo test = hardwareMap.get(Servo.class, SERVO_NAME);
        waitForStart();
        while (opModeIsActive()) {
            test.setPosition(position);
            telemetry.addData("power (from dashboard)", position);
            telemetry.addData("servo getPosition()", test.getPosition());
            telemetry.update();
        }
        test.setPosition(0);
    }
}
