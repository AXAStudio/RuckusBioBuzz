package org.firstinspires.ftc.teamcode.diagnostics.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest", group = "TeleOp")
public class servoTest extends LinearOpMode {
    Servo test;
    public static double pos;

    @Override
    public void runOpMode() {
        test = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            test.setPosition(pos);
        }
    }
}
