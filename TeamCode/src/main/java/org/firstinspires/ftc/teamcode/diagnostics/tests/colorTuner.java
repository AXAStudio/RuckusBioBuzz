package org.firstinspires.ftc.teamcode.diagnostics.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorTuner", group = "TeleOp")
public class colorTuner extends LinearOpMode {
    private ColorSensor colorSensor;
    public static double rThreshRED = 100;
    public static double gThreshRED = 100;
    public static double bThreshRED = 100;
    public static double rThreshBLUE = 100;
    public static double gThreshBLUE = 100;
    public static double bThreshBLUE = 100;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();
        while (opModeIsActive()) {
            if(colorSensor.red() > rThreshBLUE &&  colorSensor.green() > gThreshBLUE && colorSensor.blue() > bThreshBLUE ){
                telemetry.addData("BLUE BALL", true);
            }else if(colorSensor.red() > rThreshRED &&  colorSensor.green() > gThreshRED && colorSensor.blue() > bThreshRED ) {
                telemetry.addData("BLUE BALL", true);
            }
            }
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.update();
        }
    }
