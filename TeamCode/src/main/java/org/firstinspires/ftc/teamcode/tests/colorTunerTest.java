package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tools.colorTuner;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Color Tuner Test", group = "TeleOp")
public class colorTunerTest extends LinearOpMode {

    colorTuner detector;
    VisionPortal portal;

    @Override
    public void runOpMode() {

        detector = new colorTuner(telemetry); // pass telemetry here

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }

        portal.close();
    }
}