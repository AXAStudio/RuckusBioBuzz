package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.tools.blobDetection;

@TeleOp(name = "Blob Detection Test",group = "TeleOp")
public class blobDetectionTest extends LinearOpMode {

    blobDetection detector = new blobDetection();
    VisionPortal portal;

    @Override
    public void runOpMode() {

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Raw Angle", detector.rawAngle);
            telemetry.addData("Median X",  detector.median);
            telemetry.update();
        }

        portal.close();
    }
}