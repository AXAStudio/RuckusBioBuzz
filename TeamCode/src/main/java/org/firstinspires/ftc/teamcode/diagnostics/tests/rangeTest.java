package org.firstinspires.ftc.teamcode.diagnostics.tests;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.tools.blobDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Range Test",group = "TeleOp")
public class rangeTest extends LinearOpMode {
    DcMotorEx nectarTurret;
    DcMotorEx pollenTurret;
    public static int nectarTurretVel = 1000;
    public static int pollenTurretVel = 1000;
    public static int sections = 10;
    PinpointLocalizer pinpoint;
    int yPollenOffset = 0;
    int yNectarOffset = 0;
    @Override
    public void runOpMode() {
        nectarTurret = hardwareMap.get(DcMotorEx.class, "nectarTurret");
        pollenTurret = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        pinpoint = hardwareMap.get(PinpointLocalizer.class, "pinpoint");
        nectarTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            Pose pose = pinpoint.pose();
            int nectarOffsetx = (int)(-yNectarOffset*Math.sin(pose.heading())); //this just spins the vecter
            int nectarOffsety = (int)(yNectarOffset*Math.cos(pose.heading())); //this just spins the vecter
            int pollenOffsetx = (int)(-yPollenOffset*Math.sin(pose.heading())); //this just spins the vecter
            int pollenOffsety = (int)(yPollenOffset*Math.cos(pose.heading()));
            nectarTurret.setVelocity(nectarTurretVel);
            pollenTurret.setVelocity(pollenTurretVel);
            telemetry.addData("xPollen", (int)(((pose.x()+nectarOffsetx)/sections)+1));
            telemetry.addData("yNectar", (int)(((pose.y()+nectarOffsety)/sections)+1));
            telemetry.addData("xPollen", (int)(((pose.x()+pollenOffsetx)/sections)+1));
            telemetry.addData("yNectar", (int)(((pose.y()+pollenOffsety)/sections)+1));


        }

    }
}