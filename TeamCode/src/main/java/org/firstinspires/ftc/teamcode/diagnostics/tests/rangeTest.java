package org.firstinspires.ftc.teamcode.diagnostics.tests;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;
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

    zoneCheck zoneCheck;
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
        zoneCheck = new zoneCheck(Alliance.RED);


        waitForStart();
        while (opModeIsActive()) {
            Pose pose = pinpoint.pose();

            nectarTurret.setVelocity(nectarTurretVel);
            pollenTurret.setVelocity(pollenTurretVel);
            zoneCheck.inZoneNectar(pose);
            zoneCheck.inZonePollen(pose);
        }

    }
}