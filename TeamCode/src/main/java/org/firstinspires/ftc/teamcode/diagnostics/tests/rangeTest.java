package org.firstinspires.ftc.teamcode.diagnostics.tests;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.ZoneCheck;

@TeleOp(name = "Range Test",group = "TeleOp")
public class rangeTest extends LinearOpMode {
    DcMotorEx nectarTurret;
    DcMotorEx pollenTurret;
    public static int nectarTurretVel = 1000;
    public static int pollenTurretVel = 1000;
    PinpointLocalizer pinpoint;

    ZoneCheck zoneCheck;
    @Override
    public void runOpMode() {
        nectarTurret = hardwareMap.get(DcMotorEx.class, "nectarTurret");
        pollenTurret = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        pinpoint = hardwareMap.get(PinpointLocalizer.class, "pinpoint");
        nectarTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zoneCheck = new ZoneCheck(Alliance.RED);


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