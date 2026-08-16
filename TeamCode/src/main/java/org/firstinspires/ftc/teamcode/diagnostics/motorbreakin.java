package org.firstinspires.ftc.teamcode.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "motorbreakin", group = "TeleOp")
@Config
public class motorbreakin extends OpMode {
    private static final double NORMAL_SPEED = 1.0;
    private Follower follower;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }


    @Override
    public void loop() {
        follower.setTeleOpDrive(1,0,0);
    }
}
