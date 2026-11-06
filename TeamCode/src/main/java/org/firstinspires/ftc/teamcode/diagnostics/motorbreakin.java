package org.firstinspires.ftc.teamcode.diagnostics;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Configurable
public class motorbreakin extends OpMode {
    private static final double NORMAL_SPEED = 1.0;
    private Follower follower;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }


    // Ported as-is from Pedro 2, including that nothing here calls follower.update(): under
    // Pedro 2 setTeleOpDrive only stored the command, and under Pedro 3 manual() only stores it
    // too, so this OpMode has never actually driven the motors.
    @Override
    public void loop() {
        follower.manual(new DrivePowers(1, 0, 0));
    }
}
