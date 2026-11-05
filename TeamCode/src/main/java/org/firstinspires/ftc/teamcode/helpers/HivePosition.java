package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.math.Pose;

public class HivePosition {


    public HivePosition() {}

    public static Pose target(Pose robot, Alliance alliance) {
        double x = alliance == Alliance.RED ? SystemConstants.HivePosition.xred : SystemConstants.HivePosition.xblue;
        return new Pose(x, robot.y() < SystemConstants.HivePosition.HALF_Y ? SystemConstants.HivePosition.DOWN_POS : SystemConstants.HivePosition.UP_POS);
    }
}
