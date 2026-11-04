package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Config
@Configurable
public class HivePosition {
    public static double xred = 58;
    public static double xblue = 84;

    private static final double UP_POS = 90;
    private static final double DOWN_POS = 52;
    private static final double HALF_Y = 71;

    public HivePosition() {}

    public static Pose target(Pose robot, Alliance alliance) {
        double x = alliance == Alliance.RED ? xred : xblue;
        return new Pose(x, robot.y() < HALF_Y ? DOWN_POS : UP_POS);
    }
}
