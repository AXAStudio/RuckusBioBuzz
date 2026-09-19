package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/**
 * Where the hive is. {@link #target(Pose, Alliance)} picks the hive x from the alliance and the
 * up or down position from the half of the field the robot is on. Only the two x values can be
 * changed, from FTC Dashboard.
 */
@Config
public class HivePosition {
    public static double xred = 58;
    public static double xblue = 84;

    private static final double UP_POS = 90;
    private static final double DOWN_POS = 52;
    /** Field half line in y: below it the target is DOWN_POS, at or above it UP_POS. */
    private static final double HALF_Y = 71;

    private HivePosition() {}

    /** The hive point to aim at from {@code robot}: the alliance's x, down below the half, else up. */
    public static Pose target(Pose robot, Alliance alliance) {
        double x = alliance == Alliance.RED ? xred : xblue;
        return new Pose(x, robot.y() < HALF_Y ? DOWN_POS : UP_POS);
    }
}
