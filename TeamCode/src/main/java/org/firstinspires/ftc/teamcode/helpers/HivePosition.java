package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

/**
 * Where the hive is. {@link #target(Pose)} picks the up or down hive position from the half of
 * the field the robot is on; only {@link #x} can be changed.
 *
 * <p>x is public and non-final so FTC Dashboard can edit it when the instance is held in a
 * {@code public static} field of a {@code @Config} class - the shared one is
 * {@code aimingSystem.hivePos}.
 */
public class HivePosition {
    public double x = 58;

    private static final double UP_POS = 90;
    private static final double DOWN_POS = 52;
    /** Field half line in y: below it the target is DOWN_POS, at or above it UP_POS. */
    private static final double HALF_Y = 71;

    public HivePosition() {}

    /**
     * @param x The hive's X coordinate
     */
    public HivePosition(double x) {
        this.x = x;
    }

    /** The hive point to aim at from {@code robot}: the down position below the half, else up. */
    public Pose target(Pose robot) {
        return new Pose(x, robot.y() < HALF_Y ? DOWN_POS : UP_POS);
    }
}
