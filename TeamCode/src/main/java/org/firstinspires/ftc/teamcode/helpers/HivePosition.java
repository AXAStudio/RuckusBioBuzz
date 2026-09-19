package org.firstinspires.ftc.teamcode.helpers;

/**
 * Where the hive is. Its x never changes, so it is the constant {@link #X}; the up and down
 * positions are editable.
 *
 * <p>The fields are public and non-final so FTC Dashboard can edit them when the instance is held
 * in a {@code public static} field of a {@code @Config} class - the shared one is
 * {@code aimingSystem.hivePos}. Read it from anywhere as {@code aimingSystem.hivePos.upPos} and
 * {@code HivePosition.X}.
 */
public class HivePosition {
    /** The hive's X coordinate. Constant, so it is not dashboard-editable. */
    public static final double X = 58;

    public double upPos = 0;
    public double downPos = 52;

    public HivePosition() {}

    /**
     * @param upPos The up position
     * @param downPos The down position
     */
    public HivePosition(double upPos, double downPos) {
        this.upPos = upPos;
        this.downPos = downPos;
    }
}
