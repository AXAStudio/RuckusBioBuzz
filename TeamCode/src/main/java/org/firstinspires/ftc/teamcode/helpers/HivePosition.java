package org.firstinspires.ftc.teamcode.helpers;

/**
 * Where the hive is. Only {@link #x} can be changed; the up and down positions are read-only.
 *
 * <p>x is public and non-final so FTC Dashboard can edit it when the instance is held in a
 * {@code public static} field of a {@code @Config} class - the shared one is
 * {@code aimingSystem.hivePos}. upPos and downPos are final, so neither code nor the dashboard
 * can change them. Read them from anywhere as {@code aimingSystem.hivePos.x} /
 * {@code .upPos} / {@code .downPos}.
 */
public class HivePosition {
    public double x = 58;

    public final double upPos = 90;
    public final double downPos = 52;

    public HivePosition() {}

    /**
     * @param x The hive's X coordinate
     */
    public HivePosition(double x) {
        this.x = x;
    }
}
