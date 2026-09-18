package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

/**
 * A helper class to store coordinate data for Pedro Pathing.
 * This class handles the conversion from human-readable degrees to robot-ready radians.
 */
public class KeyPosition {
    public double x = 0;
    public double y = 0;

    public KeyPosition() {}

    /**
     * @param x The X coordinate
     * @param y The Y coordinate
     * @param heading Degrees (automatically converted to Radians)
     */
    public KeyPosition(double x, double y, double heading) {
        this.x = x;
        this.y = y;
    }
}
