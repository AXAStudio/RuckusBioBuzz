package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * A helper class to store coordinate data for Pedro Pathing.
 * Heading is stored in degrees so it reads and edits in degrees on FTC Dashboard;
 * {@link #toPose()} converts it to the radians Pedro uses.
 */
public class PathStep {
    public double x = 0;
    public double y = 0;
    public double heading = 0; // Degrees

    public PathStep() {}

    /**
     * @param x The X coordinate
     * @param y The Y coordinate
     * @param heading Degrees
     */
    public PathStep(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Converts this PathStep into a Pedro Pathing Pose object.
     * @return A new Pose object using the stored X and Y, and the heading in radians.
     */
    public Pose toPose() {
        return new Pose(x, y, Math.toRadians(heading));
    }
}
