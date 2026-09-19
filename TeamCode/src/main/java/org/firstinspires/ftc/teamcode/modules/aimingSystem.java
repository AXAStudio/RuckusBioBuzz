package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{

    public static double[] nectarTurretOffset = {0, 0};
    public static double[] pollenTurretOffset = {0, 0};

    private final Follower follower;
    private final Alliance alliance;
    public aimingSystem(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
    }


    public double[] aim(Pose robot) {
        return aimFrom(robot, nectarTurretPose(robot));
    }

    public double[] aimPollen(Pose robot) {
        return aimFrom(robot, pollenTurretPose(robot));
    }

    private double[] aimFrom(Pose robot, Pose turret) {
        Pose target = HivePosition.target(robot, alliance);
        double dx = target.x() - turret.x();
        double dy = target.y() - turret.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robot.heading());
        return new double[]{dist, theta};
    }

    public static Pose nectarTurretPose(Pose robot) {
        return turretPose(robot, nectarTurretOffset);
    }

    public static Pose pollenTurretPose(Pose robot) {
        return turretPose(robot, pollenTurretOffset);
    }

    private static Pose turretPose(Pose robot, double[] offset) {
        double c = Math.cos(robot.heading());
        double s = Math.sin(robot.heading());
        double ox = offset[0];
        double oy = offset[1];
        return new Pose(robot.x() + ox * c - oy * s, robot.y() + ox * s + oy * c, robot.heading());
    }
}
