package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{


    private final Follower follower;
    private final Alliance alliance;
    public aimingSystem(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
    }


    //return double array in the form of [distance to hive (in), target angle relative to front of bot (rad)]
    public double[] aim(Pose robot) {
        Pose target = HivePosition.target(robot, alliance);
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robot.heading());
        return new double[]{dist, theta};
    }
}
