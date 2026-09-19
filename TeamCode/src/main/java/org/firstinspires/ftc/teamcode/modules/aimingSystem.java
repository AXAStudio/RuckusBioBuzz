package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{
    public static int x = 58;

    private final Follower follower;
    public static HivePosition hivePos = new HivePosition(x);
    public aimingSystem(Follower follower) {
        this.follower = follower;
    }


    //return int array in the form of [target vel, target angle relative to front of bot (rad)]
    public double[] aim(Pose robot) {
        Pose target = hivePos.target(robot);
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robot.heading());
        return new double[]{dist, theta};
    }
}
