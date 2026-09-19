package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

@Config
public class predictiveAiming {

    public static double PREDICT_MS = 150;
    private static final double[] CENTER = {0, 0};

    private final Follower follower;

    public predictiveAiming(Follower follower) {
        this.follower = follower;
    }
    public Pose predictPose() {
        return predictPose(PREDICT_MS);
    }
    public Pose predictPose(double ms) {
        return follower.pose().exp(follower.twist(), ms / 1000.0);
    }

    public Pose leadPose() {
        return leadPose(CENTER);
    }

    public Pose leadPose(double[] offset) {
        Pose robot = follower.pose();
        Velocity v = follower.velocity();
        double t = PREDICT_MS / 1000.0;
        double c = Math.cos(robot.heading());
        double s = Math.sin(robot.heading());
        double rx = offset[0] * c - offset[1] * s;
        double ry = offset[0] * s + offset[1] * c;
        double vx = v.vx - v.omega * ry;
        double vy = v.vy + v.omega * rx;
        return new Pose(robot.x() + rx + vx * t, robot.y() + ry + vy * t, robot.heading());
    }
}
