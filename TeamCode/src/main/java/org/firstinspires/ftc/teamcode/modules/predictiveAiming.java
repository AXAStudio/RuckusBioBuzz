package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

@Config
public class predictiveAiming {

    public static double PREDICT_MS = 150;

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
}
