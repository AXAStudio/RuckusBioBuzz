package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class zoneCheck {
    public static double ZONE_Y = 39;
    public static final double FIELD_SIZE = 141.5;
    public static final double ROBOT_LENGTH = 18;
    public static final double ROBOT_WIDTH = 18;

    private final predictiveAiming predictor;

    public zoneCheck(predictiveAiming predictor) {
        this.predictor = predictor;
    }

    public boolean predictedInZone() {
        return inZone(predictor.predictPose());
    }

    public static boolean inZone(Pose pose) {
        double reach = ROBOT_LENGTH / 2 * Math.abs(Math.sin(pose.heading()))
            + ROBOT_WIDTH / 2 * Math.abs(Math.cos(pose.heading()));
        return pose.y() - reach <= ZONE_Y || pose.y() + reach >= FIELD_SIZE - ZONE_Y;
    }
}
