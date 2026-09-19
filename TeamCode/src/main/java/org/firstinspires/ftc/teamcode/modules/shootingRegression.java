package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.math.Pose;

public class shootingRegression {

    private final predictiveAiming predictor;

    public shootingRegression(predictiveAiming predictor) {
        this.predictor = predictor;
    }

    public double shooterspeedpollen() {
        double distance = predictedDistance();
        return distance; //insert function here for pollen
    }

    public double shooterspeednectar() {
        double distance = predictedDistance();
        return distance; //insert function here for nectar
    }

    public double predictedDistance() {
        Pose future = predictor.predictPose();
        double hiveY = future.y() < 71
            ? aimingSystem.hivePos.downPos
            : aimingSystem.hivePos.upPos;
        return Math.hypot(aimingSystem.hivePos.x - future.x(), hiveY - future.y());
    }
}
