package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

public class shootingRegression {

    private final predictiveAiming predictor;
    private final Alliance alliance;

    public shootingRegression(predictiveAiming predictor, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
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
        return future.distance(HivePosition.target(future, alliance));
    }
}
