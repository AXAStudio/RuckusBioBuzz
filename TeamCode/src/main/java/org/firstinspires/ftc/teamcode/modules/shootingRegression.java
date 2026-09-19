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
        double distance = predictedPollenDistance();
        return distance;
    }

    public double shooterspeednectar() {
        double distance = predictedNectarDistance();
        return distance;
    }

    public double predictedPollenDistance() {
        Pose future = predictor.predictPose();
        return aimingSystem.pollenTurretPose(future).distance(HivePosition.target(future, alliance));
    }

    public double predictedNectarDistance() {
        Pose future = predictor.predictPose();
        return aimingSystem.nectarTurretPose(future).distance(HivePosition.target(future, alliance));
    }
}
