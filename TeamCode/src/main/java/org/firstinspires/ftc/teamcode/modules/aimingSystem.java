package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{

    public static double[] nectarTurretOffset = {0, 0};
    public static double[] pollenTurretOffset = {0, 0};

    private final predictiveAiming predictor;
    private final Alliance alliance;
    public aimingSystem(predictiveAiming predictor, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
    }


    public double[] aim() {
        return aimFrom(predictor.leadPose(nectarTurretOffset));
    }

    public double[] aimPollen() {
        return aimFrom(predictor.leadPose(pollenTurretOffset));
    }

    private double[] aimFrom(Pose turret) {
        Pose target = HivePosition.target(predictor.leadPose(), alliance);
        double dx = target.x() - turret.x();
        double dy = target.y() - turret.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - turret.heading());
        return new double[]{dist, theta};
    }
}
