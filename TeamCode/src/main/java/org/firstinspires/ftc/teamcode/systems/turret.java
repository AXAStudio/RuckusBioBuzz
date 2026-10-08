package org.firstinspires.ftc.teamcode.systems;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;

@Config
public class turret{

    public static double[] nectarTurretOffset = {0, 0};
    public static double[] pollenTurretOffset = {0, 0};

    private final predictiveAiming predictor;
    private final Alliance alliance;

    public class aimer {
        // 1. Attributes (fields) that make up your data type
        public double theta;
        public double distance;

        // 2. Constructor to initialize your data type
        public aimer(double theta, double distance) {
            this.theta = theta;
            this.distance = distance;
        }
    }

    public turret(predictiveAiming predictor, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
    }

//in form dist, theta
    public void update(){

    }
    public aimer aimNectar() {
        return aimFrom(predictor.leadPose(nectarTurretOffset));
    }

    public aimer aimPollen() {
        return aimFrom(predictor.leadPose(pollenTurretOffset));
    }

    private aimer aimFrom(Pose turret) {
        Pose target = HivePosition.target(predictor.leadPose(), alliance);
        double dx = target.x() - turret.x();
        double dy = target.y() - turret.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - turret.heading());
        return new aimer(dist, theta);
    }

}
