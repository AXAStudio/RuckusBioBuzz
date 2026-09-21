package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{

    public static double[] launcherOffset = {0, 0};

    public static double LAUNCHER_HEADING = 0;

    private final predictiveAiming predictor;
    private final Alliance alliance;
    public aimingSystem(predictiveAiming predictor, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
    }

    public double[] aim() {
        return aimFrom(predictor.leadPose(launcherOffset));
    }

    public double targetHeading() {
        return AngleUnit.normalizeRadians(aim()[1] - LAUNCHER_HEADING);
    }

    public double[] aimFrom(Pose launcher) {
        Pose target = HivePosition.target(predictor.leadPose(), alliance);
        double dx = target.x() - launcher.x();
        double dy = target.y() - launcher.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx));
        return new double[]{dist, theta};
    }
}
