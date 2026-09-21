package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;

@Config
public class shootingRegression {

    public static double[] DISTANCES = {24, 48, 72, 96, 120};
    public static double[] VELOCITIES = {1250, 1250, 1250, 1250, 1250};

    private final aimingSystem aiming;

    public shootingRegression(aimingSystem aiming) {
        this.aiming = aiming;
    }

    public double launcherVelocity() {
        return velocityAt(distance());
    }

    public double distance() {
        return aiming.aim()[0];
    }

    public static double velocityAt(double distance) {
        int n = Math.min(DISTANCES.length, VELOCITIES.length);
        if (n == 0) return 0;
        if (distance <= DISTANCES[0]) return VELOCITIES[0];
        for (int i = 1; i < n; i++) {
            if (distance <= DISTANCES[i]) {
                double t = (distance - DISTANCES[i - 1]) / (DISTANCES[i] - DISTANCES[i - 1]);
                return VELOCITIES[i - 1] + t * (VELOCITIES[i] - VELOCITIES[i - 1]);
            }
        }
        return VELOCITIES[n - 1];
    }
}
