package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.systems.Turret;

public class ShootingRegression {

    private final Turret aiming;

    public ShootingRegression(Turret aiming) {
        this.aiming = aiming;
    }

    public double shooterspeedpollen() {
        double distance = pollenDistance();
        int speed = (int)distance;
        return speed;
    }

    public double shooterspeednectar() {
        double distance = nectarDistance();
        int speed = (int)distance;
        return speed; //regression
    }

    public double pollenDistance() {
        return aiming.aimPollen().distance;
    }

    public double nectarDistance() {
        return aiming.aimNectar().distance;
    }
}
