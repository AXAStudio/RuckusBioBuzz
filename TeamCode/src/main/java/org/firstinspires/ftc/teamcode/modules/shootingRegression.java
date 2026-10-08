package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.systems.turret;

public class shootingRegression {

    private final turret aiming;

    public shootingRegression(turret aiming) {
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
