package org.firstinspires.ftc.teamcode.modules;

public class shootingRegression {

    private final aimingSystem aiming;

    public shootingRegression(aimingSystem aiming) {
        this.aiming = aiming;
    }

    public double shooterspeedpollen() {
        double distance = pollenDistance();
        return distance;
    }

    public double shooterspeednectar() {
        double distance = nectarDistance();
        return distance;
    }

    public double pollenDistance() {
        return aiming.aimPollen()[0];
    }

    public double nectarDistance() {
        return aiming.aim()[0];
    }
}
