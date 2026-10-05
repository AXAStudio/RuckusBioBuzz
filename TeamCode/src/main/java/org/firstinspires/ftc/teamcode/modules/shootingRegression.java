package org.firstinspires.ftc.teamcode.modules;

public class shootingRegression {

    private final aimingSystem aiming;

    public shootingRegression(aimingSystem aiming) {
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
        return aiming.aimPollen()[0];
    }

    public double nectarDistance() {
        return aiming.aimNecter()[0];
    }
}
