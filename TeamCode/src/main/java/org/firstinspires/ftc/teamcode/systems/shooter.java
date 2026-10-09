package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.shootingRegression;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

@Config
public class shooter {
    public static boolean POLLEN_FLYWHEEL_REVERSED = false;
    public static boolean NECTAR_FLYWHEEL_REVERSED = false;
    public static double AT_SPEED_FRACTION = 0.05;
    public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);

    private final predictiveAiming predictor;
    private final zoneCheck zone;
    private final turret aiming;
    private final shootingRegression regression;
    private gate shooterGate;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;

    private boolean inZone;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    private enum states {
            RESET, TRACK, SHOOT
    }
    states state = states.TRACK;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        this.zone = new zoneCheck(predictor);
        aiming = new turret(hardwareMap, predictor);
        regression = new shootingRegression(aiming);

        shooterGate = new gate(hardwareMap);
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");

        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, NECTAR_PIDF);

    }

    public void setAlliance(Alliance alliance) {
        aiming.setAlliance(alliance);
    }

    public void update() {
        pollenFlywheel.setDirection(POLLEN_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(NECTAR_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        inZone = zone.predictedInZone();
        pollenTarget = inZone ? regression.shooterspeedpollen() : 0;
        nectarTarget = inZone ? regression.shooterspeednectar() : 0;
        pollenFlywheel.setVelocity(pollenTarget);
        nectarFlywheel.setVelocity(nectarTarget);
        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();

        switch (state) {
            case RESET:
                shooterGate.closeGate();
                if (!aiming.unwinding()) state = states.TRACK;
                break;
            case TRACK:
            case SHOOT:
                if (inZone) {
                    aiming.update();
                } else {
                    aiming.center();
                }
                boolean readyShoot = inZone && aiming.pollenAimed() && aiming.nectarAimed()
                        && atSpeed(pollenVelocity, pollenTarget) && atSpeed(nectarVelocity, nectarTarget);
                if (aiming.unwinding()) {
                    state = states.RESET;
                } else if (readyShoot) {
                    state = states.SHOOT;
                } else {
                    state = states.TRACK;
                }
                if (state == states.SHOOT) {
                    shooterGate.openGate();
                } else {
                    shooterGate.closeGate();
                }
                break;
        }
    }

    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.P, pidf.I, pidf.D, pidf.F));
    }

    public boolean ready() {
        return state == states.SHOOT;
    }

    public void stop() {
        pollenFlywheel.setPower(0);
        nectarFlywheel.setPower(0);
    }

    public boolean inZone() { return inZone; }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
    public double pollenTarget() { return pollenTarget; }
    public double nectarTarget() { return nectarTarget; }
    public double pollenVelocity() { return pollenVelocity; }
    public double nectarVelocity() { return nectarVelocity; }

    private static boolean atSpeed(double velocity, double target) {
        return target > 0 && Math.abs(velocity - target) <= AT_SPEED_FRACTION * target;
    }
}
