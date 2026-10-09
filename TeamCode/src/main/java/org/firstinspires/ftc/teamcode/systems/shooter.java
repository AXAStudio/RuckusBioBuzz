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
    private final flywheel flywheel;
    private final shootingRegression regression;
    private gate shooterGate;

    private boolean inZone;

    private enum states {
            RESET, TRACK, SHOOT
    }
    states state = states.TRACK;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        this.zone = new zoneCheck(predictor);
        this.flywheel = new flywheel(hardwareMap, predictor);
        aiming = new turret(hardwareMap, predictor);
        regression = new shootingRegression(aiming);

    }

    public void setAlliance(Alliance alliance) {
        aiming.setAlliance(alliance);
    }

    public void update() {

    }

    public boolean ready() {
        return state == states.SHOOT;
    }

    public boolean inZone() { return inZone; }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }


    private static boolean atSpeed(double velocity, double target) {
        return target > 0 && Math.abs(velocity - target) <= AT_SPEED_FRACTION * target;
    }
}
