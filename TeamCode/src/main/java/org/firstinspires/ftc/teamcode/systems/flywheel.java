package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.shootingRegression;

@Config
public class flywheel {
    public static boolean POLLEN_FLYWHEEL_REVERSED = false;
    public static boolean NECTAR_FLYWHEEL_REVERSED = false;
    public static double AT_SPEED_FRACTION = 0.05;
    public static double IDLE_FRACTION = 0.5;
    public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);

    private final shootingRegression regression;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    public flywheel(HardwareMap hardwareMap, turret aiming) {
        regression = new shootingRegression(aiming);

        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");

        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, NECTAR_PIDF);

    }
    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.P, pidf.I, pidf.D, pidf.F));
    }
    public void update(boolean inZone){
        pollenFlywheel.setDirection(POLLEN_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(NECTAR_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        double scale = inZone ? 1 : IDLE_FRACTION;
        pollenTarget = regression.shooterspeedpollen();
        nectarTarget = regression.shooterspeednectar();
        pollenFlywheel.setVelocity(pollenTarget * scale);
        nectarFlywheel.setVelocity(nectarTarget * scale);
        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();
    }
    public boolean atSpeedPollen() { return atSpeedPollen(AT_SPEED_FRACTION); }
    public boolean atSpeedNectar() { return atSpeedNectar(AT_SPEED_FRACTION); }
    /** Same check with a caller-chosen band, e.g. the wider band shooter holds SHOOT with. */
    public boolean atSpeedPollen(double fraction) { return atSpeed(pollenVelocity, pollenTarget, fraction); }
    public boolean atSpeedNectar(double fraction) { return atSpeed(nectarVelocity, nectarTarget, fraction); }

    private static boolean atSpeed(double velocity, double target, double fraction) {
        return target > 0 && Math.abs(velocity - target) <= fraction * target;
    }
}
