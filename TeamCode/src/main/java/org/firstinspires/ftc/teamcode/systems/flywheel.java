package org.firstinspires.ftc.teamcode.systems;


import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.shootingRegression;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

public class flywheel {
    public static boolean POLLEN_FLYWHEEL_REVERSED = false;
    public static boolean NECTAR_FLYWHEEL_REVERSED = false;
    public static double AT_SPEED_FRACTION = 0.05;
    public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);

    private final predictiveAiming predictor;
    private final turret aiming;
    private final shootingRegression regression;
    private gate shooterGate;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    public flywheel(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        aiming = new turret(hardwareMap, predictor);
        regression = new shootingRegression(aiming);

        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");

        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, NECTAR_PIDF);

    }
    private static void setVelocityPidf(DcMotorEx flywheel, PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.P, pidf.I, pidf.D, pidf.F));
    }
    public void update(){
        pollenFlywheel.setVelocity(regression.shooterspeedpollen());
        pollenFlywheel.setVelocity(regression.shooterspeednectar());
    }
    public boolean atSpeedPollen() { return pollenFlywheel.getVelocity() == regression.shooterspeedpollen(); }
    public boolean atSpeedNectar() { return nectarFlywheel.getVelocity() == regression.shooterspeednectar(); }

}
