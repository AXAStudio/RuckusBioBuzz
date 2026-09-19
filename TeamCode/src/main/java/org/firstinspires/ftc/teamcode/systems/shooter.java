package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.aimingSystem;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.shootingRegression;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

@Config
public class shooter {
    public static boolean POLLEN_FLYWHEEL_REVERSED = false;
    public static boolean NECTAR_FLYWHEEL_REVERSED = false;
    public static double AT_SPEED_FRACTION = 0.05;

    public static double POLLEN_TURRET_CENTER = 0.5;
    public static double NECTAR_TURRET_CENTER = 0.5;
    public static boolean POLLEN_TURRET_REVERSED = false;
    public static boolean NECTAR_TURRET_REVERSED = false;
    public static double TURRET_RANGE_DEG = 355;
    public static double UNWIND_HYSTERESIS_DEG = 10;
    public static double UNWIND_SETTLE_S = 1.0;

    private final predictiveAiming predictor;
    private final zoneCheck zone;
    private aimingSystem aiming;
    private shootingRegression regression;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;
    private final Turret pollenTurret;
    private final Turret nectarTurret;

    private boolean shooting;
    private boolean inZone;
    private boolean pollenAimed;
    private boolean nectarAimed;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        this.zone = new zoneCheck(predictor);

        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");
        pollenTurret = new Turret(hardwareMap.get(Servo.class, "pollenTurretServo"));
        nectarTurret = new Turret(hardwareMap.get(Servo.class, "nectarTurretServo"));

        for (DcMotorEx flywheel : new DcMotorEx[]{pollenFlywheel, nectarFlywheel}) {
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setAlliance(Alliance alliance) {
        aiming = new aimingSystem(predictor, alliance);
        regression = new shootingRegression(aiming);
    }

    public void update(boolean shooting) {
        this.shooting = shooting;
        pollenFlywheel.setDirection(POLLEN_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(NECTAR_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        inZone = zone.predictedInZone();

        pollenAimed = false;
        nectarAimed = false;
        if (shooting && inZone) {
            pollenAimed = pollenTurret.track(aiming.aimPollen()[1],
                POLLEN_TURRET_CENTER, POLLEN_TURRET_REVERSED);
            nectarAimed = nectarTurret.track(aiming.aim()[1],
                NECTAR_TURRET_CENTER, NECTAR_TURRET_REVERSED);
        } else {
            pollenTurret.center(POLLEN_TURRET_CENTER);
            nectarTurret.center(NECTAR_TURRET_CENTER);
        }

        pollenTarget = shooting ? regression.shooterspeedpollen() : 0;
        nectarTarget = shooting ? regression.shooterspeednectar() : 0;
        pollenFlywheel.setVelocity(pollenTarget);
        nectarFlywheel.setVelocity(nectarTarget);

        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();
    }

    public boolean ready() {
        return shooting && inZone && pollenAimed && nectarAimed
            && atSpeed(pollenVelocity, pollenTarget) && atSpeed(nectarVelocity, nectarTarget);
    }

    public void stop() {
        pollenFlywheel.setPower(0);
        nectarFlywheel.setPower(0);
    }

    public boolean inZone() { return inZone; }
    public boolean pollenAimed() { return pollenAimed; }
    public boolean nectarAimed() { return nectarAimed; }
    public double pollenTarget() { return pollenTarget; }
    public double nectarTarget() { return nectarTarget; }
    public double pollenVelocity() { return pollenVelocity; }
    public double nectarVelocity() { return nectarVelocity; }

    private static boolean atSpeed(double velocity, double target) {
        return target > 0 && Math.abs(velocity - target) <= AT_SPEED_FRACTION * target;
    }

    private static final class Turret {
        private final Servo servo;
        private double position = Double.NaN;
        private long settleUntilNanos;

        Turret(Servo servo) {
            this.servo = servo;
        }

        boolean track(double theta, double center, boolean reversed) {
            double turn = 360.0 / TURRET_RANGE_DEG;
            double last = Double.isNaN(position) ? center : position;
            double base = center + (reversed ? -1 : 1) * Math.toDegrees(theta) / TURRET_RANGE_DEG;
            double follow = base + Math.round((last - base) / turn) * turn;
            double overshoot = Math.max(-follow, follow - 1);

            boolean aimed = false;
            if (overshoot <= 0) {
                position = follow;
                aimed = true;
            } else if (overshoot <= UNWIND_HYSTERESIS_DEG / TURRET_RANGE_DEG) {
                position = clamp(follow);
            } else {
                double unwound = follow - Math.signum(follow - 0.5) * turn;
                if (unwound >= 0 && unwound <= 1) {
                    position = unwound;
                    settleUntilNanos = System.nanoTime() + (long) (UNWIND_SETTLE_S * 1e9);
                } else {
                    position = clamp(follow);
                }
            }
            servo.setPosition(position);
            return aimed && System.nanoTime() >= settleUntilNanos;
        }

        void center(double center) {
            position = center;
            servo.setPosition(center);
        }

        private static double clamp(double position) {
            return Math.max(0, Math.min(1, position));
        }
    }
}
