package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
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
    public static PIDFCoefficients POLLEN_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);
    public static PIDFCoefficients NECTAR_PIDF = new PIDFCoefficients(500, 0, 0, 11.7);

    public static double POLLEN_TURRET_CENTER = 0.5;
    public static double NECTAR_TURRET_CENTER = 0.5;
    public static boolean POLLEN_TURRET_REVERSED = false;
    public static boolean NECTAR_TURRET_REVERSED = false;
    public static double[] POLLEN_TURRET_GEAR_RATIO = {355, 370};
    public static double[] NECTAR_TURRET_GEAR_RATIO = {355, 370};
    public static double SERVO_RANGE_DEG = 355; // Axon Max MK1
    public static double UNWIND_HYSTERESIS_DEG = 10;
    public static double UNWIND_SETTLE_S = 1.0;

    private final predictiveAiming predictor;
    private final zoneCheck zone;
    private aimingSystem aiming;
    private shootingRegression regression;
    private gate shooterGate;

    private final DcMotorEx pollenFlywheel;
    private final DcMotorEx nectarFlywheel;
    private final Turret pollenTurret;
    private final Turret nectarTurret;

    private boolean inZone;
    private boolean pollenAimed;
    private boolean nectarAimed;
    private double pollenTarget;
    private double nectarTarget;
    private double pollenVelocity;
    private double nectarVelocity;
    private enum states {
            RESET, TRACK, SHOOT
    }
    states state = states.TRACK;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor, boolean isPollen) {
        this.predictor = predictor;
        this.zone = new zoneCheck(predictor);

        shooterGate = new gate(hardwareMap);
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenTurret");
        nectarFlywheel = hardwareMap.get(DcMotorEx.class, "nectarTurret");
        pollenTurret = new Turret(hardwareMap.get(Servo.class, "pollenTurretServo"));
        nectarTurret = new Turret(hardwareMap.get(Servo.class, "nectarTurretServo"));

        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocityPidf(pollenFlywheel, POLLEN_PIDF);
        setVelocityPidf(nectarFlywheel, NECTAR_PIDF);

    }

    public void setAlliance(Alliance alliance) {
        aiming = new aimingSystem(predictor, alliance);
        regression = new shootingRegression(aiming);
    }

    public void update() {
        pollenFlywheel.setDirection(POLLEN_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        nectarFlywheel.setDirection(NECTAR_FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        inZone = zone.predictedInZone();
        pollenAimed = false;
        nectarAimed = false;

        pollenTarget = inZone ? regression.shooterspeedpollen() : 0;
        nectarTarget = inZone ? regression.shooterspeednectar() : 0;
        pollenFlywheel.setVelocity(pollenTarget);
        nectarFlywheel.setVelocity(nectarTarget);

        pollenVelocity = pollenFlywheel.getVelocity();
        nectarVelocity = nectarFlywheel.getVelocity();
        boolean readyShoot = inZone && pollenAimed && nectarAimed
                && atSpeed(pollenVelocity, pollenTarget); //only for pollen need to make one for necter
        boolean unwinding = pollenTurret.unwinding() && !nectarTurret.unwinding();
        switch(state){
            case RESET:
                //hold the servos where the unwind sent them
                shooterGate.closeGate();
                if (!unwinding) state = states.TRACK;
                break;
            case TRACK:
                shooterGate.closeGate();
                if(readyShoot){
                    state = states.SHOOT;
                }
                break;
            case SHOOT:
                if (inZone) {
                    pollenAimed = pollenTurret.track(aiming.aimPollen()[1],
                        POLLEN_TURRET_CENTER, POLLEN_TURRET_REVERSED, POLLEN_TURRET_GEAR_RATIO);
                    nectarAimed = nectarTurret.track(aiming.aimNecter()[1],
                        NECTAR_TURRET_CENTER, NECTAR_TURRET_REVERSED, NECTAR_TURRET_GEAR_RATIO);
                } else {
                    pollenTurret.center(POLLEN_TURRET_CENTER);
                    nectarTurret.center(NECTAR_TURRET_CENTER);
                }
                shooterGate.openGate();
                //add shooting code

                if (unwinding) state = states.RESET;
                else if (readyShoot) state = states.SHOOT;// is the flywheel at speed?
                else state = states.TRACK;
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

        boolean track(double theta, double center, boolean reversed, double[] gearRatio) {
            double range = SERVO_RANGE_DEG * gearRatio[1] / gearRatio[0]; //turret degrees across the full servo travel 0-1
            double turn = 360.0 / range; //one full turret revolution in servo position units
            double last = Double.isNaN(position) ? center : position; //last commanded position or center on the first call
            double base = center + (reversed ? -1 : 1) * Math.toDegrees(theta) / range; //servo position that points at theta
            double follow = base + Math.round((last - base) / turn) * turn; //same aim shifted by whole turns to the copy closest to last
            double overshoot = Math.max(-follow, follow - 1); //how far follow is past 0 or 1 and <= 0 means it is reachable
            boolean aimed = false;

            if (overshoot <= 0) {
                position = follow;
                aimed = true;
            } else if (overshoot <= UNWIND_HYSTERESIS_DEG / range) {
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

        boolean unwinding() {
            return System.nanoTime() < settleUntilNanos;
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
