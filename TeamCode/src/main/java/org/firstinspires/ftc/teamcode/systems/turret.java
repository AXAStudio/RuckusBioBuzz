package org.firstinspires.ftc.teamcode.systems;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;

@Config
public class turret{

    public static double[] nectarTurretOffset = {3.22, 0};
    public static double[] pollenTurretOffset = {-5.91, 0};

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
    private Alliance alliance;

    private final TurretServo pollenTurret;
    private final TurretServo nectarTurret;

    private boolean pollenAimed;
    private boolean nectarAimed;

    public class aimer {
        // 1. Attributes (fields) that make up your data type
        public double theta;
        public double distance;

        // 2. Constructor to initialize your data type
        public aimer(double theta, double distance) {
            this.theta = theta;
            this.distance = distance;
        }
    }

    public turret(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        pollenTurret = new TurretServo(hardwareMap.get(Servo.class, "pollenTurretServo"));
        nectarTurret = new TurretServo(hardwareMap.get(Servo.class, "nectarTurretServo"));
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

//in form dist, theta
    public void update(){
        pollenAimed = pollenTurret.track(aimPollen().theta, POLLEN_TURRET_CENTER, POLLEN_TURRET_REVERSED, POLLEN_TURRET_GEAR_RATIO);
        nectarAimed = nectarTurret.track(aimNectar().theta, NECTAR_TURRET_CENTER, NECTAR_TURRET_REVERSED, NECTAR_TURRET_GEAR_RATIO);
    }

    public void center(){
        pollenTurret.center(POLLEN_TURRET_CENTER);
        nectarTurret.center(NECTAR_TURRET_CENTER);
        pollenAimed = false;
        nectarAimed = false;
    }

    public boolean unwinding(){
        return pollenTurret.unwinding() || nectarTurret.unwinding();
    }
    public aimer aimNectar() {
        return aimFrom(predictor.leadPose(nectarTurretOffset));
    }

    public aimer aimPollen() {
        return aimFrom(predictor.leadPose(pollenTurretOffset));
    }

    public boolean pollenAimed() { return pollenAimed; }
    public boolean nectarAimed() { return nectarAimed; }

    private aimer aimFrom(Pose turret) {
        Pose target = HivePosition.target(predictor.leadPose(), alliance);
        double dx = target.x() - turret.x();
        double dy = target.y() - turret.y();
        double dist = Math.hypot(dx, dy);
        double theta = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - turret.heading());
        return new aimer(theta, dist);
    }

    private static final class TurretServo {
        private final Servo servo;
        private double position = Double.NaN;
        private long settleUntilNanos;

        TurretServo(Servo servo) {
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
        public void center(double center){
            position = center;
            servo.setPosition(center);
        }

        boolean unwinding() {
            return System.nanoTime() < settleUntilNanos;
        }

        private static double clamp(double position) {
            return Math.max(0, Math.min(1, position));
        }
    }

}
