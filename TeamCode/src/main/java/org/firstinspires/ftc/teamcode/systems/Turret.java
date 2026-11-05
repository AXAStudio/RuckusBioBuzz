package org.firstinspires.ftc.teamcode.systems;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;

@Config
@Configurable
public class Turret {

    public static double[] nectarTurretOffset = {3.22, 0};
    public static double[] pollenTurretOffset = {-5.91, 0};

    public static double POLLEN_TURRET_CENTER = 0.5;
    public static double NECTAR_TURRET_CENTER = 0.5;
    public static double POLLEN_TURRET_MIN = 0.0;
    public static double POLLEN_TURRET_MAX = 1.0;
    public static double NECTAR_TURRET_MIN = 0.0;
    public static double NECTAR_TURRET_MAX = 1.0;
    public static double POLLEN_TURRET_START = 0.5;
    public static double NECTAR_TURRET_START = 0.5;
    public static boolean POLLEN_TURRET_REVERSED = false;
    public static boolean NECTAR_TURRET_REVERSED = false;
    public static double[] POLLEN_TURRET_GEAR_RATIO = {355, 370};
    public static double[] NECTAR_TURRET_GEAR_RATIO = {355, 370};
    public static double SERVO_RANGE_DEG = 358.6; // Axon Max MK1
    public static final double PWM_LOWER_US = 500, PWM_UPPER_US = 2500;
    public static double UNWIND_HYSTERESIS_DEG = 10;
    public static double UNWIND_SETTLE_S = 1.0;

    private final PredictiveAiming predictor;
    private Alliance alliance;

    private final TurretServo pollenTurret;
    private final TurretServo nectarTurret;
    private final HivePosition hivePosition;

    private boolean pollenAimed;
    private boolean nectarAimed;

    public class aimer {
        public double theta;
        public double distance;

        public aimer(double theta, double distance) {
            this.theta = theta;
            this.distance = distance;
        }
    }

    public static void configurePwm(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(PWM_LOWER_US, PWM_UPPER_US));
        }
    }

    public Turret(HardwareMap hardwareMap, PredictiveAiming predictor, HivePosition hivePosition) {
        this.predictor = predictor;
        pollenTurret = new TurretServo(hardwareMap.get(Servo.class, "pollenTurretServo"),
                limit(POLLEN_TURRET_START, POLLEN_TURRET_MIN, POLLEN_TURRET_MAX));
        nectarTurret = new TurretServo(hardwareMap.get(Servo.class, "nectarTurretServo"),
                limit(NECTAR_TURRET_START, NECTAR_TURRET_MIN, NECTAR_TURRET_MAX));
        this.hivePosition = hivePosition;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

//in form dist, theta
    public void update(){
        pollenAimed = pollenTurret.track(aimPollen().theta, POLLEN_TURRET_CENTER, POLLEN_TURRET_REVERSED, POLLEN_TURRET_GEAR_RATIO,
                POLLEN_TURRET_MIN, POLLEN_TURRET_MAX);
        nectarAimed = nectarTurret.track(aimNectar().theta, NECTAR_TURRET_CENTER, NECTAR_TURRET_REVERSED, NECTAR_TURRET_GEAR_RATIO,
                NECTAR_TURRET_MIN, NECTAR_TURRET_MAX);
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
        Pose target = hivePosition.target(predictor.leadPose(), alliance);
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

        TurretServo(Servo servo, double start) {
            this.servo = servo;
            configurePwm(servo); // SDK default 600-2400 us only reaches ~90% of the Axon's travel
            position = start;
            servo.setPosition(start);
        }

        boolean track(double theta, double center, boolean reversed, double[] gearRatio, double min, double max) {
            double lo = Math.max(0, Math.min(min, max)), hi = Math.min(1, Math.max(min, max)); //tolerate swapped limits
            min = lo;
            max = hi;
            double range = SERVO_RANGE_DEG * gearRatio[1] / gearRatio[0]; //turret degrees across the full servo travel 0-1
            double turn = 360.0 / range; //one full turret revolution in servo position units
            double last = Double.isNaN(position) ? center : position; //last commanded position or center on the first call
            double base = center + (reversed ? -1 : 1) * Math.toDegrees(theta) / range; //servo position that points at theta
            double follow = base + Math.round((last - base) / turn) * turn; //same aim shifted by whole turns to the copy closest to last
            double overshoot = Math.max(min - follow, follow - max); //how far follow is past a limit; <= 0 means it is reachable
            boolean aimed = false;

            if (overshoot <= 0) {
                position = follow;
                aimed = true;
            } else if (overshoot <= UNWIND_HYSTERESIS_DEG / range) {
                position = limit(follow, min, max);
            } else {
                double unwound = follow - Math.signum(follow - (min + max) / 2) * turn;
                if (unwound >= min && unwound <= max) {
                    position = unwound;
                    settleUntilNanos = System.nanoTime() + (long) (UNWIND_SETTLE_S * 1e9);
                } else {
                    position = limit(follow, min, max);
                }
            }
            servo.setPosition(position);
            return aimed && System.nanoTime() >= settleUntilNanos;
        }

        boolean unwinding() {
            return System.nanoTime() < settleUntilNanos;
        }

    }

    static double limit(double position, double min, double max) {
        double lo = Math.max(0, Math.min(min, max)), hi = Math.min(1, Math.max(min, max));
        return Math.max(lo, Math.min(hi, position));
    }

}
