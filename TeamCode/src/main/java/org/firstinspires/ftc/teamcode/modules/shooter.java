package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class shooter {
    public static double POLLEN_P = 0;
    public static double POLLEN_I = 0;
    public static double POLLEN_D = 0;
    public static double POLLEN_F = 0;
    public static boolean POLLEN_REVERSED = false;

    public static double NECTAR_P = 0;
    public static double NECTAR_I = 0;
    public static double NECTAR_D = 0;
    public static double NECTAR_F = 0;
    public static boolean NECTAR_REVERSED = false;

    private final shootingRegression regression;
    private final Flywheel pollen;
    private final Flywheel nectar;

    public shooter(HardwareMap hardwareMap, shootingRegression regression) {
        this.regression = regression;
        this.pollen = new Flywheel(hardwareMap.get(DcMotorEx.class, "pollenTurret"));
        this.nectar = new Flywheel(hardwareMap.get(DcMotorEx.class, "nectarTurret"));
    }

    public void rev() {
        pollen.run(regression.shooterspeedpollen(),
            POLLEN_P, POLLEN_I, POLLEN_D, POLLEN_F, POLLEN_REVERSED);
        nectar.run(regression.shooterspeednectar(),
            NECTAR_P, NECTAR_I, NECTAR_D, NECTAR_F, NECTAR_REVERSED);
    }

    public void stop() {
        pollen.stop();
        nectar.stop();
    }

    public double pollenVelocity() { return pollen.velocity; }
    public double nectarVelocity() { return nectar.velocity; }
    public double pollenTarget() { return pollen.target; }
    public double nectarTarget() { return nectar.target; }

    private static final class Flywheel {
        private final DcMotorEx motor;
        private double integral;
        private double lastError;
        private long lastNanos;
        double target;
        double velocity;

        Flywheel(DcMotorEx motor) {
            this.motor = motor;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        void run(double target, double p, double i, double d, double f, boolean reversed) {
            motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

            this.target = target;
            velocity = motor.getVelocity();
            double error = target - velocity;

            long now = System.nanoTime();
            double dt = lastNanos == 0 ? 0 : (now - lastNanos) / 1e9;
            lastNanos = now;

            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            lastError = error;

            double power = f * target + p * error + i * integral + d * derivative;

            if (dt > 0 && Math.abs(power) < 1) {
                integral += error * dt;
            }
            motor.setPower(Math.max(-1, Math.min(1, power)));
        }

        void stop() {
            motor.setPower(0);
            integral = 0;
            lastError = 0;
            lastNanos = 0;
            target = 0;
        }
    }
}
