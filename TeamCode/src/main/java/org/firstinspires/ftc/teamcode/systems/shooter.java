package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.aimingSystem;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.shootingRegression;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

@Config
public class shooter {
    public static boolean LAUNCHER_REVERSED = false;
    public static boolean WINDMILL_REVERSED = true;

    public static double LAUNCHER_P = 40;
    public static double LAUNCHER_I = 0;
    public static double LAUNCHER_D = 0;
    public static double LAUNCHER_F = 12.5;

    public static double AT_SPEED_TOLERANCE = 50;
    public static double AIM_TOLERANCE_DEG = 3;
    public static double WINDMILL_FEED_POWER = 1.0;

    public static boolean REQUIRE_ZONE = true;
    public static boolean REQUIRE_AIM = true;

    private final predictiveAiming predictor;
    private final zoneCheck zone;
    private aimingSystem aiming;
    private shootingRegression regression;

    private final DcMotorEx launcher;
    private final CRServo windmill;

    private double appliedP = Double.NaN, appliedI, appliedD, appliedF;

    private boolean shooting;
    private boolean inZone;
    private boolean aimed;
    private boolean feeding;
    private double target;
    private double velocity;
    private double aimErrorDeg;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.predictor = predictor;
        this.zone = new zoneCheck(predictor);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyPidf();
        windmill.setPower(0);
    }

    public void setAlliance(Alliance alliance) {
        aiming = new aimingSystem(predictor, alliance);
        regression = new shootingRegression(aiming);
    }

    public double targetHeading() {
        return aiming.targetHeading();
    }

    public void update(boolean shooting, double headingRad) {
        this.shooting = shooting;
        launcher.setDirection(LAUNCHER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        windmill.setDirection(WINDMILL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        applyPidf();

        inZone = zone.predictedInZone();
        aimErrorDeg = Math.toDegrees(AngleUnit.normalizeRadians(aiming.targetHeading() - headingRad));
        aimed = Math.abs(aimErrorDeg) <= AIM_TOLERANCE_DEG;

        target = shooting ? regression.launcherVelocity() : 0;
        launcher.setVelocity(target);
        velocity = launcher.getVelocity();

        feeding = ready();
        windmill.setPower(feeding ? WINDMILL_FEED_POWER : 0);
    }

    public boolean ready() {
        return shooting
            && (inZone || !REQUIRE_ZONE)
            && (aimed || !REQUIRE_AIM)
            && target > 0 && velocity >= target - AT_SPEED_TOLERANCE;
    }

    public void stop() {
        launcher.setVelocity(0);
        launcher.setPower(0);
        windmill.setPower(0);
    }

    public boolean feeding() { return feeding; }
    public boolean inZone() { return inZone; }
    public boolean aimed() { return aimed; }
    public double aimErrorDeg() { return aimErrorDeg; }
    public double target() { return target; }
    public double velocity() { return velocity; }
    public double distance() { return regression.distance(); }

    private void applyPidf() {
        if (LAUNCHER_P == appliedP && LAUNCHER_I == appliedI
            && LAUNCHER_D == appliedD && LAUNCHER_F == appliedF) {
            return;
        }
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F));
        appliedP = LAUNCHER_P;
        appliedI = LAUNCHER_I;
        appliedD = LAUNCHER_D;
        appliedF = LAUNCHER_F;
    }
}
