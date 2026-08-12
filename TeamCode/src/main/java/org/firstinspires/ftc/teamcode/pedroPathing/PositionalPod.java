package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwervePod;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

/**
 * A swerve pod steered by a servo running its own internal position loop.
 *
 * <p>Sits alongside {@link CoaxialPod} rather than replacing it. Both implement
 * {@code SwervePod} and {@code FollowerBuilder.swerveDrivetrain} takes a varargs of that
 * interface, so a drivetrain can mix them - which is exactly what the one-pod A/B needs, and what
 * the fallback needs if positional mode turns out worse.
 *
 * <h2>Why</h2>
 *
 * <p>Every measured limit on the CR-mode pod traces to one physical fact: the steering path has no
 * creep regime. Static friction breaks to kinetic and the pod goes from stopped to 9-80 deg/s with
 * nothing between, so any external controller with the authority to correct a small error commits
 * to degrees of travel before feedback (39 ms transport + 42 ms velocity lag) can act. Six software
 * approaches failed against that. A servo in position mode closes its own loop internally, far
 * faster than a 20 ms PWM frame allows us to, so breakaway is its problem rather than ours.
 *
 * <h2>Travel window and the seam</h2>
 *
 * <p>{@code move()} treats a heading and that heading plus 180 degrees as the same demand, inverting
 * drive power for the second, so the pod only ever needs 180 degrees of travel. The servo is
 * programmed for about 200, and the extra 20 is deliberate: in that overlap band both
 * representations of a heading are reachable, which turns the changeover into a hysteresis band the
 * controller can schedule rather than an instant it is forced through.
 *
 * <p>A forced 180 degree traverse still happens when the demand crosses the window edge, costing
 * roughly 240 ms plus settling. It is placed away from the headings the pods actually occupy -
 * measured on this drivetrain as forward 90, strafe 0/180, and the X-lock at 46.5 and 133.5.
 *
 * <p>Install the travel so the encoder's own wrap sits outside the window too. The Axon's analog
 * output is non-monotonic across that wrap - it produced a spurious 1452 deg/s reading during slew
 * characterisation - so putting both dead zones in the same unreachable place costs nothing and
 * removes both.
 *
 * <h2>Calibration</h2>
 *
 * <p>Two encoder readings, at servo position 0 and at position 1. The servo shaft and the pod are
 * 1:1 and the encoder is on that shaft, so within the window the relationship is a straight line
 * and needs no more than its endpoints.
 */
public class PositionalPod implements SwervePod {

    private final Servo turnServo;
    private final AnalogInput turnEncoder;
    private final DcMotorEx driveMotor;
    private final Pose offset;
    private final String label;

    /** Raw encoder angle, degrees, at servo position 0.0 and 1.0. The whole calibration. */
    private final double rawDegAtPos0;
    private final double rawDegAtPos1;

    private final double angleOffsetRad;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final boolean encoderReversed;

    /**
     * Rate limit on the commanded position, deg/s of pod travel. Zero disables it.
     *
     * <p>Free setpoint profiling: the servo chases whatever it is given, so slewing the command
     * shapes the motion without any of the trajectory machinery an external loop would need.
     */
    private double maxSlewDegPerSec = 0;

    private double commandedRawDeg = Double.NaN;
    private long lastMoveNano = 0;
    private double lastDrivePower = 0;
    private boolean lastMoveFlipped = false;

    private double motorCachingThreshold = 0.01;

    public PositionalPod(HardwareMap hardwareMap, String motorName, String servoName,
            String turnEncoderName, DcMotorSimple.Direction driveDirection,
            double rawDegAtPos0, double rawDegAtPos1, double angleOffsetRad, Pose podOffset,
            double analogMinVoltage, double analogMaxVoltage, boolean encoderReversed) {

        this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.turnServo = hardwareMap.get(Servo.class, servoName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);
        this.label = servoName;

        this.driveMotor.setDirection(driveDirection);
        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.rawDegAtPos0 = rawDegAtPos0;
        this.rawDegAtPos1 = rawDegAtPos1;
        this.angleOffsetRad = angleOffsetRad;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.encoderReversed = encoderReversed;
        this.offset = podOffset;
    }

    /**
     * Adopts the pod's actual position as the commanded one. <b>Call before the servo is enabled.</b>
     *
     * <p>A position-mode servo drives to whatever it is commanded the instant it has power. Without
     * this the first {@code move()} of a match slams every pod from wherever it was left to wherever
     * the kinematics happen to ask for, at full torque. Soft Start limits how violently that
     * happens; it does not stop it happening, so both are wanted, not either.
     */
    public void initFromEncoder() {
        double raw = Math.toDegrees(getRawAngleRad());
        commandedRawDeg = clampToWindow(raw);
        lastMoveNano = System.nanoTime();
        turnServo.setPosition(positionFor(commandedRawDeg));
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        long now = System.nanoTime();
        double dt = lastMoveNano == 0 ? 0 : (now - lastMoveNano) / 1.0e9;
        lastMoveNano = now;

        if (Double.isNaN(commandedRawDeg)) {
            initFromEncoder();
        }

        if (!ignoreAngleChanges) {
            // Desired heading in the same encoder frame CoaxialPod uses, so a mixed drivetrain
            // computes Swerve's avgScaling consistently across both pod types.
            double desiredRaw = Math.toDegrees(
                    MathFunctions.normalizeAngle(adjustThetaForEncoder(targetAngleRad)
                            + angleOffsetRad));

            // A heading and that heading plus 180 are the same demand. Take whichever candidate is
            // inside the window and nearest to where the pod is already commanded; the hysteresis
            // is what keeps the changeover from chattering the way the CR flip did at 85-90 deg.
            double best = Double.NaN;
            double bestCost = Double.MAX_VALUE;
            boolean bestFlipped = false;
            for (int k = -2; k <= 2; k++) {
                double candidate = desiredRaw + 180.0 * k;
                if (!inWindow(candidate)) {
                    continue;
                }
                double cost = Math.abs(candidate - commandedRawDeg);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = candidate;
                    bestFlipped = (k % 2 != 0);
                }
            }

            if (!Double.isNaN(best)) {
                lastMoveFlipped = bestFlipped;
                if (maxSlewDegPerSec > 0 && dt > 0) {
                    double step = maxSlewDegPerSec * dt;
                    double delta = MathFunctions.clamp(best - commandedRawDeg, -step, step);
                    commandedRawDeg = commandedRawDeg + delta;
                } else {
                    commandedRawDeg = best;
                }
                turnServo.setPosition(positionFor(commandedRawDeg));
            }
            // No candidate inside the window means the calibration or the programmed travel is
            // wrong. Holding the last command is the safe response; debugString() will show it.
        }

        if (lastMoveFlipped) {
            drivePower = -drivePower;
        }
        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold
                || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    // ---- window and mapping ----------------------------------------------------------

    private double windowLo() {
        return Math.min(rawDegAtPos0, rawDegAtPos1);
    }

    private double windowHi() {
        return Math.max(rawDegAtPos0, rawDegAtPos1);
    }

    private boolean inWindow(double rawDeg) {
        return rawDeg >= windowLo() && rawDeg <= windowHi();
    }

    private double clampToWindow(double rawDeg) {
        return MathFunctions.clamp(rawDeg, windowLo(), windowHi());
    }

    /** Straight line through the two calibration endpoints. */
    private double positionFor(double rawDeg) {
        double span = rawDegAtPos1 - rawDegAtPos0;
        if (Math.abs(span) < 1e-9) {
            return 0.5;
        }
        return MathFunctions.clamp((rawDeg - rawDegAtPos0) / span, 0.0, 1.0);
    }

    // ---- feedback, kept for init, verification and fault detection ---------------------

    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0) {
            return 0;
        }
        return MathFunctions.clamp((v - analogMinVoltage) / range, 0, 1) * (2.0 * Math.PI);
    }

    @Override
    public double getAngle() {
        return getRawAngleRad() - angleOffsetRad;
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        double t = encoderReversed ? wheelTheta : (2 * Math.PI - wheelTheta);
        return MathFunctions.normalizeAngle(t + Math.PI / 2.0);
    }

    /**
     * Gap between where the servo was told to go and where the encoder says the pod is.
     *
     * <p>The reason to keep the encoder wired once the servo closes its own loop. A servo that has
     * slipped its spline, stripped, or been stopped by a jam still reports nothing at all - the
     * only way to notice is that the pod is not where it was sent. Also catches a calibration that
     * has drifted.
     *
     * @return absolute error in degrees, or NaN before the first command
     */
    public double getSlipDeg() {
        if (Double.isNaN(commandedRawDeg)) {
            return Double.NaN;
        }
        return Math.abs(Math.toDegrees(getRawAngleRad()) - commandedRawDeg);
    }

    public double getCommandedRawDeg() {
        return commandedRawDeg;
    }

    /**
     * Signed tracking error, radians, in the same encoder frame {@link CoaxialPod} reports.
     *
     * <p>Lets the same scorer read both pod types. For a positional pod this is the servo's own
     * loop error rather than an external controller's, which is the point of the comparison.
     */
    public double getLastErrorRad() {
        if (Double.isNaN(commandedRawDeg)) {
            return 0;
        }
        return Math.toRadians(commandedRawDeg - Math.toDegrees(getRawAngleRad()));
    }

    /**
     * Always NaN: a positional pod has no commanded power to report.
     *
     * <p>Present so the recorder can treat both pod types alike. Everything downstream that keys
     * on servo power - resting power RMS, pulse counting - is meaningless here by construction,
     * which is exactly why criterion 8 moved to measuring holding current instead.
     */
    public double getLastTurnPower() {
        return Double.NaN;
    }

    public boolean wasLastMoveFlipped() {
        return lastMoveFlipped;
    }

    /** Rate limit on the commanded position, deg/s. Zero disables it. */
    public void setMaxSlew(double degPerSec) {
        this.maxSlewDegPerSec = degPerSec;
    }

    public void setMotorCachingThreshold(double threshold) {
        this.motorCachingThreshold = threshold;
    }

    @Override
    public Pose getOffset() {
        return offset;
    }

    @Override
    public void setToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void setToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public String debugString() {
        return String.format(Locale.US,
                "%s {positional%n  raw %.1f deg, commanded %.1f deg, slip %.2f deg%n"
                        + "  window %.1f..%.1f deg, servo pos %.4f%n  flipped %s, drive %.2f%n}",
                label, Math.toDegrees(getRawAngleRad()), commandedRawDeg, getSlipDeg(),
                windowLo(), windowHi(),
                Double.isNaN(commandedRawDeg) ? -1 : positionFor(commandedRawDeg),
                lastMoveFlipped, lastDrivePower);
    }
}
