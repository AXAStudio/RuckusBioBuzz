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
 * programmed for 190, and the extra 10 is deliberate: in that overlap arc both representations of a
 * heading are reachable, which turns the changeover into a hysteresis band the controller can
 * schedule rather than an instant it is forced through.
 *
 * <p>A forced 180 degree traverse still happens when the demand crosses that arc, costing roughly
 * 240 ms plus settling, so the arc is placed away from the headings the pods dwell at - forward 90,
 * strafe 0/180, and the X-lock diagonals at 46.49 and 133.51, the last from
 * {@code atan2(146.42, +/-154.24)}. The gaps are unequal: forward-to-X-lock is 43.51 degrees and
 * strafe-to-X-lock is 46.49, so the arc goes in a strafe-to-X-lock gap, centred. The whole arc must
 * clear the dwell set, not just one end - which is why 190 degrees of travel beats 200: it halves
 * the arc and raises achievable clearance from 13.24 degrees to 18.25.
 *
 * <p>Three degrees at each end are then held back in software ({@link #setClampMarginDeg}), because
 * in position mode the travel ends are hard stops and driving into one stalls the servo into its
 * overload cutout with nothing reported. That leaves a 184 degree commandable band for 180 degrees
 * of headings; {@link #verifyCoverage} proves the remainder is enough.
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

    /**
     * How far inside each programmed endpoint the software will actually command, in degrees.
     *
     * <p>In position mode the travel ends are hard stops. Commanding past one stalls the servo into
     * its overload cutout, which is a silent steering failure - the encoder just shows a pod that
     * stopped tracking, and nothing is reported. CR mode had no equivalent because there were no
     * ends. With 190 degrees of travel this costs 6 of the 10 degrees of overlap and leaves 4,
     * which is still five times the residual, so the margin is effectively free.
     */
    private double clampMarginDeg = 3.0;

    /** Set when no representation of the demanded heading fell inside the clamped band. */
    private boolean noCandidateFault = false;

    /**
     * Whether the two endpoints have actually been measured on this servo.
     *
     * <p>Until they have, {@code rawDegAtPos0/1} are placeholders and the position mapping is
     * fiction - {@link #initFromEncoder} would map the pod's real angle through a wrong line
     * and command an arbitrary position, which in position mode means driving there at once.
     * So an uncalibrated pod writes nothing to the servo at all. Calibration goes through
     * {@link #setRawPositionForCalibration}, which bypasses the mapping precisely because the
     * mapping is what it is there to establish.
     */
    private boolean calibrated = false;

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
        if (!calibrated) {
            return;
        }
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

        if (!calibrated) {
            return;
        }
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

            noCandidateFault = Double.isNaN(best);
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
            // No candidate inside the clamped band means the calibration or the programmed travel
            // is wrong - verifyCoverage() should have caught it before anything moved. Holding the
            // last command is the safe response, and noCandidateFault makes it visible rather than
            // letting the pod quietly stop tracking.
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

    /** Programmed travel, before the safety margin. */
    private double travelLo() {
        return Math.min(rawDegAtPos0, rawDegAtPos1);
    }

    private double travelHi() {
        return Math.max(rawDegAtPos0, rawDegAtPos1);
    }

    /** The band actually commanded: the programmed travel less the end margins. */
    private double windowLo() {
        return travelLo() + clampMarginDeg;
    }

    private double windowHi() {
        return travelHi() - clampMarginDeg;
    }

    private boolean inWindow(double rawDeg) {
        return rawDeg >= windowLo() && rawDeg <= windowHi();
    }

    private double clampToWindow(double rawDeg) {
        return MathFunctions.clamp(rawDeg, windowLo(), windowHi());
    }

    public void setCalibrated(boolean value) {
        this.calibrated = value;
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    /**
     * Calibration only: writes a servo position directly, bypassing the angle mapping.
     *
     * <p>The mapping needs the endpoints, and finding the endpoints needs to move the servo,
     * so this is the one path that may command an uncalibrated pod. Callers must approach the
     * ends incrementally and watch the encoder for a stall - the pod's mechanical range may be
     * smaller than the servo's programmed travel, and nothing else would notice.
     */
    public void setRawPositionForCalibration(double position) {
        turnServo.setPosition(MathFunctions.clamp(position, 0.0, 1.0));
    }

    public void setClampMarginDeg(double marginDeg) {
        this.clampMarginDeg = Math.abs(marginDeg);
    }

    public double getClampMarginDeg() {
        return clampMarginDeg;
    }

    public boolean hasNoCandidateFault() {
        return noCandidateFault;
    }

    /**
     * Proves every reachable wheel heading has a representation inside the clamped band.
     *
     * <p>The argument is short: a pod treats a heading and that heading plus 180 as the same
     * demand, so the representations of any heading form a lattice spaced 180 degrees apart, and
     * <em>any</em> interval at least 180 degrees wide contains a point of it. The clamped band is
     * {@code travel - 2 * margin} wide, so coverage holds whenever that is at least 180 - with
     * 190 degrees of travel and a 3 degree margin it is 184, leaving 4 degrees of slack.
     *
     * <p>That is the mathematics. This method checks the implementation, which is the part that can
     * actually be wrong: it sweeps the whole heading circle at fine resolution against the real
     * calibration and the real selection logic, and reports the worst margin any heading had. Run
     * it at construction, before anything is commanded.
     *
     * @param stepDeg sweep resolution over the heading circle
     * @return worst-case distance from a chosen position to the nearer clamp edge, in degrees;
     *         negative means some heading is not reachable and the pod must not be driven
     */
    public double verifyCoverage(double stepDeg) {
        double bandWidth = windowHi() - windowLo();
        if (bandWidth < 180.0) {
            return bandWidth - 180.0;
        }
        double worst = Double.MAX_VALUE;
        for (double wheel = 0; wheel < 360.0; wheel += stepDeg) {
            double desiredRaw = Math.toDegrees(MathFunctions.normalizeAngle(
                    adjustThetaForEncoder(Math.toRadians(wheel)) + angleOffsetRad));
            double best = Double.NaN;
            double bestMargin = -Double.MAX_VALUE;
            for (int k = -3; k <= 3; k++) {
                double candidate = desiredRaw + 180.0 * k;
                if (!inWindow(candidate)) {
                    continue;
                }
                double margin = Math.min(candidate - windowLo(), windowHi() - candidate);
                if (margin > bestMargin) {
                    bestMargin = margin;
                    best = candidate;
                }
            }
            if (Double.isNaN(best)) {
                return -1.0;
            }
            worst = Math.min(worst, bestMargin);
        }
        return worst == Double.MAX_VALUE ? -1.0 : worst;
    }

    /**
     * Commands a raw encoder angle directly, through the same clamp {@link #move} uses.
     *
     * <p>Exists so the clamp can be tested by deliberately asking for something outside it, before
     * any step response is run. A clamp that has never been exercised is an assumption.
     *
     * @return the position actually written, in [0, 1]
     */
    public double commandRawDegForTest(double rawDeg) {
        commandedRawDeg = clampToWindow(rawDeg);
        double p = positionFor(commandedRawDeg);
        turnServo.setPosition(p);
        return p;
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
                        + "  travel %.1f..%.1f, clamped to %.1f..%.1f (margin %.1f)%n"
                        + "  servo pos %.4f%n  flipped %s, drive %.2f, noCandidateFault %s%n}",
                label, Math.toDegrees(getRawAngleRad()), commandedRawDeg, getSlipDeg(),
                travelLo(), travelHi(), windowLo(), windowHi(), clampMarginDeg,
                Double.isNaN(commandedRawDeg) ? -1 : positionFor(commandedRawDeg),
                lastMoveFlipped, lastDrivePower, noCandidateFault);
    }
}
