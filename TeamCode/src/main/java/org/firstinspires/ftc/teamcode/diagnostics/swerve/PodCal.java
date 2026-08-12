package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwervePod;
import org.firstinspires.ftc.teamcode.pedroPathing.PositionalPod;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Mutable calibration state for a single coaxial swerve pod.
 *
 * <p>Every field here corresponds to a constructor argument of {@link CoaxialPod} in
 * {@code SwerveDrivetrainConstants}. The bring-up tool discovers these values at runtime; once
 * they look right you export them back into source with
 * {@link SwerveExport#generate}.
 */
public class PodCal {
    /** Hardware index, matching the {@code sm#/ss#/se#} naming convention. */
    public final int index;

    /** Human-facing corner label, e.g. {@code "LF"}. Set during wiring identification. */
    public String label;

    public String motorName;
    public String servoName;
    public String encoderName;

    /** Drive motor direction. */
    public DcMotorSimple.Direction driveDirection = DcMotorSimple.Direction.FORWARD;

    /** Turn servo direction. */
    public CRServo.Direction servoDirection = CRServo.Direction.FORWARD;

    /**
     * True when the encoder counts up as the pod rotates counter-clockwise viewed from the top.
     * Determined by {@code WIRE_SCAN} plus the operator's CW/CCW answer.
     */
    public boolean encoderReversed = false;

    /** Raw encoder angle (radians) observed when the wheel points straight forward. */
    public double angleOffsetRad = 0.0;

    /** Observed analog range of the Axon absolute output. */
    public double analogMin = 0.0;
    public double analogMax = 3.3;

    /** Turn-servo PIDF, in the same units {@code SwerveDrivetrainConstants} uses (per-radian). */
    public double kP = 0.0055 * 180 / Math.PI;
    public double kD = 0.00015 * 180 / Math.PI;
    public double kF = 0.013;

    /**
     * Integral gain. Starts at zero because it is only needed to grind out the last couple of
     * degrees that carpet stiction holds: outside 2 degrees CoaxialPod applies feed-forward, and
     * inside it P alone is too weak to break free. Raise it slowly - the integral is bounded and
     * resets on a sign change, but it is still the term that turns a steady offset into a hunt.
     */
    public double kI = 0.0;

    /**
     * Continuous static-friction feed-forward, replacing the sign-only kF relay.
     *
     * <p>Set to the pod's measured breakaway power - the smallest open-loop power that moves it
     * from rest. Applied as {@code kS * tanh(error / kSBand)}, so it has full authority where the
     * pod is stuck and tapers to nothing at the target instead of switching sign there.
     *
     * <p>Measured breakaway on this drivetrain is 0.025-0.050 depending on pod and direction.
     */
    public double kS = 0.0;

    /** Error at which {@link #kS} reaches 76% of its peak, in degrees. */
    public double kSBandDeg = 2.0;

    /**
     * Bounds on the integral term. Only matter once {@link #kI} is non-zero.
     *
     * <p>{@code kILimit} caps its contribution to the output; anything much above the measured
     * breakaway power just slams the pod. {@code kIBandDeg} keeps it out of the slew, where the
     * proportional term is already saturated and accumulating only buys an overshoot.
     * {@code kIResetDeg} stops the sign-change reset firing on encoder noise at the target.
     */
    public double kILimit = 0.08;
    public double kIBandDeg = 6.0;
    public double kIResetDeg = 2.0;

    /**
     * Take the D term from the measured pod velocity instead of from the error.
     *
     * <p>Removes the derivative kick a setpoint step and the 180 degree flip both produce, which is
     * what limits how much damping the loop can carry.
     */
    public boolean derivativeOnMeasurement = false;

    /**
     * Discrete-pulse final approach, for the last few degrees where the pod has no creep regime.
     *
     * <p>{@link #pulsePower} should sit just above the pod's measured breakaway, and
     * {@link #pulseSeconds} sets how far each pulse travels - roughly the terminal speed at that
     * power times the pulse length. A pulse shorter than one servo PWM frame may not be delivered.
     */
    public boolean pulsed = false;
    public double pulseBandDeg = 3.0;
    public double pulseTolDeg = 0.5;
    public double pulsePower = 0.055;
    public double pulseMs = 15.0;
    public double pulseStationaryDegPerSec = 20.0;
    public double pulseCoastMs = 100.0;

    /**
     * Run this pod through {@link PositionalPod} instead, letting the servo close its own loop.
     *
     * <p>Requires the servo to have been reflashed to Servo Mode AND its port reconfigured from
     * {@code ContinuousRotationServo} to {@code Servo} - the SDK builds a different device class
     * per port type, and asking for a Servo on a CR port throws.
     */
    public boolean positional = false;

    /** Raw encoder angle, degrees, at servo position 0.0 and 1.0. The whole positional calibration. */
    public double rawDegAtPos0 = 0.0;
    public double rawDegAtPos1 = 270.0;

    /** Degrees held back from each programmed endpoint, so a command cannot reach a stop. */
    public double clampMarginDeg = 3.0;

    /** True once both endpoints have been measured on this servo. Until then it will not drive. */
    public boolean posCalibrated = false;

    /**
     * Minimum change in servo power before CoaxialPod actually writes it.
     *
     * <p>This is a deadband on the control OUTPUT: below it the servo keeps whatever power it had,
     * so the loop cannot make fine corrections and instead holds a stale power, overshoots, then
     * slams to a large correction. Too high and the pod limit-cycles while driving.
     */
    public double servoCaching = 0.05;

    /** Pod position relative to robot center, matching the odometry coordinate system. */
    public double podX;
    public double podY;

    /** Fallbacks used only if a corner is assigned before any geometry is known. */
    private static final double DEFAULT_HALF_LENGTH = 146.420;
    private static final double DEFAULT_HALF_WIDTH = 154.240;

    /**
     * Analog channel this pod's encoder actually landed on, as discovered by the wiring scan.
     * {@code -1} until a scan runs. Because the Axon feedback wires are spliced two-per-analog-port,
     * this is the value most likely to disagree with the wiring diagram.
     */
    public int discoveredEncoderIndex = -1;

    public PodCal(int index, String label, double podX, double podY) {
        this.index = index;
        this.label = label;
        this.podX = podX;
        this.podY = podY;
        this.motorName = "sm" + index;
        this.servoName = "ss" + index;
        this.encoderName = "se" + index;
    }

    /**
     * Places this pod at a chassis corner, setting {@link #podX}/{@link #podY} to match.
     *
     * <p>Only rotation uses these offsets - translation ignores them entirely - so a wrong corner
     * assignment produces a drivetrain that strafes perfectly and spins the wrong way. Signs follow
     * Pedro's odometry frame: +X forward, +Y left.
     *
     * @param corner one of LF, RF, LB, RB
     * @return false if the corner was not recognised, leaving this pod untouched
     */
    public boolean setCorner(String corner) {
        if (corner == null) {
            return false;
        }

        String c = corner.trim().toUpperCase(Locale.US);
        double length = Math.abs(podX) == 0 ? DEFAULT_HALF_LENGTH : Math.abs(podX);
        double width = Math.abs(podY) == 0 ? DEFAULT_HALF_WIDTH : Math.abs(podY);

        switch (c) {
            case "LF":
                podX = length;
                podY = width;
                break;
            case "RF":
                podX = length;
                podY = -width;
                break;
            case "LB":
                podX = -length;
                podY = width;
                break;
            case "RB":
                podX = -length;
                podY = -width;
                break;
            default:
                return false;
        }

        label = c;
        return true;
    }

    /** Builds whichever pod type this calibration asks for. */
    public SwervePod toSwervePod(HardwareMap hardwareMap) {
        if (positional) {
            PositionalPod pod = new PositionalPod(hardwareMap, motorName, servoName, encoderName,
                    driveDirection, rawDegAtPos0, rawDegAtPos1, angleOffsetRad,
                    new Pose(podX, podY), analogMin, analogMax, encoderReversed);
            pod.setClampMarginDeg(clampMarginDeg);
            pod.setCalibrated(posCalibrated);
            return pod;
        }
        return toCoaxialPod(hardwareMap);
    }

    /** Builds a real Pedro pod from the current calibration. */
    public CoaxialPod toCoaxialPod(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                motorName,
                servoName,
                encoderName,
                new PIDFCoefficients(kP, kI, kD, kF),
                driveDirection,
                servoDirection,
                angleOffsetRad,
                new Pose(podX, podY),
                analogMin,
                analogMax,
                encoderReversed);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(servoCaching);
        pod.setStaticFriction(kS, Math.toRadians(kSBandDeg));
        pod.setTurnIntegralSettings(kILimit, Math.toRadians(kIBandDeg), Math.toRadians(kIResetDeg));
        pod.setDerivativeOnMeasurement(derivativeOnMeasurement);
        pod.setPulsedApproach(pulsed, Math.toRadians(pulseBandDeg), Math.toRadians(pulseTolDeg),
                pulsePower, pulseMs / 1000.0, Math.toRadians(pulseStationaryDegPerSec),
                pulseCoastMs / 1000.0);
        return pod;
    }

    /** Converts an analog reading to a raw pod angle the same way {@link CoaxialPod} does. */
    public double rawAngleRad(double volts) {
        double range = analogMax - analogMin;
        if (range == 0) {
            return 0;
        }
        double normalized = (volts - analogMin) / range;
        normalized = Math.max(0, Math.min(1, normalized));
        return normalized * 2.0 * Math.PI;
    }

    /** Pod angle after the zero offset is applied, normalized to [0, 2pi). */
    public double zeroedAngleRad(double volts) {
        return normalize((rawAngleRad(volts) - angleOffsetRad));
    }

    /**
     * Converts an encoder-space angle back into wheel-space theta - the exact inverse of
     * {@link CoaxialPod#adjustThetaForEncoder}. The visualizer needs this to draw where each wheel
     * is actually pointing in the robot frame.
     *
     * <p>Forward maps to theta = pi/2, left to pi, matching the frame
     * {@code Swerve.arcadeDrive} builds its pod vectors in.
     */
    public double wheelThetaFromEncoder(double encoderRad) {
        double t = encoderRad - Math.PI / 2.0;
        return normalize(encoderReversed ? t : -t);
    }

    private static double normalize(double radians) {
        double a = radians % (2 * Math.PI);
        return a < 0 ? a + 2 * Math.PI : a;
    }

    public boolean driveReversed() {
        return driveDirection == DcMotorSimple.Direction.REVERSE;
    }

    public boolean servoReversed() {
        return servoDirection == CRServo.Direction.REVERSE;
    }

    public void setDriveReversed(boolean reversed) {
        driveDirection = reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    public void setServoReversed(boolean reversed) {
        servoDirection = reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD;
    }

    /** Serializes to a single line for the on-hub calibration file. */
    public String serialize() {
        return index
                + "|" + label
                + "|" + motorName
                + "|" + servoName
                + "|" + encoderName
                + "|" + driveReversed()
                + "|" + servoReversed()
                + "|" + encoderReversed
                + "|" + angleOffsetRad
                + "|" + analogMin
                + "|" + analogMax
                + "|" + kP
                + "|" + kD
                + "|" + kF
                + "|" + podX
                + "|" + podY
                + "|" + kI
                + "|" + servoCaching
                + "|" + kS
                + "|" + kSBandDeg
                + "|" + kILimit
                + "|" + kIBandDeg
                + "|" + kIResetDeg
                + "|" + derivativeOnMeasurement
                + "|" + pulsed
                + "|" + pulseBandDeg
                + "|" + pulseTolDeg
                + "|" + pulsePower
                + "|" + pulseMs
                + "|" + pulseStationaryDegPerSec
                + "|" + pulseCoastMs
                + "|" + positional
                + "|" + rawDegAtPos0
                + "|" + rawDegAtPos1
                + "|" + clampMarginDeg
                + "|" + posCalibrated
                + "|" + discoveredEncoderIndex;
    }

    /** Applies a line previously produced by {@link #serialize()}. Returns false if unusable. */
    public boolean applySerialized(String line) {
        String[] p = line.split("\\|", -1);
        if (p.length < 16) {
            return false;
        }
        try {
            label = p[1];
            motorName = p[2];
            servoName = p[3];
            encoderName = p[4];
            setDriveReversed(Boolean.parseBoolean(p[5]));
            setServoReversed(Boolean.parseBoolean(p[6]));
            encoderReversed = Boolean.parseBoolean(p[7]);
            angleOffsetRad = Double.parseDouble(p[8]);
            analogMin = Double.parseDouble(p[9]);
            analogMax = Double.parseDouble(p[10]);
            kP = Double.parseDouble(p[11]);
            kD = Double.parseDouble(p[12]);
            kF = Double.parseDouble(p[13]);
            podX = Double.parseDouble(p[14]);
            podY = Double.parseDouble(p[15]);
            // Every field appended after the first calibration files were written is optional, so
            // an older file still loads and simply takes the defaults for whatever it predates.
            // This list had fallen behind: kS and kSBandDeg were persisted but nothing after them,
            // so a restart silently reverted the integral bounds, the pulse settings and - the
            // expensive one - the positional endpoint calibration.
            kI = p.length > 16 ? Double.parseDouble(p[16]) : 0.0;
            servoCaching = p.length > 17 ? Double.parseDouble(p[17]) : 0.05;
            kS = p.length > 18 ? Double.parseDouble(p[18]) : 0.0;
            kSBandDeg = p.length > 19 ? Double.parseDouble(p[19]) : 2.0;
            kILimit = p.length > 20 ? Double.parseDouble(p[20]) : 0.08;
            kIBandDeg = p.length > 21 ? Double.parseDouble(p[21]) : 6.0;
            kIResetDeg = p.length > 22 ? Double.parseDouble(p[22]) : 2.0;
            derivativeOnMeasurement = p.length > 23 && Boolean.parseBoolean(p[23]);
            pulsed = p.length > 24 && Boolean.parseBoolean(p[24]);
            pulseBandDeg = p.length > 25 ? Double.parseDouble(p[25]) : 3.0;
            pulseTolDeg = p.length > 26 ? Double.parseDouble(p[26]) : 0.5;
            pulsePower = p.length > 27 ? Double.parseDouble(p[27]) : 0.055;
            pulseMs = p.length > 28 ? Double.parseDouble(p[28]) : 15.0;
            pulseStationaryDegPerSec = p.length > 29 ? Double.parseDouble(p[29]) : 20.0;
            pulseCoastMs = p.length > 30 ? Double.parseDouble(p[30]) : 100.0;
            // The positional fields matter most of all: rawDegAtPos0/1 are a physical measurement
            // that costs a guarded walk to both mechanical limits to obtain, and losing them to an
            // OpMode restart would mean doing it again.
            positional = p.length > 31 && Boolean.parseBoolean(p[31]);
            rawDegAtPos0 = p.length > 32 ? Double.parseDouble(p[32]) : 0.0;
            rawDegAtPos1 = p.length > 33 ? Double.parseDouble(p[33]) : 270.0;
            clampMarginDeg = p.length > 34 ? Double.parseDouble(p[34]) : 3.0;
            posCalibrated = p.length > 35 && Boolean.parseBoolean(p[35]);
            discoveredEncoderIndex = p.length > 36 ? Integer.parseInt(p[36]) : -1;
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    /**
     * Checks that every mutable field survives {@link #serialize} and {@link #applySerialized}.
     *
     * <p>A guard rather than a review. This class has been extended repeatedly - static friction,
     * integral bounds, derivative source, pulse parameters, the positional block - and the
     * serialiser fell behind silently every time, because nothing fails when a field is simply not
     * written. It was caught once by reading the file on the hub; that catches today's gap, not
     * the next one.
     *
     * <p>Works by reflection so it cannot fall behind: any field added later is covered
     * automatically, and the check fails the moment one is not round-tripping. Called from the
     * OpMode's init so it runs every session rather than only when someone thinks to look.
     *
     * @return names of fields that did not survive the round trip; empty when all is well
     */
    public static List<String> roundTripGaps() {
        List<String> gaps = new ArrayList<>();
        try {
            PodCal source = new PodCal(0, "LF", 111.0, 222.0);
            PodCal defaults = new PodCal(0, "LF", 111.0, 222.0);

            // Give every field a value distinct from its default, so a field that is not persisted
            // shows up as a mismatch rather than accidentally matching.
            int seed = 0;
            for (Field f : PodCal.class.getFields()) {
                if (Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())) {
                    continue;
                }
                seed++;
                Class<?> t = f.getType();
                if (t == double.class) {
                    f.setDouble(source, 100.0 + seed * 0.125);
                } else if (t == int.class) {
                    f.setInt(source, 1000 + seed);
                } else if (t == boolean.class) {
                    f.setBoolean(source, !f.getBoolean(defaults));
                } else if (t == String.class) {
                    f.set(source, "rt" + seed);
                } else if (t.isEnum()) {
                    Object[] all = t.getEnumConstants();
                    Object current = f.get(defaults);
                    for (Object candidate : all) {
                        if (candidate != current) {
                            f.set(source, candidate);
                            break;
                        }
                    }
                } else {
                    gaps.add(f.getName() + " (unhandled type " + t.getSimpleName()
                            + "; extend roundTripGaps)");
                }
            }

            PodCal restored = new PodCal(0, "zz", 0, 0);
            if (!restored.applySerialized(source.serialize())) {
                gaps.add("applySerialized rejected its own serialize() output");
                return gaps;
            }

            for (Field f : PodCal.class.getFields()) {
                if (Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())) {
                    continue;
                }
                Object a = f.get(source);
                Object b = f.get(restored);
                boolean same = (a instanceof Double)
                        ? Math.abs((Double) a - (Double) b) < 1e-9
                        : (a == null ? b == null : a.equals(b));
                if (!same) {
                    gaps.add(f.getName() + " (" + a + " -> " + b + ")");
                }
            }
        } catch (ReflectiveOperationException | RuntimeException e) {
            gaps.add("round-trip check itself failed: " + e);
        }
        return gaps;
    }
}
