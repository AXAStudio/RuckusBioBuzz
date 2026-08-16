package com.pedropathing.ftc.drivetrains;

import com.pedropathing.drivetrain.CustomDrivetrain;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

/**
 * Swerve Drivetrain implementation.
 * Angles are in radians and positive rotation is to the left (CCW, top-down).
 *
 * @author Kabir Goyal
 * @author Baron Henderson
 */
public class Swerve extends CustomDrivetrain {
    private final SwerveConstants constants;

    protected double lastHeading = 0;

    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double epsilon;
    /** RUCKUS PATCH: see the zeroRotation comment in arcadeDrive. */
    private static final double ROTATION_EPSILON = 0.015;

    /**
     * RUCKUS PATCH: fade the translation and rotation terms to zero across their epsilon bands
     * instead of switching them off at the wall.
     *
     * <p>Measured 2026-08-16 on mydrive-001: of 601 azimuth-setpoint jumps over 15 degrees while
     * driving, 42% were rotation-epsilon crossings and 3% were translation-epsilon crossings -
     * and the translation ones were the biggest in the log, mean 74-95 degrees and up to 179.
     * The cause is structural rather than a tuning error: the pod demand is
     * {@code atan2(translation + rotation)}, so deleting either term in one step rotates the
     * demand by however far apart the two directions were. On a nearly square chassis the
     * rotation-only directions are +-43.5 / +-136.5 degrees, which is why the symptom reads as
     * "snapping to 45 degrees".
     *
     * <p>A smoothstep taper makes the demand direction continuous through the wall: the term
     * shrinks to nothing as the input approaches zero, so the demand migrates toward the other
     * term's direction instead of jumping to it. Value and slope both match at the band edge.
     */
    private static volatile boolean epsilonTaper = true;

    /**
     * RUCKUS PATCH: largest change in a pod's commanded azimuth per call, degrees per second.
     * 0 disables.
     *
     * <p>The taper fixes the demand where the input crosses a wall slowly. It cannot help when
     * the input jumps - a stick released in one loop still swings the demand ~90 degrees. This
     * bounds the rate directly, and the bound is free: the measured pod slew is 214 deg/s
     * (184-259), so a demand moving faster than that is asking for travel the hardware cannot
     * deliver and only guarantees a saturated, lagging pod. A 90 degree change now takes 420 ms
     * of demand travel against the 421 ms of gross travel the pod measured for the same step.
     *
     * <p>Simulated over mydrive-001 (51.3 s of real driving, the recorded commands replayed
     * through both mixers): physical consecutive-loop demand change p90 19.8 -> 13.2 deg, jumps
     * over 15 deg 2.9/s -> 0.9/s, demand reversals 4.25 -> 3.47/s. 300 deg/s was tried first and
     * is worse at this loop rate - it spreads one big jump into several 19 deg steps and the
     * count of violations goes UP.
     *
     * <p>Note what the limiter cannot do: it is a RATE, so a slow loop still turns it into a big
     * step. 53% of the jumps that survive it in simulation happen on loops longer than 70 ms
     * (= 15 deg at 214 deg/s). Criterion 1 is therefore a loop-rate criterion as much as a mixer
     * one; at 50 Hz true with a 25 ms p99 the same limit permits 5.4 deg per loop.
     *
     * <p>Applied on the shortest-angle difference so it never sends a pod the long way round, and
     * skipped entirely past a quarter turn, where the pod flips and reverses the drive rather
     * than rotating - rate-limiting a flip would force a real 180 degree sweep with the drive
     * pointing the wrong way throughout.
     */
    private static volatile double demandSlewDegPerSec = 214.0;

    /** Per-pod anchor for the demand slew limiter, radians; NaN until first commanded. */
    private final double[] lastCommandedTheta;
    private long lastArcadeNano = 0;

    /** Runtime switches, so each fix can be A/B'd inside one session. */
    public static void setEpsilonTaper(boolean on) {
        epsilonTaper = on;
    }

    public static void setDemandSlewDegPerSec(double degPerSec) {
        demandSlewDegPerSec = Math.max(0, degPerSec);
    }

    public static boolean getEpsilonTaper() {
        return epsilonTaper;
    }

    public static double getDemandSlewDegPerSec() {
        return demandSlewDegPerSec;
    }

    /**
     * Smoothstep from 0 at zero input to 1 at the band edge. Slope is zero at both ends, so the
     * taper adds no discontinuity of its own where it meets the untapered region.
     */
    private static double taper(double magnitude, double band) {
        if (band <= 0 || magnitude >= band) {
            return 1.0;
        }
        double u = magnitude / band;
        return u * u * (3.0 - 2.0 * u);
    }

    private List<SwervePod> pods;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;
    private double lastAvgScaling = 0;

    /** RUCKUS PATCH: when the drive inputs last carried a real command. See arcadeDrive. */
    private long lastActiveInputNano = 0;


    /**
     * RUCKUS PATCH: how long zero input must persist before X_LOCK engages. Stick release at
     * speed used to snap all four pods sideways against a still-rolling chassis - ground
     * reaction torque then buffets the pod azimuths and the whole robot judders. Measured on
     * 2026-08-13 with a scripted forward/pause/backward cycle: wheels reversed direction 6-7
     * times per second against a commanded 0.8/s, invariant to every gain tried, and cutting the
     * instant snap halved it. Within this window zero input holds pod headings (servos released,
     * the IGNORE_ANGLE_CHANGES behaviour) so the robot rolls out straight; the X still engages
     * once the chassis has had time to stop.
     */
    private static final double X_LOCK_ENGAGE_DELAY_S = 0.35;

    private final VoltageSensor voltageSensor;

    /**
     * @param constants Swerve Contants for your bot
     * @param pods SwervePods, coaxial or differential
     */
    public Swerve(HardwareMap hardwareMap, SwerveConstants constants, SwervePod... pods) {
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        updateConstants();
        this.pods = Arrays.asList(pods);
        this.lastCommandedTheta = new double[pods.length];
        Arrays.fill(this.lastCommandedTheta, Double.NaN);
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     *
     * @param forward the forward power value, which would typically be
     *                -gamepad1.left_stick_y in a normal arcade drive setup
     * @param strafe the strafe power value, which would typically be
     *               -gamepad1.left_stick_x in a normal arcade drive setup
     *               because pedro treats left as positive
     * @param rotation the rotation power value, which would typically be
     *                 -gamepad1.right_stick_x in a normal arcade drive setup
     *                 because CCW is positive
     */
    public void arcadeDrive(double forward, double strafe, double rotation) {
        strafe *= -1;

        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        // stores forward and strafe values as the translation vector with max magnitude of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(forward, strafe));

        boolean zeroTrans = rawTrans.getMagnitude() < epsilon;
        // RUCKUS PATCH: rotation gets its own, much lower epsilon. The shared 0.05 was a wall
        // that swallowed every fine heading-hold correction, forcing minimum-magnitude
        // workarounds upstream whose smallest allowed kick still deflects pod demands by
        // atan(0.05/translation) - 17+ degrees at crawl speeds, the visible periodic pod
        // wobble the driver kept reporting. 0.015 lets a damped PID correct continuously with
        // deflections too small to see. Translation keeps the original epsilon: the zero-input
        // path (X-lock, release semantics) is tuned around it.
        boolean zeroRotation = Math.abs(rotation) < Math.min(epsilon, ROTATION_EPSILON);

        // RUCKUS PATCH: X_LOCK only after zero input has persisted, so a stick release at speed
        // does not snap the pods sideways under a rolling chassis. Until then zero input behaves
        // like IGNORE_ANGLE_CHANGES: headings held, servos quiet, robot rolls out straight.
        if (!(zeroTrans && zeroRotation)) {
            lastActiveInputNano = System.nanoTime();
        }
        boolean xLockRipe = zeroTrans && zeroRotation
                && (System.nanoTime() - lastActiveInputNano) / 1.0e9 >= X_LOCK_ENGAGE_DELAY_S;

        // RUCKUS PATCH: the epsilon walls decide RELEASE semantics (X-lock, servos quiet) as
        // booleans, exactly as before - but the vectors handed to the mixer are tapered rather
        // than deleted, so the demand direction is continuous through each wall. See epsilonTaper.
        double transScale = epsilonTaper ? taper(rawTrans.getMagnitude(), epsilon) : 0.0;
        double rotEpsilon = Math.min(epsilon, ROTATION_EPSILON);
        double rotScale = epsilonTaper ? taper(Math.abs(rotation), rotEpsilon) : 0.0;

        // Untapered, these collapse to the original hard switch: scale 0 below the wall, 1 above.
        double rotationScalar = zeroRotation ? rotation * rotScale : rotation;

        Vector[] podVectors = new Vector[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            SwervePod pod = pods.get(i);

            Vector translationVector = zeroTrans ? rawTrans.times(transScale) : rawTrans;

            // actually positive rotation scalar because positive turning is to the left
            Vector rotationVector = new Vector(rotationScalar, Math.atan2(pod.getOffset().getX(), -pod.getOffset().getY()));

            // this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
            if (constants.getZeroPowerBehavior() == SwerveConstants.ZeroPowerBehavior.X_LOCK
                    && xLockRipe) {
                // Zero magnitude, pod's own radius for the angle: the X pattern, and no drive
                // power whatever the taper left in the vectors above.
                podVectors[i] = new Vector(0,
                        Math.atan2(pod.getOffset().getX(), -pod.getOffset().getY()));
            }
        }

        // finding if any vector has magnitude > maxPowerScaling
        //
        // RUCKUS PATCH: voltage compensation used to call podVector.times(...) and discard the
        // result - Vector.times returns a new Vector - so the feature was a silent no-op that
        // still paid one blocking getVoltage() ADC read per pod per call. Read the sensor once
        // and actually apply the scale.
        double maxMagnitude = maxPowerScaling;
        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < podVectors.length; i++) {
                podVectors[i] = podVectors[i].times(voltageNormalized);
            }
        }
        for (Vector podVector : podVectors) {
            maxMagnitude = Math.max(maxMagnitude, podVector.getMagnitude());
        }

        // Find the avg scaling constant (avg of cos(angle error))
        double avgScaling = 0;

        for (int i = 0; i < pods.size(); i++) {
            double currentRad = pods.get(i).getAngle();

            // ask the pod to translate the wheel-space theta into the encoder frame
            double targetRad = pods.get(i).adjustThetaForEncoder(podVectors[i].getTheta());

            // compute shortest signed error in radians using MathFunctions
            double mag = MathFunctions.getSmallestAngleDifference(currentRad, targetRad);
            double dir = MathFunctions.getTurnDirection(currentRad, targetRad);
            double errorRad = (mag == Math.PI) ? -Math.PI : mag * dir;

            avgScaling += Math.abs(Math.cos(errorRad));
        }

        avgScaling /= pods.size();
        lastAvgScaling = avgScaling;

        long nowNano = System.nanoTime();
        double slewDt = lastArcadeNano == 0 ? 0 : (nowNano - lastArcadeNano) / 1.0e9;
        lastArcadeNano = nowNano;
        // A stalled caller (mode switch, OpMode pause) must not bank up allowance.
        double maxStep = (demandSlewDegPerSec > 0 && slewDt > 0 && slewDt < 0.5)
                ? Math.toRadians(demandSlewDegPerSec) * slewDt
                : Double.POSITIVE_INFINITY;

        for (int podNum = 0; podNum < pods.size(); podNum++) {
            // Normalizing if necessary while preserving relative sizes
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude);

            // RUCKUS PATCH: inside the X_LOCK engage delay the pod vectors are still the
            // degenerate zero vectors, so the pods must be released rather than sent chasing
            // a meaningless theta - same treatment IGNORE_ANGLE_CHANGES always gets.
            boolean release = zeroTrans && zeroRotation
                    && (constants.getZeroPowerBehavior() == SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES
                            || !xLockRipe);

            double theta = finalVector.getTheta();
            if (release) {
                // Servos are off and the pods hold where they are, so the anchor stays valid and
                // is KEPT - dropping it made the first command after every release unlimited,
                // which is exactly the snap-out-of-park the limiter exists to smooth. Simulated
                // over mydrive-001: keeping it cut jumps of 85 deg or more from 4-6 per pod to
                // 1-2.
                lastCommandedTheta[podNum] = Double.isNaN(lastCommandedTheta[podNum])
                        ? theta : lastCommandedTheta[podNum];
            } else if (!Double.isNaN(lastCommandedTheta[podNum])
                    && maxStep != Double.POSITIVE_INFINITY) {
                double delta = MathFunctions.normalizeAngleSigned(
                        theta - lastCommandedTheta[podNum]);
                // Past a quarter turn the pod does not rotate at all - it flips and reverses the
                // drive, which costs no travel and is the behaviour the flip hysteresis was tuned
                // for. Rate-limiting there would force a real 180 degree sweep with the drive
                // pointing the wrong way for the whole of it. So limit ordinary rotations only,
                // and re-anchor on the ones the pod will resolve by flipping.
                if (Math.abs(delta) <= Math.PI / 2.0) {
                    if (delta > maxStep) {
                        theta = MathFunctions.normalizeAngle(lastCommandedTheta[podNum] + maxStep);
                    } else if (delta < -maxStep) {
                        theta = MathFunctions.normalizeAngle(lastCommandedTheta[podNum] - maxStep);
                    }
                }
                lastCommandedTheta[podNum] = theta;
            } else {
                lastCommandedTheta[podNum] = theta;
            }

            pods.get(podNum).move(theta, finalVector.getMagnitude() * avgScaling, release);
        }
    }

    /**
     * Updates cached values from the constants object.
     */
    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        this.maxPowerScaling = constants.getMaxPower(); // inherited from Drivetrain, used by CustomDrivetrain
        this.voltageCompensation = constants.getUseVoltageCompensation(); // inherited from Drivetrain
        this.nominalVoltage = constants.getNominalVoltage(); // inherited from Drivetrain
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();
        this.epsilon = constants.getEpsilon();
    }

    /**
     * Stops following and holds pod angles while floating drive motors.
     */
    @Override
    public void breakFollowing() {
        for (SwervePod pod : pods) {
            pod.move(pod.getAngle(), 0, true);
            pod.setToFloat();
        }
    }

    /**
     * Starts teleop drive with the configured brake mode.
     */
    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            for (SwervePod pod : pods) {
                pod.setToBreak();
            }
        }
    }

    /**
     * @param brakeMode set to true to enable brake mode in teleop
     */
    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            for (SwervePod pod : pods) {
                pod.setToBreak();
            }
        } else {
            for (SwervePod pod : pods) {
                pod.setToFloat();
            }
        }
    }

    /**
     * @return maximum x velocity
     */
    @Override
    public double xVelocity() {
        return constants.getXVelocity();
    }

    /**
     * @return maximum y velocity
     */
    @Override
    public double yVelocity() {
        return constants.getYVelocity();
    }

    /**
     * @param xMovement maximum x velocity
     */
    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    /**
     * @param yMovement maximum y velocity
     */
    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    /**
     * @return static friction coefficient used for voltage compensation
     */
    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    /**
     * @return current battery voltage
     */
    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    /**
     * @return normalized voltage for voltage compensation
     */
    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    /**
     * @return debug string for drivetrain state
     */
    @Override
    public String debugString() {
        StringBuilder sb = new StringBuilder("Swerve {");
        for (int i = 0; i < pods.size(); i++) {
            sb.append("\npod").append(i)
                    .append(": ").append(pods.get(i).debugString());
        }
        sb.append("\n\nforward input=").append(lastForward)
                .append("\nstrafe input=").append(lastStrafe)
                .append("\nrotation input=").append(lastRotation)
                .append("\nrobot heading").append(lastHeading)
                .append("\navg scaling").append(lastAvgScaling)
                .append("\n}");
        return sb.toString();
    }
}
