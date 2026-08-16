package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.math.MathFunctions;

/**
 * Closed-loop heading hold for teleop: the turn stick sets a heading RATE, and whatever setpoint
 * it leaves behind is held.
 *
 * <p><b>Why this class exists.</b> Until 2026-08-16 the competition drive path had no closed
 * heading loop at all. {@code DriveTeleOp} passed the right stick to
 * {@code Follower.setTeleOpDrive}, which routes it to {@code teleopHeadingVector} and comes out
 * of {@code calculateDrive} as the same number - a rate command straight through the mixer, with
 * no setpoint, no error and nothing to correct a shove or a drift. Every heading measurement this
 * project has published came from {@code SwerveBringUp}, which has its own hold and is a
 * diagnostic tool. This is that behaviour, in shipped code.
 *
 * <p><b>What it inherits from the bring-up implementation</b>, each part paid for on the robot:
 * <ul>
 *   <li><b>Rate stick, latched setpoint.</b> Releasing the stick locks the current heading rather
 *       than coasting.
 *   <li><b>Lead cap</b> (60 deg). The setpoint may never run further ahead than the controller
 *       can chase: past 180 the error wraps and the turn direction flips mid-turn. The cap is
 *       also what makes the sweep rate battery-proof - the setpoint runs ahead, the output
 *       saturates, and the robot turns at whatever its true maximum is today.
 *   <li><b>A release sequence, not an instant latch.</b> On zero input the controller stops
 *       correcting immediately (chasing a latched target while the chassis still carries
 *       momentum rubber-bands it), waits for the measured rotation to actually stop, and only
 *       then snaps the setpoint to wherever physics parked the robot.
 *   <li><b>Re-adopt reality on re-entry.</b> A resting setpoint goes stale - the robot can be
 *       shoved while parked and nothing corrects it, by design - so acting on it later would
 *       snap the robot somewhere nobody asked for.
 *   <li><b>Stuck-sensor guard.</b> If real rotation is being commanded and the measured heading
 *       has not moved at all, the sensor is dead and the loop would happily spin the robot at
 *       full power. It disables itself instead.
 * </ul>
 *
 * <p><b>What it deliberately does NOT inherit:</b> the epsilon-bypass trim. That exists because
 * {@code Swerve.arcadeDrive} used to delete any rotation under its epsilon, so a fine correction
 * never reached the pods. As of 2026-08-16 the mixer tapers across the epsilon band instead of
 * switching at it, so small corrections survive on their own and the bypass would be a second,
 * competing hack over the same deadband.
 *
 * <p><b>Known duplication.</b> {@code SwerveBringUp.runDriveMode} still carries its own inline
 * copy of this logic. It was left alone deliberately - rewriting the diagnostic tool's control
 * path mid-session would change what is being measured - but the two must not be allowed to
 * drift. That exact failure has already cost this project once: {@code computeTargets} was a
 * mirror of the mixer, the mixer moved its rotation epsilon and the mirror did not, and 4% of a
 * recorded session logged a demand the pods were never given.
 *
 * <p><b>Status: NOT YET VALIDATED ON THE ROBOT.</b> The gains come from the bring-up tool's
 * measured values (kP 1.20, kD 0.080, 2026-08-11 on the Pinpoint). Everything else here is
 * structure carried across from an implementation that was validated. Treat the first run as a
 * test.
 */
public class HeadingHold {

    /** Stick deflection that counts as "the driver is turning". */
    private static final double STICK_ACTIVE = 0.055;

    /** Setpoint sweep rate at full stick, rad/s (~400 deg/s). */
    private static final double STICK_RATE_RAD_S = 7.0;

    /** How far the setpoint may lead the measured heading. */
    private static final double MAX_LEAD_RAD = Math.toRadians(60);

    /** Below this measured rotation rate the chassis counts as stopped. */
    private static final double STOPPED_RATE_RAD_S = Math.toRadians(15);

    /** Longest wait for rotation to stop before latching anyway. */
    private static final double STOP_TIMEOUT_S = 1.0;

    /** Commanded rotation above which the stuck-sensor guard starts counting. */
    private static final double STUCK_TURN = 0.15;

    /** Seconds of commanded rotation with a frozen heading before the loop gives up. */
    private static final double STUCK_LIMIT_S = 0.6;

    public enum Phase {
        /** Driver input or a live correction. */
        ACTIVE,
        /** Input released, waiting for the chassis to stop rotating. */
        STOPPING,
        /** Setpoint latched; the drivetrain's own zero-input behaviour owns the hold. */
        RESTING
    }

    private final PIDFController pidf;

    private double targetRad;
    private double previousHeadingRad = Double.NaN;
    private double rateRadS;
    private Phase phase = Phase.RESTING;
    private double stopTimerS;
    private double stuckSecondsS;
    private double lastHeadingSeen = Double.NaN;
    private boolean enabled = true;
    private boolean everRun;
    private String disabledReason = "";

    public HeadingHold() {
        this(new PIDFCoefficients(1.20, 0, 0.080, 0));
    }

    public HeadingHold(PIDFCoefficients coefficients) {
        pidf = new PIDFController(coefficients);
    }

    /**
     * One loop of the hold.
     *
     * @param stick raw turn-stick value, already deadbanded by the caller
     * @param headingRad measured robot heading
     * @param translating whether the driver is commanding translation
     * @param dt seconds since the previous call
     * @return the rotation command to hand the drivetrain
     */
    public double update(double stick, double headingRad, boolean translating, double dt) {
        if (!enabled) {
            return stick;
        }
        dt = Math.min(0.25, Math.max(1e-3, dt));

        if (!everRun || Double.isNaN(previousHeadingRad)) {
            targetRad = headingRad;
            previousHeadingRad = headingRad;
            lastHeadingSeen = headingRad;
            everRun = true;
            pidf.reset();
        }

        // Wrap-aware rotation rate, for the stopped gate and the stuck guard.
        rateRadS = MathFunctions.normalizeAngleSigned(headingRad - previousHeadingRad) / dt;
        previousHeadingRad = headingRad;

        boolean stickActive = Math.abs(stick) > STICK_ACTIVE;
        boolean demand = stickActive || translating;
        double turn;

        if (demand) {
            if (phase != Phase.ACTIVE) {
                phase = Phase.ACTIVE;
                // Re-adopt reality: the resting setpoint may be stale.
                targetRad = headingRad;
                pidf.reset();
            }
            if (stickActive) {
                targetRad = MathFunctions.normalizeAngle(targetRad + stick * STICK_RATE_RAD_S * dt);
                double lead = MathFunctions.getTurnDirection(headingRad, targetRad)
                        * MathFunctions.getSmallestAngleDifference(headingRad, targetRad);
                if (lead > MAX_LEAD_RAD) {
                    targetRad = MathFunctions.normalizeAngle(headingRad + MAX_LEAD_RAD);
                } else if (lead < -MAX_LEAD_RAD) {
                    targetRad = MathFunctions.normalizeAngle(headingRad - MAX_LEAD_RAD);
                }
            }
            turn = correction(headingRad);
        } else {
            if (phase == Phase.ACTIVE) {
                phase = Phase.STOPPING;
                stopTimerS = 0;
            }
            if (phase == Phase.STOPPING) {
                stopTimerS += dt;
                if (Math.abs(rateRadS) < STOPPED_RATE_RAD_S || stopTimerS > STOP_TIMEOUT_S) {
                    targetRad = headingRad;
                    pidf.reset();
                    phase = Phase.RESTING;
                }
            }
            turn = 0;
        }

        // Commanding real rotation against a heading that has not moved at all means the sensor
        // is dead, and this loop would spin the robot at full power on a frozen number.
        if (Math.abs(turn) > STUCK_TURN) {
            if (Math.abs(headingRad - lastHeadingSeen) < 1e-9) {
                stuckSecondsS += dt;
            } else {
                stuckSecondsS = 0;
            }
            if (stuckSecondsS > STUCK_LIMIT_S) {
                enabled = false;
                disabledReason = "heading frozen while turning - sensor looks dead";
                return 0;
            }
        } else {
            stuckSecondsS = 0;
        }
        lastHeadingSeen = headingRad;
        return turn;
    }

    private double correction(double headingRad) {
        // Signed shortest-path error, so the controller never takes the long way round.
        double error = MathFunctions.getTurnDirection(headingRad, targetRad)
                * MathFunctions.getSmallestAngleDifference(headingRad, targetRad);
        pidf.updateError(error);
        return MathFunctions.clamp(pidf.run(), -1.0, 1.0);
    }

    /** Forces the setpoint to the current heading. Call on start, or after any pose reset. */
    public void latch(double headingRad) {
        targetRad = headingRad;
        previousHeadingRad = headingRad;
        lastHeadingSeen = headingRad;
        phase = Phase.RESTING;
        stuckSecondsS = 0;
        everRun = true;
        pidf.reset();
    }

    public double targetRad() {
        return targetRad;
    }

    public double rateRadS() {
        return rateRadS;
    }

    public Phase phase() {
        return phase;
    }

    public boolean enabled() {
        return enabled;
    }

    public String disabledReason() {
        return disabledReason;
    }

    public void setEnabled(boolean on) {
        enabled = on;
        if (on) {
            disabledReason = "";
        }
    }

    /** Signed error, degrees, for telemetry and the recorder. */
    public double errorDeg(double headingRad) {
        return Math.toDegrees(MathFunctions.getTurnDirection(headingRad, targetRad)
                * MathFunctions.getSmallestAngleDifference(headingRad, targetRad));
    }
}
