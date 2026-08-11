package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Automatically tunes one pod's turn PIDF by experiment rather than by feel.
 *
 * <p>The procedure mirrors what a person does by hand, in the same order, because each stage
 * depends on the one before it:
 *
 * <ol>
 *   <li><b>kF</b> - ramp raw servo power up from zero until the pod breaks away from stiction, in
 *       both directions. The average breakaway power is the feed-forward. Measuring this first
 *       matters: without enough kF, every kP looks bad for the same reason.</li>
 *   <li><b>kP</b> - step the pod through a fixed angle at several gains and score the response.</li>
 *   <li><b>kD</b> - re-run at the winning kP, adding damping to kill the overshoot kP introduced.</li>
 * </ol>
 *
 * <p>This class holds no hardware. It is a state machine: the OpMode feeds it the measured angle
 * each loop and applies whatever it asks for. That keeps the motion safety checks - START pressed,
 * one pod at a time - in the OpMode where they belong.
 */
public class PodAutoTuner {

    /** What the OpMode should do with the pod this loop. */
    public enum Action {
        /** Drive the turn servo at {@link #rawServoPower} directly, bypassing the PID. */
        RAW_SERVO,
        /** Hold the pod at {@link #targetWheelRad} using the pod's own PIDF. */
        PID_HOLD,
        /** Stop the pod; tuning is over. */
        FINISHED
    }

    private enum Phase { KF_FORWARD, KF_REVERSE, SETTLE, TRIAL, DONE }

    // ---- procedure shape -------------------------------------------------------------

    /** Multipliers applied to the starting kP, then to the starting kD. */
    private static final double[] KP_STEPS = {0.5, 1.0, 1.5, 2.25, 3.5};
    private static final double[] KD_STEPS = {0.0, 1.0, 2.0, 4.0};

    /** Angle each trial steps through. Large enough to be decisive, small enough to be quick. */
    private static final double STEP_DEGREES = 90.0;

    private static final double TRIAL_SECONDS = 1.30;
    private static final double SETTLE_SECONDS = 0.70;

    /** kF ramp: power climbs this fast until the pod moves. */
    private static final double KF_RAMP_PER_SECOND = 0.28;
    private static final double KF_MAX_POWER = 0.55;
    private static final double KF_BREAKAWAY_DEGREES = 4.0;

    /** A trial counts as settled once it is inside this band. */
    private static final double SETTLED_DEGREES = 3.0;

    // ---- outputs the OpMode reads ----------------------------------------------------

    public double rawServoPower;
    public double targetWheelRad;

    /** Set when the coefficients changed and the pod must be rebuilt before the next loop. */
    public boolean rebuildRequested;

    // ---- state -----------------------------------------------------------------------

    private final List<String> log = new ArrayList<>();

    private Phase phase = Phase.DONE;
    private int podIndex = -1;

    private double baseKp;
    private double baseKd;

    private double kfForward;
    private double kfReverse;
    private double rampPower;
    private double rampStartDeg;

    private boolean tuningKd;
    private int stepIndex;

    private double bestScore = Double.MAX_VALUE;
    private double bestKp;
    private double bestKd;

    private double trialStartDeg;
    private double trialTargetDeg;
    private double trialElapsed;
    private double trialWorstOvershoot;
    private double trialSettleTime;
    private boolean trialSettled;
    private double lastErrorDeg;
    private int trialReversals;

    private double phaseElapsed;
    private String status = "idle";

    // ---- lifecycle -------------------------------------------------------------------

    /** Begins tuning the given pod. The pod must already be zeroed and reading its encoder. */
    public void start(int index, PodCal cal, double currentWheelDeg) {
        log.clear();
        podIndex = index;
        baseKp = cal.kP;
        baseKd = cal.kD;

        kfForward = 0;
        kfReverse = 0;
        rampPower = 0;
        rampStartDeg = currentWheelDeg;

        tuningKd = false;
        stepIndex = 0;
        bestScore = Double.MAX_VALUE;
        bestKp = cal.kP;
        bestKd = cal.kD;

        phase = Phase.KF_FORWARD;
        phaseElapsed = 0;
        rawServoPower = 0;
        targetWheelRad = Math.toRadians(currentWheelDeg);
        rebuildRequested = false;

        status = "measuring kF, forward";
        log.add("Tuning pod " + index + ". Starting kP=" + fmt(baseKp) + " kD=" + fmt(baseKd));
    }

    public boolean isRunning() {
        return phase != Phase.DONE;
    }

    public String status() {
        return status;
    }

    public List<String> log() {
        return log;
    }

    public int podIndex() {
        return podIndex;
    }

    /**
     * Advances the state machine.
     *
     * @param dt seconds since the last call
     * @param wheelDeg the pod's measured wheel heading, in degrees
     * @param cal the pod's calibration, written in place as coefficients are chosen
     * @return what the OpMode should do with the pod this loop
     */
    public Action update(double dt, double wheelDeg, PodCal cal) {
        rebuildRequested = false;
        phaseElapsed += dt;

        switch (phase) {
            case KF_FORWARD:
                return rampForKf(dt, wheelDeg, cal, true);
            case KF_REVERSE:
                return rampForKf(dt, wheelDeg, cal, false);
            case SETTLE:
                return settle(cal);
            case TRIAL:
                return runTrial(dt, wheelDeg, cal);
            case DONE:
            default:
                rawServoPower = 0;
                return Action.FINISHED;
        }
    }

    // ---- kF ---------------------------------------------------------------------------

    private Action rampForKf(double dt, double wheelDeg, PodCal cal, boolean forward) {
        rampPower += KF_RAMP_PER_SECOND * dt;
        rawServoPower = forward ? rampPower : -rampPower;

        double moved = Math.abs(shortestDelta(wheelDeg, rampStartDeg));

        if (moved >= KF_BREAKAWAY_DEGREES) {
            if (forward) {
                kfForward = rampPower;
                log.add(String.format(Locale.US, "kF forward breakaway at %.3f", kfForward));
                beginKfReverse(wheelDeg);
            } else {
                kfReverse = rampPower;
                log.add(String.format(Locale.US, "kF reverse breakaway at %.3f", kfReverse));
                finishKf(cal, wheelDeg);
            }
            return Action.RAW_SERVO;
        }

        if (rampPower >= KF_MAX_POWER) {
            // Never broke away. Either the servo is unpowered or the pod is jammed; record the
            // ceiling and move on rather than stalling the whole procedure here.
            if (forward) {
                kfForward = KF_MAX_POWER;
                log.add("kF forward never broke away - check servo power or binding");
                beginKfReverse(wheelDeg);
            } else {
                kfReverse = KF_MAX_POWER;
                log.add("kF reverse never broke away - check servo power or binding");
                finishKf(cal, wheelDeg);
            }
        }

        return Action.RAW_SERVO;
    }

    private void beginKfReverse(double wheelDeg) {
        phase = Phase.KF_REVERSE;
        rampPower = 0;
        rampStartDeg = wheelDeg;
        phaseElapsed = 0;
        status = "measuring kF, reverse";
    }

    private void finishKf(PodCal cal, double wheelDeg) {
        cal.kF = (kfForward + kfReverse) / 2.0;
        log.add(String.format(Locale.US, "kF = %.4f", cal.kF));
        rebuildRequested = true;

        cal.kP = baseKp * KP_STEPS[0];
        cal.kD = baseKd;
        beginTrial(cal, wheelDeg);
    }

    // ---- trials -----------------------------------------------------------------------

    private void beginTrial(PodCal cal, double wheelDeg) {
        trialStartDeg = wheelDeg;
        trialTargetDeg = wheelDeg + STEP_DEGREES;
        targetWheelRad = Math.toRadians(trialTargetDeg);

        trialElapsed = 0;
        trialWorstOvershoot = 0;
        trialSettleTime = TRIAL_SECONDS;
        trialSettled = false;
        trialReversals = 0;
        lastErrorDeg = STEP_DEGREES;

        rawServoPower = 0;
        phase = Phase.TRIAL;
        phaseElapsed = 0;
        status = String.format(Locale.US, "%s trial %d/%d  kP=%.3f kD=%.4f",
                tuningKd ? "kD" : "kP", stepIndex + 1,
                tuningKd ? KD_STEPS.length : KP_STEPS.length, cal.kP, cal.kD);
    }

    private Action runTrial(double dt, double wheelDeg, PodCal cal) {
        trialElapsed += dt;

        double error = shortestDelta(trialTargetDeg, wheelDeg);
        double travelled = shortestDelta(wheelDeg, trialStartDeg);

        // Overshoot is distance past the target, in the direction of travel.
        double overshoot = travelled - STEP_DEGREES;
        if (overshoot > trialWorstOvershoot) {
            trialWorstOvershoot = overshoot;
        }

        if (!trialSettled && Math.abs(error) <= SETTLED_DEGREES) {
            trialSettled = true;
            trialSettleTime = trialElapsed;
        }

        // Sign changes in the error indicate ringing rather than a clean approach.
        if (error * lastErrorDeg < 0) {
            trialReversals++;
        }
        lastErrorDeg = error;

        if (trialElapsed < TRIAL_SECONDS) {
            return Action.PID_HOLD;
        }

        scoreTrial(cal, Math.abs(error));
        return Action.PID_HOLD;
    }

    private void scoreTrial(PodCal cal, double finalErrorDeg) {
        // Settling time dominates; overshoot, ringing and residual error are penalties. A trial
        // that never settles carries the full duration, so it always loses to one that did.
        double score = trialSettleTime
                + 0.012 * Math.max(0, trialWorstOvershoot)
                + 0.030 * finalErrorDeg
                + 0.040 * trialReversals;

        log.add(String.format(Locale.US,
                "%s kP=%.3f kD=%.4f -> settle %.2fs, overshoot %.1f deg, ring %d, err %.1f deg"
                        + " (score %.3f)",
                tuningKd ? "kD" : "kP", cal.kP, cal.kD,
                trialSettleTime, Math.max(0, trialWorstOvershoot), trialReversals,
                finalErrorDeg, score));

        if (score < bestScore) {
            bestScore = score;
            bestKp = cal.kP;
            bestKd = cal.kD;
        }

        stepIndex++;
        phase = Phase.SETTLE;
        phaseElapsed = 0;
        status = "settling";
    }

    /** Pause between trials so each one starts from a stationary pod. */
    private Action settle(PodCal cal) {
        if (phaseElapsed < SETTLE_SECONDS) {
            return Action.PID_HOLD;
        }
        return advance(cal);
    }

    private Action advance(PodCal cal) {
        if (!tuningKd) {
            if (stepIndex < KP_STEPS.length) {
                cal.kP = baseKp * KP_STEPS[stepIndex];
                cal.kD = baseKd;
                rebuildRequested = true;
                beginTrial(cal, Math.toDegrees(targetWheelRad));
                return Action.PID_HOLD;
            }

            // kP sweep over: lock in the winner and start damping it.
            log.add(String.format(Locale.US, "best kP = %.3f", bestKp));
            tuningKd = true;
            stepIndex = 0;
            bestScore = Double.MAX_VALUE;
        }

        if (stepIndex < KD_STEPS.length) {
            cal.kP = bestKp;
            cal.kD = baseKd * KD_STEPS[stepIndex];
            rebuildRequested = true;
            beginTrial(cal, Math.toDegrees(targetWheelRad));
            return Action.PID_HOLD;
        }

        cal.kP = bestKp;
        cal.kD = bestKd;
        rebuildRequested = true;
        phase = Phase.DONE;
        rawServoPower = 0;
        status = String.format(Locale.US, "done: kP=%.3f kD=%.4f kF=%.4f",
                cal.kP, cal.kD, cal.kF);
        log.add("Final: " + status);
        return Action.FINISHED;
    }

    // ---- helpers ----------------------------------------------------------------------

    /** Signed difference a - b wrapped to [-180, 180]. */
    private static double shortestDelta(double a, double b) {
        double d = (a - b) % 360.0;
        if (d > 180) {
            d -= 360;
        }
        if (d < -180) {
            d += 360;
        }
        return d;
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.4f", v);
    }
}
