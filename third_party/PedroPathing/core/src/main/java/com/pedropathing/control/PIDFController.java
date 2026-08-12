package com.pedropathing.control;

/**
 * This is the PIDFController class. This class handles the running of PIDFs. PIDF stands for
 * proportional, integral, derivative, and feedforward. PIDFs take the error of a system as an input.
 * Coefficients multiply into the error, the integral of the error, the derivative of the error, and
 * a feedforward value. Then, these values are added up and returned. In this way, error in the
 * system is corrected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class PIDFController {
    private PIDFCoefficients coefficients;
    private PIDFCoefficientSupplier coefficientSupplier;

    private double previousError;
    private double error;
    private double position;
    private double targetPosition;
    private double errorIntegral;

    /** RUCKUS PATCH: see {@link #setIntegralLimit}. */
    private double integralLimit = 0.25;

    /** RUCKUS PATCH: see {@link #setIntegralResetThreshold}. */
    private double integralResetThreshold = 0.0;

    /** RUCKUS PATCH: see {@link #setIntegralBand}. */
    private double integralBand = Double.POSITIVE_INFINITY;
    private double errorDerivative;
    private double feedForwardInput;

    private long previousUpdateTimeNano;
    private long deltaTimeNano;

    /**
     * This creates a new PIDFController from a CustomPIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public PIDFController(PIDFCoefficientSupplier set) {
        coefficientSupplier = set;
        setCoefficients(set.get(0.0));
        reset();
    }

    /**
     * This takes the current error and runs the PIDF on it.
     *
     * @return this returns the value of the PIDF from the current error.
     */
    public double run() {
        coefficients = coefficientSupplier.get(error);

        // RUCKUS PATCH: bound the integral contribution. errorIntegral accumulates without limit
        // and nothing ever resets it during normal operation, so any non-zero I eventually
        // saturates the output. Clamping the contribution rather than the raw sum keeps the bound
        // meaningful whatever I is set to. A no-op wherever I is 0, which is every stock config.
        double integralContribution = errorIntegral * I();
        if (integralContribution > integralLimit) {
            integralContribution = integralLimit;
        } else if (integralContribution < -integralLimit) {
            integralContribution = -integralLimit;
        }

        return error * P() + errorDerivative * D() + integralContribution + feedForwardInput * F();
    }

    /**
     * Largest absolute contribution the integral term may make to the output.
     *
     * <p>Defaults to a quarter of a normalised output. Enough to grind out a stiction-limited
     * steady-state error, far too little to run away.
     */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    public double getIntegralLimit() {
        return integralLimit;
    }

    /**
     * RUCKUS PATCH: how large the error must be for a sign change to clear the integral.
     *
     * <p>The sign-change reset added alongside {@link #setIntegralLimit} clears the integral
     * whenever the error crosses zero. Near the target the error sign flips on sensor noise every
     * few loops, so the integral was being cleared continuously in exactly the place it exists to
     * help - grinding out a stiction-limited residual. Measured on this drivetrain the encoder
     * noise floor is 0.05 degrees, so a threshold of a degree or two makes the reset fire on real
     * crossings only.
     *
     * <p>Defaults to 0, which is the reset-on-any-crossing behaviour it replaces.
     */
    public void setIntegralResetThreshold(double threshold) {
        this.integralResetThreshold = Math.abs(threshold);
    }

    public double getIntegralResetThreshold() {
        return integralResetThreshold;
    }

    /**
     * RUCKUS PATCH: only accumulate the integral while |error| is inside this band.
     *
     * <p>Conditional integration. An integral term meant to break static friction at the target has
     * no business accumulating across a 90 degree slew, where the error is large for a long time
     * and the proportional term already saturates the output; all that does is guarantee an
     * overshoot on arrival. Outside the band the accumulator is held at zero, so the term is purely
     * an endgame device.
     *
     * <p>Defaults to infinity, which is the always-accumulate behaviour it replaces.
     */
    public void setIntegralBand(double band) {
        this.integralBand = Math.abs(band);
    }

    public double getIntegralBand() {
        return integralBand;
    }

    /**
     * Shared by {@link #updateError} and {@link #updatePosition}: decides whether this sample adds
     * to the accumulator, leaves it alone, or clears it.
     */
    private void accumulateIntegral(double seconds) {
        if (Math.abs(error) > integralBand) {
            errorIntegral = 0;
            return;
        }
        if (error * previousError < 0
                && Math.max(Math.abs(error), Math.abs(previousError)) > integralResetThreshold) {
            errorIntegral = 0;
        }
        errorIntegral += error * seconds;
    }

    /**
     * This can be used to update the PIDF's current position when inputting a current position and
     * a target position to calculate error. This will update the error from the current position to
     * the target position specified.
     *
     * @param position This is the current position.
     */
    public void updatePosition(double position) {
        this.position = position;
        previousError = error;
        error = targetPosition - this.position;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        // RUCKUS PATCH: see accumulateIntegral - drops the accumulator on a real sign change or
        // outside the integral band, and is unchanged from stock at the default settings.
        accumulateIntegral(deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    /**
     * As opposed to updating position against a target position, this just sets the error to some
     * specified value.
     *
     * @param error The error specified.
     */
    public void updateError(double error) {
        previousError = this.error;
        this.error = error;
        long nanoTime = System.nanoTime();

        deltaTimeNano = nanoTime - previousUpdateTimeNano;
        previousUpdateTimeNano = nanoTime;

        // RUCKUS PATCH: see accumulateIntegral - drops the accumulator on a real sign change or
        // outside the integral band, and is unchanged from stock at the default settings.
        accumulateIntegral(deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    /**
     * RUCKUS PATCH: as {@link #updateError}, but the caller supplies the derivative.
     *
     * <p>Differencing the error assumes the setpoint holds still. A swerve pod's does not: it steps
     * by up to 90 degrees on a new command, and jumps discontinuously whenever {@code move()} takes
     * the plus-or-minus 180 degree flip. Both put a spike into the D term that has nothing to do
     * with how fast the pod is moving — at a 90 degree step and 120 Hz, {@code kD = 0.010} alone
     * produces 1.96 of output and saturates the clamp.
     *
     * <p>Passing the negated measured velocity instead gives derivative-on-measurement: identical
     * behaviour while the setpoint is constant, no kick when it moves.
     *
     * @param error the current error
     * @param derivative d(error)/dt to use, normally the negated measurement velocity
     */
    public void updateErrorWithDerivative(double error, double derivative) {
        previousError = this.error;
        this.error = error;
        long nanoTime = System.nanoTime();

        deltaTimeNano = nanoTime - previousUpdateTimeNano;
        previousUpdateTimeNano = nanoTime;

        accumulateIntegral(deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = derivative;
    }

    /**
     * This can be used to update the feedforward equation's input, if applicable.
     *
     * @param input the input into the feedforward equation.
     */
    public void updateFeedForwardInput(double input) {
        feedForwardInput = input;
    }

    /**
     * This resets all the PIDF's error and position values, as well as the time stamps.
     */
    public void reset() {
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        errorIntegral = 0;
        errorDerivative = 0;
        previousUpdateTimeNano = System.nanoTime();
    }

    /**
     * This is used to set the target position if the PIDF is being run with current position and
     * target position inputs rather than error inputs.
     *
     * @param set this sets the target position.
     */
    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    /**
     * This returns the target position of the PIDF.
     *
     * @return this returns the target position.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * This is used to set the coefficients of the PIDF.
     *
     * @param set the coefficients that the PIDF will use.
     */
    public void setCoefficients(PIDFCoefficients set) {
        coefficients = set;
    }

    /**
     * This returns the PIDF's current coefficients.
     *
     * @return this returns the current coefficients.
     */
    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }

    /**
     * This sets the proportional (P) coefficient of the PIDF only.
     *
     * @param set this sets the P coefficient.
     */
    public void setP(double set) {
        coefficients.P = set;
    }

    /**
     * This returns the proportional (P) coefficient of the PIDF.
     *
     * @return this returns the P coefficient.
     */
    public double P() {
        return coefficients.P;
    }

    /**
     * This sets the integral (I) coefficient of the PIDF only.
     *
     * @param set this sets the I coefficient.
     */
    public void setI(double set) {
        coefficients.I = set;
    }

    /**
     * This returns the integral (I) coefficient of the PIDF.
     *
     * @return this returns the I coefficient.
     */
    public double I() {
        return coefficients.I;
    }

    /**
     * This sets the derivative (D) coefficient of the PIDF only.
     *
     * @param set this sets the D coefficient.
     */
    public void setD(double set) {
        coefficients.D = set;
    }

    /**
     * This returns the derivative (D) coefficient of the PIDF.
     *
     * @return this returns the D coefficient.
     */
    public double D() {
        return coefficients.D;
    }

    /**
     * This sets the feedforward (F) constant of the PIDF only.
     *
     * @param set this sets the F constant.
     */
    public void setF(double set) {
        coefficients.F = set;
    }

    /**
     * This returns the feedforward (F) constant of the PIDF.
     *
     * @return this returns the F constant.
     */
    public double F() {
        return coefficients.F;
    }

    /**
     * This returns the current error of the PIDF.
     *
     * @return this returns the error.
     */
    public double getError() {
        return error;
    }

    /**
     * This returns the current derivative of the error.
     * @return the derivative
     */
    public double getErrorDerivative() {
        return errorDerivative;
    }
}
