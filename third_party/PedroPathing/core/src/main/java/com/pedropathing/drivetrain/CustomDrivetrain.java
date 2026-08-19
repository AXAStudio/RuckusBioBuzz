package com.pedropathing.drivetrain;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.math.Vector;

/**
 * This is the CustomDrivetrain class. This is an abstract class that extends the Drivetrain class.
 * It is intended to be used as a base class for custom drivetrain implementations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @author Kabir Goyal
 */
public abstract class CustomDrivetrain extends Drivetrain {
    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;

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
    public abstract void arcadeDrive(double forward, double strafe, double rotation);

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }

        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            }
        }
    }


    protected boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    protected Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)) :
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower,
                         Vector pathingPower, double robotHeading, Vector robotVelocity) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;

        lastTranslationalVector = translationalVector; //before rotation
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;
        
        robotVelocity.rotateVector(-robotHeading); // converts field relative velocity
        // to robot relative
        
        translationalVector.rotateVector(-robotHeading); // this should make it field centric when field centric is desired and robot centric otherwise

        double[] clamped = clampReversePower(translationalVector.getXComponent(),
                                             translationalVector.getYComponent(),
                                             robotVelocity.getXComponent(),
                                             robotVelocity.getYComponent());

        arcadeDrive(clamped[0], clamped[1], calculatedDrive[2]);
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {}
    
    /**
     * Prevents the robot from applying too much power in the opposite direction of
     * the robot's momentum. Alternating full forward (+1) and full reverse (-1) power
     * causes the control hub to restart due to low voltage spikes. This fixes it by
     * capping the amount of voltage applied opposite to the direction of motion to be
     * very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like
     * zero-power brake mode, using the motor’s own momentum for braking without consuming
     * significant energy.
     *
     * <p>RUCKUS PATCH: this used to run per axis - {@code clampReversePower(vx, motionX)} and
     * {@code clampReversePower(vy, motionY)} independently. Capping one component and not the
     * other does not shorten the command, it ROTATES it: braking out of a forward-right drive
     * clamped x to -0.2 while y kept -0.9, swinging the commanded direction by 30-45 degrees,
     * and it released the moment that velocity component crossed zero, so it chattered as well.
     * On a swerve every pod's azimuth is {@code atan2} of that vector, so the rotation lands
     * straight on the pod demands. Same intent, applied to the vector: project onto the
     * direction of travel, and if the opposing component exceeds the cap scale the WHOLE vector
     * down until it does. Direction is preserved exactly; only magnitude changes.
     *
     * @return the clamped {x, y} command
     */
    private double[] clampReversePower(double x, double y, double motionX, double motionY) {
        double speed = Math.hypot(motionX, motionY);
        if (speed < 1e-6) {
            return new double[] {x, y};
        }
        double ux = motionX / speed;
        double uy = motionY / speed;
        double along = x * ux + y * uy;
        if (along >= -REVERSE_POWER_CAP) {
            // Not opposing motion, or opposing it by less than the cap allows.
            return new double[] {x, y};
        }
        double scale = REVERSE_POWER_CAP / -along;
        return new double[] {x * scale, y * scale};
    }

    /** Largest command component allowed against the direction of travel. */
    private static final double REVERSE_POWER_CAP = 0.2;
}
