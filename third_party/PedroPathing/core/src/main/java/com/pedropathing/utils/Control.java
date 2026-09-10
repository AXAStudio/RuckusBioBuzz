/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.utils;

import com.pedropathing.math.Vector2D;

public class Control {
    /**
     * Calculates the remaining magnitude on a unit circle after subtracting a component.
     */
    public static double getRemainingMagnitude(double totalMagnitude, double usedMagnitude) {
        return Math.sqrt(Math.max(0.0, totalMagnitude * totalMagnitude - usedMagnitude * usedMagnitude));
    }

    /**
     * Allocates power to a control component while respecting a total power budget.
     */
    public static double allocatePower(double requested, double budget) {
        return Math.copySign(Math.min(Math.abs(requested), budget), requested);
    }

    public static double findNormalizingScaling(
            Vector2D staticVector, Vector2D variableVector, double maxPowerScaling) {
        double a = Math.pow(variableVector.x(), 2) + Math.pow(variableVector.y(), 2);
        double b = staticVector.x() * variableVector.x() + staticVector.y() * variableVector.y();
        double c = Math.pow(staticVector.x(), 2) + Math.pow(staticVector.y(), 2) - Math.pow(maxPowerScaling, 2);
        double scaling = (-b + Math.sqrt(b * b - a * c)) / a;
        return Math.max(0.0, Math.min(1.0, scaling));
    }

    /**
     * Scales the control output using a cosine function to avoid continuing when deviating far from the target.
     */
    public static double cosineScale(double error, double falloffRadius) {
        double clamped = Math.min(Math.abs(error) * ((Math.PI / 2) / falloffRadius), Math.PI / 2);
        return Math.cos(clamped);
    }

    /**
     * Clamps the braking power to a maximum value when it is in the opposite direction of motion. This prevents burnouts and low voltage spikes.
     *
     * @param directionOfMotion +1 or -1
     * @param maxBrakingPower   positive
     */
    public static double clampBrakingPower(double power, double directionOfMotion, double maxBrakingPower) {
        if (directionOfMotion * power >= 0) {
            return power;
        }
        return Math.copySign(Math.min(Math.abs(power), maxBrakingPower), power);
    }

    /**
     * RUCKUS PATCH: {@link #clampBrakingPower(double, double, double)} applied to the vector
     * rather than per axis.
     *
     * <p>Capping one component and not the other does not shorten the command, it ROTATES it:
     * braking out of a forward-right drive clamped x to -0.2 while y kept -0.9, swinging the
     * commanded direction by 30-45 degrees, and it released the moment that velocity component
     * crossed zero, so it chattered as well. On a swerve every pod's azimuth is {@code atan2} of
     * that vector, so the rotation lands straight on the pod demands. Same intent, applied to the
     * vector: project onto the direction of travel, and if the opposing component exceeds the cap
     * scale the WHOLE vector down until it does. Direction is preserved exactly; only magnitude
     * changes. Ported from the 2.1.2 fork's CustomDrivetrain.clampReversePower.
     *
     * @param power     commanded translation, in the same frame as {@code motion}
     * @param motion    measured velocity, in the same frame as {@code power}
     * @param maxBrakingPower largest command component allowed against the direction of travel
     * @return the clamped command
     */
    public static Vector2D clampBrakingPower(Vector2D power, Vector2D motion, double maxBrakingPower) {
        double speed = motion.magnitude();
        if (speed < 1e-6) {
            return power;
        }
        double along = power.dot(motion) / speed;
        if (along >= -maxBrakingPower) {
            // Not opposing motion, or opposing it by less than the cap allows.
            return power;
        }
        return power.times(maxBrakingPower / -along);
    }

    /**
     * Scales all values proportionally so none exceed a magnitude of 1.0
     */
    public static void desaturate(double[] powers) {
        double max = 1.0;

        for (double power : powers) {
            max = Math.max(max, Math.abs(power));
        }

        if (max > 1.0) {
            double scale = 1 / max;
            for (int i = 0; i < powers.length; i++) {
                powers[i] *= scale;
            }
        }
    }
}
