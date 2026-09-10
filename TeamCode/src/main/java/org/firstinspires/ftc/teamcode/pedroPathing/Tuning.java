package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Pedro 3 tuners, served by AutoTune ({@code com.pedropathing:tuning}) on port 10158 of the
 * robot controller. The procedures in {@code procedures/} are the Pedro Quickstart's
 * ({@code pedro3} branch, f96afa6), unmodified apart from the package.
 *
 * <p>These drive the SWERVE drivetrain from {@link SwerveDrivetrainConstants}. None of them know
 * about the bring-up tool's safe-area box - it does not exist outside SwerveBringUp - and their
 * default distances are 48 in forward and sideways (36 in for braking). The practice area is
 * 51 x 46 in. Lower the distance inputs to fit, or run on a full field.
 *
 * <p>Replaces the 2.x Tuning SelectableOpMode, whose tuners targeted the deleted 2.x follower.
 */
public class Tuning {
    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(SwerveDrivetrainConstants::createLocalizer,
                SwerveDrivetrainConstants::createSwerve);
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /** Hold/line/curve tests follow paths, so they go through the same guard as the autos. */
    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(SwerveDrivetrainConstants::createSwerve,
                SwerveDrivetrainConstants::createLocalizer, () -> {
                    SwerveDrivetrainConstants.requireForesightMeasured();
                    return new Foresight(SwerveDrivetrainConstants.foresightConfig);
                });
    }
}
