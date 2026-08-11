package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing constants for the swerve drivetrain. Selected by {@link Constants} when
 * config.jsonc declares {@code "drivetrain": "swerve"}.
 */
public class SwerveDrivetrainConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-197.1)
            .lateralZeroPowerAcceleration(-197.1)
            .useSecondaryDrivePIDF(true).useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0))

            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.003, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.015, 0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13))


            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            // .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))

            // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
            // 0.05, //0.05 to 0.3
            // 0,//0.38735914623969386,
            // 0.002)
            // )
            .centripetalScaling(0.0005).
            mass(13.732);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.9350393701) //-74.5mm
            .strafePodX(-5.9133858268) //-150.2
            .distanceUnit(DistanceUnit.INCH).hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .velocity(73.9)
            // BRING-UP SETTING: pods hold their heading when the sticks are neutral. Switch back to
            // X_LOCK for competition, where snapping to an X to resist being pushed is what you
            // want. During bring-up it hides what the pods are actually doing, because every
            // momentary drop to zero input yanks all four back to the X.
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
            .useBrakeModeInTeleOp(true);

    // Zero offsets, analog ranges, encoder pairings and drive directions below come from a
    // Swerve Bring-Up session on 2026-08-10, verified driving. kF is NOT yet tuned - the bring-up
    // used one value for every pod, so the back pods may still want more than the front.
    private static double kP = 0.315100;
    private static double kD = 0.008600;
    private static double kFFront = 0.0130;
    private static double kFBack = 0.0130;

    private static double dtLength = 146.420; //distance from robot center to front/back pod center
    private static double dtWidth = 154.240; // distance from robot center to left/right pod center

    // The encoder names do NOT follow the sm#/ss#/se# numbering. The Axon feedback wires are
    // spliced two per analog port, and the wiring scan found every pod landing on a different
    // channel than its own number. Do not "tidy" these to match.
    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm2", "ss2", "se3",
                new PIDFCoefficients(kP, 0, kD, kFFront), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(338.2), new Pose(dtLength, dtWidth),
                0.136, 3.336, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm1", "ss1", "se0",
                new PIDFCoefficients(kP, 0, kD, kFFront), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(323.3), new Pose(dtLength, -dtWidth),
                0.145, 3.350, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm3", "ss3", "se2",
                new PIDFCoefficients(kP, 0, kD, kFBack), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(62.2), new Pose(-dtLength, dtWidth),
                0.266, 3.467, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm0", "ss0", "se1",
                new PIDFCoefficients(kP, 0, kD, kFBack), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(32.0), new Pose(-dtLength, -dtWidth),
                0.075, 3.247, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    public static PathConstraints pathConstraints =
            new PathConstraints(
                    0.9,
                    2,
                    2,
                    0.03,
                    50,
                    1,
                    10,
                    1
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap),
                        leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants).build();
    }
}
