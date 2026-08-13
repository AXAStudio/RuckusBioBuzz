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
            .useSecondaryDrivePIDF(false).useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008, 0))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0))

            // Measured 2026-08-11 on the Pinpoint IMU (no odometry pods needed), across commanded
            // rotations of 15/45/90/135/180 degrees in both directions.
            //
            // kP: below ~1.0 the robot never settles (0.80 left 3-4 degrees standing); 1.40 hits a
            // stability edge, overshooting 22 then 51 degrees. 1.20 settles in ~0.65s.
            // kD: 0.003 overshot 8-12 degrees on anything past 45 degrees, because the output
            // saturates on large errors and the robot arrives carrying momentum. 0.030 cuts that to
            // 3-5 degrees for ~0.5 degrees more residual, and leaves small rotations clean
            // (15 degrees: no overshoot, settles in 0.22s).
            // kF stays 0: like the pod turn PIDF it is applied as a sign-only relay, and 0.06
            // produced 23-26 oscillations per step.
            //
            // Verified holding heading to +/-3 degrees while translating through two full circles.
            //
            // If you change these, change SwerveBringUp's headingKp/headingKd to match. The
            // bring-up tool keeps its own copy for live experimentation and does not read this
            // file; it sat at the pre-tuning 1.75/0.003 for a week after this line moved to
            // 1.20/0.030, so anyone opening the heading routine was measuring against a
            // baseline the robot had stopped using.
            .headingPIDFCoefficients(new PIDFCoefficients(1.20, 0, 0.030, 0))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.015, 0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
            //.secondaryDrivePIDFCoefficients(
            //        new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13))


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
            // Pods snap to an X with no input, so the robot resists being pushed. Note this makes
            // any momentary drop to zero input visible as all four pods yanking to the X - which is
            // correct behaviour, but during bring-up it hides what the pods are doing. Swap to
            // IGNORE_ANGLE_CHANGES while diagnosing, then put this back.
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.X_LOCK)
            .useBrakeModeInTeleOp(true);

    // Zero offsets, analog ranges, encoder pairings and drive directions below come from a
    // Swerve Bring-Up session on 2026-08-10, verified driving.
    //
    // ---------------------------------------------------------------------------------------
    // TURN GAINS: INTERIM, NOT FINAL. Updated 2026-08-12. Off the ground only - NOT yet
    // validated on carpet, at competition weight, or on a drained battery.
    // ---------------------------------------------------------------------------------------
    //
    // Committed because they are a large, measured improvement on what shipped, not because they
    // are finished. Randomised interleaved A/B against the previous gains, same battery, 40
    // pod-runs each, 90 degree steps on blocks at 13.1 V:
    //
    //     steady-state error   3.23 -> 0.96 deg mean,  7.24 -> 2.52 deg max
    //     error sign changes   2.35 -> 0.85 per step
    //     post-settle p-p      0.53 -> 0.21 deg mean, 0.45 max
    //
    // kD chosen at n=60 pod-runs against three rivals, randomised, 12.87-12.95 V. Selection was
    // lexicographic: zero loose-mode runs first, then lowest residual. kD 0.022 was the only one
    // of four to hold 0/60 (true rate bounded below 6%); kD 0.020, 0.014 and 0.010 drew 3, 1 and 1.
    // At kD 0.022: residual at t=3 s 0.77 deg mean / 2.36 max, settle to +/-2.0 deg 0.647 s with
    // 97% of pods settling, post-settle p-p 1.24 max. Per pod the residual is 0.76 / 0.73 / 1.06 /
    // 0.52 deg, so it is a broad distribution rather than one bad pod - 35% of pod-runs exceed
    // 1.0 deg and 12% exceed 1.5 deg.
    //
    // What was actually wrong, in order of size:
    //
    // 1. Loop rate. The turn PID runs at the OpMode loop rate, which was 33 Hz under a four-pod
    //    hold - slower than the servos' own 20 ms PWM frame. Raising it to ~100 Hz cut error sign
    //    changes from 1-4 to 0-1 with the gains untouched. DriveTeleOp was 30.7 Hz and is now
    //    75.8 Hz; see that file. This was the single biggest lever and it is not a gain.
    //
    // 2. kF is a relay, not a feed-forward. CoaxialPod feeds the PIDF a sign, so the F term is
    //    +/-kF. Measured breakaway is 0.025-0.050, so the shipped kF of 0.005 was 5-10x too small
    //    to do anything, and the pod parked where kP*e fell under breakaway: 0.035/0.300 = 6.7 deg.
    //    Setting kF to breakaway does fix the residual (3.52 -> 1.25 deg) and limit-cycles at
    //    42.5 deg peak-to-peak. Replaced by kS, a continuous tanh static-friction term with the
    //    same authority away from the target and no switching discontinuity at it. kF is now 0.
    //
    // 3. servoCachingThreshold 0.05 was wider than the entire breakaway range, so a settled pod's
    //    written power was frozen across a ~19 degree error window. Now 0.01; below that the extra
    //    bus traffic costs loop rate (96 -> 77 Hz) for no measurable gain.
    //
    // Negative results, all measured, so nobody repeats them:
    //
    //   - Encoder noise is NOT why kD misbehaves. Noise floor is sigma 0.042-0.054 deg against a
    //     0.1125 deg read granularity, which pushes 0.0006 of output through kD 0.010. The old
    //     "kD above 0.010 adds ringing" note was a loop-rate artefact.
    //   - Derivative-on-measurement does not permit more damping. Implemented and available
    //     (default off); kD 0.040 rings 16x, 0.070 and 0.110 are unusable.
    //   - Widening the servo PWM pulse range 600-2400 -> 500-2500 us does nothing. The servo
    //     already reaches ~90% of top speed at 0.3 power, so the extra range is never used.
    //   - No measurable backlash. Approaching one heading from both sides 8 times each, the
    //     directional gap is 0.01-0.60 deg and no pod's gap exceeds twice its standard error.
    //   - No mechanical outlier pod. Loose-mode rate is 12.8/17.4/18.2/20.7 percent across the
    //     four, statistically one band.
    //   - kI is still 0, but the old "30-45 degrees of hunting" result is not why. That predates
    //     both the caching fix and PIDFController's integral band and reset threshold. Re-testable.
    //
    // Known not met, off the ground: 90 degree settle to +/-2.0 deg is ~0.5 s against a 350 ms
    // target; residual at t=3 s is 0.54-1.0 deg mean but up to 2.5 deg worst case; pod-to-pod
    // spread is 24-84% against 15%. Roughly 17% of step responses land in a wide mode with 25 deg
    // of peak-to-peak rather than the usual 0.23 deg - that bimodality is gain-controllable and
    // is the open problem. Full trial log and traces in tools/swervetune.

    // Public so the bring-up tool can read them and report any divergence from its own working
    // copy. The tool is supposed to be able to hold different gains - that is what it is for - but
    // it should never be possible to measure at gains nobody intended, which is what happened when
    // its calibration file sat at kD 0.010 while this file said 0.022.
    public static final double turnKP = 0.200;
    public static final double turnKD = 0.022;

    /** Static-friction feed-forward, set to the measured breakaway. Replaces the kF relay. */
    public static final double turnKS = 0.035;
    public static final double turnKSBandDeg = 2.0;

    /** Servo output caching threshold applied to every pod. */
    public static final double turnServoCaching = 0.01;

    private static double dtLength = 146.420; //distance from robot center to front/back pod center
    private static double dtWidth = 154.240; // distance from robot center to left/right pod center

    // The encoder names do NOT follow the sm#/ss#/se# numbering. The Axon feedback wires are
    // spliced two per analog port, and the wiring scan found every pod landing on a different
    // channel than its own number. Do not "tidy" these to match.
    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm2", "ss2", "se3",
                new PIDFCoefficients(turnKP, 0, turnKD, 0), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(338.2), new Pose(dtLength, dtWidth),
                0.136, 3.336, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(turnServoCaching);
        pod.setStaticFriction(turnKS, Math.toRadians(turnKSBandDeg));
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm1", "ss1", "se0",
                new PIDFCoefficients(turnKP, 0, turnKD, 0), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(323.3), new Pose(dtLength, -dtWidth),
                0.145, 3.350, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(turnServoCaching);
        pod.setStaticFriction(turnKS, Math.toRadians(turnKSBandDeg));
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm3", "ss3", "se2",
                new PIDFCoefficients(turnKP, 0, turnKD, 0), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(62.2), new Pose(-dtLength, dtWidth),
                0.266, 3.467, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(turnServoCaching);
        pod.setStaticFriction(turnKS, Math.toRadians(turnKSBandDeg));
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, "sm0", "ss0", "se1",
                new PIDFCoefficients(turnKP, 0, turnKD, 0), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, Math.toRadians(32.0), new Pose(-dtLength, -dtWidth),
                0.075, 3.247, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(turnServoCaching);
        pod.setStaticFriction(turnKS, Math.toRadians(turnKSBandDeg));
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
