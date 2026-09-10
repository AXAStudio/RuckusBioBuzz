package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro 3 port of the mecanum constants. Nothing here has been run under Pedro 3.
 *
 * <p>Carried from the 2.x tuner output: the velocities (xVelocity/yVelocity) and the zero-power
 * accelerations, which are the quantities Foresight calls max achievable velocity and natural
 * deceleration. Heading and translational gains are the 2.x PIDF values (the heading kF relay
 * becomes headingStaticFF; the translational kF has no Foresight equivalent and is dropped).
 * Brake coefficients are derived from the deceleration as pure coast, v^2/(2a), and coast/brake
 * kV is v/vmax - both placeholders. Retune with the Foresight Tuner before any path.
 */
public class MecanumDrivetrainConstants {
    private static final double FORWARD_VELOCITY = 77.85071954201526;
    private static final double STRAFE_VELOCITY = 59.95660688745694;
    private static final double FORWARD_DECELERATION = 31.427000757540114;
    private static final double STRAFE_DECELERATION = 60.965944737115656;

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.proportional(0.87));
        c.headingStaticFF.set(Controller.staticFeedforward(0.03));
        c.forwardTranslational.set(Controller.pid(0.093, 0, 0.013));
        c.strafeTranslational.set(Controller.pid(0.093, 0, 0.013));

        c.coast.set(Controller.proportionalFeedforward(1.0 / FORWARD_VELOCITY));
        c.brake.set(Controller.proportionalFeedforward(1.0 / FORWARD_VELOCITY));

        c.maxAchievableForwardVelocity.set(FORWARD_VELOCITY);
        c.maxAchievableStrafeVelocity.set(STRAFE_VELOCITY);
        c.naturalForwardDeceleration.set(FORWARD_DECELERATION);
        c.naturalStrafeDeceleration.set(STRAFE_DECELERATION);

        c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
        c.quadraticBrakeCoefficients.set(Matrix.diag(
                1.0 / (2 * FORWARD_DECELERATION), 1.0 / (2 * STRAFE_DECELERATION)));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));
    });

    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontRightName.set("fR");
        c.backRightName.set("bR");
        c.backLeftName.set("bL");
        c.frontLeftName.set("fL");
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    // 2.x forwardPodY / strafePodX map to xPodOffset / yPodOffset (see SwerveDrivetrainConstants).
    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(0.629921); //inches distance from center
        c.yPodOffset.set(2.99213); // redo center of odo pods
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); //do tuning and check if the x goes up or down
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); //do tuning and check if the y goes up or down
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(new PinpointLocalizer(hardwareMap, pinpointConfig),
                new Mecanum(hardwareMap, driveConfig), new Foresight(foresightConfig));
    }
}
