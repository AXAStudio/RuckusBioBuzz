package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.aimingSystem;
import org.firstinspires.ftc.teamcode.systems.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    public static double INTAKE_FEED_POWER = 1.0;
    public static boolean INTAKE_REVERSED = false;

    private Follower follower;
    private predictiveAiming predictor;
    private shooter shooter;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;
    aimingSystem aimer;

    private DcMotor intake;
    private final PIDFController aimPid = new PIDFController(new PIDFCoefficients(1.20, 0, 0.080, 0));
    private boolean aiming = false;

    @Override
    public void init() {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        if (PoseStorage.pose != null) {
            follower.setPose(PoseStorage.pose);
        }
        predictor = new predictiveAiming(follower);
        shooter = new shooter(hardwareMap, predictor);
        aimer = new aimingSystem(predictor, alliance);


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        telemetry.addData("alliance (X blue, B red)", alliance);
        telemetry.update();

    }


    @Override
    public void loop() {


        Pose current = follower.pose();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        Pose pose = follower.pose();
        FieldView.publish(pose, predictor.predictPose());

        boolean shooting = gamepad2.right_bumper;

        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (shooting) {
            intake.setPower(shooter.ready() ? INTAKE_FEED_POWER : 0);
        } else {
            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        }
        if(gamepad1.left_trigger > 0.5){
            double aimAngle = aimer.aimFrom(pose)[1];
            follower.hold(current.withHeading(aimAngle));

        }

        telemetry.addData("alliance", alliance);
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("shooting", shooting);

        telemetry.addData("ready", shooter.ready());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.setPower(0);
    }
}
