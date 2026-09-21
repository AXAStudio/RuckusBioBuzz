package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.systems.shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    public static boolean INTAKE_REVERSED = false;
    public static double INTAKE_FEED_BOOST = 0.5;

    private Follower follower;
    private predictiveAiming predictor;
    private shooter shooter;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;

    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
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

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;
        telemetry.addData("alliance (X blue, B red)", alliance);
        telemetry.update();

    }

    @Override
    public void start() {
        shooter.setAlliance(alliance);
    }


    @Override
    public void loop() {


        Pose current = follower.pose();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        boolean shooting = gamepad2.right_bumper;

        double turn;
        if (gamepad1.left_trigger > 0.5 || shooting) {
            if (!aiming) {
                aimPid.reset();
                aiming = true;
            }
            turn = aimTurn(current, shooter.targetHeading());
        } else {
            aiming = false;
            turn = -gamepad1.right_stick_x;
        }

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turn);
        follower.update();

        Pose pose = follower.pose();
        FieldView.publish(pose, predictor.predictPose());

        shooter.update(shooting, pose.heading());

        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        if (shooter.feeding()) {
            intakePower += INTAKE_FEED_BOOST;
        }
        intakePower = Math.max(-1, Math.min(1, intakePower));
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        telemetry.addData("alliance", alliance);
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("aiming", aiming);
        telemetry.addData("shooting", shooting);
        telemetry.addData("in zone", shooter.inZone());
        telemetry.addData("hive distance (in)", "%.1f", shooter.distance());
        telemetry.addData("aim error (deg)", "%.1f", shooter.aimErrorDeg());
        telemetry.addData("launcher vel / target", "%.0f / %.0f", shooter.velocity(), shooter.target());
        telemetry.addData("aimed / ready / feeding", shooter.aimed() + " / " + shooter.ready() + " / " + shooter.feeding());
        telemetry.update();
    }

    private double aimTurn(Pose pose, double targetHeading) {
        double error = Angle.error(pose.heading(), targetHeading);
        aimPid.updateErrorWithDerivative(error, -follower.velocity().omega);
        return Utils.clamp(aimPid.run(), -1.0, 1.0);
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.setPower(0);
        if (leftIntakeServo != null) leftIntakeServo.setPower(0);
        if (rightIntakeServo != null) rightIntakeServo.setPower(0);
    }
}
