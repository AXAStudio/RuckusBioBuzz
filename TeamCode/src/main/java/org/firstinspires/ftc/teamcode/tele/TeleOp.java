package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.systems.intake.intakeState;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.systems.intake;

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
    public static double INTAKE_FEED_POWER = 1.0;
    public static boolean INTAKE_REVERSED = false;

    private Follower follower;
    private predictiveAiming predictor;
    private shooter shooter;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;
    private intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        intake = new intake(hardwareMap);

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
        Pose pose = follower.pose();
        FieldView.publish(pose, predictor.predictPose());
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        if ((shooter.ready()) || gamepad1.right_bumper) {
            intake.setState(intakeState.INTAKE);
        } else if(gamepad1.left_bumper) {
            intake.setState(intakeState.REJECT);
        }else{
            intake.setState(intakeState.STOP);
        }

        shooter.update();
        intake.update();

        telemetry.addData("alliance", alliance);
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("in zone", shooter.inZone());
        telemetry.addData("pollen velocity", shooter.pollenVelocity());
        telemetry.addData("pollen target", shooter.pollenTarget());
        telemetry.addData("nectar velocity", shooter.nectarVelocity());
        telemetry.addData("nectar target", shooter.nectarTarget());
        telemetry.addData("aimed pollen / nectar", shooter.pollenAimed() + " / " + shooter.nectarAimed());
        telemetry.addData("ready", shooter.ready());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        if (shooter != null) shooter.stop();
    }
}
