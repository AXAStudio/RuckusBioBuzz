package org.firstinspires.ftc.teamcode.tele;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.systems.Intake;

import java.util.List;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.systems.Scoopula;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    private Follower follower;
    private PredictiveAiming predictor;
    private Shooter shooterSystem;
    private List<LynxModule> hubs;
    private Alliance alliance = Alliance.RED;
    private Intake intake;
    private Scoopula scoopula;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        Pose autoEndPose = PoseStorage.take();
        if (autoEndPose != null) {
            follower.setPose(autoEndPose);
        }
        predictor = new PredictiveAiming(follower);
        shooterSystem = new Shooter(hardwareMap, predictor, alliance);
        intake = new Intake(hardwareMap);
        scoopula = new Scoopula(hardwareMap);

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
        shooterSystem.setAlliance(alliance);
        intake.setAlliance(alliance);
    }

    @Override
    public void loop() {
        Pose pose = follower.pose();
        FieldView.publish(pose, predictor.predictPose());
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        Swerve.setTranslationPriority(SystemConstants.TeleOp.TRANSLATION_PRIORITY);
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        // Shooter first, so the intake follows this loop's SHOOT decision, not last loop's.
        shooterSystem.update();
        if(shooterSystem.shooting()){
            intake.update(false, true); //just sends intake command that's overridable
        }else {
            intake.update(gamepad1.left_bumper, gamepad1.right_bumper);
        }
        scoopula.update(gamepad2.left_bumper && gamepad2.right_bumper);

        telemetry.addData("alliance", alliance);
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("in zone", shooterSystem.inZone());
        telemetry.addData("ready", shooterSystem.ready());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
        Swerve.setTranslationPriority(0);
    }
}
