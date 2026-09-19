package org.firstinspires.ftc.teamcode.tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

import org.firstinspires.ftc.teamcode.fieldview.FieldView;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private Follower follower;
    private predictiveAiming predictor;
    private List<LynxModule> hubs;

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
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        Pose pose = follower.pose();
        FieldView.publish(pose, predictor.predictPose());

        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        follower.update();
    }
}
