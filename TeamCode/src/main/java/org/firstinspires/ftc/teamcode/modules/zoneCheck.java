package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

@Config
@Configurable
public class zoneCheck {
    public static double ZONE_Y = 39;
    public static final double FIELD_SIZE = 141.5;
    public static final double ROBOT_LENGTH = 18;
    public static final double ROBOT_WIDTH = 18;
    private static int xQuality = 5;
    private static int yQuality = 10;
    private static int[][] map;
    private static Alliance alliance;


    private final predictiveAiming predictor;

    public zoneCheck(predictiveAiming predictor, Alliance alliance) {
        this.predictor = predictor;
        this.alliance = alliance;
        map = new int[yQuality][xQuality];
    }

    public boolean predictedInZone() {
        return inZone(predictor.predictPose());
    }

    public static boolean inZone(Pose pose) {
        double x = pose.x();
        double y = pose.y();
        double fieldx = 141.5;
        double fieldy = 141.5;
        if(alliance == Alliance.BLUE){
            x = fieldx-x;
            y = fieldy-y;
        }
        int xQuad = (int)(x/(fieldx/xQuality));
        int yQuad = (int)(y/(fieldy/yQuality));
        telemetry.addData("xQuad", xQuad);
        telemetry.addData("yQuad", yQuad);
        telemetry.update();
        if(map[xQuad][yQuad] == 1){
            return true;
        }
        return false;
//        double reach = ROBOT_LENGTH / 2 * Math.abs(Math.sin(pose.heading()))
//            + ROBOT_WIDTH / 2 * Math.abs(Math.cos(pose.heading()));
//        return pose.y() - reach <= ZONE_Y || pose.y() + reach >= FIELD_SIZE - ZONE_Y;
    }
}
