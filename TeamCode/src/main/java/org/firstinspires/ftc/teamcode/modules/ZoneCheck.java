package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

@Config
@Configurable
public class ZoneCheck {
    public static double ZONE_Y = 39;
    public static final double FIELD_SIZE = 141.5;
    public static final double ROBOT_LENGTH = 18;
    public static final double ROBOT_WIDTH = 18;
    private static int xQuality = 5;
    private static int yQuality = 10;
    private static int yPollenOffset = 0;
    private static int yNectarOffset = 0;
    private static int[][] mapNectar;
    private static int[][] mapPollen;
    private static Alliance alliance;

    public ZoneCheck(Alliance alliance) {
        this.alliance = alliance;
        mapPollen = new int[yQuality][xQuality];
        mapNectar = new int[yQuality][xQuality];
    }



    public static boolean inZoneNectar(Pose pose) {
        int nectarOffsetx = (int)(-yNectarOffset*Math.sin(pose.heading())); //this just spins the vecter
        int nectarOffsety = (int)(yNectarOffset*Math.cos(pose.heading())); //this just spins the vecter
        double x = pose.x()+nectarOffsetx;
        double y = pose.y()+nectarOffsety;
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
        if(mapNectar[xQuad][yQuad] == 1){
            return true;
        }
        return false;
//        double reach = ROBOT_LENGTH / 2 * Math.abs(Math.sin(pose.heading()))
//            + ROBOT_WIDTH / 2 * Math.abs(Math.cos(pose.heading()));
//        return pose.y() - reach <= ZONE_Y || pose.y() + reach >= FIELD_SIZE - ZONE_Y;
    }
    public static boolean inZonePollen(Pose pose) {
        int pollenOffsetx = (int)(-yPollenOffset*Math.sin(pose.heading())); //this just spins the vecter
        int pollenOffsety = (int)(yPollenOffset*Math.cos(pose.heading()));
        double x = pose.x()+pollenOffsetx;
        double y = pose.y()+pollenOffsety;
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
        if(mapPollen[xQuad][yQuad] == 1){
            return true;
        }
        return false;
//        double reach = ROBOT_LENGTH / 2 * Math.abs(Math.sin(pose.heading()))
//            + ROBOT_WIDTH / 2 * Math.abs(Math.cos(pose.heading()));
//        return pose.y() - reach <= ZONE_Y || pose.y() + reach >= FIELD_SIZE - ZONE_Y;
    }
}
