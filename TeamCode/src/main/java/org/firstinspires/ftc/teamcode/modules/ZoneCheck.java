package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.configs.ZoneCheckConfig;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

@Config
@Configurable
public class ZoneCheck {

    private static int[][] mapNectar;
    private static int[][] mapPollen;
    private static Alliance alliance;
    private ShooterType shooterType;
    public enum ShooterType{
        POLLEN, NECTAR
    }

    public ZoneCheck(Alliance alliance, ShooterType shooterType) {
        this.shooterType = shooterType;
        this.alliance = alliance;
        mapPollen = new int[ZoneCheckConfig.yQuality][ZoneCheckConfig.xQuality];
        mapNectar = new int[ZoneCheckConfig.yQuality][ZoneCheckConfig.xQuality];
    }



    public static boolean inZone(Pose pose) {
        int nectarOffsetx = (int)(-ZoneCheckConfig.yOffset*Math.sin(pose.heading())); //this just spins the vecter
        int nectarOffsety = (int)(ZoneCheckConfig.xOffset*Math.cos(pose.heading())); //this just spins the vecter
        double x = pose.x()+nectarOffsetx;
        double y = pose.y()+nectarOffsety;
        if(alliance == Alliance.BLUE){
            x = ZoneCheckConfig.xQuality-x;
            y = ZoneCheckConfig.yQuality-y;
        }
        int xQuad = (int)(x/(ZoneCheckConfig.FIELD_SIZE/ZoneCheckConfig.xQuality));
        int yQuad = (int)(y/(ZoneCheckConfig.FIELD_SIZE/ZoneCheckConfig.yQuality));
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
}
