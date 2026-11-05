package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class ZoneCheck {
    private static Alliance alliance;
    private static ShooterType shooterType;
    public enum ShooterType{
        POLLEN, NECTAR
    }

    public ZoneCheck(Alliance alliance, ShooterType shooterType) {
        ZoneCheck.shooterType = shooterType;
        ZoneCheck.alliance = alliance;
    }

    public void setAlliance(Alliance alliance) {
        ZoneCheck.alliance = alliance;
    }

    public static boolean inZone(Pose pose) {
        boolean pollen = shooterType == ShooterType.POLLEN;
        double offset = pollen ? SystemConstants.Zone.yOffsetPollen : SystemConstants.Zone.yOffsetNectar;
        double x = pose.x() - offset * Math.sin(pose.heading());
        double y = pose.y() + offset * Math.cos(pose.heading());
        if (alliance == Alliance.BLUE) {
            x = SystemConstants.Zone.FIELD_SIZE - x;
            y = SystemConstants.Zone.FIELD_SIZE - y;
        }
        int[][] map = pollen ? SystemConstants.Zone.mapPollen : SystemConstants.Zone.mapNectar;
        int xQuad = (int) Math.floor(x / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.xQuality));
        int yQuad = (int) Math.floor(y / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.yQuality));
        if (map == null || yQuad < 0 || yQuad >= map.length || map[yQuad] == null
                || xQuad < 0 || xQuad >= map[yQuad].length) {
            return false;
        }
        return map[yQuad][xQuad] == 1;
    }
}
