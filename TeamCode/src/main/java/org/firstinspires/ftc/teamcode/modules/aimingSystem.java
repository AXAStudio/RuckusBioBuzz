package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.HivePosition;

@Config
public class aimingSystem{
    public static int x = 58;

    private final Follower follower;
    public static HivePosition hivePos = new HivePosition(x);
    public aimingSystem(Follower follower) {
        this.follower = follower;
    }


    //return int array in the form of [target vel, target angle relative to front of bot (rad)]
    public double[] aim(int xPos, int yPos){ //check for polarity
        double adj = hivePos.target(follower.pose()).x()-xPos;
        double op = hivePos.target(follower.pose()).y() -yPos;
        double dist = Math.sqrt(adj*adj+op*op);
        double theta = Math.atan2(op, adj);
        return new double[]{dist, theta};
    }
}
