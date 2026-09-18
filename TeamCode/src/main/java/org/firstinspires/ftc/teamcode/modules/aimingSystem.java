package org.firstinspires.ftc.teamcode.modules;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.helpers.KeyPosition;

@Config
public class aimingSystem{
    public static KeyPosition hivePos = new KeyPosition(58, 52);
    //return int array in the form of [target vel, target angle relative to front of bot]
    public int[] aim(int xPos, int yPos){
        return null;
    }
}
