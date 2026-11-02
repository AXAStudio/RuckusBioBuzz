package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class ColorCheck{
    private final ColorSensor colorSensor;
    private final Alliance alliance;
    public static int thresholdRed = 100;
    public static int thresholdBlue = 100;
    public static double DOMINANCE = 1.5;
    public ColorCheck(ColorSensor colorSensor, Alliance alliance){
        this.colorSensor = colorSensor;
        this.alliance = alliance;
    }
    public boolean reject(){
        if(alliance == Alliance.BLUE && dominant(colorSensor.red(), thresholdRed, Channel.RED)){
            return true;
        }else if(alliance == Alliance.RED && dominant(colorSensor.blue(), thresholdBlue, Channel.BLUE)){
            return true;
        }else{
            return false;
        }
    }
    private enum Channel { RED, BLUE }

    private boolean dominant(int value, int threshold, Channel channel) {
        if (value <= threshold) return false;
        int green = colorSensor.green();
        int other = channel == Channel.RED ? colorSensor.blue() : colorSensor.red();
        return value > DOMINANCE * green && value > DOMINANCE * other;
    }
}
