package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoopula {
    Servo scoopula;
    public Scoopula(HardwareMap hardwareMap) {
        scoopula = hardwareMap.get(Servo.class, "scoopula");
    }
    public void update(boolean input) {
        if(input) {
            scoopula.setPosition(SystemConstants.Scoopula.scoopPos);
        }else{
            scoopula.setPosition(SystemConstants.Scoopula.unscoopPos);
        }
    }
    public boolean positionsSet() {
        return SystemConstants.Scoopula.scoopPos != SystemConstants.Scoopula.unscoopPos;
    }

}
