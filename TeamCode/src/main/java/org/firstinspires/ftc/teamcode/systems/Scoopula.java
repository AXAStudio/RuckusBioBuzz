package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoopula {
    Servo scoopula;
    double scoopPos = 0; //set
    double unscoopPos = 0; //set
    public Scoopula(HardwareMap hardwareMap) {
        scoopula = hardwareMap.get(Servo.class, "scoopula");
    }
    public void update(boolean input) {
        if(input) {
            scoopula.setPosition(scoopPos);
        }else{
            scoopula.setPosition(unscoopPos);
        }
    }
    public boolean positionsSet() {
        return scoopPos != unscoopPos;
    }

}
