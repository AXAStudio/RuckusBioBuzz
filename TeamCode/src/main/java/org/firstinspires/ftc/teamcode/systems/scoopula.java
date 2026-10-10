package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class scoopula {
    Servo scoopula;
    double scoopPos = 0; //set
    double unscoopPos = 0; //set
    public scoopula(HardwareMap hardwareMap) {
        scoopula = hardwareMap.get(Servo.class, "scoopula");
    }
    public void update(boolean input) {
        if(input) {
            scoopula.setPosition(scoopPos);
        }else{
            scoopula.setPosition(unscoopPos);
        }
    }
    /** False while scoop and unscoop are still the same placeholder value. */
    public boolean positionsSet() {
        return scoopPos != unscoopPos;
    }

}
