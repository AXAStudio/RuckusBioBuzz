package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    Servo gate;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "turretGate");
    }
    public void openGate(){
        gate.setPosition(SystemConstants.Gate.openPos);
    }
    public void closeGate(){
        gate.setPosition(SystemConstants.Gate.closePos);
    }
    public boolean positionsSet(){
        return SystemConstants.Gate.openPos != SystemConstants.Gate.closePos;
    }
}
