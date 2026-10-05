package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gate {
    Servo gate;
    double openPos = 0;
    double closePos = 0;

    public gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "turretGate");
    }
    public void openGate(){
        gate.setPosition(openPos);
    }
    public void closeGate(){
        gate.setPosition(closePos);
    }
}
