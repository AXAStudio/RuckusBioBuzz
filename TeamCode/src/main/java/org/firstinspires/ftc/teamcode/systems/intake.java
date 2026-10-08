package org.firstinspires.ftc.teamcode.systems;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class intake {
    private DcMotor intakeLeft, intakeRight;
    private ColorSensor colorSensor;
    HardwareMap hardwareMap;
    private intakeState currentState = intakeState.STOP;
    private Alliance alliance;
    public enum intakeState {
        INTAKE, STOP, REJECT
    }
    public static int thresholdRed = 100;
    public static int thresholdBlue = 100;

    public intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public void setState(intakeState state){
        this.currentState = state;
    }

    public void update(boolean leftBumper, boolean rightBumper) {
        int powerL = 0;
        int powerR = 0;
        if (currentState == intakeState.INTAKE) {
            powerL = 1;
            powerR = 1;
            if(!leftBumper && !rightBumper){
                currentState  = intakeState.STOP;
            }else if(leftBumper){
                currentState  = intakeState.REJECT;
            }
        } else if (currentState == intakeState.STOP) {
            powerL = 0;
            powerR = 0;
            if(leftBumper){
                currentState  = intakeState.REJECT;
            }else if(rightBumper){
                currentState  = intakeState.INTAKE;
            }
        }else if (currentState == intakeState.REJECT) {
            powerL = -1;
            powerR = -1;
            if(!leftBumper && !rightBumper){
                currentState  = intakeState.STOP;
            }else if(rightBumper){
                currentState  = intakeState.INTAKE;
            }
        }
        if(alliance == Alliance.BLUE && colorSensor.red() > thresholdRed){
            powerL = -1;
            powerR = -1;
        }else if(alliance == Alliance.RED && colorSensor.blue() > thresholdBlue){
            powerL = -1;
            powerR = -1;
        }

        intakeLeft.setPower(powerL);
        intakeRight.setPower(powerR);


    }

}
