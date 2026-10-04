package org.firstinspires.ftc.teamcode.systems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor intakeLeft, intakeRight;
    HardwareMap hardwareMap;
    private intakeState currentState = intakeState.STOP;
    public enum intakeState {
        INTAKE, STOP, REJECT
    }
    public intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setState(intakeState state){
        this.currentState = state;
    }

    public void update(boolean leftBumper, boolean rightBumper) {
        if (currentState == intakeState.INTAKE) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            if(!leftBumper && !rightBumper){
                currentState  = intakeState.STOP;
            }else if(leftBumper){
                currentState  = intakeState.REJECT;
            }
        } else if (currentState == intakeState.STOP) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            if(leftBumper){
                currentState  = intakeState.REJECT;
            }else if(rightBumper){
                currentState  = intakeState.INTAKE;
            }
        }else if (currentState == intakeState.REJECT) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
            if(!leftBumper && !rightBumper){
                currentState  = intakeState.STOP;
            }else if(rightBumper){
                currentState  = intakeState.INTAKE;
            }
        }


    }

}
