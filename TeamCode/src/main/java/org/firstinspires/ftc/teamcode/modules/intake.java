package org.firstinspires.ftc.teamcode.modules;


import static org.firstinspires.ftc.teamcode.modules.intake.intakeState.INTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class intake {
    private DcMotor intakeLeft, intakeRight;
    HardwareMap hardwareMap;
    private intakeState currentState = intakeState.STOP;
    public enum intakeState {
        INTAKE, STOP
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

    public void update() {
        if (currentState == intakeState.INTAKE) {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);
        } else if (currentState == intakeState.STOP) {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
        }

    }

}
