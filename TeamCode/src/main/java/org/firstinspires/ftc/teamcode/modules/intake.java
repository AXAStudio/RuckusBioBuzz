package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class intake {
    private DcMotor intakeLeft, intakeRight;
    HardwareMap hardwareMap;
    public intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void runIntake(){
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }
    public void stopIntake(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }



}
