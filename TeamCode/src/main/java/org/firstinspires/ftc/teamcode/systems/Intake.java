package org.firstinspires.ftc.teamcode.systems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;

@Config
@Configurable
public class Intake {
    private DcMotor intakeLeft, intakeRight;
    private ColorSensor colorSensor;
    HardwareMap hardwareMap;
    private intakeState currentState = intakeState.STOP;
    private ColorCheck colorCheck;
    private Alliance alliance;
    public enum intakeState {
        INTAKE, STOP, REJECT
    }
    public static double REVERSE_BRAKE_MS = 60;
    private int lastSign = 0;
    private long lastDriveNanos = 0;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorCheck = new ColorCheck(colorSensor, alliance);

        //intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
        colorCheck.setAlliance(alliance); // ColorCheck is built before the alliance is known
    }

    public intakeState getState(){
        return currentState;
    }

    public void update(boolean leftBumper, boolean rightBumper) {
        if(leftBumper || colorCheck.reject()){
            currentState = intakeState.REJECT;
        }else if(rightBumper){
            currentState = intakeState.INTAKE;
        }else{
            currentState = intakeState.STOP;
        }
        int powerL = 0;
        int powerR = 0;
        if (currentState == intakeState.INTAKE) {
            powerL = 1;
            powerR = 1;
        } else if (currentState == intakeState.REJECT) {
            powerL = -1;
            powerR = -1;
        }

        long now = System.nanoTime();
        int sign = Integer.signum(powerL);
        if (sign != 0 && lastSign != 0 && sign != lastSign && now - lastDriveNanos < REVERSE_BRAKE_MS * 1e6) {
            powerL = 0;
            powerR = 0;
        } else if (sign != 0) {
            lastSign = sign;
            lastDriveNanos = now;
        }

        intakeLeft.setPower(powerL);
        intakeRight.setPower(powerR);

    }



}
