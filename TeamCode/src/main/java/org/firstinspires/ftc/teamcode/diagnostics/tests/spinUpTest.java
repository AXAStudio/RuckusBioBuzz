package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SpinUpTest", group = "TeleOp")
public class spinUpTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx outtake;
    private int maxSpeed = 0;
    private int recovery = 0;
    private boolean dip = false;

    @Override
    public void runOpMode() {

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();
        outtake.setPower(1);

        while(opModeIsActive()){
            if(outtake.getVelocity() > maxSpeed) {
                maxSpeed = (int) outtake.getVelocity();
            }
            if(outtake.getVelocity() > maxSpeed - 50){
                recovery = (int) runtime.milliseconds();
                if(dip){
                    runtime.reset();
                }
                dip = false;
            }else{
                dip = true;
            }
            telemetry.addData("Recovery time ms", recovery);
            telemetry.addData("Current Velocity", outtake.getVelocity());
            telemetry.addData("Max Velocity Achieved", maxSpeed);
            telemetry.addData("Time Elapsed (s)", runtime.seconds());
            telemetry.update();
        }
    }
}