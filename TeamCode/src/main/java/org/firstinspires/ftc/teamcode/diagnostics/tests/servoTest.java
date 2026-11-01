package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Positional servo bench test. Edit {@code position} under servoTest in Panels
 * (http://192.168.43.1:8001, Configurables). Telemetry echoes the commanded value and
 * servo.getPosition(). If SERVO_NAME is not in the ACTIVE hardware config the OpMode stays
 * open and lists the servos that are, instead of dying at init.
 */
@Configurable
@TeleOp(name = "servoTest", group = "Test")
public class servoTest extends LinearOpMode {
    public static String SERVO_NAME = "pollenTurretServo";
    public static double position = 0.0;

    @Override
    public void runOpMode() {
        Servo test = null;
        try {
            test = hardwareMap.get(Servo.class, SERVO_NAME);
        } catch (IllegalArgumentException e) {
            while (!isStopRequested()) {
                telemetry.addData("ERROR", "no Servo named \"" + SERVO_NAME + "\" in the active config");
                telemetry.addData("Servos in active config", hardwareMap.servo.size());
                for (String name : names()) {
                    telemetry.addLine("  " + name);
                }
                telemetry.addLine("Select the right config on the Driver Station, or change SERVO_NAME.");
                telemetry.update();
                sleep(200);
            }
            return;
        }
        waitForStart();
        while (opModeIsActive()) {
            test.setPosition(position);
            telemetry.addData("commanded position", position);
            telemetry.addData("servo getPosition()", test.getPosition());
            telemetry.update();
        }
        test.setPosition(0);
    }

    private java.util.List<String> names() {
        java.util.List<String> out = new java.util.ArrayList<>();
        for (Servo s : hardwareMap.servo) {
            out.addAll(hardwareMap.getNamesOf(s));
        }
        return out;
    }
}
