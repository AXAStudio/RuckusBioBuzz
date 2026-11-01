package org.firstinspires.ftc.teamcode.diagnostics.tests;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Servo test = null;
        long lastLog = 0;
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
        // Drive during INIT too, so the bench works without pressing PLAY.
        while (!isStarted() && !isStopRequested()) {
            test.setPosition(position);
            lastLog = report(panels, test, lastLog);
            sleep(20);
        }
        while (opModeIsActive()) {
            test.setPosition(position);
            lastLog = report(panels, test, lastLog);
        }
        test.setPosition(0);
    }

    /** Echo to the Driver Station, Panels and logcat (tag servoTest, twice a second). */
    private long report(TelemetryManager panels, Servo test, long lastLog) {
        telemetry.addData("commanded position", position);
        telemetry.addData("servo getPosition()", test.getPosition());
        telemetry.update();
        panels.debug("commanded position", position);
        panels.debug("servo getPosition()", test.getPosition());
        panels.update(telemetry);
        long now = System.currentTimeMillis();
        if (now - lastLog > 500) {
            Log.i("servoTest", SERVO_NAME + " position=" + position + " getPosition=" + test.getPosition());
            return now;
        }
        return lastLog;
    }

    private java.util.List<String> names() {
        java.util.List<String> out = new java.util.ArrayList<>();
        for (Servo s : hardwareMap.servo) {
            out.addAll(hardwareMap.getNamesOf(s));
        }
        return out;
    }
}
