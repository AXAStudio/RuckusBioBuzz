package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

@Config
public class shooter {
    // Placeholders until the shooter is built and a shot can be timed.
    /** Once the gate opens it stays open at least this long, whatever the wheel speed does. */
    public static double MIN_GATE_OPEN_S = 0.3;
    /** Speed band that keeps SHOOT once entered (entry uses flywheel.AT_SPEED_FRACTION). */
    public static double HOLD_SPEED_FRACTION = 0.20;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final zoneCheck zone;
    private final turret aiming;
    private final flywheel flywheel;
    private final intake intake;
    private gate shooterGate;

    private boolean inZone;

    private enum states {
            RESET, TRACK, SHOOT
    }
    states state = states.TRACK;

    public shooter(HardwareMap hardwareMap, predictiveAiming predictor) {
        this.zone = new zoneCheck(predictor);
        aiming = new turret(hardwareMap, predictor);
        intake = new intake(hardwareMap);
        this.flywheel = new flywheel(hardwareMap, aiming);
        shooterGate = new gate(hardwareMap);

    }

    public void setAlliance(Alliance alliance) {
        aiming.setAlliance(alliance);
        intake.setAlliance(alliance);

    }
    public boolean shooting(){
        return state == states.SHOOT;
    }

    public void update() {
        inZone = zone.predictedInZone();
        flywheel.update(inZone);

        switch (state) {
            case RESET:
                shooterGate.closeGate();
                if (!aiming.unwinding()) state = states.TRACK;
                break;
            case TRACK:
            case SHOOT:
                aiming.update();
                boolean aimed = aiming.pollenAimed() && aiming.nectarAimed();
                if (aiming.unwinding()) {
                    state = states.RESET;
                } else if (state == states.SHOOT) {
                    // A ball hitting the wheel dips its speed past the 5% entry band. Re-checking
                    // the entry condition here closed the gate on that ball and handed the intake
                    // back to the driver mid-shot, so hold SHOOT for a minimum open time and then
                    // on a wider speed band. Leaving the zone still ends it at once.
                    boolean hold = shootTimer.seconds() < MIN_GATE_OPEN_S
                            || (aimed && flywheel.atSpeedPollen(HOLD_SPEED_FRACTION)
                                && flywheel.atSpeedNectar(HOLD_SPEED_FRACTION));
                    if (!inZone || !hold) state = states.TRACK;
                } else if (inZone && aimed && flywheel.atSpeedPollen() && flywheel.atSpeedNectar()) {
                    state = states.SHOOT;
                    shootTimer.reset();
                }
                if (state == states.SHOOT) {
                    shooterGate.openGate();

                } else {
                    shooterGate.closeGate();
                }
                break;
        }
    }

    public boolean ready() {
        return state == states.SHOOT;
    }

    public boolean inZone() { return zone.predictedInZone(); }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
}
