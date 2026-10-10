package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

public class shooter {
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
                boolean readyShoot = inZone && aiming.pollenAimed() && aiming.nectarAimed()
                        && flywheel.atSpeedPollen() && flywheel.atSpeedNectar();
                if (aiming.unwinding()) {
                    state = states.RESET;
                } else if (readyShoot) {
                    state = states.SHOOT;
                } else {
                    state = states.TRACK;
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

    public boolean inZone() { return inZone; }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
}
