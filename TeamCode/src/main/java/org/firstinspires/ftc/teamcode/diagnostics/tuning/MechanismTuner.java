package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.systems.flywheel;
import org.firstinspires.ftc.teamcode.systems.intake;
import org.firstinspires.ftc.teamcode.systems.turret;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Mechanism tuning OpMode behind {@code http://192.168.43.1:8080/tune}. DIAGNOSTIC ONLY - never
 * ships.
 *
 * <p>Covers everything that is not the drivetrain: both flywheels (hub velocity PIDF, step
 * response, shot / synthetic-disturbance recovery, feed-forward sweep), both turret servos
 * (center, direction, gear ratio), the gate and scoopula servo positions, and the intake color
 * sensor (labelled samples, threshold suggestion, live reject check through the shipped
 * {@link intake} class). The drivetrain is never touched: swerve tuning stays in Swerve Bring-Up.
 *
 * <p>Starting values are read from the shipped statics at init, and anything changed here is
 * exported as declarations for the shipped files; shipped statics the tool has to write so the
 * shipped intake logic sees them are restored in {@link #stop()}.
 *
 * <p>Safety: nothing moves before START, flywheel targets are capped at {@link #VEL_CAP_TPS}, and
 * every actuator stops if no web client has read state or sent a command for
 * {@link #CLIENT_TIMEOUT_MS}.
 */
@TeleOp(name = "Mechanism Tuner", group = "Diagnostics")
public class MechanismTuner extends OpMode {

    // ------------------------------------------------------------------------- limits
    /** Bare 6000 rpm motor with a 28 cpr encoder tops out near 2800 ticks/s. */
    static final double VEL_CAP_TPS = 3000;
    static final long CLIENT_TIMEOUT_MS = 30_000;
    static final long PUBLISH_INTERVAL_NS = 50_000_000L;
    static final long SLOW_INTERVAL_NS = 200_000_000L;
    static final long COLOR_INTERVAL_NS = 50_000_000L;

    // Dip detector: a wheel must sit in its at-speed band this long before a drop counts as a shot.
    static final long ARM_NS = 300_000_000L;
    static final long DIP_TIMEOUT_NS = 3_000_000_000L;
    static final long POST_WINDOW_NS = 300_000_000L;
    static final int MAX_EVENTS = 60;

    static final int MODE_OFF = 0, MODE_VEL = 1, MODE_POWER = 2;
    static final String[] MODE_NAMES = {"OFF", "VEL", "POWER"};

    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private double volts;

    private final Wheel[] wheels = new Wheel[2];
    private final TurretCal[] turrets = new TurretCal[2];
    private final ServoCal[] servos = new ServoCal[2];

    private intake intakeSys;
    private String intakeError = "";
    private String intakeMode = "OFF";
    private Alliance alliance = Alliance.RED;

    private ColorSensor color;
    private DistanceSensor colorDistance;
    private boolean colorStream = true;
    private int cr, cg, cb, ca;
    private double cdist = Double.NaN;
    private String sampleLabel = "";
    private int sampleRemaining;
    private final List<String> sampleLabels = new ArrayList<>();
    private final List<int[]> samples = new ArrayList<>();
    private final List<Double> sampleDist = new ArrayList<>();
    private String colorSummary = "{}";

    private double atSpeedFrac;
    private final FlyRecorder recorder = new FlyRecorder();
    private boolean recCurrent;
    private final List<String> events = new ArrayList<>();
    private int eventSeq;

    private boolean started;
    private String message = "Init. Nothing moves until START.";
    private long cmdSeq;
    private boolean watchdogTripped;

    private long lastLoopNs, lastPublishNs, lastSlowNs, lastColorNs, lastTelemetryNs;
    private final double[] dtRing = new double[128];
    private int dtCount;

    // Shipped statics the tool writes so the shipped intake class runs with them; restored on stop.
    private int origThrRed, origThrBlue;
    private double origDominance;

    // ------------------------------------------------------------------------- lifecycle

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        atSpeedFrac = flywheel.AT_SPEED_FRACTION;
        wheels[0] = new Wheel("pollen", "pollenTurret", flywheel.POLLEN_FLYWHEEL_REVERSED,
                flywheel.POLLEN_PIDF);
        wheels[1] = new Wheel("nectar", "nectarTurret", flywheel.NECTAR_FLYWHEEL_REVERSED,
                flywheel.NECTAR_PIDF);

        turrets[0] = new TurretCal("pollen", "pollenTurretServo", turret.POLLEN_TURRET_CENTER,
                turret.POLLEN_TURRET_REVERSED, turret.POLLEN_TURRET_GEAR_RATIO);
        turrets[1] = new TurretCal("nectar", "nectarTurretServo", turret.NECTAR_TURRET_CENTER,
                turret.NECTAR_TURRET_REVERSED, turret.NECTAR_TURRET_GEAR_RATIO);

        servos[0] = new ServoCal("gate", "turretGate", "systems/gate.java",
                new String[]{"open", "openPos", "close", "closePos"});
        servos[1] = new ServoCal("scoopula", "scoopula", "systems/scoopula.java",
                new String[]{"scoop", "scoopPos", "unscoop", "unscoopPos"});

        origThrRed = intake.thresholdRed;
        origThrBlue = intake.thresholdBlue;
        origDominance = intake.DOMINANCE;
        try {
            intakeSys = new intake(hardwareMap);
            intakeSys.setAlliance(alliance);
        } catch (RuntimeException e) {
            intakeSys = null;
            intakeError = shortError(e);
        }
        color = hardwareMap.tryGet(ColorSensor.class, "colorSensor");
        colorDistance = hardwareMap.tryGet(DistanceSensor.class, "colorSensor");

        TuningHub.INSTANCE.clearCommands();
        TuningHub.INSTANCE.setRecorder(recorder);
        lastLoopNs = System.nanoTime();
        rebuildColorSummary();
    }

    @Override
    public void init_loop() {
        service();
    }

    @Override
    public void start() {
        started = true;
        message = "Started. Mechanisms can move now.";
    }

    @Override
    public void loop() {
        service();
    }

    @Override
    public void stop() {
        stopAll();
        recorder.stop();
        intake.thresholdRed = origThrRed;
        intake.thresholdBlue = origThrBlue;
        intake.DOMINANCE = origDominance;
        TuningHub.INSTANCE.markStopped();
    }

    // ------------------------------------------------------------------------- loop

    private void service() {
        long now = System.nanoTime();
        double dt = (now - lastLoopNs) / 1e9;
        lastLoopNs = now;
        dtRing[dtCount++ % dtRing.length] = dt;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        boolean slow = now - lastSlowNs >= SLOW_INTERVAL_NS;
        if (slow) {
            lastSlowNs = now;
            readVolts();
        }
        for (Wheel w : wheels) {
            w.read(slow || recCurrent);
        }

        Map<String, String> cmd;
        while ((cmd = TuningHub.INSTANCE.poll()) != null) {
            handle(cmd);
        }

        boolean anythingMoving = intakeSys != null && !"OFF".equals(intakeMode);
        for (Wheel w : wheels) {
            w.update(now);
            anythingMoving |= w.mode != MODE_OFF;
        }
        if (anythingMoving && TuningHub.INSTANCE.msSinceClient() > CLIENT_TIMEOUT_MS) {
            stopAll();
            watchdogTripped = true;
            message = "WATCHDOG: no web client for " + (CLIENT_TIMEOUT_MS / 1000)
                    + " s - everything stopped.";
        }

        if (intakeSys != null && !"OFF".equals(intakeMode)) {
            intakeSys.update("OUT".equals(intakeMode), "IN".equals(intakeMode));
        }

        if (color != null && (colorStream || sampleRemaining > 0) && !recorder.isRecording()
                && now - lastColorNs >= COLOR_INTERVAL_NS) {
            lastColorNs = now;
            readColor();
        }

        recorder.add(dt, wheels[0].target, wheels[0].vel, wheels[1].target, wheels[1].vel,
                recCurrent ? wheels[0].cur : Double.NaN, recCurrent ? wheels[1].cur : Double.NaN, volts, wheels[0].mode, wheels[1].mode);

        if (now - lastPublishNs >= PUBLISH_INTERVAL_NS) {
            lastPublishNs = now;
            TuningHub.INSTANCE.publish(stateJson());
        }
        if (now - lastTelemetryNs >= SLOW_INTERVAL_NS) {
            lastTelemetryNs = now;
            telemetryUpdate();
        }
    }

    private void readVolts() {
        if (voltageSensor == null) {
            return;
        }
        try {
            volts = voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            volts = 0;
        }
    }

    private void readColor() {
        try {
            cr = color.red();
            cg = color.green();
            cb = color.blue();
            ca = color.alpha();
            cdist = colorDistance == null ? Double.NaN : colorDistance.getDistance(DistanceUnit.CM);
        } catch (RuntimeException e) {
            return;
        }
        if (sampleRemaining > 0) {
            sampleLabels.add(sampleLabel);
            samples.add(new int[]{cr, cg, cb, ca});
            sampleDist.add(cdist);
            if (--sampleRemaining == 0) {
                rebuildColorSummary();
                message = "Color: captured samples for " + sampleLabel + ".";
            }
        }
    }

    private void telemetryUpdate() {
        String open = TuningHub.INSTANCE.openNoteText();
        if (!open.isEmpty()) {
            telemetry.addLine(">>> " + open);
        }
        telemetry.addLine("Dashboard: http://192.168.43.1:8080/tune");
        telemetry.addData("State", started ? "RUNNING" : "INIT (press START to move)");
        telemetry.addData("Battery", FlyRecorder.r(volts, 2) + " V");
        for (Wheel w : wheels) {
            telemetry.addData(w.name, MODE_NAMES[w.mode] + "  tgt " + FlyRecorder.r(w.target, 0)
                    + "  vel " + FlyRecorder.r(w.vel, 0));
        }
        telemetry.addData("Message", message);
        telemetry.update();
    }

    // ------------------------------------------------------------------------- commands

    private static boolean isMotion(String action) {
        switch (action) {
            case "fly": case "flyPower": case "step": case "kick": case "ffSweep":
            case "turretPos": case "turretDeg": case "servoPos": case "gatePulse": case "intake":
                return true;
            default:
                return false;
        }
    }

    private void handle(Map<String, String> cmd) {
        String action = cmd.get("action") == null ? "" : cmd.get("action");
        cmdSeq = (long) num(cmd, "_seq", cmdSeq);
        if (isMotion(action) && !started) {
            message = "REFUSED " + action + ": press START on the Driver Station first.";
            return;
        }
        if (isMotion(action)) {
            watchdogTripped = false;
        }
        try {
            dispatch(action, cmd);
        } catch (RuntimeException e) {
            message = "ERROR in " + action + ": " + shortError(e);
        }
    }

    private void dispatch(String action, Map<String, String> c) {
        switch (action) {
            case "stopAll":
                stopAll();
                message = "Everything stopped.";
                break;

            // ---- flywheels
            case "fly":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.setVelocity(num(c, "vel", 0));
                }
                message = "Flywheel velocity " + FlyRecorder.r(num(c, "vel", 0), 0) + " ticks/s.";
                break;
            case "flyPower":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.setPower(num(c, "p", 0));
                }
                message = "Flywheel open-loop power " + num(c, "p", 0) + ".";
                break;
            case "flyOff":
                for (Wheel w : wheelsFor(c)) {
                    w.cancelTests();
                    w.off();
                }
                message = "Flywheel off (coasting).";
                break;
            case "flyDir":
                for (Wheel w : wheelsFor(c)) {
                    w.reversed = num(c, "reversed", 0) != 0;
                    w.applyDirection();
                }
                message = "Flywheel direction updated.";
                break;
            case "pidf":
                for (Wheel w : wheelsFor(c)) {
                    w.kP = num(c, "p", w.kP);
                    w.kI = num(c, "i", w.kI);
                    w.kD = num(c, "d", w.kD);
                    w.kF = num(c, "f", w.kF);
                    w.applyPidf();
                }
                message = "Hub velocity PIDF written.";
                break;
            case "atSpeed":
                atSpeedFrac = clamp(num(c, "frac", atSpeedFrac), 0.005, 0.5);
                message = "At-speed band now " + FlyRecorder.r(atSpeedFrac * 100, 1) + "%.";
                break;
            case "step": {
                Wheel w = oneWheel(c);
                w.startStep(num(c, "to", 0), c.containsKey("from") ? num(c, "from", 0) : Double.NaN,
                        num(c, "pre", 1.5), num(c, "sec", 3.0), c.get("label"));
                message = "Step started on " + w.name + ".";
                break;
            }
            case "kick": {
                for (Wheel w : wheelsFor(c)) {
                    w.kick(num(c, "ms", 120));
                }
                message = "Synthetic disturbance: motor output cut for " + num(c, "ms", 120) + " ms.";
                break;
            }
            case "ffSweep": {
                Wheel w = oneWheel(c);
                w.startFf(parseList(c.get("powers"), new double[]{0.3, 0.45, 0.6, 0.75, 0.9, 1.0}),
                        num(c, "hold", 2.0));
                message = "Feed-forward sweep started on " + w.name + ".";
                break;
            }
            case "clearEvents":
                events.clear();
                message = "Shot events cleared.";
                break;
            case "recStart":
                recCurrent = num(c, "cur", 0) != 0;
                recorder.start(c.get("label"));
                message = "Recording " + recorder.label() + (recCurrent ? " with current." : ".");
                break;
            case "recStop":
                recorder.stop();
                recCurrent = false;
                message = "Recording stopped, " + recorder.size() + " samples.";
                break;

            // ---- turrets
            case "turretPos":
                for (TurretCal t : turretsFor(c)) {
                    t.setPosition(num(c, "pos", t.pos));
                }
                message = "Turret position set.";
                break;
            case "turretDeg":
                for (TurretCal t : turretsFor(c)) {
                    t.setPosition(t.positionFor(num(c, "deg", 0)));
                }
                message = "Turret pointed at " + num(c, "deg", 0) + " deg (shipped formula).";
                break;
            case "turretCal":
                for (TurretCal t : turretsFor(c)) {
                    if ("1".equals(c.get("here"))) {
                        t.center = t.pos;
                    }
                    t.center = clamp(num(c, "center", t.center), 0, 1);
                    t.reversed = num(c, "reversed", t.reversed ? 1 : 0) != 0;
                    t.gearA = num(c, "grA", t.gearA);
                    t.gearB = num(c, "grB", t.gearB);
                    t.servoRange = num(c, "servoRange", t.servoRange);
                }
                message = "Turret calibration updated (tool only until exported).";
                break;

            // ---- gate / scoopula
            case "servoPos": {
                ServoCal s = servoFor(c);
                s.setPosition(num(c, "pos", s.pos));
                message = s.name + " position " + FlyRecorder.r(s.pos, 3) + ".";
                break;
            }
            case "servoMark": {
                ServoCal s = servoFor(c);
                String as = c.get("as");
                if (!s.marks.containsKey(as)) {
                    message = "servoMark: as= must be one of " + s.marks.keySet();
                    break;
                }
                s.marks.put(as, Double.isNaN(s.pos) ? null : s.pos);
                message = s.name + " '" + as + "' marked at " + FlyRecorder.r(s.pos, 3) + ".";
                break;
            }
            case "gatePulse": {
                ServoCal s = servos[0];
                Double open = s.marks.get("open");
                Double close = s.marks.get("close");
                if (open == null || close == null) {
                    message = "gatePulse needs gate 'open' and 'close' marked first.";
                    break;
                }
                s.pulse(open, close, num(c, "ms", 250));
                message = "Gate pulsed open for " + num(c, "ms", 250) + " ms.";
                break;
            }

            // ---- intake / color
            case "intake": {
                String m = c.get("mode") == null ? "OFF" : c.get("mode").toUpperCase();
                if (intakeSys == null) {
                    message = "Intake unavailable: " + intakeError;
                    break;
                }
                if (!Arrays.asList("OFF", "IN", "OUT", "WATCH").contains(m)) {
                    message = "intake mode= must be OFF, IN, OUT or WATCH.";
                    break;
                }
                intakeMode = m;
                if ("OFF".equals(m)) {
                    stopIntake();
                }
                message = "Intake " + m + " (shipped intake logic, alliance " + alliance + ").";
                break;
            }
            case "alliance":
                alliance = "BLUE".equalsIgnoreCase(c.get("side")) ? Alliance.BLUE : Alliance.RED;
                if (intakeSys != null) {
                    intakeSys.setAlliance(alliance);
                }
                message = "Alliance " + alliance + ".";
                break;
            case "colorStream":
                colorStream = num(c, "on", 1) != 0;
                message = "Color streaming " + (colorStream ? "on." : "off.");
                break;
            case "colorSample": {
                String label = c.get("label") == null ? "" : c.get("label").trim().toUpperCase();
                if (label.isEmpty() || color == null) {
                    message = color == null ? "No color sensor named colorSensor."
                            : "colorSample needs label=";
                    break;
                }
                sampleLabel = label;
                sampleRemaining = (int) clamp(num(c, "n", 40), 1, 500);
                message = "Sampling " + sampleRemaining + " reads as " + label + "...";
                break;
            }
            case "colorClear": {
                String label = c.get("label");
                for (int i = samples.size() - 1; i >= 0; i--) {
                    if (label == null || label.equalsIgnoreCase(sampleLabels.get(i))) {
                        samples.remove(i);
                        sampleLabels.remove(i);
                        sampleDist.remove(i);
                    }
                }
                rebuildColorSummary();
                message = "Color samples cleared" + (label == null ? "." : " for " + label + ".");
                break;
            }
            case "colorParams":
                intake.thresholdRed = (int) num(c, "red", intake.thresholdRed);
                intake.thresholdBlue = (int) num(c, "blue", intake.thresholdBlue);
                intake.DOMINANCE = num(c, "dom", intake.DOMINANCE);
                rebuildColorSummary();
                message = "Intake color thresholds applied live (restored on STOP unless exported).";
                break;
            case "colorSuggestApply":
                rebuildColorSummary();
                if (suggestion != null) {
                    intake.thresholdRed = suggestion[0];
                    intake.thresholdBlue = suggestion[1];
                    intake.DOMINANCE = suggestionDom;
                    rebuildColorSummary();
                    message = "Suggested thresholds applied live.";
                } else {
                    message = "No suggestion yet: sample RED, BLUE and at least one other label.";
                }
                break;

            default:
                message = "UNKNOWN COMMAND: \"" + action + "\" - nothing was done.";
                break;
        }
    }

    private void stopAll() {
        for (Wheel w : wheels) {
            w.cancelTests();
            w.off();
        }
        if (intakeSys != null) {
            stopIntake();
        }
        intakeMode = "OFF";
    }

    /** The shipped intake has no stop; with a null alliance its reject branch cannot fire. */
    private void stopIntake() {
        intakeSys.setAlliance(null);
        intakeSys.update(false, false);
        intakeSys.setAlliance(alliance);
    }

    private Wheel[] wheelsFor(Map<String, String> c) {
        String w = c.get("wheel") == null ? "both" : c.get("wheel");
        if ("pollen".equals(w)) return new Wheel[]{wheels[0]};
        if ("nectar".equals(w)) return new Wheel[]{wheels[1]};
        if ("both".equals(w)) return wheels;
        throw new IllegalArgumentException("wheel= must be pollen, nectar or both");
    }

    private Wheel oneWheel(Map<String, String> c) {
        String w = c.get("wheel");
        if ("pollen".equals(w)) return wheels[0];
        if ("nectar".equals(w)) return wheels[1];
        throw new IllegalArgumentException("this test needs wheel=pollen or wheel=nectar");
    }

    private TurretCal[] turretsFor(Map<String, String> c) {
        String w = c.get("which") == null ? "both" : c.get("which");
        if ("pollen".equals(w)) return new TurretCal[]{turrets[0]};
        if ("nectar".equals(w)) return new TurretCal[]{turrets[1]};
        if ("both".equals(w)) return turrets;
        throw new IllegalArgumentException("which= must be pollen, nectar or both");
    }

    private ServoCal servoFor(Map<String, String> c) {
        String n = c.get("name");
        for (ServoCal s : servos) {
            if (s.name.equals(n)) return s;
        }
        throw new IllegalArgumentException("name= must be gate or scoopula");
    }

    // ------------------------------------------------------------------------- flywheel

    private final class Wheel {
        final String name;
        final String device;
        DcMotorEx motor;
        String error = "";
        boolean reversed;
        int mode = MODE_OFF;
        DcMotor.RunMode runMode;
        double target, power, vel;
        double cur = Double.NaN;
        double kP, kI, kD, kF;
        String hubPidf = "";

        // dip detector: 0 arming, 1 armed, 2 in dip, 3 post-recovery window
        int dip;
        long inBandSinceNs, dipStartNs, dipMinNs, recoveredNs, kickEventUntilNs;
        double dipMin, overMax, dipTarget, dipVolts;
        String dipSource = "shot";

        long kickEndNs;
        boolean kicking;

        // step test: 0 none, 1 pre (holding from), 2 running
        int stepPhase;
        long phaseEndNs;
        double stepTo, stepSec;
        int stepStartIdx;
        boolean stepOwnsRec;
        String stepLabel;
        String lastStep = "null";

        // feed-forward sweep
        double[] ffPowers;
        int ffIdx = -1;
        double ffHold, ffSum;
        int ffCount;
        long ffLevelStartNs;
        final List<double[]> ffLevels = new ArrayList<>();
        String ffResult = "null";

        Wheel(String name, String device, boolean reversed,
              com.pedropathing.control.PIDFCoefficients shipped) {
            this.name = name;
            this.device = device;
            this.reversed = reversed;
            kP = shipped.P;
            kI = shipped.I;
            kD = shipped.D;
            kF = shipped.F;
            motor = hardwareMap.tryGet(DcMotorEx.class, device);
            if (motor == null) {
                error = "no motor named " + device;
                return;
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            applyDirection();
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            applyPidf();
        }

        void applyDirection() {
            if (motor != null) {
                motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);
            }
        }

        void applyPidf() {
            if (motor == null) return;
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP, kI, kD, kF));
            PIDFCoefficients back = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            hubPidf = FlyRecorder.r(back.p, 4) + ", " + FlyRecorder.r(back.i, 4) + ", "
                    + FlyRecorder.r(back.d, 4) + ", " + FlyRecorder.r(back.f, 4);
        }

        void setRunMode(DcMotor.RunMode m) {
            if (motor != null && runMode != m) {
                motor.setMode(m);
                runMode = m;
            }
        }

        void read(boolean withCurrent) {
            if (motor == null) return;
            vel = motor.getVelocity();
            if (withCurrent) {
                cur = motor.getCurrent(CurrentUnit.AMPS);
            }
        }

        void setVelocity(double v) {
            if (motor == null) return;
            v = clamp(v, -VEL_CAP_TPS, VEL_CAP_TPS);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (mode != MODE_VEL || v != target) {
                motor.setVelocity(v);
                dip = 0;
                inBandSinceNs = 0;
            }
            mode = MODE_VEL;
            target = v;
            power = Double.NaN;
        }

        void setPower(double p) {
            if (motor == null) return;
            p = clamp(p, -1, 1);
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(p);
            mode = MODE_POWER;
            power = p;
            target = 0;
            dip = 0;
        }

        void off() {
            if (motor == null) return;
            if (kicking) {
                motor.setMotorEnable();
                kicking = false;
            }
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            mode = MODE_OFF;
            target = 0;
            power = 0;
            dip = 0;
        }

        void cancelTests() {
            if (stepPhase != 0 && stepOwnsRec) {
                recorder.stop();
            }
            stepPhase = 0;
            ffIdx = -1;
        }

        /** Cuts the motor output so the wheel coasts, then hands it back to the hub's PIDF. */
        void kick(double ms) {
            if (motor == null || mode != MODE_VEL) {
                throw new IllegalStateException(name + " must be in VEL mode to kick");
            }
            motor.setMotorDisable();
            kicking = true;
            long now = System.nanoTime();
            kickEndNs = now + (long) (clamp(ms, 10, 1000) * 1e6);
            kickEventUntilNs = kickEndNs + 200_000_000L;
            recorder.mark(2);
        }

        void startStep(double to, double from, double pre, double sec, String label) {
            if (motor == null) throw new IllegalStateException(error);
            cancelTests();
            stepTo = clamp(to, -VEL_CAP_TPS, VEL_CAP_TPS);
            stepSec = clamp(sec, 0.5, 20);
            stepLabel = label == null || label.isEmpty()
                    ? "step-" + name + "-" + (int) stepTo : label;
            lastStep = "null";
            if (!Double.isNaN(from)) {
                setVelocity(from);
                stepPhase = 1;
                phaseEndNs = System.nanoTime() + (long) (clamp(pre, 0, 10) * 1e9);
            } else {
                beginStepRun();
            }
        }

        void beginStepRun() {
            stepOwnsRec = !recorder.isRecording();
            if (stepOwnsRec) {
                recorder.start(stepLabel);
            }
            stepStartIdx = recorder.size();
            recorder.mark(1);
            setVelocity(stepTo);
            stepPhase = 2;
            phaseEndNs = System.nanoTime() + (long) (stepSec * 1e9);
        }

        void startFf(double[] powers, double hold) {
            if (motor == null) throw new IllegalStateException(error);
            cancelTests();
            ffPowers = powers;
            ffHold = clamp(hold, 0.5, 10);
            ffLevels.clear();
            ffResult = "null";
            ffIdx = 0;
            beginFfLevel();
        }

        void beginFfLevel() {
            setPower(ffPowers[ffIdx]);
            ffLevelStartNs = System.nanoTime();
            ffSum = 0;
            ffCount = 0;
            recorder.mark(4);
        }

        void update(long now) {
            if (motor == null) return;

            if (kicking && now >= kickEndNs) {
                motor.setMotorEnable();
                // Re-issue the target: whether the hub keeps it across a disable is not documented.
                motor.setVelocity(target);
                kicking = false;
            }

            if (stepPhase == 1 && now >= phaseEndNs) {
                beginStepRun();
            } else if (stepPhase == 2 && (now >= phaseEndNs || recorder.isFull()
                    || !recorder.isRecording())) {
                int end = recorder.size();
                if (stepOwnsRec) {
                    recorder.stop();
                }
                stepPhase = 0;
                lastStep = analyseStep(this, stepStartIdx, end, stepTo);
            }

            if (ffIdx >= 0) {
                double t = (now - ffLevelStartNs) / 1e9;
                if (t >= ffHold - Math.min(0.5, ffHold / 2)) {
                    ffSum += vel;
                    ffCount++;
                }
                if (t >= ffHold) {
                    ffLevels.add(new double[]{ffPowers[ffIdx], ffCount == 0 ? 0 : ffSum / ffCount,
                            volts});
                    ffIdx++;
                    if (ffIdx >= ffPowers.length) {
                        ffIdx = -1;
                        off();
                        ffResult = ffJson();
                        message = "Feed-forward sweep finished on " + name + ".";
                    } else {
                        beginFfLevel();
                    }
                }
            }

            updateDip(now);
        }

        private void updateDip(long now) {
            if (mode != MODE_VEL || target == 0 || stepPhase != 0) {
                dip = 0;
                inBandSinceNs = 0;
                return;
            }
            double band = atSpeedFrac * Math.abs(target);
            double e = (vel - target) * Math.signum(target);  // negative = slower than target
            switch (dip) {
                case 0:
                    if (Math.abs(e) <= band) {
                        if (inBandSinceNs == 0) inBandSinceNs = now;
                        if (now - inBandSinceNs >= ARM_NS) dip = 1;
                    } else {
                        inBandSinceNs = 0;
                    }
                    break;
                case 1:
                    if (e < -band) {
                        dip = 2;
                        dipStartNs = now;
                        dipMin = e;
                        dipMinNs = now;
                        dipTarget = target;
                        dipVolts = volts;
                        dipSource = kicking || now < kickEventUntilNs ? "kick" : "shot";
                        recorder.mark(3);
                    }
                    break;
                case 2:
                    if (e < dipMin) {
                        dipMin = e;
                        dipMinNs = now;
                    }
                    if (Math.abs(e) <= band) {
                        dip = 3;
                        recoveredNs = now;
                        overMax = 0;
                    } else if (now - dipStartNs > DIP_TIMEOUT_NS) {
                        finishDip(false);
                    }
                    break;
                case 3:
                    overMax = Math.max(overMax, e);
                    if (now - recoveredNs >= POST_WINDOW_NS) {
                        finishDip(true);
                    }
                    break;
                default:
                    dip = 0;
            }
        }

        private void finishDip(boolean recovered) {
            dip = 0;
            inBandSinceNs = 0;
            double t = Math.abs(dipTarget);
            double recoverMs = recovered ? (recoveredNs - dipStartNs) / 1e6 : -1;
            double dipPct = 100 * -dipMin / t;
            String summary = name + " " + dipSource + ": dip " + FlyRecorder.r(dipPct, 1)
                    + "%, " + (recovered ? "back at speed in " + (int) recoverMs + " ms"
                    : "NOT back at speed within 3 s");
            StringBuilder sb = new StringBuilder(200);
            sb.append("{\"id\":").append(++eventSeq)
                    .append(",\"wheel\":\"").append(name)
                    .append("\",\"source\":\"").append(dipSource)
                    .append("\",\"target\":").append(FlyRecorder.r(dipTarget, 0))
                    .append(",\"dipTps\":").append(FlyRecorder.r(-dipMin, 1))
                    .append(",\"dipPct\":").append(FlyRecorder.r(dipPct, 2))
                    .append(",\"tMinMs\":").append(FlyRecorder.r((dipMinNs - dipStartNs) / 1e6, 1))
                    .append(",\"recoverMs\":").append(FlyRecorder.r(recoverMs, 1))
                    .append(",\"overPct\":").append(recovered
                            ? FlyRecorder.r(100 * overMax / t, 2) : "null")
                    .append(",\"band\":").append(FlyRecorder.r(atSpeedFrac, 4))
                    .append(",\"pidf\":[").append(kP).append(',').append(kI).append(',')
                    .append(kD).append(',').append(kF).append(']')
                    .append(",\"volts\":").append(FlyRecorder.r(dipVolts, 2))
                    .append('}');
            events.add(sb.toString());
            while (events.size() > MAX_EVENTS) {
                events.remove(0);
            }
            if ("shot".equals(dipSource)) {
                TuningHub.INSTANCE.resolveAwait("shot:" + name, summary);
            }
            message = summary + ".";
        }

        private String ffJson() {
            // Least-squares slope through the origin: velocity per unit power.
            double spv = 0, spp = 0;
            StringBuilder lv = new StringBuilder();
            for (double[] l : ffLevels) {
                spv += l[0] * l[1];
                spp += l[0] * l[0];
                if (lv.length() > 0) lv.append(',');
                lv.append('[').append(l[0]).append(',').append(FlyRecorder.r(l[1], 1)).append(',')
                        .append(FlyRecorder.r(l[2], 2)).append(']');
            }
            double maxFit = spp == 0 ? 0 : spv / spp;
            double atFull = Double.NaN;
            for (double[] l : ffLevels) {
                if (l[0] >= 0.999) atFull = l[1];
            }
            // FIRST's hub velocity PIDF guide: F = 32767 / max ticks per second.
            return "{\"levels\":[" + lv + "],\"maxVelFit\":" + FlyRecorder.r(maxFit, 1)
                    + ",\"fFromFit\":" + (maxFit > 0 ? FlyRecorder.r(32767 / maxFit, 3) : "null")
                    + ",\"maxVelAtFull\":" + (Double.isNaN(atFull) ? "null" : FlyRecorder.r(atFull, 1))
                    + ",\"fFromFull\":" + (Double.isNaN(atFull) || atFull <= 0 ? "null"
                    : FlyRecorder.r(32767 / atFull, 3)) + "}";
        }
    }

    /**
     * Step metrics straight from the recorder, so the dashboard, the host scorer and the CSV all
     * describe the same samples. Band is the shipped at-speed band (AT_SPEED_FRACTION x target):
     * that is what decides when the shooter opens the gate.
     */
    private String analyseStep(Wheel w, int from, int to, double tgt) {
        double[] vel = w == wheels[0] ? recorder.pVel : recorder.nVel;
        int n = to - from;
        if (n < 5) {
            return "{\"error\":\"too few samples (" + n + ")\"}";
        }
        double t0 = recorder.t[from];
        double v0 = vel[from];
        double delta = tgt - v0;
        double band = tgt == 0 ? 20 : atSpeedFrac * Math.abs(tgt);
        double sgn = Math.signum(delta);
        double t10 = Double.NaN, t90 = Double.NaN, tBand = Double.NaN, peak = 0;
        int lastOut = -1;
        for (int i = from; i < to; i++) {
            double frac = delta == 0 ? 1 : (vel[i] - v0) / delta;
            double t = recorder.t[i] - t0;
            if (Double.isNaN(t10) && frac >= 0.1) t10 = t;
            if (Double.isNaN(t90) && frac >= 0.9) t90 = t;
            boolean in = Math.abs(vel[i] - tgt) <= band;
            if (in && Double.isNaN(tBand)) tBand = t;
            if (!in) lastOut = i;
            peak = Math.max(peak, (vel[i] - tgt) * sgn);
        }
        double settle = lastOut == to - 1 ? Double.NaN
                : lastOut < 0 ? 0 : recorder.t[lastOut + 1] - t0;
        double tEnd = recorder.t[to - 1];
        double ssStart = tEnd - Math.min(1.0, (tEnd - t0) * 0.3);
        double sum = 0, sq = 0, mn = Double.MAX_VALUE, mx = -Double.MAX_VALUE;
        int ns = 0;
        double dtSum = 0, vSum = 0;
        double[] dts = new double[n];
        for (int i = from; i < to; i++) {
            dts[i - from] = recorder.dt[i];
            dtSum += recorder.dt[i];
            vSum += recorder.volts[i];
            if (recorder.t[i] >= ssStart) {
                double e = vel[i] - tgt;
                sum += e;
                sq += e * e;
                mn = Math.min(mn, e);
                mx = Math.max(mx, e);
                ns++;
            }
        }
        Arrays.sort(dts);
        double ssMean = ns == 0 ? Double.NaN : sum / ns;
        double ssStd = ns < 2 ? Double.NaN : Math.sqrt(Math.max(0, sq / ns - ssMean * ssMean));
        double span = recorder.t[to - 1] - recorder.t[from];
        StringBuilder sb = new StringBuilder(400);
        sb.append("{\"label\":\"").append(TuningHub.esc(w.stepLabel))
                .append("\",\"wheel\":\"").append(w.name)
                .append("\",\"from\":").append(FlyRecorder.r(v0, 1))
                .append(",\"to\":").append(FlyRecorder.r(tgt, 1))
                .append(",\"band\":").append(FlyRecorder.r(band, 1))
                .append(",\"rise10_90S\":").append(nanToNull(t90 - t10, 3))
                .append(",\"firstInBandS\":").append(nanToNull(tBand, 3))
                .append(",\"settleS\":").append(nanToNull(settle, 3))
                .append(",\"overshootPct\":").append(delta == 0 ? "null"
                        : FlyRecorder.r(100 * peak / Math.abs(delta), 2))
                .append(",\"ssErrMean\":").append(nanToNull(ssMean, 1))
                .append(",\"ssErrStd\":").append(nanToNull(ssStd, 1))
                .append(",\"ssErrP2p\":").append(ns == 0 ? "null" : FlyRecorder.r(mx - mn, 1))
                .append(",\"n\":").append(n)
                .append(",\"loopHzTrue\":").append(nanToNull((n - 1) / span, 1))
                .append(",\"dtMeanMs\":").append(FlyRecorder.r(1000 * dtSum / n, 2))
                .append(",\"dtP90Ms\":").append(FlyRecorder.r(1000 * dts[(int) (0.9 * (n - 1))], 2))
                .append(",\"volts\":").append(FlyRecorder.r(vSum / n, 2))
                .append(",\"pidf\":[").append(w.kP).append(',').append(w.kI).append(',')
                .append(w.kD).append(',').append(w.kF).append("]}");
        return sb.toString();
    }

    // ------------------------------------------------------------------------- turret / servos

    private final class TurretCal {
        final String name;
        final String device;
        Servo servo;
        double pos = Double.NaN;
        double center;
        boolean reversed;
        double gearA, gearB;
        double servoRange = turret.SERVO_RANGE_DEG;

        TurretCal(String name, String device, double center, boolean reversed, double[] gear) {
            this.name = name;
            this.device = device;
            this.center = center;
            this.reversed = reversed;
            this.gearA = gear[0];
            this.gearB = gear[1];
            servo = hardwareMap.tryGet(Servo.class, device);
        }

        /** Turret degrees across the full 0-1 servo travel, exactly as turret.TurretServo.track. */
        double range() {
            return servoRange * gearB / gearA;
        }

        double positionFor(double deg) {
            return center + (reversed ? -1 : 1) * deg / range();
        }

        double degFor(double p) {
            return (p - center) * range() * (reversed ? -1 : 1);
        }

        void setPosition(double p) {
            if (servo == null) throw new IllegalStateException("no servo named " + device);
            pos = clamp(p, 0, 1);
            servo.setPosition(pos);
        }
    }

    private final class ServoCal {
        final String name;
        final String device;
        final String file;
        final Map<String, Double> marks = new LinkedHashMap<>();
        final Map<String, String> fields = new LinkedHashMap<>();
        Servo servo;
        double pos = Double.NaN;

        ServoCal(String name, String device, String file, String[] markToField) {
            this.name = name;
            this.device = device;
            this.file = file;
            for (int i = 0; i < markToField.length; i += 2) {
                marks.put(markToField[i], null);
                fields.put(markToField[i], markToField[i + 1]);
            }
            servo = hardwareMap.tryGet(Servo.class, device);
        }

        void setPosition(double p) {
            if (servo == null) throw new IllegalStateException("no servo named " + device);
            pos = clamp(p, 0, 1);
            servo.setPosition(pos);
        }

        /** Opens, then closes on a background thread so the loop never sleeps. */
        void pulse(final double open, final double close, final double ms) {
            setPosition(open);
            final Servo s = servo;
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        Thread.sleep((long) clamp(ms, 20, 3000));
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    s.setPosition(close);
                }
            }, "MechanismTuner-gatePulse");
            t.setDaemon(true);
            t.start();
            pos = close;
        }
    }

    // ------------------------------------------------------------------------- color

    private int[] suggestion;
    private double suggestionDom;

    /** Would the shipped intake call this channel dominant? Mirrors intake.dominant(). */
    private static boolean dominant(int[] s, boolean red, int thrRed, int thrBlue, double dom) {
        int value = red ? s[0] : s[2];
        int threshold = red ? thrRed : thrBlue;
        if (value <= threshold) return false;
        int other = red ? s[2] : s[0];
        return value > dom * s[1] && value > dom * other;
    }

    private void rebuildColorSummary() {
        Map<String, List<int[]>> byLabel = new LinkedHashMap<>();
        StringBuilder csv = new StringBuilder("label,r,g,b,a,dist\n");
        for (int i = 0; i < samples.size(); i++) {
            String l = sampleLabels.get(i);
            List<int[]> list = byLabel.get(l);
            if (list == null) {
                list = new ArrayList<>();
                byLabel.put(l, list);
            }
            int[] s = samples.get(i);
            list.add(s);
            csv.append(l).append(',').append(s[0]).append(',').append(s[1]).append(',')
                    .append(s[2]).append(',').append(s[3]).append(',')
                    .append(FlyRecorder.r(sampleDist.get(i), 2)).append('\n');
        }
        TuningHub.INSTANCE.setColorCsv(csv.toString());

        StringBuilder sb = new StringBuilder(800);
        sb.append("{\"labels\":[");
        boolean first = true;
        for (Map.Entry<String, List<int[]>> e : byLabel.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            int[] mn = {Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE};
            int[] mx = {Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE};
            double[] mean = new double[4];
            int redHits = 0, blueHits = 0;
            for (int[] s : e.getValue()) {
                for (int k = 0; k < 4; k++) {
                    mn[k] = Math.min(mn[k], s[k]);
                    mx[k] = Math.max(mx[k], s[k]);
                    mean[k] += s[k] / (double) e.getValue().size();
                }
                if (dominant(s, true, intake.thresholdRed, intake.thresholdBlue, intake.DOMINANCE)) redHits++;
                if (dominant(s, false, intake.thresholdRed, intake.thresholdBlue, intake.DOMINANCE)) blueHits++;
            }
            sb.append("{\"label\":\"").append(TuningHub.esc(e.getKey()))
                    .append("\",\"n\":").append(e.getValue().size())
                    .append(",\"mean\":[").append(FlyRecorder.r(mean[0], 1)).append(',')
                    .append(FlyRecorder.r(mean[1], 1)).append(',').append(FlyRecorder.r(mean[2], 1))
                    .append(',').append(FlyRecorder.r(mean[3], 1)).append(']')
                    .append(",\"min\":[").append(mn[0]).append(',').append(mn[1]).append(',')
                    .append(mn[2]).append(',').append(mn[3]).append(']')
                    .append(",\"max\":[").append(mx[0]).append(',').append(mx[1]).append(',')
                    .append(mx[2]).append(',').append(mx[3]).append(']')
                    .append(",\"redHits\":").append(redHits)
                    .append(",\"blueHits\":").append(blueHits).append('}');
        }
        sb.append(']');

        suggestion = null;
        if (byLabel.containsKey("RED") && byLabel.containsKey("BLUE") && byLabel.size() >= 3) {
            int[] red = fit(true);
            int[] blue = fit(false);
            // One DOMINANCE is shared by both channels in the shipped intake; take the stricter.
            double dom = Math.max(red[1], blue[1]) / 100.0;
            suggestion = new int[]{red[0], blue[0]};
            suggestionDom = dom;
            int[] cur = confusion(intake.thresholdRed, intake.thresholdBlue, intake.DOMINANCE);
            int[] sug = confusion(red[0], blue[0], dom);
            sb.append(",\"suggest\":{\"thresholdRed\":").append(red[0])
                    .append(",\"thresholdBlue\":").append(blue[0])
                    .append(",\"dominance\":").append(FlyRecorder.r(dom, 2))
                    .append(",\"errorsSuggested\":").append(sug[0])
                    .append(",\"errorsCurrent\":").append(cur[0])
                    .append(",\"total\":").append(sug[1]).append('}');
        } else {
            sb.append(",\"suggest\":null");
        }
        sb.append('}');
        colorSummary = sb.toString();
    }

    /**
     * Grid search for one channel: maximise correct calls of "this sample is that colour" over
     * every stored sample (positives = that label, negatives = every other label), then take the
     * centre of the tied-best region so the boundary sits as far from both clusters as the data
     * allows. Returns {threshold, dominance x 100}.
     */
    private int[] fit(boolean red) {
        String pos = red ? "RED" : "BLUE";
        int ch = red ? 0 : 2;
        int maxV = 1;
        for (int[] s : samples) maxV = Math.max(maxV, s[ch]);
        int stepT = Math.max(1, maxV / 200);
        int best = -1;
        List<int[]> ties = new ArrayList<>();
        for (int thr = 0; thr <= maxV; thr += stepT) {
            for (int d100 = 100; d100 <= 300; d100 += 5) {
                int ok = 0;
                for (int i = 0; i < samples.size(); i++) {
                    boolean want = pos.equals(sampleLabels.get(i));
                    boolean got = dominant(samples.get(i), red, thr, thr, d100 / 100.0);
                    if (want == got) ok++;
                }
                if (ok > best) {
                    best = ok;
                    ties.clear();
                }
                if (ok == best) ties.add(new int[]{thr, d100});
            }
        }
        double mt = 0, md = 0;
        for (int[] t : ties) {
            mt += t[0] / (double) ties.size();
            md += t[1] / (double) ties.size();
        }
        int[] pick = ties.get(0);
        double bestDist = Double.MAX_VALUE;
        for (int[] t : ties) {
            double dist = Math.pow((t[0] - mt) / maxV, 2) + Math.pow((t[1] - md) / 200.0, 2);
            if (dist < bestDist) {
                bestDist = dist;
                pick = t;
            }
        }
        return pick;
    }

    /** {misclassified, total} over both channels for a parameter set. */
    private int[] confusion(int thrRed, int thrBlue, double dom) {
        int wrong = 0;
        for (int i = 0; i < samples.size(); i++) {
            String l = sampleLabels.get(i);
            if (dominant(samples.get(i), true, thrRed, thrBlue, dom) != "RED".equals(l)) wrong++;
            if (dominant(samples.get(i), false, thrRed, thrBlue, dom) != "BLUE".equals(l)) wrong++;
        }
        return new int[]{wrong, samples.size() * 2};
    }

    // ------------------------------------------------------------------------- publish

    private String stateJson() {
        StringBuilder sb = new StringBuilder(4096);
        int n = Math.min(dtCount, dtRing.length);
        double[] dts = Arrays.copyOf(dtRing, n);
        Arrays.sort(dts);
        double dtSum = 0;
        for (double d : dts) dtSum += d;
        sb.append("{\"live\":true,\"started\":").append(started)
                .append(",\"message\":\"").append(TuningHub.esc(message))
                .append("\",\"cmdSeq\":").append(cmdSeq)
                .append(",\"volts\":").append(FlyRecorder.r(volts, 2))
                .append(",\"loopHzTrue\":").append(n == 0 || dtSum == 0 ? "null" : FlyRecorder.r(n / dtSum, 1))
                .append(",\"dtMeanMs\":").append(n == 0 ? "null" : FlyRecorder.r(1000 * dtSum / n, 2))
                .append(",\"dtP90Ms\":").append(n == 0 ? "null" : FlyRecorder.r(1000 * dts[(int) (0.9 * (n - 1))], 2))
                .append(",\"watchdog\":").append(watchdogTripped)
                .append(",\"clientAgeMs\":").append(TuningHub.INSTANCE.msSinceClient())
                .append(",\"clientTimeoutMs\":").append(CLIENT_TIMEOUT_MS)
                .append(",\"velCap\":").append(VEL_CAP_TPS)
                .append(",\"atSpeedFrac\":").append(FlyRecorder.r(atSpeedFrac, 4));

        sb.append(",\"wheels\":[");
        for (int i = 0; i < wheels.length; i++) {
            Wheel w = wheels[i];
            if (i > 0) sb.append(',');
            boolean atSpeed = w.mode == MODE_VEL && w.target != 0
                    && Math.abs(w.vel - w.target) <= atSpeedFrac * Math.abs(w.target);
            sb.append("{\"name\":\"").append(w.name)
                    .append("\",\"device\":\"").append(w.device)
                    .append("\",\"ok\":").append(w.motor != null)
                    .append(",\"error\":\"").append(TuningHub.esc(w.error))
                    .append("\",\"mode\":\"").append(MODE_NAMES[w.mode])
                    .append("\",\"target\":").append(FlyRecorder.r(w.target, 1))
                    .append(",\"power\":").append(nanToNull(w.power, 3))
                    .append(",\"vel\":").append(FlyRecorder.r(w.vel, 1))
                    .append(",\"cur\":").append(nanToNull(w.cur, 2))
                    .append(",\"atSpeed\":").append(atSpeed)
                    .append(",\"reversed\":").append(w.reversed)
                    .append(",\"kicking\":").append(w.kicking)
                    .append(",\"pidf\":[").append(w.kP).append(',').append(w.kI).append(',')
                    .append(w.kD).append(',').append(w.kF).append(']')
                    .append(",\"hubPidf\":\"").append(w.hubPidf)
                    .append("\",\"dipState\":").append(w.dip)
                    .append(",\"stepPhase\":").append(w.stepPhase)
                    .append(",\"lastStep\":").append(w.lastStep)
                    .append(",\"ffRunning\":").append(w.ffIdx >= 0)
                    .append(",\"ffLevel\":").append(w.ffIdx)
                    .append(",\"ff\":").append(w.ffResult)
                    .append('}');
        }
        sb.append(']');

        sb.append(",\"events\":[");
        for (int i = Math.max(0, events.size() - 25); i < events.size(); i++) {
            if (i > Math.max(0, events.size() - 25)) sb.append(',');
            sb.append(events.get(i));
        }
        sb.append("],\"eventCount\":").append(eventSeq);

        sb.append(",\"rec\":{\"recording\":").append(recorder.isRecording())
                .append(",\"label\":\"").append(TuningHub.esc(recorder.label()))
                .append("\",\"size\":").append(recorder.size())
                .append(",\"capacity\":").append(FlyRecorder.CAPACITY)
                .append(",\"current\":").append(recCurrent).append('}');

        sb.append(",\"turrets\":[");
        for (int i = 0; i < turrets.length; i++) {
            TurretCal t = turrets[i];
            if (i > 0) sb.append(',');
            sb.append("{\"name\":\"").append(t.name)
                    .append("\",\"device\":\"").append(t.device)
                    .append("\",\"ok\":").append(t.servo != null)
                    .append(",\"pos\":").append(nanToNull(t.pos, 4))
                    .append(",\"deg\":").append(nanToNull(t.degFor(t.pos), 1))
                    .append(",\"center\":").append(FlyRecorder.r(t.center, 4))
                    .append(",\"reversed\":").append(t.reversed)
                    .append(",\"grA\":").append(t.gearA)
                    .append(",\"grB\":").append(t.gearB)
                    .append(",\"servoRange\":").append(t.servoRange)
                    .append(",\"rangeDeg\":").append(FlyRecorder.r(t.range(), 2)).append('}');
        }
        sb.append(']');

        sb.append(",\"servos\":[");
        for (int i = 0; i < servos.length; i++) {
            ServoCal s = servos[i];
            if (i > 0) sb.append(',');
            sb.append("{\"name\":\"").append(s.name)
                    .append("\",\"device\":\"").append(s.device)
                    .append("\",\"ok\":").append(s.servo != null)
                    .append(",\"pos\":").append(nanToNull(s.pos, 4))
                    .append(",\"marks\":{");
            boolean first = true;
            for (Map.Entry<String, Double> m : s.marks.entrySet()) {
                if (!first) sb.append(',');
                first = false;
                sb.append('"').append(m.getKey()).append("\":")
                        .append(m.getValue() == null ? "null" : FlyRecorder.r(m.getValue(), 4));
            }
            sb.append("}}");
        }
        sb.append(']');

        sb.append(",\"intake\":{\"ok\":").append(intakeSys != null)
                .append(",\"error\":\"").append(TuningHub.esc(intakeError))
                .append("\",\"mode\":\"").append(intakeMode)
                .append("\",\"alliance\":\"").append(alliance).append("\"}");

        int[] live = {cr, cg, cb, ca};
        sb.append(",\"color\":{\"ok\":").append(color != null)
                .append(",\"hasDistance\":").append(colorDistance != null)
                .append(",\"stream\":").append(colorStream)
                .append(",\"pausedForRec\":").append(recorder.isRecording())
                .append(",\"r\":").append(cr).append(",\"g\":").append(cg)
                .append(",\"b\":").append(cb).append(",\"a\":").append(ca)
                .append(",\"dist\":").append(nanToNull(cdist, 2))
                .append(",\"redDominant\":").append(dominant(live, true, intake.thresholdRed, intake.thresholdBlue, intake.DOMINANCE))
                .append(",\"blueDominant\":").append(dominant(live, false, intake.thresholdRed, intake.thresholdBlue, intake.DOMINANCE))
                .append(",\"thresholdRed\":").append(intake.thresholdRed)
                .append(",\"thresholdBlue\":").append(intake.thresholdBlue)
                .append(",\"dominance\":").append(intake.DOMINANCE)
                .append(",\"sampling\":").append(sampleRemaining)
                .append(",\"samplingLabel\":\"").append(TuningHub.esc(sampleLabel))
                .append("\",\"samples\":").append(samples.size())
                .append(",\"summary\":").append(colorSummary).append('}');

        sb.append(",\"export\":").append(exportJson());
        return sb.append('}').toString();
    }

    /**
     * Every value this tool owns, as {file, name, value} with value written as Java source. The
     * host splicer (tools/mechtune/mechtune.py export --write) replaces the right-hand side of
     * that declaration by name. Unmarked servo positions are left out rather than exported as 0.
     */
    private String exportJson() {
        List<String[]> out = new ArrayList<>();
        String fw = "systems/flywheel.java";
        out.add(new String[]{fw, "POLLEN_FLYWHEEL_REVERSED", Boolean.toString(wheels[0].reversed)});
        out.add(new String[]{fw, "NECTAR_FLYWHEEL_REVERSED", Boolean.toString(wheels[1].reversed)});
        out.add(new String[]{fw, "AT_SPEED_FRACTION", Double.toString(atSpeedFrac)});
        out.add(new String[]{fw, "POLLEN_PIDF", pidfSource(wheels[0])});
        out.add(new String[]{fw, "NECTAR_PIDF", pidfSource(wheels[1])});
        String tu = "systems/turret.java";
        out.add(new String[]{tu, "POLLEN_TURRET_CENTER", FlyRecorder.r(turrets[0].center, 4)});
        out.add(new String[]{tu, "NECTAR_TURRET_CENTER", FlyRecorder.r(turrets[1].center, 4)});
        out.add(new String[]{tu, "POLLEN_TURRET_REVERSED", Boolean.toString(turrets[0].reversed)});
        out.add(new String[]{tu, "NECTAR_TURRET_REVERSED", Boolean.toString(turrets[1].reversed)});
        out.add(new String[]{tu, "POLLEN_TURRET_GEAR_RATIO", gearSource(turrets[0])});
        out.add(new String[]{tu, "NECTAR_TURRET_GEAR_RATIO", gearSource(turrets[1])});
        out.add(new String[]{tu, "SERVO_RANGE_DEG", trim(turrets[0].servoRange)});
        String in = "systems/intake.java";
        out.add(new String[]{in, "thresholdRed", Integer.toString(intake.thresholdRed)});
        out.add(new String[]{in, "thresholdBlue", Integer.toString(intake.thresholdBlue)});
        out.add(new String[]{in, "DOMINANCE", Double.toString(intake.DOMINANCE)});
        for (ServoCal s : servos) {
            for (Map.Entry<String, Double> m : s.marks.entrySet()) {
                if (m.getValue() != null) {
                    out.add(new String[]{s.file, s.fields.get(m.getKey()),
                            FlyRecorder.r(m.getValue(), 4)});
                }
            }
        }
        StringBuilder sb = new StringBuilder(1024).append('[');
        for (int i = 0; i < out.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append("{\"file\":\"").append(out.get(i)[0])
                    .append("\",\"name\":\"").append(out.get(i)[1])
                    .append("\",\"value\":\"").append(TuningHub.esc(out.get(i)[2])).append("\"}");
        }
        return sb.append(']').toString();
    }

    private static String pidfSource(Wheel w) {
        return "new PIDFCoefficients(" + trim(w.kP) + ", " + trim(w.kI) + ", " + trim(w.kD)
                + ", " + trim(w.kF) + ")";
    }

    private static String gearSource(TurretCal t) {
        return "{" + trim(t.gearA) + ", " + trim(t.gearB) + "}";
    }

    private static String trim(double v) {
        return v == Math.rint(v) && Math.abs(v) < 1e9 ? Long.toString((long) v) : Double.toString(v);
    }

    // ------------------------------------------------------------------------- helpers

    private static double num(Map<String, String> c, String key, double fallback) {
        String v = c.get(key);
        if (v == null || v.trim().isEmpty()) return fallback;
        try {
            return Double.parseDouble(v.trim());
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(key + "=" + v + " is not a number");
        }
    }

    private static double[] parseList(String s, double[] fallback) {
        if (s == null || s.trim().isEmpty()) return fallback;
        String[] parts = s.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            out[i] = clamp(Double.parseDouble(parts[i].trim()), -1, 1);
        }
        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String nanToNull(double v, int digits) {
        return Double.isNaN(v) || Double.isInfinite(v) ? "null" : FlyRecorder.r(v, digits);
    }

    private static String shortError(Throwable e) {
        return e.getClass().getSimpleName() + (e.getMessage() == null ? "" : ": " + e.getMessage());
    }
}
