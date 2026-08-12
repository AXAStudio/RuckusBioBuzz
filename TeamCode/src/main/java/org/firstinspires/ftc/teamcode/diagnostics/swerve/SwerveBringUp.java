package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Interactive swerve bring-up, wiring verification and calibration tool.
 *
 * <p>Run this OpMode, then open <b>http://192.168.43.1:8080/swerve</b> on a laptop joined to the
 * robot's WiFi. Every control is also reachable from gamepad 1 so the tool still works when the
 * network does not.
 *
 * <p>The tool deliberately talks to raw {@link DcMotorEx}/{@link CRServo}/{@link AnalogInput}
 * devices for wiring work, so it functions even when the constants in
 * {@code SwerveDrivetrainConstants} are wrong or unknown. Once a pod is calibrated it is driven
 * through a real {@link CoaxialPod}, so PID tuning and kinematics behave exactly as they will in
 * competition code.
 *
 * <h2>What the automatic scans determine</h2>
 * <ul>
 *   <li><b>Encoder pairing</b> - which analog channel belongs to which pod. This matters because
 *       the Axon feedback wires are spliced two-per-analog-port, so the mapping is easy to get
 *       wrong and invisible until the robot drives strangely.</li>
 *   <li><b>Servo direction</b> - {@link CoaxialPod}'s turn PID requires that positive servo power
 *       makes the encoder reading increase. That relationship is measured, not guessed.</li>
 *   <li><b>Analog range</b> - the true min/max voltage of each Axon absolute output.</li>
 * </ul>
 *
 * <p>{@code encoderReversed} is the one value that cannot be measured from inside the robot: it
 * says whether a rising encoder reading means counter-clockwise pod rotation. The dashboard asks
 * the operator to watch one pod turn and answer once.
 */
@TeleOp(name = "Swerve Bring-Up", group = "Diagnostics")
public class SwerveBringUp extends OpMode {

    // ---------------------------------------------------------------- tuning knobs

    private static final int POD_COUNT = 4;

    /** Servo power used for wiring/sweep routines. Low enough to be safe on blocks. */
    private static final double SCAN_SERVO_POWER = 0.45;
    private static final double SCAN_SETTLE_S = 0.25;
    private static final double SCAN_MOVE_S = 0.70;
    private static final double SWEEP_SECONDS = 4.0;

    /** Drive-motor identification pulse. */
    private static final double MOTOR_PULSE_POWER = 0.25;
    private static final double MOTOR_PULSE_S = 0.60;

    /** Manual jog power for nudging a pod into alignment. */
    private static final double NUDGE_POWER = 0.25;

    /** Jog durations: a short tap for alignment, a longer run for watching rotation direction. */
    private static final double NUDGE_SECONDS = 0.35;
    private static final double SPIN_SECONDS = 3.0;

    /** Drive test stops if no command arrives within this window. */
    private static final long DRIVE_WATCHDOG_MS = 400;

    /**
     * Must match the {@link SwerveConstants} used to build the drivetrain, because the visualizer
     * reproduces {@code Swerve.arcadeDrive}'s kinematics to show the commanded pod state.
     */
    private static final double SWERVE_EPSILON = 0.05;
    private static final double SWERVE_MAX_POWER = 1.0;

    /** Minimum total analog movement (volts) for a channel to count as "responded". */
    private static final double SCAN_MIN_RESPONSE_V = 0.15;

    /** A channel must move this many times more than the runner-up to be an unambiguous match. */
    private static final double SCAN_AMBIGUITY_RATIO = 2.0;

    /**
     * Below this the analog channel is carrying no signal at all. A powered Axon outputs somewhere
     * in 0-3.3 V depending on shaft position, so a flat reading this low means the encoder is
     * unpowered or its wire is not landing on the port - not that the pod failed to turn.
     */
    private static final double DEAD_CHANNEL_VOLTS = 0.05;

    /** Voltage jumps larger than this fraction of range are treated as encoder wrap, not motion. */
    private static final double WRAP_FRACTION = 0.5;

    private static final int TRACE_LEN = 260;

    private static final File CAL_FILE = new File(AppUtil.FIRST_FOLDER, "swerve_bringup_cal.txt");

    // ---------------------------------------------------------------- modes

    private enum Mode {
        IDLE,
        WIRE_SCAN,
        MOTOR_PULSE,
        /** Manual turn-servo jog. Drives only the servo, and always times out. */
        JOG,
        ENC_SWEEP,
        PID,
        AUTOTUNE,
        DRIVE
    }

    /** Actions that can move the robot, and so require START to have been pressed. */
    private static boolean isMotionCommand(String action) {
        return "wireScan".equals(action)
                || "sweep".equals(action)
                || "pulseMotor".equals(action)
                || "spinServo".equals(action)
                || "nudge".equals(action)
                || "pidStep".equals(action)
                || "pidStepAll".equals(action)
                || "autoTune".equals(action)
                || "drive".equals(action);
    }

    // ---------------------------------------------------------------- state

    private final PodCal[] cals = new PodCal[POD_COUNT];
    private final DcMotorEx[] motors = new DcMotorEx[POD_COUNT];
    private final CRServo[] servos = new CRServo[POD_COUNT];
    private final AnalogInput[] encoders = new AnalogInput[POD_COUNT];
    private final double[] volts = new double[POD_COUNT];

    /**
     * The four analog channels addressed by their fixed config names, independent of which pod each
     * one currently belongs to. The wiring scan must work in this space: once a scan remaps a pod's
     * encoder, {@link #encoders} no longer lines up with channel numbering, so scanning against it
     * would mislabel the result on any re-run.
     */
    private final AnalogInput[] channels = new AnalogInput[POD_COUNT];
    private final String[] channelNames = new String[POD_COUNT];
    private final double[] channelVolts = new double[POD_COUNT];

    private final List<String> hwErrors = new ArrayList<>();
    private final List<String> scanNotes = new ArrayList<>();

    private CoaxialPod[] pods;
    private Swerve swerve;
    private boolean podsDirty = true;
    private String podBuildError;

    private VoltageSensor voltageSensor;

    private Mode mode = Mode.IDLE;
    private int selected = 0;
    private String message = "Ready. Put the robot on blocks before running any scan.";

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double loopHz;

    // routine bookkeeping
    private int routinePod;
    private int routinePhase;
    private boolean routineActive;

    /** Per-(pod, channel) accumulated absolute and signed movement during a wiring scan. */
    private double[][] scanAbs;
    private double[][] scanSigned;
    private double[] lastVolts = new double[POD_COUNT];

    /** Highest voltage each channel reached during a scan, used to tell "dead" from "didn't move". */
    private final double[] scanChannelPeak = new double[POD_COUNT];

    /** Observed min/max during a sweep. */
    private final double[] sweepMin = new double[POD_COUNT];
    private final double[] sweepMax = new double[POD_COUNT];

    // PID step-response trace
    /**
     * Rolling record of where the selected pod was told to point versus where it actually is.
     * Written whenever a target exists, so it covers PID steps, drive tests and auto-tuning alike.
     */
    private final double[] traceT = new double[TRACE_LEN];
    private final double[] traceTarget = new double[TRACE_LEN];
    private final double[] traceActual = new double[TRACE_LEN];
    private int traceCount;
    private int traceHead;
    private int tracePod = -1;
    private double pidTargetRad;
    private boolean pidHolding;

    /**
     * Step every pod together rather than one at a time.
     *
     * <p>On the ground a single pod turning alone has to scrub its tire while the other three pin
     * the chassis - a far harsher and more variable load than real driving, where all four rotate
     * together. Stepping them together is the representative test.
     */
    private boolean pidAllPods;

    // drive test
    private double driveForward;
    private double driveStrafe;
    private double driveTurn;
    /** Watchdog: drive stops if the browser stops sending commands (closed tab, WiFi drop). */
    private long lastDriveCmdMs;

    /** True once START has been pressed. Nothing is allowed to move before then. */
    private boolean started;

    /** Zero-input behaviour of the bench drivetrain: X_LOCK when true, hold heading when false. */
    private boolean xLock = true;

    /** How long the current jog should run before stopping itself. */
    private double jogSeconds;

    private String exportText = "";

    private final PodAutoTuner tuner = new PodAutoTuner();
    private final ElapsedTime autoTuneTimer = new ElapsedTime();

    /** Commanded pod state for the visualizer. NaN theta means "nothing commanded right now". */
    private final double[] targetTheta = new double[POD_COUNT];
    private final double[] targetPower = new double[POD_COUNT];

    // gamepad edge detection
    private boolean prevA;
    private boolean prevB;
    private boolean prevX;
    private boolean prevY;
    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevStart;

    // ---------------------------------------------------------------- lifecycle

    @Override
    public void init() {
        // Defaults mirror SwerveDrivetrainConstants: index 2 = LF, 1 = RF, 3 = LB, 0 = RB.
        double dtLength = 146.420;
        double dtWidth = 154.240;
        cals[0] = new PodCal(0, "RB", -dtLength, -dtWidth);
        cals[1] = new PodCal(1, "RF", dtLength, -dtWidth);
        cals[2] = new PodCal(2, "LF", dtLength, dtWidth);
        cals[3] = new PodCal(3, "LB", -dtLength, dtWidth);

        loadCalibration();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        acquireHardware();

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (RuntimeException e) {
            hwErrors.add("No voltage sensor found.");
        }

        SwerveBench.INSTANCE.clearCommands();
        publish();
        pushTelemetry();
    }

    @Override
    public void init_loop() {
        serviceLoop();
    }

    @Override
    public void start() {
        started = true;
        message = "Started. Robot can move now.";
        phaseTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        serviceLoop();
    }

    @Override
    public void stop() {
        allStop();
        SwerveBench.INSTANCE.markStopped();
    }

    /** Shared body so the dashboard is fully usable during init, before START is pressed. */
    private void serviceLoop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            loopHz = 0.9 * loopHz + 0.1 * (1.0 / dt);
        }

        drainCommands();
        readEncoders();
        handleGamepad();
        runMode();
        publish();
        pushTelemetry();
    }

    // ---------------------------------------------------------------- hardware

    private void acquireHardware() {
        hwErrors.clear();
        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            try {
                motors[i] = hardwareMap.get(DcMotorEx.class, c.motorName);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motors[i].setDirection(c.driveDirection);
            } catch (RuntimeException e) {
                motors[i] = null;
                hwErrors.add("Missing drive motor \"" + c.motorName + "\" (pod " + i + ").");
            }
            try {
                servos[i] = hardwareMap.get(CRServo.class, c.servoName);
                servos[i].setDirection(c.servoDirection);
                servos[i].setPower(0);
            } catch (RuntimeException e) {
                servos[i] = null;
                hwErrors.add("Missing turn servo \"" + c.servoName + "\" (pod " + i + ").");
            }
            try {
                encoders[i] = hardwareMap.get(AnalogInput.class, c.encoderName);
            } catch (RuntimeException e) {
                encoders[i] = null;
                hwErrors.add("Missing analog encoder \"" + c.encoderName + "\" (pod " + i + ").");
            }

            // Fixed channel names, never remapped, so repeated wiring scans stay meaningful.
            channelNames[i] = "se" + i;
            try {
                channels[i] = hardwareMap.get(AnalogInput.class, channelNames[i]);
            } catch (RuntimeException e) {
                channels[i] = null;
            }
        }
    }

    private void readEncoders() {
        for (int i = 0; i < POD_COUNT; i++) {
            volts[i] = readVoltage(encoders[i]);
            channelVolts[i] = readVoltage(channels[i]);
        }
    }

    private static double readVoltage(AnalogInput input) {
        if (input == null) {
            return Double.NaN;
        }
        try {
            return input.getVoltage();
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /** Rebuilds the real Pedro pods after any calibration change. */
    private void rebuildPods() {
        podBuildError = null;
        try {
            CoaxialPod[] built = new CoaxialPod[POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                built[i] = cals[i].toCoaxialPod(hardwareMap);
            }
            pods = built;

            SwerveConstants sc = new SwerveConstants()
                    .velocity(73.9)
                    // Matches competition by default so the drive test behaves like the real robot.
                    // IGNORE_ANGLE_CHANGES is available from the dashboard because X_LOCK hides what
                    // the pods are doing while tuning - every dropout yanks all four to the X.
                    .zeroPowerBehavior(xLock
                            ? SwerveConstants.ZeroPowerBehavior.X_LOCK
                            : SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
                    .useBrakeModeInTeleOp(false)
                    // Set explicitly so they cannot drift from the visualizer's copy of the math.
                    .maxPower(SWERVE_MAX_POWER)
                    .epsilon(SWERVE_EPSILON);
            // Pedro's builder passes pods as leftFront, rightFront, leftBack, rightBack; each pod
            // carries its own offset so ordering only affects debug output.
            swerve = new Swerve(hardwareMap, sc, pods[2], pods[1], pods[3], pods[0]);
            podsDirty = false;
        } catch (RuntimeException e) {
            pods = null;
            swerve = null;
            podBuildError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private void ensurePods() {
        if (podsDirty || pods == null) {
            rebuildPods();
        }
    }

    private void allStop() {
        for (int i = 0; i < POD_COUNT; i++) {
            if (servos[i] != null) {
                servos[i].setPower(0);
            }
            if (motors[i] != null) {
                motors[i].setPower(0);
            }
        }
        driveForward = 0;
        driveStrafe = 0;
        driveTurn = 0;
    }

    // ---------------------------------------------------------------- mode runner

    private void runMode() {
        // Nothing moves until the driver station START button is pressed. The dashboard stays
        // fully live during INIT so wiring can be inspected safely.
        if (!started && mode != Mode.IDLE) {
            allStop();
            mode = Mode.IDLE;
        }

        switch (mode) {
            case WIRE_SCAN:
                runWireScan();
                break;
            case ENC_SWEEP:
                runEncoderSweep();
                break;
            case MOTOR_PULSE:
                runMotorPulse();
                break;
            case JOG:
                runJog();
                break;
            case PID:
                runPidMode();
                break;
            case AUTOTUNE:
                runAutoTune();
                break;
            case DRIVE:
                runDriveMode();
                break;
            case IDLE:
            default:
                // Servos limp so pods can be turned by hand for zeroing.
                for (int i = 0; i < POD_COUNT; i++) {
                    if (servos[i] != null) {
                        servos[i].setPower(0);
                    }
                    if (motors[i] != null) {
                        motors[i].setPower(0);
                    }
                }
                break;
        }
    }

    private void setMode(Mode next) {
        allStop();
        mode = next;
        routineActive = false;
        routinePod = 0;
        routinePhase = 0;
        phaseTimer.reset();
    }

    // ---------------------------------------------------------------- wiring scan

    /**
     * Pulses each turn servo in turn while watching all analog channels.
     *
     * <p>The channel that moves is that pod's encoder, which resolves the spliced-wire mapping.
     * The direction it moves sets {@code servoDirection}, because {@link CoaxialPod} requires
     * positive servo power to increase the encoder reading.
     */
    private void runWireScan() {
        if (!routineActive) {
            // Pre-flight: if every channel is flat at zero there is no signal to measure, so say
            // that plainly instead of spending four seconds to report four identical failures.
            if (allChannelsDead()) {
                scanNotes.clear();
                scanNotes.add("All four analog channels read 0.00 V, so no encoder signal is "
                        + "reaching the Control Hub. This is a power or wiring fault common to all "
                        + "four pods, not four separate faults.");
                scanNotes.add("Check first: the Axon encoders only output a position voltage while "
                        + "the servos are powered. Confirm servo power, then that each spliced "
                        + "signal wire lands on analog 0-3 and shares ground with the hub.");
                scanNotes.add("Quick test needing no servo: leave this in IDLE so the pods are "
                        + "limp, turn a pod by hand and watch its volts on the pod card. Still "
                        + "0.00 V means the signal wire, not the servo.");
                message = "Scan aborted: no encoder signal on any channel.";
                setMode(Mode.IDLE);
                return;
            }

            scanAbs = new double[POD_COUNT][POD_COUNT];
            scanSigned = new double[POD_COUNT][POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                scanChannelPeak[i] = 0;
            }
            scanNotes.clear();
            routineActive = true;
            routinePod = 0;
            routinePhase = 0;
            phaseTimer.reset();
            // Force a known hardware direction so "positive power" is unambiguous.
            for (int i = 0; i < POD_COUNT; i++) {
                if (servos[i] != null) {
                    servos[i].setDirection(DcMotorSimple.Direction.FORWARD);
                    servos[i].setPower(0);
                }
            }
            System.arraycopy(channelVolts, 0, lastVolts, 0, POD_COUNT);
            message = "Wiring scan running - watch which pod turns.";
        }

        if (routinePod >= POD_COUNT) {
            finishWireScan();
            return;
        }

        if (routinePhase == 0) {
            // settle
            if (servos[routinePod] != null) {
                servos[routinePod].setPower(0);
            }
            if (phaseTimer.seconds() >= SCAN_SETTLE_S) {
                System.arraycopy(channelVolts, 0, lastVolts, 0, POD_COUNT);
                routinePhase = 1;
                phaseTimer.reset();
            }
            return;
        }

        // moving phase
        if (servos[routinePod] != null) {
            servos[routinePod].setPower(SCAN_SERVO_POWER);
        }
        accumulateScanDeltas(routinePod);

        if (phaseTimer.seconds() >= SCAN_MOVE_S) {
            if (servos[routinePod] != null) {
                servos[routinePod].setPower(0);
            }
            routinePod++;
            routinePhase = 0;
            phaseTimer.reset();
        }
    }

    /** True when no analog channel is carrying a usable signal. */
    private boolean allChannelsDead() {
        for (int ch = 0; ch < POD_COUNT; ch++) {
            double v = channelVolts[ch];
            if (!Double.isNaN(v) && v >= DEAD_CHANNEL_VOLTS) {
                return false;
            }
        }
        return true;
    }

    private void accumulateScanDeltas(int drivenPod) {
        double range = 3.3;
        for (int ch = 0; ch < POD_COUNT; ch++) {
            if (Double.isNaN(channelVolts[ch]) || Double.isNaN(lastVolts[ch])) {
                continue;
            }
            scanChannelPeak[ch] = Math.max(scanChannelPeak[ch], channelVolts[ch]);
            double d = channelVolts[ch] - lastVolts[ch];
            // Ignore wrap-around jumps; they are not real movement magnitude. A wrap then
            // contributes nothing, rather than a large delta with a misleading sign.
            if (Math.abs(d) < range * WRAP_FRACTION) {
                scanAbs[drivenPod][ch] += Math.abs(d);
                scanSigned[drivenPod][ch] += d;
            }
            lastVolts[ch] = channelVolts[ch];
        }
    }

    private void finishWireScan() {
        int[] assigned = new int[POD_COUNT];
        for (int p = 0; p < POD_COUNT; p++) {
            int best = -1;
            double bestVal = 0;
            double runnerUp = 0;
            for (int ch = 0; ch < POD_COUNT; ch++) {
                double v = scanAbs[p][ch];
                if (v > bestVal) {
                    runnerUp = bestVal;
                    bestVal = v;
                    best = ch;
                } else if (v > runnerUp) {
                    runnerUp = v;
                }
            }

            if (best < 0 || bestVal < SCAN_MIN_RESPONSE_V) {
                assigned[p] = -1;
                // Distinguish "the channel is dead" from "the channel is alive but nothing moved" -
                // they point at completely different faults.
                boolean anyChannelAlive = false;
                for (int ch = 0; ch < POD_COUNT; ch++) {
                    if (scanChannelPeak[ch] >= DEAD_CHANNEL_VOLTS) {
                        anyChannelAlive = true;
                        break;
                    }
                }

                if (!anyChannelAlive) {
                    scanNotes.add("Pod " + p + " (" + cals[p].servoName
                            + "): every analog channel is flat at 0 V, so there is no signal to "
                            + "measure. Check encoder power and the spliced signal wires.");
                } else {
                    scanNotes.add("Pod " + p + " (" + cals[p].servoName
                            + "): channels are live but none moved while this servo was driven. "
                            + "Either the servo is not turning (power, or a dead servo port), or "
                            + "this pod's encoder is unplugged.");
                }
                continue;
            }
            if (runnerUp > 0 && bestVal < runnerUp * SCAN_AMBIGUITY_RATIO) {
                scanNotes.add("Pod " + p + ": encoder match is ambiguous (best "
                        + fmt(bestVal) + "V vs " + fmt(runnerUp)
                        + "V). Two pods may be moving together, or a splice is shorted.");
            }

            assigned[p] = best;
            cals[p].discoveredEncoderIndex = best;
            cals[p].encoderName = channelNames[best];

            // CoaxialPod's PID assumes positive servo power raises the encoder reading.
            boolean needsReverse = scanSigned[p][best] < 0;
            cals[p].setServoReversed(needsReverse);
        }

        for (int p = 0; p < POD_COUNT; p++) {
            for (int q = p + 1; q < POD_COUNT; q++) {
                if (assigned[p] >= 0 && assigned[p] == assigned[q]) {
                    scanNotes.add("Pods " + p + " and " + q + " both mapped to "
                            + channelNames[assigned[p]]
                            + ". Re-seat the splice for that analog port.");
                }
            }
        }

        acquireHardware();
        podsDirty = true;
        routineActive = false;
        message = scanNotes.isEmpty()
                ? "Wiring scan complete - encoder pairing and servo directions set."
                : "Wiring scan finished with " + scanNotes.size() + " issue(s).";
        setMode(Mode.IDLE);
    }

    // ---------------------------------------------------------------- encoder sweep

    /** Spins each pod continuously and records the true analog endpoints of the Axon output. */
    private void runEncoderSweep() {
        if (!routineActive) {
            routineActive = true;
            routinePod = 0;
            phaseTimer.reset();
            for (int i = 0; i < POD_COUNT; i++) {
                sweepMin[i] = Double.MAX_VALUE;
                sweepMax[i] = -Double.MAX_VALUE;
                if (servos[i] != null) {
                    servos[i].setDirection(cals[i].servoDirection);
                }
            }
            message = "Sweeping pods to measure encoder range.";
        }

        if (routinePod >= POD_COUNT) {
            for (int i = 0; i < POD_COUNT; i++) {
                if (sweepMax[i] > sweepMin[i]) {
                    cals[i].analogMin = sweepMin[i];
                    cals[i].analogMax = sweepMax[i];
                }
            }
            podsDirty = true;
            routineActive = false;
            message = "Encoder range captured for all pods.";
            setMode(Mode.IDLE);
            return;
        }

        int p = routinePod;
        if (servos[p] != null) {
            servos[p].setPower(SCAN_SERVO_POWER);
        }
        if (!Double.isNaN(volts[p])) {
            sweepMin[p] = Math.min(sweepMin[p], volts[p]);
            sweepMax[p] = Math.max(sweepMax[p], volts[p]);
        }

        if (phaseTimer.seconds() >= SWEEP_SECONDS) {
            if (servos[p] != null) {
                servos[p].setPower(0);
            }
            routinePod++;
            phaseTimer.reset();
        }
    }

    // ---------------------------------------------------------------- motor pulse

    private void runMotorPulse() {
        if (!routineActive) {
            routineActive = true;
            routinePod = selected;
            phaseTimer.reset();
            message = "Pulsing " + cals[routinePod].motorName + " - watch which wheel spins.";
        }
        if (phaseTimer.seconds() < MOTOR_PULSE_S) {
            if (motors[routinePod] != null) {
                motors[routinePod].setPower(MOTOR_PULSE_POWER);
            }
        } else {
            allStop();
            routineActive = false;
            setMode(Mode.IDLE);
        }
    }

    /**
     * Holds a manual turn-servo jog until it times out. The servo power was already applied by the
     * command handler; this only enforces the deadline so a pod can never be left spinning.
     */
    private void runJog() {
        for (int i = 0; i < POD_COUNT; i++) {
            if (motors[i] != null) {
                motors[i].setPower(0);
            }
        }
        if (phaseTimer.seconds() >= jogSeconds) {
            allStop();
            routineActive = false;
            setMode(Mode.IDLE);
        }
    }

    /** Applies servo power for a manual jog and arms the timeout. */
    private void beginJog(double power, double seconds, String note) {
        if (servos[selected] == null) {
            message = "Pod " + selected + " has no turn servo.";
            return;
        }
        allStop();
        servos[selected].setDirection(cals[selected].servoDirection);
        servos[selected].setPower(power);
        jogSeconds = seconds;
        mode = Mode.JOG;
        routineActive = true;
        phaseTimer.reset();
        message = note;
    }

    // ---------------------------------------------------------------- PID tuning

    private void runPidMode() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        for (int i = 0; i < POD_COUNT; i++) {
            if (motors[i] != null) {
                motors[i].setPower(0);
            }
            // Pods not under test stay limp, unless every pod is being stepped together.
            if (!pidAllPods && i != selected && servos[i] != null) {
                servos[i].setPower(0);
            }
        }

        if (!pidHolding) {
            return;
        }

        if (pidAllPods) {
            for (int i = 0; i < POD_COUNT; i++) {
                pods[i].move(pidTargetRad, 0.0, false);
            }
        } else {
            pods[selected].move(pidTargetRad, 0.0, false);
        }

        recordTrace(selected, pidTargetRad);
    }

    /**
     * Records commanded versus measured wheel heading for the graph.
     *
     * <p>Both are converted to wheel space, the frame the operator actually thinks in, rather than
     * the encoder frame {@link CoaxialPod} works in internally.
     */
    private void recordTrace(int pod, double targetWheelRad) {
        if (Double.isNaN(volts[pod])) {
            return;
        }

        PodCal c = cals[pod];
        double actualDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[pod])));
        double targetDeg = Math.toDegrees(normalizeTwoPi(targetWheelRad));

        // Keep the target on the same side of the wrap as the measurement, so a pod sitting at 359
        // chasing 1 draws as a small step rather than a full-scale cliff.
        double delta = targetDeg - actualDeg;
        if (delta > 180) {
            targetDeg -= 360;
        } else if (delta < -180) {
            targetDeg += 360;
        }

        // A pod treats target+180 as the same demand - move() flips it and drives in reverse when
        // that is the shorter rotation. Near a flip boundary the raw target snaps back and forth by
        // 180 degrees on input noise alone, while the pod sensibly does nothing. Plotting the
        // nearer equivalent shows what is actually being asked of the pod instead of that artefact.
        delta = targetDeg - actualDeg;
        if (delta > 90) {
            targetDeg -= 180;
        } else if (delta < -90) {
            targetDeg += 180;
        }

        if (pod != tracePod) {
            tracePod = pod;
            traceCount = 0;
            traceHead = 0;
        }

        traceT[traceHead] = phaseTimer.seconds();
        traceTarget[traceHead] = targetDeg;
        traceActual[traceHead] = actualDeg;
        traceHead = (traceHead + 1) % TRACE_LEN;
        if (traceCount < TRACE_LEN) {
            traceCount++;
        }
    }

    private void clearTrace(int pod) {
        tracePod = pod;
        traceCount = 0;
        traceHead = 0;
    }

    // ---------------------------------------------------------------- auto tuning

    private void runAutoTune() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        int pod = tuner.podIndex();
        if (Double.isNaN(volts[pod])) {
            message = "Pod " + pod + " encoder is unreadable; auto-tune stopped.";
            setMode(Mode.IDLE);
            return;
        }

        // Everything except the pod under test stays still.
        for (int i = 0; i < POD_COUNT; i++) {
            if (i != pod) {
                if (servos[i] != null) {
                    servos[i].setPower(0);
                }
            }
            if (motors[i] != null) {
                motors[i].setPower(0);
            }
        }

        PodCal c = cals[pod];
        double wheelDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[pod])));
        double dt = Math.max(1e-4, autoTuneTimer.seconds());
        autoTuneTimer.reset();

        PodAutoTuner.Action action = tuner.update(dt, wheelDeg, c);

        if (tuner.rebuildRequested) {
            // Coefficients live in the pod's PIDF, which is set at construction, so the pod has to
            // be rebuilt for a new gain to take effect.
            rebuildPods();
            if (pods == null) {
                message = "Cannot rebuild pods: " + podBuildError;
                setMode(Mode.IDLE);
                return;
            }
        }

        switch (action) {
            case RAW_SERVO:
                if (servos[pod] != null) {
                    servos[pod].setDirection(c.servoDirection);
                    servos[pod].setPower(tuner.rawServoPower);
                }
                break;
            case PID_HOLD:
                pods[pod].move(tuner.targetWheelRad, 0.0, false);
                recordTrace(pod, tuner.targetWheelRad);
                break;
            case FINISHED:
            default:
                allStop();
                saveCalibration();
                message = "Auto-tune complete. " + tuner.status();
                setMode(Mode.IDLE);
                return;
        }

        message = "Auto-tuning pod " + pod + ": " + tuner.status();
    }

    private void startAutoTune() {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            return;
        }
        if (Double.isNaN(volts[selected])) {
            message = "Pod " + selected + " encoder is unreadable; cannot auto-tune.";
            return;
        }

        PodCal c = cals[selected];
        double wheelDeg = Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[selected])));

        allStop();
        clearTrace(selected);
        tuner.start(selected, c, wheelDeg);
        autoTuneTimer.reset();
        phaseTimer.reset();
        mode = Mode.AUTOTUNE;
        routineActive = true;
        message = "Auto-tuning pod " + selected + ".";
    }

    private void startPidStep(double targetDeg, boolean allPods) {
        pidAllPods = allPods;
        startPidStep(targetDeg);
    }

    private void startPidStep(double targetDeg) {
        ensurePods();
        if (pods == null) {
            message = "Cannot build pods: " + podBuildError;
            return;
        }
        pidTargetRad = Math.toRadians(targetDeg);
        clearTrace(selected);
        pidHolding = true;
        phaseTimer.reset();
        mode = Mode.PID;
        message = String.format(Locale.US, "Pod %d stepping to %.0f deg.", selected, targetDeg);
    }

    // ---------------------------------------------------------------- drive test

    private void runDriveMode() {
        ensurePods();
        if (swerve == null) {
            message = "Cannot build drivetrain: " + podBuildError;
            setMode(Mode.IDLE);
            return;
        }

        // If the browser stops sending commands - tab closed, laptop asleep, WiFi drop - the last
        // command would otherwise latch and the robot would keep driving.
        boolean commanded = driveForward != 0 || driveStrafe != 0 || driveTurn != 0;
        if (commanded && System.currentTimeMillis() - lastDriveCmdMs > DRIVE_WATCHDOG_MS) {
            driveForward = 0;
            driveStrafe = 0;
            driveTurn = 0;
            message = "Drive watchdog tripped - no command from the dashboard.";
        }

        swerve.arcadeDrive(driveForward, driveStrafe, driveTurn);

        // computeTargets() runs at publish time, so the value here is from the previous loop -
        // close enough for a graph, and it keeps arcadeDrive's kinematics as the single source.
        if (!Double.isNaN(targetTheta[selected])) {
            recordTrace(selected, targetTheta[selected]);
        }
    }

    // ---------------------------------------------------------------- zeroing

    /** Captures the current raw angle of every pod as its forward reference. */
    private void captureZeros(boolean allPods) {
        int from = allPods ? 0 : selected;
        int to = allPods ? POD_COUNT : selected + 1;
        for (int i = from; i < to; i++) {
            if (Double.isNaN(volts[i])) {
                message = "Pod " + i + " encoder is not readable; zero not captured.";
                continue;
            }
            cals[i].angleOffsetRad = cals[i].rawAngleRad(volts[i]);
        }
        podsDirty = true;
        saveCalibration();
        message = allPods
                ? "Captured forward zero for all pods."
                : "Captured forward zero for pod " + selected + ".";
    }

    // ---------------------------------------------------------------- commands

    private void drainCommands() {
        Map<String, String> cmd;
        while ((cmd = SwerveBench.INSTANCE.poll()) != null) {
            handleCommand(cmd);
        }
    }

    private void handleCommand(Map<String, String> cmd) {
        String action = cmd.get("action");
        if (action == null) {
            return;
        }
        int pod = intArg(cmd, "pod", selected);
        if (pod >= 0 && pod < POD_COUNT) {
            selected = pod;
        }

        if (!started && isMotionCommand(action)) {
            message = "Press START on the driver station before running \"" + action + "\".";
            return;
        }

        switch (action) {
            case "select":
                // The selection was already applied from the "pod" parameter above; this exists so
                // the dashboard can change pods without triggering anything else.
                break;
            case "stop":
                allStop();
                pidHolding = false;
                setMode(Mode.IDLE);
                message = "Stopped.";
                break;
            case "wireScan":
                setMode(Mode.WIRE_SCAN);
                break;
            case "sweep":
                setMode(Mode.ENC_SWEEP);
                break;
            case "pulseMotor":
                setMode(Mode.MOTOR_PULSE);
                break;
            case "spinServo":
                beginJog(SCAN_SERVO_POWER, SPIN_SECONDS,
                        "Spinning " + cals[selected].servoName + " - watch it from above.");
                break;
            case "nudge": {
                double dir = doubleArg(cmd, "dir", 1);
                double sign = dir < 0 ? -1 : 1;
                beginJog(NUDGE_POWER * sign, NUDGE_SECONDS, "Nudging pod " + selected + ".");
                break;
            }
            case "zeroPod":
                captureZeros(false);
                break;
            case "zeroAll":
                captureZeros(true);
                break;
            case "setEncoderReversed": {
                // No "value" means toggle. The dashboard builds its controls once and reuses them,
                // so a handler cannot bake in the opposite of a state that keeps changing.
                boolean value = cmd.get("value") == null
                        ? !cals[selected].encoderReversed
                        : Boolean.parseBoolean(cmd.get("value"));
                if ("all".equals(cmd.get("scope"))) {
                    for (PodCal c : cals) {
                        c.encoderReversed = value;
                    }
                    message = "encoderReversed set to " + value + " on all pods.";
                } else {
                    cals[selected].encoderReversed = value;
                    message = "Pod " + selected + " encoderReversed = " + value;
                }
                podsDirty = true;
                saveCalibration();
                break;
            }
            case "setDriveReversed":
                cals[selected].setDriveReversed(cmd.get("value") == null
                        ? !cals[selected].driveReversed()
                        : Boolean.parseBoolean(cmd.get("value")));
                if (motors[selected] != null) {
                    motors[selected].setDirection(cals[selected].driveDirection);
                }
                podsDirty = true;
                saveCalibration();
                message = "Pod " + selected + " drive direction "
                        + cals[selected].driveDirection.name();
                break;
            case "setServoReversed":
                cals[selected].setServoReversed(cmd.get("value") == null
                        ? !cals[selected].servoReversed()
                        : Boolean.parseBoolean(cmd.get("value")));
                if (servos[selected] != null) {
                    servos[selected].setDirection(cals[selected].servoDirection);
                }
                podsDirty = true;
                saveCalibration();
                break;
            case "setLabel": {
                String corner = cmd.get("value");
                if (corner == null) {
                    break;
                }

                // Swap with whichever pod already holds this corner, so the four pods always
                // occupy four distinct corners. Two pods sharing one would silently break the
                // rotation kinematics.
                PodCal previousOwner = findByLabel(corner);
                String vacated = cals[selected].label;

                if (!cals[selected].setCorner(corner)) {
                    message = "Unknown corner \"" + corner + "\". Use LF, RF, LB or RB.";
                    break;
                }

                if (previousOwner != null && previousOwner != cals[selected]) {
                    previousOwner.setCorner(vacated);
                    message = "Pod " + selected + " is now " + corner
                            + "; pod " + previousOwner.index + " took " + vacated + ".";
                } else {
                    message = "Pod " + selected + " is now " + corner + ".";
                }

                podsDirty = true;
                saveCalibration();
                break;
            }
            case "setPidf": {
                PodCal c = cals[selected];
                if ("all".equals(cmd.get("scope"))) {
                    for (PodCal each : cals) {
                        each.kP = doubleArg(cmd, "kp", each.kP);
                        each.kI = doubleArg(cmd, "ki", each.kI);
                        each.kD = doubleArg(cmd, "kd", each.kD);
                        each.kF = doubleArg(cmd, "kf", each.kF);
                    }
                } else {
                    c.kP = doubleArg(cmd, "kp", c.kP);
                    c.kI = doubleArg(cmd, "ki", c.kI);
                    c.kD = doubleArg(cmd, "kd", c.kD);
                    c.kF = doubleArg(cmd, "kf", c.kF);
                    c.servoCaching = doubleArg(cmd, "cache", c.servoCaching);
                }
                podsDirty = true;
                saveCalibration();
                message = "PIDF updated.";
                break;
            }
            case "setRange":
                cals[selected].analogMin = doubleArg(cmd, "min", cals[selected].analogMin);
                cals[selected].analogMax = doubleArg(cmd, "max", cals[selected].analogMax);
                podsDirty = true;
                saveCalibration();
                break;
            case "pidStep":
                startPidStep(doubleArg(cmd, "deg", 90), false);
                break;
            case "pidStepAll":
                startPidStep(doubleArg(cmd, "deg", 90), true);
                break;
            case "autoTune":
                startAutoTune();
                break;
            case "pidHold":
                pidHolding = false;
                allStop();
                message = "Pod released.";
                break;
            case "setXLock":
                xLock = cmd.get("value") == null ? !xLock : Boolean.parseBoolean(cmd.get("value"));
                podsDirty = true;
                message = xLock
                        ? "Zero input locks the pods into an X."
                        : "Zero input holds pod headings (easier to read while tuning).";
                break;
            case "drive":
                driveForward = doubleArg(cmd, "f", 0);
                driveStrafe = doubleArg(cmd, "s", 0);
                driveTurn = doubleArg(cmd, "t", 0);
                lastDriveCmdMs = System.currentTimeMillis();
                mode = Mode.DRIVE;
                message = "Drive test active.";
                break;
            case "export":
                exportText = SwerveExport.generate(orderedForExport());
                message = "Constants generated.";
                break;
            case "save":
                saveCalibration();
                message = "Calibration saved to " + CAL_FILE.getName();
                break;
            case "reload":
                loadCalibration();
                acquireHardware();
                podsDirty = true;
                message = "Calibration reloaded.";
                break;
            default:
                break;
        }
    }

    /** Pedro's factory order: leftFront, rightFront, leftBack, rightBack. */
    private PodCal[] orderedForExport() {
        PodCal lf = findByLabel("LF");
        PodCal rf = findByLabel("RF");
        PodCal lb = findByLabel("LB");
        PodCal rb = findByLabel("RB");
        return new PodCal[] {
                lf != null ? lf : cals[2],
                rf != null ? rf : cals[1],
                lb != null ? lb : cals[3],
                rb != null ? rb : cals[0]
        };
    }

    private PodCal findByLabel(String label) {
        for (PodCal c : cals) {
            if (label.equalsIgnoreCase(c.label)) {
                return c;
            }
        }
        return null;
    }

    private static int intArg(Map<String, String> cmd, String key, int fallback) {
        try {
            String v = cmd.get(key);
            return v == null ? fallback : Integer.parseInt(v);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static double doubleArg(Map<String, String> cmd, String key, double fallback) {
        try {
            String v = cmd.get(key);
            return v == null ? fallback : Double.parseDouble(v);
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    // ---------------------------------------------------------------- gamepad fallback

    private void handleGamepad() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !prevDpadUp) {
            selected = (selected + 1) % POD_COUNT;
        }
        if (down && !prevDpadDown) {
            selected = (selected + POD_COUNT - 1) % POD_COUNT;
        }
        prevDpadUp = up;
        prevDpadDown = down;

        // Zeroing is safe before START because it only reads encoders; motion is not.
        boolean canMove = started;

        if (gamepad1.a && !prevA && canMove) {
            setMode(Mode.MOTOR_PULSE);
        }
        prevA = gamepad1.a;

        if (gamepad1.b && !prevB && canMove) {
            setMode(Mode.WIRE_SCAN);
        }
        prevB = gamepad1.b;

        if (gamepad1.x && !prevX) {
            captureZeros(false);
        }
        prevX = gamepad1.x;

        if (gamepad1.y && !prevY && canMove) {
            setMode(Mode.ENC_SWEEP);
        }
        prevY = gamepad1.y;

        if (gamepad1.start && !prevStart) {
            allStop();
            pidHolding = false;
            setMode(Mode.IDLE);
        }
        prevStart = gamepad1.start;

        // Bumpers jog the selected pod, but must not hijack an automatic routine mid-run.
        boolean scanning = mode == Mode.WIRE_SCAN || mode == Mode.ENC_SWEEP;
        if ((gamepad1.left_bumper || gamepad1.right_bumper) && canMove && !scanning) {
            beginJog(gamepad1.right_bumper ? NUDGE_POWER : -NUDGE_POWER,
                    NUDGE_SECONDS, "Jogging pod " + selected + ".");
        }
    }

    // ---------------------------------------------------------------- persistence

    private void saveCalibration() {
        FileWriter w = null;
        try {
            w = new FileWriter(CAL_FILE, false);
            w.write("# Swerve bring-up calibration. Generated by SwerveBringUp.\n");
            for (PodCal c : cals) {
                w.write(c.serialize());
                w.write("\n");
            }
        } catch (IOException e) {
            message = "Could not save calibration: " + e.getMessage();
        } finally {
            closeQuietly(w);
        }
    }

    private void loadCalibration() {
        if (!CAL_FILE.exists()) {
            return;
        }
        BufferedReader r = null;
        try {
            r = new BufferedReader(new FileReader(CAL_FILE));
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                int idx;
                try {
                    idx = Integer.parseInt(line.substring(0, line.indexOf('|')));
                } catch (RuntimeException e) {
                    continue;
                }
                if (idx >= 0 && idx < POD_COUNT) {
                    cals[idx].applySerialized(line);
                }
            }
        } catch (IOException e) {
            // Not added to hwErrors: acquireHardware() clears that list right after this runs.
            message = "Could not read saved calibration: " + e.getMessage();
        } finally {
            closeQuietly(r);
        }
    }

    private static void closeQuietly(java.io.Closeable c) {
        if (c != null) {
            try {
                c.close();
            } catch (IOException ignored) {
                // nothing useful to do during bring-up
            }
        }
    }

    // ---------------------------------------------------------------- commanded state

    /**
     * Reproduces the pod vectors {@code Swerve.arcadeDrive} builds, so the visualizer can draw
     * where each wheel is being told to point alongside where it actually is.
     *
     * <p>Pedro constructs these as {@code Vector(magnitude, theta)}, which flips theta by pi for a
     * negative magnitude. Building them from cartesian components here is equivalent, and lets
     * atan2 resolve direction directly.
     */
    private void computeTargets() {
        for (int i = 0; i < POD_COUNT; i++) {
            targetTheta[i] = Double.NaN;
            targetPower[i] = 0;
        }

        if (mode == Mode.PID && pidHolding) {
            targetTheta[selected] = pidTargetRad;
            return;
        }
        if (mode != Mode.DRIVE) {
            return;
        }

        double forward = driveForward;
        double strafe = -driveStrafe; // arcadeDrive negates strafe before building the vector
        double rotation = driveTurn;

        double transMag = Math.min(1.0, Math.hypot(strafe, forward));
        double transTheta = Math.atan2(forward, strafe);
        boolean zeroTrans = transMag < SWERVE_EPSILON;
        boolean zeroRot = Math.abs(rotation) < SWERVE_EPSILON;

        if (zeroTrans && zeroRot) {
            if (!xLock) {
                // Pods just hold their heading, so there is nothing being commanded to show.
                return;
            }
            // X_LOCK points each pod along its own radius, which is what draws the X.
            for (int i = 0; i < POD_COUNT; i++) {
                targetTheta[i] = Math.atan2(cals[i].podX, -cals[i].podY);
                targetPower[i] = 0;
            }
            return;
        }

        double rotScalar = zeroRot ? 0 : rotation;
        double[] px = new double[POD_COUNT];
        double[] py = new double[POD_COUNT];
        double maxMag = SWERVE_MAX_POWER;

        for (int i = 0; i < POD_COUNT; i++) {
            double tx = zeroTrans ? 0 : transMag * Math.cos(transTheta);
            double ty = zeroTrans ? 0 : transMag * Math.sin(transTheta);

            // Perpendicular to the pod's radius: atan2(x, -y) then rotated a quarter turn.
            double rotTheta = Math.atan2(cals[i].podX, -cals[i].podY) + Math.PI / 2.0;
            px[i] = tx + rotScalar * Math.cos(rotTheta);
            py[i] = ty + rotScalar * Math.sin(rotTheta);
            maxMag = Math.max(maxMag, Math.hypot(px[i], py[i]));
        }

        for (int i = 0; i < POD_COUNT; i++) {
            targetTheta[i] = Math.atan2(py[i], px[i]);
            targetPower[i] = Math.hypot(px[i], py[i]) * SWERVE_MAX_POWER / maxMag;
        }
    }

    // ---------------------------------------------------------------- state publishing

    private void publish() {
        computeTargets();
        StringBuilder sb = new StringBuilder(2048);
        sb.append("{");
        sb.append("\"live\":true");
        sb.append(",\"mode\":\"").append(mode.name()).append('"');
        sb.append(",\"selected\":").append(selected);
        sb.append(",\"loopHz\":").append(fmt(loopHz));
        sb.append(",\"voltage\":").append(fmt(batteryVolts()));
        sb.append(",\"busy\":").append(routineActive);
        sb.append(",\"started\":").append(started);
        sb.append(",\"xLock\":").append(xLock);
        sb.append(",\"message\":\"").append(esc(message)).append('"');
        sb.append(",\"phase\":").append(fmt(phaseTimer.seconds()));

        sb.append(",\"pods\":[");
        for (int i = 0; i < POD_COUNT; i++) {
            if (i > 0) {
                sb.append(',');
            }
            appendPod(sb, i);
        }
        sb.append(']');

        sb.append(",\"errors\":[");
        for (int i = 0; i < hwErrors.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('"').append(esc(hwErrors.get(i))).append('"');
        }
        sb.append(']');

        sb.append(",\"notes\":[");
        for (int i = 0; i < scanNotes.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('"').append(esc(scanNotes.get(i))).append('"');
        }
        sb.append(']');

        // Emitted oldest-first; the ring buffer's start walks forward once it has wrapped.
        int start = (traceCount < TRACE_LEN) ? 0 : traceHead;
        sb.append(",\"trace\":{\"pod\":").append(tracePod);
        appendTraceSeries(sb, ",\"t\":[", traceT, start);
        appendTraceSeries(sb, "],\"tgt\":[", traceTarget, start);
        appendTraceSeries(sb, "],\"act\":[", traceActual, start);
        sb.append("]}");

        sb.append(",\"tune\":{\"running\":").append(tuner.isRunning())
                .append(",\"pod\":").append(tuner.podIndex())
                .append(",\"status\":\"").append(esc(tuner.status()))
                .append("\",\"log\":[");
        List<String> tuneLog = tuner.log();
        int logFrom = Math.max(0, tuneLog.size() - 14);
        for (int i = logFrom; i < tuneLog.size(); i++) {
            if (i > logFrom) {
                sb.append(',');
            }
            sb.append('"').append(esc(tuneLog.get(i))).append('"');
        }
        sb.append("]}");

        sb.append(",\"cmd\":{\"f\":").append(fmt(driveForward))
                .append(",\"s\":").append(fmt(driveStrafe))
                .append(",\"t\":").append(fmt(driveTurn))
                .append('}');

        sb.append(",\"export\":\"").append(esc(exportText)).append('"');
        sb.append('}');

        SwerveBench.INSTANCE.publish(sb.toString());
    }

    private void appendPod(StringBuilder sb, int i) {
        PodCal c = cals[i];
        double v = volts[i];
        boolean readable = !Double.isNaN(v);
        sb.append("{\"i\":").append(i);
        sb.append(",\"label\":\"").append(esc(c.label)).append('"');
        sb.append(",\"motor\":\"").append(esc(c.motorName)).append('"');
        sb.append(",\"servo\":\"").append(esc(c.servoName)).append('"');
        sb.append(",\"enc\":\"").append(esc(c.encoderName)).append('"');
        sb.append(",\"hasMotor\":").append(motors[i] != null);
        sb.append(",\"hasServo\":").append(servos[i] != null);
        sb.append(",\"hasEnc\":").append(encoders[i] != null);
        sb.append(",\"volts\":").append(readable ? fmt(v) : "null");
        sb.append(",\"rawDeg\":").append(readable ? fmt(Math.toDegrees(c.rawAngleRad(v))) : "null");
        sb.append(",\"zeroDeg\":")
                .append(readable ? fmt(Math.toDegrees(c.zeroedAngleRad(v))) : "null");

        // Visualizer geometry: where the wheel actually points, and where it is being told to.
        sb.append(",\"wheelDeg\":").append(readable
                ? fmt(Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(v))))
                : "null");
        sb.append(",\"tgtDeg\":").append(Double.isNaN(targetTheta[i])
                ? "null"
                : fmt(Math.toDegrees(normalizeTwoPi(targetTheta[i]))));
        sb.append(",\"cmdPower\":").append(fmt(targetPower[i]));
        sb.append(",\"podX\":").append(fmt(c.podX));
        sb.append(",\"podY\":").append(fmt(c.podY));
        sb.append(",\"offsetDeg\":").append(fmt(Math.toDegrees(c.angleOffsetRad)));
        sb.append(",\"minV\":").append(fmt(c.analogMin));
        sb.append(",\"maxV\":").append(fmt(c.analogMax));
        sb.append(",\"encRev\":").append(c.encoderReversed);
        sb.append(",\"drvRev\":").append(c.driveReversed());
        sb.append(",\"srvRev\":").append(c.servoReversed());
        sb.append(",\"kp\":").append(fmt(c.kP));
        sb.append(",\"ki\":").append(fmt(c.kI));
        sb.append(",\"kd\":").append(fmt(c.kD));
        sb.append(",\"kf\":").append(fmt(c.kF));
        sb.append(",\"cache\":").append(fmt(c.servoCaching));
        sb.append(",\"discovered\":").append(c.discoveredEncoderIndex);
        sb.append(",\"servoPower\":")
                .append(servos[i] != null ? fmt(servos[i].getPower()) : "0");
        sb.append(",\"drivePower\":")
                .append(motors[i] != null ? fmt(motors[i].getPower()) : "0");
        sb.append('}');
    }

    private void appendTraceSeries(StringBuilder sb, String prefix, double[] values, int start) {
        sb.append(prefix);
        for (int i = 0; i < traceCount; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append(fmt(values[(start + i) % TRACE_LEN]));
        }
    }

    private static double normalizeTwoPi(double radians) {
        double a = radians % (2 * Math.PI);
        return a < 0 ? a + 2 * Math.PI : a;
    }

    private double batteryVolts() {
        if (voltageSensor == null) {
            return 0;
        }
        try {
            return voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            return 0;
        }
    }

    private static String fmt(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "0";
        }
        return String.format(Locale.US, "%.4f", v);
    }

    private static String esc(String s) {
        if (s == null) {
            return "";
        }
        StringBuilder out = new StringBuilder(s.length() + 16);
        for (int i = 0; i < s.length(); i++) {
            char ch = s.charAt(i);
            switch (ch) {
                case '"':
                    out.append("\\\"");
                    break;
                case '\\':
                    out.append("\\\\");
                    break;
                case '\n':
                    out.append("\\n");
                    break;
                case '\r':
                    out.append("\\r");
                    break;
                case '\t':
                    out.append("\\t");
                    break;
                default:
                    if (ch < 0x20) {
                        out.append(String.format(Locale.US, "\\u%04x", (int) ch));
                    } else {
                        out.append(ch);
                    }
                    break;
            }
        }
        return out.toString();
    }

    // ---------------------------------------------------------------- telemetry

    private void pushTelemetry() {
        telemetry.addData("Dashboard", "http://192.168.43.1:8080/swerve");
        telemetry.addData("Mode", "%s%s", mode, routineActive ? " (running)" : "");
        telemetry.addData("Selected Pod", "%d (%s)", selected, cals[selected].label);
        telemetry.addData("Status", message);
        telemetry.addData("Loop Hz", "%.0f", loopHz);
        telemetry.addData("Battery", "%.2f V", batteryVolts());

        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            String marker = (i == selected) ? ">" : " ";
            if (Double.isNaN(volts[i])) {
                telemetry.addData(marker + " pod" + i + " " + c.label, "encoder unreadable");
            } else {
                telemetry.addData(marker + " pod" + i + " " + c.label,
                        "%.3fV raw %.1f zero %.1f",
                        volts[i],
                        Math.toDegrees(c.rawAngleRad(volts[i])),
                        Math.toDegrees(c.zeroedAngleRad(volts[i])));
            }
        }

        if (!hwErrors.isEmpty()) {
            telemetry.addData("Hardware Errors", hwErrors.size());
            for (int i = 0; i < Math.min(hwErrors.size(), 4); i++) {
                telemetry.addData("  err" + i, hwErrors.get(i));
            }
        }
        for (int i = 0; i < Math.min(scanNotes.size(), 4); i++) {
            telemetry.addData("  scan" + i, scanNotes.get(i));
        }

        telemetry.addLine("Gamepad: dpad=select  A=pulse motor  B=wire scan  "
                + "X=zero pod  Y=sweep  bumpers=nudge  START=stop");
        telemetry.update();
    }
}
