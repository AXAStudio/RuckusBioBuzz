package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        /** Robot heading step response: displace open-loop, then close the loop and record. */
        HEADING,
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
                || "rawServo".equals(action)
                || "autoTune".equals(action)
                || "headingStep".equals(action)
                || "headingGoto".equals(action)
                || "drive".equals(action);
    }

    // ---------------------------------------------------------------- state

    private final PodCal[] cals = new PodCal[POD_COUNT];
    private final DcMotorEx[] motors = new DcMotorEx[POD_COUNT];
    private final CRServo[] servos = new CRServo[POD_COUNT];
    private final AnalogInput[] encoders = new AnalogInput[POD_COUNT];
    private final double[] volts = new double[POD_COUNT];

    /**
     * Turn-servo power actually commanded this loop, per pod.
     *
     * <p>Tracked here rather than read back from the {@code CRServo}, because during closed-loop
     * modes the pod writes through its own device object and {@code CoaxialPod}'s output caching
     * means the controller's output and the servo's held power are different numbers. The recorder
     * wants the held one.
     */
    private final double[] servoCmd = new double[POD_COUNT];

    /** Loop-rate recording of every pod, pulled as CSV from {@code /swerve/rec.csv}. */
    private final PodRecorder recorder = new PodRecorder();

    // Scratch rows reused each loop so recording allocates nothing in the control path.
    private final double[] recWheel = new double[POD_COUNT];
    private final double[] recTarget = new double[POD_COUNT];
    private final double[] recError = new double[POD_COUNT];
    private final boolean[] recFlipped = new boolean[POD_COUNT];

    /**
     * Which pods went through {@code CoaxialPod.move()} this loop.
     *
     * <p>Only those pods have a meaningful error and turn power to read back; for the rest the
     * pod's cached values are left over from whenever it was last driven, and recording them would
     * put stale numbers in the trace that look like real measurements.
     */
    private final boolean[] podMoved = new boolean[POD_COUNT];

    /**
     * The servo PWM configuration actually in force, read back from the hardware.
     *
     * <p>Nothing in this codebase ever called {@code setPwmRange}, so these are whatever the SDK
     * defaults to - and the frame period in particular is a hard bound on actuation bandwidth that
     * no amount of loop rate can improve. Reported rather than assumed.
     */
    private final double[] pwmLower = new double[POD_COUNT];
    private final double[] pwmUpper = new double[POD_COUNT];
    private final double[] pwmFrame = new double[POD_COUNT];

    /**
     * Servo rail current, milliamps, for the whole hub.
     *
     * <p>{@code LynxModule} exposes GPIO and I2C bus current but not the servo rail; the channel
     * exists in the Lynx protocol and is reached by sending the ADC command directly.
     *
     * <p>Aggregate across all four ports, which is enough: with one servo's PWM enabled and the
     * rest limp, the difference between enabled and disabled is that servo's holding current, and
     * everything else on the rail cancels. Read at the slow sensor cadence, not every loop - it is
     * a bus round trip and a holding current is DC.
     */
    private LynxModule servoRailModule;
    private double servoRailMa;
    private double batteryMa;
    private double totalMa;

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

    /**
     * Smoothed per-stage loop cost in milliseconds.
     *
     * <p>The pod turn loop runs at the OpMode's loop rate, so the loop rate is a control parameter,
     * not a diagnostic curiosity: at 30 Hz the derivative is differencing over 33 ms and the servo
     * is being told something new less often than its own 20 ms PWM frame. Knowing which stage
     * costs what is the difference between fixing that and guessing at it.
     */
    private double msEncoders;
    private double msHeading;
    private double msMode;
    private double msPublish;

    private static double smooth(double previous, long nanos) {
        return 0.9 * previous + 0.1 * (nanos / 1.0e6);
    }

    /**
     * How often state is serialised and telemetry pushed.
     *
     * <p>Both were running every loop and together cost 8-13 ms of a 20-27 ms loop, which is to say
     * the dashboard was consuming half the control bandwidth. 20 Hz is far faster than anyone reads
     * a web page, well inside {@code SwerveBench}'s 1500 ms liveness window, and irrelevant to the
     * recorder, which samples every loop regardless.
     */
    private static final double PUBLISH_INTERVAL_S = 0.05;

    /**
     * How often the Pinpoint and the raw analog channels are read when nothing needs them.
     *
     * <p>The Pinpoint costs 5.5 ms of I2C per read and pod rotation does not use heading at all;
     * the four {@code channels[]} reads exist only for the wiring scan. Both still refresh slowly
     * so the dashboard shows live numbers instead of frozen ones.
     */
    private static final double IDLE_SENSOR_INTERVAL_S = 0.2;

    private final ElapsedTime publishTimer = new ElapsedTime();
    private final ElapsedTime idleSensorTimer = new ElapsedTime();

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

    // ---- robot heading (Pinpoint IMU; works with no odometry pods attached) ----
    private GoBildaPinpointDriver pinpoint;
    private double headingRad;
    private boolean headingOk;

    /** Heading PIDF under test. Units match FollowerConstants.headingPIDFCoefficients (radians). */
    private double headingKp = 1.75;
    private double headingKd = 0.003;
    private double headingKf = 0.0;

    private double headingTargetRad;
    private boolean headingClosedLoop;
    private double headingOpenLoopPower;

    /**
     * Heading controller, driven exactly as Pedro drives it.
     *
     * <p>{@code ErrorCalculator} builds a SIGNED error as
     * {@code getTurnDirection(current, target) * getSmallestAngleDifference(current, target)}, and
     * {@code VectorCalculator.getHeadingVector} feeds the turn direction in as the feed-forward
     * input before clamping the output to max power. Reproducing that exactly is what makes gains
     * tuned here transfer to {@code FollowerConstants.headingPIDFCoefficients}.
     */
    private final PIDFController headingPidf = new PIDFController(
            new com.pedropathing.control.PIDFCoefficientSupplier() {
                @Override
                public PIDFCoefficients get(double error) {
                    // Supplied per call, because PIDFController.run() re-reads the supplier and
                    // discards anything set with setCoefficients().
                    return new PIDFCoefficients(headingKp, 0, headingKd, headingKf);
                }
            });

    /** Right stick sweeps a heading SETPOINT rather than commanding rotation power directly. */
    private boolean headingHold = true;
    private boolean headingStickActive;

    /** Frozen-heading watchdog: a dead sensor plus a heading controller means a full-power spin. */
    private double headingLastSeen;
    private double headingStuckSeconds;
    private static final double HEADING_STUCK_LIMIT_S = 1.2;
    /**
     * Setpoint sweep rate at full stick, matched to what the robot can actually do.
     *
     * <p>Measured on this drivetrain: 0.3 stick -> 72 deg/s, 0.6 -> 165, 1.0 -> 328. Sweeping the
     * setpoint slower than the robot can turn wastes the difference - the controller only ever
     * needs a fraction of full power to keep up, so a held stick feels weak. Pedro's own teleop
     * maps the stick straight to rotation power, so full stick must mean full authority here too.
     */
    private static final double HEADING_STICK_RATE = 5.5;   // rad/s, ~315 deg/s

    /**
     * How far the setpoint may lead the measured heading.
     *
     * <p>Without this the setpoint outruns the robot, the error grows past 180 degrees, and
     * {@code getTurnDirection} flips to the now-shorter way round - so the robot reverses, stalls,
     * and surges. Capping the lead keeps the error inside the controller's linear region and makes
     * a held stick produce smooth continuous rotation.
     */
    private static final double HEADING_MAX_LEAD = Math.toRadians(60);

    /** How long the current jog should run before stopping itself. */
    private double jogSeconds;

    private String exportText = "";

    private final PodAutoTuner tuner = new PodAutoTuner();
    private final ElapsedTime autoTuneTimer = new ElapsedTime();
    private final ElapsedTime headingStickTimer = new ElapsedTime();

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
            // The servos are all on the Control Hub itself, so the parent module owns the rail.
            if (servoRailModule == null || module.isParent()) {
                servoRailModule = module;
            }
        }

        acquireHardware();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.update();
        } catch (RuntimeException e) {
            pinpoint = null;
            hwErrors.add("No \"pinpoint\" device; heading tuning unavailable.");
        }

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (RuntimeException e) {
            hwErrors.add("No voltage sensor found.");
        }

        SwerveBench.INSTANCE.clearCommands();
        SwerveBench.INSTANCE.setRecorder(recorder);
        computeTargets();
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

        boolean refreshIdleSensors = idleSensorTimer.seconds() >= IDLE_SENSOR_INTERVAL_S;
        if (refreshIdleSensors) {
            idleSensorTimer.reset();
        }

        long mark = System.nanoTime();
        readEncoders(refreshIdleSensors);
        msEncoders = smooth(msEncoders, System.nanoTime() - mark);

        mark = System.nanoTime();
        readHeading(refreshIdleSensors);
        if (refreshIdleSensors) {
            readServoRailCurrent();
        }
        msHeading = smooth(msHeading, System.nanoTime() - mark);

        handleGamepad();

        mark = System.nanoTime();
        runMode();
        msMode = smooth(msMode, System.nanoTime() - mark);
        // Before record() so the trace carries this loop's commanded angles, not the previous
        // loop's; publish() then reuses what was computed here.
        computeTargets();
        record(dt);

        if (publishTimer.seconds() >= PUBLISH_INTERVAL_S) {
            publishTimer.reset();
            mark = System.nanoTime();
            publish();
            pushTelemetry();
            msPublish = smooth(msPublish, System.nanoTime() - mark);
        }
    }

    /**
     * Appends one loop's worth of every pod's state to {@link #recorder}.
     *
     * <p>Runs after {@link #runMode()} so it captures the outputs this loop actually produced.
     */
    private void record(double dt) {
        if (!recorder.recording()) {
            return;
        }

        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            recWheel[i] = Double.isNaN(volts[i])
                    ? Double.NaN
                    : Math.toDegrees(c.wheelThetaFromEncoder(c.zeroedAngleRad(volts[i])));
            recTarget[i] = Double.isNaN(targetTheta[i])
                    ? Double.NaN
                    : Math.toDegrees(normalizeTwoPi(targetTheta[i]));

            if (podMoved[i] && pods != null) {
                recError[i] = Math.toDegrees(pods[i].getLastErrorRad());
                recFlipped[i] = pods[i].wasLastMoveFlipped();
                servoCmd[i] = pods[i].getLastTurnPower();
            } else {
                recError[i] = Double.NaN;
                recFlipped[i] = false;
            }
        }

        recorder.add(dt, batteryVolts(), loopHz, mode.ordinal(), servoRailMa,
                volts, recWheel, recTarget, recError, servoCmd, recFlipped);
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
                setServo(i, 0);
            } catch (RuntimeException e) {
                servos[i] = null;
                hwErrors.add("Missing turn servo \"" + c.servoName + "\" (pod " + i + ").");
            }
            readPwmRange(i);
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

    /** True while heading is actually part of the control law rather than just a readout. */
    private boolean headingInUse() {
        return mode == Mode.HEADING || (mode == Mode.DRIVE && headingHold);
    }

    /** Reads the hub's servo rail current in milliamps, or leaves the last value on failure. */
    private void readServoRailCurrent() {
        if (servoRailModule == null) {
            return;
        }
        try {
            LynxGetADCCommand cmd = new LynxGetADCCommand(servoRailModule,
                    LynxGetADCCommand.Channel.SERVO_CURRENT,
                    LynxGetADCCommand.Mode.ENGINEERING);
            servoRailMa = cmd.sendReceive().getValue();
            batteryMa = new LynxGetADCCommand(servoRailModule,
                    LynxGetADCCommand.Channel.BATTERY_CURRENT,
                    LynxGetADCCommand.Mode.ENGINEERING).sendReceive().getValue();
            totalMa = servoRailModule.getCurrent(CurrentUnit.MILLIAMPS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (RuntimeException | LynxNackException e) {
            // A dropped ADC read is not worth failing a routine over; the stale value is obvious
            // in the trace because it repeats.
        }
    }

    /**
     * Enables or disables a servo's PWM output.
     *
     * <p>Disabling is how a holding current gets measured: the rail reading with one servo enabled
     * and holding, minus the reading with it disabled, is that servo's contribution with every
     * other draw on the rail cancelling out.
     */
    private void applyPwmEnable(int i, boolean enabled) {
        if (!(servos[i] instanceof PwmControl)) {
            message = cals[i].servoName + " does not support PWM control.";
            return;
        }
        if (enabled) {
            ((PwmControl) servos[i]).setPwmEnable();
        } else {
            setServo(i, 0);
            ((PwmControl) servos[i]).setPwmDisable();
        }
        message = cals[i].servoName + " PWM " + (enabled ? "enabled" : "disabled");
    }

    /** Reads back the PWM pulse limits and frame period the hardware is actually using. */
    private void readPwmRange(int i) {
        pwmLower[i] = 0;
        pwmUpper[i] = 0;
        pwmFrame[i] = 0;
        if (!(servos[i] instanceof PwmControl)) {
            return;
        }
        try {
            PwmControl.PwmRange r = ((PwmControl) servos[i]).getPwmRange();
            pwmLower[i] = r.usPulseLower;
            pwmUpper[i] = r.usPulseUpper;
            pwmFrame[i] = r.usFrame;
        } catch (RuntimeException e) {
            hwErrors.add("Could not read PWM range for " + cals[i].servoName + ": " + e.getMessage());
        }
    }

    /**
     * Widens (or restores) the servo PWM pulse range.
     *
     * <p>The range is a property of the controller port, not of the Java object, so this applies to
     * the {@link CoaxialPod} writing to the same port as well.
     *
     * <p>Changing it rescales every gain: the same commanded power becomes a different pulse width
     * and therefore a different speed. Re-tune after, not before.
     */
    private void applyPwmRange(double lower, double upper, double frame, boolean allPods) {
        int from = allPods ? 0 : selected;
        int to = allPods ? POD_COUNT : selected + 1;
        for (int i = from; i < to; i++) {
            if (!(servos[i] instanceof PwmControl)) {
                message = cals[i].servoName + " does not support PWM control.";
                continue;
            }
            try {
                setServo(i, 0);
                ((PwmControl) servos[i]).setPwmRange(
                        new PwmControl.PwmRange(lower, upper, frame));
                readPwmRange(i);
            } catch (RuntimeException e) {
                message = "setPwmRange failed on " + cals[i].servoName + ": " + e.getMessage();
                return;
            }
        }
        message = String.format(Locale.US, "PWM range %.0f-%.0f us, frame %.0f us.",
                lower, upper, frame);
    }

    /** Heading comes off the Pinpoint's IMU, so it is valid with no odometry pods attached. */
    private void readHeading(boolean refreshIdleSensors) {
        if (pinpoint == null) {
            headingOk = false;
            return;
        }
        if (!headingInUse() && !refreshIdleSensors) {
            return;
        }
        try {
            pinpoint.update();
            double h = pinpoint.getHeading(AngleUnit.RADIANS);
            headingOk = !Double.isNaN(h);
            if (headingOk) {
                headingRad = h;
            }
        } catch (RuntimeException e) {
            headingOk = false;
        }
    }

    /**
     * Reads the four pod encoders every loop, and the four raw channels only when something wants
     * them.
     *
     * <p>{@code channels[]} duplicates {@code encoders[]} through fixed names so a wiring scan
     * stays meaningful after a remap. Outside a scan nothing reads it, and it was costing half of
     * this method's 4.6 ms every loop.
     */
    private void readEncoders(boolean refreshIdleSensors) {
        for (int i = 0; i < POD_COUNT; i++) {
            volts[i] = readVoltage(encoders[i]);
        }
        if (mode == Mode.WIRE_SCAN || refreshIdleSensors) {
            for (int i = 0; i < POD_COUNT; i++) {
                channelVolts[i] = readVoltage(channels[i]);
            }
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
                    // X_LOCK matches competition, but it cannot coexist with heading hold:
                    // arcadeDrive engages the lock whenever |rotation| < epsilon, and the lock
                    // points pods ALONG their radius while rotating points them PERPENDICULAR to
                    // it. Every time the heading PID output dips under epsilon on approach, all
                    // four pods snap 90 degrees, the robot stops rotating, error grows, output
                    // rises and they snap back - which reads as rotation happening in bursts.
                    .zeroPowerBehavior(xLock && !headingHold
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

    /**
     * Applies turn-servo power and records what was applied.
     *
     * <p>Every open-loop servo write in this class goes through here, so {@link #servoCmd} is a
     * faithful log of what the hardware was told rather than a reconstruction.
     */
    /** Drives the bench drivetrain and notes that every pod went through {@code move()}. */
    private void arcade(double forward, double strafe, double rotation) {
        swerve.arcadeDrive(forward, strafe, rotation);
        for (int i = 0; i < POD_COUNT; i++) {
            podMoved[i] = true;
        }
    }

    private void setServo(int i, double power) {
        servoCmd[i] = power;
        if (servos[i] != null) {
            servos[i].setPower(power);
        }
    }

    private void allStop() {
        for (int i = 0; i < POD_COUNT; i++) {
            if (servos[i] != null) {
                setServo(i, 0);
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
        // Re-established each loop by whichever branch actually drives pods through move().
        for (int i = 0; i < POD_COUNT; i++) {
            podMoved[i] = false;
        }

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
            case HEADING:
                runHeadingTune();
                break;
            case DRIVE:
                runDriveMode();
                break;
            case IDLE:
            default:
                // Servos limp so pods can be turned by hand for zeroing.
                for (int i = 0; i < POD_COUNT; i++) {
                    if (servos[i] != null) {
                        setServo(i, 0);
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
                    setServo(i, 0);
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
                setServo(routinePod, 0);
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
            setServo(routinePod, SCAN_SERVO_POWER);
        }
        accumulateScanDeltas(routinePod);

        if (phaseTimer.seconds() >= SCAN_MOVE_S) {
            if (servos[routinePod] != null) {
                setServo(routinePod, 0);
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
            setServo(p, SCAN_SERVO_POWER);
        }
        if (!Double.isNaN(volts[p])) {
            sweepMin[p] = Math.min(sweepMin[p], volts[p]);
            sweepMax[p] = Math.max(sweepMax[p], volts[p]);
        }

        if (phaseTimer.seconds() >= SWEEP_SECONDS) {
            if (servos[p] != null) {
                setServo(p, 0);
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
        setServo(selected, power);
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
                setServo(i, 0);
            }
        }

        if (!pidHolding) {
            return;
        }

        if (pidAllPods) {
            for (int i = 0; i < POD_COUNT; i++) {
                pods[i].move(pidTargetRad, 0.0, false);
                podMoved[i] = true;
            }
        } else {
            pods[selected].move(pidTargetRad, 0.0, false);
            podMoved[selected] = true;
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
                    setServo(i, 0);
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
                    setServo(pod, tuner.rawServoPower);
                }
                break;
            case PID_HOLD:
                pods[pod].move(tuner.targetWheelRad, 0.0, false);
                podMoved[pod] = true;
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


    /**
     * Rotation power to drive the robot's heading to {@link #headingTargetRad}, computed the way
     * {@code VectorCalculator.getHeadingVector} computes it.
     */
    private double headingCorrection() {
        double direction = MathFunctions.getTurnDirection(headingRad, headingTargetRad);
        double error = direction
                * MathFunctions.getSmallestAngleDifference(headingRad, headingTargetRad);

        headingPidf.updateFeedForwardInput(direction);
        headingPidf.updateError(error);
        return MathFunctions.clamp(headingPidf.run(), -SWERVE_MAX_POWER, SWERVE_MAX_POWER);
    }

    // ---------------------------------------------------------------- heading tuning

    /** Open-loop displacement time before the loop is closed. */
    private static final double HEADING_OPEN_LOOP_S = 0.55;

    /**
     * Robot heading step response.
     *
     * <p>The robot is first rotated open-loop away from the captured target, with the controller
     * off, then the loop is closed and the recovery recorded. Displacing under control would only
     * ever show the controller chasing itself; letting it start from a genuine offset is what
     * exposes the gain.
     */
    private void runHeadingTune() {
        ensurePods();
        if (swerve == null || !headingOk) {
            message = swerve == null
                    ? "Cannot build drivetrain: " + podBuildError
                    : "No heading from the Pinpoint; cannot tune heading.";
            setMode(Mode.IDLE);
            return;
        }

        double error = MathFunctions.normalizeAngleSigned(headingTargetRad - headingRad);

        if (!headingClosedLoop) {
            // Open loop: spin away from the target so the closed-loop phase starts displaced.
            if (phaseTimer.seconds() < HEADING_OPEN_LOOP_S) {
                arcade(0, 0, headingOpenLoopPower);
                message = String.format(Locale.US, "Displacing open-loop (%.0f deg so far)",
                        Math.toDegrees(Math.abs(error)));
                return;
            }
            // Close the loop and start the recording from here.
            headingClosedLoop = true;
            headingPidf.reset();
            clearTrace(selected);
            phaseTimer.reset();
        }

        arcade(0, 0, headingCorrection());

        recordHeadingTrace();

        if (phaseTimer.seconds() > 3.0) {
            arcade(0, 0, 0);
            message = String.format(Locale.US,
                    "Heading step done. Final error %.1f deg.", Math.toDegrees(error));
            setMode(Mode.IDLE);
        }
    }

    /** Reuses the pod trace buffer to graph robot heading against its target. */
    private void recordHeadingTrace() {
        double actualDeg = Math.toDegrees(headingRad);
        double targetDeg = Math.toDegrees(headingTargetRad);
        double delta = targetDeg - actualDeg;
        if (delta > 180) {
            targetDeg -= 360;
        } else if (delta < -180) {
            targetDeg += 360;
        }

        traceT[traceHead] = phaseTimer.seconds();
        traceTarget[traceHead] = targetDeg;
        traceActual[traceHead] = actualDeg;
        traceHead = (traceHead + 1) % TRACE_LEN;
        if (traceCount < TRACE_LEN) {
            traceCount++;
        }
    }

    private void startHeadingStep(double displaceDeg) {
        ensurePods();
        if (swerve == null) {
            message = "Cannot build drivetrain: " + podBuildError;
            return;
        }
        if (!headingOk) {
            message = "No heading from the Pinpoint; cannot tune heading.";
            return;
        }

        headingTargetRad = headingRad;
        headingOpenLoopPower = displaceDeg >= 0 ? 0.35 : -0.35;
        headingClosedLoop = false;
        clearTrace(selected);
        tracePod = -2;   // marks the trace as heading rather than a pod
        allStop();
        phaseTimer.reset();
        autoTuneTimer.reset();
        mode = Mode.HEADING;
        routineActive = true;
        message = "Heading step: displacing open-loop, then closing the loop.";
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

        double turn = driveTurn;

        if (headingHold && headingOk) {
            // The stick sets a heading RATE; the controller holds whatever setpoint it leaves
            // behind. Releasing it therefore locks the current heading instead of coasting, and a
            // shove off-heading is corrected rather than accepted.
            double dt = Math.min(0.25, Math.max(1e-3, headingStickTimer.seconds()));
            headingStickTimer.reset();

            boolean stickActive = Math.abs(driveTurn) > 0.02;

            if (stickActive) {
                headingTargetRad = MathFunctions.normalizeAngle(
                        headingTargetRad + driveTurn * HEADING_STICK_RATE * dt);

                // Never let the setpoint lead further than the controller can chase. Past 180 the
                // error wraps and getTurnDirection flips, which reverses the robot mid-turn.
                double lead = MathFunctions.getTurnDirection(headingRad, headingTargetRad)
                        * MathFunctions.getSmallestAngleDifference(headingRad, headingTargetRad);
                if (lead > HEADING_MAX_LEAD) {
                    headingTargetRad = MathFunctions.normalizeAngle(headingRad + HEADING_MAX_LEAD);
                } else if (lead < -HEADING_MAX_LEAD) {
                    headingTargetRad = MathFunctions.normalizeAngle(headingRad - HEADING_MAX_LEAD);
                }
            } else if (headingStickActive) {
                // Stick just released. Latch the heading we are at rather than unwinding the lead,
                // which would otherwise keep turning for another lead-angle after letting go.
                headingTargetRad = headingRad;
                headingPidf.reset();
            }

            headingStickActive = stickActive;
            turn = headingCorrection();

            // If we are commanding real rotation but the measured heading has not moved at all,
            // the sensor is dead and this loop would happily spin the robot at full power.
            if (Math.abs(turn) > 0.15) {
                if (Math.abs(headingRad - headingLastSeen) < 1e-6) {
                    headingStuckSeconds += dt;
                } else {
                    headingStuckSeconds = 0;
                }
                if (headingStuckSeconds > HEADING_STUCK_LIMIT_S) {
                    headingHold = false;
                    podsDirty = true;
                    driveForward = 0;
                    driveStrafe = 0;
                    driveTurn = 0;
                    headingStuckSeconds = 0;
                    message = "Heading has not changed while turning - sensor looks dead. "
                            + "Heading hold disabled; recalibrate the Pinpoint.";
                    arcade(0, 0, 0);
                    return;
                }
            } else {
                headingStuckSeconds = 0;
            }
            headingLastSeen = headingRad;
        }

        arcade(driveForward, driveStrafe, turn);

        if (headingHold && headingOk) {
            // Heading is the interesting signal while holding, and it is what the circle test and
            // the rotation tests are judged on.
            recordHeadingTrace();
        } else if (!Double.isNaN(targetTheta[selected])) {
            // computeTargets() runs at publish time, so the value here is from the previous loop -
            // close enough for a graph, and it keeps arcadeDrive's kinematics as the single source.
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
                        // Propagated with the rest: the output caching threshold is a control
                        // parameter, not a display preference, and leaving it out of scope=all
                        // meant a swept value silently applied to one pod out of four.
                        each.servoCaching = doubleArg(cmd, "cache", each.servoCaching);
                        each.kS = doubleArg(cmd, "ks", each.kS);
                        each.kSBandDeg = doubleArg(cmd, "ksband", each.kSBandDeg);
                        each.kILimit = doubleArg(cmd, "kilimit", each.kILimit);
                        each.kIBandDeg = doubleArg(cmd, "kiband", each.kIBandDeg);
                        each.kIResetDeg = doubleArg(cmd, "kireset", each.kIResetDeg);
                        each.derivativeOnMeasurement = boolArg(cmd, "dom", each.derivativeOnMeasurement);
                        each.pulsed = boolArg(cmd, "pulsed", each.pulsed);
                        each.pulseBandDeg = doubleArg(cmd, "pband", each.pulseBandDeg);
                        each.pulseTolDeg = doubleArg(cmd, "ptol", each.pulseTolDeg);
                        each.pulsePower = doubleArg(cmd, "ppow", each.pulsePower);
                        each.pulseMs = doubleArg(cmd, "pms", each.pulseMs);
                        each.pulseCoastMs = doubleArg(cmd, "pcoast", each.pulseCoastMs);
                    }
                } else {
                    c.kP = doubleArg(cmd, "kp", c.kP);
                    c.kI = doubleArg(cmd, "ki", c.kI);
                    c.kD = doubleArg(cmd, "kd", c.kD);
                    c.kF = doubleArg(cmd, "kf", c.kF);
                    c.servoCaching = doubleArg(cmd, "cache", c.servoCaching);
                    c.kS = doubleArg(cmd, "ks", c.kS);
                    c.kSBandDeg = doubleArg(cmd, "ksband", c.kSBandDeg);
                    c.kILimit = doubleArg(cmd, "kilimit", c.kILimit);
                    c.kIBandDeg = doubleArg(cmd, "kiband", c.kIBandDeg);
                    c.kIResetDeg = doubleArg(cmd, "kireset", c.kIResetDeg);
                    c.derivativeOnMeasurement = boolArg(cmd, "dom", c.derivativeOnMeasurement);
                    c.pulsed = boolArg(cmd, "pulsed", c.pulsed);
                    c.pulseBandDeg = doubleArg(cmd, "pband", c.pulseBandDeg);
                    c.pulseTolDeg = doubleArg(cmd, "ptol", c.pulseTolDeg);
                    c.pulsePower = doubleArg(cmd, "ppow", c.pulsePower);
                    c.pulseMs = doubleArg(cmd, "pms", c.pulseMs);
                    c.pulseCoastMs = doubleArg(cmd, "pcoast", c.pulseCoastMs);
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
            case "setPwmEnable":
                applyPwmEnable(selected, Boolean.parseBoolean(cmd.get("value")));
                break;
            case "setPwmRange": {
                double lower = doubleArg(cmd, "lower", 600);
                double upper = doubleArg(cmd, "upper", 2400);
                double frame = doubleArg(cmd, "frame", PwmControl.PwmRange.usFrameDefault);
                applyPwmRange(lower, upper, frame, "all".equals(cmd.get("scope")));
                break;
            }
            case "recStart":
                recorder.start(cmd.get("label"));
                message = "Recording run " + recorder.runId() + ".";
                break;
            case "recStop":
                recorder.stop();
                message = "Recording stopped: " + recorder.count() + " samples"
                        + (recorder.overflowed() ? " (BUFFER FULL - run truncated)." : ".");
                break;
            case "rawServo": {
                // Open-loop drive at a known power, which is how breakaway, deadband and max slew
                // rate get measured. Reuses the jog path so the same timeout protects it.
                double pow = MathFunctions.clamp(doubleArg(cmd, "pow", 0), -1.0, 1.0);
                double sec = MathFunctions.clamp(doubleArg(cmd, "sec", 0.5), 0.0, 5.0);
                beginJog(pow, sec, String.format(Locale.US,
                        "Pod %d open loop at %.3f for %.2fs.", selected, pow, sec));
                break;
            }
            case "pidStep":
                startPidStep(doubleArg(cmd, "deg", 90), false);
                break;
            case "pidStepAll":
                startPidStep(doubleArg(cmd, "deg", 90), true);
                break;
            case "autoTune":
                startAutoTune();
                break;
            case "headingStep":
                startHeadingStep(doubleArg(cmd, "deg", 90));
                break;
            case "setHeadingPidf":
                headingKp = doubleArg(cmd, "hkp", headingKp);
                headingKd = doubleArg(cmd, "hkd", headingKd);
                headingKf = doubleArg(cmd, "hkf", headingKf);
                message = String.format(Locale.US, "Heading PIDF: kP=%.4f kD=%.4f kF=%.4f",
                        headingKp, headingKd, headingKf);
                break;
            case "pidHold":
                pidHolding = false;
                allStop();
                message = "Pod released.";
                break;
            case "headingGoto": {
                // Commands a heading change of a chosen size and holds it, so rotations of very
                // different magnitudes can be compared with the same controller.
                if (!headingOk) {
                    message = "No heading from the Pinpoint.";
                    break;
                }
                headingTargetRad = MathFunctions.normalizeAngle(
                        headingRad + Math.toRadians(doubleArg(cmd, "deg", 90)));
                headingPidf.reset();
                headingHold = true;
                podsDirty = true;
                driveForward = 0;
                driveStrafe = 0;
                driveTurn = 0;
                lastDriveCmdMs = System.currentTimeMillis();
                clearTrace(selected);
                tracePod = -2;
                phaseTimer.reset();
                headingStickTimer.reset();
                mode = Mode.DRIVE;
                message = String.format(Locale.US, "Turning %.0f deg and holding.",
                        doubleArg(cmd, "deg", 90));
                break;
            }
            case "recalibrateImu":
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                try {
                    allStop();
                    pinpoint.recalibrateIMU();
                    headingStuckSeconds = 0;
                    message = "Pinpoint IMU recalibrating - keep the robot still for a moment.";
                } catch (RuntimeException e) {
                    message = "Recalibrate failed: " + e.getMessage();
                }
                break;
            case "resetImu":
                if (pinpoint == null) {
                    message = "No pinpoint device.";
                    break;
                }
                try {
                    allStop();
                    pinpoint.resetPosAndIMU();
                    headingStuckSeconds = 0;
                    message = "Pinpoint position and IMU reset - keep the robot still.";
                } catch (RuntimeException e) {
                    message = "Reset failed: " + e.getMessage();
                }
                break;
            case "setHeadingHold":
                headingHold = cmd.get("value") == null
                        ? !headingHold
                        : Boolean.parseBoolean(cmd.get("value"));
                if (headingHold && headingOk) {
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                }
                podsDirty = true;   // zero-power behaviour depends on this
                message = headingHold
                        ? "Right stick steers a heading setpoint; the robot holds it."
                        : "Right stick commands rotation power directly.";
                break;
            case "setXLock":
                xLock = cmd.get("value") == null ? !xLock : Boolean.parseBoolean(cmd.get("value"));
                podsDirty = true;
                message = xLock
                        ? "Zero input locks the pods into an X."
                        : "Zero input holds pod headings (easier to read while tuning).";
                break;
            case "drive":
                if (mode != Mode.DRIVE && headingOk) {
                    // Adopt the heading we are already at, so enabling drive never snaps the robot.
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                    headingStickTimer.reset();
                }
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

    private static boolean boolArg(Map<String, String> cmd, String key, boolean fallback) {
        String v = cmd.get(key);
        return v == null ? fallback : Boolean.parseBoolean(v);
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
            if (pidAllPods) {
                for (int i = 0; i < POD_COUNT; i++) {
                    targetTheta[i] = pidTargetRad;
                }
            } else {
                targetTheta[selected] = pidTargetRad;
            }
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
        StringBuilder sb = new StringBuilder(2048);
        sb.append("{");
        sb.append("\"live\":true");
        sb.append(",\"mode\":\"").append(mode.name()).append('"');
        sb.append(",\"selected\":").append(selected);
        sb.append(",\"loopHz\":").append(fmt(loopHz));
        sb.append(",\"voltage\":").append(fmt(batteryVolts()));
        sb.append(",\"servoMa\":").append(fmt(servoRailMa));
        sb.append(",\"batteryMa\":").append(fmt(batteryMa));
        sb.append(",\"totalMa\":").append(fmt(totalMa));
        sb.append(",\"busy\":").append(routineActive);
        sb.append(",\"started\":").append(started);
        sb.append(",\"xLock\":").append(xLock);
        sb.append(",\"heading\":{\"ok\":").append(headingOk)
                .append(",\"deg\":").append(fmt(Math.toDegrees(headingRad)))
                .append(",\"targetDeg\":").append(fmt(Math.toDegrees(headingTargetRad)))
                .append(",\"kp\":").append(fmt(headingKp))
                .append(",\"kd\":").append(fmt(headingKd))
                .append(",\"kf\":").append(fmt(headingKf))
                .append(",\"closedLoop\":").append(headingClosedLoop)
                .append(",\"hold\":").append(headingHold)
                .append('}');
        sb.append(",\"message\":\"").append(esc(message)).append('"');
        sb.append(",\"phase\":").append(fmt(phaseTimer.seconds()));
        sb.append(",\"timing\":{\"encoders\":").append(fmt(msEncoders))
                .append(",\"heading\":").append(fmt(msHeading))
                .append(",\"mode\":").append(fmt(msMode))
                .append(",\"publish\":").append(fmt(msPublish))
                .append('}');
        sb.append(",\"rec\":{\"recording\":").append(recorder.recording())
                .append(",\"runId\":").append(recorder.runId())
                .append(",\"samples\":").append(recorder.count())
                .append(",\"overflowed\":").append(recorder.overflowed())
                .append(",\"label\":\"").append(esc(recorder.label()))
                .append("\"}");

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
        sb.append(",\"ks\":").append(fmt(c.kS));
        sb.append(",\"ksband\":").append(fmt(c.kSBandDeg));
        sb.append(",\"kilimit\":").append(fmt(c.kILimit));
        sb.append(",\"kiband\":").append(fmt(c.kIBandDeg));
        sb.append(",\"kireset\":").append(fmt(c.kIResetDeg));
        sb.append(",\"dom\":").append(c.derivativeOnMeasurement);
        sb.append(",\"pulsed\":").append(c.pulsed);
        sb.append(",\"pband\":").append(fmt(c.pulseBandDeg));
        sb.append(",\"ptol\":").append(fmt(c.pulseTolDeg));
        sb.append(",\"ppow\":").append(fmt(c.pulsePower));
        sb.append(",\"pms\":").append(fmt(c.pulseMs));
        sb.append(",\"pcoast\":").append(fmt(c.pulseCoastMs));
        sb.append(",\"pwmLo\":").append(fmt(pwmLower[i]));
        sb.append(",\"pwmHi\":").append(fmt(pwmUpper[i]));
        sb.append(",\"pwmFrame\":").append(fmt(pwmFrame[i]));
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
