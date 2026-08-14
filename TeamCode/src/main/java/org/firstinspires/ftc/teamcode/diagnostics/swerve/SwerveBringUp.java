package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.pedropathing.ftc.drivetrains.SwervePod;
import org.firstinspires.ftc.teamcode.pedroPathing.PositionalPod;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrainConstants;
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
import com.qualcomm.robotcore.hardware.Servo;
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
        /** Guarded walk to a servo position, for measuring a positional pod's endpoints. */
        CAL_POS,
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
                || "drive".equals(action)
                || "calGoto".equals(action)
                || "calHome".equals(action)
                // Drives the servo to an endpoint - physical motion, same as calGoto.
                || "calPositional".equals(action);
    }

    // ---------------------------------------------------------------- state

    private final PodCal[] cals = new PodCal[POD_COUNT];
    private final DcMotorEx[] motors = new DcMotorEx[POD_COUNT];
    private final CRServo[] servos = new CRServo[POD_COUNT];

    /**
     * The same ports, when configured as positional servos instead.
     *
     * <p>The SDK builds a different device class per port type, so a port declared {@code Servo}
     * cannot be fetched as a {@code CRServo} and vice versa - the get simply throws. During the
     * positional A/B one port is Servo and three are CR, so both arrays are populated and every
     * site uses whichever is non-null. Losing this would cost PWM enable/disable on exactly the
     * pod under test, which is what the holding-current measurement toggles.
     */
    private final Servo[] posServos = new Servo[POD_COUNT];
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

    /**
     * Drive-motor power actually commanded this loop, per pod, for the same reason as
     * {@link #servoCmd}: {@code DcMotorEx.getPower()} is a live Lynx transaction, not a cached
     * field, and the dashboard was paying eight of those per publish just to display two numbers.
     */
    private final double[] motorCmd = new double[POD_COUNT];

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

    /** Worst-case clamp margin from the last positional coverage proof, degrees. */
    private double positionalCoverageDeg = Double.NaN;

    // ---- guarded endpoint calibration ----
    /**
     * The endpoints have to be found by driving to them, which is the one time an
     * uncalibrated positional pod moves. The pod's mechanical range may be smaller than the
     * servo's programmed travel, so this walks there in small steps and stops the moment the
     * encoder stops following - the difference between measuring a limit and grinding into one.
     */
    private static final double CAL_STEP = 0.01;
    private static final double CAL_DWELL_S = 0.15;
    private static final double CAL_MIN_MOVE_DEG = 1.0;

    /**
     * Dwells at an unchanged command before calling it a mechanical limit.
     *
     * <p>calHome measured a genuine 0.38 s stiction stall mid-travel, so a couple of dwells is far
     * too eager - it would abort on ordinary friction and never reach an end. Twelve dwells is
     * 1.8 s, five times that stall, and still well inside the servo's first overload stage at
     * 5.1 s, so a real end stop is caught long before anything is stressed.
     */
    private static final int CAL_MAX_STALLS = 12;

    private double calPos;
    private double calTargetPos;
    private double calLastRaw;
    private int calStalls;
    private int calSteps;

    /**
     * Whether {@link #calPos} is known to match where the servo physically is.
     *
     * <p>{@code Servo.getPosition()} returns the controller's last <em>commanded</em> position, not
     * a measurement - after a restart it is a default that has nothing to do with the pod. Walking
     * outward from it would make the first 0.02 step an absolute command to somewhere arbitrary,
     * and the pod would cross up to half its travel in one go. The incremental walk protects every
     * step except the one that matters, so calGoto refuses until calHome has established a true
     * starting position.
     */
    private boolean calHomed;
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

    private SwervePod[] pods;
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

    /** Smoothed loop period, seconds. {@link #loopHz} is derived as its inverse. */
    private double loopDtEma;

    /** Hubs with MANUAL bulk caching, cleared once per loop at the top of serviceLoop. */
    private final List<LynxModule> lynxModules = new ArrayList<>();

    /**
     * The rotation value actually handed to {@code arcadeDrive} this loop. With heading hold on
     * this is the heading PID's output, not the stick - and the visualizer/recorder mirror in
     * {@link #computeTargets} has to use the same number or the recorded targets describe a
     * command that was never sent.
     */
    private double appliedTurn;

    /** Whether the heading was usable last DRIVE loop, to catch the sensor coming back. */
    private boolean headingWasOkInDrive;

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
    private double msTelemetry;

    /** Last unrecognised command action, surfaced in errors[] so a typo cannot pass unnoticed. */
    private volatile String unknownCommand;

    /**
     * Turn gains this tool is holding that differ from the ones the robot actually ships with.
     *
     * <p>Divergence is legitimate - holding non-shipped gains is what a tuning tool is for - so
     * this reports rather than corrects. What is not legitimate is measuring at gains nobody
     * intended, which is exactly what happened when the calibration file sat at kD 0.010 while
     * SwerveDrivetrainConstants said 0.022: every number taken through the tool in that window was
     * at a configuration no one had chosen, and nothing said so.
     *
     * @return one description per differing pod and coefficient, empty when they agree
     */
    private List<String> gainDivergences() {
        List<String> out = new ArrayList<>();
        for (int i = 0; i < POD_COUNT; i++) {
            PodCal c = cals[i];
            // Shipped gains are per-pod arrays (ss-indexed, same as this tool's pod index).
            appendIfDifferent(out, i, "kP", c.kP, SwerveDrivetrainConstants.turnKPPerPod[i]);
            appendIfDifferent(out, i, "kD", c.kD, SwerveDrivetrainConstants.turnKDPerPod[i]);
            appendIfDifferent(out, i, "kS", c.kS, SwerveDrivetrainConstants.turnKSPerPod[i]);
            appendIfDifferent(out, i, "kS band", c.kSBandDeg,
                    SwerveDrivetrainConstants.turnKSBandDegPerPod[i]);
            appendIfDifferent(out, i, "cache", c.servoCaching,
                    SwerveDrivetrainConstants.turnServoCaching);
            appendIfDifferent(out, i, "kF", c.kF, 0.0);
            appendIfDifferent(out, i, "kI", c.kI, 0.0);

            // Calibration, not just gains. Wrong gains cost tuning time; a wrong zero points the
            // wheel somewhere else entirely, and until 2026-08-13 nothing compared these at all -
            // a repair and a re-zero left this file 30-175 degrees out with no warning anywhere.
            // Tolerance is deliberately loose: these are hand-captured and a tenth of a degree of
            // disagreement is not worth shouting about, but tens of degrees is.
            double zeroDeg = Math.toDegrees(c.angleOffsetRad);
            double shippedZero = SwerveDrivetrainConstants.podZeroDeg[i];
            // Wrap-aware: 359 against 1 is two degrees apart, not 358.
            double zeroGap = Math.abs(((zeroDeg - shippedZero) % 360 + 540) % 360 - 180);
            if (zeroGap > 0.5) {
                out.add(String.format(Locale.US,
                        "pod %d zero: tool %.1f deg, shipped %.1f deg (%.1f apart) - the robot is "
                                + "NOT calibrated the way competition code will drive it",
                        i, zeroDeg, shippedZero, zeroGap));
            }
            appendIfDifferentTol(out, i, "min V", c.analogMin,
                    SwerveDrivetrainConstants.podMinV[i], 0.01, "V");
            appendIfDifferentTol(out, i, "max V", c.analogMax,
                    SwerveDrivetrainConstants.podMaxV[i], 0.01, "V");

            // Directions too. A flipped drive direction and a wrong zero produce the same
            // haywire drive, and on 2026-08-13 the tool held two direction flips the shipped
            // file knew nothing about - nothing compared them.
            if (c.driveReversed() != SwerveDrivetrainConstants.podDriveReversed[i]) {
                out.add(String.format(Locale.US,
                        "pod %d drive direction: tool %s, shipped %s - this corner will push the "
                                + "wrong way under competition code",
                        i, c.driveReversed() ? "REVERSE" : "FORWARD",
                        SwerveDrivetrainConstants.podDriveReversed[i] ? "REVERSE" : "FORWARD"));
            }
        }
        return out;
    }

    /**
     * Reports a divergence only when it exceeds {@code tol}, so hand-captured values that agree to
     * within measurement noise do not fill the error panel and train people to ignore it.
     */
    private void appendIfDifferentTol(List<String> out, int pod, String name, double tool,
            double shipped, double tol, String unit) {
        if (Math.abs(tool - shipped) <= tol) {
            return;
        }
        out.add(String.format(Locale.US,
                "pod %d %s: tool %.3f %s, shipped %.3f %s - the robot is NOT calibrated the way "
                        + "competition code will drive it",
                pod, name, tool, unit, shipped, unit));
    }

    private static void appendIfDifferent(List<String> out, int pod, String name,
            double tool, double shipped) {
        if (Math.abs(tool - shipped) > 1e-9) {
            out.add(String.format(Locale.US,
                    "pod %d %s: tool %.4f, shipped %.4f - measurements here are NOT at the "
                            + "competition configuration", pod, name, tool, shipped));
        }
    }

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
     *
     * <p>NOT a constant any more, and the 8-13 ms above no longer holds: measured on 2026-08-13
     * the two together cost 42.7 ms of a 53.6 ms loop, so the dashboard is now eating four fifths
     * of the control bandwidth rather than half. publish() has grown a lot of fields since that
     * note was written. {@code setPublishHz} makes the rate switchable at runtime so the effect of
     * loop rate on pod tracking can be A/B'd without reflashing, and so a driving session can buy
     * control bandwidth back by giving up dashboard refresh.
     */
    private static final double PUBLISH_INTERVAL_DEFAULT_S = 0.05;

    /** Live publish period, seconds. Changed by {@code setPublishHz}. */
    private volatile double publishIntervalS = PUBLISH_INTERVAL_DEFAULT_S;

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
    // Seeded from what actually ships, so a heading session starts where the robot is rather than
    // where it used to be. These had been left at 1.75/0.003, the pre-tuning values, while
    // SwerveDrivetrainConstants moved to 1.20/0.030 - so the tool opened on gains the robot had
    // not used since 2026-08-11, and anything measured from that start would have been compared
    // against the wrong baseline. Not persisted, deliberately: the source of truth is
    // FollowerConstants.headingPIDFCoefficients, and a copy that outlived a session would just be
    // a second place to disagree.
    private double headingKp = 1.20;
    private double headingKd = 0.030;
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

    /**
     * Right stick sweeps a heading SETPOINT rather than commanding rotation power directly.
     *
     * <p>OFF by default as of 2026-08-13 late: with the pod turn loop at its current gains the
     * heading PIDF (kP 1.20 / kD 0.030, tuned 2026-08-11 at a much slower loop rate) shakes the
     * whole robot violently while driving. Raw right-stick rotation is the usable mode until the
     * heading loop is retuned at the current loop rate; the dashboard's drive panel has the
     * toggle for when that work happens.
     */
    private boolean headingHold = false;
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
            // MANUAL, cleared once at the top of every loop, not AUTO. Under AUTO a REPEATED read
            // of the same channel inside one loop forces a fresh bulk transaction, and DRIVE reads
            // every pod encoder three times per loop (readEncoders, Swerve's avgScaling, and
            // CoaxialPod.move) - roughly eight extra bus round-trips per loop that MANUAL serves
            // from the cache. Within-loop consistency is a feature here: the recorded volts[] and
            // the error the pod acted on come from the same snapshot.
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            lynxModules.add(module);
            // The servos are all on the Control Hub itself, so the parent module owns the rail.
            if (servoRailModule == null || module.isParent()) {
                servoRailModule = module;
            }
        }

        acquireHardware();

        // After acquireHardware, not before: its first act is hwErrors.clear(), so anything
        // reported earlier was being thrown away unread. Guard, not a review - PodCal's serialiser
        // has fallen behind its fields more than once, and nothing fails when a field is simply
        // never written.
        for (String gap : PodCal.roundTripGaps()) {
            hwErrors.add("PodCal does not persist: " + gap);
        }

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
        // Seed the cache: the idle-sensor timer does not fire for another IDLE_SENSOR_INTERVAL_S,
        // and until it does every reader - dashboard, telemetry, and any recording started
        // immediately - would see 0 V and have no way to tell that from a dead battery.
        refreshBatteryVolts();

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
        // MANUAL bulk caching: one fresh snapshot per loop, repeats served from cache.
        for (int i = 0; i < lynxModules.size(); i++) {
            lynxModules.get(i).clearBulkCache();
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            // Smooth dt and invert, not smooth 1/dt. Averaging instantaneous rates overweights
            // the short loops - the exact Jensen error that had every rate in this project
            // reading ~1.8x optimistic until 2026-08-13.
            loopDtEma = loopDtEma == 0 ? dt : 0.9 * loopDtEma + 0.1 * dt;
            loopHz = 1.0 / loopDtEma;
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
        if (refreshIdleSensors || fastCurrent) {
            readServoRailCurrent();
        }
        if (refreshIdleSensors) {
            refreshBatteryVolts();
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

        if (publishTimer.seconds() >= publishIntervalS) {
            publishTimer.reset();
            // Timed separately: msPublish used to cover both, which meant a 42 ms reading could
            // not distinguish "the JSON got too big" from "the SDK's telemetry push is slow".
            mark = System.nanoTime();
            publish();
            msPublish = smooth(msPublish, System.nanoTime() - mark);

            mark = System.nanoTime();
            pushTelemetry();
            msTelemetry = smooth(msTelemetry, System.nanoTime() - mark);
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

            if (podMoved[i] && pods != null && pods[i] instanceof CoaxialPod) {
                CoaxialPod cp = (CoaxialPod) pods[i];
                recError[i] = Math.toDegrees(cp.getLastErrorRad());
                recFlipped[i] = cp.wasLastMoveFlipped();
                servoCmd[i] = cp.getLastTurnPower();
            } else if (podMoved[i] && pods != null && pods[i] instanceof PositionalPod) {
                // Same error convention so one scorer reads both. Turn power is NaN by
                // construction - a positional pod has none - and everything downstream that keys
                // on it is meaningless here, which is why criterion 8 moved to holding current.
                PositionalPod pp = (PositionalPod) pods[i];
                recError[i] = Math.toDegrees(pp.getLastErrorRad());
                recFlipped[i] = pp.wasLastMoveFlipped();
                servoCmd[i] = pp.getLastTurnPower();
            } else {
                recError[i] = Double.NaN;
                recFlipped[i] = false;
            }
        }

        recorder.add(dt, batteryVolts(), loopHz, mode.ordinal(), servoRailMa, batteryMa,
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
            // A port declared Servo cannot be fetched as a CRServo and vice versa - the SDK
            // builds a different device class per port type and the get simply throws. During the
            // positional A/B one port is Servo and three are CR, so try both and keep whichever
            // answers; every site downstream uses the one that is non-null.
            servos[i] = null;
            posServos[i] = null;
            String crFailure = null;
            try {
                servos[i] = hardwareMap.get(CRServo.class, c.servoName);
                servos[i].setDirection(c.servoDirection);
                setServo(i, 0);
            } catch (RuntimeException e) {
                crFailure = e.getMessage();
            }
            if (servos[i] == null) {
                try {
                    posServos[i] = hardwareMap.get(Servo.class, c.servoName);
                } catch (RuntimeException e) {
                    // Name both attempts. "Missing" was misleading when the device was present
                    // and simply of the other type, which cost a bench session to work out.
                    hwErrors.add("Turn servo \"" + c.servoName + "\" (pod " + i
                            + ") is neither a ContinuousRotationServo nor a Servo. As CRServo: "
                            + crFailure + ". As Servo: " + e.getMessage());
                }
            }
            if (cals[i].positional && posServos[i] == null) {
                hwErrors.add("Pod " + i + " is configured positional but \"" + c.servoName
                        + "\" did not resolve as a Servo. Its port must be declared <Servo> in "
                        + "the active hardware configuration, not <ContinuousRotationServo>.");
            }
            if (!cals[i].positional && servos[i] == null && posServos[i] != null) {
                hwErrors.add("Pod " + i + " is configured continuous-rotation but \"" + c.servoName
                        + "\" resolved as a positional Servo. Set it positional, or change the "
                        + "port back to <ContinuousRotationServo>.");
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

    /**
     * When set, the servo rail current is read every loop instead of on the 5 Hz idle path.
     *
     * <p>Off by default and deliberately so: each read is three Lynx transactions, which is
     * exactly the cost that made {@code publish()} expensive. It exists because current is the
     * only torque proxy available here, and a 5 Hz sample cannot see the peak of a 0.37 s rise -
     * the "max" it reports is wherever the sampler happened to land. Turn it on for a current
     * measurement, off again afterwards, and do not read pod-tracking numbers off a trace
     * recorded with it on.
     */
    private volatile boolean fastCurrent;

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
        Object dev = servos[i] != null ? servos[i] : posServos[i];
        if (!(dev instanceof PwmControl)) {
            message = cals[i].servoName + " does not support PWM control.";
            return;
        }
        if (enabled) {
            ((PwmControl) dev).setPwmEnable();
        } else {
            setServo(i, 0);
            ((PwmControl) dev).setPwmDisable();
        }
        message = cals[i].servoName + " PWM " + (enabled ? "enabled" : "disabled");
    }

    /** Reads back the PWM pulse limits and frame period the hardware is actually using. */
    private void readPwmRange(int i) {
        pwmLower[i] = 0;
        pwmUpper[i] = 0;
        pwmFrame[i] = 0;
        Object dev = servos[i] != null ? servos[i] : posServos[i];
        if (!(dev instanceof PwmControl)) {
            return;
        }
        try {
            PwmControl.PwmRange r = ((PwmControl) dev).getPwmRange();
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
            Object dev = servos[i] != null ? servos[i] : posServos[i];
            if (!(dev instanceof PwmControl)) {
                message = cals[i].servoName + " does not support PWM control.";
                continue;
            }
            try {
                setServo(i, 0);
                ((PwmControl) dev).setPwmRange(
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
            SwervePod[] built = new SwervePod[POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                built[i] = cals[i].toSwervePod(hardwareMap);
            }
            for (int pi = 0; pi < built.length; pi++) {
                SwervePod pod = built[pi];
                if (pod instanceof PositionalPod) {
                    // Coverage is proved against the real calibration before anything is commanded.
                    // A band narrower than 180 degrees leaves headings unreachable, and the pod
                    // would stop tracking silently rather than report it.
                    double margin = ((PositionalPod) pod).verifyCoverage(0.25);
                    positionalCoverageDeg = margin;
                    if (margin < 0) {
                        throw new IllegalStateException("positional pod cannot reach every "
                                + "heading: clamped band is " + fmt(margin + 180) + " deg wide, "
                                + "needs 180. Re-check the endpoint calibration.");
                    }
                    // Before anything commands it: a position-mode servo drives to whatever it is
                    // told the instant it has power. A refusal here means the boot read could not
                    // be trusted, and the pod stays uncommanded rather than being sent somewhere
                    // on the strength of a bad number.
                    if (!((PositionalPod) pod).initFromEncoder()
                            && ((PositionalPod) pod).hasInitReadFault()) {
                        hwErrors.add("Pod " + pi + ": boot encoder reads disagreed by more than "
                                + "2 deg, so nothing was commanded. Check the pod is not being "
                                + "moved, and that the encoder wrap is outside the travel band.");
                    }
                }
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

    /** Every open-loop drive-motor write goes through here so {@link #motorCmd} stays faithful. */
    private void setMotor(int i, double power) {
        motorCmd[i] = power;
        if (motors[i] != null) {
            motors[i].setPower(power);
        }
    }

    private void allStop() {
        for (int i = 0; i < POD_COUNT; i++) {
            if (servos[i] != null) {
                setServo(i, 0);
            }
            setMotor(i, 0);
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
            case CAL_POS:
                runCalPos();
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
                    setMotor(i, 0);
                }
                break;
        }
    }

    private void setMode(Mode next) {
        // Leaving AUTOTUNE any way other than the tuner finishing must tell the tuner: its
        // update() simply stops being called, so without this isRunning() stays true forever.
        if (mode == Mode.AUTOTUNE && next != Mode.AUTOTUNE && tuner.isRunning()) {
            tuner.abort("mode changed to " + next);
        }
        // A wiring scan forces every turn servo to Direction.FORWARD so "positive power" is
        // unambiguous. Only finishWireScan restored them, so aborting a scan (STOP, another
        // mode) left reversed pods with a flipped feedback sign - closed loop runs away.
        if (mode == Mode.WIRE_SCAN && next != Mode.WIRE_SCAN) {
            for (int i = 0; i < POD_COUNT; i++) {
                if (servos[i] != null) {
                    servos[i].setDirection(cals[i].servoDirection);
                }
            }
        }
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
            setMotor(routinePod, MOTOR_PULSE_POWER);
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
            setMotor(i, 0);
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
        // Through setMode, not a bare assignment: setMode owns the transition cleanup (aborting
        // a live autotune, restoring scan-forced servo directions) and a jog can be commanded
        // from any mode.
        setMode(Mode.JOG);
        servos[selected].setDirection(cals[selected].servoDirection);
        setServo(selected, power);
        jogSeconds = seconds;
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
            setMotor(i, 0);
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
            setMotor(i, 0);
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

    /** One guarded step per dwell toward {@link #calTargetPos}, aborting on a stall. */
    private void runCalPos() {
        int i = selected;
        // Deliberately the raw Servo, not the pod object. Calibration is what produces the
        // endpoints a PositionalPod needs in order to exist, so requiring one here deadlocks:
        // the pod will not build without a valid band, and the band cannot be measured without
        // moving the servo.
        if (posServos[i] == null) {
            message = "Pod " + i + " is not on a Servo-configured port.";
            setMode(Mode.IDLE);
            return;
        }
        if (phaseTimer.seconds() < CAL_DWELL_S) {
            return;
        }
        phaseTimer.reset();

        double raw = Math.toDegrees(cals[i].rawAngleRad(volts[i]));
        boolean moved = calSteps == 0
                || Math.abs(((raw - calLastRaw) + 540.0) % 360.0 - 180.0) >= CAL_MIN_MOVE_DEG;

        // Hold the command when the pod is not following, rather than advancing anyway. Advancing
        // through a stall winds the servo's internal loop up against stiction, and calHome showed
        // what that produces: 25 degrees of accumulated error released at 390 deg/s. Waiting
        // instead keeps the command at most one step ahead of the pod, so a break is one step.
        if (!moved) {
            calStalls++;
            if (calStalls >= CAL_MAX_STALLS) {
                message = String.format(Locale.US,
                        "Calibration STOPPED at position %.3f, encoder %.2f deg: no movement for "
                                + "%.1f s at an unchanged command. This is a mechanical limit, not "
                                + "stiction. Mark it with calMark.",
                        calPos, raw, CAL_MAX_STALLS * CAL_DWELL_S);
                setMode(Mode.IDLE);
            }
            return;
        }

        calStalls = 0;
        calLastRaw = raw;

        if (Math.abs(calTargetPos - calPos) < 1e-6) {
            message = String.format(Locale.US,
                    "Reached commanded position %.3f, encoder %.2f deg. Mark it with calMark.",
                    calPos, raw);
            setMode(Mode.IDLE);
            return;
        }

        calPos += MathFunctions.clamp(calTargetPos - calPos, -CAL_STEP, CAL_STEP);
        posServos[i].setPosition(MathFunctions.clamp(calPos, 0.0, 1.0));
        calSteps++;
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
            // Zeroing the sticks is not enough under heading hold: the setpoint may still be up
            // to the lead cap ahead of the robot, and the heading PID would finish that rotation
            // with nobody at the controls. A watchdog trip means stop, so the setpoint comes back
            // to wherever the robot is.
            if (headingHold && headingOk) {
                headingTargetRad = headingRad;
                headingPidf.reset();
            }
            message = "Drive watchdog tripped - no command from the dashboard.";
        }

        // Heading source recovering mid-drive: the target still says whatever it said when the
        // sensor died, anywhere up to 180 degrees away, and resuming the PID against it would
        // spin the robot at full power with no stick input. Adopt the current heading instead.
        if (headingHold && headingOk && !headingWasOkInDrive) {
            headingTargetRad = headingRad;
            headingPidf.reset();
        }
        headingWasOkInDrive = headingOk;

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
                    appliedTurn = 0;
                    arcade(0, 0, 0);
                    return;
                }
            } else {
                headingStuckSeconds = 0;
            }
            headingLastSeen = headingRad;
        }

        appliedTurn = turn;
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
        List<String> failed = new ArrayList<>();
        for (int i = from; i < to; i++) {
            if (Double.isNaN(volts[i])) {
                failed.add(String.valueOf(i));
                continue;
            }
            cals[i].angleOffsetRad = cals[i].rawAngleRad(volts[i]);
        }
        podsDirty = true;
        saveCalibration();
        // The success line must not paper over a failure: it used to overwrite the "encoder not
        // readable" message, so zeroAll reported success while a dead pod silently kept its old
        // zero - the kind of thing that points a wheel the wrong way with no warning anywhere.
        if (!failed.isEmpty()) {
            message = "Zero NOT captured for pod(s) " + String.join(", ", failed)
                    + " - encoder unreadable. Their old zeros are still in effect.";
        } else {
            message = allPods
                    ? "Captured forward zero for all pods."
                    : "Captured forward zero for pod " + selected + ".";
        }
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
            case "setFastCurrent": {
                fastCurrent = boolArg(cmd, "value", !fastCurrent);
                message = fastCurrent
                        ? "Servo rail current now read EVERY LOOP. This costs loop rate - it is a "
                                + "measurement mode, not a monitoring mode. Turn it off after."
                        : "Servo rail current back on the idle-sensor path.";
                break;
            }

            case "setPublishHz": {
                // Clamped, not validated-and-rejected: 0 would stall the dashboard into looking
                // like a dead robot, and anything above the loop rate just publishes every loop.
                double hz = doubleArg(cmd, "value", 1 / PUBLISH_INTERVAL_DEFAULT_S);
                hz = Math.max(1.0, Math.min(200.0, hz));
                publishIntervalS = 1.0 / hz;
                message = String.format(Locale.US,
                        "Publish rate %.1f Hz. Lower frees control bandwidth; the recorder is "
                                + "unaffected because it samples every loop.", hz);
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
            case "setPositional": {
                if (!PodCal.POSITIONAL_ENABLED && boolArg(cmd, "value", true)) {
                    message = "Positional mode is shelved - see "
                            + "tools/swervetune/POSITIONAL_SHELVED.md. The drivetrain is "
                            + "continuous-rotation only; enabling this needs a source change to "
                            + "PodCal.POSITIONAL_ENABLED, not a command.";
                    break;
                }
                cals[selected].positional = boolArg(cmd, "value", !cals[selected].positional);
                cals[selected].rawDegAtPos0 = doubleArg(cmd, "raw0", cals[selected].rawDegAtPos0);
                cals[selected].rawDegAtPos1 = doubleArg(cmd, "raw1", cals[selected].rawDegAtPos1);
                cals[selected].posCalibrated = boolArg(cmd, "cal", cals[selected].posCalibrated);
                if (boolArg(cmd, "clearmarks", false)) {
                    cals[selected].posMarked0 = false;
                    cals[selected].posMarked1 = false;
                    cals[selected].posCalibrated = false;
                    // Drop the endpoints too. Leaving one real and one stale is what let a
                    // half-finished calibration present a 112 degree band as if it meant something.
                    cals[selected].rawDegAtPos0 = 0.0;
                    cals[selected].rawDegAtPos1 = 270.0;
                }
                calHomed = false;
                podsDirty = true;
                saveCalibration();
                message = "Pod " + selected + (cals[selected].positional
                        ? " is positional. Its port must be configured as Servo, not "
                          + "ContinuousRotationServo."
                        : " is back to continuous rotation.");
                break;
            }
            case "calPositional": {
                // Drives the servo to each end of its travel and records what the encoder reads
                // there. Two points is the whole calibration: shaft and pod are 1:1 and the
                // encoder is on that shaft, so between them the relationship is a straight line.
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                setMode(Mode.IDLE);
                posServos[selected].setPosition(doubleArg(cmd, "pos", 0.0));
                message = String.format(Locale.US,
                        "Pod %d driven to position %.3f. Wait for it to stop, read rawDeg, then "
                                + "send setPositional with raw0/raw1.",
                        selected, doubleArg(cmd, "pos", 0.0));
                break;
            }
            case "calHome": {
                // The one unavoidable uncontrolled move. Nothing can know where a position-mode
                // servo physically is until it has been commanded somewhere, so this commands
                // mid-travel - at most half the travel away from anywhere in the band, and Soft
                // Start bounds how fast. Everything after it walks from a known position.
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                allStop();
                double before = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                calPos = MathFunctions.clamp(doubleArg(cmd, "pos", 0.5), 0.0, 1.0);
                posServos[selected].setPosition(calPos);
                calHomed = true;
                calLastRaw = before;
                calSteps = 0;
                calStalls = 0;
                phaseTimer.reset();
                message = String.format(Locale.US,
                        "Pod %d commanded to %.3f from encoder %.1f deg. FIRST COMMANDED MOVE - "
                                + "expect up to half the travel, Soft Start limited. Let it stop, "
                                + "check the encoder settled, then calGoto.", selected, calPos, before);
                break;
            }
            case "calGoto": {
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo-configured port.";
                    break;
                }
                if (!calHomed) {
                    message = "Run calHome first: without it the starting position is the "
                            + "controller's cached default, not where the pod is, and the first "
                            + "step would be an absolute jump.";
                    break;
                }
                allStop();
                // calPos is carried forward from calHome and each completed walk, so it tracks
                // where the servo actually is rather than what the controller last cached.
                // No ensurePods() here: see runCalPos.
                calTargetPos = MathFunctions.clamp(doubleArg(cmd, "pos", 0.5), 0.0, 1.0);
                calStalls = 0;
                calSteps = 0;
                calLastRaw = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                phaseTimer.reset();
                mode = Mode.CAL_POS;
                routineActive = true;
                message = String.format(Locale.US,
                        "Walking pod %d from position %.3f to %.3f in %.3f steps, stopping on a "
                                + "stall.", selected, calPos, calTargetPos, CAL_STEP);
                break;
            }
            case "calMark": {
                if (posServos[selected] == null) {
                    message = "Pod " + selected + " is not on a Servo port.";
                    break;
                }
                double raw = Math.toDegrees(cals[selected].rawAngleRad(volts[selected]));
                PodCal mc = cals[selected];
                if (intArg(cmd, "which", 0) == 0) {
                    mc.rawDegAtPos0 = raw;
                    mc.posMarked0 = true;
                } else {
                    mc.rawDegAtPos1 = raw;
                    mc.posMarked1 = true;
                }
                // Both endpoints actually measured - not a span wide enough to look plausible.
                // One real endpoint against the other's stale default can span over 100 degrees
                // and would otherwise have declared the pod calibrated on a fiction.
                double span = Math.abs(mc.rawDegAtPos1 - mc.rawDegAtPos0);
                mc.posCalibrated = mc.posMarked0 && mc.posMarked1 && span > 100.0;
                podsDirty = true;
                saveCalibration();
                message = String.format(Locale.US,
                        "Marked endpoint %d at %.2f deg. Marked: pos0=%s pos1=%s. Span %.1f deg. "
                                + "calibrated=%s.",
                        intArg(cmd, "which", 0), raw, mc.posMarked0, mc.posMarked1, span,
                        mc.posCalibrated);
                break;
            }
            case "probeClamp": {
                // Deliberately asks for a position outside the clamp, in both directions, and
                // reports what was actually written. A clamp that has never been exercised is an
                // assumption, and this one stands between a command and a hard stop.
                if (pods == null || !(pods[selected] instanceof PositionalPod)) {
                    message = "Pod " + selected + " is not positional.";
                    break;
                }
                PositionalPod pp = (PositionalPod) pods[selected];
                double over = doubleArg(cmd, "over", 30.0);
                setMode(Mode.IDLE);
                double lo = pp.commandRawDegForTest(cals[selected].rawDegAtPos0
                        + (cals[selected].rawDegAtPos1 > cals[selected].rawDegAtPos0 ? -over : over));
                double hi = pp.commandRawDegForTest(cals[selected].rawDegAtPos1
                        + (cals[selected].rawDegAtPos1 > cals[selected].rawDegAtPos0 ? over : -over));
                message = String.format(Locale.US,
                        "Clamp probe: asked %.0f deg past each end, wrote positions %.4f and %.4f "
                                + "(both must be strictly inside 0 and 1).", over, lo, hi);
                break;
            }
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
                // Clear the stick-release latch: if the stick was active on the previous DRIVE
                // loop, the next loop would see it "just released" and latch the target back to
                // the current heading, silently cancelling this command.
                headingStickActive = false;
                podsDirty = true;
                driveForward = 0;
                driveStrafe = 0;
                driveTurn = 0;
                lastDriveCmdMs = System.currentTimeMillis();
                clearTrace(selected);
                tracePod = -2;
                // setMode, not a bare assignment: it stops whatever routine was mid-flight and
                // restores scan-forced servo directions. It re-zeroes the drive inputs, which
                // this command already set to zero, so ordering is safe.
                setMode(Mode.DRIVE);
                phaseTimer.reset();
                headingStickTimer.reset();
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
                if (mode != Mode.DRIVE) {
                    // Entering drive from any other mode goes through setMode so a routine that
                    // was mid-flight (wire scan, pulse, autotune) is actually stopped and its
                    // busy flag cleared, instead of leaving routineActive latched true and its
                    // last servo powers held. Assigning mode directly skipped all of that.
                    // setMode also zeroes the drive inputs, so it must run before they are set.
                    setMode(Mode.DRIVE);
                    if (headingOk) {
                        // Adopt the heading we are already at, so enabling drive never snaps.
                        headingTargetRad = headingRad;
                        headingPidf.reset();
                        headingStickTimer.reset();
                    }
                }
                driveForward = doubleArg(cmd, "f", 0);
                driveStrafe = doubleArg(cmd, "s", 0);
                driveTurn = doubleArg(cmd, "t", 0);
                lastDriveCmdMs = System.currentTimeMillis();
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
                // A misspelt or imagined action used to fall through here and report nothing, so
                // the caller saw a successful HTTP response for a command that did not exist. That
                // burned a whole staircase run against a "setMode" that was never implemented.
                // Silence is the worst answer available; say so instead.
                message = "UNKNOWN COMMAND: \"" + action + "\" - nothing was done.";
                unknownCommand = action;
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
        // That includes every automatic mode, not just the scans: a bumper press during
        // AUTOTUNE stranded the tuner mid-run, and during CAL_POS it abandoned a guarded walk.
        boolean scanning = mode == Mode.WIRE_SCAN || mode == Mode.ENC_SWEEP
                || mode == Mode.AUTOTUNE || mode == Mode.CAL_POS || mode == Mode.HEADING;
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
        // The rotation arcadeDrive actually received this loop. Under heading hold that is the
        // heading PID's output, and mirroring the raw stick here instead put targets in the
        // recorder that were never commanded - smooth stick, wild "demand".
        double rotation = appliedTurn;

        double transMag = Math.min(1.0, Math.hypot(strafe, forward));
        double transTheta = Math.atan2(forward, strafe);
        boolean zeroTrans = transMag < SWERVE_EPSILON;
        boolean zeroRot = Math.abs(rotation) < SWERVE_EPSILON;

        if (zeroTrans && zeroRot) {
            // Mirror the behaviour the drivetrain was actually built with: rebuildPods only
            // engages X_LOCK when xLock is on AND heading hold is off. Keying this on xLock
            // alone recorded X-lock targets while the pods were in fact released.
            if (!(xLock && !headingHold)) {
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
        sb.append(",\"fastCurrent\":").append(fastCurrent);
        sb.append(",\"posCoverage\":")
                .append(Double.isNaN(positionalCoverageDeg)
                        ? "null" : fmt(positionalCoverageDeg));
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
                .append(",\"telemetry\":").append(fmt(msTelemetry))
                .append(",\"publishHz\":").append(fmt(publishIntervalS > 0 ? 1 / publishIntervalS : 0))
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

        // Hardware faults first, then any divergence from the shipped gains. The divergence
        // entries are recomputed every publish rather than stored, so they clear themselves the
        // moment the gains match again.
        sb.append(",\"errors\":[");
        boolean firstError = true;
        for (String e : hwErrors) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc(e)).append('"');
            firstError = false;
        }
        if (unknownCommand != null) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc("UNKNOWN COMMAND \"" + unknownCommand
                    + "\" was sent and ignored - check the caller for a typo")).append('"');
            firstError = false;
        }
        for (String d : gainDivergences()) {
            if (!firstError) {
                sb.append(',');
            }
            sb.append('"').append(esc(d)).append('"');
            firstError = false;
        }
        sb.append(']');

        // Scalars kept for old readers; the perPod arrays are what the guard actually compares.
        sb.append(",\"shipped\":{\"kp\":").append(fmt(SwerveDrivetrainConstants.turnKP))
                .append(",\"kd\":").append(fmt(SwerveDrivetrainConstants.turnKD))
                .append(",\"ks\":").append(fmt(SwerveDrivetrainConstants.turnKS))
                .append(",\"ksband\":").append(fmt(SwerveDrivetrainConstants.turnKSBandDeg))
                .append(",\"cache\":").append(fmt(SwerveDrivetrainConstants.turnServoCaching));
        sb.append(",\"perPod\":[");
        for (int i = 0; i < POD_COUNT; i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"kp\":").append(fmt(SwerveDrivetrainConstants.turnKPPerPod[i]))
                    .append(",\"kd\":").append(fmt(SwerveDrivetrainConstants.turnKDPerPod[i]))
                    .append(",\"ks\":").append(fmt(SwerveDrivetrainConstants.turnKSPerPod[i]))
                    .append(",\"ksband\":")
                    .append(fmt(SwerveDrivetrainConstants.turnKSBandDegPerPod[i]))
                    .append('}');
        }
        sb.append("]}");

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
        sb.append(",\"hasServo\":").append(servos[i] != null || posServos[i] != null);
        sb.append(",\"servoType\":\"").append(servos[i] != null ? "CRServo"
                : (posServos[i] != null ? "Servo" : "none")).append('"');
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
        sb.append(",\"positional\":").append(c.positional);
        sb.append(",\"posCalibrated\":").append(c.posCalibrated);
        sb.append(",\"posMarked0\":").append(c.posMarked0);
        sb.append(",\"posMarked1\":").append(c.posMarked1);
        sb.append(",\"clampMargin\":").append(fmt(c.clampMarginDeg));
        sb.append(",\"raw0\":").append(fmt(c.rawDegAtPos0));
        sb.append(",\"raw1\":").append(fmt(c.rawDegAtPos1));
        sb.append(",\"pwmLo\":").append(fmt(pwmLower[i]));
        sb.append(",\"pwmHi\":").append(fmt(pwmUpper[i]));
        sb.append(",\"pwmFrame\":").append(fmt(pwmFrame[i]));
        sb.append(",\"discovered\":").append(c.discoveredEncoderIndex);

        // Cached command values, never hardware reads. getPower() on a CRServo or DcMotorEx is a
        // live Lynx transaction, and eight of them per publish was 30+ ms of the 38 ms publish
        // cost - the "publish is slow in DRIVE" mystery from 2026-08-13 in its entirety. In
        // closed-loop modes the pod's own cache is the truth (it writes through its own device
        // object); everywhere else the bench's shadow of its last write is.
        double shownServo = servoCmd[i];
        double shownDrive = motorCmd[i];
        if (podMoved[i] && pods != null && pods[i] instanceof CoaxialPod) {
            shownServo = ((CoaxialPod) pods[i]).getLastTurnPower();
            shownDrive = ((CoaxialPod) pods[i]).getLastDrivePower();
        } else if (podMoved[i] && pods != null && pods[i] instanceof PositionalPod) {
            shownServo = ((PositionalPod) pods[i]).getLastTurnPower();
            shownDrive = ((PositionalPod) pods[i]).getLastDrivePower();
        }
        sb.append(",\"servoPower\":").append(fmt(shownServo));
        sb.append(",\"drivePower\":").append(fmt(shownDrive));
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

    /**
     * Last cached battery reading. See {@link #refreshBatteryVolts()} for why this is not read on
     * demand.
     */
    private volatile double cachedVolts;

    /**
     * Reads the battery and caches it. Called only from the slow idle-sensor path.
     *
     * <p>{@code getVoltage()} is a Lynx ADC transaction, not something bulk caching covers. It used
     * to be called inline from {@code record()} - every loop - and again from {@code publish()} and
     * {@code pushTelemetry()}. In IDLE that is nearly free, but in DRIVE the bus is already
     * carrying eight actuator writes per loop and the read queues behind them: publish() measured
     * 37-56 ms in DRIVE against 9-13 ms in IDLE, which dragged the control loop down to 20-27 Hz.
     *
     * <p>Battery voltage does not change at loop rate, so sampling it at the idle-sensor rate costs
     * nothing real. The trade is that the recorder's volts column is now stair-stepped at that
     * rate rather than per-sample, which is ample for sag across a step but too coarse to catch a
     * sub-200 ms transient - measure that deliberately if it is ever the question.
     */
    private void refreshBatteryVolts() {
        if (voltageSensor == null) {
            cachedVolts = 0;
            return;
        }
        try {
            cachedVolts = voltageSensor.getVoltage();
        } catch (RuntimeException e) {
            cachedVolts = 0;
        }
    }

    private double batteryVolts() {
        return cachedVolts;
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
