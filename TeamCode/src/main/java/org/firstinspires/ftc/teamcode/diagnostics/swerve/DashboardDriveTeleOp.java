package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrainConstants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Driver Station teleop that drives exactly like the bring-up dashboard's DRIVE mode.
 *
 * <p><b>Diagnostic, not shipped.</b> It reads the bring-up tool's on-hub calibration
 * ({@code swerve_bringup_cal.txt}) and safe-area box ({@code swerve_field_box.txt}), so it
 * drives the pods with whatever gains, zeros and directions the dashboard last saved - not
 * {@code SwerveDrivetrainConstants}. Every per-pod edit (setPidf, zeros, directions, range)
 * saves the file immediately. Not saved anywhere, so not seen here: the heading PIDF
 * (setHeadingPidf - this class uses the 1.20 / 0.080 defaults), the gain-schedule tuning, the
 * mixer taper/slew, the PWM range and live odoConfig.
 *
 * <p>Everything below the input stage is a copy of {@code SwerveBringUp.runDriveMode} and its
 * helpers, constants included: the same {@link SwerveConfig} (X_LOCK, no brake, no voltage
 * compensation, epsilon 0.05), the same heading PIDF (kP 1.20, kD 0.080) and hold phase machine
 * with the epsilon-bypass trim, the same field-oriented transform, and the same box limit. The
 * input stage reproduces {@code dashboard.html}'s gamepad path: vector deadband 0.06 on the left
 * stick, scalar 0.06 on the right, no speed scaling. {@code DriveTeleOp} differs from it on
 * every one of those points. If {@code runDriveMode} changes, this copy must follow - the
 * bring-up's {@code computeTargets} mirror drifted once already.
 *
 * <p>Known, deliberate differences from driving through the dashboard:
 * <ul>
 *   <li>Stick values arrive every loop instead of every 60 ms plus WiFi latency.
 *   <li>No publish, recorder or HTTP path, so the loop rate differs (unmeasured here).
 *   <li>The Pinpoint is read every loop. Bring-up reads it at 5 Hz when heading hold is off,
 *       which also slows its box and field-oriented updates in that case.
 *   <li>A box that fails the frame check is disarmed for this run only; the file is left for
 *       the bring-up tool to judge.
 *   <li>No 400 ms command watchdog: the Driver Station already zeroes a lost gamepad.
 *   <li>Runtime-only dashboard knobs (taper/slew, gain-schedule tuning, PWM range) are not
 *       persisted anywhere, so this sees their defaults unless it runs in the same app session.
 * </ul>
 *
 * <p>Controls (gamepad 1): left stick translate, right stick X turn, Y toggles heading hold,
 * X toggles X-lock, BACK toggles field-oriented (turning it on captures the current heading as
 * field forward, same as ticking the dashboard box).
 */
@TeleOp(name = "Dashboard Drive (bring-up parity)", group = "Diagnostics")
public class DashboardDriveTeleOp extends OpMode {

    private static final int POD_COUNT = 4;

    /** dashboard.html PAD_DEADBAND. */
    private static final double PAD_DEADBAND = 0.06;

    // ---- copied from SwerveBringUp ----
    private static final double SWERVE_EPSILON = 0.05;
    private static final double SWERVE_MAX_POWER = 1.0;

    private static final double HEADING_STOPPED_RATE_RAD_S = Math.toRadians(15);
    private static final double HEADING_STOP_TIMEOUT_S = 1.0;
    private static final double HEADING_MIN_ENGAGED_TURN = 0.06;
    private static final double HEADING_TRIM_MAX = 0.20;
    private static final double HEADING_TRIM_ENGAGE_RAD = Math.toRadians(1.2);
    private static final double HEADING_TRIM_RELEASE_RAD = Math.toRadians(0.5);
    private static final double HEADING_TRIM_RATE_GATE_RAD_S = Math.toRadians(10);
    private static final double HEADING_EPS_BYPASS = 0.055;
    private static final double HEADING_TRIM_MIN_TRANS = 0.25;
    private static final double HEADING_STUCK_LIMIT_S = 1.2;
    private static final double HEADING_STICK_RATE = 7.0;
    private static final double HEADING_MAX_LEAD = Math.toRadians(60);

    private static final double POSE_SANITY_IN_S = 80.0;
    private static final double BOX_MARGIN_BASE_IN = 4.0;
    private static final double BOX_MARGIN_LOOKAHEAD_S = 0.30;
    private static final double BOX_TAPER_IN = 6.0;
    private static final double BOX_WITNESS_TOLERANCE_IN = 12.0;

    private static final File CAL_FILE = new File(AppUtil.FIRST_FOLDER, "swerve_bringup_cal.txt");
    private static final File BOX_FILE = new File(AppUtil.FIRST_FOLDER, "swerve_field_box.txt");

    private static final double TELEMETRY_INTERVAL_S = 0.1;

    private final PodCal[] cals = new PodCal[POD_COUNT];
    private final List<LynxModule> hubs = new ArrayList<>();
    private Swerve swerve;
    private String podBuildError;
    private boolean calLoaded;

    private GoBildaPinpointDriver pinpoint;
    private double headingRad;
    private boolean headingOk;
    private double poseXIn, poseYIn, poseVxIn, poseVyIn;
    private boolean poseOk;

    private double boxMinX, boxMinY, boxMaxX, boxMaxY;
    private boolean boxValid;
    private boolean boxNeedsFrameCheck;
    private boolean boxClampedNow;
    private double boxWitnessX, boxWitnessY;
    private boolean boxWitnessValid;

    private boolean xLock = true;
    private boolean headingHold = true;
    private boolean driveFieldOriented;
    private double focRefRad;

    private double driveForward, driveStrafe, driveTurn;
    private double appliedForward, appliedStrafe, appliedTurn;

    private final double headingKp = 1.20;
    private final double headingKd = 0.080;
    private final double headingKf = 0.0;
    private final PIDFController headingPidf = new PIDFController(
            new com.pedropathing.control.PIDFCoefficientSupplier() {
                @Override
                public PIDFCoefficients get(double error) {
                    return new PIDFCoefficients(headingKp, 0, headingKd, headingKf);
                }
            });

    private enum HeadingHoldPhase { ACTIVE, STOPPING, RESTING }

    private HeadingHoldPhase headingPhase = HeadingHoldPhase.RESTING;
    private double headingTargetRad;
    private boolean headingWasOkInDrive;
    private boolean headingTrimEngaged;
    private double headingRateRadS;
    private double headingPrevForRate = Double.NaN;
    private double headingLastSeen;
    private double headingStuckSeconds;
    private final ElapsedTime headingStopTimer = new ElapsedTime();
    private final ElapsedTime headingRateTimer = new ElapsedTime();
    private final ElapsedTime headingStickTimer = new ElapsedTime();

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private double loopDtEma;
    private String message = "";

    private boolean prevX, prevY, prevBack;

    @Override
    public void init() {
        // Same defaults as SwerveBringUp.init, overwritten by the saved calibration.
        double dtLength = 146.420;
        double dtWidth = 154.240;
        cals[0] = new PodCal(0, "RB", -dtLength, -dtWidth);
        cals[1] = new PodCal(1, "RF", dtLength, -dtWidth);
        cals[2] = new PodCal(2, "LF", dtLength, dtWidth);
        cals[3] = new PodCal(3, "LB", -dtLength, dtWidth);
        calLoaded = loadCalibration();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hubs.add(module);
        }

        for (PodCal c : cals) {
            try {
                DcMotorEx m = hardwareMap.get(DcMotorEx.class, c.motorName);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m.setDirection(c.driveDirection);
            } catch (RuntimeException e) {
                // rebuildPods reports the missing device.
            }
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            com.pedropathing.revhub.localizers.PinpointConfig pc =
                    SwerveDrivetrainConstants.pinpointConfig;
            pinpoint.setOffsets(pc.xPodOffset.get(), pc.yPodOffset.get(), pc.offsetUnits.get());
            if (pc.ticksPerUnit.get().isPresent()) {
                pinpoint.setEncoderResolution(pc.ticksPerUnit.get().getAsDouble(),
                        pc.encoderResolutionUnit.get());
            } else {
                pinpoint.setEncoderResolution(pc.podType.get());
            }
            pinpoint.setEncoderDirections(pc.xPodDirection.get(), pc.yPodDirection.get());
            pinpoint.update();
        } catch (RuntimeException e) {
            pinpoint = null;
        }
        loadBox();

        if (calLoaded) {
            rebuildPods();
        }
        pushTelemetry();
    }

    @Override
    public void init_loop() {
        clearCaches();
        readHeading();
        pushTelemetry();
    }

    @Override
    public void start() {
        readHeading();
        // What the "drive" command does on entry: adopt the heading we are at, so nothing snaps.
        if (headingOk) {
            headingTargetRad = headingRad;
            headingPidf.reset();
            headingStickTimer.reset();
        }
        loopTimer.reset();
    }

    @Override
    public void loop() {
        clearCaches();

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0) {
            loopDtEma = loopDtEma == 0 ? dt : 0.9 * loopDtEma + 0.1 * dt;
        }

        readHeading();
        handleButtons();
        readSticks();

        if (swerve == null) {
            message = calLoaded
                    ? "Cannot build drivetrain: " + podBuildError
                    : "No " + CAL_FILE.getName() + " on the hub - save a calibration from "
                            + "Swerve Bring-Up first. Not driving.";
        } else {
            runDriveMode();
        }

        if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_S) {
            telemetryTimer.reset();
            pushTelemetry();
        }
    }

    @Override
    public void stop() {
        if (swerve != null) {
            swerve.applyDrive(new DrivePowers(0, 0, 0));
        }
    }

    private void clearCaches() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
    }

    // ---------------------------------------------------------------- input (dashboard.html)

    private void readSticks() {
        // dashboard.html pollPad: axes[1]/[0]/[2] of the Standard Gamepad mapping are the same
        // physical sticks, with the same sign, as left_stick_y/left_stick_x/right_stick_x.
        double rawF = -gamepad1.left_stick_y;
        double rawS = -gamepad1.left_stick_x;
        double mag = Math.sqrt(rawF * rawF + rawS * rawS);
        double k = mag > PAD_DEADBAND ? (mag - PAD_DEADBAND) / (1 - PAD_DEADBAND) / mag : 0;
        driveForward = rawF * k;
        driveStrafe = rawS * k;
        driveTurn = padAxis(-gamepad1.right_stick_x);
    }

    private static double padAxis(double v) {
        double a = Math.abs(v);
        if (a < PAD_DEADBAND) {
            return 0;
        }
        return (v < 0 ? -1 : 1) * (a - PAD_DEADBAND) / (1 - PAD_DEADBAND);
    }

    private void handleButtons() {
        if (gamepad1.y && !prevY) {
            // setHeadingHold
            headingHold = !headingHold;
            if (headingHold && headingOk) {
                headingTargetRad = headingRad;
                headingPidf.reset();
            }
            message = headingHold
                    ? "Right stick steers a heading setpoint; the robot holds it."
                    : "Right stick commands rotation power directly.";
        }
        if (gamepad1.x && !prevX) {
            // setXLock - zero-power behaviour lives in the SwerveConfig, so the pods rebuild.
            xLock = !xLock;
            rebuildPods();
            message = xLock
                    ? "Zero input locks the pods into an X."
                    : "Zero input holds pod headings.";
        }
        if (gamepad1.back && !prevBack) {
            driveFieldOriented = !driveFieldOriented;
            if (driveFieldOriented) {
                // focRef, sent by the dashboard as its checkbox turns on.
                if (headingOk) {
                    focRefRad = headingRad;
                    message = String.format(Locale.US,
                            "Field forward captured at %.1f deg.", Math.toDegrees(headingRad));
                } else {
                    message = "No heading available - field forward not captured.";
                }
            }
        }
        prevY = gamepad1.y;
        prevX = gamepad1.x;
        prevBack = gamepad1.back;
    }

    // ---------------------------------------------------------------- SwerveBringUp copies

    private boolean loadCalibration() {
        if (!CAL_FILE.exists()) {
            return false;
        }
        BufferedReader r = null;
        boolean any = false;
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
                if (idx >= 0 && idx < POD_COUNT && cals[idx].applySerialized(line)) {
                    any = true;
                }
            }
        } catch (IOException e) {
            message = "Could not read saved calibration: " + e.getMessage();
            return false;
        } finally {
            closeQuietly(r);
        }
        return any;
    }

    private void loadBox() {
        if (!BOX_FILE.exists()) {
            return;
        }
        BufferedReader r = null;
        try {
            r = new BufferedReader(new FileReader(BOX_FILE));
            String line;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("#") || line.trim().isEmpty()) {
                    continue;
                }
                String[] p = line.split("\\|");
                if (p.length >= 4 && "witness".equals(p[0])) {
                    boxWitnessX = Double.parseDouble(p[1]);
                    boxWitnessY = Double.parseDouble(p[2]);
                    boxWitnessValid = true;
                } else if (p.length >= 4) {
                    boxMinX = Double.parseDouble(p[0]);
                    boxMinY = Double.parseDouble(p[1]);
                    boxMaxX = Double.parseDouble(p[2]);
                    boxMaxY = Double.parseDouble(p[3]);
                    boxValid = true;
                    boxNeedsFrameCheck = true;
                }
            }
        } catch (IOException | NumberFormatException e) {
            boxValid = false;
        } finally {
            closeQuietly(r);
        }
    }

    private static void closeQuietly(java.io.Closeable c) {
        if (c != null) {
            try {
                c.close();
            } catch (IOException ignored) {
                // nothing useful to do
            }
        }
    }

    private void rebuildPods() {
        podBuildError = null;
        try {
            SwervePod[] pods = new SwervePod[POD_COUNT];
            for (int i = 0; i < POD_COUNT; i++) {
                pods[i] = cals[i].toSwervePod(hardwareMap);
            }
            SwerveConfig sc = new SwerveConfig(c -> {
                c.voltageCompensation.set(false);
                c.zeroPowerBehavior.set(xLock
                        ? SwerveConfig.ZeroPowerBehavior.X_LOCK
                        : SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(false);
                c.epsilon.set(SWERVE_EPSILON);
            });
            swerve = new Swerve(hardwareMap, sc, pods[2], pods[1], pods[3], pods[0]);
        } catch (RuntimeException e) {
            swerve = null;
            podBuildError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private void readHeading() {
        if (pinpoint == null) {
            headingOk = false;
            poseOk = false;
            return;
        }
        try {
            pinpoint.update();
            double h = pinpoint.getHeading(AngleUnit.RADIANS);
            headingOk = !Double.isNaN(h);
            if (headingOk) {
                headingRad = h;
            }
            Pose2D p = pinpoint.getPosition();
            double x = p.getX(DistanceUnit.INCH);
            double y = p.getY(DistanceUnit.INCH);
            poseOk = headingOk && !Double.isNaN(x) && !Double.isNaN(y);
            if (!poseOk) {
                return;
            }
            poseXIn = x;
            poseYIn = y;
            poseVxIn = pinpoint.getVelX(DistanceUnit.INCH);
            poseVyIn = pinpoint.getVelY(DistanceUnit.INCH);
            if (boxValid && Math.hypot(poseVxIn, poseVyIn) > POSE_SANITY_IN_S) {
                boxValid = false;
                message = "Box disarmed: odometry jumped faster than the robot can move "
                        + "(picked up or dragged?). Re-mark it in Swerve Bring-Up.";
            }
            if (boxNeedsFrameCheck) {
                boxNeedsFrameCheck = false;
                double absHeading = Math.abs(Angle.normalizeSigned(headingRad));
                boolean atOrigin = Math.abs(poseXIn) < 1.0 && Math.abs(poseYIn) < 1.0
                        && absHeading < Math.toRadians(2.0);
                boolean witnessFar = !boxWitnessValid
                        || Math.hypot(boxWitnessX, boxWitnessY) > 6.0;
                if (boxValid && atOrigin && witnessFar) {
                    boxValid = false;
                    message = "Saved box disarmed: the odometry frame looks reset. Re-mark "
                            + "corners A and B in Swerve Bring-Up.";
                } else if (boxValid && boxWitnessValid) {
                    double moved = Math.hypot(poseXIn - boxWitnessX, poseYIn - boxWitnessY);
                    if (moved > BOX_WITNESS_TOLERANCE_IN) {
                        message = String.format(Locale.US,
                                "Box loaded, but the robot is %.1f in from where it was when the "
                                        + "box was last written.", moved);
                    }
                }
            }
        } catch (RuntimeException e) {
            headingOk = false;
            poseOk = false;
        }
    }

    private static double outwardScale(double slackIn) {
        if (slackIn <= 0) {
            return 0.0;
        }
        if (slackIn >= BOX_TAPER_IN) {
            return 1.0;
        }
        double u = slackIn / BOX_TAPER_IN;
        return u * u * (3.0 - 2.0 * u);
    }

    private double[] applyBoxLimit(double forward, double strafe) {
        boxClampedNow = false;
        if (!boxValid) {
            return new double[] {forward, strafe};
        }
        if (!poseOk) {
            if (forward != 0 || strafe != 0) {
                boxClampedNow = true;
                message = "Box armed but pose unreadable - translation refused.";
            }
            return new double[] {0, 0};
        }
        double ch = Math.cos(headingRad);
        double sh = Math.sin(headingRad);
        double vx = forward * ch - strafe * sh;
        double vy = forward * sh + strafe * ch;

        double hiX = BOX_MARGIN_BASE_IN + Math.max(0, poseVxIn) * BOX_MARGIN_LOOKAHEAD_S;
        double loX = BOX_MARGIN_BASE_IN + Math.max(0, -poseVxIn) * BOX_MARGIN_LOOKAHEAD_S;
        double hiY = BOX_MARGIN_BASE_IN + Math.max(0, poseVyIn) * BOX_MARGIN_LOOKAHEAD_S;
        double loY = BOX_MARGIN_BASE_IN + Math.max(0, -poseVyIn) * BOX_MARGIN_LOOKAHEAD_S;

        boolean clamped = false;
        double fx = outwardScale(boxMaxX - hiX - poseXIn);
        if (vx > 0 && fx < 1.0) {
            vx *= fx;
            clamped = true;
        }
        double fxLo = outwardScale(poseXIn - (boxMinX + loX));
        if (vx < 0 && fxLo < 1.0) {
            vx *= fxLo;
            clamped = true;
        }
        double fy = outwardScale(boxMaxY - hiY - poseYIn);
        if (vy > 0 && fy < 1.0) {
            vy *= fy;
            clamped = true;
        }
        double fyLo = outwardScale(poseYIn - (boxMinY + loY));
        if (vy < 0 && fyLo < 1.0) {
            vy *= fyLo;
            clamped = true;
        }
        boxClampedNow = clamped;
        if (!clamped) {
            return new double[] {forward, strafe};
        }
        return new double[] {vx * ch + vy * sh, -vx * sh + vy * ch};
    }

    private double headingCorrection() {
        double direction = Angle.turnDirection(headingRad, headingTargetRad);
        double error = direction * Angle.smallestDifference(headingRad, headingTargetRad);
        headingPidf.updateFeedForwardInput(direction);
        headingPidf.updateError(error);
        return Utils.clamp(headingPidf.run(), -SWERVE_MAX_POWER, SWERVE_MAX_POWER);
    }

    /** SwerveBringUp.runDriveMode minus the watchdog and the recorder. See its comments. */
    private void runDriveMode() {
        if (headingHold && headingOk && !headingWasOkInDrive) {
            headingTargetRad = headingRad;
            headingPidf.reset();
        }
        headingWasOkInDrive = headingOk;

        double turn = driveTurn;

        if (headingHold && headingOk) {
            double dt = Math.min(0.25, Math.max(1e-3, headingStickTimer.seconds()));
            headingStickTimer.reset();

            boolean stickActive = Math.abs(driveTurn) > 0.055;

            double rateDt = Math.min(0.25, Math.max(1e-3, headingRateTimer.seconds()));
            headingRateTimer.reset();
            if (!Double.isNaN(headingPrevForRate)) {
                headingRateRadS = Angle.normalizeSigned(headingRad - headingPrevForRate) / rateDt;
            }
            headingPrevForRate = headingRad;

            boolean translating = Math.abs(driveForward) >= SWERVE_EPSILON
                    || Math.abs(driveStrafe) >= SWERVE_EPSILON;
            double headingAbsErr = Angle.smallestDifference(headingRad, headingTargetRad);

            boolean demand = stickActive || translating;

            if (demand) {
                if (headingPhase != HeadingHoldPhase.ACTIVE) {
                    headingPhase = HeadingHoldPhase.ACTIVE;
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                }

                if (stickActive) {
                    headingTargetRad = Angle.normalize(
                            headingTargetRad + driveTurn * HEADING_STICK_RATE * dt);
                    double lead = Angle.turnDirection(headingRad, headingTargetRad)
                            * Angle.smallestDifference(headingRad, headingTargetRad);
                    if (lead > HEADING_MAX_LEAD) {
                        headingTargetRad = Angle.normalize(headingRad + HEADING_MAX_LEAD);
                    } else if (lead < -HEADING_MAX_LEAD) {
                        headingTargetRad = Angle.normalize(headingRad - HEADING_MAX_LEAD);
                    }
                }

                turn = headingCorrection();
                if (!stickActive) {
                    double dir = Angle.turnDirection(headingRad, headingTargetRad);
                    if (headingAbsErr > HEADING_TRIM_ENGAGE_RAD) {
                        headingTrimEngaged = true;
                    } else if (headingAbsErr < HEADING_TRIM_RELEASE_RAD) {
                        headingTrimEngaged = false;
                    }
                    if (headingTrimEngaged) {
                        double transMag = Math.hypot(driveForward, driveStrafe);
                        if (transMag >= HEADING_TRIM_MIN_TRANS
                                && Math.abs(headingRateRadS) < HEADING_TRIM_RATE_GATE_RAD_S) {
                            double assisted = Math.abs(turn) + HEADING_EPS_BYPASS;
                            if (assisted > HEADING_TRIM_MAX) {
                                assisted = Math.max(HEADING_TRIM_MAX, Math.abs(turn));
                            }
                            turn = assisted * (turn != 0 ? Math.signum(turn) : dir);
                        }
                    } else if (translating) {
                        if (Math.hypot(driveForward, driveStrafe) >= HEADING_TRIM_MIN_TRANS) {
                            turn = 0;
                        }
                    } else if (Math.abs(turn) < HEADING_MIN_ENGAGED_TURN) {
                        turn = HEADING_MIN_ENGAGED_TURN * (turn != 0 ? Math.signum(turn) : dir);
                    }
                }
            } else {
                if (headingPhase == HeadingHoldPhase.ACTIVE) {
                    headingPhase = HeadingHoldPhase.STOPPING;
                    headingStopTimer.reset();
                }
                if (headingPhase == HeadingHoldPhase.STOPPING
                        && (Math.abs(headingRateRadS) < HEADING_STOPPED_RATE_RAD_S
                                || headingStopTimer.seconds() > HEADING_STOP_TIMEOUT_S)) {
                    headingTargetRad = headingRad;
                    headingPidf.reset();
                    headingPhase = HeadingHoldPhase.RESTING;
                }
                turn = 0;
            }

            if (Math.abs(turn) > 0.15) {
                if (Math.abs(headingRad - headingLastSeen) < 1e-6) {
                    headingStuckSeconds += dt;
                } else {
                    headingStuckSeconds = 0;
                }
                if (headingStuckSeconds > HEADING_STUCK_LIMIT_S) {
                    headingHold = false;
                    headingStuckSeconds = 0;
                    message = "Heading has not changed while turning - sensor looks dead. "
                            + "Heading hold disabled; recalibrate the Pinpoint.";
                    appliedForward = 0;
                    appliedStrafe = 0;
                    appliedTurn = 0;
                    swerve.applyDrive(new DrivePowers(0, 0, 0));
                    return;
                }
            } else {
                headingStuckSeconds = 0;
            }
            headingLastSeen = headingRad;
        }

        double effForward = driveForward;
        double effStrafe = driveStrafe;
        if (driveFieldOriented) {
            if (headingOk) {
                double rel = headingRad - focRefRad;
                double cos = Math.cos(rel);
                double sin = Math.sin(rel);
                effForward = driveForward * cos + driveStrafe * sin;
                effStrafe = -driveForward * sin + driveStrafe * cos;
            } else {
                effForward = 0;
                effStrafe = 0;
                message = "Field-oriented drive lost heading - translation stopped.";
            }
        }

        double[] fenced = applyBoxLimit(effForward, effStrafe);
        appliedTurn = turn;
        appliedForward = fenced[0];
        appliedStrafe = fenced[1];
        swerve.applyDrive(new DrivePowers(fenced[0], fenced[1], turn));
    }

    // ---------------------------------------------------------------- telemetry

    private void pushTelemetry() {
        telemetry.addData("calibration", calLoaded
                ? CAL_FILE.getName() + (swerve != null ? " - pods built" : " - BUILD FAILED")
                : "MISSING - will not drive");
        if (podBuildError != null) {
            telemetry.addData("pod build error", podBuildError);
        }
        for (PodCal c : cals) {
            telemetry.addLine(String.format(Locale.US,
                    "pod %d %s  kP %.3f kD %.3f kS %.3f band %.1f cache %.3f pulsed %b",
                    c.index, c.label, c.kP, c.kD, c.kS, c.kSBandDeg, c.servoCaching, c.pulsed));
        }
        telemetry.addData("heading hold (Y)", headingHold
                ? headingPhase + String.format(Locale.US, " err %.2f deg", Math.toDegrees(
                        Angle.turnDirection(headingRad, headingTargetRad)
                                * Angle.smallestDifference(headingRad, headingTargetRad)))
                : "OFF");
        telemetry.addData("X-lock (X)", xLock);
        telemetry.addData("field oriented (BACK)", driveFieldOriented);
        telemetry.addData("box", boxValid
                ? String.format(Locale.US, "ARMED x[%.1f, %.1f] y[%.1f, %.1f]%s",
                        boxMinX, boxMaxX, boxMinY, boxMaxY, boxClampedNow ? "  CLAMPING" : "")
                : "not armed");
        telemetry.addData("pose", headingOk
                ? String.format(Locale.US, "x %.1f  y %.1f in  h %.1f deg",
                        poseXIn, poseYIn, Math.toDegrees(headingRad))
                : "NO HEADING");
        telemetry.addData("applied f/s/t", String.format(Locale.US, "%.2f / %.2f / %.2f",
                appliedForward, appliedStrafe, appliedTurn));
        // Smoothed period inverted, not smoothed 1/dt. For the driver only, not a measurement.
        telemetry.addData("loop (smoothed period)", loopDtEma > 0
                ? String.format(Locale.US, "%.1f ms", loopDtEma * 1000) : "-");
        if (!message.isEmpty()) {
            telemetry.addData("msg", message);
        }
        telemetry.update();
    }
}
