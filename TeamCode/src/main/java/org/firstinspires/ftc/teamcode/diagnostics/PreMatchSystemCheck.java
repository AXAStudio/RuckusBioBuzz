package org.firstinspires.ftc.teamcode.diagnostics;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Autonomous(name = "PreMatchSystemCheck", group = "Diagnostics")
public class PreMatchSystemCheck extends OpMode {
    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);
    private static final double CHECK_POWER = 0.30;
    private static final double DRIVE_SECONDS = 0.85;
    private static final double STOP_SECONDS = 0.25;

    private static final double LOW_BATTERY_WARNING_VOLTS = 12.0;
    private static final double MIN_TRANSLATION_INCHES = 1.25;
    private static final double MIN_TURN_RADIANS = Math.toRadians(8.0);
    private static final double MAX_TRANSLATION_DRIFT_INCHES = 2.5;
    private static final double MAX_TURN_DRIFT_INCHES = 3.0;
    private static final double MAX_HEADING_DRIFT_RADIANS = Math.toRadians(18.0);
    private static final double MIN_COMMANDED_DRIVE_POWER = 0.04;
    private static final double MIN_OBSERVED_VELOCITY = 0.75;

    /** Pods Swerve.debug() must report, one nested map each. */
    private static final int EXPECTED_PODS = 4;

    private static final CheckStep[] STEPS = new CheckStep[] {
            new CheckStep("Settle before checks", CheckType.SETTLE, 0.0, 0.0, 0.0, 0.40),
            new CheckStep("Forward drive response", CheckType.FORWARD_POSITIVE,
                    CHECK_POWER, 0.0, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after forward", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Backward drive response", CheckType.FORWARD_NEGATIVE,
                    -CHECK_POWER, 0.0, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after backward", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Left strafe response", CheckType.STRAFE_POSITIVE,
                    0.0, CHECK_POWER, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after left strafe", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Right strafe response", CheckType.STRAFE_NEGATIVE,
                    0.0, -CHECK_POWER, 0.0, DRIVE_SECONDS),
            new CheckStep("Stop after right strafe", CheckType.SETTLE, 0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Counterclockwise turn response", CheckType.TURN_POSITIVE,
                    0.0, 0.0, CHECK_POWER, DRIVE_SECONDS),
            new CheckStep("Stop after counterclockwise turn", CheckType.SETTLE,
                    0.0, 0.0, 0.0, STOP_SECONDS),
            new CheckStep("Clockwise turn response", CheckType.TURN_NEGATIVE,
                    0.0, 0.0, -CHECK_POWER, DRIVE_SECONDS)
    };

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final List<String> failures = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();

    private Follower follower;
    /** The sensor Pedro 2's Swerve.getVoltage() read; Pedro 3's Drivetrain has no accessor. */
    private VoltageSensor voltageSensor;
    private Pose stepStartPose = START_POSE;
    private int stepIndex;
    private boolean running;
    private boolean complete;
    private boolean batteryChecked;
    private String fatalInitError;
    private String lastStepResult = "Waiting for start";
    private String lastDebugString = "";
    private double maxVelocityThisStep;
    private double maxDrivePowerThisStep;
    private double maxServoPowerThisStep;

    @Override
    public void init() {
        try {
            follower = Constants.createFollower(hardwareMap);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            follower.setPose(START_POSE);
            // Pedro 2's update() with no path only refreshed the pose; Pedro 3's IDLE update()
            // writes every servo. Keep init actuator-silent.
            follower.localizer.update();
            validateDebug(follower.drivetrain.debug());
        } catch (RuntimeException e) {
            fatalInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            addFailure("Follower failed to initialize. Check hardware names and Pedro constants.");
        }

        updateTelemetry();
    }

    @Override
    public void init_loop() {
        if (follower != null) {
            follower.localizer.update();
            validateDebug(follower.drivetrain.debug());
        }

        updateTelemetry();
    }

    @Override
    public void start() {
        if (follower == null) {
            complete = true;
            return;
        }

        follower.setPose(START_POSE);
        follower.manual(DrivePowers.zero());
        follower.update();

        checkBatteryVoltage();

        stepIndex = 0;
        complete = false;
        running = true;
        beginStep();
    }

    @Override
    public void loop() {
        if (follower == null || complete) {
            stopFollower();
            updateTelemetry();
            return;
        }

        CheckStep step = STEPS[stepIndex];
        follower.manual(new DrivePowers(step.forward, step.strafe, step.turn));
        follower.update();

        sampleStepHealth();

        if (stepTimer.seconds() >= step.durationSeconds) {
            finishStep(step);
            stepIndex++;

            if (stepIndex >= STEPS.length) {
                complete = true;
                running = false;
                lastStepResult = failures.isEmpty()
                        ? "All autonomous checks completed"
                        : "Autonomous checks completed with issues";
                stopFollower();
            } else {
                beginStep();
            }
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        stopFollower();
    }

    private void beginStep() {
        stepStartPose = follower.pose();
        maxVelocityThisStep = 0.0;
        maxDrivePowerThisStep = 0.0;
        maxServoPowerThisStep = 0.0;
        lastStepResult = "Running: " + STEPS[stepIndex].name;
        stepTimer.reset();
    }

    private void finishStep(CheckStep step) {
        int failureCountBeforeStep = failures.size();
        Pose endPose = follower.pose();
        double dx = endPose.x() - stepStartPose.x();
        double dy = endPose.y() - stepStartPose.y();
        double headingDelta = angleDelta(endPose.heading(), stepStartPose.heading());

        validatePose(endPose);

        switch (step.type) {
            case FORWARD_POSITIVE:
                evaluateTranslation(step.name, dx, dy, headingDelta, 1, "X");
                break;
            case FORWARD_NEGATIVE:
                evaluateTranslation(step.name, dx, dy, headingDelta, -1, "X");
                break;
            case STRAFE_POSITIVE:
                evaluateTranslation(step.name, dy, dx, headingDelta, 1, "Y");
                break;
            case STRAFE_NEGATIVE:
                evaluateTranslation(step.name, dy, dx, headingDelta, -1, "Y");
                break;
            case TURN_POSITIVE:
                evaluateTurn(step.name, dx, dy, headingDelta, 1);
                break;
            case TURN_NEGATIVE:
                evaluateTurn(step.name, dx, dy, headingDelta, -1);
                break;
            case SETTLE:
            default:
                lastStepResult = step.name + " complete";
                return;
        }

        if (step.type != CheckType.SETTLE
                && maxDrivePowerThisStep < MIN_COMMANDED_DRIVE_POWER
                && maxVelocityThisStep < MIN_OBSERVED_VELOCITY) {
            addFailure(step.name + " saw almost no drivetrain output. Check module servo angles, "
                    + "drive motor wiring, and Pedro swerve constants.");
        }

        lastStepResult = failures.size() == failureCountBeforeStep
                ? "PASS: " + step.name
                : "FAIL: " + step.name;
    }

    private void evaluateTranslation(
            String stepName,
            double axisDelta,
            double crossAxisDelta,
            double headingDelta,
            int expectedSign,
            String axisName) {
        double signedDelta = axisDelta * expectedSign;
        double allowedCrossDrift = Math.max(MAX_TRANSLATION_DRIFT_INCHES,
                Math.abs(axisDelta) * 0.8);

        if (signedDelta < MIN_TRANSLATION_INCHES) {
            addFailure(stepName + " did not move in expected " + axisName + " direction. "
                    + "Delta=" + format(axisDelta) + " in.");
        }

        if (Math.abs(crossAxisDelta) > allowedCrossDrift) {
            addFailure(stepName + " drifted sideways too much. Cross delta="
                    + format(crossAxisDelta) + " in.");
        }

        if (Math.abs(headingDelta) > MAX_HEADING_DRIFT_RADIANS) {
            addFailure(stepName + " rotated while translating. Heading drift="
                    + format(Math.toDegrees(headingDelta)) + " deg.");
        }
    }

    private void evaluateTurn(String stepName, double dx, double dy, double headingDelta,
            int expectedSign) {
        double signedHeadingDelta = headingDelta * expectedSign;
        double translationDrift = Math.hypot(dx, dy);

        if (signedHeadingDelta < MIN_TURN_RADIANS) {
            addFailure(stepName + " did not turn in expected direction. Heading delta="
                    + format(Math.toDegrees(headingDelta)) + " deg.");
        }

        if (translationDrift > MAX_TURN_DRIFT_INCHES) {
            addFailure(stepName + " translated too much while turning. Drift="
                    + format(translationDrift) + " in.");
        }
    }

    private void sampleStepHealth() {
        Pose pose = follower.pose();
        validatePose(pose);

        double speed = follower.velocity().toVector2D().magnitude();
        if (isFinite(speed)) {
            maxVelocityThisStep = Math.max(maxVelocityThisStep, speed);
        }

        Map<String, Object> debug = follower.drivetrain.debug();
        validateDebug(debug);

        maxDrivePowerThisStep = Math.max(maxDrivePowerThisStep, maxAbsPodValue(debug, "drivePower"));
        maxServoPowerThisStep = Math.max(maxServoPowerThisStep, maxAbsPodValue(debug, "servoPower"));
    }

    private void checkBatteryVoltage() {
        if (batteryChecked || follower == null) {
            return;
        }

        batteryChecked = true;
        double voltage = readVoltage();
        if (!isFinite(voltage)) {
            addFailure("Battery voltage reading is invalid.");
        } else if (voltage < LOW_BATTERY_WARNING_VOLTS) {
            addWarning("Battery is low before match: " + format(voltage) + " V.");
        }
    }

    private double readVoltage() {
        return voltageSensor == null ? Double.NaN : voltageSensor.getVoltage();
    }

    private void validatePose(Pose pose) {
        if (pose == null
                || !isFinite(pose.x())
                || !isFinite(pose.y())
                || !isFinite(pose.heading())) {
            addFailure("Follower/localizer returned an invalid pose.");
        }
    }

    /**
     * Pedro 2 exposed a debug STRING and this check parsed it ("pod0".."pod3", "drive Power = ").
     * Pedro 3's Drivetrain.debug() is a map - scalar mixer fields plus one nested map per pod,
     * keyed by the pod's configured name - so the same checks run on the map directly.
     */
    private void validateDebug(Map<String, Object> debug) {
        lastDebugString = debug == null ? "" : debug.toString();

        if (debug == null || debug.isEmpty()) {
            addFailure("Pedro drivetrain debug output is empty.");
            return;
        }

        if (!allFinite(debug)) {
            addFailure("Pedro drivetrain debug output contains invalid numeric values.");
        }

        int pods = 0;
        for (Object value : debug.values()) {
            if (value instanceof Map) {
                pods++;
            }
        }
        if (pods != EXPECTED_PODS) {
            addFailure("Pedro drivetrain debug reports " + pods + " pods, expected "
                    + EXPECTED_PODS + ".");
        }
    }

    private boolean allFinite(Map<?, ?> map) {
        for (Object value : map.values()) {
            if (value instanceof Map) {
                if (!allFinite((Map<?, ?>) value)) {
                    return false;
                }
            } else if (value instanceof Number && !isFinite(((Number) value).doubleValue())) {
                return false;
            }
        }
        return true;
    }

    private double maxAbsPodValue(Map<String, Object> debug, String key) {
        double max = 0.0;
        for (Object value : debug.values()) {
            if (!(value instanceof Map)) {
                continue;
            }
            Object field = ((Map<?, ?>) value).get(key);
            if (field instanceof Number) {
                double v = ((Number) field).doubleValue();
                if (isFinite(v)) {
                    max = Math.max(max, Math.abs(v));
                }
            }
        }
        return max;
    }

    private void updateTelemetry() {
        if (fatalInitError != null) {
            telemetry.addData("Fatal Init Error", fatalInitError);
        }

        telemetry.addData("Overall", overallStatus());
        telemetry.addData("Last Step", lastStepResult);

        if (running && !complete) {
            CheckStep step = STEPS[stepIndex];
            telemetry.addData("Step", "%d/%d %s", stepIndex + 1, STEPS.length, step.name);
            telemetry.addData("Step Time", "%.2f / %.2f s", stepTimer.seconds(),
                    step.durationSeconds);
            telemetry.addData("Command", "f %.2f | s %.2f | t %.2f",
                    step.forward, step.strafe, step.turn);
        }

        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("X Position (in)", "%.2f", pose.x());
            telemetry.addData("Y Position (in)", "%.2f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addData("Max Velocity This Step", "%.2f", maxVelocityThisStep);
            telemetry.addData("Max Drive Power This Step", "%.2f", maxDrivePowerThisStep);
            telemetry.addData("Max Servo Power This Step", "%.2f", maxServoPowerThisStep);
            telemetry.addData("Battery (V)", "%.2f", readVoltage());
        }

        addIssueTelemetry("Failures", failures);
        addIssueTelemetry("Warnings", warnings);
        telemetry.addData("Swerve Debug", lastDebugString);
        telemetry.update();
    }

    private void addIssueTelemetry(String caption, List<String> issues) {
        if (issues.isEmpty()) {
            telemetry.addData(caption, "none");
            return;
        }

        int limit = Math.min(issues.size(), 6);
        for (int i = 0; i < limit; i++) {
            telemetry.addData(caption + " " + (i + 1), issues.get(i));
        }

        if (issues.size() > limit) {
            telemetry.addData(caption + " More", issues.size() - limit);
        }
    }

    private String overallStatus() {
        if (fatalInitError != null || !failures.isEmpty()) {
            return "FAIL";
        }

        if (!running && !complete) {
            return "READY";
        }

        if (!complete) {
            return "RUNNING";
        }

        return warnings.isEmpty() ? "PASS" : "PASS WITH WARNINGS";
    }

    private void stopFollower() {
        if (follower == null) {
            return;
        }

        follower.manual(DrivePowers.zero());
        follower.update();
    }

    private void addFailure(String issue) {
        if (!failures.contains(issue)) {
            failures.add(issue);
        }
    }

    private void addWarning(String issue) {
        if (!warnings.contains(issue)) {
            warnings.add(issue);
        }
    }

    private double angleDelta(double current, double previous) {
        double delta = current - previous;
        while (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return delta;
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private String format(double value) {
        return String.format("%.2f", value);
    }

    private enum CheckType {
        SETTLE,
        FORWARD_POSITIVE,
        FORWARD_NEGATIVE,
        STRAFE_POSITIVE,
        STRAFE_NEGATIVE,
        TURN_POSITIVE,
        TURN_NEGATIVE
    }

    private static class CheckStep {
        final String name;
        final CheckType type;
        final double forward;
        final double strafe;
        final double turn;
        final double durationSeconds;

        CheckStep(String name, CheckType type, double forward, double strafe, double turn,
                double durationSeconds) {
            this.name = name;
            this.type = type;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.durationSeconds = durationSeconds;
        }
    }
}
