package org.firstinspires.ftc.teamcode.diagnostics;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

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
            follower.setStartingPose(START_POSE);
            follower.update();
            lastDebugString = follower.getDrivetrain().debugString();
            validateDebugString(lastDebugString);
        } catch (RuntimeException e) {
            fatalInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            addFailure("Follower failed to initialize. Check hardware names and Pedro constants.");
        }

        updateTelemetry();
    }

    @Override
    public void init_loop() {
        if (follower != null) {
            follower.update();
            lastDebugString = follower.getDrivetrain().debugString();
            validateDebugString(lastDebugString);
        }

        updateTelemetry();
    }

    @Override
    public void start() {
        if (follower == null) {
            complete = true;
            return;
        }

        follower.setStartingPose(START_POSE);
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
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
        follower.setTeleOpDrive(step.forward, step.strafe, step.turn, true);
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
        stepStartPose = follower.getPose();
        maxVelocityThisStep = 0.0;
        maxDrivePowerThisStep = 0.0;
        maxServoPowerThisStep = 0.0;
        lastStepResult = "Running: " + STEPS[stepIndex].name;
        stepTimer.reset();
    }

    private void finishStep(CheckStep step) {
        int failureCountBeforeStep = failures.size();
        Pose endPose = follower.getPose();
        double dx = endPose.getX() - stepStartPose.getX();
        double dy = endPose.getY() - stepStartPose.getY();
        double headingDelta = angleDelta(endPose.getHeading(), stepStartPose.getHeading());

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
        Pose pose = follower.getPose();
        validatePose(pose);

        Vector velocity = follower.getVelocity();
        if (velocity != null && isFinite(velocity.getMagnitude())) {
            maxVelocityThisStep = Math.max(maxVelocityThisStep, velocity.getMagnitude());
        }

        lastDebugString = follower.getDrivetrain().debugString();
        validateDebugString(lastDebugString);

        maxDrivePowerThisStep = Math.max(maxDrivePowerThisStep,
                maxAbsDebugValue(lastDebugString, "drive Power = "));
        maxServoPowerThisStep = Math.max(maxServoPowerThisStep,
                maxAbsDebugValue(lastDebugString, "servo Power = "));
    }

    private void checkBatteryVoltage() {
        if (batteryChecked || follower == null) {
            return;
        }

        batteryChecked = true;
        double voltage = follower.getDrivetrain().getVoltage();
        if (!isFinite(voltage)) {
            addFailure("Battery voltage reading is invalid.");
        } else if (voltage < LOW_BATTERY_WARNING_VOLTS) {
            addWarning("Battery is low before match: " + format(voltage) + " V.");
        }
    }

    private void validatePose(Pose pose) {
        if (pose == null
                || !isFinite(pose.getX())
                || !isFinite(pose.getY())
                || !isFinite(pose.getHeading())) {
            addFailure("Follower/localizer returned an invalid pose.");
        }
    }

    private void validateDebugString(String debugString) {
        if (debugString == null || debugString.length() == 0) {
            addFailure("Pedro drivetrain debug string is empty.");
            return;
        }

        if (debugString.contains("NaN") || debugString.contains("Infinity")) {
            addFailure("Pedro drivetrain debug string contains invalid numeric values.");
        }

        for (int i = 0; i < 4; i++) {
            if (!debugString.contains("pod" + i)) {
                addFailure("Pedro drivetrain debug is missing pod" + i + ".");
            }
        }
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
            Pose pose = follower.getPose();
            telemetry.addData("X Position (in)", "%.2f", pose.getX());
            telemetry.addData("Y Position (in)", "%.2f", pose.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Max Velocity This Step", "%.2f", maxVelocityThisStep);
            telemetry.addData("Max Drive Power This Step", "%.2f", maxDrivePowerThisStep);
            telemetry.addData("Max Servo Power This Step", "%.2f", maxServoPowerThisStep);
            telemetry.addData("Battery (V)", "%.2f", follower.getDrivetrain().getVoltage());
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

        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
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

    private double maxAbsDebugValue(String debugString, String label) {
        double max = 0.0;
        int searchStart = 0;

        while (debugString != null) {
            int labelIndex = debugString.indexOf(label, searchStart);
            if (labelIndex < 0) {
                return max;
            }

            int valueStart = labelIndex + label.length();
            int valueEnd = valueStart;
            while (valueEnd < debugString.length()
                    && isNumberCharacter(debugString.charAt(valueEnd))) {
                valueEnd++;
            }

            if (valueEnd > valueStart) {
                try {
                    double value = Double.parseDouble(debugString.substring(valueStart, valueEnd));
                    if (isFinite(value)) {
                        max = Math.max(max, Math.abs(value));
                    }
                } catch (NumberFormatException ignored) {
                    addFailure("Could not parse Pedro drivetrain debug output.");
                }
            }

            searchStart = valueEnd + 1;
        }

        return max;
    }

    private boolean isNumberCharacter(char value) {
        return (value >= '0' && value <= '9')
                || value == '-'
                || value == '+'
                || value == '.'
                || value == 'E'
                || value == 'e';
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
