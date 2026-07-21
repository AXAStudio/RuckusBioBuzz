package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Full18Auto", group = "Auto")
public class Full18Auto extends OpMode {

  private static final double NUMBER_XPOS = 1.000;
  private static final PathStep POSE_GATE_INTAKE_STEP = new PathStep(
    11.05245972673443,
    51.90197461212974,
    150
  );
  private static final PathStep START_STEP = new PathStep(
    21.872,
    122.757,
    90.000
  );
  private static final PathStep POINT_1 = new PathStep(8.102, 58.153, 180.000);
  private static final PathStep POINT_2 = new PathStep(57.978, 90.155, 180.000);
  private static final PathStep POINT_4 = new PathStep(57.978, 90.155, 150.000);
  private static final PathStep POINT_5 = new PathStep(14.905, 82.625, 180.000);
  private static final PathStep POINT_6 = new PathStep(
    52.719,
    108.508,
    180.000
  );

  private Follower follower;
  private PathChain path1;
  private PathChain path2;
  private PathChain path3;
  private PathChain path4;
  private PathChain path5;
  private PathChain path6;
  private PathChain[] repeat3Paths;
  private double[] repeat3PathSpeeds;
  private int sequenceIndex;
  private long stepStartTime;
  private boolean stepStarted;
  private boolean pathFinished;
  private static final int REPEAT_LOOP_CAPACITY = 1;
  private final int[] repeatLoopIterations = new int[REPEAT_LOOP_CAPACITY];
  private final int[] repeatLoopPathIndexes = new int[REPEAT_LOOP_CAPACITY];
  private static final int PARALLEL_EVENT_CAPACITY = 8;
  private final String[] activeParallelEventNames =
    new String[PARALLEL_EVENT_CAPACITY];
  private final long[] activeParallelEventStartTimes =
    new long[PARALLEL_EVENT_CAPACITY];
  private final long[] activeParallelEventDurations =
    new long[PARALLEL_EVENT_CAPACITY];

  @Override
  public void init() {
    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(START_STEP.toPose());

    buildPaths();
    updateTelemetry("Initialized");
  }

  @Override
  public void init_loop() {
    follower.update();
    updateTelemetry("Ready");
  }

  @Override
  public void start() {
    sequenceIndex = 0;
    stepStarted = false;
    pathFinished = false;
    resetRepeatLoops();
    resetParallelEvents();

    follower.setStartingPose(START_STEP.toPose());
  }

  @Override
  public void loop() {
    follower.update();
    updateParallelEvents();

    runSequence();

    updateTelemetry(pathFinished ? "Done" : "Running");
  }

  @Override
  public void stop() {
    finishAllParallelEvents();

    if (follower == null) {
      return;
    }

    follower.startTeleopDrive(true);
    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    follower.update();
  }

  private void buildPaths() {
    path1 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          START_STEP.toPose(),
          new Pose(90.924, 67.605),
          new Pose(53.701, 49.043),
          new Pose(11.696, 60.121),
          POINT_1.toPose()
        )
      )
      .setHeadingInterpolation(closestPoint ->
        interpolateHeading(
          Math.toRadians(142.000),
          Math.toRadians(180.000),
          closestPoint.getTValue(),
          0.700
        )
      )
      .addParametricCallback(0.010, () -> startParallelEvent("Shoot", 1800L))
      .addParametricCallback(0.450, () -> startParallelEvent("Intake", 3000L))
      .build();

    path2 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          POINT_1.toPose(),
          new Pose(52.397, 55.795),
          POINT_2.toPose()
        )
      )
      .setLinearHeadingInterpolation(
        Math.toRadians(180.000),
        Math.toRadians(180.000)
      )
      .addParametricCallback(0.670, () -> startParallelEvent("Shoot", 2200L))
      .build();

    path3 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          POINT_2.toPose(),
          new Pose(55.957, 63.278),
          new Pose(0.000, 66.334),
          new Pose(12.057, 58.852),
          POSE_GATE_INTAKE_STEP.toPose()
        )
      )
      .setLinearHeadingInterpolation(
        Math.toRadians(150.000),
        Math.toRadians(150.000)
      )
      .addParametricCallback(0.400, () -> startParallelEvent("Intake", 2500L))
      .build();

    path4 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          POSE_GATE_INTAKE_STEP.toPose(),
          new Pose(40.276, 69.600),
          POINT_4.toPose()
        )
      )
      .setLinearHeadingInterpolation(
        Math.toRadians(150.000),
        Math.toRadians(150.000)
      )
      .addParametricCallback(0.710, () -> startParallelEvent("Shoot", 1780L))
      .build();

    path5 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          POINT_4.toPose(),
          new Pose(41.584, 81.247),
          POINT_5.toPose()
        )
      )
      .setHeadingInterpolation(closestPoint ->
        interpolateHeading(
          Math.toRadians(150.000),
          Math.toRadians(180.000),
          closestPoint.getTValue(),
          0.250
        )
      )
      .addParametricCallback(0.220, () -> startParallelEvent("Shoot", 400L))
      .addParametricCallback(0.530, () -> startParallelEvent("Intake", 1700L))
      .build();

    path6 = follower
      .pathBuilder()
      .addPath(new BezierLine(POINT_5.toPose(), POINT_6.toPose()))
      .setLinearHeadingInterpolation(
        Math.toRadians(180.000),
        Math.toRadians(180.000)
      )
      .addParametricCallback(0.430, () -> startParallelEvent("Shoot", 0L))
      .build();

    repeat3Paths = new PathChain[] { path3, path4 };
    repeat3PathSpeeds = new double[] { 1.000, 1.000 };
  }

  private void runSequence() {
    if (pathFinished) {
      return;
    }

    switch (sequenceIndex) {
      case 0:
        followPathStep(path1, 1.000);
        break;
      case 1:
        followPathStep(path2, 1.000);
        break;
      case 2:
        followRepeatStep(repeat3Paths, repeat3PathSpeeds, 3, 0);
        break;
      case 3:
        followPathStep(path5, 1.000);
        break;
      case 4:
        followPathStep(path6, 1.000);
        break;
      default:
        pathFinished = true;
        finishAllParallelEvents();
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
        break;
    }
  }

  private static double interpolateHeading(
    double startHeading,
    double endHeading,
    double tValue,
    double curve
  ) {
    double clampedT = Math.max(0.0, Math.min(1.0, tValue));
    double clampedCurve = Math.max(0.25, Math.min(4.0, curve));
    double shapedT = Math.pow(clampedT, clampedCurve);
    double deltaHeading = normalizeRadians(endHeading - startHeading);
    return normalizeRadians(startHeading + deltaHeading * shapedT);
  }

  private static double normalizeRadians(double angle) {
    while (angle <= -Math.PI) {
      angle += 2.0 * Math.PI;
    }
    while (angle > Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    return angle;
  }

  private void followPathStep(PathChain path, double pathSpeed) {
    if (!stepStarted) {
      follower.followPath(path, clampPathSpeed(pathSpeed), true);
      stepStarted = true;
    }

    if (!follower.isBusy()) {
      advanceSequence();
    }
  }

  private void followRepeatStep(
    PathChain[] repeatPaths,
    double[] repeatPathSpeeds,
    int repeatCount,
    int repeatSlot
  ) {
    if (
      repeatSlot < 0 ||
      repeatSlot >= REPEAT_LOOP_CAPACITY ||
      repeatPaths == null ||
      repeatPaths.length == 0 ||
      repeatCount <= 0
    ) {
      advanceSequence();
      return;
    }

    int pathIndex = Math.max(
      0,
      Math.min(repeatPaths.length - 1, repeatLoopPathIndexes[repeatSlot])
    );
    PathChain path = repeatPaths[pathIndex];
    double pathSpeed = repeatPathSpeeds != null &&
      pathIndex < repeatPathSpeeds.length
      ? repeatPathSpeeds[pathIndex]
      : 1.0;

    if (!stepStarted) {
      follower.followPath(path, clampPathSpeed(pathSpeed), true);
      stepStarted = true;
    }

    if (follower.isBusy()) {
      return;
    }

    stepStarted = false;
    repeatLoopPathIndexes[repeatSlot]++;

    if (repeatLoopPathIndexes[repeatSlot] >= repeatPaths.length) {
      repeatLoopPathIndexes[repeatSlot] = 0;
      repeatLoopIterations[repeatSlot]++;
    }

    if (repeatLoopIterations[repeatSlot] >= repeatCount) {
      repeatLoopIterations[repeatSlot] = 0;
      repeatLoopPathIndexes[repeatSlot] = 0;
      advanceSequence();
    }
  }

  private double clampPathSpeed(double pathSpeed) {
    return Math.max(0.05, Math.min(1.0, pathSpeed));
  }

  private void runWaitStep(long durationMs) {
    if (!stepStarted) {
      stepStartTime = System.currentTimeMillis();
      stepStarted = true;
    }

    if (System.currentTimeMillis() - stepStartTime >= durationMs) {
      advanceSequence();
    }
  }

  private void runTimedEventStep(String eventName, long durationMs) {
    if (!stepStarted) {
      stepStartTime = System.currentTimeMillis();
      startEvent(eventName);
      stepStarted = true;
    }

    if (System.currentTimeMillis() - stepStartTime >= durationMs) {
      finishEvent(eventName);
      advanceSequence();
    }
  }

  private void startParallelEvent(String eventName, long durationMs) {
    startEvent(eventName);

    long clampedDurationMs = Math.max(0L, durationMs);
    if (clampedDurationMs == 0L) {
      // A zero duration means "fire once" -- finish immediately instead of leaving the event
      // permanently active, since updateParallelEvents() never finishes an event with a
      // non-positive duration.
      finishEvent(eventName);
      return;
    }

    long now = System.currentTimeMillis();
    int slot = -1;

    for (int i = 0; i < activeParallelEventNames.length; i++) {
      if (eventName.equals(activeParallelEventNames[i])) {
        slot = i;
        break;
      }
    }

    if (slot < 0) {
      for (int i = 0; i < activeParallelEventNames.length; i++) {
        if (activeParallelEventNames[i] == null) {
          slot = i;
          break;
        }
      }
    }

    if (slot < 0) {
      return;
    }

    activeParallelEventNames[slot] = eventName;
    activeParallelEventStartTimes[slot] = now;
    activeParallelEventDurations[slot] = clampedDurationMs;
  }

  private void updateParallelEvents() {
    long now = System.currentTimeMillis();

    for (int i = 0; i < activeParallelEventNames.length; i++) {
      String eventName = activeParallelEventNames[i];
      if (eventName == null || activeParallelEventDurations[i] <= 0L) {
        continue;
      }

      if (
        now - activeParallelEventStartTimes[i] >=
        activeParallelEventDurations[i]
      ) {
        finishEvent(eventName);
        clearParallelEvent(i);
      }
    }
  }

  private void finishAllParallelEvents() {
    for (int i = 0; i < activeParallelEventNames.length; i++) {
      String eventName = activeParallelEventNames[i];
      if (eventName != null) {
        finishEvent(eventName);
        clearParallelEvent(i);
      }
    }
  }

  private void resetParallelEvents() {
    for (int i = 0; i < activeParallelEventNames.length; i++) {
      clearParallelEvent(i);
    }
  }

  private void resetRepeatLoops() {
    for (int i = 0; i < REPEAT_LOOP_CAPACITY; i++) {
      repeatLoopIterations[i] = 0;
      repeatLoopPathIndexes[i] = 0;
    }
  }

  private void clearParallelEvent(int index) {
    activeParallelEventNames[index] = null;
    activeParallelEventStartTimes[index] = 0L;
    activeParallelEventDurations[index] = 0L;
  }

  private void startEvent(String eventName) {
    switch (eventName) {
      case "Shoot":
        startShoot();
        break;
      case "Intake":
        startIntake();
        break;
      default:
        break;
    }
  }

  private void finishEvent(String eventName) {
    switch (eventName) {
      case "Shoot":
        finishShoot();
        break;
      case "Intake":
        finishIntake();
        break;
      default:
        break;
    }
  }

  private void startShoot() {
    // TODO: start Shoot mechanism here.
  }

  private void finishShoot() {
    // TODO: stop Shoot mechanism here.
  }

  private void startIntake() {
    // TODO: start Intake mechanism here.
  }

  private void finishIntake() {
    // TODO: stop Intake mechanism here.
  }

  private void advanceSequence() {
    sequenceIndex++;
    stepStarted = false;
  }

  private void updateTelemetry(String state) {
    Pose pose = follower.getPose();

    telemetry.addData("State", state);
    telemetry.addData("Sequence", sequenceIndex);
    telemetry.addData("X", "%.2f", pose.getX());
    telemetry.addData("Y", "%.2f", pose.getY());
    telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.getHeading()));
    telemetry.update();
  }
}
