package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import android.util.Size;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrainConstants;
import org.firstinspires.ftc.teamcode.pipelines.PollenDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "RedPollenAuto", group = "Auto")
public class RedPollenAuto extends OpMode {
  // PathStep fields are editable from FTC Dashboard; paths are built in init(), so edits apply on the next init.

  private static final double NUMBER_SHOOT_MS = 1000.000;
  private static final boolean FLAG_TAKE_THIRD_SHOT = true;
  public static PathStep START_STEP = new PathStep(
    55.000,
    9.000,
    90.000
  );
  public static PathStep POINT_1 = new PathStep(55.000, 28.000, 84.600);
  public static PathStep POINT_2 = new PathStep(30.000, 62.000, 41.300);
  public static PathStep POINT_3 = new PathStep(15.600, 47.167, 180.000);
  public static PathStep POINT_4 = new PathStep(30.000, 47.167, 180.000);
  public static PathStep POINT_5 = new PathStep(55.000, 28.000, 84.600);
  public static PathStep POINT_6 = new PathStep(17.000, 106.000, 90.000);
  public static PathStep POINT_7 = new PathStep(17.000, 106.000, 90.000);

  private Follower follower;
  private AutoPath chain1;
  private AutoPath chain2;
  private AutoPath chain3;
  private AutoPath chain4;
  private AutoPath chain5;
  private AutoPath chain6;
  private AutoPath chain7;
  private AutoPath[] repeat1Paths;
  private double[] repeat1PathSpeeds;
  private long[] repeat1HoldMs;
  private String[] repeat1HoldEvents;
  private AutoPath[] repeat2Paths;
  private double[] repeat2PathSpeeds;
  private long[] repeat2HoldMs;
  private String[] repeat2HoldEvents;
  private AutoPath activePath;
  private int sequenceIndex;
  private long stepStartTime;
  private boolean stepStarted;
  private boolean pathFinished;
  private static final int REPEAT_LOOP_CAPACITY = 2;
  private final int[] repeatLoopIterations = new int[REPEAT_LOOP_CAPACITY];
  private final int[] repeatLoopPathIndexes = new int[REPEAT_LOOP_CAPACITY];
  private static final int PARALLEL_EVENT_CAPACITY = 1;
  private final String[] activeParallelEventNames =
    new String[PARALLEL_EVENT_CAPACITY];
  private final long[] activeParallelEventStartTimes =
    new long[PARALLEL_EVENT_CAPACITY];
  private final long[] activeParallelEventDurations =
    new long[PARALLEL_EVENT_CAPACITY];

  // --- Pollen Pickup ------------------------------------------------------------
  // Camera model from the visualizer's Settings -> Vision.
  // NOT MEASURED: these are placeholders. Every field position the robot drives to
  // comes from them - measure the mount before trusting a pickup.
  private static final String CAMERA_NAME = "Webcam 1";
  private static final int CAMERA_WIDTH = 640;
  private static final int CAMERA_HEIGHT = 480;
  private static final double CAMERA_HFOV_DEG = 62.0;
  private static final double CAMERA_VFOV_DEG = 48.0;
  private static final double CAMERA_FORWARD_IN = 8.0;
  private static final double CAMERA_LEFT_IN = 0.0;
  private static final double CAMERA_HEIGHT_IN = 10.0;
  private static final double CAMERA_PITCH_DEG = 25.0;
  private static final double CAMERA_YAW_DEG = 0.0;
  private static final double CAMERA_MAX_RANGE_IN = 96.0;
  private static final double POLLEN_RADIUS_IN = 1.4;
  private static final double POLLEN_INTAKE_CREEP_SPEED = 0.35;
  /** AUTO midline (BIOBUZZ G402): 0 unchecked, 1 stay on red (x <= mid), 2 stay on blue. */
  private static final int POLLEN_MIDLINE_SIDE = 1;
  private static final double FIELD_MIDLINE_X = 70.75;

  private static final int POLLEN_SEARCH = 0;
  private static final int POLLEN_APPROACH = 1;
  private static final int POLLEN_INTAKE = 2;
  private static final int POLLEN_RETURN = 3;

  private VisionPortal visionPortal;
  private PollenDetectionPipeline pollenPipeline;
  private int pollenPhase;
  private long pollenPhaseStart;
  private double pollenTargetX;
  private double pollenTargetY;
  private boolean pollenIntakeRunning;
  private String pollenIntakeEvent = "";

  /** One Pollen Pickup step, as the visualizer planned it. */
  private static final class PollenPickup {

    final double searchX, searchY, searchHeadingDeg;
    final double targetX, targetY, zoneRadius;
    final int minBalls;
    final long timeoutMs;
    final boolean blindOnTimeout;
    final double standoffInches, retargetInches, approachSpeed;
    final double intakeDriveInches;
    final long intakeMs;
    final String intakeEvent;
    final boolean returnToStart;

    PollenPickup(
      double searchX,
      double searchY,
      double searchHeadingDeg,
      double targetX,
      double targetY,
      double zoneRadius,
      int minBalls,
      long timeoutMs,
      boolean blindOnTimeout,
      double standoffInches,
      double retargetInches,
      double approachSpeed,
      double intakeDriveInches,
      long intakeMs,
      String intakeEvent,
      boolean returnToStart
    ) {
      this.searchX = searchX;
      this.searchY = searchY;
      this.searchHeadingDeg = searchHeadingDeg;
      this.targetX = targetX;
      this.targetY = targetY;
      this.zoneRadius = zoneRadius;
      this.minBalls = minBalls;
      this.timeoutMs = timeoutMs;
      this.blindOnTimeout = blindOnTimeout;
      this.standoffInches = standoffInches;
      this.retargetInches = retargetInches;
      this.approachSpeed = approachSpeed;
      this.intakeDriveInches = intakeDriveInches;
      this.intakeMs = intakeMs;
      this.intakeEvent = intakeEvent;
      this.returnToStart = returnToStart;
    }
  }

  // Tip-1 spill
  private static final PollenPickup POLLEN_PICKUP_1 = new PollenPickup(
    55.0,
    28.0,
    84.6, // search pose: where the route has the robot
    60.0,
    60.0,
    9.0, // expected POLLEN and zone radius
    1,
    1200L,
    false,
    10.0,
    3.0,
    0.6,
    6.0,
    500L,
    "Intake",
    true
  );

  // Flower pollen
  private static final PollenPickup POLLEN_PICKUP_2 = new PollenPickup(
    30.0,
    47.1667,
    180.0, // search pose: where the route has the robot
    9.0,
    47.1667,
    9.0, // expected POLLEN and zone radius
    1,
    1200L,
    true,
    10.0,
    3.0,
    0.6,
    3.5,
    600L,
    "Intake",
    true
  );

  @Override
  public void init() {
    // Pedro 3 follows paths with Foresight, whose braking model and gains must be measured
    // on the robot first. Refuse to build a path-following OpMode on placeholders.
    SwerveDrivetrainConstants.requireForesightMeasured();

    follower = Constants.createFollower(hardwareMap);
    follower.setPose(START_STEP.toPose());

    buildPaths();

    // Pollen Pickup: PollenDetectionPipeline tuned from Settings -> Vision.
    PollenDetectionPipeline.Config pollenConfig =
      new PollenDetectionPipeline.Config();
    pollenConfig.hsvLowA = new double[] { 15.0, 90.0, 70.0 };
    pollenConfig.hsvHighA = new double[] { 38.0, 255.0, 255.0 };
    pollenConfig.hsvLowB = new double[] { 12.0, 60.0, 45.0 };
    pollenConfig.hsvHighB = new double[] { 42.0, 255.0, 255.0 };
    pollenConfig.openRadius = 3;
    pollenConfig.closeHGap = 16;
    pollenConfig.closeVRadius = 12;
    pollenConfig.minArea = 350.0;
    pollenConfig.maxArea = 100000.0;
    pollenConfig.maxAspect = 5.0;
    pollenConfig.clumpMergeGap = 28;
    pollenConfig.singleBallAreaPx = 1600.0;
    pollenPipeline = new PollenDetectionPipeline(null, pollenConfig);
    visionPortal = new VisionPortal.Builder()
      .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
      .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
      .addProcessor(pollenPipeline)
      .build();
    updateTelemetry("Initialized");
  }

  @Override
  public void init_loop() {
    // Pedro 3's update() with no path stops the drivetrain, writing every servo each loop.
    // Refresh the localizer alone so init stays actuator-silent.
    follower.localizer.update();
    updateTelemetry("Ready");
  }

  @Override
  public void start() {
    sequenceIndex = 0;
    stepStarted = false;
    pathFinished = false;
    activePath = null;
    resetRepeatLoops();
    resetParallelEvents();

    follower.setPose(START_STEP.toPose());
  }

  @Override
  public void loop() {
    follower.update();
    // Path callbacks run right after the follower update, where Pedro 2 ran them.
    if (activePath != null) {
      activePath.update(follower);
    }
    updateParallelEvents();

    runSequence();

    updateTelemetry(pathFinished ? "Done" : "Running");
  }

  @Override
  public void stop() {
    finishAllParallelEvents();

    if (pollenIntakeRunning) {
      finishEvent(pollenIntakeEvent);
      pollenIntakeRunning = false;
    }
    if (visionPortal != null) {
      visionPortal.close();
      visionPortal = null;
    }

    if (follower == null) {
      return;
    }

    follower.manual(DrivePowers.zero());
    follower.update();
  }

  private void buildPaths() {
    chain1 = new AutoPath(
      Paths.line(START_STEP.toPose(), POINT_1.toPose()).heading(
        headingInterpolator(
          Math.toRadians(90.000),
          Math.toRadians(84.600),
          1.000
        )
      )
    );

    chain2 = new AutoPath(
      Paths.curve(
        POINT_1.toPose(),
        new Pose(30.000, 28.000),
        POINT_2.toPose()
      ).heading(
        headingInterpolator(
          Math.toRadians(84.600),
          Math.toRadians(41.300),
          1.000
        )
      )
    );

    chain3 = new AutoPath(
      Paths.line(POINT_2.toPose(), POINT_3.toPose()).heading(
        headingInterpolator(
          Math.toRadians(41.300),
          Math.toRadians(180.000),
          1.000
        )
      )
    );

    chain4 = new AutoPath(
      Paths.line(POINT_3.toPose(), POINT_4.toPose()).constant(
        Math.toRadians(180.000)
      )
    );

    chain5 = new AutoPath(
      Paths.line(POINT_4.toPose(), POINT_5.toPose()).heading(
        headingInterpolator(
          Math.toRadians(180.000),
          Math.toRadians(84.600),
          1.000
        )
      )
    );

    chain6 = new AutoPath(
      Paths.curve(
        POINT_5.toPose(),
        new Pose(30.000, 26.000),
        new Pose(22.000, 64.000),
        POINT_6.toPose()
      ).heading(
        headingInterpolator(
          Math.toRadians(84.600),
          Math.toRadians(90.000),
          1.000
        )
      )
    );

    chain7 = new AutoPath(
      Paths.curve(
        POINT_4.toPose(),
        new Pose(26.000, 78.000),
        POINT_7.toPose()
      ).heading(
        headingInterpolator(
          Math.toRadians(180.000),
          Math.toRadians(90.000),
          1.000
        )
      )
    );

    repeat1Paths = new AutoPath[] { chain5, null, chain6 };
    repeat1PathSpeeds = new double[] { 1.000, 1.0, 1.000 };
    repeat1HoldMs = new long[] { 0, Math.round(NUMBER_SHOOT_MS), 0 };
    repeat1HoldEvents = new String[] { null, "Shoot", null };

    repeat2Paths = new AutoPath[] { chain7 };
    repeat2PathSpeeds = new double[] { 1.000 };
    repeat2HoldMs = new long[] { 0 };
    repeat2HoldEvents = new String[] { null };
  }

  private void runSequence() {
    if (pathFinished) {
      return;
    }

    switch (sequenceIndex) {
      case 0:
        followPathStep(chain1, 1.000);
        break;
      case 1:
        runTimedEventStep("Shoot", Math.round(NUMBER_SHOOT_MS));
        break;
      case 2:
        runWaitStep(800L);
        break;
      case 3:
        runPollenPickupStep(POLLEN_PICKUP_1);
        break;
      case 4:
        followPathStep(chain2, 1.000);
        break;
      case 5:
        runTimedEventStep("Shoot", Math.round(NUMBER_SHOOT_MS));
        break;
      case 6:
        followPathStep(chain3, 1.000);
        break;
      case 7:
        runTimedEventStep("Flower knock", 800L);
        break;
      case 8:
        followPathStep(chain4, 1.000);
        break;
      case 9:
        runPollenPickupStep(POLLEN_PICKUP_2);
        break;
      case 10:
        if (FLAG_TAKE_THIRD_SHOT) {
          followRepeatStep(
            repeat1Paths,
            repeat1PathSpeeds,
            repeat1HoldMs,
            repeat1HoldEvents,
            1,
            0
          );
        } else {
          advanceSequence();
        }
        break;
      case 11:
        if (!(FLAG_TAKE_THIRD_SHOT)) {
          followRepeatStep(
            repeat2Paths,
            repeat2PathSpeeds,
            repeat2HoldMs,
            repeat2HoldEvents,
            1,
            1
          );
        } else {
          advanceSequence();
        }
        break;
      default:
        pathFinished = true;
        activePath = null;
        finishAllParallelEvents();
        follower.manual(DrivePowers.zero());
        break;
    }
  }

  /**
   * Heading from start to end on the curve parameter t, shaped by t^curve - exactly what the
   * visualizer draws, and Pedro 2's setLinearHeadingInterpolation at curve 1. Pedro 3's
   * Path.linear() interpolates on distance fraction instead, which differs on most Beziers.
   */
  private static Interpolator headingInterpolator(
    double startHeading,
    double endHeading,
    double curve
  ) {
    double clampedCurve = Math.max(0.25, Math.min(4.0, curve));
    double deltaHeading = normalizeRadians(endHeading - startHeading);
    return (pathCurve, t) ->
      normalizeRadians(
        startHeading +
          deltaHeading * Math.pow(Math.max(0.0, Math.min(1.0, t)), clampedCurve)
      );
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

  private void followPathStep(AutoPath path, double pathSpeed) {
    if (!stepStarted) {
      startPath(path, pathSpeed);
      stepStarted = true;
    }

    if (!follower.isBusy()) {
      advanceSequence();
    }
  }

  /** Pedro 2's followPath(path, maxPower, holdEnd = true), callbacks re-armed as it did. */
  private void startPath(AutoPath path, double pathSpeed) {
    path.arm();
    activePath = path;
    follower.holdEnd.set(true);
    follower.follow(withPathSpeed(path.path, clampPathSpeed(pathSpeed)));
  }

  /**
   * Pedro 2 capped drive POWER per path; Pedro 3 has no power cap. The nearest thing is
   * Foresight's maxPathSpeed - a fraction of max achievable VELOCITY - applied only while
   * this path is followed. At 1.0 nothing is applied.
   */
  private Path withPathSpeed(Path path, double pathSpeed) {
    if (pathSpeed >= 1.0 || !(follower.algorithm() instanceof Foresight)) {
      return path;
    }
    return path.with(
      ((Foresight) follower.algorithm()).config.maxPathSpeed.at(pathSpeed)
    );
  }

  private void followRepeatStep(
    AutoPath[] repeatPaths,
    double[] repeatPathSpeeds,
    long[] repeatHoldMs,
    String[] repeatHoldEvents,
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
    AutoPath path = repeatPaths[pathIndex];
    double pathSpeed = repeatPathSpeeds != null &&
      pathIndex < repeatPathSpeeds.length
      ? repeatPathSpeeds[pathIndex]
      : 1.0;

    // A null chain marks a wait or an event sitting between two paths: the
    // slot holds a duration instead of something to drive.
    if (path == null) {
      long holdMs = repeatHoldMs != null && pathIndex < repeatHoldMs.length
        ? repeatHoldMs[pathIndex]
        : 0L;

      if (!stepStarted) {
        stepStartTime = System.currentTimeMillis();
        stepStarted = true;

        String eventName = repeatHoldEvents != null &&
          pathIndex < repeatHoldEvents.length
          ? repeatHoldEvents[pathIndex]
          : null;
        if (eventName != null) {
          startParallelEvent(eventName, holdMs);
        }
      }

      if (System.currentTimeMillis() - stepStartTime < holdMs) {
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
      return;
    }

    if (!stepStarted) {
      startPath(path, pathSpeed);
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
      // A zero duration means "fire once" -- finish immediately instead of leaving the
      // event permanently active, since updateParallelEvents() never finishes an event
      // with a non-positive duration.
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
      case "Flower knock":
        startFlowerKnock();
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
      case "Flower knock":
        finishFlowerKnock();
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

  private void startFlowerKnock() {
    // TODO: start Flower knock mechanism here.
  }

  private void finishFlowerKnock() {
    // TODO: stop Flower knock mechanism here.
  }

  private void startIntake() {
    // TODO: start Intake mechanism here.
  }

  private void finishIntake() {
    // TODO: stop Intake mechanism here.
  }

  /**
   * A Pollen Pickup: look for POLLEN, drive to the best clump in the zone, run the intake,
   * and come back to where the route has the robot, so every later path starts where it
   * was planned from.
   *
   * The approach re-plans whenever the estimate moves more than retargetInches, which is
   * what makes a camera mount measured to the nearest inch good enough: the field
   * position of a clump is least certain far away, and the step keeps refining it as it
   * closes in. Once the POLLEN drops below the bottom of the image it keeps the last aim.
   */
  private void runPollenPickupStep(PollenPickup step) {
    long now = System.currentTimeMillis();
    if (!stepStarted) {
      stepStarted = true;
      stepStartTime = now;
      pollenPhase = POLLEN_SEARCH;
      pollenPhaseStart = now;
      activePath = null;
    }

    switch (pollenPhase) {
      case POLLEN_SEARCH: {
        double[] seen = bestPollenInZone(step);
        if (seen != null) {
          pollenTargetX = seen[0];
          pollenTargetY = seen[1];
          startPollenApproach(step);
          return;
        }
        if (now - stepStartTime < step.timeoutMs) {
          return;
        }
        if (step.blindOnTimeout) {
          pollenTargetX = step.targetX;
          pollenTargetY = step.targetY;
          startPollenApproach(step);
          return;
        }
        // Nothing in the zone: leave the robot where the route has it.
        advanceSequence();
        return;
      }
      case POLLEN_APPROACH: {
        double[] seen = bestPollenInZone(step);
        if (
          seen != null &&
          Math.hypot(seen[0] - pollenTargetX, seen[1] - pollenTargetY) >
          step.retargetInches
        ) {
          pollenTargetX = seen[0];
          pollenTargetY = seen[1];
          startPollenApproach(step);
          return;
        }
        if (!follower.isBusy()) {
          startPollenIntake(step);
        }
        return;
      }
      case POLLEN_INTAKE: {
        if (follower.isBusy() || now - pollenPhaseStart < step.intakeMs) {
          return;
        }
        if (pollenIntakeRunning) {
          finishEvent(pollenIntakeEvent);
          pollenIntakeRunning = false;
        }
        if (step.returnToStart) {
          startPollenReturn(step);
        } else {
          advanceSequence();
        }
        return;
      }
      default: {
        if (!follower.isBusy()) {
          advanceSequence();
        }
      }
    }
  }

  private void startPollenApproach(PollenPickup step) {
    Pose pose = follower.pose();
    double dx = pollenTargetX - pose.x();
    double dy = pollenTargetY - pose.y();
    double distance = Math.hypot(dx, dy);
    double facing = distance > 1e-6 ? Math.atan2(dy, dx) : pose.heading();
    double travel = Math.max(0.0, distance - step.standoffInches);

    pollenPhase = POLLEN_APPROACH;
    pollenPhaseStart = System.currentTimeMillis();
    if (travel < 0.5) {
      // Already inside the standoff: nothing to approach.
      startPollenIntake(step);
      return;
    }
    followPollenLeg(
      pose.x(),
      pose.y(),
      pose.heading(),
      pose.x() + Math.cos(facing) * travel,
      pose.y() + Math.sin(facing) * travel,
      facing,
      step.approachSpeed
    );
  }

  private void startPollenIntake(PollenPickup step) {
    Pose pose = follower.pose();
    pollenPhase = POLLEN_INTAKE;
    pollenPhaseStart = System.currentTimeMillis();

    if (!step.intakeEvent.isEmpty() && !pollenIntakeRunning) {
      pollenIntakeEvent = step.intakeEvent;
      startEvent(pollenIntakeEvent);
      pollenIntakeRunning = true;
    }

    double dx = pollenTargetX - pose.x();
    double dy = pollenTargetY - pose.y();
    double facing = Math.hypot(dx, dy) > 1e-6
      ? Math.atan2(dy, dx)
      : pose.heading();
    if (step.intakeDriveInches > 0.25) {
      followPollenLeg(
        pose.x(),
        pose.y(),
        pose.heading(),
        pose.x() + Math.cos(facing) * step.intakeDriveInches,
        pose.y() + Math.sin(facing) * step.intakeDriveInches,
        facing,
        POLLEN_INTAKE_CREEP_SPEED
      );
    }
  }

  private void startPollenReturn(PollenPickup step) {
    Pose pose = follower.pose();
    pollenPhase = POLLEN_RETURN;
    pollenPhaseStart = System.currentTimeMillis();
    followPollenLeg(
      pose.x(),
      pose.y(),
      pose.heading(),
      step.searchX,
      step.searchY,
      Math.toRadians(step.searchHeadingDeg),
      1.0
    );
  }

  /** A straight leg built at runtime, turning the short way from one heading to the other. */
  private void followPollenLeg(
    double fromX,
    double fromY,
    double fromHeading,
    double toX,
    double toY,
    double toHeading,
    double speed
  ) {
    activePath = null;
    if (Math.hypot(toX - fromX, toY - fromY) < 0.5) {
      // A zero-length line has no tangent; there is nowhere to go.
      return;
    }
    Path leg = Paths.line(new Pose(fromX, fromY), new Pose(toX, toY)).heading(
      headingInterpolator(fromHeading, toHeading, 1.0)
    );
    follower.holdEnd.set(true);
    follower.follow(withPathSpeed(leg, clampPathSpeed(speed)));
  }

  /**
   * The best clump this frame that is inside the step's zone and on the allowed side of
   * the midline, as a field position: most estimated balls first, nearest on a tie. Null
   * when there is none.
   */
  private double[] bestPollenInZone(PollenPickup step) {
    if (pollenPipeline == null) {
      return null;
    }
    Pose pose = follower.pose();
    double cos = Math.cos(pose.heading());
    double sin = Math.sin(pose.heading());
    int frameWidth = Math.max(1, pollenPipeline.getFrameWidth());
    int frameHeight = Math.max(1, pollenPipeline.getFrameHeight());

    double[] best = null;
    int bestCount = -1;
    double bestDistance = Double.MAX_VALUE;
    for (PollenDetectionPipeline.Clump clump : pollenPipeline.getAllClumps()) {
      if (clump.estimatedBallCount < step.minBalls) {
        continue;
      }
      // The camera model is for CAMERA_WIDTH x CAMERA_HEIGHT; scale if the stream differs.
      double[] robot = pollenPixelToRobot(
        (clump.centerX * CAMERA_WIDTH) / frameWidth,
        (clump.centerY * CAMERA_HEIGHT) / frameHeight
      );
      if (robot == null) {
        continue;
      }
      if (
        Math.hypot(robot[0] - CAMERA_FORWARD_IN, robot[1] - CAMERA_LEFT_IN) >
        CAMERA_MAX_RANGE_IN
      ) {
        continue;
      }
      double fieldX = pose.x() + robot[0] * cos - robot[1] * sin;
      double fieldY = pose.y() + robot[0] * sin + robot[1] * cos;
      if (
        Math.hypot(fieldX - step.targetX, fieldY - step.targetY) >
        step.zoneRadius
      ) {
        continue;
      }
      if (POLLEN_MIDLINE_SIDE == 1 && fieldX > FIELD_MIDLINE_X) {
        continue;
      }
      if (POLLEN_MIDLINE_SIDE == 2 && fieldX < FIELD_MIDLINE_X) {
        continue;
      }
      double distance = Math.hypot(fieldX - pose.x(), fieldY - pose.y());
      if (
        clump.estimatedBallCount > bestCount ||
        (clump.estimatedBallCount == bestCount && distance < bestDistance)
      ) {
        best = new double[] { fieldX, fieldY };
        bestCount = clump.estimatedBallCount;
        bestDistance = distance;
      }
    }
    return best;
  }

  /**
   * Where a pixel lands on the TILES, in the robot frame (+x forward, +y left), for a
   * POLLEN centre POLLEN_RADIUS_IN above them. Null above the horizon. Mirrors
   * pixelToRobot in the visualizer's src/utils/pollenVision.ts.
   */
  private static double[] pollenPixelToRobot(double u, double v) {
    double pitch = Math.toRadians(CAMERA_PITCH_DEG);
    double yaw = Math.toRadians(CAMERA_YAW_DEG);
    double cosPitch = Math.cos(pitch);
    double sinPitch = Math.sin(pitch);
    double fx =
      CAMERA_WIDTH / 2.0 / Math.tan(Math.toRadians(CAMERA_HFOV_DEG) / 2.0);
    double fy =
      CAMERA_HEIGHT / 2.0 / Math.tan(Math.toRadians(CAMERA_VFOV_DEG) / 2.0);
    double xn = (u - CAMERA_WIDTH / 2.0) / fx;
    double yn = (v - CAMERA_HEIGHT / 2.0) / fy;

    // ray = xn * right + yn * down + forward, with right = (0, -1, 0),
    // down = (-sin pitch, 0, -cos pitch), forward = (cos pitch, 0, -sin pitch), then yawed.
    double rayX = -yn * sinPitch + cosPitch;
    double rayY = -xn;
    double rayZ = -yn * cosPitch - sinPitch;
    double yawedX = rayX * Math.cos(yaw) - rayY * Math.sin(yaw);
    double yawedY = rayX * Math.sin(yaw) + rayY * Math.cos(yaw);

    double drop = CAMERA_HEIGHT_IN - POLLEN_RADIUS_IN;
    if (rayZ >= -1e-9 || drop <= 0.0) {
      return null;
    }
    double t = drop / -rayZ;
    return new double[] {
      CAMERA_FORWARD_IN + t * yawedX,
      CAMERA_LEFT_IN + t * yawedY,
    };
  }

  private void advanceSequence() {
    sequenceIndex++;
    stepStarted = false;
  }

  private void updateTelemetry(String state) {
    Pose pose = follower.pose();

    telemetry.addData("State", state);
    telemetry.addData("Sequence", sequenceIndex);
    telemetry.addData("X", "%.2f", pose.x());
    telemetry.addData("Y", "%.2f", pose.y());
    telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));
    telemetry.update();
  }

  /**
   * A Pedro 3 path plus the per-path callbacks Pedro 2's PathBuilder had and Pedro 3 dropped.
   *
   * <p>Segment indexes count the paths of a chain (Paths.path(...)) from 0, as Pedro 2
   * attached a callback to the path it was added after. Each trigger keeps Pedro 2's rule:
   * <ul>
   *   <li>parametric - once the segment's DISTANCE completion reaches the value (Pedro 2's
   *       getPathCompletion()), at the parametric end, or once the follower has left it;
   *   <li>temporal - that many milliseconds after the segment began; dropped if the segment
   *       ends first, as Pedro 2 dropped it with its path;
   *   <li>pose - once the follower's curve parameter passes the point on the segment closest
   *       to the pose, or once the follower has left it.
   * </ul>
   */
  private static final class AutoPath {

    private static final int PARAMETRIC = 0;
    private static final int TEMPORAL = 1;
    private static final int POSE = 2;

    final Path path;
    private final List<PathSegment> segments;
    private final List<Trigger> triggers = new ArrayList<>();
    private int segmentIndex;
    private long segmentStartMs;

    AutoPath(Path path) {
      this.path = path;
      this.segments = path.getSegments();
    }

    AutoPath addParametricCallback(
      int segment,
      double completion,
      Runnable action
    ) {
      triggers.add(new Trigger(PARAMETRIC, segment, completion, action));
      return this;
    }

    AutoPath addTemporalCallback(int segment, long delayMs, Runnable action) {
      triggers.add(new Trigger(TEMPORAL, segment, delayMs, action));
      return this;
    }

    AutoPath addPoseCallback(
      int segment,
      Pose pose,
      Runnable action,
      double initialGuess
    ) {
      double t = segments
        .get(segment)
        .curve.closestParameter(pose.toVector2D(), initialGuess);
      triggers.add(new Trigger(POSE, segment, t, action));
      return this;
    }

    /** Call as the path is handed to the follower; Pedro 2 re-armed callbacks on every followPath. */
    void arm() {
      segmentIndex = 0;
      segmentStartMs = System.currentTimeMillis();
      for (Trigger trigger : triggers) {
        trigger.fired = false;
      }
    }

    /** Call once per loop, after follower.update(). */
    void update(Follower follower) {
      long now = System.currentTimeMillis();
      // Once the follower stops following this path, every segment counts as left.
      int index = follower.following() ? follower.pathIndex() : segments.size();
      if (index != segmentIndex) {
        segmentIndex = index;
        segmentStartMs = now;
      }

      PathSegment current = index < segments.size()
        ? follower.currentSegment()
        : null;
      double parameter = current != null
        ? Math.max(0.0, Math.min(1.0, follower.parametricCompletion()))
        : 1.0;
      double completion = current != null
        ? current.curve.pathCompletion(parameter)
        : 1.0;
      boolean atEnd = follower.atParametricEnd();

      for (Trigger trigger : triggers) {
        if (trigger.fired) {
          continue;
        }
        boolean left = trigger.segment < segmentIndex;
        boolean here = trigger.segment == segmentIndex;
        boolean ready;
        if (trigger.kind == TEMPORAL) {
          if (left) {
            trigger.fired = true;
            continue;
          }
          ready = here && now - segmentStartMs >= trigger.value;
        } else if (trigger.kind == POSE) {
          ready = left || (here && parameter >= trigger.value);
        } else {
          ready = left || (here && (atEnd || completion >= trigger.value));
        }
        if (ready) {
          trigger.fired = true;
          trigger.action.run();
        }
      }
    }

    private static final class Trigger {

      final int kind;
      final int segment;
      final double value;
      final Runnable action;
      boolean fired;

      Trigger(int kind, int segment, double value, Runnable action) {
        this.kind = kind;
        this.segment = segment;
        this.value = value;
        this.action = action;
      }
    }
  }
}
