package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveDrivetrainConstants;

@Autonomous(name = "Full18Auto", group = "Auto")
public class Full18Auto extends OpMode {

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
  private AutoPath path1;
  private AutoPath path2;
  private AutoPath path3;
  private AutoPath path4;
  private AutoPath path5;
  private AutoPath path6;
  private AutoPath[] repeat3Paths;
  private double[] repeat3PathSpeeds;
  private AutoPath activePath;
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
    // Pedro 3 follows paths with Foresight, whose braking model and gains have not been
    // measured on this robot yet. Refuse to build a path-following OpMode on placeholders.
    SwerveDrivetrainConstants.requireForesightMeasured();

    follower = Constants.createFollower(hardwareMap);
    follower.setPose(START_STEP.toPose());

    buildPaths();
    updateTelemetry("Initialized");
  }

  @Override
  public void init_loop() {
    // Pedro 2's update() with no path only refreshed the pose. Pedro 3's update() in IDLE
    // calls drivetrain.stop(), which writes every servo and zero-power behaviour each loop,
    // so refresh the localizer alone to keep init actuator-silent as before.
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
    // Pedro 2 ran path callbacks inside follower.update(); keep them at the same point.
    if (activePath != null) {
      activePath.fireReadyCallbacks(follower);
    }
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

    follower.manual(DrivePowers.zero());
    follower.update();
  }

  private void buildPaths() {
    path1 = new AutoPath(
      Paths.curve(
        START_STEP.toPose(),
        new Pose(90.924, 67.605),
        new Pose(53.701, 49.043),
        new Pose(11.696, 60.121),
        POINT_1.toPose()
      ).heading((curve, t) ->
        interpolateHeading(
          Math.toRadians(142.000),
          Math.toRadians(180.000),
          t,
          0.700
        )
      )
    )
      .addParametricCallback(0.010, () -> startParallelEvent("Shoot", 1800L))
      .addParametricCallback(0.450, () -> startParallelEvent("Intake", 3000L));

    path2 = new AutoPath(
      Paths.curve(
        POINT_1.toPose(),
        new Pose(52.397, 55.795),
        POINT_2.toPose()
      ).heading(
        linearHeading(Math.toRadians(180.000), Math.toRadians(180.000))
      )
    ).addParametricCallback(0.670, () -> startParallelEvent("Shoot", 2200L));

    path3 = new AutoPath(
      Paths.curve(
        POINT_2.toPose(),
        new Pose(55.957, 63.278),
        new Pose(0.000, 66.334),
        new Pose(12.057, 58.852),
        POSE_GATE_INTAKE_STEP.toPose()
      ).heading(
        linearHeading(Math.toRadians(150.000), Math.toRadians(150.000))
      )
    ).addParametricCallback(0.400, () -> startParallelEvent("Intake", 2500L));

    path4 = new AutoPath(
      Paths.curve(
        POSE_GATE_INTAKE_STEP.toPose(),
        new Pose(40.276, 69.600),
        POINT_4.toPose()
      ).heading(
        linearHeading(Math.toRadians(150.000), Math.toRadians(150.000))
      )
    ).addParametricCallback(0.710, () -> startParallelEvent("Shoot", 1780L));

    path5 = new AutoPath(
      Paths.curve(
        POINT_4.toPose(),
        new Pose(41.584, 81.247),
        POINT_5.toPose()
      ).heading((curve, t) ->
        interpolateHeading(
          Math.toRadians(150.000),
          Math.toRadians(180.000),
          t,
          0.250
        )
      )
    )
      .addParametricCallback(0.220, () -> startParallelEvent("Shoot", 400L))
      .addParametricCallback(0.530, () -> startParallelEvent("Intake", 1700L));

    path6 = new AutoPath(
      Paths.line(POINT_5.toPose(), POINT_6.toPose()).heading(
        linearHeading(Math.toRadians(180.000), Math.toRadians(180.000))
      )
    ).addParametricCallback(0.430, () -> startParallelEvent("Shoot", 0L));

    repeat3Paths = new AutoPath[] { path3, path4 };
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
        activePath = null;
        finishAllParallelEvents();
        follower.manual(DrivePowers.zero());
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

  /**
   * Pedro 2's setLinearHeadingInterpolation: shortest-way linear in the curve PARAMETER t.
   * Pedro 3's Path.linear() interpolates on curve.pathCompletion(t) - distance fraction - instead,
   * which differs on any Bezier whose speed along t is not uniform. (Upstream 3.0.0's default
   * pathCompletion also ran backwards on a Line; patched in the vendored tree, see
   * RUCKUS_PATCHES.md.) This keeps the Pedro 2 behaviour exactly.
   */
  private static Interpolator linearHeading(
    double startHeading,
    double endHeading
  ) {
    double start = Angle.normalize(startHeading);
    double end = Angle.normalize(endHeading);
    double delta =
      Angle.turnDirection(start, end) * Angle.smallestDifference(end, start);
    return (curve, t) -> Angle.normalize(start + delta * t);
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

  private void followRepeatStep(
    AutoPath[] repeatPaths,
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
    AutoPath path = repeatPaths[pathIndex];
    double pathSpeed = repeatPathSpeeds != null &&
      pathIndex < repeatPathSpeeds.length
      ? repeatPathSpeeds[pathIndex]
      : 1.0;

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

  /** Pedro 2's followPath(path, maxPower, holdEnd=true), callbacks re-armed as it did. */
  private void startPath(AutoPath path, double pathSpeed) {
    path.armCallbacks();
    activePath = path;
    follower.holdEnd.set(true);
    follower.follow(withPathSpeed(path.path, clampPathSpeed(pathSpeed)));
  }

  /**
   * Pedro 2 capped drive POWER per path. Pedro 3 has no power cap; the nearest thing is
   * Foresight's maxPathSpeed, a fraction of max achievable VELOCITY, applied while this path is
   * followed. Every call site passes 1.0, where no modifier is applied at all.
   */
  private Path withPathSpeed(Path path, double pathSpeed) {
    if (pathSpeed >= 1.0 || !(follower.algorithm() instanceof Foresight)) {
      return path;
    }
    return path.with(
      ((Foresight) follower.algorithm()).config.maxPathSpeed.at(pathSpeed)
    );
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
    Pose pose = follower.pose();

    telemetry.addData("State", state);
    telemetry.addData("Sequence", sequenceIndex);
    telemetry.addData("X", "%.2f", pose.x());
    telemetry.addData("Y", "%.2f", pose.y());
    telemetry.addData("Heading", "%.2f", Math.toDegrees(pose.heading()));
    telemetry.update();
  }

  /**
   * A path plus Pedro 2's parametric callbacks, which Pedro 3 dropped.
   *
   * <p>Pedro 2's ParametricCallback fired once per followPath when
   * {@code atParametricEnd() || getPathCompletion() >= t}, where getPathCompletion() is the
   * DISTANCE fraction along the curve at the closest point - not the raw parameter. This keeps
   * that trigger: arc-length completion from the current curve's remaining distance at the
   * follower's closest parameter. (Pedro 3's Follower.completion() is not used: it goes through
   * the default Curve.pathCompletion, which returns the fraction remaining on a Line.)
   */
  private static final class AutoPath {

    final Path path;
    private final List<Double> triggers = new ArrayList<>();
    private final List<Runnable> actions = new ArrayList<>();
    private boolean[] fired = new boolean[0];

    AutoPath(Path path) {
      this.path = path;
    }

    AutoPath addParametricCallback(double completion, Runnable action) {
      triggers.add(completion);
      actions.add(action);
      fired = new boolean[triggers.size()];
      return this;
    }

    void armCallbacks() {
      fired = new boolean[triggers.size()];
    }

    void fireReadyCallbacks(Follower follower) {
      double completion = follower.atParametricEnd()
        ? 1.0
        : distanceCompletion(follower);
      for (int i = 0; i < triggers.size(); i++) {
        if (!fired[i] && completion >= triggers.get(i)) {
          fired[i] = true;
          actions.get(i).run();
        }
      }
    }

    private static double distanceCompletion(Follower follower) {
      Curve curve = follower.currentCurve();
      if (curve == null) {
        return 1.0;
      }
      double length = curve.length();
      if (length <= 0) {
        return 1.0;
      }
      return (
        1.0 - curve.remainingDistance(follower.parametricCompletion()) / length
      );
    }
  }
}
