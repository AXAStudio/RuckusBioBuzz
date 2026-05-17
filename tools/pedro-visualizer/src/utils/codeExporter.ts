import prettier from "prettier";
import type {
  Point,
  Line,
  BasePoint,
  PathChain,
  SequenceItem,
  PoseVariable,
  EventMarker,
} from "../types";
import { getCurvePoint } from "./math";

// Lazy-load Prettier's Java plugin; fall back gracefully if unavailable
let cachedJavaPlugin: any | null = null;
async function loadJavaPlugin() {
  if (cachedJavaPlugin !== null) return cachedJavaPlugin;
  const candidates = ["prettier/plugins/java.js", "prettier/plugins/java"];
  for (const path of candidates) {
    try {
      const mod = await import(path);
      cachedJavaPlugin = (mod as any).default ?? mod;
      return cachedJavaPlugin;
    } catch (err) {
      // ignore and try next
    }
  }
  cachedJavaPlugin = null;
  return null;
}

/**
 * Generate Java code from path data
 */
function sanitizeIdentifier(input: string | undefined, fallback: string): string {
  const cleaned = (input || "").replace(/[^a-zA-Z0-9]/g, "");
  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}${cleaned}`;
  return cleaned;
}

function sanitizeJavaConstantName(input: string | undefined, fallback: string): string {
  const cleaned = (input || "")
    .replace(/([a-z])([A-Z])/g, "$1_$2")
    .replace(/[^a-zA-Z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "")
    .toUpperCase();

  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}_${cleaned}`;
  return cleaned;
}

function uniqueJavaConstantName(baseName: string, usedNames: Set<string>): string {
  let candidate = baseName;
  let duplicateIndex = 2;

  while (usedNames.has(candidate)) {
    candidate = `${baseName}_${duplicateIndex}`;
    duplicateIndex++;
  }

  usedNames.add(candidate);
  return candidate;
}

function sanitizeClassName(input: string | undefined, fallback: string): string {
  const cleaned = sanitizeIdentifier(input, fallback);
  return cleaned.charAt(0).toUpperCase() + cleaned.slice(1);
}

function fixed(value: number): string {
  return value.toFixed(3);
}

function pathSpeed(line: Line | undefined): number {
  const speed = Number(line?.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

function normalizeEventMarkers(line: Line, pathIndex = 0): Required<EventMarker>[] {
  return (line.eventMarkers || []).map((marker, markerIndex) => {
    const position = Number(marker.position);
    const durationMs = Number(marker.durationMs ?? 0);

    return {
      id: marker.id || `path-${pathIndex + 1}-event-${markerIndex + 1}`,
      name: marker.name?.trim() || `Path ${pathIndex + 1} Event ${markerIndex + 1}`,
      position: Number.isFinite(position) ? Math.max(0, Math.min(1, position)) : 0.5,
      durationMs: Number.isFinite(durationMs) ? Math.max(0, Math.round(durationMs)) : 0,
    };
  });
}

function headingCurve(line: Line): number {
  if (line.endPoint.heading !== "linear") return 1;
  const curve = Number(line.endPoint.headingCurve ?? 1);
  if (!Number.isFinite(curve)) return 1;
  return Math.max(0.25, Math.min(4, curve));
}

function usesCurvedHeading(line: Line): boolean {
  return line.endPoint.heading === "linear" && Math.abs(headingCurve(line) - 1) > 0.001;
}

function pathStepHeadingDegrees(point: Point, pointRole: "start" | "end"): number {
  if (point.heading === "constant") {
    return point.degrees ?? 0;
  }

  if (point.heading === "linear") {
    return pointRole === "start" ? point.startDeg ?? 0 : point.endDeg ?? 0;
  }

  return 0;
}

function buildPathStepCode(name: string, point: Point, pointRole: "start" | "end"): string {
  return `private static final PathStep ${name} = new PathStep(${fixed(point.x)}, ${fixed(point.y)}, ${fixed(pathStepHeadingDegrees(point, pointRole))});`;
}

function buildPoseVariablePathStepCode(name: string, variable: PoseVariable): string {
  return `private static final PathStep ${name} = new PathStep(${fixed(Number(variable.x) || 0)}, ${fixed(Number(variable.y) || 0)}, ${fixed(Number(variable.heading) || 0)});`;
}

function buildPathSegmentCode(line: Line, startExpression: string): string {
  const headingTypeToFunctionName = {
    constant: "setConstantHeadingInterpolation",
    linear: "setLinearHeadingInterpolation",
    tangential: "setTangentHeadingInterpolation",
  };

  const controlPoints = line.controlPoints
    .map((point) => `new Pose(${point.x.toFixed(3)}, ${point.y.toFixed(3)})`)
    .join(",\n            ");

  const curveType = line.controlPoints.length === 0 ? "BezierLine" : "BezierCurve";

  const allPoints = controlPoints
    ? `${startExpression},\n            ${controlPoints},\n            new Pose(${line.endPoint.x.toFixed(3)}, ${line.endPoint.y.toFixed(3)})`
    : `${startExpression},\n            new Pose(${line.endPoint.x.toFixed(3)}, ${line.endPoint.y.toFixed(3)})`;

  const headingConfig =
    line.endPoint.heading === "constant"
      ? `Math.toRadians(${line.endPoint.degrees ?? 0})`
      : line.endPoint.heading === "linear"
        ? `Math.toRadians(${line.endPoint.startDeg ?? 0}), Math.toRadians(${line.endPoint.endDeg ?? 0})`
        : "";

  const reverseConfig = line.endPoint.reverse ? "\n          .setReversed()" : "";

  return `.addPath(
            new ${curveType}(
              ${allPoints}
            )
          )
          .${headingTypeToFunctionName[line.endPoint.heading]}(${headingConfig})${reverseConfig}`;
}

function buildTeamCodePathSegmentCode(
  line: Line,
  startExpression: string,
  endExpression: string,
  pathIndex = 0,
): string {
  const controlPoints = line.controlPoints
    .map((point) => `new Pose(${fixed(point.x)}, ${fixed(point.y)})`)
    .join(",\n              ");

  const curveType = line.controlPoints.length === 0 ? "BezierLine" : "BezierCurve";
  const allPoints = controlPoints
    ? `${startExpression},\n              ${controlPoints},\n              ${endExpression}`
    : `${startExpression},\n              ${endExpression}`;

  const headingCall =
    line.endPoint.heading === "constant"
      ? `.setConstantHeadingInterpolation(Math.toRadians(${fixed(line.endPoint.degrees ?? 0)}))`
      : usesCurvedHeading(line)
        ? `.setHeadingInterpolation(closestPoint -> interpolateHeading(Math.toRadians(${fixed(line.endPoint.startDeg ?? 0)}), Math.toRadians(${fixed(line.endPoint.endDeg ?? 0)}), closestPoint.getTValue(), ${fixed(headingCurve(line))}))`
        : line.endPoint.heading === "linear"
          ? `.setLinearHeadingInterpolation(Math.toRadians(${fixed(line.endPoint.startDeg ?? 0)}), Math.toRadians(${fixed(line.endPoint.endDeg ?? 0)}))`
          : `.setTangentHeadingInterpolation()`;

  const reverseConfig = line.endPoint.reverse ? "\n          .setReversed()" : "";
  const callbackConfig = normalizeEventMarkers(line, pathIndex)
    .map(
      (marker) =>
        `\n          .addParametricCallback(${fixed(marker.position)}, () -> startParallelEvent(${javaStringLiteral(marker.name)}, ${marker.durationMs}L))`,
    )
    .join("");

  return `.addPath(
            new ${curveType}(
              ${allPoints}
            )
          )
          ${headingCall}${reverseConfig}${callbackConfig}`;
}

function javaStringLiteral(value: string): string {
  return JSON.stringify(value);
}

function sanitizeJavaMethodSuffix(input: string, fallback: string): string {
  const words = (input || "")
    .replace(/[^a-zA-Z0-9]+/g, " ")
    .trim()
    .split(/\s+/)
    .filter(Boolean);

  const suffix = words
    .map((word) => word.charAt(0).toUpperCase() + word.slice(1))
    .join("");

  const cleaned = suffix.replace(/[^a-zA-Z0-9]/g, "");
  if (!cleaned) return fallback;
  return /^[0-9]/.test(cleaned) ? `${fallback}${cleaned}` : cleaned;
}

export async function generateTeamCodeAutoCode(
  startPoint: Point,
  lines: Line[],
  pathChains: PathChain[] = [],
  className = "GeneratedSwerveAuto",
  sequence: SequenceItem[] = [],
  poseVariables: PoseVariable[] = [],
): Promise<string> {
  const autoClassName = sanitizeClassName(className, "GeneratedSwerveAuto");
  const linesWithIds = lines.map((line, idx) => ({
    ...line,
    id: line.id || `line-${idx + 1}`,
  }));
  const lineById = new Map(linesWithIds.map((line) => [line.id!, line]));
  void pathChains;

  const poseVariablesById = new Map(poseVariables.map((variable) => [variable.id, variable]));
  const usedPoseVariableIds = new Set<string>();
  if (startPoint.poseVariableId) usedPoseVariableIds.add(startPoint.poseVariableId);
  linesWithIds.forEach((line) => {
    if (line.endPoint.poseVariableId) usedPoseVariableIds.add(line.endPoint.poseVariableId);
  });

  const usedConstantNames = new Set<string>();
  const poseVariableConstantById = new Map<string, string>();

  [...usedPoseVariableIds].forEach((poseVariableId, idx) => {
    const variable = poseVariablesById.get(poseVariableId);
    if (!variable) return;

    const baseName = `POSE_${sanitizeJavaConstantName(
      variable.name,
      `VARIABLE_${idx + 1}`,
    )}_STEP`;
    poseVariableConstantById.set(
      poseVariableId,
      uniqueJavaConstantName(baseName, usedConstantNames),
    );
  });

  const pointStepName = (point: Point, fallbackName: string) =>
    point.poseVariableId
      ? poseVariableConstantById.get(point.poseVariableId) || fallbackName
      : fallbackName;

  const pointStepExpression = (point: Point, fallbackName: string) =>
    `${pointStepName(point, fallbackName)}.toPose()`;

  const pathStepDeclarations: string[] = [];
  poseVariableConstantById.forEach((constantName, poseVariableId) => {
    const variable = poseVariablesById.get(poseVariableId);
    if (variable) {
      pathStepDeclarations.push(buildPoseVariablePathStepCode(constantName, variable));
    }
  });

  if (pointStepName(startPoint, "START_STEP") === "START_STEP") {
    pathStepDeclarations.push(buildPathStepCode("START_STEP", startPoint, "start"));
  }

  linesWithIds.forEach((line, idx) => {
    const fallbackName = `POINT_${idx + 1}`;
    if (pointStepName(line.endPoint, fallbackName) === fallbackName) {
      pathStepDeclarations.push(buildPathStepCode(fallbackName, line.endPoint, "end"));
    }
  });

  const pathStepDeclarationBlock = pathStepDeclarations.join("\n    ");

  const pathVariableByLineId = new Map(
    linesWithIds.map((line, idx) => [line.id!, `path${idx + 1}`]),
  );
  const chainFieldDeclarations = linesWithIds
    .map((_, idx) => `private PathChain path${idx + 1};`)
    .join("\n    ");

  const chainAssignments = linesWithIds
    .map((line, idx) => {
      const startExpression =
        idx <= 0
          ? pointStepExpression(startPoint, "START_STEP")
          : pointStepExpression(linesWithIds[idx - 1].endPoint, `POINT_${idx}`);
      const endExpression = pointStepExpression(line.endPoint, `POINT_${idx + 1}`);
      const segmentSnippet = buildTeamCodePathSegmentCode(
        line,
        startExpression,
        endExpression,
        idx,
      );

      return `path${idx + 1} = follower.pathBuilder()
          ${segmentSnippet}
          .build();`;
    })
    .join("\n\n      ");

  const sequenceItems =
    sequence.length > 0
      ? sequence
      : linesWithIds.map((line) => ({ kind: "path", lineId: line.id! }) as SequenceItem);
  const eventItems = sequenceItems.filter((item) => item.kind === "event");
  const eventMethods = new Map<string, { name: string; suffix: string }>();
  const pathEventMarkers = linesWithIds.flatMap((line, lineIndex) =>
    normalizeEventMarkers(line, lineIndex),
  );
  const parallelEventCapacity = Math.max(1, pathEventMarkers.length);

  function registerEventMethod(eventName: string, fallbackSuffix: string) {
    if (eventMethods.has(eventName)) {
      return;
    }

    const baseSuffix = sanitizeJavaMethodSuffix(eventName, fallbackSuffix);
    let suffix = baseSuffix;
    let duplicateIndex = 2;
    while ([...eventMethods.values()].some((event) => event.suffix === suffix)) {
      suffix = `${baseSuffix}${duplicateIndex}`;
      duplicateIndex++;
    }
    eventMethods.set(eventName, { name: eventName, suffix });
  }

  eventItems.forEach((item, idx) => {
    const eventName = item.name?.trim() || `Event ${idx + 1}`;
    registerEventMethod(eventName, `Event${idx + 1}`);
  });

  pathEventMarkers.forEach((marker, idx) => {
    registerEventMethod(marker.name, `PathEvent${idx + 1}`);
  });

  const sequenceCases = sequenceItems
    .map((item, idx) => {
      if (item.kind === "path") {
        const pathVariable = pathVariableByLineId.get(item.lineId);
        if (!pathVariable || !lineById.has(item.lineId)) {
          return `case ${idx}:
        advanceSequence();
        break;`;
        }

        return `case ${idx}:
        followPathStep(${pathVariable}, ${fixed(pathSpeed(lineById.get(item.lineId)))});
        break;`;
      }

      const durationMs = Math.max(0, Number(item.durationMs) || 0);
      if (item.kind === "wait") {
        return `case ${idx}:
        runWaitStep(${durationMs}L);
        break;`;
      }

      return `case ${idx}:
        runTimedEventStep(${javaStringLiteral(item.name || item.kind)}, ${durationMs}L);
        break;`;
    })
    .join("\n      ");
  const startEventCases = [...eventMethods.values()]
    .map((event) => `case ${javaStringLiteral(event.name)}:
        start${event.suffix}();
        break;`)
    .join("\n      ");
  const finishEventCases = [...eventMethods.values()]
    .map((event) => `case ${javaStringLiteral(event.name)}:
        finish${event.suffix}();
        break;`)
    .join("\n      ");
  const eventMethodStubs = [...eventMethods.values()]
    .map(
      (event) => `private void start${event.suffix}() {
        // TODO: start ${event.name} mechanism here.
    }

    private void finish${event.suffix}() {
        // TODO: stop ${event.name} mechanism here.
    }`,
    )
    .join("\n\n    ");

  const file = `package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "${autoClassName}", group = "Auto")
public class ${autoClassName} extends OpMode {
    ${pathStepDeclarationBlock}

    private Follower follower;
    ${chainFieldDeclarations}
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;
    private static final int PARALLEL_EVENT_CAPACITY = ${parallelEventCapacity};
    private final String[] activeParallelEventNames = new String[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventStartTimes = new long[PARALLEL_EVENT_CAPACITY];
    private final long[] activeParallelEventDurations = new long[PARALLEL_EVENT_CAPACITY];

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(${pointStepExpression(startPoint, "START_STEP")});

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
        resetParallelEvents();

        follower.setStartingPose(${pointStepExpression(startPoint, "START_STEP")});
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
      ${chainAssignments}
    }

    private void runSequence() {
        if (pathFinished) {
            return;
        }

        switch (sequenceIndex) {
      ${sequenceCases}
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

            if (now - activeParallelEventStartTimes[i] >= activeParallelEventDurations[i]) {
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

    private void clearParallelEvent(int index) {
        activeParallelEventNames[index] = null;
        activeParallelEventStartTimes[index] = 0L;
        activeParallelEventDurations[index] = 0L;
    }

    private void startEvent(String eventName) {
        switch (eventName) {
      ${startEventCases}
            default:
                break;
        }
    }

    private void finishEvent(String eventName) {
        switch (eventName) {
      ${finishEventCases}
            default:
                break;
        }
    }

    ${eventMethodStubs}

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
}`;

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(file, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return file;
  }
}

export async function generateJavaCode(
  startPoint: Point,
  lines: Line[],
  exportMode: "full" | "class" | "coordinates" = "class",
  pathChains: PathChain[] = [],
): Promise<string> {
  const linesWithIds = lines.map((line, idx) => ({
    ...line,
    id: line.id || `line-${idx + 1}`,
  }));
  const lineById = new Map(linesWithIds.map((line) => [line.id!, line]));

  const inputChains =
    pathChains.length > 0
      ? pathChains
      : linesWithIds.map((line, idx) => ({
          id: line.id!,
          name: line.name || `Path ${idx + 1}`,
          color: "#22c55e",
          lineIds: [line.id!],
        }));

  const normalizedChains: PathChain[] = inputChains
    .map((chain, idx) => ({
      ...chain,
      id: chain.id || `chain-${idx + 1}`,
      name: chain.name || `PathChain${idx + 1}`,
      lineIds: (chain.lineIds || []).filter((id) => lineById.has(id)),
    }))
    .filter((chain) => chain.lineIds.length > 0);

  const fieldDeclarations = normalizedChains
    .map((chain, idx) => {
      const variableName = sanitizeIdentifier(chain.name, `pathChain${idx + 1}`);
      return `public PathChain ${variableName};`;
    })
    .join("\n    ");

  const pathAssignments = normalizedChains
    .map((chain, chainIdx) => {
      const variableName = sanitizeIdentifier(chain.name, `pathChain${chainIdx + 1}`);

      const segmentSnippets = chain.lineIds
        .map((lineId) => {
          const line = lineById.get(lineId);
          if (!line) return null;

          const lineIndex = linesWithIds.findIndex((ln) => ln.id === line.id);
          const startExpression =
            lineIndex <= 0
              ? `new Pose(${startPoint.x.toFixed(3)}, ${startPoint.y.toFixed(3)})`
              : `new Pose(${linesWithIds[lineIndex - 1].endPoint.x.toFixed(3)}, ${linesWithIds[lineIndex - 1].endPoint.y.toFixed(3)})`;

          return buildPathSegmentCode(line, startExpression);
        })
        .filter((segment): segment is string => Boolean(segment));

      return `${variableName} = follower.pathBuilder()
          ${segmentSnippets.join("\n          ")}
          .build();`;
    })
    .join("\n\n      ");

  // If coordinates-only mode, return just the path assignments
  if (exportMode === "coordinates") {
    return pathAssignments;
  }

  const pathsClass = `public static class Paths {
    ${fieldDeclarations}

    public Paths(Follower follower) {
      ${pathAssignments}
    }
  }`;

  let file = "";
  if (exportMode === "class") {
    file = pathsClass;
  } else {
    file = `package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  ${pathsClass}

  public int autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return 0;
  }
}`;
  }

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(file, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return file;
  }
}

/**
 * Generate an array of waypoints (not sampled points) along the path
 */
export function generatePointsArray(startPoint: Point, lines: Line[]): string {
  const points: BasePoint[] = [];

  // Add start point
  points.push(startPoint);

  // Add all waypoints (end points and control points)
  lines.forEach((line) => {
    // Add control points for this line
    line.controlPoints.forEach((controlPoint) => {
      points.push(controlPoint);
    });

    // Add end point of this line
    points.push(line.endPoint);
  });

  // Format as string array, removing decimal places for whole numbers
  const pointsString = points
    .map((point) => {
      const x = Number.isInteger(point.x)
        ? point.x.toFixed(1)
        : point.x.toFixed(3);
      const y = Number.isInteger(point.y)
        ? point.y.toFixed(1)
        : point.y.toFixed(3);
      return `(${x}, ${y})`;
    })
    .join(", ");

  return `[${pointsString}]`;
}

/**
 * Generate Sequential Command code
 */
export async function generateSequentialCommandCode(
  startPoint: Point,
  lines: Line[],
  fileName: string | null = null,
  sequence?: SequenceItem[],
): Promise<string> {
  // Determine class name from file name or use default
  let className = "AutoPath";
  if (fileName) {
    const baseName = fileName.split(/[\\/]/).pop() || "";
    className = baseName.replace(".pp", "").replace(/[^a-zA-Z0-9]/g, "_");
    if (!className) className = "AutoPath";
  }

  // Collect all pose names including control points
  const allPoseDeclarations: string[] = [];
  const allPoseInitializations: string[] = [];

  // Track all pose variable names
  const poseVariableNames: Map<string, string> = new Map();

  // Add start point
  allPoseDeclarations.push("  private Pose startPoint;");
  poseVariableNames.set("startPoint", "startPoint");
  allPoseInitializations.push('    startPoint = pp.get("startPoint");');

  // Process each line
  lines.forEach((line, lineIdx) => {
    const endPointName = line.name
      ? line.name.replace(/[^a-zA-Z0-9]/g, "")
      : `point${lineIdx + 1}`;

    // Add end point declaration
    allPoseDeclarations.push(`  private Pose ${endPointName};`);
    poseVariableNames.set(`point${lineIdx + 1}`, endPointName);
    allPoseInitializations.push(
      `    ${endPointName} = pp.get(\"${endPointName}\");`,
    );

    // Add control points if they exist
    if (line.controlPoints && line.controlPoints.length > 0) {
      line.controlPoints.forEach((_, controlIdx) => {
        const controlPointName = `${endPointName}_control${controlIdx + 1}`;
        allPoseDeclarations.push(`  private Pose ${controlPointName};`);
        allPoseInitializations.push(
          `    ${controlPointName} = pp.get(\"${controlPointName}\");`,
        );
        // Store for use in path building
        poseVariableNames.set(
          `${endPointName}_control${controlIdx + 1}`,
          controlPointName,
        );
      });
    }
  });

  // Generate path chain declarations
  const pathChainDeclarations = lines
    .map((_, idx) => {
      const startPoseName =
        idx === 0
          ? "startPoint"
          : lines[idx - 1]?.name
            ? lines[idx - 1]!.name!.replace(/[^a-zA-Z0-9]/g, "")
            : `point${idx}`;
      const endPoseName = lines[idx].name
        ? lines[idx].name.replace(/[^a-zA-Z0-9]/g, "")
        : `point${idx + 1}`;
      const pathName = `${startPoseName}TO${endPoseName}`;
      return `  private PathChain ${pathName};`;
    })
    .join("\n");

  // Generate ProgressTracker field
  const progressTrackerField = `  private final ProgressTracker progressTracker;`;

  // Generate addCommands calls with event handling; iterate sequence if provided
  const commands: string[] = [];

  const defaultSequence: SequenceItem[] = lines.map((ln, idx) => ({
    kind: "path",
    lineId: ln.id || `line-${idx + 1}`,
  }));
  const seq = sequence && sequence.length ? sequence : defaultSequence;

  seq.forEach((item, idx) => {
    if (item.kind === "wait" || item.kind === "event") {
      commands.push(`        new WaitCommand(${(item as any).durationMs})`);
      return;
    }
    const lineIdx = lines.findIndex((l) => l.id === (item as any).lineId);
    if (lineIdx < 0) {
      return; // skip if sequence references a missing line
    }
    const line = lines[lineIdx];
    if (!line) {
      return;
    }
    const startPoseName =
      lineIdx === 0
        ? "startPoint"
        : lines[lineIdx - 1]?.name
          ? lines[lineIdx - 1]!.name!.replace(/[^a-zA-Z0-9]/g, "")
          : `point${lineIdx}`;
    const endPoseName = line.name
      ? line.name.replace(/[^a-zA-Z0-9]/g, "")
      : `point${lineIdx + 1}`;
    const pathName = `${startPoseName}TO${endPoseName}`;
    const pathDisplayName = `${startPoseName}TO${endPoseName}`;

    if (line.eventMarkers && line.eventMarkers.length > 0) {
      // Path has event markers - use reg.java style structure
      // First: InstantCommand to set up tracker
      commands.push(`        new InstantCommand(
            () -> {
              progressTracker.setCurrentChain(${pathName});
              progressTracker.setCurrentPathName("${pathDisplayName}");`);

      // Add event registrations
      line.eventMarkers.forEach((event) => {
        commands[commands.length - 1] += `
              progressTracker.registerEvent("${event.name}", ${event.position.toFixed(3)});`;
      });

      commands[commands.length - 1] += `
            })`;

      // Second: ParallelRaceGroup for following path with event handling
      commands.push(`        new ParallelRaceGroup(
            new FollowPathCommand(follower, ${pathName}),
            new SequentialCommandGroup(`);

      // Add WaitUntilCommand for each event
      line.eventMarkers.forEach((event, eventIdx) => {
        if (eventIdx > 0) commands[commands.length - 1] += ",";
        commands[commands.length - 1] += `
                new WaitUntilCommand(() -> progressTracker.shouldTriggerEvent("${event.name}")),
                new InstantCommand(
                    () -> {
                      progressTracker.executeEvent("${event.name}");
                    })`;
      });

      commands[commands.length - 1] += `
            ))`;
    } else {
      // No event markers - simple InstantCommand + FollowPathCommand
      commands.push(`        new InstantCommand(
            () -> {
              progressTracker.setCurrentChain(${pathName});
              progressTracker.setCurrentPathName("${pathDisplayName}");
            }),
        new FollowPathCommand(follower, ${pathName})`);
    }
  });

  // Generate path building
  const pathBuilders = lines
    .map((line, idx) => {
      const startPoseName =
        idx === 0
          ? "startPoint"
          : lines[idx - 1]?.name
            ? lines[idx - 1]!.name!.replace(/[^a-zA-Z0-9]/g, "")
            : `point${idx}`;
      const endPoseName = line.name
        ? line.name.replace(/[^a-zA-Z0-9]/g, "")
        : `point${idx + 1}`;
      const pathName = `${startPoseName}TO${endPoseName}`;

      const isCurve = line.controlPoints.length > 0;
      const curveType = isCurve ? "BezierCurve" : "BezierLine";

      // Build control points string
      let controlPointsStr = "";
      if (isCurve) {
        const controlPoints: string[] = [];
        line.controlPoints.forEach((_, cpIdx) => {
          const controlPointName = `${endPoseName}_control${cpIdx + 1}`;
          controlPoints.push(controlPointName);
        });
        controlPointsStr = controlPoints.join(", ") + ", ";
      }

      // Determine heading interpolation
      let headingConfig = "";
      if (line.endPoint.heading === "constant") {
        headingConfig = `setConstantHeadingInterpolation(${endPoseName}.getHeading())`;
      } else if (line.endPoint.heading === "linear") {
        headingConfig = `setLinearHeadingInterpolation(${startPoseName}.getHeading(), ${endPoseName}.getHeading())`;
      } else {
        headingConfig = `setTangentHeadingInterpolation()`;
      }

      // Build reverse config
      const reverseConfig = line.endPoint.reverse
        ? "\n            .setReversed()"
        : "";

      return `${pathName} =
        follower
            .pathBuilder()
            .addPath(new ${curveType}(${startPoseName}, ${controlPointsStr}${endPoseName}))
            .${headingConfig}${reverseConfig}
            .build();`;
    })
    .join("\n\n    ");

  const sequentialCommandCode = `
package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Pathing.ProgressTracker;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.PedroPathReader;

public class ${className} extends SequentialCommandGroup {

  private final Follower follower;
  ${progressTrackerField}

  // Poses
${allPoseDeclarations.join("\n")}

  // Path chains
${pathChainDeclarations}

  public ${className}(final Drivetrain drive, HardwareMap hw, Telemetry telemetry) throws IOException {
    this.follower = drive.getFollower();
    this.progressTracker = new ProgressTracker(follower, telemetry);

    PedroPathReader pp = new PedroPathReader("${fileName ? fileName.split(/[\\/]/).pop() + ".pp" || "AutoPath.pp" : "AutoPath.pp"}", hw.appContext);

    // Load poses
${allPoseInitializations.join("\n")}

    follower.setStartingPose(startPoint);

    buildPaths();

    addCommands(
${commands.join(",\n")});
  }

  public void buildPaths() {
    ${pathBuilders}
  }
}
`;

  try {
    const javaPlugin = await loadJavaPlugin();
    const formattedCode = await prettier.format(sequentialCommandCode, {
      parser: "java",
      plugins: javaPlugin ? [javaPlugin] : [],
    });
    return formattedCode;
  } catch (error) {
    console.error("Code formatting error:", error);
    return sequentialCommandCode;
  }
}
