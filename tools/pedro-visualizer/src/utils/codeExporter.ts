import prettier from "prettier";
import type {
  Point,
  Line,
  BasePoint,
  PathChain,
  SequenceItem,
  PoseVariable,
  EventMarker,
  NumberVariable,
} from "../types";
import { buildJavaExpressionFromNumberExpression } from "./numberExpressions";
import { getCurvePoint } from "./math";

// Lazy-load Prettier's Java plugin; fall back gracefully if unavailable
let cachedJavaPlugin: any | null = null;
async function loadJavaPlugin() {
  if (cachedJavaPlugin !== null) return cachedJavaPlugin;
  const candidates = [() => import("prettier-plugin-java")];
  for (const loadPlugin of candidates) {
    try {
      const mod = await loadPlugin();
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
function sanitizeIdentifier(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = (input || "").replace(/[^a-zA-Z0-9]/g, "");
  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}${cleaned}`;
  return cleaned;
}

function sanitizeJavaConstantName(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = (input || "")
    .replace(/([a-z])([A-Z])/g, "$1_$2")
    .replace(/[^a-zA-Z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "")
    .toUpperCase();

  if (!cleaned) return fallback;
  if (/^[0-9]/.test(cleaned)) return `${fallback}_${cleaned}`;
  return cleaned;
}

function uniqueJavaConstantName(
  baseName: string,
  usedNames: Set<string>,
): string {
  let candidate = baseName;
  let duplicateIndex = 2;

  while (usedNames.has(candidate)) {
    candidate = `${baseName}_${duplicateIndex}`;
    duplicateIndex++;
  }

  usedNames.add(candidate);
  return candidate;
}

function sanitizeClassName(
  input: string | undefined,
  fallback: string,
): string {
  const cleaned = sanitizeIdentifier(input, fallback);
  return cleaned.charAt(0).toUpperCase() + cleaned.slice(1);
}

function fixed(value: number): string {
  return value.toFixed(3);
}

function pathSpeedValue(line: Line | undefined): number {
  const speed = Number(line?.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

function buildNumberVariableCode(
  name: string,
  variable: NumberVariable,
): string {
  const value = Number(variable.value);
  return `private static final double ${name} = ${fixed(Number.isFinite(value) ? value : 0)};`;
}

type NumberExpressionType = "double" | "int" | "long" | "position";

type NormalizedEventMarker = {
  id: string;
  name: string;
  triggerType: "parametric" | "temporal" | "pose";
  position: number;
  positionVariableId?: string;
  triggerMs: number;
  triggerMsVariableId?: string;
  poseX: number;
  poseXVariableId?: string;
  poseY: number;
  poseYVariableId?: string;
  durationMs: number;
  durationVariableId?: string;
};

function normalizeEventMarkers(
  line: Line,
  pathIndex = 0,
): NormalizedEventMarker[] {
  return (line.eventMarkers || []).map((marker, markerIndex) => {
    const position = Number(marker.position);
    const triggerMs = Number(marker.triggerMs ?? 0);
    const durationMs = Number(marker.durationMs ?? 0);
    const triggerType =
      marker.triggerType === "temporal" || marker.triggerType === "pose"
        ? marker.triggerType
        : "parametric";

    return {
      id: marker.id || `path-${pathIndex + 1}-event-${markerIndex + 1}`,
      name:
        marker.name?.trim() || `Path ${pathIndex + 1} Event ${markerIndex + 1}`,
      triggerType,
      position: Number.isFinite(position)
        ? Math.max(0, Math.min(1, position))
        : 0.5,
      positionVariableId: marker.positionVariableId,
      triggerMs: Number.isFinite(triggerMs)
        ? Math.max(0, Math.round(triggerMs))
        : 0,
      triggerMsVariableId: marker.triggerMsVariableId,
      poseX: Number.isFinite(Number(marker.poseX))
        ? Number(marker.poseX)
        : Number(line.endPoint.x) || 0,
      poseXVariableId: marker.poseXVariableId,
      poseY: Number.isFinite(Number(marker.poseY))
        ? Number(marker.poseY)
        : Number(line.endPoint.y) || 0,
      poseYVariableId: marker.poseYVariableId,
      durationMs: Number.isFinite(durationMs)
        ? Math.max(0, Math.round(durationMs))
        : 0,
      durationVariableId: marker.durationVariableId,
    };
  });
}

function buildTeamCodeCallback(
  marker: NormalizedEventMarker,
  numberExpression: (
    variableId: string | undefined,
    fallbackExpression: string,
    expressionType: NumberExpressionType,
  ) => string,
): string {
  const action = `() -> startParallelEvent(${javaStringLiteral(marker.name)}, ${numberExpression(marker.durationVariableId, `${marker.durationMs}L`, "long")})`;

  if (marker.triggerType === "temporal") {
    return `.addTemporalCallback(${numberExpression(marker.triggerMsVariableId, `${marker.triggerMs}L`, "long")}, ${action})`;
  }

  if (marker.triggerType === "pose") {
    return `.addPoseCallback(new Pose(${numberExpression(marker.poseXVariableId, fixed(marker.poseX), "double")}, ${numberExpression(marker.poseYVariableId, fixed(marker.poseY), "double")}), ${action}, ${numberExpression(marker.positionVariableId, fixed(marker.position), "position")})`;
  }

  return `.addParametricCallback(${numberExpression(marker.positionVariableId, fixed(marker.position), "position")}, ${action})`;
}

function headingCurve(line: Line): number {
  if (line.endPoint.heading !== "linear") return 1;
  const curve = Number(line.endPoint.headingCurve ?? 1);
  if (!Number.isFinite(curve)) return 1;
  return Math.max(0.25, Math.min(4, curve));
}

function usesCurvedHeading(line: Line): boolean {
  return (
    line.endPoint.heading === "linear" &&
    Math.abs(headingCurve(line) - 1) > 0.001
  );
}

function pathStepHeadingDegrees(
  point: Point,
  pointRole: "start" | "end",
): number {
  if (point.heading === "constant") {
    return point.degrees ?? 0;
  }

  if (point.heading === "linear") {
    return pointRole === "start" ? (point.startDeg ?? 0) : (point.endDeg ?? 0);
  }

  return 0;
}

function buildPathStepCode(
  name: string,
  point: Point,
  pointRole: "start" | "end",
  numberVariables: NumberVariable[],
  numberVariableConstantById: Map<string, string>,
): string {
  const xExpression = buildJavaExpressionFromNumberExpression(
    point.xExpression,
    fixed(point.x),
    numberVariables,
    numberVariableConstantById,
  );
  const yExpression = buildJavaExpressionFromNumberExpression(
    point.yExpression,
    fixed(point.y),
    numberVariables,
    numberVariableConstantById,
  );
  const headingFallback = fixed(pathStepHeadingDegrees(point, pointRole));
  const headingExpression = point.heading === "constant"
    ? buildJavaExpressionFromNumberExpression(
        point.degreesExpression,
        headingFallback,
        numberVariables,
        numberVariableConstantById,
      )
    : point.heading === "linear"
      ? buildJavaExpressionFromNumberExpression(
          pointRole === "start" ? point.startDegExpression : point.endDegExpression,
          headingFallback,
          numberVariables,
          numberVariableConstantById,
        )
      : headingFallback;
  return `private static final PathStep ${name} = new PathStep(${xExpression}, ${yExpression}, ${headingExpression});`;
}

function buildPoseVariablePathStepCode(
  name: string,
  variable: PoseVariable,
  numberVariables: NumberVariable[],
  numberVariableConstantById: Map<string, string>,
): string {
  const xExpression = buildJavaExpressionFromNumberExpression(
    variable.xExpression,
    fixed(Number(variable.x) || 0),
    numberVariables,
    numberVariableConstantById,
  );
  const yExpression = buildJavaExpressionFromNumberExpression(
    variable.yExpression,
    fixed(Number(variable.y) || 0),
    numberVariables,
    numberVariableConstantById,
  );
  const headingExpression = buildJavaExpressionFromNumberExpression(
    variable.headingExpression,
    fixed(Number(variable.heading) || 0),
    numberVariables,
    numberVariableConstantById,
  );
  return `private static final PathStep ${name} = new PathStep(${xExpression}, ${yExpression}, ${headingExpression});`;
}

function buildPathSegmentCode(
  line: Line,
  startExpression: string,
  numberVariables: NumberVariable[],
  numberVariableConstantById: Map<string, string>,
): string {
  const headingTypeToFunctionName = {
    constant: "setConstantHeadingInterpolation",
    linear: "setLinearHeadingInterpolation",
    tangential: "setTangentHeadingInterpolation",
  };

  const controlPoints = line.controlPoints
    .map((point) => `new Pose(${point.x.toFixed(3)}, ${point.y.toFixed(3)})`)
    .join(",\n            ");

  const curveType =
    line.controlPoints.length === 0 ? "BezierLine" : "BezierCurve";

  const endXExpression = buildJavaExpressionFromNumberExpression(
    line.endPoint.xExpression,
    fixed(line.endPoint.x),
    numberVariables,
    numberVariableConstantById,
  );
  const endYExpression = buildJavaExpressionFromNumberExpression(
    line.endPoint.yExpression,
    fixed(line.endPoint.y),
    numberVariables,
    numberVariableConstantById,
  );

  const allPoints = controlPoints
    ? `${startExpression},\n            ${controlPoints},\n            new Pose(${endXExpression}, ${endYExpression})`
    : `${startExpression},\n            new Pose(${endXExpression}, ${endYExpression})`;

  const headingConfig =
    line.endPoint.heading === "constant"
      ? `Math.toRadians(${buildJavaExpressionFromNumberExpression(
          line.endPoint.degreesExpression,
          fixed(line.endPoint.degrees ?? 0),
          numberVariables,
          numberVariableConstantById,
        )})`
      : line.endPoint.heading === "linear"
        ? `Math.toRadians(${buildJavaExpressionFromNumberExpression(
            line.endPoint.startDegExpression,
            fixed(line.endPoint.startDeg ?? 0),
            numberVariables,
            numberVariableConstantById,
          )}), Math.toRadians(${buildJavaExpressionFromNumberExpression(
            line.endPoint.endDegExpression,
            fixed(line.endPoint.endDeg ?? 0),
            numberVariables,
            numberVariableConstantById,
          )})`
        : "";

  const reverseConfig = line.endPoint.reverse
    ? "\n          .setReversed()"
    : "";

  return `.addPath(
            new ${curveType}(
              ${allPoints}
            )
          )
          .${headingTypeToFunctionName[line.endPoint.heading]}(${headingConfig})${reverseConfig}`;
}

function buildPoseExpression(
  point: { x: number; y: number; xExpression?: string; yExpression?: string },
  numberVariables: NumberVariable[],
  numberVariableConstantById: Map<string, string>,
): string {
  const xExpression = buildJavaExpressionFromNumberExpression(
    point.xExpression,
    fixed(point.x),
    numberVariables,
    numberVariableConstantById,
  );
  const yExpression = buildJavaExpressionFromNumberExpression(
    point.yExpression,
    fixed(point.y),
    numberVariables,
    numberVariableConstantById,
  );
  return `new Pose(${xExpression}, ${yExpression})`;
}

function buildTeamCodePathSegmentCode(
  line: Line,
  startExpression: string,
  endExpression: string,
  pathIndex = 0,
  numberExpression: (
    variableId: string | undefined,
    fallbackExpression: string,
    expressionType: NumberExpressionType,
  ) => string,
): string {
  const controlPoints = line.controlPoints
    .map((point) => `new Pose(${fixed(point.x)}, ${fixed(point.y)})`)
    .join(",\n              ");

  const curveType =
    line.controlPoints.length === 0 ? "BezierLine" : "BezierCurve";
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

  const reverseConfig = line.endPoint.reverse
    ? "\n          .setReversed()"
    : "";
  const callbackConfig = normalizeEventMarkers(line, pathIndex)
    .map(
      (marker) =>
        `\n          ${buildTeamCodeCallback(marker, numberExpression)}`,
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
  numberVariables: NumberVariable[] = [],
): Promise<string> {
  const autoClassName = sanitizeClassName(className, "GeneratedSwerveAuto");
  const linesWithIds = lines.map((line, idx) => ({
    ...line,
    id: line.id || `line-${idx + 1}`,
  }));
  const lineById = new Map(linesWithIds.map((line) => [line.id!, line]));
  void pathChains;

  const poseVariablesById = new Map(
    poseVariables.map((variable) => [variable.id, variable]),
  );
  const usedPoseVariableIds = new Set<string>();
  if (startPoint.poseVariableId)
    usedPoseVariableIds.add(startPoint.poseVariableId);
  linesWithIds.forEach((line) => {
    if (line.endPoint.poseVariableId)
      usedPoseVariableIds.add(line.endPoint.poseVariableId);
  });

  const usedConstantNames = new Set<string>();
  const poseVariableConstantById = new Map<string, string>();
  const numberVariableConstantById = new Map<string, string>();

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

  numberVariables.forEach((variable, idx) => {
    const baseName = `NUMBER_${sanitizeJavaConstantName(
      variable.name,
      `VARIABLE_${idx + 1}`,
    )}`;
    numberVariableConstantById.set(
      variable.id,
      uniqueJavaConstantName(baseName, usedConstantNames),
    );
  });

  const numberExpression = (
    variableId: string | undefined,
    fallbackExpression: string,
    expressionType: NumberExpressionType,
  ): string => {
    const constantName = variableId
      ? numberVariableConstantById.get(variableId)
      : undefined;
    if (!constantName) return fallbackExpression;
    if (expressionType === "position") {
      const value = Number(
        numberVariables.find((variable) => variable.id === variableId)?.value,
      );
      return Number.isFinite(value) && value > 1
        ? `(${constantName} / 100.0)`
        : constantName;
    }
    if (expressionType === "int") return `(int) Math.round(${constantName})`;
    if (expressionType === "long") return `Math.round(${constantName})`;
    return constantName;
  };

  const pathSpeedExpression = (line: Line | undefined): string =>
    numberExpression(
      line?.speedVariableId,
      fixed(pathSpeedValue(line)),
      "double",
    );

  const pointStepName = (point: Point, fallbackName: string) =>
    point.poseVariableId
      ? poseVariableConstantById.get(point.poseVariableId) || fallbackName
      : fallbackName;

  const pointStepExpression = (point: Point, fallbackName: string) =>
    `${pointStepName(point, fallbackName)}.toPose()`;

  const pathStepDeclarations: string[] = [];
  numberVariableConstantById.forEach((constantName, variableId) => {
    const variable = numberVariables.find((item) => item.id === variableId);
    if (variable) {
      pathStepDeclarations.push(
        buildNumberVariableCode(constantName, variable),
      );
    }
  });

  poseVariableConstantById.forEach((constantName, poseVariableId) => {
    const variable = poseVariablesById.get(poseVariableId);
    if (variable) {
      pathStepDeclarations.push(
        buildPoseVariablePathStepCode(
          constantName,
          variable,
          numberVariables,
          numberVariableConstantById,
        ),
      );
    }
  });

  if (pointStepName(startPoint, "START_STEP") === "START_STEP") {
    pathStepDeclarations.push(
      buildPathStepCode(
        "START_STEP",
        startPoint,
        "start",
        numberVariables,
        numberVariableConstantById,
      ),
    );
  }

  linesWithIds.forEach((line, idx) => {
    const fallbackName = `POINT_${idx + 1}`;
    if (pointStepName(line.endPoint, fallbackName) === fallbackName) {
      pathStepDeclarations.push(
        buildPathStepCode(
          fallbackName,
          line.endPoint,
          "end",
          numberVariables,
          numberVariableConstantById,
        ),
      );
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
      const endExpression = pointStepExpression(
        line.endPoint,
        `POINT_${idx + 1}`,
      );
      const segmentSnippet = buildTeamCodePathSegmentCode(
        line,
        startExpression,
        endExpression,
        idx,
        numberExpression,
      );

      return `path${idx + 1} = follower.pathBuilder()
          ${segmentSnippet}
          .build();`;
    })
    .join("\n\n      ");

  const sequenceItems =
    sequence.length > 0
      ? sequence
      : linesWithIds.map(
          (line) => ({ kind: "path", lineId: line.id! }) as SequenceItem,
        );
  const repeatItems = sequenceItems
    .map((item, sequenceIndex) => ({ item, sequenceIndex }))
    .filter(
      (
        entry,
      ): entry is {
        item: Extract<SequenceItem, { kind: "repeat" }>;
        sequenceIndex: number;
      } =>
        entry.item.kind === "repeat" &&
        (entry.item.lineIds || []).some((lineId) => lineById.has(lineId)),
    );
  const repeatSlotBySequenceIndex = new Map(
    repeatItems.map((entry, repeatSlot) => [entry.sequenceIndex, repeatSlot]),
  );
  const repeatFieldDeclarations = repeatItems
    .map((entry) => {
      const fieldName = `repeat${entry.sequenceIndex + 1}`;
      return `private PathChain[] ${fieldName}Paths;
    private double[] ${fieldName}PathSpeeds;`;
    })
    .join("\n    ");
  const repeatAssignments = repeatItems
    .map((entry) => {
      const fieldName = `repeat${entry.sequenceIndex + 1}`;
      const validLineIds = (entry.item.lineIds || []).filter((lineId) =>
        lineById.has(lineId),
      );
      const pathList = validLineIds
        .map((lineId) => pathVariableByLineId.get(lineId))
        .filter(Boolean)
        .join(", ");
      const speedList = validLineIds
        .map((lineId) => pathSpeedExpression(lineById.get(lineId)))
        .join(", ");
      return `${fieldName}Paths = new PathChain[] { ${pathList} };
      ${fieldName}PathSpeeds = new double[] { ${speedList} };`;
    })
    .join("\n\n      ");
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
    while (
      [...eventMethods.values()].some((event) => event.suffix === suffix)
    ) {
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
        followPathStep(${pathVariable}, ${pathSpeedExpression(lineById.get(item.lineId))});
        break;`;
      }

      if (item.kind === "wait") {
        return `case ${idx}:
        runWaitStep(${numberExpression(item.durationVariableId, `${Math.max(0, Number(item.durationMs) || 0)}L`, "long")});
        break;`;
      }

      if (item.kind === "repeat") {
        const repeatSlot = repeatSlotBySequenceIndex.get(idx);
        if (repeatSlot === undefined) {
          return `case ${idx}:
        advanceSequence();
        break;`;
        }
        const fieldName = `repeat${idx + 1}`;
        return `case ${idx}:
        followRepeatStep(${fieldName}Paths, ${fieldName}PathSpeeds, ${numberExpression(item.countVariableId, `${Math.max(1, Math.min(20, Math.round(Number(item.count) || 1)))}`, "int")}, ${repeatSlot});
        break;`;
      }

      return `case ${idx}:
        runTimedEventStep(${javaStringLiteral(item.name || item.kind)}, ${numberExpression(item.durationVariableId, `${Math.max(0, Number(item.durationMs) || 0)}L`, "long")});
        break;`;
    })
    .join("\n      ");
  const startEventCases = [...eventMethods.values()]
    .map(
      (event) => `case ${javaStringLiteral(event.name)}:
        start${event.suffix}();
        break;`,
    )
    .join("\n      ");
  const finishEventCases = [...eventMethods.values()]
    .map(
      (event) => `case ${javaStringLiteral(event.name)}:
        finish${event.suffix}();
        break;`,
    )
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
    ${repeatFieldDeclarations}
    private int sequenceIndex;
    private long stepStartTime;
    private boolean stepStarted;
    private boolean pathFinished;
    private static final int REPEAT_LOOP_CAPACITY = ${Math.max(1, repeatItems.length)};
    private final int[] repeatLoopIterations = new int[REPEAT_LOOP_CAPACITY];
    private final int[] repeatLoopPathIndexes = new int[REPEAT_LOOP_CAPACITY];
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
        resetRepeatLoops();
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

      ${repeatAssignments}
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

        int pathIndex = Math.max(0, Math.min(repeatPaths.length - 1, repeatLoopPathIndexes[repeatSlot]));
        PathChain path = repeatPaths[pathIndex];
        double pathSpeed =
            repeatPathSpeeds != null && pathIndex < repeatPathSpeeds.length
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
  numberVariables: NumberVariable[] = [],
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

  const numberVariableConstantById = new Map(
    numberVariables.map((variable) => [
      variable.id,
      fixed(Number.isFinite(Number(variable.value)) ? Number(variable.value) : 0),
    ]),
  );

  const fieldDeclarations = normalizedChains
    .map((chain, idx) => {
      const variableName = sanitizeIdentifier(
        chain.name,
        `pathChain${idx + 1}`,
      );
      return `public PathChain ${variableName};`;
    })
    .join("\n    ");

  const pathAssignments = normalizedChains
    .map((chain, chainIdx) => {
      const variableName = sanitizeIdentifier(
        chain.name,
        `pathChain${chainIdx + 1}`,
      );

      const segmentSnippets = chain.lineIds
        .map((lineId) => {
          const line = lineById.get(lineId);
          if (!line) return null;

          const lineIndex = linesWithIds.findIndex((ln) => ln.id === line.id);
          const startExpression =
            lineIndex <= 0
              ? buildPoseExpression(
                  startPoint,
                  numberVariables,
                  numberVariableConstantById,
                )
              : buildPoseExpression(
                  linesWithIds[lineIndex - 1].endPoint,
                  numberVariables,
                  numberVariableConstantById,
                );

          return buildPathSegmentCode(
            line,
            startExpression,
            numberVariables,
            numberVariableConstantById,
          );
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
  const rawSeq = sequence && sequence.length ? sequence : defaultSequence;
  const seq: SequenceItem[] = [];
  rawSeq.forEach((item) => {
    if (item.kind !== "repeat") {
      seq.push(item);
      return;
    }
    const repeatCount = Math.max(
      1,
      Math.min(20, Math.round(Number(item.count) || 1)),
    );
    for (let repeatIndex = 0; repeatIndex < repeatCount; repeatIndex++) {
      (item.lineIds || []).forEach((lineId) => {
        seq.push({ kind: "path", lineId });
      });
    }
  });

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
