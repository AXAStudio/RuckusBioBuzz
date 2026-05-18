import type {
  BasePoint,
  Line,
  Point,
  PoseVariable,
  PathVariable,
  NumberVariable,
} from "../types";
import { mirrorPoseExpression } from "./numberExpressions";

export type MirrorAxis = "x" | "y";

type PathMirrorData = {
  startPoint?: Point;
  lines?: Line[];
  poseVariables?: PoseVariable[];
  pathVariables?: PathVariable[];
  numberVariables?: NumberVariable[];
};

const DEFAULT_FIELD_SIZE = 141.5;

function mirrorCoordinate(value: number, fieldSize: number): number {
  const numeric = Number(value);
  return fieldSize - (Number.isFinite(numeric) ? numeric : 0);
}

function mirrorEventMarkerCoordinate(
  value: number | undefined,
  shouldMirror: boolean,
  fieldSize: number,
): number | undefined {
  if (!shouldMirror || value === undefined) return value;
  return mirrorCoordinate(value, fieldSize);
}

export function mirrorHeadingDegrees(
  heading: number,
  axis: MirrorAxis,
): number {
  const numeric = Number(heading);
  const mirrored = axis === "x"
    ? 180 - (Number.isFinite(numeric) ? numeric : 0)
    : -(Number.isFinite(numeric) ? numeric : 0);
  const normalized = ((((mirrored + 180) % 360) + 360) % 360) - 180;
  return Math.abs(normalized) < 1e-9 ? 0 : normalized;
}

export function mirrorBasePoint<T extends BasePoint>(
  point: T,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): T {
  const next = { ...point };

  if (axis === "x") {
    next.x = mirrorCoordinate(next.x, fieldSize);
    next.xExpression = mirrorPoseExpression(point.xExpression, point.x, (inner) =>
      `(${fieldSize} - (${inner}))`,
    );
  } else {
    next.y = mirrorCoordinate(next.y, fieldSize);
    next.yExpression = mirrorPoseExpression(point.yExpression, point.y, (inner) =>
      `(${fieldSize} - (${inner}))`,
    );
  }

  return next;
}

export function mirrorPoint(
  point: Point,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): Point {
  const mirrored = mirrorBasePoint(point, axis, fieldSize);

  if (mirrored.heading === "linear") {
    return {
      ...mirrored,
      startDeg: mirrorHeadingDegrees(mirrored.startDeg, axis),
      endDeg: mirrorHeadingDegrees(mirrored.endDeg, axis),
      startDegExpression: mirrorPoseExpression(
        point.startDegExpression,
        point.startDeg ?? 0,
        (inner) => (axis === "x" ? `(180 - (${inner}))` : `-((${inner}))`),
      ),
      endDegExpression: mirrorPoseExpression(
        point.endDegExpression,
        point.endDeg ?? 0,
        (inner) => (axis === "x" ? `(180 - (${inner}))` : `-((${inner}))`),
      ),
    };
  }

  if (mirrored.heading === "constant") {
    return {
      ...mirrored,
      degrees: mirrorHeadingDegrees(mirrored.degrees, axis),
      degreesExpression: mirrorPoseExpression(
        point.degreesExpression,
        point.degrees ?? 0,
        (inner) => (axis === "x" ? `(180 - (${inner}))` : `-((${inner}))`),
      ),
    };
  }

  return mirrored;
}

export function mirrorPoseVariable(
  variable: PoseVariable,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): PoseVariable {
  const mirrored = mirrorBasePoint(variable, axis, fieldSize);
  return {
    ...mirrored,
    heading: mirrorHeadingDegrees(mirrored.heading, axis),
    xExpression:
      axis === "x"
        ? mirrorPoseExpression(variable.xExpression, variable.x, (inner) =>
            `(${fieldSize} - (${inner}))`,
          )
        : variable.xExpression,
    yExpression:
      axis === "y"
        ? mirrorPoseExpression(variable.yExpression, variable.y, (inner) =>
            `(${fieldSize} - (${inner}))`,
          )
        : variable.yExpression,
    headingExpression: mirrorPoseExpression(
      variable.headingExpression,
      variable.heading,
      (inner) => (axis === "x" ? `(180 - (${inner}))` : `-((${inner}))`),
    ),
  };
}

export function mirrorLine(
  line: Line,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): Line {
  return {
    ...line,
    endPoint: mirrorPoint(line.endPoint, axis, fieldSize),
    controlPoints: (line.controlPoints || []).map((point) =>
      mirrorBasePoint(point, axis, fieldSize),
    ),
    eventMarkers: (line.eventMarkers || []).map((marker) => ({
      ...marker,
      poseX: mirrorEventMarkerCoordinate(
        marker.poseX,
        marker.triggerType === "pose" && axis === "x",
        fieldSize,
      ),
      poseY: mirrorEventMarkerCoordinate(
        marker.poseY,
        marker.triggerType === "pose" && axis === "y",
        fieldSize,
      ),
    })),
  };
}

function collectMirroredNumberVariableIds(
  lines: Line[] | undefined,
  axis: MirrorAxis,
  output: Set<string>,
) {
  if (!Array.isArray(lines)) return;

  lines.forEach((line) => {
    (line.eventMarkers || []).forEach((marker) => {
      if (marker.triggerType !== "pose") return;

      const variableId =
        axis === "x" ? marker.poseXVariableId : marker.poseYVariableId;
      if (variableId) {
        output.add(variableId);
      }
    });
  });
}

function mirrorNumberVariables(
  variables: NumberVariable[] | undefined,
  mirroredIds: Set<string>,
  fieldSize: number,
): NumberVariable[] | undefined {
  if (!Array.isArray(variables) || mirroredIds.size === 0) {
    return variables;
  }

  return variables.map((variable) =>
    mirroredIds.has(variable.id)
      ? {
          ...variable,
          value: mirrorCoordinate(variable.value, fieldSize),
        }
      : variable,
  );
}

export function mirrorPathVariable(
  variable: PathVariable,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): PathVariable {
  return {
    ...variable,
    startPoint: mirrorPoint(variable.startPoint, axis, fieldSize),
    lines: variable.lines.map((line) => mirrorLine(line, axis, fieldSize)),
  };
}

export function mirrorPathData<T extends PathMirrorData>(
  data: T,
  axis: MirrorAxis = "x",
  fieldSize = DEFAULT_FIELD_SIZE,
): T {
  const mirroredNumberVariableIds = new Set<string>();
  collectMirroredNumberVariableIds(data.lines, axis, mirroredNumberVariableIds);
  (data.pathVariables || []).forEach((variable) =>
    collectMirroredNumberVariableIds(
      variable.lines,
      axis,
      mirroredNumberVariableIds,
    ),
  );

  return {
    ...data,
    startPoint: data.startPoint
      ? mirrorPoint(data.startPoint, axis, fieldSize)
      : data.startPoint,
    lines: Array.isArray(data.lines)
      ? data.lines.map((line) => mirrorLine(line, axis, fieldSize))
      : data.lines,
    poseVariables: Array.isArray(data.poseVariables)
      ? data.poseVariables.map((variable) =>
          mirrorPoseVariable(variable, axis, fieldSize),
        )
      : data.poseVariables,
    pathVariables: Array.isArray(data.pathVariables)
      ? data.pathVariables.map((variable) =>
          mirrorPathVariable(variable, axis, fieldSize),
        )
      : data.pathVariables,
    numberVariables: mirrorNumberVariables(
      data.numberVariables,
      mirroredNumberVariableIds,
      fieldSize,
    ),
  };
}
