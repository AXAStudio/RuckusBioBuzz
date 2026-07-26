import type {
  BasePoint,
  Line,
  Point,
  PoseVariable,
  PathVariable,
  Variable,
} from "../types";
import { mirrorPoseExpression } from "./numberExpressions";

export type MirrorAxis = "x" | "y";

type PathMirrorData = {
  startPoint?: Point;
  lines?: Line[];
  variables?: Variable[];
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
    eventMarkers: (line.eventMarkers || []).map((marker) => {
      const mirrorX = marker.triggerType === "pose" && axis === "x";
      const mirrorY = marker.triggerType === "pose" && axis === "y";

      return {
        ...marker,
        poseX: mirrorEventMarkerCoordinate(marker.poseX, mirrorX, fieldSize),
        poseY: mirrorEventMarkerCoordinate(marker.poseY, mirrorY, fieldSize),
        // Mirror the expression itself so it keeps tracking its variables.
        poseXExpression: mirrorX
          ? mirrorPoseExpression(
              marker.poseXExpression,
              Number(marker.poseX) || 0,
              (inner) => `(${fieldSize} - (${inner}))`,
            )
          : marker.poseXExpression,
        poseYExpression: mirrorY
          ? mirrorPoseExpression(
              marker.poseYExpression,
              Number(marker.poseY) || 0,
              (inner) => `(${fieldSize} - (${inner}))`,
            )
          : marker.poseYExpression,
      };
    }),
  };
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

/**
 * Mirrors a variable across the field. Numbers and booleans are left alone:
 * they may be used for things that have nothing to do with position, and the
 * expressions that reference them are mirrored at the usage site instead.
 */
export function mirrorVariable(
  variable: Variable,
  axis: MirrorAxis,
  fieldSize = DEFAULT_FIELD_SIZE,
): Variable {
  if (variable.type === "pose") {
    return mirrorPoseVariable(variable, axis, fieldSize);
  }
  if (variable.type === "path") {
    return mirrorPathVariable(variable, axis, fieldSize);
  }
  return variable;
}

export function mirrorPathData<T extends PathMirrorData>(
  data: T,
  axis: MirrorAxis = "x",
  fieldSize = DEFAULT_FIELD_SIZE,
): T {
  return {
    ...data,
    startPoint: data.startPoint
      ? mirrorPoint(data.startPoint, axis, fieldSize)
      : data.startPoint,
    lines: Array.isArray(data.lines)
      ? data.lines.map((line) => mirrorLine(line, axis, fieldSize))
      : data.lines,
    variables: Array.isArray(data.variables)
      ? data.variables.map((variable) =>
          mirrorVariable(variable, axis, fieldSize),
        )
      : data.variables,
  };
}
