import type { BasePoint, Line, Point, PoseVariable } from "../types";

export type MirrorAxis = "x" | "y";

type PathMirrorData = {
  startPoint?: Point;
  lines?: Line[];
  poseVariables?: PoseVariable[];
};

const DEFAULT_FIELD_SIZE = 141.5;

function mirrorCoordinate(value: number, fieldSize: number): number {
  const numeric = Number(value);
  return fieldSize - (Number.isFinite(numeric) ? numeric : 0);
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
  } else {
    next.y = mirrorCoordinate(next.y, fieldSize);
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
    };
  }

  if (mirrored.heading === "constant") {
    return {
      ...mirrored,
      degrees: mirrorHeadingDegrees(mirrored.degrees, axis),
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
  };
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
    poseVariables: Array.isArray(data.poseVariables)
      ? data.poseVariables.map((variable) =>
          mirrorPoseVariable(variable, axis, fieldSize),
        )
      : data.poseVariables,
  };
}
