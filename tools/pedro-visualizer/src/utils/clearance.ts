/**
 * Does the robot fit where the path sends it?
 *
 * A path is a curve through the middle of the robot, and a curve can thread a
 * gap the robot itself cannot. What actually has to fit is the footprint, held
 * at the heading the robot is really at, at every point along the route — so
 * this walks the route, plants the footprint at even spacing, and measures it
 * against the obstacles and the field walls.
 *
 * The footprint is a polygon in the robot's own frame, `+x` forward at heading
 * zero, matching `getRobotCorners`. That is a rectangle for a robot described by
 * width and height, and the real outline when one has been taken from CAD, so a
 * robot with a protruding intake is checked as the shape it is.
 */
import type { BasePoint, Line, Shape } from "../types";
import type { HeadingTransition } from "./animation";
import {
  minDistanceToPolygon,
  pointInPolygon,
  pointToLineDistance,
} from "./geometry";
import { getCurvePoint, goalHeadingAt, headingAlongPath } from "./math";
import { curveParameterAtDistance } from "./motionProfile";

/** Samples per path used to build the arc-length table. */
const ARC_SAMPLES = 200;

/** Default spacing between footprint samples, inches. */
export const DEFAULT_CLEARANCE_STEP = 0.75;

/**
 * Ceiling on footprint samples for the whole route. Each sample tests the
 * footprint against every obstacle, so an unbounded route would stall the page.
 */
export const MAX_CLEARANCE_SAMPLES = 4000;

/** Any overlap at all reports as negative, even when no vertex is contained. */
const MIN_OVERLAP_DEPTH = 1e-6;

export type ClearanceKind = "obstacle" | "wall";
export type ClearanceSeverity = "hit" | "tight";

/** A stretch of one path where the robot is closer than the margin allows. */
export interface ClearanceSpan {
  lineId: string;
  lineIndex: number;
  /** Distance into the path, inches. */
  startDistance: number;
  endDistance: number;
  /** Worst signed clearance in the span; negative means overlapping. */
  worstClearance: number;
  kind: ClearanceKind;
  obstacleId?: string;
  obstacleName?: string;
  severity: ClearanceSeverity;
  /** Where the worst sample sits, for drawing and for jumping to it. */
  worstPose: { x: number; y: number; heading: number };
  /** The footprint at that pose, in field inches. */
  worstFootprint: BasePoint[];
}

export interface ClearanceLineReport {
  lineId: string;
  lineIndex: number;
  /** Smallest signed clearance anywhere on the path. */
  minClearance: number;
  hit: boolean;
  tight: boolean;
  spans: ClearanceSpan[];
}

export interface ClearanceReport {
  spans: ClearanceSpan[];
  byLine: Map<string, ClearanceLineReport>;
  /** Smallest signed clearance over the whole route, `Infinity` if unchecked. */
  minClearance: number;
  hitCount: number;
  tightCount: number;
  margin: number;
  /** Spacing actually used, which widens if the route would exceed the ceiling. */
  step: number;
  /** True when the ceiling forced a wider spacing than asked for. */
  coarse: boolean;
  /**
   * Where the robot is staged, measured before it drives anywhere. Called out
   * separately because it is the one problem no path can fix — the robot is
   * already sitting in the obstacle.
   */
  startPose: {
    clearance: number;
    kind: ClearanceKind;
    obstacleName?: string;
  } | null;
}

export interface ClearanceOptions {
  /** Field is a square this many inches on a side. */
  fieldSize: number;
  /** Report anything closer than this, inches. */
  margin?: number;
  step?: number;
  maxSamples?: number;
}

export interface ClearanceInput {
  startPoint: BasePoint;
  lines: Line[];
  /** Robot outline in its own frame, `+x` forward. */
  footprint: BasePoint[];
  obstacles?: Shape[];
  /** Route start point per line id; without it, plain array order is used. */
  lineStartPoints?: Map<string, BasePoint>;
  /** How each path picks up its heading goal, keyed by line id. */
  headingTransitions?: Map<string, HeadingTransition>;
  /**
   * Measure only these paths. The route is still walked in full so start points
   * stay right; the paths left out simply are not sampled. Used by the endpoint
   * search, which re-measures two paths a few hundred times and has no reason to
   * pay for the rest of the route each time.
   */
  measureLineIds?: Set<string>;
}

export const EMPTY_CLEARANCE_REPORT: ClearanceReport = {
  spans: [],
  byLine: new Map(),
  minClearance: Infinity,
  hitCount: 0,
  tightCount: 0,
  margin: 0,
  step: DEFAULT_CLEARANCE_STEP,
  coarse: false,
  startPose: null,
};

/* -------------------------------------------------------------------------
 * Footprints
 * ---------------------------------------------------------------------- */

/**
 * The plain rectangle, in the same frame and corner order `getRobotCorners`
 * uses: `width` runs along the robot's forward direction, `height` across it.
 */
export function rectangleFootprint(width: number, height: number): BasePoint[] {
  const halfWidth = Math.max(0, Number(width) || 0) / 2;
  const halfHeight = Math.max(0, Number(height) || 0) / 2;
  return [
    { x: -halfWidth, y: -halfHeight },
    { x: halfWidth, y: -halfHeight },
    { x: halfWidth, y: halfHeight },
    { x: -halfWidth, y: halfHeight },
  ];
}

/**
 * The outline to check with: the real one from CAD when there is one, otherwise
 * the bounding rectangle.
 *
 * A stored outline is rescaled to whatever the robot's dimensions currently say.
 * The dimensions are editable after a CAD import, and an outline that ignored
 * that would quietly check the wrong size robot.
 */
export function footprintFromSettings(settings: {
  rWidth?: number;
  rHeight?: number;
  robotOutline?: { points: BasePoint[]; lengthInches: number; widthInches: number };
}): BasePoint[] {
  const width = Number(settings?.rWidth) || 0;
  const height = Number(settings?.rHeight) || 0;
  const outline = settings?.robotOutline;

  if (outline?.points && outline.points.length >= 3) {
    const scaleX = outline.lengthInches > 0 ? width / outline.lengthInches : 1;
    const scaleY = outline.widthInches > 0 ? height / outline.widthInches : 1;
    if (Number.isFinite(scaleX) && Number.isFinite(scaleY) && scaleX > 0 && scaleY > 0) {
      return outline.points.map((point) => ({
        x: point.x * scaleX,
        y: point.y * scaleY,
      }));
    }
  }

  return rectangleFootprint(width, height);
}

/** Places a robot-frame footprint on the field at a pose. */
export function transformFootprint(
  footprint: BasePoint[],
  x: number,
  y: number,
  headingDegrees: number,
  out?: BasePoint[],
): BasePoint[] {
  const radians = (headingDegrees * Math.PI) / 180;
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);
  const target = out && out.length === footprint.length ? out : new Array(footprint.length);

  for (let i = 0; i < footprint.length; i++) {
    const point = footprint[i];
    const px = point.x;
    const py = point.y;
    // Same rotation as getRobotCorners, so a footprint and the drawn robot
    // never disagree about which way the robot is pointing.
    target[i] = { x: x + px * cos - py * sin, y: y + px * sin + py * cos };
  }

  return target as BasePoint[];
}

/* -------------------------------------------------------------------------
 * Polygon tests
 * ---------------------------------------------------------------------- */

function orientation(a: BasePoint, b: BasePoint, c: BasePoint): number {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function onSegment(a: BasePoint, b: BasePoint, point: BasePoint): boolean {
  return (
    Math.min(a.x, b.x) <= point.x &&
    point.x <= Math.max(a.x, b.x) &&
    Math.min(a.y, b.y) <= point.y &&
    point.y <= Math.max(a.y, b.y)
  );
}

export function segmentsIntersect(
  a1: BasePoint,
  a2: BasePoint,
  b1: BasePoint,
  b2: BasePoint,
): boolean {
  const d1 = orientation(a1, a2, b1);
  const d2 = orientation(a1, a2, b2);
  const d3 = orientation(b1, b2, a1);
  const d4 = orientation(b1, b2, a2);

  if (((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0))) return true;

  // Collinear touches still count: a robot edge lying exactly along an obstacle
  // edge is contact, not clearance.
  if (d1 === 0 && onSegment(a1, a2, b1)) return true;
  if (d2 === 0 && onSegment(a1, a2, b2)) return true;
  if (d3 === 0 && onSegment(b1, b2, a1)) return true;
  if (d4 === 0 && onSegment(b1, b2, a2)) return true;

  return false;
}

/** Shortest distance between the two outlines, ignoring which side they are on. */
export function polygonBoundaryDistance(a: BasePoint[], b: BasePoint[]): number {
  let best = Infinity;

  for (let i = 0; i < a.length; i++) {
    const a1 = a[i];
    const a2 = a[(i + 1) % a.length];

    for (let j = 0; j < b.length; j++) {
      const b1 = b[j];
      const b2 = b[(j + 1) % b.length];

      if (segmentsIntersect(a1, a2, b1, b2)) return 0;

      // Non-crossing segments: the closest pair always involves an endpoint.
      const distance = Math.min(
        pointToLineDistance([a1.x, a1.y], [b1.x, b1.y], [b2.x, b2.y]),
        pointToLineDistance([a2.x, a2.y], [b1.x, b1.y], [b2.x, b2.y]),
        pointToLineDistance([b1.x, b1.y], [a1.x, a1.y], [a2.x, a2.y]),
        pointToLineDistance([b2.x, b2.y], [a1.x, a1.y], [a2.x, a2.y]),
      );
      if (distance < best) best = distance;
    }
  }

  return best;
}

export function polygonsOverlap(a: BasePoint[], b: BasePoint[]): boolean {
  for (let i = 0; i < a.length; i++) {
    const a1 = a[i];
    const a2 = a[(i + 1) % a.length];
    for (let j = 0; j < b.length; j++) {
      if (segmentsIntersect(a1, a2, b[j], b[(j + 1) % b.length])) return true;
    }
  }

  // No crossing edges leaves only containment, and one vertex each settles it.
  if (a.length && pointInPolygon([a[0].x, a[0].y], b)) return true;
  if (b.length && pointInPolygon([b[0].x, b[0].y], a)) return true;
  return false;
}

/**
 * How far the two shapes are apart, negative when they overlap.
 *
 * Overlap is measured as how far past the boundary the intrusion reaches, not
 * as the distance between outlines: once two shapes cross, that distance is
 * zero no matter how deep the robot is buried, which would read as "just
 * touching" for a path straight through the middle of an obstacle.
 */
export function polygonClearance(a: BasePoint[], b: BasePoint[]): number {
  if (!a.length || !b.length) return Infinity;
  if (!polygonsOverlap(a, b)) return polygonBoundaryDistance(a, b);

  let depth = 0;
  for (const point of a) {
    if (pointInPolygon([point.x, point.y], b)) {
      depth = Math.max(depth, minDistanceToPolygon([point.x, point.y], b));
    }
  }
  for (const point of b) {
    if (pointInPolygon([point.x, point.y], a)) {
      depth = Math.max(depth, minDistanceToPolygon([point.x, point.y], a));
    }
  }

  return -Math.max(depth, MIN_OVERLAP_DEPTH);
}

/**
 * Distance from the footprint to the nearest field wall, negative once any part
 * of the robot is over the line. Testing vertices is exact here because the
 * walls are straight and the footprint's edges are too.
 */
export function wallClearance(footprint: BasePoint[], fieldSize: number): number {
  let best = Infinity;
  for (const point of footprint) {
    best = Math.min(
      best,
      point.x,
      point.y,
      fieldSize - point.x,
      fieldSize - point.y,
    );
  }
  return best;
}

/* -------------------------------------------------------------------------
 * Walking the route
 * ---------------------------------------------------------------------- */

function arcLengthTable(curvePoints: BasePoint[], samples: number): Float32Array {
  const table = new Float32Array(samples + 1);
  let previous = getCurvePoint(0, curvePoints);
  let total = 0;

  for (let i = 1; i <= samples; i++) {
    const point = getCurvePoint(i / samples, curvePoints);
    total += Math.hypot(point.x - previous.x, point.y - previous.y);
    table[i] = total;
    previous = point;
  }

  return table;
}

interface RouteLine {
  line: Line;
  lineIndex: number;
  curvePoints: BasePoint[];
  arcLengths: Float32Array;
  length: number;
  /** False when the caller asked for this path to be skipped. */
  measured: boolean;
}

function routeLinesOf(input: ClearanceInput): RouteLine[] {
  const { startPoint, lines, lineStartPoints, measureLineIds } = input;
  const routeLines: RouteLine[] = [];
  let currentStart: BasePoint = startPoint;

  lines.forEach((line, lineIndex) => {
    if (!line?.endPoint) return;
    const routeStart = line.id ? lineStartPoints?.get(line.id) : undefined;
    if (lineStartPoints && !routeStart) return; // Not on the route.
    if (routeStart) currentStart = routeStart;

    const measured = !measureLineIds || measureLineIds.has(line.id || "");
    const curvePoints = [currentStart, ...line.controlPoints, line.endPoint];
    // Building the arc-length table is the expensive part, so skip it entirely
    // for paths nobody asked about.
    const arcLengths = measured
      ? arcLengthTable(curvePoints, ARC_SAMPLES)
      : EMPTY_ARC_LENGTHS;

    routeLines.push({
      line,
      lineIndex,
      curvePoints,
      arcLengths,
      length: measured ? arcLengths[arcLengths.length - 1] : 0,
      measured,
    });

    currentStart = line.endPoint;
  });

  return routeLines;
}

const EMPTY_ARC_LENGTHS = new Float32Array(1);

interface ClearanceSample {
  distance: number;
  clearance: number;
  kind: ClearanceKind;
  obstacleId?: string;
  obstacleName?: string;
  x: number;
  y: number;
  heading: number;
  footprint: BasePoint[];
}

/**
 * Measures the robot's footprint against the obstacles and the walls along the
 * whole route.
 *
 * Samples are spaced evenly along each curve rather than by curve parameter: a
 * Bezier covers ground at wildly different rates across its span, so stepping
 * the parameter would sample densely at one end and stride past a whole
 * obstacle at the other.
 */
export function checkClearance(
  input: ClearanceInput,
  options: ClearanceOptions,
): ClearanceReport {
  const fieldSize = Number(options.fieldSize);
  const margin = Math.max(0, Number(options.margin) || 0);
  const requestedStep = Number(options.step) > 0 ? Number(options.step) : DEFAULT_CLEARANCE_STEP;
  const maxSamples =
    Number(options.maxSamples) > 0 ? Number(options.maxSamples) : MAX_CLEARANCE_SAMPLES;

  const footprint = input.footprint;
  if (!footprint?.length || !Number.isFinite(fieldSize) || fieldSize <= 0) {
    return { ...EMPTY_CLEARANCE_REPORT, margin, step: requestedStep };
  }

  const routeLines = routeLinesOf(input);
  if (!routeLines.length) {
    return { ...EMPTY_CLEARANCE_REPORT, margin, step: requestedStep };
  }

  const obstacles = (input.obstacles || []).filter(
    (shape) => shape?.vertices && shape.vertices.length >= 3,
  );

  const totalLength = routeLines.reduce(
    (sum, entry) => sum + (entry.measured ? entry.length : 0),
    0,
  );
  const step = Math.max(requestedStep, totalLength / maxSamples);
  const coarse = step > requestedStep + 1e-9;

  const spans: ClearanceSpan[] = [];
  const byLine = new Map<string, ClearanceLineReport>();
  let minClearance = Infinity;
  let startPose: ClearanceReport["startPose"] = null;

  let previousEnd: BasePoint | null = null;

  for (const entry of routeLines) {
    const { line, lineIndex, curvePoints, arcLengths, length } = entry;
    const lineId = line.id || `index-${lineIndex}`;
    const transition = input.headingTransitions?.get(line.id || "");

    // The endpoint of one path and the start of the next are the same pose —
    // the heading model hands the next path the heading the robot arrives with.
    // Checking it twice would report one tight corner as two problems and
    // double the count telemetry and the export warnings show. A branch starts
    // somewhere else, so the join has to actually line up.
    const start = curvePoints[0];
    const continues =
      previousEnd !== null &&
      Math.hypot(start.x - previousEnd.x, start.y - previousEnd.y) < 1e-6;
    previousEnd = line.endPoint;

    if (!entry.measured) continue;

    const samples: ClearanceSample[] = [];
    // Always finish on the endpoint: the last stride rarely lands on it, and the
    // endpoint is exactly where a robot tends to be parked against something.
    const stepCount = length > 0 ? Math.max(1, Math.ceil(length / step)) : 0;

    for (let i = continues ? 1 : 0; i <= stepCount; i++) {
      const distance = stepCount > 0 ? Math.min(length, i * step) : 0;
      const t = length > 0 ? curveParameterAtDistance(arcLengths, distance) : 0;
      const position = getCurvePoint(t, curvePoints);
      const heading = headingAlongPath(
        goalHeadingAt(line, curvePoints, t),
        transition?.entryHeading,
        t,
        transition?.catchUp,
      );

      const placed = transformFootprint(footprint, position.x, position.y, heading);

      let clearance = wallClearance(placed, fieldSize);
      let kind: ClearanceKind = "wall";
      let obstacleId: string | undefined;
      let obstacleName: string | undefined;

      for (const shape of obstacles) {
        const value = polygonClearance(placed, shape.vertices);
        if (value < clearance) {
          clearance = value;
          kind = "obstacle";
          obstacleId = shape.id;
          obstacleName = shape.name;
        }
      }

      if (clearance < minClearance) minClearance = clearance;

      // The very first sample of the route is the robot as staged.
      if (!startPose && distance === 0 && !continues) {
        startPose = { clearance, kind, obstacleName };
      }

      samples.push({
        distance,
        clearance,
        kind,
        obstacleId,
        obstacleName,
        x: position.x,
        y: position.y,
        heading,
        footprint: placed,
      });
    }

    const lineSpans = groupSpans(samples, margin, lineId, lineIndex);
    spans.push(...lineSpans);

    const lineMin = samples.reduce(
      (best, sample) => Math.min(best, sample.clearance),
      Infinity,
    );

    byLine.set(lineId, {
      lineId,
      lineIndex,
      minClearance: lineMin,
      hit: lineSpans.some((span) => span.severity === "hit"),
      tight: lineSpans.some((span) => span.severity === "tight"),
      spans: lineSpans,
    });
  }

  return {
    spans,
    byLine,
    minClearance,
    hitCount: spans.filter((span) => span.severity === "hit").length,
    tightCount: spans.filter((span) => span.severity === "tight").length,
    margin,
    step,
    coarse,
    startPose,
  };
}

/**
 * Collapses runs of too-close samples into one span each, so a path that grazes
 * an obstacle for six inches reads as one problem rather than eight.
 */
function groupSpans(
  samples: ClearanceSample[],
  margin: number,
  lineId: string,
  lineIndex: number,
): ClearanceSpan[] {
  const spans: ClearanceSpan[] = [];
  let open: ClearanceSample[] = [];

  const close = () => {
    if (!open.length) return;

    let worst = open[0];
    for (const sample of open) {
      if (sample.clearance < worst.clearance) worst = sample;
    }

    spans.push({
      lineId,
      lineIndex,
      startDistance: open[0].distance,
      endDistance: open[open.length - 1].distance,
      worstClearance: worst.clearance,
      kind: worst.kind,
      obstacleId: worst.obstacleId,
      obstacleName: worst.obstacleName,
      severity: worst.clearance < 0 ? "hit" : "tight",
      worstPose: { x: worst.x, y: worst.y, heading: worst.heading },
      worstFootprint: worst.footprint,
    });

    open = [];
  };

  for (const sample of samples) {
    // A clean stretch between two problems keeps them apart; a run of tight
    // samples that dips below zero partway is still one span, reported as a hit.
    if (sample.clearance < margin || sample.clearance < 0) open.push(sample);
    else close();
  }
  close();

  return spans;
}

/* -------------------------------------------------------------------------
 * Moving a path out of trouble
 * ---------------------------------------------------------------------- */

/**
 * What the search is allowed to move.
 *
 * Ordered by how much it costs the driver to accept. A control point changes
 * only the shape of the curve between two waypoints the robot still hits, so it
 * is almost free. An endpoint changes where the robot ends up, which is usually
 * a scoring position. The start point changes where the robot is staged for the
 * match, which someone has to physically do differently.
 */
export type FixHandleKind =
  | "controlPoint"
  | "endPoint"
  | "previousEndPoint"
  | "startPoint";

export const FIX_HANDLE_LABELS: Record<FixHandleKind, string> = {
  controlPoint: "control point",
  endPoint: "endpoint",
  previousEndPoint: "the previous path's endpoint",
  startPoint: "starting point",
};

/** A move that gets the robot clear of whatever it was hitting. */
export interface ClearanceFix {
  handle: FixHandleKind;
  /** The path the handle belongs to; empty for the route's start point. */
  lineId: string;
  /** Which control point, when the handle is one. */
  controlPointIndex?: number;
  dx: number;
  dy: number;
  /** How far the handle moves, inches. */
  distance: number;
  /** Worst clearance before and after, over every path the move affects. */
  before: number;
  after: number;
  /** False when the search could only get it clear, not to the full margin. */
  meetsMargin: boolean;
}

export interface ClearanceFixSearch {
  /** Furthest a handle may be moved, inches. */
  maxDistance?: number;
  /** Coarse radius step, inches. Refined along the winning direction after. */
  stepSize?: number;
  /** Compass directions tried at each radius. */
  directions?: number;
  /** Restrict the search to these handles, in the order given. */
  handles?: FixHandleKind[];
}

/** Sampling spacing used while scanning candidates, before confirming. */
const SEARCH_SCAN_STEP = 1.5;

/**
 * Smallest move worth calling a fix, inches. Below this the search has found a
 * rounding-level difference, and offering to nudge a waypoint four thousandths
 * of an inch is noise dressed up as a suggestion.
 */
const MIN_FIX_DISTANCE = 0.02;

/**
 * Least clearance a partial fix has to buy, inches. Without it the search
 * happily reports moving a waypoint half an inch to go from 0.99in to 0.99in.
 */
const MIN_FIX_GAIN = 0.05;

function meaningful(candidate: { dx: number; dy: number }): boolean {
  return Math.hypot(candidate.dx, candidate.dy) >= MIN_FIX_DISTANCE;
}

/** A candidate handle, and how to build the route with it moved. */
interface FixHandle {
  kind: FixHandleKind;
  lineId: string;
  controlPointIndex?: number;
  measureLineIds: Set<string>;
  apply: (dx: number, dy: number) => ClearanceInput;
  /**
   * What the search is trying to maximise. Defaults to the worst clearance
   * anywhere it measured, which is right for a handle that owns the whole
   * stretch it affects.
   */
  score?: (report: ClearanceReport) => number;
}

/** Whether a point is something the fix may move. */
function movablePoint(point: BasePoint | undefined): boolean {
  // A locked point was pinned on purpose, and one driven by a pose variable
  // takes its position from the variable, so writing x/y would not stick.
  return !!point && !point.locked && !point.poseVariableId;
}

/** Whether a path's endpoint is something the fix button may move. */
export function canMoveEndpoint(line: Line | undefined): boolean {
  return movablePoint(line?.endPoint);
}

/**
 * Everything the fix is allowed to move to clear a problem on one path.
 *
 * The start point only shows up for the path that actually begins there, since
 * every other path begins at the previous path's endpoint — which is that
 * path's own handle.
 */
function handlesForLine(
  input: ClearanceInput,
  options: ClearanceOptions,
  lineId: string,
  allowed: FixHandleKind[],
): FixHandle[] {
  const lineIndex = input.lines.findIndex((line) => line?.id === lineId);
  if (lineIndex < 0) return [];

  const original = input.lines[lineIndex];
  if (!original?.endPoint) return [];

  // The route order decides what follows this path, which a move of its
  // endpoint also affects.
  const routeLines = routeLinesOf({ ...input, measureLineIds: new Set<string>() });
  const position = routeLines.findIndex((entry) => entry.line.id === lineId);
  if (position < 0) return [];

  const follower = routeLines[position + 1];
  const followerContinues =
    follower !== undefined &&
    Math.hypot(
      follower.curvePoints[0].x - original.endPoint.x,
      follower.curvePoints[0].y - original.endPoint.y,
    ) < 1e-6;

  const withFollower = new Set<string>([lineId]);
  if (followerContinues && follower.line.id) withFollower.add(follower.line.id);

  /** Rebuilds the input with one line replaced. */
  const replaceLine = (next: Line): Line[] =>
    input.lines.map((line, index) => (index === lineIndex ? next : line));

  const handles: FixHandle[] = [];

  if (allowed.includes("controlPoint")) {
    original.controlPoints.forEach((point, controlIndex) => {
      if (!movablePoint(point)) return;
      handles.push({
        kind: "controlPoint",
        lineId,
        controlPointIndex: controlIndex,
        // A control point only bends this path. Where the robot starts and ends
        // is untouched, so nothing else needs re-measuring.
        measureLineIds: new Set([lineId]),
        apply: (dx, dy) => ({
          ...input,
          lines: replaceLine({
            ...original,
            controlPoints: original.controlPoints.map((entry, index) =>
              index === controlIndex
                ? { ...entry, x: entry.x + dx, y: entry.y + dy }
                : entry,
            ),
          }),
        }),
      });
    });
  }

  if (allowed.includes("endPoint") && movablePoint(original.endPoint)) {
    handles.push({
      kind: "endPoint",
      lineId,
      measureLineIds: withFollower,
      apply: (dx, dy) => {
        const movedEnd = {
          ...original.endPoint,
          x: original.endPoint.x + dx,
          y: original.endPoint.y + dy,
        };

        // The path after this one starts where this one ends, so its recorded
        // start has to move with it or the search would score a stale route.
        let lineStartPoints = input.lineStartPoints;
        if (lineStartPoints && followerContinues && follower.line.id) {
          lineStartPoints = new Map(lineStartPoints);
          lineStartPoints.set(follower.line.id, movedEnd);
        }

        return { ...input, lines: replaceLine({ ...original, endPoint: movedEnd }), lineStartPoints };
      },
    });
  }

  // The first stretch of a path is pinned to wherever the previous path left the
  // robot, so a collision there has no lever on this path at all — its own
  // endpoint and control points are all downstream of the problem. The handle
  // that can move it belongs to the path before. Tried last because it edits a
  // different path than the one whose button was pressed.
  const predecessor = position > 0 ? routeLines[position - 1] : undefined;
  const continuesFromPredecessor =
    predecessor !== undefined &&
    Math.hypot(
      predecessor.line.endPoint.x - routeLines[position].curvePoints[0].x,
      predecessor.line.endPoint.y - routeLines[position].curvePoints[0].y,
    ) < 1e-6;

  if (
    allowed.includes("previousEndPoint") &&
    continuesFromPredecessor &&
    predecessor.line.id &&
    movablePoint(predecessor.line.endPoint)
  ) {
    const previous = predecessor.line;
    const previousId = previous.id as string;
    const previousIndex = input.lines.findIndex((line) => line?.id === previousId);

    handles.push({
      kind: "previousEndPoint",
      lineId: previousId,
      // Both paths meet at this point, so both have to come out of it clear.
      measureLineIds: new Set([previousId, lineId]),
      apply: (dx, dy) => {
        const movedEnd = {
          ...previous.endPoint,
          x: previous.endPoint.x + dx,
          y: previous.endPoint.y + dy,
        };

        let lineStartPoints = input.lineStartPoints;
        if (lineStartPoints) {
          lineStartPoints = new Map(lineStartPoints);
          lineStartPoints.set(lineId, movedEnd);
        }

        return {
          ...input,
          lines: input.lines.map((line, index) =>
            index === previousIndex ? { ...line, endPoint: movedEnd } : line,
          ),
          lineStartPoints,
        };
      },
    });
  }

  // Only the path that genuinely starts at the route's start point may move it.
  const startsAtStartPoint =
    position === 0 &&
    Math.hypot(
      routeLines[0].curvePoints[0].x - input.startPoint.x,
      routeLines[0].curvePoints[0].y - input.startPoint.y,
    ) < 1e-6;

  if (allowed.includes("startPoint") && startsAtStartPoint && movablePoint(input.startPoint)) {
    handles.push(startPointHandle(input, new Set([lineId]), options));
  }

  return handles;
}

/**
 * Moving where the robot is staged, which shifts the start of the first path.
 *
 * Scored on the staged pose rather than on the whole first path. A path that
 * drives through a goal is a problem for that path's own handles; re-staging the
 * robot cannot fix it, and scoring it that way would make the search give up on
 * a start pose it could perfectly well move out of trouble. The rest of the path
 * still has a veto: a move that makes it worse than it already was scores as
 * that worse number.
 */
function startPointHandle(
  input: ClearanceInput,
  measureLineIds: Set<string>,
  options: ClearanceOptions,
): FixHandle {
  const routeLines = routeLinesOf({ ...input, measureLineIds: new Set<string>() });
  const first = routeLines[0];

  const baselineRest = checkClearance({ ...input, measureLineIds }, options).minClearance;

  return {
    kind: "startPoint",
    lineId: "",
    measureLineIds,
    score: (report) => {
      const pose = report.startPose?.clearance ?? Infinity;
      const rest = report.minClearance;
      return rest < baselineRest - 1e-6 ? Math.min(pose, rest) : pose;
    },
    apply: (dx, dy) => {
      const moved = {
        ...input.startPoint,
        x: input.startPoint.x + dx,
        y: input.startPoint.y + dy,
      };

      let lineStartPoints = input.lineStartPoints;
      if (lineStartPoints && first?.line.id) {
        lineStartPoints = new Map(lineStartPoints);
        lineStartPoints.set(first.line.id, moved);
      }

      return { ...input, startPoint: moved, lineStartPoints };
    },
  };
}

/**
 * Finds the smallest move that gets the robot clear on a path.
 *
 * Handles are tried in order of what they cost to accept, and the first that
 * reaches the safety margin wins — bending the curve is preferred over moving
 * where the robot parks, and moving where the robot parks over re-staging it.
 *
 * Each search is a spiral rather than a push along the collision normal,
 * because a handle governs the shape of the whole curve: the direction that
 * clears a problem in the middle of a path is often not the direction away from
 * the thing being hit.
 */
export function suggestClearanceFix(
  input: ClearanceInput,
  options: ClearanceOptions,
  lineId: string,
  search: ClearanceFixSearch = {},
): ClearanceFix | null {
  // The start point deliberately is not in here. It is judged on where the robot
  // is staged rather than on the whole path, and mixing objectives makes the
  // search pick whichever handle happens to score better against a different
  // question — it would re-stage the robot to gain an inch at the start while
  // leaving the path driven into a wall. It has its own button.
  const allowed = search.handles ?? ["controlPoint", "endPoint", "previousEndPoint"];
  return bestFixAcross(handlesForLine(input, options, lineId, allowed), input, options, search);
}

/**
 * Finds the smallest move of the route's start point that gets the robot clear.
 *
 * Offered on its own because the start pose is a problem the paths cannot fix:
 * the robot is already sitting in the obstacle before it drives anywhere.
 */
export function suggestStartPointFix(
  input: ClearanceInput,
  options: ClearanceOptions,
  search: ClearanceFixSearch = {},
): ClearanceFix | null {
  if (!movablePoint(input.startPoint)) return null;

  // Re-staging the robot a foot or two is an ordinary thing to do, and a robot
  // parked inside a goal has to come most of its own width to get out, so this
  // reaches further than a mid-path nudge does.
  const reach: ClearanceFixSearch = { maxDistance: 24, ...search };

  const routeLines = routeLinesOf({ ...input, measureLineIds: new Set<string>() });
  const first = routeLines[0];
  if (!first) return null;

  // Moving where the robot is staged moves the start of the first path, so that
  // is what has to come out clear.
  const measured = new Set<string>([first.line.id || ""]);
  return bestFixAcross([startPointHandle(input, measured, options)], input, options, reach);
}

/**
 * Runs the spiral search over each handle in turn and returns the first result
 * that reaches the margin, falling back to the best partial improvement.
 */
function bestFixAcross(
  handles: FixHandle[],
  input: ClearanceInput,
  options: ClearanceOptions,
  search: ClearanceFixSearch,
): ClearanceFix | null {
  const maxDistance = Number(search.maxDistance) > 0 ? Number(search.maxDistance) : 14;
  const stepSize = Number(search.stepSize) > 0 ? Number(search.stepSize) : 0.5;
  const directions = Number(search.directions) > 0 ? Math.round(Number(search.directions)) : 12;

  // The scan tries hundreds of candidates per handle, so it samples the route
  // coarsely; whatever it settles on is then re-measured at full resolution, and
  // a candidate that does not survive that is discarded. Scanning at the real
  // resolution made a single click take most of a second.
  const scanOptions: ClearanceOptions = {
    ...options,
    step: Math.max(Number(options.step) || DEFAULT_CLEARANCE_STEP, SEARCH_SCAN_STEP),
  };

  const margin = Math.max(0, Number(options.margin) || 0);
  // Getting off the obstacle is the job; reaching the margin as well is a bonus.
  // They are separate targets because a single handle often cannot reach the
  // margin — the tightest point on a path may be somewhere it has no leverage.
  const clearTarget = 1e-3;
  const marginTarget = Math.max(margin, clearTarget);

  let bestPartial: ClearanceFix | null = null;

  for (const handle of handles) {
    const score = handle.score ?? ((report: ClearanceReport) => report.minClearance);
    const measureWith = (dx: number, dy: number, at: ClearanceOptions): number =>
      score(
        checkClearance(
          { ...handle.apply(dx, dy), measureLineIds: handle.measureLineIds },
          at,
        ),
      );

    const scan = (dx: number, dy: number) => measureWith(dx, dy, scanOptions);
    const measure = (dx: number, dy: number) => measureWith(dx, dy, options);

    const before = measure(0, 0);
    if (before >= marginTarget) continue;

    const asFix = (
      candidate: { dx: number; dy: number; value: number },
      meetsMargin: boolean,
    ): ClearanceFix => ({
      handle: handle.kind,
      lineId: handle.lineId,
      controlPointIndex: handle.controlPointIndex,
      dx: candidate.dx,
      dy: candidate.dy,
      distance: Math.hypot(candidate.dx, candidate.dy),
      before,
      after: candidate.value,
      meetsMargin,
    });

    const perRadius: Array<{ dx: number; dy: number; value: number; radius: number }> = [];
    let reachedMargin: ClearanceFix | null = null;

    for (let radius = stepSize; radius <= maxDistance + 1e-9; radius += stepSize) {
      let bestAtRadius: { dx: number; dy: number; value: number } | null = null;

      for (let i = 0; i < directions; i++) {
        const angle = (2 * Math.PI * i) / directions;
        const dx = radius * Math.cos(angle);
        const dy = radius * Math.sin(angle);
        const value = scan(dx, dy);
        if (!bestAtRadius || value > bestAtRadius.value) {
          bestAtRadius = { dx, dy, value };
        }
      }

      if (!bestAtRadius) break;
      perRadius.push({ ...bestAtRadius, radius });

      // The first radius that reaches the margin is the smallest move that
      // does, which is the one least likely to undo whatever the path was
      // placed for. Refine and confirm at full resolution before believing it.
      if (bestAtRadius.value >= marginTarget) {
        const refined = refineAlongDirection(
          measure,
          bestAtRadius.dx,
          bestAtRadius.dy,
          radius,
          Math.max(0, radius - stepSize),
          marginTarget,
        );
        if (refined.value >= marginTarget && meaningful(refined)) {
          reachedMargin = asFix(refined, true);
          break;
        }
        // The coarse scan was optimistic here; keep looking.
      }
    }

    // A handle that fully solves it ends the search: the ones after it are
    // ordered as more disruptive, so there is nothing better left to find.
    if (reachedMargin) return reachedMargin;

    // The margin is out of reach for this handle. Aim for the plateau rather
    // than for bare contact-free — stopping at the first move that merely gets
    // off the obstacle would leave the robot at 0.00in when a slightly larger
    // move buys real room.
    const reachable = perRadius.reduce((most, entry) => Math.max(most, entry.value), -Infinity);
    if (!(reachable >= clearTarget)) continue;

    const target = reachable - 0.02;
    const smallest = perRadius.find((entry) => entry.value >= target);
    if (!smallest) continue;

    const refined = refineAlongDirection(
      measure,
      smallest.dx,
      smallest.dy,
      smallest.radius,
      Math.max(0, smallest.radius - stepSize),
      target,
    );

    // Confirmed at full resolution, or not offered. The coarse scan can be
    // optimistic, and a move that reshapes a path by a foot and leaves it still
    // overlapping is worse than leaving it alone and saying so.
    if (refined.value < clearTarget) continue;
    if (refined.value < before + MIN_FIX_GAIN) continue;
    if (!meaningful(refined)) continue;

    const partial = asFix(refined, false);
    if (!bestPartial || partial.after > bestPartial.after) bestPartial = partial;
  }

  // Nothing reached the margin; offer whichever handle got furthest, as long as
  // it at least gets the robot off the obstacle.
  return bestPartial;
}

/**
 * Walks back along a direction that already works to find the shortest move
 * that still does, so the handle lands as close to where it was as possible.
 */
function refineAlongDirection(
  measure: (dx: number, dy: number) => number,
  dx: number,
  dy: number,
  radius: number,
  floor: number,
  target: number,
): { dx: number; dy: number; value: number } {
  const length = Math.hypot(dx, dy);
  if (!(length > 0)) return { dx, dy, value: measure(dx, dy) };

  const unitX = dx / length;
  const unitY = dy / length;

  let low = floor;
  let high = radius;
  let bestValue = measure(dx, dy);

  for (let i = 0; i < 8; i++) {
    const middle = (low + high) / 2;
    const value = measure(unitX * middle, unitY * middle);
    if (value >= target) {
      high = middle;
      bestValue = value;
    } else {
      low = middle;
    }
  }

  return { dx: unitX * high, dy: unitY * high, value: bestValue };
}

/** One-line description of a span, for chips and warnings. */
export function describeClearanceSpan(span: ClearanceSpan): string {
  const target =
    span.kind === "wall"
      ? "field wall"
      : span.obstacleName?.trim() || "obstacle";

  if (span.severity === "hit") {
    return `Hits ${target} at ${span.startDistance.toFixed(1)}in`;
  }
  return `${span.worstClearance.toFixed(1)}in from ${target}`;
}
