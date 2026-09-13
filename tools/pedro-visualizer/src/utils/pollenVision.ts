/**
 * The geometry behind a Pollen Pickup step.
 *
 * PollenDetectionPipeline reports clumps in PIXELS. Driving to one needs a
 * place on the FIELD, and the only thing that connects the two is the camera's
 * mount: where the lens sits on the robot, how far it is tilted down, and how
 * wide it sees. This module is that connection, written once, so the editor's
 * overlay, its warnings, the time estimate and the generated Java all agree
 * about what the robot can see and where it will drive.
 *
 * Frames:
 *   field  - visualizer / Pedro inches, heading degrees counter-clockwise
 *            from +x, the same frame as every path.
 *   robot  - +x forward, +y left, +z up from the TILES, origin at robot centre.
 *   camera - +x right, +y down, +z out of the lens (OpenCV's convention).
 *   image  - u right, v down, pixels, (0, 0) at the top-left corner.
 *
 * The ground model is a flat floor with POLLEN centres at the ball radius.
 * That is exactly right for a ball sitting on the TILES and the reason a
 * mis-measured camera height or pitch moves every estimate: the generated
 * step re-plans as it closes in, where the error shrinks with range.
 */
import type {
  BasePoint,
  PollenPipelineSettings,
  SequencePollenItem,
  Settings,
  VisionSettings,
} from "../types";
import {
  DEFAULT_POLLEN_PIPELINE,
  DEFAULT_VISION_SETTINGS,
  FIELD_SIZE,
} from "../config/defaults";
import { pointInPolygon } from "./geometry";
import { calculateMotionProfileTime } from "./timeCalculator";

/** BIOBUZZ Competition Manual V1 Sec 9.8: POLLEN are ~2.8 in. balls. */
export const POLLEN_DIAMETER_INCHES = 2.8;
export const POLLEN_RADIUS_INCHES = POLLEN_DIAMETER_INCHES / 2;

/**
 * Fraction of max velocity the intake creep drives at. The generated Java uses
 * the same constant, so the time estimate matches what runs.
 */
export const INTAKE_CREEP_SPEED = 0.35;

export interface FieldPose {
  x: number;
  y: number;
  /** Degrees, counter-clockwise from +x. */
  heading: number;
}

const DEG = Math.PI / 180;
const finite = (value: unknown, fallback: number) =>
  Number.isFinite(Number(value)) ? Number(value) : fallback;
const clamp = (value: number, low: number, high: number) =>
  Math.max(low, Math.min(high, value));

/* ------------------------------------------------------------------------
 * Settings
 * --------------------------------------------------------------------- */

function normalizeTriple(
  input: unknown,
  fallback: [number, number, number],
  highs: [number, number, number],
): [number, number, number] {
  const values = Array.isArray(input) ? input : [];
  return [0, 1, 2].map((i) =>
    Math.round(clamp(finite(values[i], fallback[i]), 0, highs[i])),
  ) as [number, number, number];
}

const HSV_HIGHS: [number, number, number] = [179, 255, 255];

export function normalizePipelineSettings(input: unknown): PollenPipelineSettings {
  const source = (input && typeof input === "object" ? input : {}) as Partial<PollenPipelineSettings>;
  const d = DEFAULT_POLLEN_PIPELINE;
  return {
    hsvLowA: normalizeTriple(source.hsvLowA, d.hsvLowA, HSV_HIGHS),
    hsvHighA: normalizeTriple(source.hsvHighA, d.hsvHighA, HSV_HIGHS),
    hsvLowB: normalizeTriple(source.hsvLowB, d.hsvLowB, HSV_HIGHS),
    hsvHighB: normalizeTriple(source.hsvHighB, d.hsvHighB, HSV_HIGHS),
    openRadius: Math.round(clamp(finite(source.openRadius, d.openRadius), 0, 50)),
    closeHGap: Math.round(clamp(finite(source.closeHGap, d.closeHGap), 0, 200)),
    closeVRadius: Math.round(clamp(finite(source.closeVRadius, d.closeVRadius), 0, 200)),
    minArea: clamp(finite(source.minArea, d.minArea), 1, 10_000_000),
    maxArea: clamp(finite(source.maxArea, d.maxArea), 1, 10_000_000),
    maxAspect: clamp(finite(source.maxAspect, d.maxAspect), 1, 100),
    clumpMergeGap: Math.round(clamp(finite(source.clumpMergeGap, d.clumpMergeGap), 0, 500)),
    singleBallAreaPx: clamp(finite(source.singleBallAreaPx, d.singleBallAreaPx), 1, 10_000_000),
  };
}

/**
 * Fills in anything missing and pulls everything into a usable range, always
 * returning a new object - the editor binds to these fields, and mutating a
 * shared default would change every project at once.
 */
export function normalizeVisionSettings(input: unknown): VisionSettings {
  const source = (input && typeof input === "object" ? input : {}) as Partial<VisionSettings>;
  const d = DEFAULT_VISION_SETTINGS;
  return {
    cameraName:
      typeof source.cameraName === "string" && source.cameraName.trim()
        ? source.cameraName.trim()
        : d.cameraName,
    imageWidth: Math.round(clamp(finite(source.imageWidth, d.imageWidth), 16, 4096)),
    imageHeight: Math.round(clamp(finite(source.imageHeight, d.imageHeight), 16, 4096)),
    horizontalFovDeg: clamp(finite(source.horizontalFovDeg, d.horizontalFovDeg), 5, 170),
    verticalFovDeg: clamp(finite(source.verticalFovDeg, d.verticalFovDeg), 5, 170),
    mountForwardInches: clamp(finite(source.mountForwardInches, d.mountForwardInches), -30, 30),
    mountLeftInches: clamp(finite(source.mountLeftInches, d.mountLeftInches), -30, 30),
    mountHeightInches: clamp(finite(source.mountHeightInches, d.mountHeightInches), 0.5, 48),
    mountPitchDeg: clamp(finite(source.mountPitchDeg, d.mountPitchDeg), -89, 89),
    mountYawDeg: clamp(finite(source.mountYawDeg, d.mountYawDeg), -180, 180),
    measured: Boolean(source.measured),
    maxRangeInches: clamp(finite(source.maxRangeInches, d.maxRangeInches), 6, 200),
    pipeline: normalizePipelineSettings(source.pipeline),
  };
}

/** The vision settings a project uses, whether or not it has saved any. */
export function visionOf(settings: Pick<Settings, "vision"> | undefined): VisionSettings {
  return normalizeVisionSettings(settings?.vision);
}

/* ------------------------------------------------------------------------
 * Projection
 * --------------------------------------------------------------------- */

interface CameraBasis {
  /** Lens position in the robot frame. */
  position: [number, number, number];
  /** Camera axes expressed in the robot frame. */
  right: [number, number, number];
  down: [number, number, number];
  forward: [number, number, number];
  fx: number;
  fy: number;
  cx: number;
  cy: number;
}

function cameraBasis(vision: VisionSettings): CameraBasis {
  const pitch = vision.mountPitchDeg * DEG;
  const yaw = vision.mountYawDeg * DEG;
  const cosP = Math.cos(pitch);
  const sinP = Math.sin(pitch);
  const cosY = Math.cos(yaw);
  const sinY = Math.sin(yaw);

  // Unyawed axes: the level camera (right = -y, down = -z, forward = +x)
  // rotated about the robot's y axis by the pitch. Tilting down swings the
  // image's "down" toward the robot's feet, so it picks up a -x component; the
  // three stay orthonormal, which fieldToPixel relies on.
  const yawed = (v: [number, number, number]): [number, number, number] => [
    v[0] * cosY - v[1] * sinY,
    v[0] * sinY + v[1] * cosY,
    v[2],
  ];

  return {
    position: [vision.mountForwardInches, vision.mountLeftInches, vision.mountHeightInches],
    right: yawed([0, -1, 0]),
    down: yawed([-sinP, 0, -cosP]),
    forward: yawed([cosP, 0, -sinP]),
    fx: vision.imageWidth / 2 / Math.tan((vision.horizontalFovDeg * DEG) / 2),
    fy: vision.imageHeight / 2 / Math.tan((vision.verticalFovDeg * DEG) / 2),
    cx: vision.imageWidth / 2,
    cy: vision.imageHeight / 2,
  };
}

const dot = (a: [number, number, number], b: [number, number, number]) =>
  a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/**
 * Where a pixel lands on the floor, in the robot frame, for a ball whose
 * centre is `centreHeight` above the TILES. Null when the ray never comes down
 * to that height in front of the camera (at or above the horizon).
 */
export function pixelToRobot(
  u: number,
  v: number,
  vision: VisionSettings,
  centreHeight = POLLEN_RADIUS_INCHES,
): { x: number; y: number } | null {
  const basis = cameraBasis(vision);
  const xn = (u - basis.cx) / basis.fx;
  const yn = (v - basis.cy) / basis.fy;
  const ray: [number, number, number] = [
    xn * basis.right[0] + yn * basis.down[0] + basis.forward[0],
    xn * basis.right[1] + yn * basis.down[1] + basis.forward[1],
    xn * basis.right[2] + yn * basis.down[2] + basis.forward[2],
  ];

  const drop = basis.position[2] - centreHeight;
  if (ray[2] >= -1e-9 || drop <= 0) return null;
  const t = drop / -ray[2];
  return {
    x: basis.position[0] + t * ray[0],
    y: basis.position[1] + t * ray[1],
  };
}

/** A robot-frame point placed on the field at a pose. */
export function robotToField(pose: FieldPose, point: { x: number; y: number }): { x: number; y: number } {
  const h = pose.heading * DEG;
  const c = Math.cos(h);
  const s = Math.sin(h);
  return {
    x: pose.x + point.x * c - point.y * s,
    y: pose.y + point.x * s + point.y * c,
  };
}

/** The inverse of robotToField. */
export function fieldToRobot(pose: FieldPose, point: { x: number; y: number }): { x: number; y: number } {
  const h = pose.heading * DEG;
  const c = Math.cos(h);
  const s = Math.sin(h);
  const dx = point.x - pose.x;
  const dy = point.y - pose.y;
  return { x: dx * c + dy * s, y: -dx * s + dy * c };
}

export function pixelToField(
  u: number,
  v: number,
  pose: FieldPose,
  vision: VisionSettings,
): { x: number; y: number } | null {
  const robot = pixelToRobot(u, v, vision);
  return robot ? robotToField(pose, robot) : null;
}

/**
 * Where a POLLEN centre at a field point appears in the image, or null when it
 * is behind the lens. The pixel can be outside the image; `inImage` says.
 */
export function fieldToPixel(
  point: { x: number; y: number },
  pose: FieldPose,
  vision: VisionSettings,
): { u: number; v: number; depth: number; inImage: boolean } | null {
  const basis = cameraBasis(vision);
  const robot = fieldToRobot(pose, point);
  const rel: [number, number, number] = [
    robot.x - basis.position[0],
    robot.y - basis.position[1],
    POLLEN_RADIUS_INCHES - basis.position[2],
  ];
  const zc = dot(rel, basis.forward);
  if (zc <= 1e-6) return null;
  const u = basis.cx + (basis.fx * dot(rel, basis.right)) / zc;
  const v = basis.cy + (basis.fy * dot(rel, basis.down)) / zc;
  return {
    u,
    v,
    depth: zc,
    inImage: u >= 0 && u <= vision.imageWidth && v >= 0 && v <= vision.imageHeight,
  };
}

/**
 * How many pixels one POLLEN covers at a field point, the number the pipeline's
 * MIN_AREA filter is compared against. A projected disc: angular diameter
 * across and down, times the focal lengths.
 */
export function pollenPixelArea(
  point: { x: number; y: number },
  pose: FieldPose,
  vision: VisionSettings,
): number {
  const basis = cameraBasis(vision);
  const robot = fieldToRobot(pose, point);
  const rel: [number, number, number] = [
    robot.x - basis.position[0],
    robot.y - basis.position[1],
    POLLEN_RADIUS_INCHES - basis.position[2],
  ];
  const distance = Math.hypot(rel[0], rel[1], rel[2]);
  if (distance <= POLLEN_RADIUS_INCHES) return Infinity;
  const angular = 2 * Math.atan(POLLEN_RADIUS_INCHES / distance);
  return (Math.PI / 4) * (basis.fx * angular) * (basis.fy * angular);
}

/**
 * The furthest a single POLLEN can sit, straight out along the camera, and
 * still be big enough to pass the pipeline's MIN_AREA filter. Past this the
 * pipeline drops it as noise, however well the camera is aimed.
 */
export function reliableRangeInches(vision: VisionSettings): number {
  const minArea = vision.pipeline.minArea;
  const pose: FieldPose = { x: 0, y: 0, heading: 0 };
  const along = (g: number) => {
    const yaw = vision.mountYawDeg * DEG;
    return {
      x: vision.mountForwardInches + g * Math.cos(yaw),
      y: vision.mountLeftInches + g * Math.sin(yaw),
    };
  };
  if (pollenPixelArea(along(0.01), pose, vision) < minArea) return 0;

  let low = 0;
  let high = vision.maxRangeInches;
  if (pollenPixelArea(along(high), pose, vision) >= minArea) return high;
  for (let i = 0; i < 40; i++) {
    const middle = (low + high) / 2;
    if (pollenPixelArea(along(middle), pose, vision) >= minArea) low = middle;
    else high = middle;
  }
  return low;
}

/**
 * The patch of floor the camera sees, as a field polygon, cut off at
 * `rangeLimit` from the lens. Walks the image border and projects each point;
 * a ray that never reaches the floor (above the horizon) or reaches it past
 * the limit is pulled back to the limit along its own bearing.
 */
export function cameraFootprint(
  pose: FieldPose,
  vision: VisionSettings,
  rangeLimit = vision.maxRangeInches,
  samplesPerEdge = 12,
): BasePoint[] {
  const W = vision.imageWidth;
  const H = vision.imageHeight;
  const basis = cameraBasis(vision);
  const lens = { x: basis.position[0], y: basis.position[1] };

  const border: [number, number][] = [];
  for (let i = 0; i < samplesPerEdge; i++) border.push([(W * i) / samplesPerEdge, H]); // bottom, left->right
  for (let i = 0; i < samplesPerEdge; i++) border.push([W, H - (H * i) / samplesPerEdge]); // right, up
  for (let i = 0; i < samplesPerEdge; i++) border.push([W - (W * i) / samplesPerEdge, 0]); // top, right->left
  for (let i = 0; i < samplesPerEdge; i++) border.push([0, (H * i) / samplesPerEdge]); // left, down

  const points: BasePoint[] = [];
  for (const [u, v] of border) {
    let robot = pixelToRobot(u, v, vision);
    if (!robot) {
      // Above the horizon: keep the bearing, stop at the limit.
      const xn = (u - basis.cx) / basis.fx;
      const bearing = Math.atan2(
        xn * basis.right[1] + basis.forward[1],
        xn * basis.right[0] + basis.forward[0],
      );
      robot = { x: lens.x + rangeLimit * Math.cos(bearing), y: lens.y + rangeLimit * Math.sin(bearing) };
    } else {
      const dx = robot.x - lens.x;
      const dy = robot.y - lens.y;
      const range = Math.hypot(dx, dy);
      if (range > rangeLimit) {
        robot = { x: lens.x + (dx / range) * rangeLimit, y: lens.y + (dy / range) * rangeLimit };
      }
    }
    points.push(robotToField(pose, robot));
  }
  return points;
}

/* ------------------------------------------------------------------------
 * Planning a step
 * --------------------------------------------------------------------- */

export type PollenFindingLevel = "error" | "warning" | "note";

export interface PollenFinding {
  level: PollenFindingLevel;
  message: string;
}

export interface PollenLeg {
  phase: "search" | "approach" | "intake" | "return";
  from: FieldPose;
  to: FieldPose;
  seconds: number;
}

export interface PollenStepPlan {
  searchPose: FieldPose;
  target: { x: number; y: number };
  /** Where the approach stops, facing the POLLEN. */
  standoffPose: FieldPose;
  /** Where the intake creep ends. */
  intakeEndPose: FieldPose;
  legs: PollenLeg[];
  /** Seconds the time estimate charges, with the search at its expected length. */
  expectedSeconds: number;
  /** Seconds if the search runs to its timeout and the pickup still happens. */
  worstSeconds: number;
  /** Camera view from the search pose, clipped to where a single ball still registers. */
  reliableFootprint: BasePoint[];
  /** Camera view from the search pose, clipped to the max range. */
  footprint: BasePoint[];
  reliableRange: number;
  /** How the expected POLLEN looks from the search pose. */
  targetPixel: { u: number; v: number; inImage: boolean } | null;
  targetPixelArea: number;
  /** Estimated balls a single POLLEN at the target reads as. */
  targetBallEstimate: number;
  findings: PollenFinding[];
}

export interface PollenPlanContext {
  settings: Pick<Settings, "maxVelocity" | "maxAcceleration" | "maxDeceleration" | "vision" | "autoMidline">;
  fieldSize?: number;
  /** True when anything that moves the robot runs after this step. */
  followedByPaths?: boolean;
}

const headingTo = (from: { x: number; y: number }, to: { x: number; y: number }) =>
  (Math.atan2(to.y - from.y, to.x - from.x) * 180) / Math.PI;

function legSeconds(
  distance: number,
  speed: number,
  settings: PollenPlanContext["settings"],
): number {
  if (!(distance > 1e-6)) return 0;
  const fraction = clamp(speed, 0.05, 1);
  const vMax = (Number(settings.maxVelocity) || 40) * fraction;
  const aMax = (Number(settings.maxAcceleration) || 30) * fraction;
  const dMax = (Number(settings.maxDeceleration ?? settings.maxAcceleration) || 30) * fraction;
  return calculateMotionProfileTime(distance, vMax, aMax, dMax);
}

/** Every number a Pollen Pickup step is allowed to hold, pulled into range. */
export function normalizePollenItem(item: Partial<SequencePollenItem> & { id: string }): SequencePollenItem {
  return {
    kind: "pollen",
    id: item.id,
    name: typeof item.name === "string" && item.name.trim() ? item.name : "Pollen Pickup",
    targetX: clamp(finite(item.targetX, FIELD_SIZE / 2), 0, FIELD_SIZE),
    targetY: clamp(finite(item.targetY, FIELD_SIZE / 2), 0, FIELD_SIZE),
    zoneRadius: clamp(finite(item.zoneRadius, 12), 1, 100),
    minBalls: Math.round(clamp(finite(item.minBalls, 1), 1, 20)),
    timeoutMs: Math.round(clamp(finite(item.timeoutMs, 1500), 0, 30000)),
    expectedSearchMs: Math.round(clamp(finite(item.expectedSearchMs, 300), 0, 30000)),
    onTimeout: item.onTimeout === "blind" ? "blind" : "skip",
    standoffInches: clamp(finite(item.standoffInches, 10), 0, 60),
    retargetInches: clamp(finite(item.retargetInches, 3), 0.5, 48),
    approachSpeed: clamp(finite(item.approachSpeed, 0.6), 0.05, 1),
    intakeDriveInches: clamp(finite(item.intakeDriveInches, 8), 0, 48),
    intakeMs: Math.round(clamp(finite(item.intakeMs, 600), 0, 30000)),
    intakeEvent: typeof item.intakeEvent === "string" ? item.intakeEvent : "Intake",
    returnToStart: item.returnToStart !== false,
    enabledExpression: item.enabledExpression,
    locked: Boolean(item.locked),
  };
}

export function planPollenStep(
  rawItem: SequencePollenItem,
  searchPose: FieldPose,
  context: PollenPlanContext,
): PollenStepPlan {
  const item = normalizePollenItem(rawItem);
  const vision = visionOf(context.settings);
  const fieldSize = context.fieldSize ?? FIELD_SIZE;
  const settings = context.settings;
  const target = { x: item.targetX, y: item.targetY };
  const findings: PollenFinding[] = [];

  // --- the legs -------------------------------------------------------------
  const approachHeading = headingTo(searchPose, target);
  const distance = Math.hypot(target.x - searchPose.x, target.y - searchPose.y);
  const unit = distance > 1e-6
    ? { x: (target.x - searchPose.x) / distance, y: (target.y - searchPose.y) / distance }
    : { x: Math.cos(searchPose.heading * DEG), y: Math.sin(searchPose.heading * DEG) };
  const standoffDistance = Math.max(0, distance - item.standoffInches);
  const facing = distance > 1e-6 ? approachHeading : searchPose.heading;

  const standoffPose: FieldPose = {
    x: searchPose.x + unit.x * standoffDistance,
    y: searchPose.y + unit.y * standoffDistance,
    heading: facing,
  };
  const intakeEndPose: FieldPose = {
    x: standoffPose.x + unit.x * item.intakeDriveInches,
    y: standoffPose.y + unit.y * item.intakeDriveInches,
    heading: facing,
  };

  const search: PollenLeg = {
    phase: "search",
    from: searchPose,
    to: searchPose,
    seconds: item.expectedSearchMs / 1000,
  };
  const approach: PollenLeg = {
    phase: "approach",
    from: searchPose,
    to: standoffPose,
    seconds: legSeconds(standoffDistance, item.approachSpeed, settings),
  };
  const intake: PollenLeg = {
    phase: "intake",
    from: standoffPose,
    to: intakeEndPose,
    seconds: Math.max(
      item.intakeMs / 1000,
      legSeconds(item.intakeDriveInches, INTAKE_CREEP_SPEED, settings),
    ),
  };
  const back = Math.hypot(intakeEndPose.x - searchPose.x, intakeEndPose.y - searchPose.y);
  const returning: PollenLeg = {
    phase: "return",
    from: intakeEndPose,
    to: searchPose,
    seconds: legSeconds(back, 1, settings),
  };
  const legs = item.returnToStart
    ? [search, approach, intake, returning]
    : [search, approach, intake];

  const drive = approach.seconds + intake.seconds + (item.returnToStart ? returning.seconds : 0);
  const expectedSeconds = search.seconds + drive;
  const worstSeconds = item.timeoutMs / 1000 + drive;

  // --- what the camera sees from the search pose --------------------------------
  const reliableRange = reliableRangeInches(vision);
  const footprint = cameraFootprint(searchPose, vision, vision.maxRangeInches);
  const reliableFootprint = cameraFootprint(
    searchPose,
    vision,
    Math.min(reliableRange, vision.maxRangeInches),
  );
  const pixel = fieldToPixel(target, searchPose, vision);
  const targetPixelArea = pixel ? pollenPixelArea(target, searchPose, vision) : 0;
  const targetBallEstimate = targetPixelArea / vision.pipeline.singleBallAreaPx;

  // --- findings ----------------------------------------------------------------
  if (!vision.measured) {
    findings.push({
      level: "warning",
      message:
        "Camera mount not measured — where this step thinks POLLEN is depends on it. Measure it in Settings → Vision.",
    });
  }

  if (!pixel || !pixel.inImage) {
    findings.push({
      level: "error",
      message:
        "The expected POLLEN is outside the camera's view from where this step starts — aim the previous path's end heading at it, or move the step.",
    });
  } else if (targetPixelArea < vision.pipeline.minArea) {
    findings.push({
      level: "warning",
      message: `A single POLLEN at the expected point covers ~${Math.round(
        targetPixelArea,
      )}px, under the pipeline's MIN_AREA of ${Math.round(
        vision.pipeline.minArea,
      )}px — it will be filtered out as noise. Start closer (a ball registers out to ~${reliableRange.toFixed(
        0,
      )}in).`,
    });
  }

  // The pipeline counts balls by area against one calibrated size, so the same
  // single ball reads as several up close and as a fraction far away - and Min
  // balls is compared in those units.
  if (pixel?.inImage && targetPixelArea >= vision.pipeline.minArea) {
    const reads = Math.max(1, Math.round(targetBallEstimate));
    if (reads !== 1) {
      findings.push({
        level: item.minBalls > 1 && item.minBalls <= reads ? "warning" : "note",
        message: `A single POLLEN at the expected point reads as ~${reads} balls to the pipeline: it covers ~${Math.round(
          targetPixelArea,
        )}px and "One ball" is calibrated at ${Math.round(
          vision.pipeline.singleBallAreaPx,
        )}px. Min balls counts in those units${
          item.minBalls > 1 && item.minBalls <= reads
            ? `, so Min balls ${item.minBalls} still accepts one ball here`
            : ""
        }.`,
      });
    }
  }

  const zoneEdgePoints = Array.from({ length: 16 }, (_, i) => {
    const a = (i / 16) * Math.PI * 2;
    return { x: target.x + item.zoneRadius * Math.cos(a), y: target.y + item.zoneRadius * Math.sin(a) };
  });
  const zoneOutOfView = zoneEdgePoints.filter(
    (point) => !pointInPolygon([point.x, point.y], footprint),
  ).length;
  if (pixel?.inImage && zoneOutOfView > 0) {
    findings.push({
      level: "note",
      message: `Part of the search zone is outside the camera's view (${Math.round(
        (zoneOutOfView / zoneEdgePoints.length) * 100,
      )}% of its edge) — POLLEN there will not be found from this pose.`,
    });
  }

  const midline = settings.autoMidline;
  if (midline === "red" || midline === "blue") {
    const middle = fieldSize / 2;
    const wrongSide = (x: number) => (midline === "red" ? x > middle : x < middle);
    if (wrongSide(target.x)) {
      findings.push({
        level: "error",
        message: `The expected POLLEN is on the ${
          midline === "red" ? "blue" : "red"
        } side of the midline. Detections past it are rejected, so this step can never pick it up.`,
      });
    } else if (wrongSide(target.x + (midline === "red" ? item.zoneRadius : -item.zoneRadius))) {
      findings.push({
        level: "note",
        message: "The search zone reaches across the midline. Clumps past it are rejected at runtime.",
      });
    }
  }

  if (distance < item.standoffInches) {
    findings.push({
      level: "note",
      message: `The robot already starts within the ${item.standoffInches}in standoff, so there is no approach — it goes straight to the intake.`,
    });
  }

  if (!item.returnToStart && context.followedByPaths) {
    findings.push({
      level: "warning",
      message:
        "Return is off but paths follow this step. Those paths are planned from the search pose, and the robot will be wherever the POLLEN was.",
    });
  }

  if (item.onTimeout === "blind") {
    findings.push({
      level: "note",
      message: "On timeout the robot drives to the expected point anyway (blind pickup).",
    });
  }

  if (item.expectedSearchMs > item.timeoutMs) {
    findings.push({
      level: "warning",
      message: `Expected search time (${item.expectedSearchMs}ms) is longer than the timeout (${item.timeoutMs}ms).`,
    });
  }

  return {
    searchPose,
    target,
    standoffPose,
    intakeEndPose,
    legs,
    expectedSeconds,
    worstSeconds,
    reliableFootprint,
    footprint,
    reliableRange,
    targetPixel: pixel ? { u: pixel.u, v: pixel.v, inImage: pixel.inImage } : null,
    targetPixelArea,
    targetBallEstimate,
    findings,
  };
}

/** A new step aimed at a point, with the defaults the editor starts from. */
export function makePollenItem(id: string, target?: { x: number; y: number }): SequencePollenItem {
  return normalizePollenItem({
    id,
    name: "Pollen Pickup",
    targetX: target?.x,
    targetY: target?.y,
  });
}

/* ------------------------------------------------------------------------
 * Every step on a route
 * --------------------------------------------------------------------- */

interface TimelineLike {
  type: string;
  itemId?: string;
  atPoint?: { x: number; y: number };
  startHeading?: number;
}

interface RouteStepLike {
  kind: string;
  item?: { id?: string };
}

/**
 * Plans every Pollen Pickup the route runs, from the pose the timeline has the
 * robot in when each begins. The editor, the MCP review and the collision check
 * all come through here, so they describe one plan per step.
 */
export function planPollenSteps(
  timeline: TimelineLike[],
  sequence: { kind: string; id?: string }[],
  routeSteps: RouteStepLike[],
  fallbackPose: { x: number; y: number },
  settings: PollenPlanContext["settings"],
): Map<string, PollenStepPlan> {
  const plans = new Map<string, PollenStepPlan>();
  for (const event of timeline || []) {
    if (event.type !== "maneuver" || !event.itemId || plans.has(event.itemId)) continue;
    const item = sequence.find(
      (candidate) => candidate.kind === "pollen" && candidate.id === event.itemId,
    ) as SequencePollenItem | undefined;
    if (!item) continue;
    const at = routeSteps.findIndex(
      (step) => step.kind === "pollen" && step.item?.id === item.id,
    );
    plans.set(
      item.id,
      planPollenStep(
        item,
        {
          x: event.atPoint?.x ?? fallbackPose.x,
          y: event.atPoint?.y ?? fallbackPose.y,
          heading: Number(event.startHeading) || 0,
        },
        {
          settings,
          followedByPaths: routeSteps.slice(at + 1).some((step) => step.kind === "path"),
        },
      ),
    );
  }
  return plans;
}

/** The driven legs of those plans, in the shape the collision check walks. */
export function pollenLegsOf(
  plans: Map<string, PollenStepPlan>,
  sequence: { kind: string; id?: string; name?: string }[],
): { id: string; name?: string; phase: "approach" | "intake" | "return"; from: FieldPose; to: FieldPose }[] {
  const legs: { id: string; name?: string; phase: "approach" | "intake" | "return"; from: FieldPose; to: FieldPose }[] = [];
  plans.forEach((plan, id) => {
    const item = sequence.find((candidate) => candidate.kind === "pollen" && candidate.id === id);
    plan.legs.forEach((leg) => {
      if (leg.phase === "search") return;
      legs.push({ id, name: item?.name, phase: leg.phase, from: leg.from, to: leg.to });
    });
  });
  return legs;
}
