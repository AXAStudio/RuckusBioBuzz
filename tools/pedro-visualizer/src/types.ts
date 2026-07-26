// Exported type definitions for use in Svelte and TS modules

export interface BasePoint {
  x: number;
  xExpression?: string;
  y: number;
  yExpression?: string;
  locked?: boolean;
  poseVariableId?: string;
}

/**
 * Every user-defined value lives in one `variables` array. The `type` field
 * discriminates the union, so the UI can offer a single "Variables" section
 * with a type dropdown instead of one section per kind.
 */
export type VariableType = "number" | "boolean" | "pose" | "path";

export interface VariableBase {
  id: string;
  name: string;
  type: VariableType;
  /** Optional free-form note shown as a tooltip in the variables list. */
  description?: string;
}

export interface NumberVariable extends VariableBase {
  type: "number";
  value: number;
  /** Expression that may reference other variables, e.g. "depth - 10". */
  valueExpression?: string;
}

export interface BooleanVariable extends VariableBase {
  type: "boolean";
  value: boolean;
  /** Expression evaluating to a boolean, e.g. "depth > 10 && isRed". */
  valueExpression?: string;
}

export interface PoseVariable extends VariableBase {
  type: "pose";
  x: number;
  xExpression?: string;
  y: number;
  yExpression?: string;
  heading: number;
  headingExpression?: string;
}

export interface PathVariable extends VariableBase {
  type: "path";
  startPoint: Point;
  lines: Line[];
}

export type Variable =
  | NumberVariable
  | BooleanVariable
  | PoseVariable
  | PathVariable;

/** Variables usable as scalars inside expressions. */
export type ScalarVariable = NumberVariable | BooleanVariable;

export type Point = BasePoint &
  (
    | {
        heading: "linear";
        startDeg: number;
        startDegExpression?: string;
        endDeg: number;
        endDegExpression?: string;
        headingCurve?: number;
        headingCurveExpression?: string;
        degrees?: never;
        degreesExpression?: never;
        reverse?: never;
        reverseExpression?: never;
      }
    | {
        heading: "constant";
        degrees: number;
        degreesExpression?: string;
        startDeg?: never;
        startDegExpression?: never;
        endDeg?: never;
        endDegExpression?: never;
        headingCurveExpression?: never;
        reverse?: never;
        reverseExpression?: never;
      }
    | {
        heading: "tangential";
        degrees?: never;
        degreesExpression?: never;
        startDeg?: never;
        startDegExpression?: never;
        endDeg?: never;
        endDegExpression?: never;
        headingCurveExpression?: never;
        reverse: boolean;
        /** Boolean expression driving `reverse`, e.g. "isRedAlliance". */
        reverseExpression?: string;
      }
  );

export type ControlPoint = BasePoint;


export interface WaitSegment {
  name?: string;
  durationMs: number;
  durationExpression?: string;
  position?: "before" | "after";
}

export type EventTriggerType = "parametric" | "temporal" | "pose";

export interface EventMarker {
  id?: string;
  name: string;
  triggerType?: EventTriggerType;
  position: number;
  positionExpression?: string;
  /** @deprecated Migrated to `positionExpression` on load. */
  positionVariableId?: string;
  triggerMs?: number;
  triggerMsExpression?: string;
  /** @deprecated Migrated to `triggerMsExpression` on load. */
  triggerMsVariableId?: string;
  poseX?: number;
  poseXExpression?: string;
  /** @deprecated Migrated to `poseXExpression` on load. */
  poseXVariableId?: string;
  poseY?: number;
  poseYExpression?: string;
  /** @deprecated Migrated to `poseYExpression` on load. */
  poseYVariableId?: string;
  durationMs?: number;
  durationExpression?: string;
  /** @deprecated Migrated to `durationExpression` on load. */
  durationVariableId?: string;
  /** Bind the whole pose trigger to a pose variable. */
  poseVariableId?: string;
  /** Boolean expression; when false the marker is skipped in export. */
  enabledExpression?: string;
}

export interface Line {
  id?: string;
  endPoint: Point;
  controlPoints: ControlPoint[];
  color: string;
  name?: string;
  speed?: number;
  speedExpression?: string;
  /** @deprecated Migrated to `speedExpression` on load. */
  speedVariableId?: string;
  locked?: boolean;
  /**
   * Ends the PathChain here, so the robot decelerates to a stop and settles on
   * this endpoint before the next path starts. Off by default: consecutive
   * paths are merged into one chain and driven without stopping.
   */
  stopAtEnd?: boolean;
  /** Boolean expression; when false the path is skipped in export. */
  enabledExpression?: string;
  waitBefore?: WaitSegment;
  waitAfter?: WaitSegment;
  waitBeforeMs?: number;
  waitBeforeExpression?: string;
  waitAfterMs?: number;
  waitAfterExpression?: string;
  waitBeforeName?: string;
  waitAfterName?: string;
  eventMarkers?: EventMarker[];
}

export type SequencePathItem = {
  kind: "path";
  lineId: string;
};

export type SequenceWaitItem = {
  kind: "wait";
  id: string;
  name: string;
  durationMs: number;
  durationExpression?: string;
  /** @deprecated Migrated to `durationExpression` on load. */
  durationVariableId?: string;
  enabledExpression?: string;
  locked?: boolean;
};

export type SequenceEventItem = {
  kind: "event";
  id: string;
  name: string;
  durationMs: number;
  durationExpression?: string;
  /** @deprecated Migrated to `durationExpression` on load. */
  durationVariableId?: string;
  enabledExpression?: string;
  locked?: boolean;
};

export type SequenceRepeatItem = {
  kind: "repeat";
  id: string;
  name: string;
  count: number;
  countExpression?: string;
  /** @deprecated Migrated to `countExpression` on load. */
  countVariableId?: string;
  enabledExpression?: string;
  lineIds: string[];
  locked?: boolean;
};

/**
 * An `if` block wrapping a group of paths, mirroring how a repeat loop wraps
 * them. The paths run only when `condition` resolves true; in generated code
 * the whole group is wrapped in a Java `if` so it stays switchable at runtime.
 */
export type SequenceConditionalItem = {
  kind: "conditional";
  id: string;
  name: string;
  /** Boolean expression; empty means "always run". */
  condition?: string;
  lineIds: string[];
  locked?: boolean;
};

export type SequenceItem =
  | SequencePathItem
  | SequenceWaitItem
  | SequenceEventItem
  | SequenceRepeatItem
  | SequenceConditionalItem;

/** Sequence items that wrap a group of paths. */
export type SequenceGroupItem = SequenceRepeatItem | SequenceConditionalItem;

export interface PathChain {
  id: string;
  name: string;
  color: string;
  lineIds: string[];
}

export interface Settings {
  xVelocity: number;
  yVelocity: number;
  aVelocity: number;
  kFriction: number;
  rWidth: number;
  rHeight: number;
  safetyMargin: number;
  maxVelocity: number; // inches/sec
  maxAcceleration: number; // inches/sec²
  maxDeceleration?: number; // inches/sec²
  fieldMap: string;
  customFieldImage?: string; // Base64 data URL for custom field image
  robotImage?: string;
  theme: "light" | "dark" | "auto";
  showGhostPaths?: boolean; // Show collision overlays via ghost paths
  showOnionLayers?: boolean; // Show robot body at intervals along the path
  onionLayerSpacing?: number; // Distance in inches between onion layers
  onionColor?: string; // Color for onion-layer colliders
  onionNextPointOnly?: boolean; // When true, onion layers show only for the next point (UI-only for now)
  showHeadingArrow?: boolean; // Show arrow indicating robot heading direction
  headingArrowLength?: number; // Length of the heading arrow in pixels
  headingArrowColor?: string; // Color of the heading arrow
  headingArrowThickness?: number; // Thickness/stroke width of the heading arrow
  pathOpacity?: number; // Opacity of path lines (0-1)
  showVelocityGradient?: boolean; // Color paths by instantaneous motion-profile velocity
  showEventPins?: boolean; // Show labeled event trigger pins on the field
  showEventTimeline?: boolean; // Show colored parallel-event duration strips on paths
  showAutoCountdown?: boolean; // Show the 30 second autonomous countdown overlay
  showPathAnnotations?: boolean; // Show per-segment length and time labels
  showStopPoints?: boolean; // Mark endpoints where the robot comes to a stop
  /**
   * How much turning eats into driving speed, 0..1. Driving and turning share
   * one motor-power budget: at 1 the drivetrain is saturated so their costs add
   * up, at 0 there is enough headroom that they overlap for free. Calibrate it
   * against the robot.
   */
  turnCoupling?: number;
  /**
   * Sideways acceleration the wheels can hold before the follower loses the
   * line, in in/s². It caps speed through a curve at `sqrt(a · r)`, which is
   * what makes a corner cost time. Measure it by driving an arc until the robot
   * starts sliding.
   */
  maxLateralAcceleration?: number;
  showSwerveModules?: boolean; // Show estimated swerve wheel angles on robot previews
  /**
   * The robot's real outline, taken from a CAD upload, in the robot's own frame
   * with `+x` forward. Clearance checks use it instead of the bounding
   * rectangle, so a robot with a corner intake is not treated as a box that
   * fills every gap it passes. Stored with the size it was measured at so it
   * still fits if `rWidth`/`rHeight` are edited afterwards.
   */
  robotOutline?: {
    points: BasePoint[];
    lengthInches: number;
    widthInches: number;
  };
  /** Draw the spans where the robot is closer to something than the margin. */
  showClearance?: boolean;
}

export interface Shape {
  id: string;
  name?: string;
  vertices: BasePoint[];
  color: string;
  fillColor: string;
}

export type TimelineEventType = "travel" | "wait";

/**
 * The PathChain a travel segment belongs to. Consecutive paths are followed as
 * one chain, which PedroPathing drives without stopping in between, so the
 * motion profile spans the whole chain rather than each path.
 */
export interface ProfilePoint {
  /** Distance along the path, ascending from 0. */
  distance: number;
  /** 1 / radius at this point, in 1/inch. Zero on a straight. */
  curvature: number;
  /** Radians of heading change per inch travelled. Zero when facing is fixed. */
  headingRate: number;
}

/** A speed curve sampled along a move, respecting curvature limits. */
export interface MotionProfile {
  length: number;
  totalTime: number;
  /** Ascending distances; `velocities[i]` and `times[i]` line up with these. */
  distances: number[];
  velocities: number[];
  times: number[];
}

export interface ChainProfile {
  /** Index of the chain in the route, for grouping and stop counts. */
  index: number;
  /** When the chain starts moving, in seconds. */
  startTime: number;
  /** Combined length of every path in the chain, in inches. */
  length: number;
  /** Distance along the chain at which this path begins, in inches. */
  offset: number;
  /**
   * Time along the chain's own profile at which this path begins, ignoring any
   * time added for turning. Turning stretches a path's wall-clock duration, so
   * this is what maps a moment back onto the chain's speed curve.
   */
  enterTime: number;
  /** The path's duration on the chain profile alone, before turning is added. */
  translationDuration: number;
  maxVelocity: number;
  maxAcceleration: number;
  maxDeceleration: number;
  /**
   * The chain's sampled speed curve. Present whenever the route has been timed;
   * everything that needs a position or a speed reads it so the field, the
   * animation and the clock cannot drift apart.
   */
  profile?: MotionProfile;
}

export interface TimelineEvent {
  type: TimelineEventType;
  duration: number;
  startTime: number;
  endTime: number;
  name?: string;
  waitPosition?: "before" | "after";
  lineIndex?: number; // for travel
  /** Set on travel events: the chain this path is driven as part of. */
  chain?: ChainProfile;
  /**
   * Set on travel events: distance reached at evenly spaced curve parameters
   * along this path. A Bezier's parameter is not proportional to its arc
   * length, so this is what turns a distance back into the parameter to sample
   * the curve at — without it the robot surges and dawdles along a path it
   * should cross at a steady speed.
   */
  arcLengths?: Float32Array;
  /**
   * Set on travel events: the fraction of the path spent rotating to pick up
   * its heading goal, when the robot arrives facing a different way. Zero when
   * the headings already line up.
   */
  headingCatchUp?: number;
  /**
   * Where this travel segment starts, following the sequence. Repeat loops and
   * `if` blocks make this differ from the previous entry in `lines`, so the
   * animation must read it from here rather than recomputing by array index.
   */
  startPoint?: BasePoint;
  startHeading?: number;
  targetHeading?: number;
  atPoint?: BasePoint;
}

export interface TimePrediction {
  totalTime: number;
  segmentTimes: number[];
  totalDistance: number;
  timeline: TimelineEvent[];
}

export interface DirectorySettings {
  autoPathsDirectory: string;
}

export interface FileInfo {
  name: string;
  path: string;
  size: number;
  modified: Date;
  error?: string;
}
