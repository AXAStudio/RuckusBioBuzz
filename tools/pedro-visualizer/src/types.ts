// Exported type definitions for use in Svelte and TS modules

export interface BasePoint {
  x: number;
  y: number;
  locked?: boolean;
  poseVariableId?: string;
}

export interface PoseVariable {
  id: string;
  name: string;
  x: number;
  y: number;
  heading: number;
}

export interface PathVariable {
  id: string;
  name: string;
  startPoint: Point;
  lines: Line[];
}

export interface NumberVariable {
  id: string;
  name: string;
  value: number;
}

export type Point = BasePoint &
  (
    | {
        heading: "linear";
        startDeg: number;
        endDeg: number;
        headingCurve?: number;
        degrees?: never;
        reverse?: never;
      }
    | {
        heading: "constant";
        degrees: number;
        startDeg?: never;
        endDeg?: never;
        reverse?: never;
      }
    | {
        heading: "tangential";
        degrees?: never;
        startDeg?: never;
        endDeg?: never;
        reverse: boolean;
      }
  );

export type ControlPoint = BasePoint;


export interface WaitSegment {
  name?: string;
  durationMs: number;
  position?: "before" | "after";
}

export type EventTriggerType = "parametric" | "temporal" | "pose";

export interface EventMarker {
  id?: string;
  name: string;
  triggerType?: EventTriggerType;
  position: number;
  positionVariableId?: string;
  triggerMs?: number;
  triggerMsVariableId?: string;
  poseX?: number;
  poseXVariableId?: string;
  poseY?: number;
  poseYVariableId?: string;
  durationMs?: number;
  durationVariableId?: string;
}

export interface Line {
  id?: string;
  endPoint: Point;
  controlPoints: ControlPoint[];
  color: string;
  name?: string;
  speed?: number;
  speedVariableId?: string;
  locked?: boolean;
  waitBefore?: WaitSegment;
  waitAfter?: WaitSegment;
  waitBeforeMs?: number;
  waitAfterMs?: number;
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
  durationVariableId?: string;
  locked?: boolean;
};

export type SequenceEventItem = {
  kind: "event";
  id: string;
  name: string;
  durationMs: number;
  durationVariableId?: string;
  locked?: boolean;
};

export type SequenceRepeatItem = {
  kind: "repeat";
  id: string;
  name: string;
  count: number;
  countVariableId?: string;
  lineIds: string[];
  locked?: boolean;
};

export type SequenceItem =
  | SequencePathItem
  | SequenceWaitItem
  | SequenceEventItem
  | SequenceRepeatItem;

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
  showAutoCountdown?: boolean; // Show the 30 second autonomous countdown overlay
  showPathAnnotations?: boolean; // Show per-segment length and time labels
  showSwerveModules?: boolean; // Show estimated swerve wheel angles on robot previews
}

export interface Shape {
  id: string;
  name?: string;
  vertices: BasePoint[];
  color: string;
  fillColor: string;
}

export type TimelineEventType = "travel" | "wait";

export interface TimelineEvent {
  type: TimelineEventType;
  duration: number;
  startTime: number;
  endTime: number;
  name?: string;
  waitPosition?: "before" | "after";
  lineIndex?: number; // for travel
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
