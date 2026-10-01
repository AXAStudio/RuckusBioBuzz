import type {
  Point,
  Line,
  Shape,
  Settings,
  VisionSettings,
  PollenPipelineSettings,
} from "../types";
import { getRandomColor } from "../utils";
// Shared with the robot (TeamCode bundles it as a resource); see the BIOBUZZ preset below.
import biobuzzField from "../../../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/field/biobuzz_field.json";

/**
 * Default robot dimensions
 */
export const DEFAULT_ROBOT_WIDTH = 16;
export const DEFAULT_ROBOT_HEIGHT = 16;

/**
 * Default canvas drawing settings
 */
export const POINT_RADIUS = 1.15;
export const LINE_WIDTH = 0.57;
export const FIELD_SIZE = 141.5;

/**
 * Available field maps
 */
export const AVAILABLE_FIELD_MAPS = [
  { value: "biobuzz.webp", label: "BIOBUZZ Field (2026-2027)" },
  { value: "decode.webp", label: "DECODE Field (2025-2026)" },
  { value: "intothedeep.webp", label: "Into The Deep Field (2024-2025)" },
  { value: "centerstage.webp", label: "Centerstage (2023-2024)" },
  { value: "custom", label: "Custom Field (Upload)" },
];

/**
 * PollenDetectionPipeline's own constants, as TeamCode ships them
 * (TeamCode/.../pipelines/PollenDetectionPipeline.java). Exported into a
 * generated auto as a PollenDetectionPipeline.Config, so a value left here
 * behaves exactly like the pipeline with no config at all.
 */
export const DEFAULT_POLLEN_PIPELINE: PollenPipelineSettings = {
  hsvLowA: [15, 90, 70],
  hsvHighA: [38, 255, 255],
  hsvLowB: [12, 60, 45],
  hsvHighB: [42, 255, 255],
  openRadius: 3,
  closeHGap: 16,
  closeVRadius: 12,
  minArea: 350,
  maxArea: 100_000,
  maxAspect: 5,
  clumpMergeGap: 28,
  singleBallAreaPx: 1600,
};

/**
 * The camera model. Nothing below the pipeline block is measured: the webcam
 * name is the one TeamCode's blob test uses, the field of view is a typical
 * 640x480 webcam's, and the mount is a guess at a front-mounted camera tilted
 * down. `measured: false` keeps every Pollen Pickup saying so until someone
 * measures the real robot and ticks it.
 */
export const DEFAULT_VISION_SETTINGS: VisionSettings = {
  cameraName: "Webcam 1",
  imageWidth: 640,
  imageHeight: 480,
  horizontalFovDeg: 62,
  verticalFovDeg: 48,
  mountForwardInches: 8,
  mountLeftInches: 0,
  mountHeightInches: 10,
  mountPitchDeg: 25,
  mountYawDeg: 0,
  measured: false,
  maxRangeInches: 96,
  pipeline: DEFAULT_POLLEN_PIPELINE,
};

/** A fresh copy, so an edit to one project's settings never leaks into the defaults. */
export function defaultVisionSettings(): VisionSettings {
  return JSON.parse(JSON.stringify(DEFAULT_VISION_SETTINGS));
}

/**
 * Default settings
 */
export const DEFAULT_SETTINGS: Settings = {
  xVelocity: 75,
  yVelocity: 65,
  aVelocity: Math.PI,
  kFriction: 0.1,
  rWidth: DEFAULT_ROBOT_WIDTH,
  rHeight: DEFAULT_ROBOT_HEIGHT,
  safetyMargin: 1,
  maxVelocity: 40,
  maxAcceleration: 30,
  maxDeceleration: 30,
  fieldMap: "biobuzz.webp",
  robotImage: "/robot.png",
  theme: "auto",
  showGhostPaths: false,
  showOnionLayers: false,
  onionLayerSpacing: 3, // inches between each robot body trace
  onionColor: "#dc2626",
  onionNextPointOnly: false,
  showHeadingArrow: false,
  headingArrowLength: 50,
  headingArrowColor: "#ffffff",
  headingArrowThickness: 2,
  pathOpacity: 1,
  showVelocityGradient: false,
  showEventPins: true,
  showEventTimeline: true,
  showAutoCountdown: true,
  showPathAnnotations: false,
  showStopPoints: true,
  turnCoupling: 1,
  maxLateralAcceleration: 30,
  showSwerveModules: true,
  showClearance: true,
  autoMidline: "off",
  vision: DEFAULT_VISION_SETTINGS,
  stateVfx: {},
};

/**
 * Get default starting point
 */
export function getDefaultStartPoint(): Point {
  return {
    x: 56,
    y: 8,
    heading: "linear",
    startDeg: 90,
    endDeg: 180,
    locked: false,
  };
}

/**
 * Get default initial path lines
 */
export function getDefaultLines(): Line[] {
  return [
    {
      id: `line-${Math.random().toString(36).slice(2)}`,
      name: "Path 1",
      endPoint: { x: 56, y: 36, heading: "linear", startDeg: 90, endDeg: 180 },
      controlPoints: [],
      color: "#ffc516",
      speed: 1,
      locked: false,
      waitBeforeMs: 0,
      waitAfterMs: 0,
      waitBeforeName: "",
      waitAfterName: "",
    },
  ];
}

/**
 * Field obstacles, per field map.
 *
 * A preset is what the field itself puts in the robot's way at drive height -
 * not everything the field has. Anything the robot passes under is left out,
 * and said so below.
 */
const FIELD_OBSTACLE_PRESETS: Record<string, () => Shape[]> = {
  // DECODE (2025-2026): Pedro Pathing's own goal outlines, as upstream ships
  // them (Pedro-Pathing/Visualizer src/config/defaults.ts at e976a746).
  "decode.webp": () => [
    {
      id: "triangle-1",
      name: "Red Goal",
      vertices: [
        { x: 141.5, y: 70.0 },
        { x: 141.5, y: 141.5 },
        { x: 118.3, y: 141.5 },
        { x: 135.5, y: 118.0 },
        { x: 136.3, y: 70.2 },
      ],
      color: "#dc2626",
      fillColor: "#ff6b6b",
    },
    {
      id: "triangle-2",
      name: "Blue Goal",
      vertices: [
        { x: 6.2, y: 116.9 },
        { x: 25.0, y: 141.5 },
        { x: 0.0, y: 141.5 },
        { x: 0.0, y: 70.0 },
        { x: 6.0, y: 70.0 },
      ],
      color: "#2563eb",
      fillColor: "#60a5fa",
    },
  ],

  // BIOBUZZ (2026-2027). The obstacles live in TeamCode's
  // field/biobuzz_field.json - the one copy, shared with Swerve Bring-Up's
  // field fence on the robot. Measurement notes are in that file. Copied per
  // call, as every preset is, so an edit in the editor never reaches the import.
  "biobuzz.webp": () =>
    biobuzzField.obstacles.map((o) => ({
      id: o.id,
      name: o.name,
      vertices: o.vertices.map((v) => ({ x: v.x, y: v.y })),
      color: o.color,
      fillColor: o.fillColor,
    })),
};

/**
 * Get default shapes (field obstacles) for a field map. Field maps with no
 * preset - Into The Deep, Centerstage, and any custom image - start with no
 * obstacles rather than another season's.
 */
export function getDefaultShapes(
  fieldMap: string = DEFAULT_SETTINGS.fieldMap,
): Shape[] {
  const preset = FIELD_OBSTACLE_PRESETS[fieldMap];
  return preset ? preset() : [];
}

/**
 * True while the obstacles are still a preset as shipped (or none at all), so
 * switching field maps can swap them without asking. Colour is ignored -
 * position is what a path is checked against. Any other edit counts as the
 * user's own work and is only replaced on confirmation.
 */
export function isUntouchedObstaclePreset(shapes: Shape[]): boolean {
  if (!shapes || shapes.length === 0) {
    return true;
  }
  const key = (list: Shape[]) =>
    JSON.stringify(
      list.map((shape) => [
        shape.id,
        shape.name ?? "",
        shape.vertices.map((v) => [
          Math.round(v.x * 1000) / 1000,
          Math.round(v.y * 1000) / 1000,
        ]),
      ]),
    );
  const current = key(shapes);
  return Object.values(FIELD_OBSTACLE_PRESETS).some(
    (make) => key(make()) === current,
  );
}
