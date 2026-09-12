import type { Point, Line, Shape, Settings } from "../types";
import { getRandomColor } from "../utils";

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
  fieldMap: "decode.webp",
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
  "decode.webp": () => [
    {
      id: "triangle-1",
      name: "Red Goal",
      vertices: [
        { x: 141.5, y: 70 },
        { x: 141.5, y: 141.5 },
        { x: 120, y: 141.5 },
        { x: 138, y: 119 },
        { x: 138, y: 70 },
      ],
      color: "#dc2626",
      fillColor: "#ff6b6b",
    },
    {
      id: "triangle-2",
      name: "Blue Goal",
      vertices: [
        { x: 6, y: 119 },
        { x: 25, y: 141.5 },
        { x: 0, y: 141.5 },
        { x: 0, y: 70 },
        { x: 6, y: 70 },
      ],
      color: "#2563eb",
      fillColor: "#60a5fa",
    },
  ],

  // BIOBUZZ (2026-2027), from the Competition Manual V1 - the same numbers
  // scripts/gen_biobuzz_field.py draws biobuzz.webp from, in visualizer
  // coordinates (x = 0 the red wall, y = 0 the audience wall).
  //
  // Almost none of the HIVE Structure is in a ROBOT's way. The bottom of a
  // HIVE is 25.5 in. above the TILES (Fig 9-10) and an FTC ROBOT is at most
  // 18 in. tall, so a ROBOT drives under the CELLS, under the crossbar at
  // 43.95 in., and under the Frame legs, which climb from the rails to that
  // apex. What is left on the floor is the two base rails: ~2.0 in. wide,
  // 38.95 in. long (Fig 9-8 frame depth), 49.46 in. apart outside face to
  // outside face (Fig 9-8 frame width, measured on Fig 9-2's top view as
  // outside-to-outside with ~2.0 in. rails). Those two rectangles are the
  // whole centre-FIELD obstacle.
  "biobuzz.webp": () => [
    {
      id: "biobuzz-hive-rail-red",
      name: "HIVE Frame Rail - Red Side",
      vertices: [
        { x: 46.02, y: 51.28 },
        { x: 48.02, y: 51.28 },
        { x: 48.02, y: 90.22 },
        { x: 46.02, y: 90.22 },
      ],
      color: "#dc2626",
      fillColor: "#ff6b6b",
    },
    {
      id: "biobuzz-hive-rail-blue",
      name: "HIVE Frame Rail - Blue Side",
      vertices: [
        { x: 93.48, y: 51.28 },
        { x: 95.48, y: 51.28 },
        { x: 95.48, y: 90.22 },
        { x: 93.48, y: 90.22 },
      ],
      color: "#2563eb",
      fillColor: "#60a5fa",
    },
    // The four FLOWERS are bolted to the perimeter wall and their lower ring
    // sits on the TILES (Sec 9.7), so they are obstacles all the way down.
    // Each is ~5.6 in. along the wall and reaches ~4.8 in. into the FIELD,
    // centred on a TILE seam two seams round from a corner.
    {
      id: "biobuzz-flower-far",
      name: "FLOWER - Far Wall",
      vertices: [
        { x: 44.37, y: 141.5 },
        { x: 49.97, y: 141.5 },
        { x: 49.97, y: 136.7 },
        { x: 44.37, y: 136.7 },
      ],
      color: "#b7791f",
      fillColor: "#f6c667",
    },
    {
      id: "biobuzz-flower-blue-wall",
      name: "FLOWER - Blue Wall",
      vertices: [
        { x: 141.5, y: 91.53 },
        { x: 141.5, y: 97.13 },
        { x: 136.7, y: 97.13 },
        { x: 136.7, y: 91.53 },
      ],
      color: "#b7791f",
      fillColor: "#f6c667",
    },
    {
      id: "biobuzz-flower-audience",
      name: "FLOWER - Audience Wall",
      vertices: [
        { x: 91.53, y: 0 },
        { x: 97.13, y: 0 },
        { x: 97.13, y: 4.8 },
        { x: 91.53, y: 4.8 },
      ],
      color: "#b7791f",
      fillColor: "#f6c667",
    },
    {
      id: "biobuzz-flower-red-wall",
      name: "FLOWER - Red Wall",
      vertices: [
        { x: 0, y: 44.37 },
        { x: 0, y: 49.97 },
        { x: 4.8, y: 49.97 },
        { x: 4.8, y: 44.37 },
      ],
      color: "#b7791f",
      fillColor: "#f6c667",
    },
  ],
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
