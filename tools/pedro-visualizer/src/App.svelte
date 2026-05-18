<script lang="ts">
  import type {
    Line,
    BasePoint,
    Settings,
    Point,
    SequenceItem,
    PathChain,
    Shape,
    PoseVariable,
    PathVariable,
    NumberVariable,
    EventTriggerType,
    TimePrediction,
  } from "./types";
  import * as d3 from "d3";
  import {
    snapToGrid,
    gridSize,
    currentFilePath,
    isUnsaved,
    showGrid,
    dualPathMode,
    secondFilePath,
    activePaths,
  } from "./stores";
  import Two from "two.js";
  import type { Path } from "two.js/src/path";
  import type { Line as PathLine } from "two.js/src/shapes/line";
  import ControlTab from "./lib/ControlTab.svelte";
  import Navbar from "./lib/Navbar.svelte";
  import MathTools from "./lib/MathTools.svelte";
  import SaveDialog from "./lib/components/SaveDialog.svelte";
  import DualPathSaveDialog from "./lib/components/DualPathSaveDialog.svelte";
  import ProgressDialog from "./lib/components/ProgressDialog.svelte";
  import _ from "lodash";
  import hotkeys from "hotkeys-js";
  import { createAnimationController } from "./utils/animation";
  import {
    calculateCurveLength,
    calculateMotionProfileDistanceAtTime,
    calculateMotionProfileTime,
    calculateMotionProfileVelocityAtDistance,
    calculatePathTime,
    getAnimationDuration,
    getPathSpeed,
  } from "./utils";
  import { exportAsGif, downloadBlob } from "./utils/gifExporter";

  import {
    calculateRobotState,
    generateGhostPathPoints,
    generateOnionLayers,
  } from "./utils";
  import {
    easeInOutQuad,
    getCurvePoint,
    getRandomColor,
    quadraticToCubic,
    radiansToDegrees,
    shortestRotation,
    downloadTrajectory,
    loadTrajectoryFromFile,
    loadRobotImage,
    updateRobotImageDisplay,
  } from "./utils";
  import {
    POINT_RADIUS,
    LINE_WIDTH,
    DEFAULT_ROBOT_WIDTH,
    DEFAULT_ROBOT_HEIGHT,
    DEFAULT_SETTINGS,
    FIELD_SIZE,
    getDefaultStartPoint,
    getDefaultLines,
    getDefaultShapes,
  } from "./config";
  import { loadSettings, saveSettings } from "./utils/settingsPersistence";
  import * as browserFileStore from "./utils/browserFileStore";
  import { onMount, tick } from "svelte";
  import { debounce } from "lodash";
  import { createHistory, type AppState } from "./utils/history";
  // Browser-only build: file operations use the browser file store and
  // localStorage. Electron-specific APIs have been removed.

  function normalizeEventMarkers(markers: Line["eventMarkers"] = []) {
    return (markers || []).map((marker, index) => {
      const position = Number(marker.position ?? 0.5);
      const durationMs = Number(marker.durationMs ?? 0);
      const triggerMs = Number(marker.triggerMs ?? 0);
      const triggerType: EventTriggerType =
        marker.triggerType === "temporal" || marker.triggerType === "pose"
          ? marker.triggerType
          : "parametric";

      return {
        ...marker,
        id: marker.id || `event-${Math.random().toString(36).slice(2)}`,
        name: (marker.name || "").trim() || `Event ${index + 1}`,
        triggerType,
        position: Number.isFinite(position)
          ? Math.max(0, Math.min(1, position))
          : 0.5,
        triggerMs: Number.isFinite(triggerMs)
          ? Math.max(0, Math.round(triggerMs))
          : 0,
        ...(Number.isFinite(Number(marker.poseX))
          ? { poseX: Number(marker.poseX) }
          : {}),
        ...(Number.isFinite(Number(marker.poseY))
          ? { poseY: Number(marker.poseY) }
          : {}),
        durationMs: Number.isFinite(durationMs)
          ? Math.max(0, Math.round(durationMs))
          : 0,
      };
    });
  }

  function normalizeLines(input: Line[]): Line[] {
    return (input || []).map((line) => ({
      ...line,
      id: line.id || `line-${Math.random().toString(36).slice(2)}`,
      controlPoints: line.controlPoints || [],
      color: line.color || getRandomColor(),
      name: line.name || "",
      speed: Math.max(0.05, Math.min(1, Number(line.speed ?? 1) || 1)),
      waitBeforeMs: Math.max(
        0,
        Number(line.waitBeforeMs ?? line.waitBefore?.durationMs ?? 0),
      ),
      waitAfterMs: Math.max(
        0,
        Number(line.waitAfterMs ?? line.waitAfter?.durationMs ?? 0),
      ),
      waitBeforeName: line.waitBeforeName ?? line.waitBefore?.name ?? "",
      waitAfterName: line.waitAfterName ?? line.waitAfter?.name ?? "",
      eventMarkers: normalizeEventMarkers(line.eventMarkers),
    }));
  }

  function normalizePoseVariables(input: PoseVariable[] | undefined): PoseVariable[] {
    if (!Array.isArray(input)) return [];

    return input.map((variable, index) => ({
      id: variable.id || `pose-${Math.random().toString(36).slice(2)}`,
      name: (variable.name || "").trim() || `Pose ${index + 1}`,
      x: Number.isFinite(Number(variable.x)) ? Number(variable.x) : 0,
      y: Number.isFinite(Number(variable.y)) ? Number(variable.y) : 0,
      heading: Number.isFinite(Number(variable.heading))
        ? Number(variable.heading)
        : 0,
    }));
  }

  function normalizeNumberVariables(input: NumberVariable[] | undefined): NumberVariable[] {
    if (!Array.isArray(input)) return [];

    return input.map((variable, index) => {
      const value = Number(variable.value);
      return {
        id: variable.id || `number-${Math.random().toString(36).slice(2)}`,
        name: (variable.name || "").trim() || `Number ${index + 1}`,
        value: Number.isFinite(value) ? value : 0,
      };
    });
  }

  function numberVariableValue(
    variableId: string | undefined,
    variablesById: Map<string, NumberVariable>,
    fallback: number,
  ): number {
    if (!variableId) return fallback;
    const value = Number(variablesById.get(variableId)?.value);
    return Number.isFinite(value) ? value : fallback;
  }

  function applyNumberVariablesToLines(
    sourceLines: Line[],
    sourceNumberVariables: NumberVariable[],
  ): Line[] {
    const variablesById = new Map(
      sourceNumberVariables.map((variable) => [variable.id, variable]),
    );

    return sourceLines.map((line) => ({
      ...line,
      speed: Math.max(
        0.05,
        Math.min(
          1,
          numberVariableValue(
            line.speedVariableId,
            variablesById,
            Number(line.speed ?? 1) || 1,
          ),
        ),
      ),
      eventMarkers: (line.eventMarkers || []).map((marker) => ({
        ...marker,
        position: Math.max(
          0,
          Math.min(
            1,
            numberVariableValue(
              marker.positionVariableId,
              variablesById,
              Number(marker.position ?? 0.5) || 0.5,
            ) > 1
              ? numberVariableValue(
                  marker.positionVariableId,
                  variablesById,
                  Number(marker.position ?? 0.5) || 0.5,
                ) / 100
              : numberVariableValue(
                  marker.positionVariableId,
                  variablesById,
                  Number(marker.position ?? 0.5) || 0.5,
                ),
          ),
        ),
        triggerMs: Math.max(
          0,
          Math.round(
            numberVariableValue(
              marker.triggerMsVariableId,
              variablesById,
              Number(marker.triggerMs ?? 0) || 0,
            ),
          ),
        ),
        poseX: numberVariableValue(
          marker.poseXVariableId,
          variablesById,
          Number(marker.poseX ?? line.endPoint?.x ?? 0) || 0,
        ),
        poseY: numberVariableValue(
          marker.poseYVariableId,
          variablesById,
          Number(marker.poseY ?? line.endPoint?.y ?? 0) || 0,
        ),
        durationMs: Math.max(
          0,
          Math.round(
            numberVariableValue(
              marker.durationVariableId,
              variablesById,
              Number(marker.durationMs ?? 0) || 0,
            ),
          ),
        ),
      })),
    }));
  }

  function applyNumberVariablesToSequence(
    sourceSequence: SequenceItem[],
    sourceNumberVariables: NumberVariable[],
  ): SequenceItem[] {
    const variablesById = new Map(
      sourceNumberVariables.map((variable) => [variable.id, variable]),
    );

    return sourceSequence.map((item) => {
      if (item.kind === "repeat") {
        return {
          ...item,
          count: Math.max(
            1,
            Math.min(
              20,
              Math.round(
                numberVariableValue(item.countVariableId, variablesById, item.count),
              ),
            ),
          ),
        };
      }

      if (item.kind === "wait" || item.kind === "event") {
        return {
          ...item,
          durationMs: Math.max(
            0,
            Math.round(
              numberVariableValue(
                item.durationVariableId,
                variablesById,
                item.durationMs,
              ),
            ),
          ),
        };
      }

      return item;
    });
  }

  function normalizePathVariables(
    input: PathVariable[] | undefined,
    sourcePoseVariables: PoseVariable[] = [],
    sourceNumberVariables: NumberVariable[] = [],
  ): PathVariable[] {
    if (!Array.isArray(input)) return [];

    return input.map((variable, index) => {
      const normalizedLines = applyNumberVariablesToLines(
        normalizeLines(variable.lines || []),
        sourceNumberVariables,
      );
      const normalizedPath = applyPoseVariablesToPath(
        variable.startPoint || getDefaultStartPoint(),
        normalizedLines,
        sourcePoseVariables,
      );

      return {
        id: variable.id || `path-variable-${Math.random().toString(36).slice(2)}`,
        name: (variable.name || "").trim() || `Path Variable ${index + 1}`,
        startPoint: normalizedPath.startPoint,
        lines: normalizedPath.lines,
      };
    });
  }

  function bindPointToPoseVariable(point: Point, variable: PoseVariable): Point {
    const targetHeading = Number.isFinite(Number(variable.heading))
      ? Number(variable.heading)
      : 0;
    const basePoint = {
      x: Number(variable.x) || 0,
      y: Number(variable.y) || 0,
      locked: point.locked,
      poseVariableId: variable.id,
    };

    if (point.heading === "linear") {
      return {
        ...basePoint,
        heading: "linear",
        startDeg: Number.isFinite(Number(point.startDeg))
          ? Number(point.startDeg)
          : targetHeading,
        endDeg: targetHeading,
        headingCurve: point.headingCurve ?? 1,
      };
    }

    if (point.heading === "tangential") {
      return {
        ...basePoint,
        heading: "tangential",
        reverse: point.reverse ?? false,
      };
    }

    return {
      ...basePoint,
      heading: "constant",
      degrees: targetHeading,
    };
  }

  function clearMissingPoseVariable(point: Point, variablesById: Map<string, PoseVariable>): Point {
    if (!point.poseVariableId) return point;

    const variable = variablesById.get(point.poseVariableId);
    if (variable) {
      return bindPointToPoseVariable(point, variable);
    }

    const { poseVariableId, ...nextPoint } = point;
    return nextPoint as Point;
  }

  function applyPoseVariablesToPath(
    sourceStartPoint: Point,
    sourceLines: Line[],
    sourcePoseVariables: PoseVariable[],
  ): { startPoint: Point; lines: Line[] } {
    const variablesById = new Map(
      sourcePoseVariables.map((variable) => [variable.id, variable]),
    );

    return {
      startPoint: clearMissingPoseVariable(sourceStartPoint, variablesById),
      lines: sourceLines.map((line) => ({
        ...line,
        endPoint: clearMissingPoseVariable(line.endPoint, variablesById),
      })),
    };
  }

  // Canvas state
  let two: Two;
  let twoElement: HTMLDivElement;
  let width = 0;
  let height = 0;
  // Robot state
  $: robotWidth = settings?.rWidth || DEFAULT_ROBOT_WIDTH;
  $: robotHeight = settings?.rHeight || DEFAULT_ROBOT_HEIGHT;
  let robotXY: BasePoint = { x: 0, y: 0 };
  let robotHeading: number = 0;
  let additionalRobotStates: Array<{ xy: BasePoint; heading: number }> = [];
  // Animation state
  let percent: number = 0;
  let playing = false;
  let animationFrame: number;
  let startTime: number | null = null;
  let previousTime: number | null = null;
  // Save dialog state
  let showSaveDialog = false;
  let showDualPathSaveDialog = false;
  let isSaving = false;
  // GIF export state
  let exportingGif = false;
  let gifExportProgress = 0;
  let gifExportStatus = "Preparing...";
  let cancelGifExport = false;
  // Path data
  let settings: Settings = { ...DEFAULT_SETTINGS };
  let startPoint: Point = getDefaultStartPoint();
  let lines: Line[] = normalizeLines(getDefaultLines());
  let poseVariables: PoseVariable[] = [];
  let pathVariables: PathVariable[] = [];
  let numberVariables: NumberVariable[] = [];

  function normalizeLegacyFieldMap(input: Settings): Settings {
    const next = { ...input };

    if (typeof next.fieldMap === "string" && next.fieldMap.startsWith("custom||")) {
      const [, embeddedImage = ""] = next.fieldMap.split("||");
      next.fieldMap = "custom";
      if (embeddedImage && !next.customFieldImage) {
        next.customFieldImage = embeddedImage;
      }
    }

    if (!next.fieldMap) {
      next.fieldMap = DEFAULT_SETTINGS.fieldMap;
    }

    return next;
  }

  $: fieldMapSrc =
    settings.fieldMap === "custom"
      ? settings.customFieldImage || "/fields/decode.webp"
      : settings.fieldMap
        ? `/fields/${settings.fieldMap}`
        : "/fields/decode.webp";
  let sequence: SequenceItem[] = lines.map((ln) => ({
    kind: "path",
    lineId: ln.id!,
  }));
  const makeChainId = () =>
    `chain-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  const defaultPathChainName = "Main Chain";
  const createDefaultPathChain = (sourceLines: Line[]): PathChain => ({
    id: makeChainId(),
    name: defaultPathChainName,
    color: getRandomColor(),
    lineIds: sourceLines.map((ln) => ln.id!).filter(Boolean),
  });
  let pathChains: PathChain[] = [createDefaultPathChain(lines)];
  let shapes: Shape[] = getDefaultShapes();
  let optimizingLineIds: Record<string, boolean> = {};
  let optimizingAll = false;

  // Second path data (for alliance coordination) - DEPRECATED, use additionalPaths
  let secondStartPoint: Point | null = null;
  let secondLines: Line[] = [];
  let secondSequence: SequenceItem[] = [];
  let secondShapes: Shape[] = [];

  // Multiple paths data (new system - supports up to 4 paths total)
  interface AdditionalPathData {
    filePath: string;
    startPoint: Point | null;
    lines: Line[];
    sequence: SequenceItem[];
    shapes: Shape[];
    settings: Settings;
    pathChains?: PathChain[];
    poseVariables?: PoseVariable[];
    pathVariables?: PathVariable[];
    numberVariables?: NumberVariable[];
    color?: string; // Optional custom color for this path
  }
  let additionalPaths: AdditionalPathData[] = [];

  type CanvasPathElement = Path | PathLine;
  type PathOverlayItem = {
    id: string;
    x: number;
    y: number;
    label: string;
    color?: string;
  };
  type SwerveWheelOverlay = {
    id: string;
    x: number;
    y: number;
    angle: number;
    length: number;
    thickness: number;
    opacity: number;
    color: string;
  };
  type OnionLayerState = {
    x: number;
    y: number;
    heading: number;
    corners: BasePoint[];
    lineIndex: number;
    t?: number;
  };
  type CurveSample = {
    point: BasePoint;
    distance: number;
    t: number;
  };

  const PATH_RENDER_SAMPLES = 100;
  const AUTO_COUNTDOWN_SECONDS = 30;
  const SWERVE_X_LOCK_LOCAL_CSS_ANGLES = [-45, 45, -135, 135];

  function clampNumber(value: number, min: number, max: number): number {
    if (!Number.isFinite(value)) return min;
    return Math.max(min, Math.min(max, value));
  }

  function setImageFallback(e: Event, fallbackSrc: string) {
    (e.currentTarget as HTMLImageElement).src = fallbackSrc;
  }

  function formatSeconds(value: number): string {
    const absolute = Math.abs(value);
    if (absolute >= 60) {
      const minutes = Math.floor(absolute / 60);
      const seconds = absolute % 60;
      return `${minutes}:${seconds.toFixed(1).padStart(4, "0")}`;
    }
    return `${absolute.toFixed(1)}s`;
  }

  function formatCountdown(value: number): string {
    return value < 0 ? `+${formatSeconds(value)}` : formatSeconds(value);
  }

  function getCountdownColor(remainingSeconds: number): string {
    if (remainingSeconds <= 3) return "#dc2626";
    if (remainingSeconds <= 10) return "#f59e0b";
    return "#16a34a";
  }

  function getLineStartPoint(
    sourceStartPoint: Point,
    sourceLines: Line[],
    lineIndex: number,
  ): BasePoint | null {
    if (lineIndex === 0) return sourceStartPoint;
    return sourceLines[lineIndex - 1]?.endPoint || null;
  }

  function sampleLineCurve(
    sourceStartPoint: BasePoint,
    line: Line,
    samples = PATH_RENDER_SAMPLES,
  ): CurveSample[] {
    const sampleCount = Math.max(1, Math.round(samples));
    const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
    const sampledPoints: CurveSample[] = [
      {
        point: { x: sourceStartPoint.x, y: sourceStartPoint.y },
        distance: 0,
        t: 0,
      },
    ];
    let distance = 0;
    let previousPoint = sourceStartPoint;

    for (let i = 1; i <= sampleCount; i++) {
      const t = i / sampleCount;
      const point = getCurvePoint(t, curvePoints);
      const dx = point.x - previousPoint.x;
      const dy = point.y - previousPoint.y;
      distance += Math.sqrt(dx * dx + dy * dy);
      sampledPoints.push({
        point: { x: point.x, y: point.y },
        distance,
        t,
      });
      previousPoint = point;
    }

    return sampledPoints;
  }

  function getLinePointAtT(
    sourceStartPoint: BasePoint,
    line: Line,
    position: number,
  ): BasePoint {
    const t = clampNumber(position, 0, 1);
    return getCurvePoint(t, [sourceStartPoint, ...line.controlPoints, line.endPoint]);
  }

  function getLinePointAtDistance(
    sourceStartPoint: BasePoint,
    line: Line,
    targetDistance: number,
  ): BasePoint {
    const samples = sampleLineCurve(sourceStartPoint, line);
    const totalDistance = samples[samples.length - 1]?.distance || 0;
    const clampedDistance = clampNumber(targetDistance, 0, totalDistance);

    for (let i = 1; i < samples.length; i++) {
      const previous = samples[i - 1];
      const current = samples[i];
      if (current.distance < clampedDistance) continue;

      const segmentDistance = current.distance - previous.distance;
      const ratio =
        segmentDistance <= 0
          ? 0
          : (clampedDistance - previous.distance) / segmentDistance;

      return {
        x: previous.point.x + (current.point.x - previous.point.x) * ratio,
        y: previous.point.y + (current.point.y - previous.point.y) * ratio,
      };
    }

    return samples[samples.length - 1]?.point || sourceStartPoint;
  }

  function getLineMotionValues(line: Line, routeSettings: Settings) {
    const pathSpeed = getPathSpeed(line);
    return {
      maxVelocity: Math.max(0, Number(routeSettings.maxVelocity) || 0) * pathSpeed,
      maxAcceleration:
        Math.max(0, Number(routeSettings.maxAcceleration) || 0) * pathSpeed,
      maxDeceleration:
        Math.max(
          0,
          Number(routeSettings.maxDeceleration ?? routeSettings.maxAcceleration) || 0,
        ) * pathSpeed,
      referenceVelocity: Math.max(0, Number(routeSettings.maxVelocity) || 0),
    };
  }

  function velocityToColor(ratio: number): string {
    const t = clampNumber(ratio, 0, 1);
    const blue = [37, 99, 235];
    const red = [239, 68, 68];
    const rgb = blue.map((channel, index) =>
      Math.round(channel + (red[index] - channel) * t),
    );
    return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
  }

  function createStandardPathElement(
    line: Line,
    sourceStartPoint: BasePoint,
  ): CanvasPathElement {
    let lineElem: CanvasPathElement;

    if (line.controlPoints.length > 2) {
      const cps = [sourceStartPoint, ...line.controlPoints, line.endPoint];
      let points = [
        new Two.Anchor(
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          0,
          0,
          0,
          0,
          Two.Commands.move,
        ),
      ];
      for (let i = 1; i <= PATH_RENDER_SAMPLES; ++i) {
        const point = getCurvePoint(i / PATH_RENDER_SAMPLES, cps);
        points.push(
          new Two.Anchor(
            x(point.x),
            y(point.y),
            0,
            0,
            0,
            0,
            Two.Commands.line,
          ),
        );
      }
      points.forEach((point) => (point.relative = false));
      lineElem = new Two.Path(points);
      lineElem.automatic = false;
    } else if (line.controlPoints.length > 0) {
      let cp1 = line.controlPoints[1]
        ? line.controlPoints[0]
        : quadraticToCubic(sourceStartPoint, line.controlPoints[0], line.endPoint)
            .Q1;
      let cp2 =
        line.controlPoints[1] ??
        quadraticToCubic(sourceStartPoint, line.controlPoints[0], line.endPoint)
          .Q2;
      let points = [
        new Two.Anchor(
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          x(sourceStartPoint.x),
          y(sourceStartPoint.y),
          x(cp1.x),
          y(cp1.y),
          Two.Commands.move,
        ),
        new Two.Anchor(
          x(line.endPoint.x),
          y(line.endPoint.y),
          x(cp2.x),
          y(cp2.y),
          x(line.endPoint.x),
          y(line.endPoint.y),
          Two.Commands.curve,
        ),
      ];
      points.forEach((point) => (point.relative = false));

      lineElem = new Two.Path(points);
      lineElem.automatic = false;
    } else {
      lineElem = new Two.Line(
        x(sourceStartPoint.x),
        y(sourceStartPoint.y),
        x(line.endPoint.x),
        y(line.endPoint.y),
      );
    }

    return lineElem;
  }

  function stylePathElement(
    element: CanvasPathElement,
    id: string,
    stroke: string,
    opacity: number,
    locked?: boolean,
  ) {
    element.id = id;
    element.stroke = stroke;
    element.linewidth = x(LINE_WIDTH);
    element.noFill();
    if (locked) {
      element.dashes = [x(2), x(2)];
      element.opacity = opacity * 0.7;
    } else {
      element.dashes = [];
      element.opacity = opacity;
    }
  }

  function createVelocityGradientPathElements(
    line: Line,
    sourceStartPoint: BasePoint,
    idBase: string,
    opacity: number,
    routeSettings: Settings,
  ): CanvasPathElement[] {
    const samples = sampleLineCurve(sourceStartPoint, line);
    const totalLength = samples[samples.length - 1]?.distance || 0;
    const motion = getLineMotionValues(line, routeSettings);

    if (
      totalLength <= 0 ||
      motion.maxVelocity <= 0 ||
      motion.maxAcceleration <= 0 ||
      motion.maxDeceleration <= 0 ||
      motion.referenceVelocity <= 0
    ) {
      return [];
    }

    const elements: CanvasPathElement[] = [];

    for (let i = 1; i < samples.length; i++) {
      const previous = samples[i - 1];
      const current = samples[i];
      const midpointDistance = (previous.distance + current.distance) / 2;
      const velocity = calculateMotionProfileVelocityAtDistance(
        midpointDistance,
        totalLength,
        motion.maxVelocity,
        motion.maxAcceleration,
        motion.maxDeceleration,
      );
      const color = velocityToColor(velocity / motion.referenceVelocity);
      const segment = new Two.Line(
        x(previous.point.x),
        y(previous.point.y),
        x(current.point.x),
        y(current.point.y),
      );
      stylePathElement(segment, `${idBase}-velocity-${i}`, color, opacity, line.locked);
      elements.push(segment);
    }

    return elements;
  }

  function createPathDrawElements(
    line: Line,
    sourceStartPoint: BasePoint,
    idBase: string,
    stroke: string,
    opacity: number,
    routeSettings: Settings,
  ): CanvasPathElement[] {
    if (routeSettings.showVelocityGradient) {
      const gradientElements = createVelocityGradientPathElements(
        line,
        sourceStartPoint,
        idBase,
        opacity,
        routeSettings,
      );
      if (gradientElements.length > 0) return gradientElements;
    }

    const lineElem = createStandardPathElement(line, sourceStartPoint);
    stylePathElement(lineElem, idBase, stroke, opacity, line.locked);
    return [lineElem];
  }

  function getLineDurationMap(timePrediction: TimePrediction | null | undefined) {
    const durationMap = new Map<number, number>();
    timePrediction?.timeline?.forEach((event) => {
      if (
        event.type === "travel" &&
        event.lineIndex !== undefined &&
        Number.isFinite(event.duration)
      ) {
        durationMap.set(event.lineIndex, event.duration);
      }
    });
    return durationMap;
  }

  function getLineLength(sourceStartPoint: BasePoint, line: Line): number {
    return calculateCurveLength(sourceStartPoint, line.controlPoints, line.endPoint);
  }

  function getFallbackLineDuration(
    sourceStartPoint: BasePoint,
    line: Line,
    routeSettings: Settings,
  ): number {
    const length = getLineLength(sourceStartPoint, line);
    const motion = getLineMotionValues(line, routeSettings);

    if (
      motion.maxVelocity > 0 &&
      motion.maxAcceleration > 0 &&
      motion.maxDeceleration > 0
    ) {
      return calculateMotionProfileTime(
        length,
        motion.maxVelocity,
        motion.maxAcceleration,
        motion.maxDeceleration,
      );
    }

    const averageVelocity =
      ((routeSettings.xVelocity + routeSettings.yVelocity) / 2) * getPathSpeed(line);
    return averageVelocity > 0 ? length / averageVelocity : 0;
  }

  function buildPathAnnotations(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
  ): PathOverlayItem[] {
    const durationMap = getLineDurationMap(timePrediction);

    return sourceLines
      .map((line, lineIndex) => {
        const lineStartPoint = getLineStartPoint(sourceStartPoint, sourceLines, lineIndex);
        if (!lineStartPoint || !line?.endPoint) return null;

        const length = getLineLength(lineStartPoint, line);
        const duration =
          durationMap.get(lineIndex) ??
          getFallbackLineDuration(lineStartPoint, line, routeSettings);
        const midpoint = getLinePointAtDistance(lineStartPoint, line, length / 2);

        return {
          id: `${idPrefix}-path-label-${lineIndex}`,
          x: x(midpoint.x),
          y: y(midpoint.y),
          label: `${length.toFixed(1)} in / ${formatSeconds(duration)}`,
          color: line.color,
        };
      })
      .filter(Boolean) as PathOverlayItem[];
  }

  function buildEventPins(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
  ): PathOverlayItem[] {
    const durationMap = getLineDurationMap(timePrediction);
    const pins: PathOverlayItem[] = [];

    sourceLines.forEach((line, lineIndex) => {
      const lineStartPoint = getLineStartPoint(sourceStartPoint, sourceLines, lineIndex);
      if (!lineStartPoint || !line?.eventMarkers?.length) return;

      const length = getLineLength(lineStartPoint, line);
      const duration =
        durationMap.get(lineIndex) ??
        getFallbackLineDuration(lineStartPoint, line, routeSettings);
      const motion = getLineMotionValues(line, routeSettings);

      line.eventMarkers.forEach((marker, markerIndex) => {
        const triggerType =
          marker.triggerType === "temporal" || marker.triggerType === "pose"
            ? marker.triggerType
            : "parametric";
        const position = clampNumber(Number(marker.position ?? 0.5), 0, 1);
        let fieldPoint: BasePoint;

        if (
          triggerType === "pose" &&
          Number.isFinite(Number(marker.poseX)) &&
          Number.isFinite(Number(marker.poseY))
        ) {
          fieldPoint = {
            x: Number(marker.poseX),
            y: Number(marker.poseY),
          };
        } else if (
          triggerType === "temporal" &&
          duration > 0 &&
          motion.maxVelocity > 0 &&
          motion.maxAcceleration > 0 &&
          motion.maxDeceleration > 0
        ) {
          const triggerSeconds = Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000;
          const triggerDistance = calculateMotionProfileDistanceAtTime(
            Math.min(triggerSeconds, duration),
            length,
            motion.maxVelocity,
            motion.maxAcceleration,
            motion.maxDeceleration,
          );
          fieldPoint = getLinePointAtDistance(lineStartPoint, line, triggerDistance);
        } else {
          fieldPoint = getLinePointAtT(lineStartPoint, line, position);
        }

        pins.push({
          id: `${idPrefix}-event-pin-${lineIndex}-${marker.id || markerIndex}`,
          x: x(fieldPoint.x),
          y: y(fieldPoint.y),
          label: marker.name || `Event ${markerIndex + 1}`,
          color: "#a855f7",
        });
      });
    });

    return pins;
  }

  function tangentCssAngleForLine(
    sourceStartPoint: BasePoint,
    line: Line,
    position: number,
  ): number {
    const t = clampNumber(position, 0, 1);
    const beforeT = clampNumber(t - 0.01, 0, 1);
    const afterT = clampNumber(t + 0.01, 0, 1);
    const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
    const before = getCurvePoint(beforeT, curvePoints);
    const after = getCurvePoint(afterT, curvePoints);
    const dx = after.x - before.x;
    const dy = after.y - before.y;

    if (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9) {
      return 0;
    }

    return (Math.atan2(-dy, dx) * 180) / Math.PI;
  }

  function getTravelProgressForSwerve(
    playbackPercent: number,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
  ): { lineIndex: number; t: number; angle: number } | null {
    if (!timePrediction?.timeline?.length || timePrediction.totalTime <= 0) {
      return null;
    }

    const currentTime = (clampNumber(playbackPercent, 0, 100) / 100) * timePrediction.totalTime;
    const activeEvent =
      timePrediction.timeline.find(
        (event) => currentTime >= event.startTime && currentTime <= event.endTime,
      ) || timePrediction.timeline[timePrediction.timeline.length - 1];

    if (activeEvent.type !== "travel" || activeEvent.lineIndex === undefined) {
      return null;
    }

    const lineIndex = activeEvent.lineIndex;
    const line = sourceLines[lineIndex];
    const lineStart = getLineStartPoint(sourceStartPoint, sourceLines, lineIndex);

    if (!line || !lineStart || activeEvent.duration <= 0) {
      return null;
    }

    const timeIntoEvent = clampNumber(
      currentTime - activeEvent.startTime,
      0,
      activeEvent.duration,
    );
    const lineLength = getLineLength(lineStart, line);
    let lineT = 0;

    const motion = getLineMotionValues(line, routeSettings);
    if (
      lineLength > 0 &&
      motion.maxVelocity > 0 &&
      motion.maxAcceleration > 0 &&
      motion.maxDeceleration > 0
    ) {
      const distance = calculateMotionProfileDistanceAtTime(
        timeIntoEvent,
        lineLength,
        motion.maxVelocity,
        motion.maxAcceleration,
        motion.maxDeceleration,
      );
      lineT = clampNumber(distance / lineLength, 0, 1);
    } else {
      lineT = easeInOutQuad(clampNumber(timeIntoEvent / activeEvent.duration, 0, 1));
    }

    return {
      lineIndex,
      t: lineT,
      angle: tangentCssAngleForLine(lineStart, line, lineT),
    };
  }

  function getSwerveWheelAnglesCss(
    playbackPercent: number,
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
    robotHeadingCss: number,
  ): number[] {
    const travelState = getTravelProgressForSwerve(
      playbackPercent,
      sourceStartPoint,
      sourceLines,
      timePrediction,
      routeSettings,
    );

    if (travelState) {
      return [travelState.angle, travelState.angle, travelState.angle, travelState.angle];
    }

    return SWERVE_X_LOCK_LOCAL_CSS_ANGLES.map(
      (angle) => robotHeadingCss + angle,
    );
  }

  function buildSwerveWheelModules(
    idPrefix: string,
    center: BasePoint,
    robotHeadingCss: number,
    wheelAnglesCss: number[],
    opacity = 1,
    color = "#06b6d4",
    scale = 1,
  ): SwerveWheelOverlay[] {
    const robotPixelWidth = Math.max(1, x(robotWidth));
    const robotPixelHeight = Math.max(1, x(robotHeight));
    const halfForward = robotPixelWidth * 0.34 * scale;
    const halfSide = robotPixelHeight * 0.34 * scale;
    const wheelLength = clampNumber(
      Math.min(robotPixelWidth, robotPixelHeight) * 0.28 * scale,
      6,
      24,
    );
    const wheelThickness = Math.max(3, wheelLength * 0.36);
    const headingRad = (robotHeadingCss * Math.PI) / 180;
    const cos = Math.cos(headingRad);
    const sin = Math.sin(headingRad);
    const localOffsets = [
      { x: halfForward, y: -halfSide },
      { x: halfForward, y: halfSide },
      { x: -halfForward, y: -halfSide },
      { x: -halfForward, y: halfSide },
    ];

    return localOffsets.map((offset, index) => ({
      id: `${idPrefix}-swerve-module-${index}`,
      x: center.x + offset.x * cos - offset.y * sin,
      y: center.y + offset.x * sin + offset.y * cos,
      angle: wheelAnglesCss[index] ?? robotHeadingCss,
      length: wheelLength,
      thickness: wheelThickness,
      opacity,
      color,
    }));
  }

  function getVisibleOnionLayers(
    sourceStartPoint: Point,
    sourceLines: Line[],
    timePrediction: TimePrediction | null | undefined,
    routeSettings: Settings,
  ): OnionLayerState[] {
    let layers = generateOnionLayers(
      sourceStartPoint,
      sourceLines,
      routeSettings.rWidth,
      routeSettings.rHeight,
      routeSettings.onionLayerSpacing || 6,
    ) as OnionLayerState[];

    if (
      routeSettings.onionNextPointOnly &&
      timePrediction?.timeline?.length
    ) {
      const currentTime = (timePrediction.totalTime || 0) * (percent / 100);
      const travelEvents = timePrediction.timeline.filter(
        (event) => event.type === "travel",
      );
      let selectedLineIndex: number | null = null;
      const currentTravel = travelEvents.find(
        (event) => event.startTime <= currentTime && event.endTime >= currentTime,
      );

      if (currentTravel) {
        selectedLineIndex = currentTravel.lineIndex as number;
      } else {
        const nextTravel = travelEvents.find(
          (event) => event.startTime > currentTime,
        );
        if (nextTravel) {
          selectedLineIndex = nextTravel.lineIndex as number;
        } else if (travelEvents.length) {
          selectedLineIndex = travelEvents[travelEvents.length - 1].lineIndex as number;
        }
      }

      if (selectedLineIndex !== null) {
        layers = layers.filter((layer) => layer.lineIndex === selectedLineIndex);
      }
    }

    return layers;
  }

  function buildOnionSwerveModules(
    idPrefix: string,
    sourceStartPoint: Point,
    sourceLines: Line[],
    layers: OnionLayerState[],
    opacity: number,
    color: string,
  ): SwerveWheelOverlay[] {
    return layers.flatMap((layer, layerIndex) => {
      const line = sourceLines[layer.lineIndex];
      const lineStart = getLineStartPoint(sourceStartPoint, sourceLines, layer.lineIndex);
      if (!line || !lineStart) return [];

      const wheelAngle = tangentCssAngleForLine(lineStart, line, layer.t ?? 0.5);
      return buildSwerveWheelModules(
        `${idPrefix}-onion-${layerIndex}`,
        { x: x(layer.x), y: y(layer.y) },
        layer.heading,
        [wheelAngle, wheelAngle, wheelAngle, wheelAngle],
        opacity,
        color,
        0.65,
      );
    });
  }

  const history = createHistory();
  const { canUndoStore, canRedoStore } = history;
  const OPTIMIZER_BASE_URL = "https://fpa.pedropathing.com";

  function getAppState(): AppState {
    return {
      startPoint,
      lines,
      shapes,
      sequence,
      settings,
      pathChains,
      poseVariables,
      pathVariables,
      numberVariables,
    };
  }

  // Use the stores for reactivity
  $: canUndo = $canUndoStore;
  $: canRedo = $canRedoStore;

  function recordChange() {
    history.record(getAppState());
  }

  function undoAction() {
    const prev = history.undo();
    if (prev) {
      startPoint = prev.startPoint;
      lines = prev.lines;
      shapes = prev.shapes;
      sequence = prev.sequence;
      settings = prev.settings;
      pathChains = prev.pathChains;
      poseVariables = prev.poseVariables || [];
      pathVariables = prev.pathVariables || [];
      numberVariables = prev.numberVariables || [];
      isUnsaved.set(true);
      two && two.update();
    }

    // undoAction completes; no file-picker behavior here
  }

  function redoAction() {
    const next = history.redo();
    if (next) {
      startPoint = next.startPoint;
      lines = next.lines;
      shapes = next.shapes;
      sequence = next.sequence;
      settings = next.settings;
      pathChains = next.pathChains;
      poseVariables = next.poseVariables || [];
      pathVariables = next.pathVariables || [];
      numberVariables = next.numberVariables || [];
      isUnsaved.set(true);
      two && two.update();
    }
  }

  function normalizePathChains(
    sourceChains: PathChain[] | undefined,
    sourceLines: Line[],
  ): PathChain[] {
    const validLineIds = new Set(sourceLines.map((ln) => ln.id!).filter(Boolean));
    const cleaned = (sourceChains || [])
      .map((chain) => ({
        ...chain,
        id: chain.id || makeChainId(),
        name: (chain.name || "").trim() || defaultPathChainName,
        color: chain.color || getRandomColor(),
        lineIds: (chain.lineIds || []).filter((id) => validLineIds.has(id)),
      }));

    if (cleaned.length === 0) {
      return [createDefaultPathChain(sourceLines)];
    }

    return cleaned;
  }

  $: {
    const normalized = normalizePathChains(pathChains, lines);
    const current = JSON.stringify(pathChains);
    const next = JSON.stringify(normalized);
    if (current !== next) {
      pathChains = normalized;
    }
  }

  $: {
    // Ensure arrays are reactive when items are added/removed
    lines = lines;
    shapes = shapes;
  }

  // Two.js groups
  let lineGroup = new Two.Group();
  lineGroup.id = "line-group";
  let pointGroup = new Two.Group();
  pointGroup.id = "point-group";
  let shapeGroup = new Two.Group();
  shapeGroup.id = "shape-group";
  // Coordinate converters
  let x: d3.ScaleLinear<number, number, number>;
  let y: d3.ScaleLinear<number, number, number>;

  // Animation controller
  let loopAnimation = true;
  let animationController: ReturnType<typeof createAnimationController>;
  $: timePrediction = calculatePathTime(startPoint, lines, settings, sequence);
  $: animationDuration = getAnimationDuration(timePrediction.totalTime / 1000);
  
  // Second path timeline (for dual path mode)
  $: secondTimePrediction = $dualPathMode && secondStartPoint && secondLines.length > 0 
    ? calculatePathTime(secondStartPoint, secondLines, settings, secondSequence)
    : null;
  
  // Calculate max duration across all paths for playbar scaling
  $: effectiveAnimationDuration = (() => {
    // In multi-path mode, only use additional paths for duration
    if ($activePaths.length > 0) {
      let maxTime = 0;
      additionalPaths.forEach((pathData) => {
        if (pathData.startPoint && pathData.lines.length > 0) {
          const pathTime = calculatePathTime(
            pathData.startPoint,
            pathData.lines,
            pathData.settings,
            pathData.sequence
          );
          if (pathTime) {
            maxTime = Math.max(maxTime, pathTime.totalTime);
          }
        }
      });
      return maxTime > 0 ? getAnimationDuration(maxTime / 1000) : animationDuration;
    }
    
    // In normal/dual mode, check main path and second path
    let maxTime = timePrediction.totalTime;
    
    if ($dualPathMode && secondTimePrediction) {
      maxTime = Math.max(maxTime, secondTimePrediction.totalTime);
    }
    
    return getAnimationDuration(maxTime / 1000);
  })();
  
  // Load additional paths when activePaths changes
  $: {
    loadAdditionalPaths($activePaths);
  }

  async function loadAdditionalPaths(paths: string[]) {
    const newAdditionalPaths: AdditionalPathData[] = [];
    
    // Multi-path mode is isolated - turn off old dual path mode
    if (paths.length > 0) {
      dualPathMode.set(false);
      secondStartPoint = null;
      secondLines = [];
      secondShapes = [];
      secondSequence = [];
      secondFilePath.set(null);
    }
    
    const colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']; // Red, Teal, Blue, Salmon

    for (let i = 0; i < Math.min(paths.length, 4); i++) {
      const filePath = paths[i];
      try {
        const content = await browserFileStore.readFile(filePath);
          const data = JSON.parse(content);

        if (data.startPoint && data.lines) {
          const loadedPoseVariables = normalizePoseVariables(data.poseVariables);
          const loadedNumberVariables = normalizeNumberVariables(data.numberVariables);
          const normalizedLines = applyNumberVariablesToLines(
            normalizeLines(data.lines || []),
            loadedNumberVariables,
          );
          const normalizedPath = applyPoseVariablesToPath(
            data.startPoint,
            normalizedLines,
            loadedPoseVariables,
          );
          const normalizedSequence = applyNumberVariablesToSequence(
            data.sequence || normalizedPath.lines.map((ln: Line) => ({
              kind: "path",
              lineId: ln.id!,
            })),
            loadedNumberVariables,
          );
          newAdditionalPaths.push({
            filePath,
            startPoint: normalizedPath.startPoint,
            lines: normalizedPath.lines,
            shapes: data.shapes || [],
            sequence: normalizedSequence,
            settings: data.settings || { ...DEFAULT_SETTINGS },
            pathChains: normalizePathChains(data.pathChains, normalizedPath.lines),
            poseVariables: loadedPoseVariables,
            pathVariables: normalizePathVariables(
              data.pathVariables,
              loadedPoseVariables,
              loadedNumberVariables,
            ),
            numberVariables: loadedNumberVariables,
            color: colors[i],
          });
        }
      } catch (error) {
        console.error(`Failed to load additional path ${filePath}:`, error);
      }
    }

    additionalPaths = newAdditionalPaths;
  }
  
  let secondRobotXY: BasePoint = { x: 0, y: 0 };
  let secondRobotHeading: number = 0;
  /**
   * Converter for X axis from inches to pixels.
   */
  $: x = d3
    .scaleLinear()
    .domain([0, FIELD_SIZE])
    .range([0, width || FIELD_SIZE]);
  /**
   * Converter for Y axis from inches to pixels.
   */
  $: y = d3
    .scaleLinear()
    .domain([0, FIELD_SIZE])
    .range([height || FIELD_SIZE, 0]);
  $: {
    // Calculate robot state using the Timeline
    if (timePrediction && timePrediction.timeline && lines.length > 0) {
      const state = calculateRobotState(
        percent,
        timePrediction.timeline,
        lines,
        startPoint,
        settings,
        x,
        y,
      );
      robotXY = { x: state.x, y: state.y };
      robotHeading = state.heading;
    } else {
      // Fallback for initialization
      robotXY = { x: x(startPoint.x), y: y(startPoint.y) };
      robotHeading = 0;
    }
  }

  $: points = (() => {
    let _points = [];
    
    // Only show main path points when NOT in multi-path mode
    if ($activePaths.length === 0) {
      let startPointElem = new Two.Circle(
        x(startPoint.x),
        y(startPoint.y),
        x(POINT_RADIUS),
      );
      startPointElem.id = `point-0-0`;
      startPointElem.fill = lines[0].color;
      startPointElem.noStroke();

      _points.push(startPointElem);

      lines.forEach((line, idx) => {
        if (!line || !line.endPoint) return; // Skip invalid lines or lines without endPoint
        [line.endPoint, ...line.controlPoints].forEach((point, idx1) => {
          if (idx1 > 0) {
            let pointGroup = new Two.Group();
            pointGroup.id = `point-${idx + 1}-${idx1}`;

            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `point-${idx + 1}-${idx1}-background`;
            pointElem.fill = line.color;
            pointElem.noStroke();

            let pointText = new Two.Text(
              `${idx1}`,
              x(point.x),
              y(point.y - 0.15),
            );
            pointText.id = `point-${idx + 1}-${idx1}-text`;
            pointText.size = x(1.55);
            pointText.leading = 1;
            pointText.family = "ui-sans-serif, system-ui, sans-serif";
            pointText.alignment = "center";
            pointText.baseline = "middle";
            pointText.fill = "white";
            pointText.noStroke();

            pointGroup.add(pointElem, pointText);
            _points.push(pointGroup);
          } else {
            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `point-${idx + 1}-${idx1}`;
            pointElem.fill = line.color;
            pointElem.noStroke();
            _points.push(pointElem);
          }
        });
      });
    }
    
    // Add obstacle vertices as draggable points
    shapes.forEach((shape, shapeIdx) => {
      shape.vertices.forEach((vertex, vertexIdx) => {
        let pointGroup = new Two.Group();
        pointGroup.id = `obstacle-${shapeIdx}-${vertexIdx}`;

        let pointElem = new Two.Circle(
          x(vertex.x),
          y(vertex.y),
          x(POINT_RADIUS),
        );
        pointElem.id = `obstacle-${shapeIdx}-${vertexIdx}-background`;
        pointElem.fill = shape.fillColor; // Match obstacle fill color
        pointElem.noStroke();

        let pointText = new Two.Text(
          `${vertexIdx + 1}`,
          x(vertex.x),
          y(vertex.y - 0.15),
        );
        pointText.id = `obstacle-${shapeIdx}-${vertexIdx}-text`;
        pointText.size = x(1.55);
        pointText.leading = 1;
        pointText.family = "ui-sans-serif, system-ui, sans-serif";
        pointText.alignment = "center";
        pointText.baseline = "middle";
        pointText.fill = "white";
        pointText.noStroke();
        pointGroup.add(pointElem, pointText);
        _points.push(pointGroup);
      });
    });

    // Add second path points (for dual path mode) - not in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && secondStartPoint && secondLines.length > 0) {
      let secondStartPointElem = new Two.Circle(
        x(secondStartPoint.x),
        y(secondStartPoint.y),
        x(POINT_RADIUS),
      );
      secondStartPointElem.id = `second-point-0-0`;
      secondStartPointElem.fill = secondLines[0]?.color || "#888";
      secondStartPointElem.noStroke();
      _points.push(secondStartPointElem);

      secondLines.forEach((line, idx) => {
        if (!line || !line.endPoint) return;
        [line.endPoint, ...line.controlPoints].forEach((point, idx1) => {
          if (idx1 > 0) {
            let pointGroup = new Two.Group();
            pointGroup.id = `second-point-${idx + 1}-${idx1}`;

            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `second-point-${idx + 1}-${idx1}-background`;
            pointElem.fill = line.color;
            pointElem.noStroke();

            let pointText = new Two.Text(
              `${idx1}`,
              x(point.x),
              y(point.y - 0.15),
            );
            pointText.id = `second-point-${idx + 1}-${idx1}-text`;
            pointText.size = x(1.55);
            pointText.leading = 1;
            pointText.family = "ui-sans-serif, system-ui, sans-serif";
            pointText.alignment = "center";
            pointText.baseline = "middle";
            pointText.fill = "white";
            pointText.noStroke();

            pointGroup.add(pointElem, pointText);
            _points.push(pointGroup);
          } else {
            let pointElem = new Two.Circle(
              x(point.x),
              y(point.y),
              x(POINT_RADIUS),
            );
            pointElem.id = `second-point-${idx + 1}-${idx1}`;
            pointElem.fill = line.color;
            pointElem.noStroke();
            _points.push(pointElem);
          }
        });
      });
    }

    // Add all control points for additional paths (full editing support)
    if ($activePaths.length > 0) {
      additionalPaths.forEach((pathData, pathIdx) => {
        if (!pathData.startPoint || !pathData.lines.length) return;
        
        // Add starting point
        let startPointElem = new Two.Circle(
          x(pathData.startPoint.x),
          y(pathData.startPoint.y),
          x(POINT_RADIUS * 0.9),
        );
        startPointElem.id = `additional-path-${pathIdx}-point-0-0`;
        startPointElem.fill = pathData.color || pathData.lines[0]?.color || "#888";
        startPointElem.noStroke();
        startPointElem.opacity = 0.8;
        _points.push(startPointElem);
        
        // Add all line points and control points
        pathData.lines.forEach((line, lineIdx) => {
          if (!line || !line.endPoint) return;
          
          [line.endPoint, ...line.controlPoints].forEach((point, pointIdx) => {
            if (pointIdx > 0) {
              // Control point with number
              let pointGroup = new Two.Group();
              pointGroup.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}`;

              let pointElem = new Two.Circle(
                x(point.x),
                y(point.y),
                x(POINT_RADIUS * 0.9),
              );
              pointElem.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}-background`;
              pointElem.fill = pathData.color || line.color;
              pointElem.noStroke();

              let pointText = new Two.Text(
                `${pointIdx}`,
                x(point.x),
                y(point.y - 0.15),
              );
              pointText.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}-text`;
              pointText.size = x(1.4);
              pointText.leading = 1;
              pointText.family = "ui-sans-serif, system-ui, sans-serif";
              pointText.alignment = "center";
              pointText.baseline = "middle";
              pointText.fill = "white";
              pointText.noStroke();

              pointGroup.add(pointElem, pointText);
              pointGroup.opacity = 0.8;
              _points.push(pointGroup);
            } else {
              // End point without number
              let pointElem = new Two.Circle(
                x(point.x),
                y(point.y),
                x(POINT_RADIUS * 0.9),
              );
              pointElem.id = `additional-path-${pathIdx}-point-${lineIdx + 1}-${pointIdx}`;
              pointElem.fill = pathData.color || line.color;
              pointElem.noStroke();
              pointElem.opacity = 0.8;
              _points.push(pointElem);
            }
          });
        });
      });
    }

    return _points;
  })();

  $: path = (() => {
    // Hide main path when in multi-path mode (isolated visualization)
    if ($activePaths.length > 0) {
      return [];
    }
    
    let _path: CanvasPathElement[] = [];

    lines.forEach((line, idx) => {
      if (!line || !line.endPoint) return; // Skip invalid lines or lines without endPoint
      let _startPoint =
        idx === 0 ? startPoint : lines[idx - 1]?.endPoint || null;
      if (!_startPoint) return; // Skip if previous line's endPoint is missing

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `line-${idx + 1}`,
          line.color,
          settings.pathOpacity || 1.0,
          settings,
        ),
      );
    });

    return _path;
  })();

  // Second path rendering (for dual path mode)
  $: secondPath = (() => {
    // Don't show second path when in multi-path mode (use activePaths instead)
    if ($activePaths.length > 0 || !$dualPathMode || !secondStartPoint || secondLines.length === 0) {
      return [];
    }

    let _path: CanvasPathElement[] = [];

    secondLines.forEach((line, idx) => {
      if (!line || !line.endPoint) return;
      let _startPoint =
        idx === 0 ? secondStartPoint : secondLines[idx - 1]?.endPoint || null;
      if (!_startPoint) return;

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `second-line-${idx + 1}`,
          line.color,
          settings.pathOpacity || 1.0,
          {
            ...settings,
            showVelocityGradient: settings.showVelocityGradient,
          },
        ),
      );
    });

    return _path;
  })();

  // Render all additional paths
  $: additionalPathElements = additionalPaths.map((pathData, pathIdx) => {
    if (!pathData.startPoint || pathData.lines.length === 0) {
      return [];
    }

    let _path: CanvasPathElement[] = [];
    // All paths should be clearly visible - only slight opacity variation
    const pathOpacityBase = settings.pathOpacity || 1.0;
    const opacity = (1.0 - (pathIdx * 0.1)) * pathOpacityBase;

    pathData.lines.forEach((line, idx) => {
      if (!line || !line.endPoint) return;
      let _startPoint =
        idx === 0 ? pathData.startPoint : pathData.lines[idx - 1]?.endPoint || null;
      if (!_startPoint) return;

      _path.push(
        ...createPathDrawElements(
          line,
          _startPoint,
          `additional-path-${pathIdx}-line-${idx + 1}`,
          pathData.color || line.color,
          opacity,
          {
            ...pathData.settings,
            showVelocityGradient: settings.showVelocityGradient,
          },
        ),
      );
    });

    return _path;
  });

  $: activeAutoTotalTime = (() => {
    if ($activePaths.length > 0) {
      return additionalPaths.reduce((maxTime, pathData) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return maxTime;
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
        );
        return Math.max(maxTime, pathTime.totalTime || 0);
      }, 0);
    }

    return Math.max(
      timePrediction?.totalTime || 0,
      secondTimePrediction?.totalTime || 0,
    );
  })();

  $: currentAutoTime =
    activeAutoTotalTime > 0
      ? (clampNumber(percent, 0, 100) / 100) * activeAutoTotalTime
      : 0;
  $: autoCountdownRemaining = AUTO_COUNTDOWN_SECONDS - currentAutoTime;
  $: autoCountdownProgress = clampNumber(
    currentAutoTime / AUTO_COUNTDOWN_SECONDS,
    0,
    1,
  );
  $: autoCountdownColor = getCountdownColor(autoCountdownRemaining);

  $: pathAnnotations = (() => {
    if (!settings.showPathAnnotations) return [];

    if ($activePaths.length > 0) {
      return additionalPaths.flatMap((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return [];
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
        );
        return buildPathAnnotations(
          `additional-${pathIndex}`,
          pathData.startPoint,
          pathData.lines,
          pathTime,
          pathData.settings,
        );
      });
    }

    const annotations = buildPathAnnotations(
      "main",
      startPoint,
      lines,
      timePrediction,
      settings,
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      annotations.push(
        ...buildPathAnnotations(
          "second",
          secondStartPoint,
          secondLines,
          secondTimePrediction,
          settings,
        ),
      );
    }

    return annotations;
  })();

  $: eventPins = (() => {
    if (!settings.showEventPins) return [];

    if ($activePaths.length > 0) {
      return additionalPaths.flatMap((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return [];
        const pathTime = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
        );
        return buildEventPins(
          `additional-${pathIndex}`,
          pathData.startPoint,
          pathData.lines,
          pathTime,
          pathData.settings,
        );
      });
    }

    const pins = buildEventPins(
      "main",
      startPoint,
      lines,
      timePrediction,
      settings,
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      pins.push(
        ...buildEventPins(
          "second",
          secondStartPoint,
          secondLines,
          secondTimePrediction,
          settings,
        ),
      );
    }

    return pins;
  })();

  $: swerveWheelOverlays = (() => {
    if (!settings.showSwerveModules) return [];

    const overlays: SwerveWheelOverlay[] = [];

    if ($activePaths.length > 0) {
      additionalPaths.forEach((pathData, pathIndex) => {
        if (!pathData.startPoint || pathData.lines.length === 0) return;
        const pathTimePrediction = calculatePathTime(
          pathData.startPoint,
          pathData.lines,
          pathData.settings,
          pathData.sequence,
        );
        const maxDuration = effectiveAnimationDuration;
        const thisDuration = getAnimationDuration(pathTimePrediction.totalTime / 1000);
        const completionPercent = maxDuration > 0 ? (thisDuration / maxDuration) * 100 : 0;
        const actualPercent = Math.min(percent, completionPercent);
        const normalizedPercent =
          completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;
        const robotState = additionalRobotStates[pathIndex];

        if (!robotState) return;

        overlays.push(
          ...buildSwerveWheelModules(
            `additional-${pathIndex}`,
            robotState.xy,
            robotState.heading,
            getSwerveWheelAnglesCss(
              normalizedPercent,
              pathData.startPoint,
              pathData.lines,
              pathTimePrediction,
              pathData.settings,
              robotState.heading,
            ),
            0.9 - pathIndex * 0.12,
            pathData.color || "#06b6d4",
          ),
        );
      });

      return overlays;
    }

    overlays.push(
      ...buildSwerveWheelModules(
        "main",
        robotXY,
        robotHeading,
        getSwerveWheelAnglesCss(
          percent,
          startPoint,
          lines,
          timePrediction,
          settings,
          robotHeading,
        ),
        1,
        "#06b6d4",
      ),
    );

    if ($dualPathMode && secondStartPoint && secondLines.length > 0 && secondTimePrediction) {
      const maxDuration = effectiveAnimationDuration;
      const thisDuration = getAnimationDuration(secondTimePrediction.totalTime / 1000);
      const completionPercent = maxDuration > 0 ? (thisDuration / maxDuration) * 100 : 0;
      const actualPercent = Math.min(percent, completionPercent);
      const normalizedPercent =
        completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

      overlays.push(
        ...buildSwerveWheelModules(
          "second",
          secondRobotXY,
          secondRobotHeading,
          getSwerveWheelAnglesCss(
            normalizedPercent,
            secondStartPoint,
            secondLines,
            secondTimePrediction,
            settings,
            secondRobotHeading,
          ),
          0.8,
          "#fca5a5",
        ),
      );
    }

    if (settings.showOnionLayers) {
      const mainLayers = getVisibleOnionLayers(
        startPoint,
        lines,
        timePrediction,
        settings,
      );
      overlays.push(
        ...buildOnionSwerveModules(
          "main",
          startPoint,
          lines,
          mainLayers,
          0.72,
          settings.onionColor || "#dc2626",
        ),
      );

      if ($dualPathMode && secondStartPoint && secondLines.length > 0 && secondTimePrediction) {
        const secondLayers = getVisibleOnionLayers(
          secondStartPoint,
          secondLines,
          secondTimePrediction,
          settings,
        );
        overlays.push(
          ...buildOnionSwerveModules(
            "second",
            secondStartPoint,
            secondLines,
            secondLayers,
            0.65,
            "#fca5a5",
          ),
        );
      }
    }

    return overlays;
  })();

  $: shapeElements = (() => {
    // Obstacles removed: return empty array for shape elements
    let _shapes: Path[] = [];

    shapes.forEach((shape, idx) => {
      if (shape.vertices.length >= 3) {
        // Create polygon from vertices - properly format for Two.js
        let vertices = [];

        // Start with move command for first vertex
        vertices.push(
          new Two.Anchor(
            x(shape.vertices[0].x),
            y(shape.vertices[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        // Add line commands for remaining vertices
        for (let i = 1; i < shape.vertices.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(shape.vertices[i].x),
              y(shape.vertices[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the shape
        vertices.push(
          new Two.Anchor(
            x(shape.vertices[0].x),
            y(shape.vertices[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let shapeElement = new Two.Path(vertices);
        shapeElement.id = `shape-${idx}`;
        shapeElement.stroke = shape.color;
        shapeElement.fill = shape.color;
        shapeElement.opacity = 0.4;
        shapeElement.linewidth = x(0.8);
        shapeElement.automatic = false;

        _shapes.push(shapeElement);
      }
    });

    return _shapes;
  })();

  $: ghostPathElement = (() => {
    let ghostPath: Path | null = null;

    // Don't show ghost paths in multi-path mode
    if ($activePaths.length === 0 && settings.showGhostPaths && lines.length > 0) {
      const ghostPoints = generateGhostPathPoints(
        startPoint,
        lines,
        settings.rWidth,
        settings.rHeight,
        50,
      );

      if (ghostPoints.length >= 3) {
        // Create polygon from ghost path points
        let vertices = [];

        // Start with move command for first point
        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        // Add line commands for remaining points
        for (let i = 1; i < ghostPoints.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(ghostPoints[i].x),
              y(ghostPoints[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the shape
        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        ghostPath = new Two.Path(vertices);
        ghostPath.id = "ghost-path";
        ghostPath.stroke = "#a78bfa"; // Light purple/lavender
        ghostPath.fill = "#a78bfa";
        ghostPath.opacity = 0.15;
        ghostPath.linewidth = x(0.5);
        ghostPath.automatic = false;
      }
    }

    return ghostPath;
  })();

  // Second ghost path for dual path mode
  $: secondGhostPathElement = (() => {
    let ghostPath: Path | null = null;

    // Don't show second ghost path in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && settings.showGhostPaths && secondLines.length > 0 && secondStartPoint) {
      const ghostPoints = generateGhostPathPoints(
        secondStartPoint,
        secondLines,
        settings.rWidth,
        settings.rHeight,
        50,
      );

      if (ghostPoints.length >= 3) {
        let vertices = [];

        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < ghostPoints.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(ghostPoints[i].x),
              y(ghostPoints[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        vertices.push(
          new Two.Anchor(
            x(ghostPoints[0].x),
            y(ghostPoints[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        ghostPath = new Two.Path(vertices);
        ghostPath.id = "ghost-path-2";
        ghostPath.stroke = "#fca5a5"; // Light red/pink for second robot
        ghostPath.fill = "#fca5a5";
        ghostPath.opacity = 0.15;
        ghostPath.linewidth = x(0.5);
        ghostPath.automatic = false;
      }
    }

    return ghostPath;
  })();

  // Ghost paths for additional paths in multi-path mode
  $: additionalGhostPathElements = (() => {
    let ghostPaths: Path[] = [];

    if ($activePaths.length > 0 && settings.showGhostPaths) {
      additionalPaths.forEach((pathData, pathIdx) => {
        if (!pathData.startPoint || !pathData.lines.length) return;

        const ghostPoints = generateGhostPathPoints(
          pathData.startPoint,
          pathData.lines,
          settings.rWidth,
          settings.rHeight,
          50,
        );

        if (ghostPoints.length >= 3) {
          let vertices = [];

          vertices.push(
            new Two.Anchor(
              x(ghostPoints[0].x),
              y(ghostPoints[0].y),
              0,
              0,
              0,
              0,
              Two.Commands.move,
            ),
          );

          for (let i = 1; i < ghostPoints.length; i++) {
            vertices.push(
              new Two.Anchor(
                x(ghostPoints[i].x),
                y(ghostPoints[i].y),
                0,
                0,
                0,
                0,
                Two.Commands.line,
              ),
            );
          }

          vertices.push(
            new Two.Anchor(
              x(ghostPoints[0].x),
              y(ghostPoints[0].y),
              0,
              0,
              0,
              0,
              Two.Commands.close,
            ),
          );

          vertices.forEach((point) => (point.relative = false));

          const ghostPath = new Two.Path(vertices);
          ghostPath.id = `ghost-path-additional-${pathIdx}`;
          ghostPath.stroke = pathData.color || "#a78bfa";
          ghostPath.fill = pathData.color || "#a78bfa";
          ghostPath.opacity = 0.15;
          ghostPath.linewidth = x(0.5);
          ghostPath.automatic = false;
          
          ghostPaths.push(ghostPath);
        }
      });
    }

    return ghostPaths;
  })();

  $: onionLayerElements = (() => {
    let onionLayers: Path[] = [];

    // Don't show onion layers in multi-path mode
    if ($activePaths.length === 0 && settings.showOnionLayers && lines.length > 0) {
      const spacing = settings.onionLayerSpacing || 6;
      let layers = generateOnionLayers(
        startPoint,
        lines,
        settings.rWidth,
        settings.rHeight,
        spacing,
      );

      // If user requested onion layers only for the next point, filter to the relevant line
      if (
        settings.onionNextPointOnly &&
        timePrediction &&
        timePrediction.timeline
      ) {
        const currentTime = (timePrediction.totalTime || 0) * (percent / 100);
        const travelEvents = (timePrediction.timeline || []).filter(
          (ev) => ev.type === "travel",
        );

        let selectedLineIndex: number | null = null;

        // Current travel segment
        const currentTravel = travelEvents.find(
          (ev) => ev.startTime <= currentTime && ev.endTime >= currentTime,
        );
        if (currentTravel) {
          selectedLineIndex = currentTravel.lineIndex as number;
        } else {
          // Next upcoming travel segment
          const nextTravel = travelEvents.find(
            (ev) => ev.startTime > currentTime,
          );
          if (nextTravel) selectedLineIndex = nextTravel.lineIndex as number;
          else if (travelEvents.length)
            selectedLineIndex = travelEvents[travelEvents.length - 1]
              .lineIndex as number;
        }

        if (selectedLineIndex !== null) {
          layers = layers.filter((l: any) => l.lineIndex === selectedLineIndex);
        }
      }

      layers.forEach((layer, idx) => {
        // Create a rectangle from the robot corners
        let vertices: any[] = [];

        // Create path from corners: front-left -> front-right -> back-right -> back-left
        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < layer.corners.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(layer.corners[i].x),
              y(layer.corners[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        // Close the path by returning to the first corner
        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let onionRect = new Two.Path(vertices);
        onionRect.id = `onion-layer-${idx}`;
        onionRect.stroke = settings.onionColor || "#dc2626";
        onionRect.noFill();
        // Increase opacity so colliders are more visible
        onionRect.opacity = 0.9;
        onionRect.linewidth = x(0.28);
        onionRect.automatic = false;

        onionLayers.push(onionRect);
      });
    }

    return onionLayers;
  })();

  // Second onion layers for dual path mode
  $: secondOnionLayerElements = (() => {
    let onionLayers: Path[] = [];

    // Don't show second onion layers in multi-path mode
    if ($activePaths.length === 0 && $dualPathMode && settings.showOnionLayers && secondLines.length > 0 && secondStartPoint) {
      const spacing = settings.onionLayerSpacing || 6;
      let layers = generateOnionLayers(
        secondStartPoint,
        secondLines,
        settings.rWidth,
        settings.rHeight,
        spacing,
      );

      // If user requested onion layers only for the next point, filter to the relevant line
      if (
        settings.onionNextPointOnly &&
        secondTimePrediction &&
        secondTimePrediction.timeline
      ) {
        const currentTime = (secondTimePrediction.totalTime || 0) * (percent / 100);
        const travelEvents = (secondTimePrediction.timeline || []).filter(
          (ev) => ev.type === "travel",
        );

        let selectedLineIndex: number | null = null;

        const currentTravel = travelEvents.find(
          (ev) => ev.startTime <= currentTime && ev.endTime >= currentTime,
        );
        if (currentTravel) {
          selectedLineIndex = currentTravel.lineIndex as number;
        } else {
          const nextTravel = travelEvents.find(
            (ev) => ev.startTime > currentTime,
          );
          if (nextTravel) selectedLineIndex = nextTravel.lineIndex as number;
          else if (travelEvents.length)
            selectedLineIndex = travelEvents[travelEvents.length - 1]
              .lineIndex as number;
        }

        if (selectedLineIndex !== null) {
          layers = layers.filter((l: any) => l.lineIndex === selectedLineIndex);
        }
      }

      layers.forEach((layer, idx) => {
        let vertices: any[] = [];

        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.move,
          ),
        );

        for (let i = 1; i < layer.corners.length; i++) {
          vertices.push(
            new Two.Anchor(
              x(layer.corners[i].x),
              y(layer.corners[i].y),
              0,
              0,
              0,
              0,
              Two.Commands.line,
            ),
          );
        }

        vertices.push(
          new Two.Anchor(
            x(layer.corners[0].x),
            y(layer.corners[0].y),
            0,
            0,
            0,
            0,
            Two.Commands.close,
          ),
        );

        vertices.forEach((point) => (point.relative = false));

        let onionRect = new Two.Path(vertices);
        onionRect.id = `second-onion-layer-${idx}`;
        onionRect.stroke = "#fca5a5"; // Light red/pink for second path
        onionRect.noFill();
        onionRect.opacity = 0.9;
        onionRect.linewidth = x(0.28);
        onionRect.automatic = false;

        onionLayers.push(onionRect);
      });
    }

    return onionLayers;
  })();

  let isLoaded = false;
  // Reactively trigger when any saveable data changes
  $: {
    if (isLoaded && (lines || shapes || startPoint || settings)) {
      isUnsaved.set(true);
    }
  }

  // Allow the app to stabilize before tracking changes
  onMount(() => {
    setTimeout(() => {
      isLoaded = true;
      recordChange();
    }, 500);
  });
  onMount(async () => {
    // Load saved settings
    const savedSettings = await loadSettings();
    settings = normalizeLegacyFieldMap({ ...savedSettings });

    // Update robot dimensions from loaded settings
    robotWidth = settings.rWidth;
    robotHeight = settings.rHeight;
  });
  // Debounced save function
  const debouncedSaveSettings = debounce(async (settingsToSave: Settings) => {
    await saveSettings(settingsToSave);
  }, 1000);
  // Save after 1 second of inactivity

  // Watch for settings changes and save
  $: {
    if (settings) {
      debouncedSaveSettings(settings);
    }
  }

  // Initialize animation controller
  onMount(() => {
    animationController = createAnimationController(
      animationDuration,
      (newPercent) => {
        percent = newPercent;
      },
      () => {
        // Animation completed callback
        console.log("Animation completed");
        playing = false;
      },
    );
  });
  $: if (animationController) {
    animationController.setDuration(effectiveAnimationDuration);
  }

  $: if (animationController) {
    animationController.setLoop(loopAnimation);
    // Sync UI state with controller
    playing = animationController.isPlaying();
  }

  // Save Function
  // Save the current project into the browser-backed store (or download)
  async function saveProject() {
    try {
      await saveFile();
    } catch (e) {
      console.error("Failed to save project:", e);
      alert("Failed to save file.");
    }
  }

  // Save an additional path back to its file
  async function saveAdditionalPath(pathIdx: number) {
    const pathData = additionalPaths[pathIdx];
    if (!pathData || !pathData.filePath) return;
    
    try {
      const fileData = JSON.stringify({
        startPoint: pathData.startPoint,
        lines: pathData.lines,
        shapes: pathData.shapes,
        sequence: pathData.sequence,
        pathChains: pathData.pathChains || [],
        poseVariables: pathData.poseVariables || [],
        pathVariables: pathData.pathVariables || [],
        numberVariables: pathData.numberVariables || [],
        settings: pathData.settings,
        version: "1.2.1",
        timestamp: new Date().toISOString(),
      });
      
      await browserFileStore.writeFile(pathData.filePath, fileData);
      console.log(`Auto-saved additional path: ${pathData.filePath}`);
    } catch (error) {
      console.error(`Failed to save additional path ${pathData.filePath}:`, error);
      throw error;
    }
  }

  // Keyboard shortcut for save
  hotkeys("cmd+s, ctrl+s", function (event, handler) {
    event.preventDefault();
    if ($activePaths.length > 0) {
      // Multiple paths mode - save all modified paths
      showDualPathSaveDialog = true;
    } else if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
      showDualPathSaveDialog = true;
    } else {
      showSaveDialog = true;
    }
  });

  // Export path animation as GIF
  async function exportPathAsGif() {
    if (!twoElement || !two) {
      alert("Canvas not ready. Please try again.");
      return;
    }
    
    // Two.js can render as canvas or SVG; exporter supports both.
    const rendererElement = two.renderer.domElement;
    if (!rendererElement) {
      alert("Unable to access renderer for export.");
      return;
    }
    
    // Check if we have paths to export
    const hasActivePaths = $activePaths.length > 0;
    const hasDualPath = $dualPathMode && secondStartPoint && secondLines.length > 0;
    const hasSinglePath = lines.length > 0;
    
    if (!hasActivePaths && !hasDualPath && !hasSinglePath) {
      alert("No paths to export. Please create a path first.");
      return;
    }

    try {
      exportingGif = true;
      cancelGifExport = false;
      gifExportProgress = 0;
      gifExportStatus = "Calculating animation duration...";

      const scale = 0.65;
      const viewWidth = twoElement.clientWidth;
      const viewHeight = twoElement.clientHeight;
      const robotPixelWidth = x(robotWidth);
      const robotPixelHeight = x(robotHeight);

      const imageCache = new Map<string, HTMLImageElement>();
      const loadImage = (src: string) =>
        new Promise<HTMLImageElement>((resolve, reject) => {
          if (imageCache.has(src)) {
            resolve(imageCache.get(src)!);
            return;
          }
          const image = new Image();
          image.onload = () => {
            imageCache.set(src, image);
            resolve(image);
          };
          image.onerror = () => reject(new Error(`Failed to load image: ${src}`));
          image.src = src;
        });

      const fieldImage = await loadImage(fieldMapSrc).catch(async () => {
        return loadImage("/fields/decode.webp");
      });
      const robotImage = await loadImage(settings.robotImage || "/robot.png").catch(async () => {
        return loadImage("/robot.png");
      });

      const drawRobot = (
        ctx: CanvasRenderingContext2D,
        xy: BasePoint,
        headingDeg: number,
        opacity = 1,
      ) => {
        ctx.save();
        ctx.globalAlpha = opacity;
        ctx.translate(xy.x * scale, xy.y * scale);
        ctx.rotate((headingDeg * Math.PI) / 180);
        ctx.drawImage(
          robotImage,
          (-robotPixelWidth * scale) / 2,
          (-robotPixelHeight * scale) / 2,
          robotPixelWidth * scale,
          robotPixelHeight * scale,
        );
        ctx.restore();
      };

      // Calculate total animation duration
      let totalDuration = 0;
      
      if (hasActivePaths) {
        // Multiple paths mode - use the longest path duration
        for (const pathData of additionalPaths) {
          if (!pathData.startPoint) continue;
          const pathTime = calculatePathTime(
            pathData.startPoint,
            pathData.lines,
            pathData.settings,
            pathData.sequence
          );
          totalDuration = Math.max(totalDuration, pathTime?.totalTime || 0);
        }
      } else if (hasDualPath) {
        // Dual path mode - use the longer path
        const path1Time = calculatePathTime(startPoint, lines, settings, sequence);
        const path2Time = secondStartPoint 
          ? calculatePathTime(secondStartPoint, secondLines, settings, secondSequence)
          : { totalTime: 0 };
        totalDuration = Math.max(path1Time?.totalTime || 0, path2Time?.totalTime || 0);
      } else {
        // Single path mode
        const pathTime = calculatePathTime(startPoint, lines, settings, sequence);
        totalDuration = pathTime?.totalTime || 0;
      }

      if (totalDuration <= 0) {
        alert("Path duration is too short to export.");
        exportingGif = false;
        return;
      }

      gifExportStatus = "Preparing animation...";
      
      // Stop any playing animation and reset to start
      const wasPlaying = playing;
      pause();
      percent = 0;
      animationController.reset();
        two.update(); // Make sure Two.js renders the initial state
      await tick(); // Allow UI to update

      gifExportStatus = "Capturing frames...";
      
      // Export as GIF with manual frame control
      const durationMs = totalDuration * 1000;
      const blob = await exportAsGif({
        source: rendererElement as HTMLCanvasElement | SVGSVGElement,
        duration: durationMs,
        fps: 20, // Higher FPS for smoother animation
        quality: 15, // Slightly lower quality for smaller file size
        scale, // Lower resolution for smaller file size
        shouldCancel: () => cancelGifExport,
        onDrawBackground: (ctx, outputWidth, outputHeight) => {
          ctx.drawImage(fieldImage, 0, 0, outputWidth, outputHeight);
        },
        onDrawForeground: (ctx) => {
          if ($activePaths.length === 0) {
            drawRobot(ctx, robotXY, robotHeading, 1);
            if ($dualPathMode && secondStartPoint && secondLines.length > 0) {
              drawRobot(ctx, secondRobotXY, secondRobotHeading, 0.8);
            }
            return;
          }

          additionalRobotStates.forEach((robotState, idx) => {
            const opacity = Math.max(0.2, 1 - idx * 0.15);
            drawRobot(ctx, robotState.xy, robotState.heading, opacity);
          });
        },
        onProgress: (progress) => {
          gifExportProgress = progress;
          if (progress < 0.5) {
            gifExportStatus = `Capturing frames... ${Math.round(progress * 200)}%`;
          } else {
            gifExportStatus = `Encoding GIF... ${Math.round((progress - 0.5) * 200)}%`;
          }
        },
        onFrameAdvance: async (frameIndex, totalFrames) => {
          // Calculate the percentage for this frame
          const framePercent = (frameIndex / (totalFrames - 1)) * 100;
          
          // Update the animation to this frame
          percent = framePercent;
          animationController.seekToPercent(framePercent);
          two.update(); // Force Two.js to render
          
          // Allow UI to update before capturing
          await tick();
        },
      });

      // Reset animation
      percent = 0;
      animationController.reset();
      
      // Resume playing if it was playing before
      if (wasPlaying) {
        play();
      }

      gifExportStatus = "Saving file...";
      
      // Download the GIF
      const fileName = $currentFilePath
        ? $currentFilePath.split(/[\/\\]/).pop()?.replace(/\.pp$/, "")
        : hasActivePaths
          ? "multiple_paths"
          : hasDualPath
            ? "dual_path"
            : "path_animation";
      
      downloadBlob(blob, `${fileName}.gif`);

      exportingGif = false;
      gifExportProgress = 0;
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : String(error);
      console.error("GIF export error:", errorMsg);
      
      // Don't show alert if user cancelled
      if (!errorMsg.includes('cancelled')) {
        alert("Failed to export GIF: " + errorMsg);
      }
      
      cancelGifExport = false;
      exportingGif = false;
      gifExportProgress = 0;
      pause();
    }
  }
  $: {
    // This handles both 'travel' (movement) and 'wait' (stationary rotation) events.
    // Don't show main robot in multi-path mode
    if ($activePaths.length === 0 && timePrediction && timePrediction.timeline && lines.length > 0) {
      const state = calculateRobotState(
        percent,
        timePrediction.timeline,
        lines,
        startPoint,
        settings,
        x,
        y,
      );
      robotXY = { x: state.x, y: state.y };
      robotHeading = state.heading;
    } else {
      // Fallback for initialization or empty state
      robotXY = { x: x(startPoint.x), y: y(startPoint.y) };
      // Calculate initial heading based on start point settings
      if (startPoint.heading === "linear") robotHeading = -startPoint.startDeg;
      else if (startPoint.heading === "constant")
        robotHeading = -startPoint.degrees;
      else robotHeading = 0;
    }
  }

  // Second robot state calculation (for dual path mode)
  $: {
    // Don't show second robot in multi-path mode
    if (
      $activePaths.length === 0 &&
      $dualPathMode &&
      timePrediction &&
      secondTimePrediction &&
      secondTimePrediction.timeline &&
      secondLines.length > 0 &&
      secondStartPoint
    ) {
      // Calculate actual percent for this path based on max duration
      const maxDuration = effectiveAnimationDuration;
      const thisDuration = getAnimationDuration(secondTimePrediction.totalTime / 1000);
      const completionPercent = (thisDuration / maxDuration) * 100;
      
      // If this path should be complete, cap at 100% (robot waits at end)
      const actualPercent = Math.min(percent, completionPercent);
      const normalizedPercent = completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

      const state = calculateRobotState(
        normalizedPercent,
        secondTimePrediction.timeline,
        secondLines,
        secondStartPoint,
        settings,
        x,
        y,
      );
      secondRobotXY = { x: state.x, y: state.y };
      secondRobotHeading = state.heading;
    } else {
      // Fallback or not in dual mode
      secondRobotXY = { x: 0, y: 0 };
      secondRobotHeading = 0;
    }
  }

  // Calculate robot states for all additional paths
  $: {
    additionalRobotStates = additionalPaths.map((pathData) => {
      if (!pathData.startPoint) {
        return {
          xy: { x: 0, y: 0 },
          heading: 0,
        };
      }

      const pathTimePrediction = calculatePathTime(
        pathData.startPoint,
        pathData.lines,
        pathData.settings,
        pathData.sequence
      );
      
      if (pathTimePrediction && pathTimePrediction.timeline && pathData.lines.length > 0 && pathData.startPoint) {
        // Calculate actual percent for this path based on max duration
        const maxDuration = effectiveAnimationDuration;
        const thisDuration = getAnimationDuration(pathTimePrediction.totalTime / 1000);
        const completionPercent = (thisDuration / maxDuration) * 100;
        
        // If this path should be complete, cap at 100% (robot waits at end)
        const actualPercent = Math.min(percent, completionPercent);
        const normalizedPercent = completionPercent > 0 ? (actualPercent / completionPercent) * 100 : 0;

        const state = calculateRobotState(
          normalizedPercent,
          pathTimePrediction.timeline,
          pathData.lines,
          pathData.startPoint,
          pathData.settings,
          x,
          y,
        );
        
        return {
          xy: { x: state.x, y: state.y },
          heading: state.heading,
        };
      }
      
      return {
        xy: { x: 0, y: 0 },
        heading: 0,
      };
    });
  }

  // Event markers removed: no runtime visualization created

  $: (() => {
    if (!two) {
      return;
    }

    two.renderer.domElement.style["z-index"] = "30";
    two.renderer.domElement.style["position"] = "absolute";
    two.renderer.domElement.style["top"] = "0px";
    two.renderer.domElement.style["left"] = "0px";
    two.renderer.domElement.style["width"] = "100%";
    two.renderer.domElement.style["height"] = "100%";

    two.clear();

    two.add(...shapeElements);
    if (ghostPathElement) {
      two.add(ghostPathElement);
    }
    if (secondGhostPathElement) {
      two.add(secondGhostPathElement);
    }
    if (additionalGhostPathElements.length > 0) {
      two.add(...additionalGhostPathElements);
    }
    if (onionLayerElements.length > 0) {
      two.add(...onionLayerElements);
    }
    if (secondOnionLayerElements.length > 0) {
      two.add(...secondOnionLayerElements);
    }
    two.add(...path);
    if ($dualPathMode && secondPath.length > 0) {
      two.add(...secondPath);
    }
    // Add all additional paths
    if ($activePaths.length > 0) {
      additionalPathElements.forEach((pathElements) => {
        if (pathElements.length > 0) {
          two.add(...pathElements);
        }
      });
    }
    two.add(...points);

    two.update();
  })();
  async function saveFileAs() {
    const win: any = window as any;
    const content = JSON.stringify(
      {
        startPoint,
        lines,
        shapes,
        sequence,
        pathChains,
        poseVariables,
        pathVariables,
        numberVariables,
        settings,
        version: "1.2.1",
        timestamp: new Date().toISOString(),
      },
      null,
      2,
    );

    // Prefer File System Access API if available: opens native Save dialog
    if (win.showSaveFilePicker) {
      try {
        const opts = {
          suggestedName: $currentFilePath
            ? $currentFilePath.split(/[\/]/).pop()
            : "path.pp",
          types: [
            {
              description: "Path files",
              accept: { "application/json": [".pp", ".json"] },
            },
          ],
        };

        const handle = await win.showSaveFilePicker(opts);
        if (!handle) {
          // User cancelled
          return;
        }

        const writable = await handle.createWritable();
        await writable.write(content);
        await writable.close();

        // Update app state to reflect saved file
        try {
          currentFilePath.set(
            handle.name || (typeof handle === "string" ? handle : null),
          );
        } catch (e) {
          // ignore
        }
        isUnsaved.set(false);
        alert(`Saved to: ${handle.name || "selected file"}`);
        return;
      } catch (err) {
        console.error("SaveFilePicker error:", err);
        // fall through to download fallback
      }
    }

    // If showSaveFilePicker is not available or failed, try showOpenFilePicker to let user pick an existing file to overwrite
    if (win.showOpenFilePicker) {
      try {
        const [handle] = await win.showOpenFilePicker({
          types: [
            {
              description: "Path files",
              accept: { "application/json": [".pp", ".json"] },
            },
          ],
          multiple: false,
        });

        if (handle) {
          const writable = await handle.createWritable();
          await writable.write(content);
          await writable.close();
          try {
            currentFilePath.set(handle.name || null);
          } catch (e) {}
          isUnsaved.set(false);
          alert(`Saved to local file: ${handle.name || "selected file"}`);
          return;
        }
      } catch (err) {
        console.error("showOpenFilePicker error:", err);
        // fall through to download fallback
      }
    }

    // Fallback for browsers without File System Access (e.g., Firefox).
    // Automatically save into the app's browser-backed storage to avoid forcing a download.
    try {
      await saveFile();
      alert(
        "Your browser does not support native file dialogs. The project was saved to the app's storage.\n\nOpen the File Manager to download or export the file to your computer.",
      );
    } catch (err) {
      console.error("Failed to save into app storage:", err);
      // As a last resort, download the file
      try {
        downloadTrajectory(
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          poseVariables,
          pathVariables,
          numberVariables,
          settings,
        );
      } catch (err2) {
        console.error("Save As fallback failed:", err2);
        alert(
          "Failed to save file. Your browser may not support file picker APIs.",
        );
      }
    }
  }

  function animate(timestamp: number) {
    if (!startTime) {
      startTime = timestamp;
    }

    if (previousTime !== null) {
      const deltaTime = timestamp - previousTime;
      if (percent >= 100) {
        percent = 0;
      } else {
        percent += (0.65 / lines.length) * (deltaTime * 0.1);
      }
    }

    previousTime = timestamp;

    if (playing) {
      requestAnimationFrame(animate);
    }
  }

  function play() {
    animationController.play();
    playing = true;
  }

  function pause() {
    animationController.pause();
    playing = false;
  }

  function resetAnimation() {
    animationController.reset();
    playing = false;
  }

  // Handle slider changes
  function handleSeek(newPercent: number) {
    if (animationController) {
      animationController.seekToPercent(newPercent);
    }
  }

  onMount(() => {
    two = new Two({
      fitted: true,
      type: Two.Types.svg,
    }).appendTo(twoElement);

    updateRobotImageDisplay();

    let currentElem: string | null = null;
    let isDown = false;
    let dragOffset = { x: 0, y: 0 }; // Store offset to prevent snapping to center

    const isLockedPathElem = (id: string | null): boolean => {
      if (!id || !id.startsWith("point")) return false;
      const parts = id.split("-");
      const lineIdx = Number(parts[1]) - 1;
      const pointIdx = Number(parts[2]);
      if (Number.isNaN(lineIdx)) return false;
      if (lineIdx < 0) return !!startPoint.locked || !!startPoint.poseVariableId;
      if (pointIdx === 0 && lines[lineIdx]?.endPoint?.poseVariableId) return true;
      return !!lines[lineIdx]?.locked;
    };

    two.renderer.domElement.addEventListener("mousemove", (evt: MouseEvent) => {
      const elem = document.elementFromPoint(evt.clientX, evt.clientY);

      if (isDown && currentElem) {
        const parts = currentElem.split("-");
        const isPathPoint = parts[0] === "point";
        const isShapePoint = parts[0] === "shape";

        // Skip dragging locked paths
        if (isPathPoint) {
          const hitLine = Number(parts[1]) - 1;
          if (hitLine >= 0 && lines[hitLine]?.locked) return;
        }

        // Use simple bounding rect math to match D3 scales which are bound to clientWidth/Height
        const rect = two.renderer.domElement.getBoundingClientRect();
        const xPos = evt.clientX - rect.left;
        const yPos = evt.clientY - rect.top;

        // Get current store values for reactivity
        const currentGridSize = $gridSize;
        const currentSnapToGrid = $snapToGrid;
        const currentShowGrid = $showGrid;

        // Apply drag offset (in inches) to the raw mouse position
        let rawInchX = x.invert(xPos) + dragOffset.x;
        let rawInchY = y.invert(yPos) + dragOffset.y;

        let inchX = rawInchX;
        let inchY = rawInchY;

        // Always apply grid snapping when enabled
        if (currentSnapToGrid && currentShowGrid && currentGridSize > 0) {
          // Force snap to nearest grid point
          inchX = Math.round(rawInchX / currentGridSize) * currentGridSize;
          inchY = Math.round(rawInchY / currentGridSize) * currentGridSize;

          // Clamp to field boundaries
          inchX = Math.max(0, Math.min(FIELD_SIZE, inchX));
          inchY = Math.max(0, Math.min(FIELD_SIZE, inchY));
        }

        // Handle path point dragging
        if (currentElem.startsWith("obstacle-")) {
          // Handle obstacle vertex dragging
          const parts = currentElem.split("-");
          const shapeIdx = Number(parts[1]);
          const vertexIdx = Number(parts[2]);

          shapes[shapeIdx].vertices[vertexIdx].x = inchX;
          shapes[shapeIdx].vertices[vertexIdx].y = inchY;
          shapes = [...shapes];
        } else if (currentElem.startsWith("second-point-")) {
          // Handle second path point dragging
          const parts = currentElem.split("-");
          const line = Number(parts[2]) - 1;
          const point = Number(parts[3]);

          if (line === -1) {
            // This is the second starting point
            if (secondStartPoint?.locked) return;
            if (secondStartPoint) {
              secondStartPoint.x = inchX;
              secondStartPoint.y = inchY;
            }
          } else if (secondLines[line]) {
            if (point === 0 && secondLines[line].endPoint) {
              secondLines[line].endPoint.x = inchX;
              secondLines[line].endPoint.y = inchY;
            } else {
              if (secondLines[line]?.locked) return;
              secondLines[line].controlPoints[point - 1].x = inchX;
              secondLines[line].controlPoints[point - 1].y = inchY;
            }
          }
          secondLines = [...secondLines];
        } else if (currentElem.startsWith("additional-path-")) {
          // Handle additional path point dragging
          const parts = currentElem.split("-");
          const pathIdx = Number(parts[2]);
          const line = Number(parts[4]) - 1;
          const point = Number(parts[5]);

          if (!additionalPaths[pathIdx]) return;

          if (line === -1) {
            // This is the starting point
            if (additionalPaths[pathIdx].startPoint) {
              additionalPaths[pathIdx].startPoint.x = inchX;
              additionalPaths[pathIdx].startPoint.y = inchY;
              additionalPaths = [...additionalPaths];
              // Auto-save changes to additional path files
              saveAdditionalPath(pathIdx).catch(err => console.error('Failed to auto-save additional path:', err));
            }
          } else if (additionalPaths[pathIdx].lines[line]) {
            if (point === 0 && additionalPaths[pathIdx].lines[line].endPoint) {
              additionalPaths[pathIdx].lines[line].endPoint.x = inchX;
              additionalPaths[pathIdx].lines[line].endPoint.y = inchY;
            } else if (additionalPaths[pathIdx].lines[line].controlPoints[point - 1]) {
              additionalPaths[pathIdx].lines[line].controlPoints[point - 1].x = inchX;
              additionalPaths[pathIdx].lines[line].controlPoints[point - 1].y = inchY;
            }
            additionalPaths = [...additionalPaths];
          }
          // Auto-save changes to additional path files
          saveAdditionalPath(pathIdx).catch(err => console.error('Failed to auto-save additional path:', err));
        } else {
          // Handle path point dragging
          const line = Number(currentElem.split("-")[1]) - 1;
          const point = Number(currentElem.split("-")[2]);

          if (line === -1) {
            // This is the starting point
            if (startPoint.locked || startPoint.poseVariableId) return;
            startPoint.x = inchX;
            startPoint.y = inchY;
          } else if (lines[line]) {
            if (point === 0 && lines[line].endPoint) {
              if (lines[line].endPoint.poseVariableId) return;
              lines[line].endPoint.x = inchX;
              lines[line].endPoint.y = inchY;
            } else {
              if (lines[line]?.locked) return;
              lines[line].controlPoints[point - 1].x = inchX;
              lines[line].controlPoints[point - 1].y = inchY;
            }
          }
        }
      } else {
        if (
          (elem?.id.startsWith("point") && !isLockedPathElem(elem.id)) ||
          elem?.id.startsWith("second-point") ||
          elem?.id.startsWith("additional-path-") ||
          elem?.id.startsWith("obstacle")
        ) {
          two.renderer.domElement.style.cursor = "pointer";
          currentElem = elem.id;
        } else {
          two.renderer.domElement.style.cursor = "auto";
          currentElem = null;
        }
      }
    });

    two.renderer.domElement.addEventListener("mousedown", (evt: MouseEvent) => {
      if (currentElem && isLockedPathElem(currentElem)) {
        isDown = false;
        return;
      }

      isDown = true;

      if (currentElem) {
        const rect = two.renderer.domElement.getBoundingClientRect();
        const mouseX = x.invert(evt.clientX - rect.left);
        const mouseY = y.invert(evt.clientY - rect.top);

        let objectX = 0;
        let objectY = 0;

        if (currentElem.startsWith("obstacle-")) {
          const parts = currentElem.split("-");
          const shapeIdx = Number(parts[1]);
          const vertexIdx = Number(parts[2]);
          if (shapes[shapeIdx]?.vertices[vertexIdx]) {
            objectX = shapes[shapeIdx].vertices[vertexIdx].x;
            objectY = shapes[shapeIdx].vertices[vertexIdx].y;
          }
        } else if (currentElem.startsWith("second-point-")) {
          const parts = currentElem.split("-");
          const line = Number(parts[2]) - 1;
          const point = Number(parts[3]);

          if (line === -1) {
            if (secondStartPoint) {
              objectX = secondStartPoint.x;
              objectY = secondStartPoint.y;
            }
          } else if (secondLines[line]) {
            if (point === 0 && secondLines[line].endPoint) {
              objectX = secondLines[line].endPoint.x;
              objectY = secondLines[line].endPoint.y;
            } else if (secondLines[line].controlPoints[point - 1]) {
              objectX = secondLines[line].controlPoints[point - 1].x;
              objectY = secondLines[line].controlPoints[point - 1].y;
            }
          }
        } else if (currentElem.startsWith("additional-path-")) {
          const parts = currentElem.split("-");
          const pathIdx = Number(parts[2]);
          const line = Number(parts[4]) - 1;
          const point = Number(parts[5]);

          if (additionalPaths[pathIdx]) {
            if (line === -1) {
              // Starting point
              if (additionalPaths[pathIdx].startPoint) {
                objectX = additionalPaths[pathIdx].startPoint.x;
                objectY = additionalPaths[pathIdx].startPoint.y;
              }
            } else if (additionalPaths[pathIdx].lines[line]) {
              if (point === 0 && additionalPaths[pathIdx].lines[line].endPoint) {
                objectX = additionalPaths[pathIdx].lines[line].endPoint.x;
                objectY = additionalPaths[pathIdx].lines[line].endPoint.y;
              } else if (additionalPaths[pathIdx].lines[line].controlPoints[point - 1]) {
                objectX = additionalPaths[pathIdx].lines[line].controlPoints[point - 1].x;
                objectY = additionalPaths[pathIdx].lines[line].controlPoints[point - 1].y;
              }
            }
          }
        } else {
          const line = Number(currentElem.split("-")[1]) - 1;
          const point = Number(currentElem.split("-")[2]);

          if (line === -1) {
            objectX = startPoint.x;
            objectY = startPoint.y;
          } else if (lines[line]) {
            if (point === 0 && lines[line].endPoint) {
              objectX = lines[line].endPoint.x;
              objectY = lines[line].endPoint.y;
            } else if (lines[line].controlPoints[point - 1]) {
              objectX = lines[line].controlPoints[point - 1].x;
              objectY = lines[line].controlPoints[point - 1].y;
            }
          }
        }

        dragOffset = {
          x: objectX - mouseX,
          y: objectY - mouseY,
        };
      }
    });

    two.renderer.domElement.addEventListener("mouseup", () => {
      isDown = false;
      dragOffset = { x: 0, y: 0 };
      recordChange();
    });

    // Double-click on the field to create a new path at that position
    two.renderer.domElement.addEventListener("dblclick", (evt: MouseEvent) => {
      // Ignore dblclicks on existing points/lines
      const elem = document.elementFromPoint(evt.clientX, evt.clientY);
      if (
        elem?.id &&
        (elem.id.startsWith("point") ||
          elem.id.startsWith("obstacle") ||
          elem.id.startsWith("line"))
      ) {
        return;
      }

      const rect = two.renderer.domElement.getBoundingClientRect();
      const rawInchX = x.invert(evt.clientX - rect.left);
      const rawInchY = y.invert(evt.clientY - rect.top);

      // Apply grid snapping if enabled
      const currentGridSize = $gridSize;
      const currentSnapToGrid = $snapToGrid;
      const currentShowGrid = $showGrid;

      let inchX = rawInchX;
      let inchY = rawInchY;

      if (currentSnapToGrid && currentShowGrid && currentGridSize > 0) {
        inchX = Math.round(rawInchX / currentGridSize) * currentGridSize;
        inchY = Math.round(rawInchY / currentGridSize) * currentGridSize;
      }

      // Clamp to field boundaries
      inchX = Math.max(0, Math.min(FIELD_SIZE, inchX));
      inchY = Math.max(0, Math.min(FIELD_SIZE, inchY));

      // Create a new line with endPoint at the clicked position
      const newLine: Line = {
        id: `line-${Math.random().toString(36).slice(2)}`,
        endPoint: {
          x: inchX,
          y: inchY,
          heading: "tangential",
          reverse: false,
        },
        controlPoints: [],
        color: getRandomColor(),
        speed: 1,
        locked: false,
        waitBeforeMs: 0,
        waitAfterMs: 0,
        waitBeforeName: "",
        waitAfterName: "",
      };

      lines = [...lines, newLine];
      sequence = [...sequence, { kind: "path", lineId: newLine.id! }];
      recordChange();
      two.update();
    });
  });
  document.addEventListener("keydown", function (evt) {
    if (evt.code === "Space" && document.activeElement === document.body) {
      if (playing) {
        pause();
      } else {
        play();
      }
    }
  });
  async function saveFile() {
    try {
      const content = JSON.stringify(
        {
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          poseVariables,
          pathVariables,
          numberVariables,
          settings,
          version: "1.2.1",
          timestamp: new Date().toISOString(),
        },
        null,
        2,
      );

      if ($currentFilePath) {
        await browserFileStore.writeFile($currentFilePath, content);
        isUnsaved.set(false);
        // Provide simple feedback
        alert(`Saved to project storage: ${$currentFilePath}`);
      } else {
        // No current project file selected — save into browser cache as a new file
        const defaultName = `path_${Date.now()}.pp`;
        await browserFileStore.writeFile(defaultName, content);
        currentFilePath.set(defaultName);
        isUnsaved.set(false);
        alert(`Saved to project storage as: ${defaultName}`);
      }
    } catch (err) {
      console.error("Failed to save project to storage:", err);
      alert("Failed to save project to browser storage.");
    }
  }

  async function loadFile(evt: Event) {
    const elem = evt.target as HTMLInputElement;
    const file = elem.files?.[0];

    if (!file) return;

    // Check if file is a .pp file
    if (!file.name.endsWith(".pp")) {
      alert("Please select a .pp file");
      // Reset the file input
      elem.value = "";
      return;
    }

	    // Parse and load the uploaded file, then cache it into the browser store.
	    loadTrajectoryFromFile(evt, async (data) => {
	      const loadedPoseVariables = normalizePoseVariables(data.poseVariables);
	      const loadedNumberVariables = normalizeNumberVariables(data.numberVariables);

      // Ensure startPoint has all required fields
      const loadedStartPoint = data.startPoint || {
        x: 72,
        y: 72,
        heading: "tangential",
        reverse: false,
      };

      // Normalize lines with all required fields
	      const normalizedLines = applyNumberVariablesToLines(
	        normalizeLines(data.lines || []),
	        loadedNumberVariables,
	      );
      const normalizedPath = applyPoseVariablesToPath(
        loadedStartPoint,
        normalizedLines,
        loadedPoseVariables,
      );
      startPoint = normalizedPath.startPoint;
	      lines = normalizedPath.lines;
	      poseVariables = loadedPoseVariables;
	      numberVariables = loadedNumberVariables;
	      pathVariables = normalizePathVariables(
	        data.pathVariables,
	        loadedPoseVariables,
	        loadedNumberVariables,
	      );

      // Derive sequence from data or create default
	      sequence = applyNumberVariablesToSequence(
	        (
	          data.sequence && data.sequence.length
	            ? data.sequence
	            : normalizedLines.map((ln) => ({
	                kind: "path",
	                lineId: ln.id!,
	              }))
	        ) as SequenceItem[],
	        loadedNumberVariables,
	      );
      pathChains = normalizePathChains(data.pathChains, normalizedPath.lines);

      // Load shapes with defaults
      shapes = data.shapes || [];

      // Load settings (including robot size) if present
      if (data.settings) {
        settings = { ...settings, ...data.settings };
        robotWidth = settings.rWidth;
        robotHeight = settings.rHeight;
      }

      isUnsaved.set(false);
      recordChange();

      // Cache the uploaded file into the browser-backed store for later access
      try {
        const content = JSON.stringify(data);
        await browserFileStore.writeFile(file.name, content);
        currentFilePath.set(file.name);
      } catch (err) {
        console.warn("Failed to cache uploaded file to store:", err);
      }
    });

    // Reset the file input
    elem.value = "";
  }

  // Electron file-copying logic removed — browser store and upload are used instead.

  // Helper function to load data into app state
	  function loadData(data: any) {
	    const loadedPoseVariables = normalizePoseVariables(data.poseVariables);
	    const loadedNumberVariables = normalizeNumberVariables(data.numberVariables);

    // Ensure startPoint has all required fields
    const loadedStartPoint = data.startPoint || {
      x: 72,
      y: 72,
      heading: "tangential",
      reverse: false,
    };

    // Normalize lines with all required fields
	    const normalizedLines = applyNumberVariablesToLines(
	      normalizeLines(data.lines || []),
	      loadedNumberVariables,
	    );
    const normalizedPath = applyPoseVariablesToPath(
      loadedStartPoint,
      normalizedLines,
      loadedPoseVariables,
    );
    startPoint = normalizedPath.startPoint;
	    lines = normalizedPath.lines;
	    poseVariables = loadedPoseVariables;
	    numberVariables = loadedNumberVariables;
	    pathVariables = normalizePathVariables(
	      data.pathVariables,
	      loadedPoseVariables,
	      loadedNumberVariables,
	    );

    // Derive sequence from data or create default
	    sequence = applyNumberVariablesToSequence(
	      (
	        data.sequence && data.sequence.length
	          ? data.sequence
	          : normalizedLines.map((ln) => ({
	              kind: "path",
	              lineId: ln.id!,
	            }))
	      ) as SequenceItem[],
	      loadedNumberVariables,
	    );
    pathChains = normalizePathChains(data.pathChains, normalizedPath.lines);

    // Load shapes with defaults
    shapes = data.shapes || [];

    // Load settings (including robot size) if present
    if (data.settings) {
      settings = { ...settings, ...data.settings };
      robotWidth = settings.rWidth;
      robotHeight = settings.rHeight;
    }

    isUnsaved.set(false);
    recordChange();
  }

  function toHeadingDegrees(point: Point, position: "start" | "end"): number {
    if (!point) return 0;
    if (point.heading === "linear") {
      return position === "start" ? (point.startDeg ?? 0) : (point.endDeg ?? 0);
    }
    if (point.heading === "constant") {
      return point.degrees ?? 0;
    }
    return 0;
  }

  function buildOptimizationPayload(lineIndex: number) {
    const line = lines[lineIndex];
    if (!line) throw new Error("Line not found");

    const startPt =
      lineIndex === 0 ? startPoint : lines[lineIndex - 1]?.endPoint;
    if (!startPt) throw new Error("Missing start point for optimization");

    const waypoints = [startPt, ...line.controlPoints, line.endPoint].map(
      (p) => [p.x, p.y],
    );

    return {
      waypoints,
      start_heading_degrees: toHeadingDegrees(startPt, "start"),
      end_heading_degrees: toHeadingDegrees(line.endPoint, "end"),
      x_velocity: settings.xVelocity,
      y_velocity: settings.yVelocity,
      angular_velocity: settings.aVelocity,
      friction_coefficient: settings.kFriction,
      robot_width: settings.rWidth,
      robot_height: settings.rHeight,
      min_coord_field: 0,
      max_coord_field: FIELD_SIZE,
      interpolation:
        line.endPoint.heading === "tangential"
          ? "tangent"
          : line.endPoint.heading === "constant"
            ? "constant"
            : "linear",
      obstacles: shapes.map((shape) => shape.vertices.map((v) => [v.x, v.y])),
    };
  }

  function sleep(ms: number) {
    return new Promise((res) => setTimeout(res, ms));
  }

  async function runOptimization(payload: any) {
    const response = await fetch(`${OPTIMIZER_BASE_URL}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });

    if (!response.ok) {
      const errorText = await response.text().catch(() => "");
      throw new Error(
        `Optimizer request failed (${response.status}): ${errorText || response.statusText}`,
      );
    }

    const data = await response.json();
    if (data?.status === "completed" && data.result) {
      return data.result;
    }
    if (data?.status === "error") {
      throw new Error(
        `Optimization failed: ${data.message || "Unknown error"}`,
      );
    }
    throw new Error("Unexpected API response format");
  }

  async function optimizeLine(
    lineId: string,
    targetControlPointIndex?: number,
  ) {
    const lineIndex = lines.findIndex((l) => l.id === lineId);
    if (lineIndex === -1) {
      alert("Could not find line to optimize.");
      return;
    }

    if (optimizingLineIds[lineId]) return;
    optimizingLineIds = { ...optimizingLineIds, [lineId]: true };

    try {
      const payload = buildOptimizationPayload(lineIndex);
      const result = await runOptimization(payload);

      const optimizedWaypoints = Array.isArray(result?.optimized_waypoints)
        ? result.optimized_waypoints
        : Array.isArray(result)
          ? result
          : null;

      if (!optimizedWaypoints || optimizedWaypoints.length < 2) {
        throw new Error("Unexpected optimizer response format.");
      }

      const interior = optimizedWaypoints
        .slice(1, optimizedWaypoints.length - 1)
        .map((p: number[]) => ({ x: p[0], y: p[1] }));

      const newLines = [...lines];
      const current = newLines[lineIndex];

      if (typeof targetControlPointIndex === "number") {
        // Only replace the targeted control point; keep others and endpoint untouched
        const replacement =
          interior[targetControlPointIndex] ?? interior[interior.length - 1];
        if (replacement) {
          const cps = [...current.controlPoints];
          if (cps[targetControlPointIndex]) {
            cps[targetControlPointIndex] = replacement;
            newLines[lineIndex] = {
              ...current,
              controlPoints: cps,
            };
            lines = normalizeLines(newLines);
            recordChange();
          }
        }
      } else {
        // Replace entire line (control points and endpoint)
        newLines[lineIndex] = {
          ...current,
          endPoint: {
            ...current.endPoint,
            x: optimizedWaypoints[optimizedWaypoints.length - 1][0],
            y: optimizedWaypoints[optimizedWaypoints.length - 1][1],
          },
          controlPoints: interior,
        };
        lines = normalizeLines(newLines);
        recordChange();
      }
    } catch (err) {
      console.error(err);
      alert((err as Error).message || "Optimization failed.");
    } finally {
      optimizingLineIds = { ...optimizingLineIds, [lineId]: false };
    }
  }

  async function optimizeAllLines() {
    if (optimizingAll) return;
    optimizingAll = true;
    try {
      for (const ln of lines) {
        if (!ln?.id) continue;
        await optimizeLine(ln.id);
      }
    } finally {
      optimizingAll = false;
    }
  }

  function loadRobot(evt: Event) {
    const file = (evt.currentTarget as HTMLInputElement).files?.[0];
    if (file) {
      loadRobotImage(file, () => updateRobotImageDisplay());
    }
  }

  function addNewLine() {
    lines = [
      ...lines,
      {
        id: `line-${Math.random().toString(36).slice(2)}`,
        endPoint: {
          x: _.random(36, 108),
          y: _.random(36, 108),
          heading: "tangential",
          reverse: true,
        } as Point,
        controlPoints: [],
        color: getRandomColor(),
        speed: 1,
        locked: false,
        waitBeforeMs: 0,
        waitAfterMs: 0,
        waitBeforeName: "",
        waitAfterName: "",
      },
    ];
    sequence = [
      ...sequence,
      { kind: "path", lineId: lines[lines.length - 1].id! },
    ];
    recordChange();
  }

  function addControlPoint() {
    if (lines.length > 0) {
      const lastLine = lines[lines.length - 1];
      lastLine.controlPoints.push({
        x: _.random(36, 108),
        y: _.random(36, 108),
      });
      recordChange();
    }
  }

  function removeControlPoint() {
    if (lines.length > 0) {
      const lastLine = lines[lines.length - 1];
      if (lastLine.controlPoints.length > 0) {
        lastLine.controlPoints.pop();
        recordChange();
      }
    }
  }

  // Keyboard shortcuts for quick path editing
  hotkeys("w", function (event, handler) {
    event.preventDefault();
    addNewLine();
  });
  hotkeys("a", function (event, handler) {
    event.preventDefault();
    addControlPoint();
    two.update();
  });
  hotkeys("s", function (event, handler) {
    event.preventDefault();
    removeControlPoint();
    two.update();
  });
  hotkeys("cmd+z, ctrl+z", function (event) {
    event.preventDefault();
    undoAction();
  });
  hotkeys("cmd+shift+z, ctrl+shift+z, ctrl+y", function (event) {
    event.preventDefault();
    redoAction();
  });
  function applyTheme(theme: "light" | "dark" | "auto") {
    let actualTheme = theme;
    if (theme === "auto") {
      // Check system preference
      if (
        window.matchMedia &&
        window.matchMedia("(prefers-color-scheme: dark)").matches
      ) {
        actualTheme = "dark";
      } else {
        actualTheme = "light";
      }
    }

    if (actualTheme === "dark") {
      document.documentElement.classList.add("dark");
    } else {
      document.documentElement.classList.remove("dark");
    }
  }

  // Watch for theme changes in settings
  $: if (settings) {
    applyTheme(settings.theme);
  }

  // Watch for system theme changes if auto mode is enabled
  let mediaQuery: MediaQueryList;
  onMount(() => {
    if (settings?.theme === "auto") {
      mediaQuery = window.matchMedia("(prefers-color-scheme: dark)");
      const handleSystemThemeChange = () => {
        if (settings.theme === "auto") {
          applyTheme("auto");
        }
      };
      mediaQuery.addEventListener("change", handleSystemThemeChange);

      return () => {
        mediaQuery.removeEventListener("change", handleSystemThemeChange);
      };
    }
  });

  // Auto-export for CI/testing: if the app is loaded with URL hash #export-gif-test, automatically run GIF export once mounted
  onMount(() => {
    if (
      typeof window !== "undefined" &&
      window.location &&
      window.location.hash === "#export-gif-test"
    ) {
      // Delay slightly to allow initial rendering and Two.js to initialize
      setTimeout(async () => {
        try {
          // auto GIF export removed (exportGif deleted)
          console.log("Auto GIF export skipped (exportGif removed)");
        } catch (err) {
          console.error("Auto GIF export failed:", err);
        }
      }, 1500);
    }

    // Handle save dialog event
    const handleSaveDialog = async (event: any) => {
      const { fileName } = event.detail;
      isSaving = true;
      try {
        // Create a full file path with .pp extension if not present
        const fullFileName = fileName.endsWith(".pp") ? fileName : fileName + ".pp";
        
        // Call the file manager's save function through the browser file store
        const fileData = JSON.stringify({
          startPoint,
          lines,
          shapes,
          sequence,
          pathChains,
          poseVariables,
          pathVariables,
          numberVariables,
          settings,
        });
        
        await browserFileStore.writeFile(fullFileName, fileData);
        currentFilePath.set(fullFileName);
        isUnsaved.set(false);
        
        // Show success feedback
        showSaveDialog = false;
      } catch (error) {
        console.error("Save failed:", error);
        alert("Failed to save file: " + (error instanceof Error ? error.message : String(error)));
      } finally {
        isSaving = false;
      }
    };

    window.addEventListener("save", handleSaveDialog);

    // Handle dual path save dialog event
    const handleDualPathSave = async (event: any) => {
      const { target } = event.detail;
      isSaving = true;
      try {
        if (target === "first" && $currentFilePath) {
          const fileData = JSON.stringify({
            startPoint,
            lines,
            shapes,
            sequence,
            pathChains,
            poseVariables,
            pathVariables,
            numberVariables,
            settings,
            version: "1.2.1",
            timestamp: new Date().toISOString(),
          });
          await browserFileStore.writeFile($currentFilePath, fileData);
          isUnsaved.set(false);
        } else if (target === "second" && $secondFilePath) {
          const fileData = JSON.stringify({
            startPoint: secondStartPoint,
            lines: secondLines,
            shapes: secondShapes,
            sequence: secondSequence,
            settings,
            version: "1.2.1",
            timestamp: new Date().toISOString(),
          });
          await browserFileStore.writeFile($secondFilePath, fileData);
        } else if (target === "both") {
          // Save first path
          if ($currentFilePath) {
            const fileData1 = JSON.stringify({
              startPoint,
              lines,
              shapes,
              sequence,
              pathChains,
              poseVariables,
              pathVariables,
              numberVariables,
              settings,
              version: "1.2.1",
              timestamp: new Date().toISOString(),
            });
            await browserFileStore.writeFile($currentFilePath, fileData1);
          }
          // Save second path
          if ($secondFilePath) {
            const fileData2 = JSON.stringify({
              startPoint: secondStartPoint,
              lines: secondLines,
              shapes: secondShapes,
              sequence: secondSequence,
              settings,
              version: "1.2.1",
              timestamp: new Date().toISOString(),
            });
            await browserFileStore.writeFile($secondFilePath, fileData2);
          }
          isUnsaved.set(false);
        }
        showDualPathSaveDialog = false;
      } catch (error) {
        console.error("Dual path save failed:", error);
        alert("Failed to save: " + (error instanceof Error ? error.message : String(error)));
      } finally {
        isSaving = false;
      }
    };

    window.addEventListener("saveDualPath", handleDualPathSave);

    return () => {
      window.removeEventListener("save", handleSaveDialog);
      window.removeEventListener("saveDualPath", handleDualPathSave);
    };
  });
</script>

<svelte:window
  on:mousemove={(e) => {
    // (debug window dragging removed)
  }}
  on:mouseup={() => {}}
/>

<Navbar
  bind:lines
  bind:startPoint
  bind:shapes
  bind:sequence
  bind:pathChains
  bind:poseVariables
  bind:pathVariables
  bind:numberVariables
  bind:secondStartPoint
  bind:secondLines
  bind:secondShapes
  bind:secondSequence
  bind:settings
  bind:robotWidth
  bind:robotHeight
  {percent}
  {saveProject}
  {saveFileAs}
  {loadFile}
  {undoAction}
  {redoAction}
  {recordChange}
  {canUndo}
  {canRedo}
  {optimizeAllLines}
  {optimizingAll}
  {twoElement}
  bind:playing
  {play}
  {pause}
  {exportPathAsGif}
/>

<SaveDialog
  bind:isOpen={showSaveDialog}
  bind:isSaving
  fileName={$currentFilePath?.split(/[\\/]/).pop()?.replace(/\.pp$/, "") || "my_path"}
/>

<DualPathSaveDialog bind:isOpen={showDualPathSaveDialog} />

<ProgressDialog
  bind:isOpen={exportingGif}
  progress={gifExportProgress}
  statusMessage={gifExportStatus}
  onCancel={() => {
    cancelGifExport = true;
    gifExportStatus = "Cancelling...";
  }}
/>

<!--   {saveFile} -->
<div
  class="w-screen h-screen pt-20 p-2 flex flex-row justify-center items-center gap-2"
>
  <div class="flex h-full justify-center items-center">
    <div
      bind:this={twoElement}
      bind:clientWidth={width}
      bind:clientHeight={height}
      class="h-full aspect-square rounded-lg shadow-md bg-neutral-50 dark:bg-neutral-900 relative overflow-clip"
      role="application"
      style="
    user-select: none;
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    -webkit-touch-callout: none;
    -webkit-tap-highlight-color: transparent;
    user-drag: none;
    -webkit-user-drag: none;
    -khtml-user-drag: none;
    -moz-user-drag: none;
    -ms-user-drag: none;
    -o-user-drag: none;
  "
      on:contextmenu={(e) => e.preventDefault()}
      on:dragstart={(e) => e.preventDefault()}
      on:selectstart={(e) => e.preventDefault()}
      tabindex="-1"
    >
      <img
        src={fieldMapSrc}
        alt="Field"
        class="absolute top-0 left-0 w-full h-full rounded-lg z-10"
        style="
    background: transparent; 
    pointer-events: none; 
    user-select: none; 
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    -webkit-touch-callout: none;
    -webkit-tap-highlight-color: transparent;
    user-drag: none;
    -webkit-user-drag: none;
    -moz-user-drag: none;
    -ms-user-drag: none;
    -o-user-drag: none;
  "
        draggable="false"
        on:error={(e) => {
          console.error("Failed to load field map:", settings.fieldMap);
          setImageFallback(e, "/fields/decode.webp");
        }}
        on:dragstart={(e) => e.preventDefault()}
        on:selectstart={(e) => e.preventDefault()}
      />
      <MathTools {x} {y} {twoElement} {robotXY} />
      {#if settings.showAutoCountdown && activeAutoTotalTime > 0}
        <div
          class="absolute top-3 left-1/2 -translate-x-1/2 z-[45] pointer-events-none w-[min(78%,28rem)]"
        >
          <div
            class="rounded-md border border-black/15 dark:border-white/15 bg-white/90 dark:bg-neutral-950/85 shadow-sm px-3 py-2"
            style={`--countdown-color: ${autoCountdownColor};`}
          >
            <div class="flex items-center justify-between gap-3 text-xs font-semibold text-neutral-800 dark:text-neutral-100">
              <span>Auto</span>
              <span style="color: var(--countdown-color);">
                {formatCountdown(autoCountdownRemaining)}
              </span>
            </div>
            <div class="mt-1 h-1.5 overflow-hidden rounded-full bg-neutral-200 dark:bg-neutral-800">
              <div
                class="h-full rounded-full"
                style={`width: ${Math.max(3, autoCountdownProgress * 100)}%; background: var(--countdown-color);`}
              />
            </div>
          </div>
        </div>
      {/if}

      {#each pathAnnotations as annotation (annotation.id)}
        <div
          class="absolute z-[42] pointer-events-none -translate-x-1/2 -translate-y-1/2 whitespace-nowrap rounded border border-black/10 dark:border-white/15 bg-white/90 dark:bg-neutral-950/85 px-2 py-0.5 text-[11px] font-semibold text-neutral-800 dark:text-neutral-100 shadow-sm"
          style={`left: ${annotation.x}px; top: ${annotation.y}px; border-left: 3px solid ${annotation.color || "#2563eb"};`}
        >
          {annotation.label}
        </div>
      {/each}

      {#each eventPins as marker (marker.id)}
        <div
          class="absolute z-[43] pointer-events-none flex -translate-x-1/2 -translate-y-full flex-col items-center"
          style={`left: ${marker.x}px; top: ${marker.y}px; color: ${marker.color || "#a855f7"};`}
        >
          <div
            class="rounded-md border border-current bg-white/95 dark:bg-neutral-950/90 px-1.5 py-0.5 text-[11px] font-semibold leading-tight text-neutral-900 dark:text-neutral-50 shadow-sm"
          >
            {marker.label}
          </div>
          <div
            class="h-3 w-3 rotate-45 border-b border-r border-current bg-white/95 dark:bg-neutral-950/90 -mt-1"
          />
          <div class="h-2 w-2 rounded-full bg-current shadow-sm" />
        </div>
      {/each}

      {#each swerveWheelOverlays as module (module.id)}
        <div
          class="absolute z-[44] pointer-events-none rounded-full border border-black/40 dark:border-white/50 shadow-sm"
          style={`left: ${module.x}px; top: ${module.y}px; width: ${module.length}px; height: ${module.thickness}px; margin-left: ${-module.length / 2}px; margin-top: ${-module.thickness / 2}px; transform: rotate(${module.angle}deg); opacity: ${module.opacity}; background: ${module.color};`}
        />
      {/each}

      <!-- Main robot: only show in normal mode -->
      {#if $activePaths.length === 0}
        <img
          src={settings.robotImage || "/robot.png"}
          alt="Robot"
          style={`position: absolute; top: ${robotXY.y}px;
left: ${robotXY.x}px; transform: translate(-50%, -50%) rotate(${robotHeading}deg); z-index: 20; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none;`}
          draggable="false"
          on:error={(e) => {
            console.error("Failed to load robot image:", settings.robotImage);
            setImageFallback(e, "/robot.png");
          }}
          on:dragstart={(e) => e.preventDefault()}
          on:selectstart={(e) => e.preventDefault()}
        />
        <!-- Heading arrow for main robot -->
        {#if settings.showHeadingArrow}
          <svg
            style={`position: absolute; top: ${robotXY.y}px; left: ${robotXY.x}px; z-index: 21; pointer-events: none; overflow: visible;`}
            width="1"
            height="1"
          >
            <defs>
              <marker
                id="arrowhead-main"
                markerWidth="10"
                markerHeight="10"
                refX="6.5"
                refY="3"
                orient="auto"
              >
                <polygon
                  points="0 0, 7 3, 0 6"
                  fill={settings.headingArrowColor || "#ffffff"}
                />
              </marker>
            </defs>
            <line
              x1="0"
              y1="0"
              x2="{(settings.headingArrowLength || 50) * Math.cos(-robotHeading * Math.PI / 180)}"
              y2="{(settings.headingArrowLength || 50) * -Math.sin(-robotHeading * Math.PI / 180)}"
              stroke={settings.headingArrowColor || "#ffffff"}
              stroke-width={settings.headingArrowThickness || 3}
              marker-end="url(#arrowhead-main)"
            />
          </svg>
        {/if}
      {/if}
      <!-- Second robot: only show in dual path mode (not multi-path mode) -->
      {#if $activePaths.length === 0 && $dualPathMode}
        <img
          src={settings.robotImage || "/robot.png"}
          alt="Robot 2"
          style={`position: absolute; top: ${secondRobotXY.y}px;
left: ${secondRobotXY.x}px; transform: translate(-50%, -50%) rotate(${secondRobotHeading}deg); z-index: 19; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none; opacity: 0.8;`}
          draggable="false"
          on:error={(e) => {
            console.error("Failed to load robot image:", settings.robotImage);
            setImageFallback(e, "/robot.png");
          }}
          on:dragstart={(e) => e.preventDefault()}
          on:selectstart={(e) => e.preventDefault()}
        />
        <!-- Heading arrow for second robot -->
        {#if settings.showHeadingArrow}
          <svg
            style={`position: absolute; top: ${secondRobotXY.y}px; left: ${secondRobotXY.x}px; z-index: 19; pointer-events: none; overflow: visible; opacity: 0.8;`}
            width="1"
            height="1"
          >
            <defs>
              <marker
                id="arrowhead-second"
                markerWidth="10"
                markerHeight="10"
                refX="6.5"
                refY="3"
                orient="auto"
              >
                <polygon
                  points="0 0, 7 3, 0 6"
                  fill={settings.headingArrowColor || "#ffffff"}
                />
              </marker>
            </defs>
            <line
              x1="0"
              y1="0"
              x2="{(settings.headingArrowLength || 50) * Math.cos(-secondRobotHeading * Math.PI / 180)}"
              y2="{(settings.headingArrowLength || 50) * -Math.sin(-secondRobotHeading * Math.PI / 180)}"
              stroke={settings.headingArrowColor || "#ffffff"}
              stroke-width={settings.headingArrowThickness || 3}
              marker-end="url(#arrowhead-second)"
            />
          </svg>
        {/if}
      {/if}
      <!-- Additional robots: only show in multi-path mode -->
      {#if $activePaths.length > 0}
        {#each additionalRobotStates as robotState, idx}
          <img
            src={settings.robotImage || "/robot.png"}
            alt="Robot {idx + 1}"
            style={`position: absolute; top: ${robotState.xy.y}px;
left: ${robotState.xy.x}px; transform: translate(-50%, -50%) rotate(${robotState.heading}deg); z-index: ${20 - idx}; width: ${x(robotWidth)}px; height: ${x(robotHeight)}px;user-select: none; -webkit-user-select: none; -moz-user-select: none;-ms-user-select: none;
pointer-events: none; opacity: ${1.0 - idx * 0.15};`}
            draggable="false"
            on:error={(e) => {
              console.error("Failed to load robot image:", settings.robotImage);
              setImageFallback(e, "/robot.png");
            }}
            on:dragstart={(e) => e.preventDefault()}
            on:selectstart={(e) => e.preventDefault()}
          />
          <!-- Heading arrow for additional robots -->
          {#if settings.showHeadingArrow}
            <svg
              style={`position: absolute; top: ${robotState.xy.y}px; left: ${robotState.xy.x}px; z-index: ${20 - idx}; pointer-events: none; overflow: visible; opacity: ${1.0 - idx * 0.15};`}
              width="1"
              height="1"
            >
              <defs>
                <marker
                  id="arrowhead-{idx}"
                  markerWidth="10"
                  markerHeight="10"
                  refX="6.5"
                  refY="3"
                  orient="auto"
                >
                  <polygon
                    points="0 0, 7 3, 0 6"
                    fill={settings.headingArrowColor || "#ffffff"}
                  />
                </marker>
              </defs>
              <line
                x1="0"
                y1="0"
                x2="{(settings.headingArrowLength || 50) * Math.cos(-robotState.heading * Math.PI / 180)}"
                y2="{(settings.headingArrowLength || 50) * -Math.sin(-robotState.heading * Math.PI / 180)}"
                stroke={settings.headingArrowColor || "#ffffff"}
                stroke-width={settings.headingArrowThickness || 3}
                marker-end="url(#arrowhead-{idx})"
              />
            </svg>
          {/if}
        {/each}
      {/if}
    </div>
  </div>
  <ControlTab
    bind:playing
    {play}
    {pause}
    bind:startPoint
    bind:lines
    bind:sequence
    bind:pathChains
    bind:poseVariables
    bind:pathVariables
    bind:numberVariables
    bind:robotWidth
    bind:robotHeight
    bind:settings
    bind:percent
    bind:robotXY
    bind:robotHeading
    bind:shapes
    {x}
    {y}
    {handleSeek}
    bind:loopAnimation
    {recordChange}
    {optimizeLine}
    {optimizingLineIds}
  />
</div>
