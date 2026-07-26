import {
  getCurvePoint,
  easeInOutQuad,
  goalHeadingAt,
  headingAlongPath,
  shortestRotation,
  shapedShortestRotation,
  radiansToDegrees,
} from "./math";
import { getRobotCorners } from "./geometry";
import { calculateMotionProfileDistanceAtTime } from "./timeCalculator";
import { curveParameterAtDistance, profileDistanceAtTime } from "./motionProfile";
import type { Point, Line, TimelineEvent, BasePoint, Settings } from "../types";
import type { ScaleLinear } from "d3";

/** How a path picks up its heading goal when the robot arrives facing elsewhere. */
export interface HeadingTransition {
  entryHeading: number;
  /** Fraction of the path spent catching up. */
  catchUp: number;
}

export interface RobotState {
  x: number;
  y: number;
  heading: number;
}

type AnimationState = {
  playing: boolean;
  percent: number;
  accumulatedSeconds: number;
  lastTimestamp: number | null;
  animationFrameId: number | null;
  totalDuration: number;
  loop: boolean;
};

function getPathSpeed(line: Line): number {
  const speed = Number(line.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

const curveLengthCache = new Map<string, number>();
const MAX_CURVE_LENGTH_CACHE_SIZE = 500;

function pointCacheKey(point: BasePoint): string {
  return `${Number(point.x).toFixed(4)},${Number(point.y).toFixed(4)}`;
}

function curveLengthCacheKey(
  start: BasePoint,
  controlPoints: BasePoint[],
  end: BasePoint,
  samples: number,
): string {
  return [samples, start, ...controlPoints, end]
    .map((value) =>
      typeof value === "number" ? String(value) : pointCacheKey(value),
    )
    .join("|");
}

function calculateCachedCurveLength(
  start: BasePoint,
  controlPoints: BasePoint[],
  end: BasePoint,
  samples = 100,
): number {
  const key = curveLengthCacheKey(start, controlPoints, end, samples);
  const cached = curveLengthCache.get(key);
  if (cached !== undefined) return cached;

  let length = 0;
  let prev = start;
  for (let i = 1; i <= samples; i++) {
    const t = i / samples;
    const point = getCurvePoint(t, [start, ...controlPoints, end]);
    const dx = point.x - prev.x;
    const dy = point.y - prev.y;
    length += Math.sqrt(dx * dx + dy * dy);
    prev = point;
  }

  curveLengthCache.set(key, length);
  if (curveLengthCache.size > MAX_CURVE_LENGTH_CACHE_SIZE) {
    const oldestKey = curveLengthCache.keys().next().value;
    if (oldestKey) curveLengthCache.delete(oldestKey);
  }

  return length;
}

/**
 * Calculate the robot position and heading based on the Timeline
 */
export function calculateRobotState(
  percent: number,
  timeline: TimelineEvent[],
  lines: Line[],
  startPoint: Point,
  settings: Settings,
  xScale: ScaleLinear<number, number, number>,
  yScale: ScaleLinear<number, number, number>,
): RobotState {
  if (!timeline || timeline.length === 0) {
    return { x: xScale(startPoint.x), y: yScale(startPoint.y), heading: 0 };
  }

  // Calculate current time in seconds based on percent (0-100)
  const totalDuration = timeline[timeline.length - 1].endTime;
  const currentSeconds = (percent / 100) * totalDuration;

  // Find the active event for this time
  const activeEvent =
    timeline.find(
      (e) => currentSeconds >= e.startTime && currentSeconds <= e.endTime,
    ) || timeline[timeline.length - 1];

  if (activeEvent.type === "wait") {
    // --- STATIONARY ROTATION ---
    const point = activeEvent.atPoint ?? startPoint;

    // Calculate progress (0.0 to 1.0) within this specific wait event. A
    // zero-length event would divide by zero, so treat it as finished.
    const eventProgress =
      activeEvent.duration > 0
        ? (currentSeconds - activeEvent.startTime) / activeEvent.duration
        : 1;
    const clampedProgress = Number.isFinite(eventProgress)
      ? Math.max(0, Math.min(1, eventProgress))
      : 1;

    // Interpolate heading smoothly
    const startHeading = Number(activeEvent.startHeading) || 0;
    const currentHeading = shortestRotation(
      startHeading,
      Number.isFinite(Number(activeEvent.targetHeading))
        ? Number(activeEvent.targetHeading)
        : startHeading,
      clampedProgress,
    );

    // Note: We use negative heading for visualizer (SVG/CSS rotation is CW, Math is usually CCW)
    return {
      x: xScale(point.x),
      y: yScale(point.y),
      heading: -currentHeading,
    };
  } else {
    // --- MOVEMENT TRAVEL ---
    const lineIdx = activeEvent.lineIndex ?? -1;
    const currentLine = lines[lineIdx];

    // A path can appear anywhere in the route — inside a repeat loop or an `if`
    // block — so its start comes from the timeline, not from the previous entry
    // in `lines`. The array-order fallback only covers older timelines.
    const prevPoint =
      activeEvent.startPoint ??
      (lineIdx <= 0 ? startPoint : lines[lineIdx - 1]?.endPoint);

    if (!currentLine?.endPoint || !prevPoint) {
      return { x: xScale(startPoint.x), y: yScale(startPoint.y), heading: 0 };
    }

    // Calculate progress (in seconds) within this specific travel event
    const timeIntoEvent = currentSeconds - activeEvent.startTime;

    // Determine fraction along the path using motion profile when available
    let linePercent = 0;
    const curvePoints = [prevPoint, ...currentLine.controlPoints, currentLine.endPoint];

    const segLength = calculateCachedCurveLength(
      prevPoint as BasePoint,
      currentLine.controlPoints as BasePoint[],
      currentLine.endPoint as BasePoint,
    );

    const clamp = (value: number, min: number, max: number) =>
      Number.isFinite(value) ? Math.max(min, Math.min(max, value)) : min;
    const chain = activeEvent.chain;
    const timeInEvent = clamp(timeIntoEvent, 0, activeEvent.duration);

    if (chain?.profile && chain.length > 0) {
      // The path is one slice of a PathChain, so the position comes from the
      // chain's single profile — the robot is still at speed at both ends of it
      // rather than braking to a stop on a waypoint it drives straight through.
      // Turning stretches the path's wall-clock duration, so step through it in
      // proportion rather than reading the chain clock directly.
      const progress =
        activeEvent.duration > 0 ? timeInEvent / activeEvent.duration : 1;
      const timeIntoChain =
        chain.enterTime + progress * chain.translationDuration;
      const distanceAlongChain = profileDistanceAtTime(
        chain.profile,
        timeIntoChain,
      );
      // Distance maps to a curve parameter through the path's own arc-length
      // table. Dividing by the length instead would advance the parameter at a
      // constant rate, which on a curve with bunched control points makes the
      // robot surge and dawdle across a path it should cross steadily.
      const pathLength = activeEvent.arcLengths
        ? activeEvent.arcLengths[activeEvent.arcLengths.length - 1]
        : segLength;
      const distanceIntoPath = clamp(
        distanceAlongChain - chain.offset,
        0,
        pathLength,
      );
      linePercent = activeEvent.arcLengths
        ? curveParameterAtDistance(activeEvent.arcLengths, distanceIntoPath)
        : pathLength > 0
          ? distanceIntoPath / pathLength
          : 0;
    } else if (
      settings &&
      settings.maxVelocity !== undefined &&
      settings.maxAcceleration !== undefined
    ) {
      const pathSpeed = getPathSpeed(currentLine);
      const distance = calculateMotionProfileDistanceAtTime(
        timeInEvent,
        segLength,
        settings.maxVelocity * pathSpeed,
        settings.maxAcceleration * pathSpeed,
        (settings.maxDeceleration ?? settings.maxAcceleration) * pathSpeed,
      );
      linePercent = segLength > 0 ? clamp(distance / segLength, 0, 1) : 0;
    } else {
      // Fallback: use easing over the event duration (preserves previous behaviour)
      const timeProgress =
        activeEvent.duration > 0 ? timeIntoEvent / activeEvent.duration : 1;
      linePercent = easeInOutQuad(clamp(timeProgress, 0, 1));
    }

    // Calculate Position
    const robotInchesXY = getCurvePoint(linePercent, curvePoints);

    const robotXY = { x: xScale(robotInchesXY.x), y: yScale(robotInchesXY.y) };

    // The path states a heading goal; the robot rotates toward it at the rate
    // the drivetrain allows rather than snapping to it where two paths meet.
    const goalHeading = goalHeadingAt(currentLine, curvePoints, linePercent);
    const robotHeading = -headingAlongPath(
      goalHeading,
      activeEvent.startHeading,
      linePercent,
      activeEvent.headingCatchUp,
    );

    return {
      x: robotXY.x,
      y: robotXY.y,
      heading: robotHeading,
    };
  }
}

/**
 * Create an animation controller for the robot simulation
 */
export function createAnimationController(
  totalDuration: number,
  onPercentChange: (percent: number) => void,
  onComplete?: () => void,
) {
  const state: AnimationState = {
    playing: false,
    percent: 0,
    accumulatedSeconds: 0, // total elapsed seconds (not tied to a single startTime)
    lastTimestamp: null, // last rAF timestamp seen while playing
    animationFrameId: null,
    totalDuration,
    loop: true,
  };

  let isExternalChange = false;

  function updatePercentFromAccumulated() {
    if (state.totalDuration > 0) {
      const rawPercent = (state.accumulatedSeconds / state.totalDuration) * 100;
      // clamp between 0 and 100 for non-looping; for looping we'll handle wrapping separately
      state.percent = Math.max(0, Math.min(100, rawPercent));
    } else {
      state.percent = 0;
    }
  }

  function animate(timestamp: number) {
    // If we aren't playing anymore, ensure we don't schedule anything further.
    if (!state.playing) {
      state.lastTimestamp = null;
      state.animationFrameId = null;
      return;
    }

    // Initialize lastTimestamp on first tick after play
    if (state.lastTimestamp === null) {
      state.lastTimestamp = timestamp;
      state.animationFrameId = requestAnimationFrame(animate);
      return;
    }

    // Compute delta time since last frame (in seconds)
    const deltaSeconds = (timestamp - state.lastTimestamp) / 1000;
    state.lastTimestamp = timestamp;

    // Advance accumulated time
    state.accumulatedSeconds += deltaSeconds;

    if (state.totalDuration > 0) {
      if (state.loop) {
        // For looping, wrap accumulatedSeconds so it doesn't grow unbounded.
        // Use modulo to allow continuous time even for large deltas.
        state.accumulatedSeconds =
          state.accumulatedSeconds % state.totalDuration;
        updatePercentFromAccumulated();
        if (!isExternalChange) onPercentChange(state.percent);
        // keep animating
        state.animationFrameId = requestAnimationFrame(animate);
      } else {
        // Not looping: clamp to duration and stop when done
        if (state.accumulatedSeconds >= state.totalDuration) {
          state.accumulatedSeconds = state.totalDuration;
          updatePercentFromAccumulated();
          if (!isExternalChange) onPercentChange(100);
          state.playing = false;
          state.lastTimestamp = null;
          if (state.animationFrameId) {
            cancelAnimationFrame(state.animationFrameId);
            state.animationFrameId = null;
          }
          if (onComplete) onComplete();
          return;
        } else {
          updatePercentFromAccumulated();
          if (!isExternalChange) onPercentChange(state.percent);
          state.animationFrameId = requestAnimationFrame(animate);
        }
      }
    } else {
      // duration is zero or invalid
      state.percent = 0;
      if (!isExternalChange) onPercentChange(state.percent);
      state.animationFrameId = requestAnimationFrame(animate);
    }
  }

  function play() {
    // If already playing, nothing to do
    if (state.playing) return;

    // If at the very end and not looping, reset to start so play restarts
    if (
      !state.loop &&
      state.totalDuration > 0 &&
      state.accumulatedSeconds >= state.totalDuration
    ) {
      state.accumulatedSeconds = 0;
      state.percent = 0;
      if (!isExternalChange) onPercentChange(0);
    }

    state.playing = true;
    // schedule the loop if not already scheduled
    if (state.animationFrameId === null) {
      state.lastTimestamp = null; // ensure animate initializes its timestamp properly
      state.animationFrameId = requestAnimationFrame(animate);
    }
  }

  function pause() {
    if (!state.playing) return;
    state.playing = false;
    // cancel outstanding rAF if any
    if (state.animationFrameId !== null) {
      cancelAnimationFrame(state.animationFrameId);
      state.animationFrameId = null;
    }
    state.lastTimestamp = null;
  }

  function reset() {
    state.accumulatedSeconds = 0;
    state.percent = 0;
    state.lastTimestamp = null;
    if (!isExternalChange) onPercentChange(0);
  }

  return {
    play,
    pause,
    reset() {
      pause();
      reset();
    },
    seekToPercent(targetPercent: number) {
      isExternalChange = true;
      const clamped = Math.max(0, Math.min(100, targetPercent));
      if (state.totalDuration > 0) {
        state.accumulatedSeconds = (clamped / 100) * state.totalDuration;
      } else {
        state.accumulatedSeconds = 0;
      }
      updatePercentFromAccumulated();
      onPercentChange(clamped);

      // If playing, we keep animating; lastTimestamp will sync on next tick
      // Clear the external flag immediately so normal anim ticks resume updating.
      // Use setTimeout(..., 0) so this call does not interrupt the current stack where this may be called
      setTimeout(() => {
        isExternalChange = false;
      }, 0);
    },
    setDuration(duration: number) {
      // If duration changes, keep current progress proportionally if possible
      const oldDuration = state.totalDuration;
      if (oldDuration > 0) {
        const progress = state.accumulatedSeconds / oldDuration;
        state.totalDuration = duration;
        state.accumulatedSeconds = progress * Math.max(0, duration);
      } else {
        state.totalDuration = duration;
        state.accumulatedSeconds = Math.min(
          state.accumulatedSeconds,
          Math.max(0, duration),
        );
      }
      updatePercentFromAccumulated();
      if (!isExternalChange) onPercentChange(state.percent);
    },
    setLoop(loop: boolean) {
      state.loop = loop;
    },
    isPlaying() {
      return state.playing;
    },
    getPercent() {
      updatePercentFromAccumulated();
      return state.percent;
    },
    getDuration() {
      return state.totalDuration;
    },
    isLooping() {
      return state.loop;
    },
  };
}

/**
 * Generate ghost path points that trace the robot's body along its path
 * Creates swept area by connecting consecutive robot corners properly
 * @param startPoint - The starting point of the path
 * @param lines - The path lines to trace
 * @param robotWidth - Robot width in inches
 * @param robotHeight - Robot height in inches
 * @param samples - Number of samples along the path (default 50)
 * @returns Array of points forming the boundary of the robot's swept path
 */
export function generateGhostPathPoints(
  startPoint: Point,
  lines: Line[],
  robotWidth: number,
  robotHeight: number,
  samples: number = 200, // Higher default for smoother turns
  /** Route start point per line id; falls back to plain array order. */
  lineStartPoints?: Map<string, BasePoint>,
  /** How each path picks up its heading goal, keyed by line id. */
  headingTransitions?: Map<string, HeadingTransition>,
): BasePoint[] {
  if (lines.length === 0) return [];

  // Collect robot states with center, heading, and offset rails
  const robotStates: Array<{
    center: BasePoint;
    heading: number;
    left: BasePoint;
    right: BasePoint;
  }> = [];

  let currentLineStart: BasePoint = startPoint;

  // For each line segment
  for (let lineIdx = 0; lineIdx < lines.length; lineIdx++) {
    const line = lines[lineIdx];
    const routeStart = line.id ? lineStartPoints?.get(line.id) : undefined;
    if (lineStartPoints && !routeStart) continue; // Not part of the route.
    if (routeStart) currentLineStart = routeStart;
    const curvePoints = [
      currentLineStart,
      ...line.controlPoints,
      line.endPoint,
    ];

    // Sample along this line segment with a minimum to better capture curves
    const samplesPerLine = Math.max(10, Math.ceil(samples / lines.length));
    for (let i = 0; i <= samplesPerLine; i++) {
      const t = i / samplesPerLine;
      const robotPosInches = getCurvePoint(t, curvePoints);

      // Same heading model as the animated robot, so the swept area shows the
      // rotation the robot actually makes across a path boundary.
      const transition = headingTransitions?.get(line.id || "");
      const heading = headingAlongPath(
        goalHeadingAt(line, curvePoints, t),
        transition?.entryHeading,
        t,
        transition?.catchUp,
      );

      // Build left/right rails directly from center + normal offsets
      const headingRad = (heading * Math.PI) / 180;
      const nx = -Math.sin(headingRad);
      const ny = Math.cos(headingRad);
      const halfW = robotWidth / 2;

      const leftPoint = {
        x: robotPosInches.x + nx * halfW,
        y: robotPosInches.y + ny * halfW,
      };
      const rightPoint = {
        x: robotPosInches.x - nx * halfW,
        y: robotPosInches.y - ny * halfW,
      };

      robotStates.push({
        center: { x: robotPosInches.x, y: robotPosInches.y },
        heading,
        left: leftPoint,
        right: rightPoint,
      });
    }

    currentLineStart = line.endPoint;
  }

  if (robotStates.length === 0) return [];
  if (robotStates.length === 1) {
    // Single pose: return rectangle corners
    const single = robotStates[0];
    const heading = single.heading;
    const corners = getRobotCorners(
      single.center.x,
      single.center.y,
      heading,
      robotWidth,
      robotHeight,
    );
    return corners;
  }

  // Build swept boundary by tracing left rail forward and right rail backward
  const leftRail: BasePoint[] = [];
  const rightRail: BasePoint[] = [];

  for (let i = 0; i < robotStates.length; i++) {
    leftRail.push(robotStates[i].left);
  }

  for (let i = 0; i < robotStates.length; i++) {
    rightRail.push(robotStates[i].right);
  }

  // Trace one side forward, then the other side backward. Closing the polygon
  // provides the end/start bridges without crossing the boundary on tight turns.
  const boundary: BasePoint[] = [...leftRail, ...rightRail.reverse()];

  // Remove consecutive duplicates and ensure closure
  const result: BasePoint[] = [];
  const threshold = 1e-4;

  for (let i = 0; i < boundary.length; i++) {
    const curr = boundary[i];
    const prev = result[result.length - 1];

    if (
      !prev ||
      Math.abs(curr.x - prev.x) > threshold ||
      Math.abs(curr.y - prev.y) > threshold
    ) {
      result.push(curr);
    }
  }

  if (result.length >= 3) {
    const first = result[0];
    const last = result[result.length - 1];
    if (
      Math.abs(first.x - last.x) > threshold ||
      Math.abs(first.y - last.y) > threshold
    ) {
      result.push({ ...first });
    }
  }

  return result.length >= 3 ? result : [];
}

/**
 * Generate onion layer robot bodies at regular intervals along the path
 * Returns an array of robot states (position, heading, and corner points) for drawing
 * @param startPoint - The starting point of the path
 * @param lines - The path lines to trace
 * @param robotWidth - Robot width in inches
 * @param robotHeight - Robot height in inches
 * @param spacing - Distance in inches between each robot trace (default 6)
 * @returns Array of robot states with corner points for rendering
 */
export function generateOnionLayers(
  startPoint: Point,
  lines: Line[],
  robotWidth: number,
  robotHeight: number,
  spacing: number = 6,
  /** Route start point per line id; falls back to plain array order. */
  lineStartPoints?: Map<string, BasePoint>,
  /** How each path picks up its heading goal, keyed by line id. */
  headingTransitions?: Map<string, HeadingTransition>,
): Array<{ x: number; y: number; heading: number; corners: BasePoint[]; lineIndex: number; t: number }> {
  if (lines.length === 0) return [];

  // A non-positive spacing would never advance `nextLayerDistance` and would
  // spin forever in the sampling loop below, hanging the page.
  const layerSpacing = Number(spacing) > 0 ? Number(spacing) : 6;

  const layers: Array<{
    x: number;
    y: number;
    heading: number;
    corners: BasePoint[];
    lineIndex: number;
    t: number;
  }> = [];

  /** The lines that are on the route, with the point each one starts at. */
  const routeLines: Array<{ line: Line; lineIndex: number; start: BasePoint }> = [];
  let currentLineStart: BasePoint = startPoint;

  lines.forEach((line, lineIndex) => {
    if (!line?.endPoint) return;
    const routeStart = line.id ? lineStartPoints?.get(line.id) : undefined;
    if (lineStartPoints && !routeStart) return; // Not part of the route.
    if (routeStart) currentLineStart = routeStart;
    routeLines.push({ line, lineIndex, start: currentLineStart });
    currentLineStart = line.endPoint;
  });

  // Calculate total path length
  let totalLength = 0;

  routeLines.forEach(({ line, start }) => {
    const curvePoints = [start, ...line.controlPoints, line.endPoint];

    // Approximate line length by sampling
    const samples = 100;
    let prevPos = curvePoints[0];

    for (let i = 1; i <= samples; i++) {
      const t = i / samples;
      const pos = getCurvePoint(t, curvePoints);
      const dx = pos.x - prevPos.x;
      const dy = pos.y - prevPos.y;
      totalLength += Math.sqrt(dx * dx + dy * dy);
      prevPos = pos;
    }
  });

  // Sample robot positions at regular intervals
  let accumulatedLength = 0;
  let nextLayerDistance = layerSpacing;

  for (const { line, lineIndex: li, start } of routeLines) {
    const curvePoints = [start, ...line.controlPoints, line.endPoint];
    const samples = 100;
    let prevPos = curvePoints[0];
    let prevT = 0;

    for (let i = 1; i <= samples; i++) {
      const t = i / samples;
      const pos = getCurvePoint(t, curvePoints);
      const dx = pos.x - prevPos.x;
      const dy = pos.y - prevPos.y;
      const segmentLength = Math.sqrt(dx * dx + dy * dy);

      accumulatedLength += segmentLength;

      // Check if we've reached the next layer position
      while (
        accumulatedLength >= nextLayerDistance &&
        nextLayerDistance <= totalLength
      ) {
        // Interpolate exact position for this layer
        const overshoot = accumulatedLength - nextLayerDistance;
        const interpolationT =
          segmentLength > 0 ? 1 - overshoot / segmentLength : 1;
        const layerT = prevT + (t - prevT) * interpolationT;
        const robotPosInches = getCurvePoint(layerT, curvePoints);

        // Same heading model as the animated robot, so consecutive bodies do
        // not jump where two paths meet.
        const transition = headingTransitions?.get(line.id || "");
        const heading = headingAlongPath(
          goalHeadingAt(line, curvePoints, layerT),
          transition?.entryHeading,
          layerT,
          transition?.catchUp,
        );

        // Get robot corners for this position
        const corners = getRobotCorners(
          robotPosInches.x,
          robotPosInches.y,
          heading,
          robotWidth,
          robotHeight,
        );

        layers.push({
          x: robotPosInches.x,
          y: robotPosInches.y,
          heading: heading,
          corners: corners,
          lineIndex: li,
          t: layerT,
        });

        nextLayerDistance += layerSpacing;
      }

      prevPos = pos;
      prevT = t;
    }
  }

  return layers;
}
