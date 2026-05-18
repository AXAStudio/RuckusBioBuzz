import type {
  Point,
  BasePoint,
  Line,
  Settings,
  TimePrediction,
  TimelineEvent,
  SequenceItem,
} from "../types";
import {
  getCurvePoint,
  getLineStartHeading,
  getLineEndHeading,
  getAngularDifference,
} from "./math";

export interface TravelLineTimingMeta {
  executionIndex: number;
  lineIndex: number;
  line: Line;
  startPoint: BasePoint;
  length: number;
  duration: number;
  startTime: number;
  endTime: number;
  maxVelocity: number;
  maxAcceleration: number;
  maxDeceleration: number;
  hasMotionProfile: boolean;
}

export interface EventTimingWindow {
  name: string;
  lineIndex: number;
  lineName: string;
  markerIndex: number;
  triggerType: "parametric" | "temporal" | "pose";
  startTime: number;
  endTime: number;
  durationMs: number;
  startPercent: number;
  endPercent: number;
  triggerPathPercent: number;
}

function clampNumber(value: number, min: number, max: number) {
  if (!Number.isFinite(value)) return min;
  return Math.max(min, Math.min(max, value));
}

/**
 * Calculate the length of a curve by sampling points
 */
export function calculateCurveLength(
  start: BasePoint,
  controlPoints: BasePoint[],
  end: BasePoint,
  samples: number = 100,
): number {
  let length = 0;
  let prevPoint: BasePoint = start;

  for (let i = 1; i <= samples; i++) {
    const t = i / samples;
    const point = getCurvePoint(t, [start, ...controlPoints, end]);
    const dx = point.x - prevPoint.x;
    const dy = point.y - prevPoint.y;
    length += Math.sqrt(dx * dx + dy * dy);
    prevPoint = point;
  }

  return length;
}

/**
 * Calculate time for a motion profile (trapezoidal or triangular)
 */
export function calculateMotionProfileTime(
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const deceleration = maxDec || maxAcc;

  const accDist = (maxVel * maxVel) / (2 * maxAcc);
  const decDist = (maxVel * maxVel) / (2 * deceleration);

  if (distance >= accDist + decDist) {
    const accTime = maxVel / maxAcc;
    const decTime = maxVel / deceleration;
    const constDist = distance - accDist - decDist;
    const constTime = constDist / maxVel;

    return accTime + constTime + decTime;
  } else {
    const vPeak = Math.sqrt(
      (2 * distance * maxAcc * deceleration) / (maxAcc + deceleration),
    );
    const accTime = vPeak / maxAcc;
    const decTime = vPeak / deceleration;

    return accTime + decTime;
  }
}

export function calculateMotionProfileDistanceAtTime(
  elapsedTime: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const t = Math.max(0, Number(elapsedTime) || 0);
  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    const accTime = velocity / acceleration;
    const decTime = velocity / deceleration;
    const constDist = totalDistance - accDist - decDist;
    const constTime = constDist / velocity;
    const totalTime = accTime + constTime + decTime;
    const clampedT = Math.min(t, totalTime);

    if (clampedT <= accTime) {
      return 0.5 * acceleration * clampedT * clampedT;
    }

    if (clampedT <= accTime + constTime) {
      return accDist + velocity * (clampedT - accTime);
    }

    const decT = clampedT - accTime - constTime;
    return Math.min(
      totalDistance,
      accDist + constDist + velocity * decT - 0.5 * deceleration * decT * decT,
    );
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const accTime = vPeak / acceleration;
  const decTime = vPeak / deceleration;
  const totalTime = accTime + decTime;
  const clampedT = Math.min(t, totalTime);

  if (clampedT <= accTime) {
    return 0.5 * acceleration * clampedT * clampedT;
  }

  const decT = clampedT - accTime;
  const peakDistance = 0.5 * acceleration * accTime * accTime;
  return Math.min(
    totalDistance,
    peakDistance + vPeak * decT - 0.5 * deceleration * decT * decT,
  );
}

export function calculateMotionProfileTimeAtDistance(
  distanceTraveled: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const traveled = clampNumber(Number(distanceTraveled) || 0, 0, totalDistance);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    const accTime = velocity / acceleration;
    const decTime = velocity / deceleration;
    const constDist = totalDistance - accDist - decDist;
    const constTime = constDist / velocity;
    const totalTime = accTime + constTime + decTime;

    if (traveled <= accDist) {
      return Math.sqrt((2 * traveled) / acceleration);
    }

    if (traveled <= accDist + constDist) {
      return accTime + (traveled - accDist) / velocity;
    }

    const remainingDistance = Math.max(0, totalDistance - traveled);
    return totalTime - Math.sqrt((2 * remainingDistance) / deceleration);
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const accTime = vPeak / acceleration;
  const decTime = vPeak / deceleration;
  const totalTime = accTime + decTime;
  const peakDistance = 0.5 * acceleration * accTime * accTime;

  if (traveled <= peakDistance) {
    return Math.sqrt((2 * traveled) / acceleration);
  }

  const remainingDistance = Math.max(0, totalDistance - traveled);
  return totalTime - Math.sqrt((2 * remainingDistance) / deceleration);
}

export function calculateMotionProfileVelocityAtDistance(
  distanceTraveled: number,
  distance: number,
  maxVel: number,
  maxAcc: number,
  maxDec?: number,
): number {
  const totalDistance = Math.max(0, Number(distance) || 0);
  const velocity = Math.max(0, Number(maxVel) || 0);
  const acceleration = Math.max(0, Number(maxAcc) || 0);
  const deceleration = Math.max(0, Number(maxDec ?? maxAcc) || 0);

  if (
    totalDistance <= 0 ||
    velocity <= 0 ||
    acceleration <= 0 ||
    deceleration <= 0
  ) {
    return 0;
  }

  const traveled = Math.max(0, Math.min(totalDistance, Number(distanceTraveled) || 0));
  const accDist = (velocity * velocity) / (2 * acceleration);
  const decDist = (velocity * velocity) / (2 * deceleration);

  if (totalDistance >= accDist + decDist) {
    if (traveled <= accDist) {
      return Math.sqrt(2 * acceleration * traveled);
    }

    if (traveled <= totalDistance - decDist) {
      return velocity;
    }

    return Math.sqrt(2 * deceleration * Math.max(0, totalDistance - traveled));
  }

  const vPeak = Math.sqrt(
    (2 * totalDistance * acceleration * deceleration) /
      (acceleration + deceleration),
  );
  const peakDistance = (vPeak * vPeak) / (2 * acceleration);

  if (traveled <= peakDistance) {
    return Math.sqrt(2 * acceleration * traveled);
  }

  return Math.sqrt(2 * deceleration * Math.max(0, totalDistance - traveled));
}

export function getPathSpeed(line: Line): number {
  const speed = Number(line.speed ?? 1);
  if (!Number.isFinite(speed)) return 1;
  return Math.max(0.05, Math.min(1, speed));
}

function getLineStartPointAtIndex(
  startPoint: Point,
  lines: Line[],
  lineIndex: number,
): BasePoint | null {
  if (lineIndex === 0) return startPoint;
  return lines[lineIndex - 1]?.endPoint || null;
}

function sampleLineCurve(
  sourceStartPoint: BasePoint,
  line: Line,
  samples = 100,
) {
  const sampleCount = Math.max(1, Math.round(samples));
  const curvePoints = [sourceStartPoint, ...line.controlPoints, line.endPoint];
  const sampledPoints = [
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

function getLineDistanceAtT(
  sourceStartPoint: BasePoint,
  line: Line,
  position: number,
): number {
  const targetT = clampNumber(position, 0, 1);
  const samples = sampleLineCurve(sourceStartPoint, line);
  for (let i = 1; i < samples.length; i++) {
    const previous = samples[i - 1];
    const current = samples[i];
    if (current.t < targetT) continue;

    const tSpan = current.t - previous.t;
    const ratio = tSpan <= 0 ? 0 : (targetT - previous.t) / tSpan;
    return previous.distance + (current.distance - previous.distance) * ratio;
  }
  return samples[samples.length - 1]?.distance || 0;
}

function getNearestDistanceOnLine(
  sourceStartPoint: BasePoint,
  line: Line,
  fieldPoint: BasePoint,
): number {
  const samples = sampleLineCurve(sourceStartPoint, line);
  let bestDistance = 0;
  let bestDistanceSq = Number.POSITIVE_INFINITY;

  samples.forEach((sample) => {
    const dx = sample.point.x - fieldPoint.x;
    const dy = sample.point.y - fieldPoint.y;
    const distanceSq = dx * dx + dy * dy;
    if (distanceSq < bestDistanceSq) {
      bestDistanceSq = distanceSq;
      bestDistance = sample.distance;
    }
  });

  return bestDistance;
}

function getLineMotionValues(line: Line, settings: Settings) {
  const pathSpeed = getPathSpeed(line);
  return {
    maxVelocity: Math.max(0, Number(settings.maxVelocity) || 0) * pathSpeed,
    maxAcceleration:
      Math.max(0, Number(settings.maxAcceleration) || 0) * pathSpeed,
    maxDeceleration:
      Math.max(
        0,
        Number(settings.maxDeceleration ?? settings.maxAcceleration) || 0,
      ) * pathSpeed,
  };
}

export function calculatePathTime(
  startPoint: Point,
  lines: Line[],
  settings: Settings,
  sequence?: SequenceItem[],
): TimePrediction {
  const msToSeconds = (value?: number | string) => {
    const numeric = Number(value);
    if (!Number.isFinite(numeric) || numeric <= 0) return 0;
    return numeric / 1000;
  };

  const useMotionProfile =
    settings.maxVelocity !== undefined &&
    settings.maxAcceleration !== undefined;

  const segmentLengths: number[] = [];
  const segmentTimes: number[] = [];
  const timeline: TimelineEvent[] = [];

  let currentTime = 0;
  let currentHeading = 0;

  // Initialize heading based on start point settings
  // Note: This initialization is technically overridden by the idx===0 check below
  // to ensure no initial turning, but kept for fallback logic.
  if (startPoint.heading === "linear") currentHeading = startPoint.startDeg;
  else if (startPoint.heading === "constant")
    currentHeading = startPoint.degrees;
  else if (startPoint.heading === "tangential") {
    if (lines.length > 0) {
      const firstLine = lines[0];
      const nextP =
        firstLine.controlPoints.length > 0
          ? firstLine.controlPoints[0]
          : firstLine.endPoint;
      const angle =
        Math.atan2(nextP.y - startPoint.y, nextP.x - startPoint.x) *
        (180 / Math.PI);
      currentHeading = startPoint.reverse ? angle + 180 : angle;
    } else {
      currentHeading = 0;
    }
  }

  // Create map and default sequence
  const lineById = new Map<string, Line>();
  lines.forEach((ln) => {
    if (!ln.id) ln.id = `line-${Math.random().toString(36).slice(2)}`;
    lineById.set(ln.id, ln);
  });

  const seq: SequenceItem[] =
    sequence && sequence.length
      ? sequence
      : lines.map((ln) => ({ kind: "path", lineId: ln.id! }));

  const expandedSeq: SequenceItem[] = [];
  seq.forEach((item) => {
    if (item.kind !== "repeat") {
      expandedSeq.push(item);
      return;
    }

    const repeatCount = Math.max(1, Math.min(20, Math.round(Number(item.count) || 1)));
    for (let repeatIndex = 0; repeatIndex < repeatCount; repeatIndex++) {
      (item.lineIds || []).forEach((lineId) => {
        expandedSeq.push({ kind: "path", lineId });
      });
    }
  });

  let lastPoint: Point = startPoint;

  expandedSeq.forEach((item, idx) => {
    if (item.kind === "wait" || item.kind === "event") {
      const waitSeconds = msToSeconds(item.durationMs);
      if (waitSeconds > 0) {
        timeline.push({
          type: "wait",
          name: item.name,
          duration: waitSeconds,
          startTime: currentTime,
          endTime: currentTime + waitSeconds,
          startHeading: currentHeading,
          targetHeading: currentHeading,
          atPoint: lastPoint,
        });
        currentTime += waitSeconds;
      }
      return;
    }

    if (item.kind !== "path") {
      return;
    }

    const line = lineById.get(item.lineId);
    if (!line || !line.endPoint) {
      // Skip missing or malformed lines in sequence
      return;
    }
    const prevPoint = lastPoint;

    // --- ROTATION CHECK ---
    const requiredStartHeading = getLineStartHeading(line, prevPoint);
    if (idx === 0) currentHeading = requiredStartHeading;
    const diff = Math.abs(
      getAngularDifference(currentHeading, requiredStartHeading),
    );
    if (diff > 0.1) {
      const diffRad = diff * (Math.PI / 180);
      const rotTime = diffRad / settings.aVelocity;
      timeline.push({
        type: "wait",
        duration: rotTime,
        startTime: currentTime,
        endTime: currentTime + rotTime,
        startHeading: currentHeading,
        targetHeading: requiredStartHeading,
        atPoint: prevPoint,
      });
      currentTime += rotTime;
      currentHeading = requiredStartHeading;
    }

    // --- TRAVEL ---
    const length = calculateCurveLength(
      prevPoint,
      line.controlPoints as any,
      line.endPoint as any,
    );
    segmentLengths.push(length);
    let segmentTime = 0;
    const pathSpeed = getPathSpeed(line);
    if (useMotionProfile) {
      segmentTime = calculateMotionProfileTime(
        length,
        settings.maxVelocity! * pathSpeed,
        settings.maxAcceleration! * pathSpeed,
        settings.maxDeceleration !== undefined
          ? settings.maxDeceleration * pathSpeed
          : undefined,
      );
    } else {
      const avgVelocity = ((settings.xVelocity + settings.yVelocity) / 2) * pathSpeed;
      segmentTime = length / avgVelocity;
    }
    segmentTimes.push(segmentTime);
    const lineIndex = lines.findIndex((l) => l.id === line.id);
    timeline.push({
      type: "travel",
      duration: segmentTime,
      startTime: currentTime,
      endTime: currentTime + segmentTime,
      lineIndex,
    });
    currentTime += segmentTime;
    currentHeading = getLineEndHeading(line, prevPoint);
    lastPoint = line.endPoint as Point;
  });

  const totalTime = currentTime;
  const totalDistance = segmentLengths.reduce((sum, length) => sum + length, 0);

  return {
    totalTime,
    segmentTimes,
    totalDistance,
    timeline,
  };
}

export function buildTravelLineTimingMetas(
  startPoint: Point,
  lines: Line[],
  timePrediction: TimePrediction | null | undefined,
  settings: Settings,
  sequence?: SequenceItem[],
): TravelLineTimingMeta[] {
  const lineById = new Map<string, Line>();
  lines.forEach((line, index) => {
    if (!line.id) {
      line.id = `line-${index + 1}`;
    }
    lineById.set(line.id, line);
  });

  const seq: SequenceItem[] =
    sequence && sequence.length
      ? sequence
      : lines.map((line) => ({ kind: "path", lineId: line.id! }));

  const expandedPaths: Array<{
    lineIndex: number;
    line: Line;
    startPoint: BasePoint;
  }> = [];

  let currentPoint: Point = startPoint;
  seq.forEach((item) => {
    if (item.kind === "repeat") {
      const repeatCount = Math.max(
        1,
        Math.min(20, Math.round(Number(item.count) || 1)),
      );
      for (let repeatIndex = 0; repeatIndex < repeatCount; repeatIndex++) {
        (item.lineIds || []).forEach((lineId) => {
          const line = lineById.get(lineId);
          if (!line || !line.endPoint) return;
          const lineIndex = lines.findIndex((candidate) => candidate.id === line.id);
          expandedPaths.push({
            lineIndex,
            line,
            startPoint: currentPoint,
          });
          currentPoint = line.endPoint as Point;
        });
      }
      return;
    }

    if (item.kind !== "path") return;
    const line = lineById.get(item.lineId);
    if (!line || !line.endPoint) return;
    const lineIndex = lines.findIndex((candidate) => candidate.id === line.id);
    expandedPaths.push({
      lineIndex,
      line,
      startPoint: currentPoint,
    });
    currentPoint = line.endPoint as Point;
  });

  const travelEvents = (timePrediction?.timeline || []).filter(
    (event) => event.type === "travel" && Number.isFinite(event.duration),
  );

  return expandedPaths.map((entry, executionIndex) => {
    const travelEvent = travelEvents[executionIndex];
    const length = calculateCurveLength(
      entry.startPoint,
      entry.line.controlPoints,
      entry.line.endPoint,
    );
    const motion = getLineMotionValues(entry.line, settings);
    const hasMotionProfile =
      length > 0 &&
      motion.maxVelocity > 0 &&
      motion.maxAcceleration > 0 &&
      motion.maxDeceleration > 0;
    const fallbackDuration = hasMotionProfile
      ? calculateMotionProfileTime(
          length,
          motion.maxVelocity,
          motion.maxAcceleration,
          motion.maxDeceleration,
        )
      : 0;
    const duration = Number(travelEvent?.duration) || fallbackDuration;
    const startTime = Number(travelEvent?.startTime) || 0;

    return {
      executionIndex,
      lineIndex: entry.lineIndex,
      line: entry.line,
      startPoint: entry.startPoint,
      length,
      duration,
      startTime,
      endTime: startTime + duration,
      maxVelocity: motion.maxVelocity,
      maxAcceleration: motion.maxAcceleration,
      maxDeceleration: motion.maxDeceleration,
      hasMotionProfile,
    };
  });
}

export function buildEventTimingWindows(
  startPoint: Point,
  lines: Line[],
  timePrediction: TimePrediction | null | undefined,
  settings: Settings,
  sequence?: SequenceItem[],
): EventTimingWindow[] {
  const metas = buildTravelLineTimingMetas(
    startPoint,
    lines,
    timePrediction,
    settings,
    sequence,
  );
  const totalTimelineTime = Math.max(
    timePrediction?.totalTime || 0,
    ...metas.map((meta) => meta.endTime),
  );
  const windows: EventTimingWindow[] = [];

  function distanceAtLocalTime(meta: TravelLineTimingMeta, localTime: number) {
    const clampedTime = clampNumber(localTime, 0, meta.duration);
    if (meta.hasMotionProfile) {
      return calculateMotionProfileDistanceAtTime(
        clampedTime,
        meta.length,
        meta.maxVelocity,
        meta.maxAcceleration,
        meta.maxDeceleration,
      );
    }
    return meta.length * (meta.duration > 0 ? clampedTime / meta.duration : 1);
  }

  metas.forEach((meta) => {
    (meta.line.eventMarkers || []).forEach((marker, markerIndex) => {
      const triggerType =
        marker.triggerType === "temporal" || marker.triggerType === "pose"
          ? marker.triggerType
          : "parametric";
      const position = clampNumber(Number(marker.position ?? 0.5), 0, 1);
      let triggerDistance = 0;
      let localTriggerTime = 0;

      if (
        triggerType === "pose" &&
        Number.isFinite(Number(marker.poseX)) &&
        Number.isFinite(Number(marker.poseY))
      ) {
        triggerDistance = getNearestDistanceOnLine(meta.startPoint, meta.line, {
          x: Number(marker.poseX),
          y: Number(marker.poseY),
        });
        localTriggerTime = meta.hasMotionProfile
          ? calculateMotionProfileTimeAtDistance(
              triggerDistance,
              meta.length,
              meta.maxVelocity,
              meta.maxAcceleration,
              meta.maxDeceleration,
            )
          : meta.duration * (meta.length > 0 ? triggerDistance / meta.length : 0);
      } else if (triggerType === "temporal") {
        localTriggerTime = clampNumber(
          Math.max(0, Number(marker.triggerMs ?? 0) || 0) / 1000,
          0,
          meta.duration,
        );
        triggerDistance = distanceAtLocalTime(meta, localTriggerTime);
      } else {
        triggerDistance = getLineDistanceAtT(meta.startPoint, meta.line, position);
        localTriggerTime = meta.hasMotionProfile
          ? calculateMotionProfileTimeAtDistance(
              triggerDistance,
              meta.length,
              meta.maxVelocity,
              meta.maxAcceleration,
              meta.maxDeceleration,
            )
          : meta.duration * (meta.length > 0 ? triggerDistance / meta.length : position);
      }

      const durationMs = Math.max(0, Math.round(Number(marker.durationMs ?? 0) || 0));
      const startTime = meta.startTime + localTriggerTime;
      const endTime =
        durationMs > 0
          ? startTime + durationMs / 1000
          : totalTimelineTime;

      windows.push({
        name: marker.name?.trim() || `Event ${markerIndex + 1}`,
        lineIndex: meta.lineIndex,
        lineName: meta.line.name?.trim() || `Path ${meta.lineIndex + 1}`,
        markerIndex,
        triggerType,
        startTime,
        endTime,
        durationMs,
        startPercent:
          totalTimelineTime > 0
            ? clampNumber((startTime / totalTimelineTime) * 100, 0, 100)
            : 0,
        endPercent:
          totalTimelineTime > 0
            ? clampNumber((endTime / totalTimelineTime) * 100, 0, 100)
            : 0,
        triggerPathPercent:
          meta.length > 0 ? clampNumber((triggerDistance / meta.length) * 100, 0, 100) : 0,
      });
    });
  });

  return windows.sort((a, b) => a.startTime - b.startTime);
}

export function formatTime(totalSeconds: number): string {
  if (totalSeconds <= 0) return "0.0s";
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  if (minutes > 0) {
    return `${minutes}:${seconds.toFixed(1).padStart(4, "0")}s`;
  }
  return `${seconds.toFixed(1)}s`;
}

export function getAnimationDuration(
  totalTime: number,
  speedFactor: number = 1.0,
): number {
  return (totalTime * 1000) / speedFactor;
}
