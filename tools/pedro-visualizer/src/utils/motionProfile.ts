import type { BasePoint, MotionProfile, ProfilePoint } from "../types";

/**
 * Velocity profiling along a path.
 *
 * A trapezoid over the straight-line distance is only right for a straight
 * path. On a curve the robot is also limited sideways: holding radius `r` at
 * speed `v` needs a centripetal acceleration of `v² / r`, and past what the
 * wheels can give the follower simply cannot hold the line. So the speed is
 * capped curve by curve and then smoothed with the usual forward/backward
 * passes, which is what turns a corner into a real slow-down instead of a
 * number the robot could never drive.
 */

export interface ProfileLimits {
  maxVelocity: number;
  maxAcceleration: number;
  maxDeceleration: number;
  /** Sideways acceleration the wheels can hold before losing the line. */
  maxLateralAcceleration: number;
  /** Fastest the robot can spin, in rad/s. */
  maxAngularVelocity: number;
  /**
   * How much turning eats into driving speed, 0..1. At 1 the drivetrain is
   * saturated, so the time to turn and the time to drive add up; at 0 there is
   * headroom and they overlap for free.
   */
  turnCoupling: number;
  /**
   * Tightest radius worth modelling. Sampled geometry reports a near-zero
   * radius at a hard join between two paths, which would demand a full stop the
   * follower never actually makes — it rounds the corner. Half the robot is a
   * physical floor: inside that the robot is pivoting, and the turn model
   * already covers the cost of that.
   */
  minRadius: number;
}

const EMPTY: MotionProfile = {
  length: 0,
  totalTime: 0,
  distances: [0],
  velocities: [0],
  times: [0],
};

/** Menger curvature of three points: 4·area ÷ (|ab|·|bc|·|ca|). */
export function curvatureThrough(
  a: BasePoint,
  b: BasePoint,
  c: BasePoint,
): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const bcx = c.x - b.x;
  const bcy = c.y - b.y;
  const cax = a.x - c.x;
  const cay = a.y - c.y;

  const ab = Math.hypot(abx, aby);
  const bc = Math.hypot(bcx, bcy);
  const ca = Math.hypot(cax, cay);
  if (ab <= 0 || bc <= 0 || ca <= 0) return 0;

  // Twice the signed triangle area.
  const cross = abx * bcy - aby * bcx;
  const curvature = (2 * Math.abs(cross)) / (ab * bc * ca);
  return Number.isFinite(curvature) ? curvature : 0;
}

/**
 * Samples a polyline into distance/curvature pairs, which is the only shape the
 * profiler needs to know about.
 */
const shortestTurn = (from: number, to: number): number => {
  let delta = (to - from) % 360;
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;
  return delta;
};

/**
 * Turns a polyline into the distance/curvature/heading-rate samples the
 * profiler works from.
 *
 * `headings` are the heading goals in degrees at each point. Where the goal
 * moves quickly per inch travelled, the robot has to spin quickly, and spinning
 * competes with driving for the same motor power — so the rate is what the
 * speed cap is built from. It is smoothed over `blendDistance` because a robot
 * blends a heading change over roughly its own length rather than snapping to
 * it at a point.
 */
export function profilePointsFromPolyline(
  points: BasePoint[],
  headings?: number[],
  blendDistance = 0,
  /**
   * Distance at each point. Supply it when the caller already knows where the
   * samples sit: measuring the chords instead would put the profile on a
   * slightly shorter ruler than everything else uses, and mixing the two makes
   * the robot's position drift and then snap back at each path boundary.
   */
  distances?: number[],
): ProfilePoint[] {
  if (points.length === 0) return [];

  const result: ProfilePoint[] = [{ distance: 0, curvature: 0, headingRate: 0 }];
  const steps: number[] = [0];
  let distance = 0;

  for (let i = 1; i < points.length; i++) {
    const previous = points[i - 1];
    const current = points[i];
    const step = distances
      ? Math.max(0, distances[i] - distances[i - 1])
      : Math.hypot(current.x - previous.x, current.y - previous.y);
    distance += step;
    steps.push(step);

    const next = points[i + 1];
    let headingRate = 0;
    if (headings && headings.length > i && step > 0) {
      headingRate =
        Math.abs(shortestTurn(headings[i - 1], headings[i])) * (Math.PI / 180) / step;
    }

    result.push({
      distance,
      curvature: next ? curvatureThrough(previous, current, next) : 0,
      headingRate,
    });
  }

  if (headings && blendDistance > 0 && result.length > 2) {
    // Spread each reading over the blend distance: a heading goal that jumps
    // between two paths would otherwise read as an infinite spin rate at one
    // sample and demand a full stop there.
    const averageStep =
      distance > 0 ? distance / Math.max(1, result.length - 1) : blendDistance;
    const half = Math.max(1, Math.round(blendDistance / Math.max(averageStep, 1e-6) / 2));
    const smoothed = new Array<number>(result.length);

    for (let i = 0; i < result.length; i++) {
      let weighted = 0;
      let span = 0;
      for (let k = i - half; k <= i + half; k++) {
        if (k < 1 || k >= result.length) continue;
        weighted += result[k].headingRate * steps[k];
        span += steps[k];
      }
      smoothed[i] = span > 0 ? weighted / span : 0;
    }
    for (let i = 0; i < result.length; i++) result[i].headingRate = smoothed[i];
  }

  return result;
}

/**
 * Builds the speed curve for one continuous move that starts and ends at rest.
 *
 * Forward pass: never accelerate harder than the drivetrain can.
 * Backward pass: always leave enough room to brake for what is coming.
 */
export function buildMotionProfile(
  points: ProfilePoint[],
  limits: ProfileLimits,
): MotionProfile {
  const maxVelocity = Math.max(0, Number(limits.maxVelocity) || 0);
  const maxAcceleration = Math.max(0, Number(limits.maxAcceleration) || 0);
  const maxDeceleration = Math.max(0, Number(limits.maxDeceleration) || 0);
  const maxLateral = Math.max(0, Number(limits.maxLateralAcceleration) || 0);
  const minRadius = Math.max(1e-3, Number(limits.minRadius) || 1e-3);

  if (
    points.length < 2 ||
    maxVelocity <= 0 ||
    maxAcceleration <= 0 ||
    maxDeceleration <= 0
  ) {
    const length = points.length ? points[points.length - 1].distance : 0;
    return { ...EMPTY, length, distances: [0, length], velocities: [0, 0], times: [0, 0] };
  }

  const count = points.length;
  const distances = points.map((point) => point.distance);
  const length = distances[count - 1];

  // Speed ceiling at each sample: the drivetrain's own cap, and whatever the
  // curve allows.
  const maxAngular = Math.max(0, Number(limits.maxAngularVelocity) || 0);
  const coupling = Math.max(0, Math.min(1, Number(limits.turnCoupling) || 0));

  const velocities = points.map((point) => {
    let cap = maxVelocity;

    if (maxLateral > 0) {
      const radius = point.curvature > 0 ? 1 / point.curvature : Infinity;
      const usableRadius = Math.max(minRadius, radius);
      if (Number.isFinite(usableRadius)) {
        cap = Math.min(cap, Math.sqrt(maxLateral * usableRadius));
      }
    }

    if (point.headingRate > 0 && maxAngular > 0) {
      // Spinning at `headingRate` radians per inch means driving an inch costs
      // that much rotation, so the robot can go no faster than the rate it can
      // spin allows.
      cap = Math.min(cap, maxAngular / point.headingRate);

      // And rotation draws on the same power as driving: at full coupling the
      // per-inch costs add, which is the same trade as doing them one after the
      // other, while at zero they overlap and only the hard limit above counts.
      if (coupling > 0) {
        const perInch = 1 / maxVelocity + (coupling * point.headingRate) / maxAngular;
        cap = Math.min(cap, 1 / perInch);
      }
    }

    return cap;
  });

  // A chain starts and ends at rest — that is what makes it a chain.
  velocities[0] = 0;
  velocities[count - 1] = 0;

  for (let i = 1; i < count; i++) {
    const step = distances[i] - distances[i - 1];
    if (step <= 0) {
      velocities[i] = Math.min(velocities[i], velocities[i - 1]);
      continue;
    }
    const reachable = Math.sqrt(
      velocities[i - 1] * velocities[i - 1] + 2 * maxAcceleration * step,
    );
    velocities[i] = Math.min(velocities[i], reachable);
  }

  for (let i = count - 2; i >= 0; i--) {
    const step = distances[i + 1] - distances[i];
    if (step <= 0) {
      velocities[i] = Math.min(velocities[i], velocities[i + 1]);
      continue;
    }
    const stoppable = Math.sqrt(
      velocities[i + 1] * velocities[i + 1] + 2 * maxDeceleration * step,
    );
    velocities[i] = Math.min(velocities[i], stoppable);
  }

  // Integrate: over a short step the speed is close enough to linear that the
  // average of the endpoints is the right divisor.
  const times: number[] = new Array(count);
  times[0] = 0;
  for (let i = 1; i < count; i++) {
    const step = distances[i] - distances[i - 1];
    const averageSpeed = (velocities[i - 1] + velocities[i]) / 2;
    times[i] = times[i - 1] + (step > 0 && averageSpeed > 0 ? step / averageSpeed : 0);
  }

  return {
    length,
    totalTime: times[count - 1],
    distances,
    velocities,
    times,
  };
}

/** Index of the last sample at or before `value` in an ascending array. */
function lowerBound(values: number[], value: number): number {
  let low = 0;
  let high = values.length - 1;

  while (low < high) {
    const middle = (low + high + 1) >> 1;
    if (values[middle] <= value) low = middle;
    else high = middle - 1;
  }

  return low;
}

function interpolate(
  from: number[],
  to: number[],
  value: number,
): number {
  if (from.length === 0) return 0;
  if (from.length === 1) return to[0];

  const clamped = Math.max(from[0], Math.min(from[from.length - 1], value));
  const index = Math.min(lowerBound(from, clamped), from.length - 2);
  const span = from[index + 1] - from[index];
  if (span <= 0) return to[index];

  const ratio = (clamped - from[index]) / span;
  return to[index] + (to[index + 1] - to[index]) * ratio;
}

/**
 * The curve parameter at a given distance along a path.
 *
 * A Bezier's parameter is not proportional to its arc length — with control
 * points bunched to one side the curve covers far more ground per unit of `t`
 * in some places than others. Advancing `t` in proportion to distance therefore
 * makes the robot surge and dawdle along a path it should cross at a steady
 * speed. `arcLengths` holds the distance reached at evenly spaced `t` values, so
 * inverting it recovers the parameter that actually corresponds to a distance.
 */
export function curveParameterAtDistance(
  arcLengths: Float32Array | number[],
  distance: number,
): number {
  const last = arcLengths.length - 1;
  if (last < 1) return 0;

  const total = arcLengths[last];
  if (!(total > 0)) return 0;

  const target = Math.max(0, Math.min(total, distance));
  let low = 0;
  let high = last;
  while (low < high) {
    const middle = (low + high + 1) >> 1;
    if (arcLengths[middle] <= target) low = middle;
    else high = middle - 1;
  }
  if (low >= last) return 1;

  const span = arcLengths[low + 1] - arcLengths[low];
  const withinStep = span > 0 ? (target - arcLengths[low]) / span : 0;
  return (low + withinStep) / last;
}

/**
 * Within one interval the robot holds a constant acceleration, so distance
 * moves as a quadratic in time rather than a straight line. Interpolating
 * linearly instead would hold the speed flat across each interval and step it
 * at every boundary — smooth acceleration chopped into stairs, which is what a
 * viewer reads as stutter.
 */
function intervalAcceleration(
  v0: number,
  v1: number,
  span: number,
): number {
  if (span <= 0) return 0;
  return (v1 * v1 - v0 * v0) / (2 * span);
}

export function profileTimeAtDistance(
  profile: MotionProfile,
  distance: number,
): number {
  const { distances, times, velocities } = profile;
  const last = distances.length - 1;
  if (last < 1) return 0;

  const target = Math.max(distances[0], Math.min(distances[last], distance));
  const index = Math.min(lowerBound(distances, target), last - 1);
  const span = distances[index + 1] - distances[index];
  if (span <= 0) return times[index];

  const into = target - distances[index];
  const v0 = velocities[index];
  const acceleration = intervalAcceleration(v0, velocities[index + 1], span);
  const reached = Math.sqrt(Math.max(0, v0 * v0 + 2 * acceleration * into));

  if (Math.abs(acceleration) < 1e-9) {
    return times[index] + (v0 > 0 ? into / v0 : 0);
  }
  return times[index] + (reached - v0) / acceleration;
}

export function profileDistanceAtTime(
  profile: MotionProfile,
  time: number,
): number {
  const { distances, times, velocities } = profile;
  const last = times.length - 1;
  if (last < 1) return 0;

  const target = Math.max(times[0], Math.min(times[last], time));
  const index = Math.min(lowerBound(times, target), last - 1);
  const span = distances[index + 1] - distances[index];
  if (span <= 0) return distances[index];

  const elapsed = target - times[index];
  const v0 = velocities[index];
  const acceleration = intervalAcceleration(v0, velocities[index + 1], span);

  return Math.min(
    distances[index + 1],
    distances[index] + v0 * elapsed + 0.5 * acceleration * elapsed * elapsed,
  );
}

export function profileVelocityAtDistance(
  profile: MotionProfile,
  distance: number,
): number {
  const { distances, velocities } = profile;
  const last = distances.length - 1;
  if (last < 1) return velocities[0] ?? 0;

  const target = Math.max(distances[0], Math.min(distances[last], distance));
  const index = Math.min(lowerBound(distances, target), last - 1);
  const span = distances[index + 1] - distances[index];
  if (span <= 0) return velocities[index];

  // Speed follows v^2 = v0^2 + 2·a·d under constant acceleration.
  const v0 = velocities[index];
  const acceleration = intervalAcceleration(v0, velocities[index + 1], span);
  return Math.sqrt(Math.max(0, v0 * v0 + 2 * acceleration * (target - distances[index])));
}
