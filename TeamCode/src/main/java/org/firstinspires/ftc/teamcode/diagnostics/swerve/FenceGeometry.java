package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import java.util.List;

/**
 * Plane geometry for the bring-up fence's keep-out zones. No Android, no hardware - every method
 * is a pure function, so it is checked on the host ({@code FenceGeometryCheck}) before it ever
 * steers a robot.
 *
 * <p>Keep-outs are CONVEX polygons in the pose frame, inches (the BIOBUZZ rails and FLOWERS from
 * {@code field/biobuzz_field.json}). The robot is a circle of its half-diagonal: conservative at
 * every heading, which is the right side to err on for a fence nobody is watching.
 */
public final class FenceGeometry {

    private FenceGeometry() {
    }

    /** A convex polygon, vertices in order (either winding). */
    public static final class Polygon {
        public final String name;
        public final double[] xs;
        public final double[] ys;

        public Polygon(String name, double[] xs, double[] ys) {
            if (xs.length != ys.length || xs.length < 3) {
                throw new IllegalArgumentException(name + ": a polygon needs 3+ vertices");
            }
            this.name = name;
            this.xs = xs.clone();
            this.ys = ys.clone();
        }

        double cx() {
            double s = 0;
            for (double x : xs) {
                s += x;
            }
            return s / xs.length;
        }

        double cy() {
            double s = 0;
            for (double y : ys) {
                s += y;
            }
            return s / ys.length;
        }
    }

    /**
     * Where a point stands relative to a polygon: distance to its boundary (0 inside), and the
     * unit direction AWAY from the polygon - from the closest boundary point toward the point
     * outside, the nearest edge's outward normal inside.
     */
    public static final class Relation {
        public final double distance;
        public final boolean inside;
        public final double nx;
        public final double ny;

        Relation(double distance, boolean inside, double nx, double ny) {
            this.distance = distance;
            this.inside = inside;
            this.nx = nx;
            this.ny = ny;
        }
    }

    public static Relation relate(Polygon p, double px, double py) {
        int n = p.xs.length;
        double best = Double.POSITIVE_INFINITY;
        double bestCx = 0;
        double bestCy = 0;
        int bestEdge = 0;
        for (int i = 0; i < n; i++) {
            double ax = p.xs[i];
            double ay = p.ys[i];
            double bx = p.xs[(i + 1) % n];
            double by = p.ys[(i + 1) % n];
            double ex = bx - ax;
            double ey = by - ay;
            double len2 = ex * ex + ey * ey;
            double t = len2 == 0 ? 0 : ((px - ax) * ex + (py - ay) * ey) / len2;
            t = Math.max(0, Math.min(1, t));
            double cx = ax + t * ex;
            double cy = ay + t * ey;
            double d = Math.hypot(px - cx, py - cy);
            if (d < best) {
                best = d;
                bestCx = cx;
                bestCy = cy;
                bestEdge = i;
            }
        }
        boolean inside = contains(p, px, py);
        if (!inside && best > 1e-9) {
            return new Relation(best, false, (px - bestCx) / best, (py - bestCy) / best);
        }
        // On or inside the boundary: push out through the nearest edge.
        double ax = p.xs[bestEdge];
        double ay = p.ys[bestEdge];
        double bx = p.xs[(bestEdge + 1) % n];
        double by = p.ys[(bestEdge + 1) % n];
        double ex = bx - ax;
        double ey = by - ay;
        double len = Math.hypot(ex, ey);
        double nx = len == 0 ? 1 : ey / len;
        double ny = len == 0 ? 0 : -ex / len;
        // Orient away from the centroid, which is inside any convex polygon.
        if ((ax - p.cx()) * nx + (ay - p.cy()) * ny < 0) {
            nx = -nx;
            ny = -ny;
        }
        return new Relation(0, inside, nx, ny);
    }

    /** Point in convex polygon, boundary counted as inside. */
    public static boolean contains(Polygon p, double px, double py) {
        int n = p.xs.length;
        int sign = 0;
        for (int i = 0; i < n; i++) {
            double ax = p.xs[i];
            double ay = p.ys[i];
            double bx = p.xs[(i + 1) % n];
            double by = p.ys[(i + 1) % n];
            double cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
            if (Math.abs(cross) < 1e-12) {
                continue;
            }
            int s = cross > 0 ? 1 : -1;
            if (sign == 0) {
                sign = s;
            } else if (s != sign) {
                return false;
            }
        }
        return true;
    }

    /** Smoothstep from 0 at {@code slack <= 0} to 1 at {@code slack >= taper}. */
    public static double taper(double slack, double taper) {
        if (slack <= 0) {
            return 0.0;
        }
        if (slack >= taper) {
            return 1.0;
        }
        double u = slack / taper;
        return u * u * (3.0 - 2.0 * u);
    }

    /**
     * Clamps a field-frame velocity command against one keep-out, the polygon analogue of the
     * box's per-axis wall clamp: only the component pointing INTO the obstacle is reduced, by the
     * same smoothstep taper, reaching zero where the robot's circle is {@code marginBase +
     * lookahead x (measured speed toward it)} from the polygon. Motion along or away from it is
     * untouched, so the robot slides past instead of stopping dead.
     *
     * @return {vx, vy, 1 if clamped else 0}
     */
    public static double[] clampKeepOut(Polygon p, double px, double py, double vx, double vy,
            double measuredVx, double measuredVy, double robotRadius, double marginBase,
            double lookaheadS, double taperIn) {
        Relation r = relate(p, px, py);
        double toward = -(vx * r.nx + vy * r.ny);
        if (toward <= 0) {
            return new double[] {vx, vy, 0};
        }
        double measuredToward = Math.max(0, -(measuredVx * r.nx + measuredVy * r.ny));
        double margin = marginBase + measuredToward * lookaheadS;
        double slack = (r.inside ? -r.distance : r.distance) - robotRadius - margin;
        double keep = taper(slack, taperIn);
        if (keep >= 1.0) {
            return new double[] {vx, vy, 0};
        }
        double removed = toward * (1.0 - keep);
        return new double[] {vx + r.nx * removed, vy + r.ny * removed, 1};
    }

    /** Smallest distance from a point to any keep-out, minus nothing - raw boundary distance. */
    public static double clearance(List<Polygon> keepOuts, double px, double py) {
        double best = Double.POSITIVE_INFINITY;
        for (Polygon p : keepOuts) {
            Relation r = relate(p, px, py);
            best = Math.min(best, r.inside ? -r.distance : r.distance);
        }
        return best;
    }

    /** The keep-out a point is closest to, or null when there are none. */
    public static Polygon nearest(List<Polygon> keepOuts, double px, double py) {
        Polygon best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Polygon p : keepOuts) {
            Relation r = relate(p, px, py);
            double d = r.inside ? -r.distance : r.distance;
            if (d < bestD) {
                bestD = d;
                best = p;
            }
        }
        return best;
    }

    /**
     * Minimum keep-out clearance along a Bezier of any degree (control points as {x, y}),
     * sampled. The control polygon's convex hull bounds the curve against a BOX, but not against
     * an obstacle in the middle of it - a curve whose control points all clear a rail can still
     * cross it - so curves are sampled. The return is pessimistic by half a sample spacing, so a
     * thin obstacle cannot slip between two samples.
     */
    public static double bezierClearance(List<Polygon> keepOuts, double[][] controlPoints,
            int samples) {
        if (keepOuts.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }
        double best = Double.POSITIVE_INFINITY;
        double prevX = 0;
        double prevY = 0;
        double maxStep = 0;
        for (int s = 0; s <= samples; s++) {
            double[] pt = bezier(controlPoints, (double) s / samples);
            best = Math.min(best, clearance(keepOuts, pt[0], pt[1]));
            if (s > 0) {
                maxStep = Math.max(maxStep, Math.hypot(pt[0] - prevX, pt[1] - prevY));
            }
            prevX = pt[0];
            prevY = pt[1];
        }
        return best - maxStep / 2;
    }

    /** De Casteljau. */
    public static double[] bezier(double[][] cps, double t) {
        int n = cps.length;
        double[] x = new double[n];
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            x[i] = cps[i][0];
            y[i] = cps[i][1];
        }
        for (int k = n - 1; k > 0; k--) {
            for (int i = 0; i < k; i++) {
                x[i] = x[i] + t * (x[i + 1] - x[i]);
                y[i] = y[i] + t * (y[i + 1] - y[i]);
            }
        }
        return new double[] {x[0], y[0]};
    }

    /**
     * Where to hold to get the robot's circle {@code clearance} clear of every keep-out, starting
     * from (px, py): pushed out along each offender's away-direction. A few passes settle the
     * case of two obstacles close together; if they cannot be satisfied at once the last push
     * wins and the caller's breach timer stops the follower anyway.
     */
    public static double[] pushOut(List<Polygon> keepOuts, double px, double py,
            double clearance) {
        double x = px;
        double y = py;
        for (int pass = 0; pass < 4; pass++) {
            boolean moved = false;
            for (Polygon p : keepOuts) {
                Relation r = relate(p, x, y);
                double d = r.inside ? -r.distance : r.distance;
                if (d < clearance) {
                    double push = clearance - d;
                    x += r.nx * push;
                    y += r.ny * push;
                    moved = true;
                }
            }
            if (!moved) {
                break;
            }
        }
        return new double[] {x, y};
    }

    /** Half-extent of a length x width robot along the field x axis at heading h (radians). */
    public static double reachX(double length, double width, double h) {
        return length / 2 * Math.abs(Math.cos(h)) + width / 2 * Math.abs(Math.sin(h));
    }

    /** Half-extent along the field y axis at heading h (radians). */
    public static double reachY(double length, double width, double h) {
        return length / 2 * Math.abs(Math.sin(h)) + width / 2 * Math.abs(Math.cos(h));
    }
}
