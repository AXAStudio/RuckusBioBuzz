import org.firstinspires.ftc.teamcode.diagnostics.swerve.FenceGeometry;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.FenceGeometry.Polygon;

import java.util.Arrays;
import java.util.List;

/**
 * Host-side check of the bring-up fence geometry (FenceGeometry is pure Java), against the real
 * BIOBUZZ shapes. Run from the repo root:
 *
 *   D=TeamCode/src/main/java/org/firstinspires/ftc/teamcode/diagnostics/swerve
 *   javac -d /tmp/fence $D/FenceGeometry.java tools/swervetune/FenceGeometryCheck.java \
 *     && java -cp /tmp/fence FenceGeometryCheck
 *
 * Exits non-zero on the first failure.
 */
public class FenceGeometryCheck {
    static int checks = 0;

    static void near(String what, double got, double want, double tol) {
        checks++;
        if (!(Math.abs(got - want) <= tol)) {
            throw new AssertionError(what + ": got " + got + ", want " + want);
        }
    }

    static void is(String what, boolean cond) {
        checks++;
        if (!cond) {
            throw new AssertionError(what);
        }
    }

    public static void main(String[] args) {
        // biobuzz_field.json, HIVE Frame Rail - Red Side, and FLOWER - Red Wall.
        Polygon rail = new Polygon("rail", new double[] {45.59, 48.01, 48.01, 45.59},
                new double[] {50.97, 50.97, 90.53, 90.53});
        Polygon flower = new Polygon("flower", new double[] {0.0, 0.0, 2.6, 5.1, 5.1, 2.6},
                new double[] {43.57, 50.77, 50.52, 49.47, 44.87, 43.82});
        List<Polygon> keepOuts = Arrays.asList(rail, flower);
        double r = Math.hypot(18, 18) / 2;   // 18 x 18 robot, half-diagonal
        double base = 4.0;
        double look = 0.30;
        double taper = 6.0;

        // relate, outside: west of the rail
        FenceGeometry.Relation w = FenceGeometry.relate(rail, 40, 70);
        is("west outside", !w.inside);
        near("west distance", w.distance, 5.59, 1e-9);
        near("west nx", w.nx, -1, 1e-9);
        near("west ny", w.ny, 0, 1e-9);
        // outside past a corner: diagonal normal
        FenceGeometry.Relation c = FenceGeometry.relate(rail, 44.59, 49.97);
        near("corner distance", c.distance, Math.sqrt(2), 1e-9);
        near("corner nx", c.nx, -Math.sqrt(0.5), 1e-9);
        near("corner ny", c.ny, -Math.sqrt(0.5), 1e-9);
        // inside, nearer the west face: pushed out west
        FenceGeometry.Relation in = FenceGeometry.relate(rail, 46.0, 70);
        is("inside", in.inside);
        near("inside nx", in.nx, -1, 1e-9);
        // either winding
        Polygon cw = new Polygon("cw", new double[] {45.59, 45.59, 48.01, 48.01},
                new double[] {50.97, 90.53, 90.53, 50.97});
        is("cw contains", FenceGeometry.contains(cw, 46.8, 70));
        near("cw inside nx", FenceGeometry.relate(cw, 46.0, 70).nx, -1, 1e-9);
        // hexagon
        is("flower contains", FenceGeometry.contains(flower, 3.0, 47.0));
        is("flower excludes", !FenceGeometry.contains(flower, 5.2, 47.0));

        // clamp: inside the hard margin, heading at the rail - stopped
        double[] v = FenceGeometry.clampKeepOut(rail, 30, 70, 1, 0, 0, 0, r, base, look, taper);
        near("into rail vx", v[0], 0, 1e-12);
        near("into rail vy", v[1], 0, 1e-12);
        is("into rail clamped", v[2] == 1);
        // diagonal: the toward component goes, the sliding component stays
        v = FenceGeometry.clampKeepOut(rail, 30, 70, 1, 1, 0, 0, r, base, look, taper);
        near("slide vx", v[0], 0, 1e-12);
        near("slide vy", v[1], 1, 1e-12);
        // parallel and away are never touched
        v = FenceGeometry.clampKeepOut(rail, 30, 70, 0, 1, 0, 0, r, base, look, taper);
        is("parallel untouched", v[0] == 0 && v[1] == 1 && v[2] == 0);
        v = FenceGeometry.clampKeepOut(rail, 30, 70, -1, 0, 0, 0, r, base, look, taper);
        is("away untouched", v[0] == -1 && v[1] == 0 && v[2] == 0);
        // far away: untouched
        v = FenceGeometry.clampKeepOut(rail, 10, 70, 1, 0, 0, 0, r, base, look, taper);
        is("far untouched", v[0] == 1 && v[2] == 0);
        // in the taper: partially kept, and the kept fraction matches the smoothstep
        double x = 45.59 - r - base - 3.0;   // 3 in of slack of a 6 in taper -> 0.5
        v = FenceGeometry.clampKeepOut(rail, x, 70, 1, 0, 0, 0, r, base, look, taper);
        near("taper half", v[0], 0.5, 1e-9);
        // speed toward it grows the margin: 10 in/s adds 3 in, which eats the 3 in of slack
        v = FenceGeometry.clampKeepOut(rail, x, 70, 1, 0, 10, 0, r, base, look, taper);
        near("lookahead stops", v[0], 0, 1e-9);
        // continuity across the taper: monotone in distance, no step
        double prev = -1;
        for (double d = 0; d <= 8; d += 0.05) {
            double[] s = FenceGeometry.clampKeepOut(rail, 45.59 - r - base - d, 70, 1, 0,
                    0, 0, r, base, look, taper);
            is("monotone at " + d, s[0] >= prev - 1e-12);
            is("bounded step at " + d, prev < 0 || s[0] - prev < 0.05);
            prev = s[0];
        }

        // curves: a line through the rail is refused, one under it is clear
        double through = FenceGeometry.bezierClearance(keepOuts,
                new double[][] {{30, 70}, {60, 70}}, 64);
        is("through rail negative", through < 0);
        double under = FenceGeometry.bezierClearance(keepOuts,
                new double[][] {{30, 20}, {60, 20}}, 64);
        near("under rail clearance", under, 50.97 - 20 - 30.0 / 64 / 2, 1e-9);
        // a cubic whose CONTROL POINTS all clear the rail by miles but whose curve crosses it
        double sneaky = FenceGeometry.bezierClearance(keepOuts,
                new double[][] {{30, 40}, {60, 100}, {30, 100}, {60, 40}}, 128);
        is("hull-clear curve still caught", sneaky < 0);

        // push-out from inside the rail lands exactly at the clearance
        double[] p = FenceGeometry.pushOut(keepOuts, 46.0, 70, r + base);
        near("pushOut x", p[0], 45.59 - r - base, 1e-9);
        near("pushOut y", p[1], 70, 1e-9);
        near("pushOut clearance", FenceGeometry.clearance(keepOuts, p[0], p[1]), r + base, 1e-9);

        // heading-aware reach of an 18 x 18 robot
        near("reach 0", FenceGeometry.reachX(18, 18, 0), 9, 1e-12);
        near("reach 45", FenceGeometry.reachX(18, 18, Math.PI / 4), 9 * Math.sqrt(2), 1e-12);

        System.out.println("FenceGeometryCheck: " + checks + " checks passed.");
    }
}
