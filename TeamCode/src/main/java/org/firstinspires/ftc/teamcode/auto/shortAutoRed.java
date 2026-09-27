package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class shortAutoRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private AutoPaths paths; // Paths defined in the AutoPaths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new AutoPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.pose().x());
        panelsTelemetry.debug("Y", follower.pose().y());
        panelsTelemetry.debug("Heading", follower.pose().heading());
        panelsTelemetry.update(telemetry);
    }

    public static class AutoPaths {
        public Path MainChain;

        public AutoPaths() {
            MainChain = Paths.path(
                    Paths.line(
                            new Pose(9.000, 14.726),
                            new Pose(9.065, 9.000)
                    ).heading(headingInterpolator(Math.toRadians(270.000), Math.toRadians(270), 1.000)),
                    Paths.curve(
                            new Pose(9.065, 9.000),
                            new Pose(27.988, 99.783),
                            new Pose(46.448, 121.703)
                    ).heading(headingInterpolator(Math.toRadians(270.000), Math.toRadians(90), 1.350)),
                    Paths.line(
                            new Pose(46.448, 121.703),
                            new Pose(46.517, 127.037)
                    ).heading(headingInterpolator(Math.toRadians(90.000), Math.toRadians(90), 1.000)),
                    Paths.line(
                            new Pose(46.517, 127.037),
                            new Pose(12.318, 116.585)
                    ).heading(headingInterpolator(Math.toRadians(90.000), Math.toRadians(0.000), 4.000))
            );
        }

        /**
         * Heading from start to end on the curve parameter t, shaped by t^curve - exactly what the
         * visualizer draws, and Pedro 2's setLinearHeadingInterpolation at curve 1. Pedro 3's
         * Path.linear() interpolates on distance fraction instead, which differs on most Beziers.
         */
        private static Interpolator headingInterpolator(
                double startHeading,
                double endHeading,
                double curve
        ) {
            double clampedCurve = Math.max(0.25, Math.min(4.0, curve));
            double deltaHeading = normalizeRadians(endHeading - startHeading);
            return (pathCurve, t) ->
                    normalizeRadians(
                            startHeading + deltaHeading * Math.pow(Math.max(0.0, Math.min(1.0, t)), clampedCurve)
                    );
        }

        private static double normalizeRadians(double angle) {
            while (angle <= -Math.PI) {
                angle += 2.0 * Math.PI;
            }
            while (angle > Math.PI) {
                angle -= 2.0 * Math.PI;
            }
            return angle;
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}