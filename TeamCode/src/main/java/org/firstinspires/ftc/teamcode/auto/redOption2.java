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
public class redOption2 extends OpMode {
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
                            new Pose(56.000, 9.002),
                            new Pose(9.806, 9.223)
                    ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(180.000), 1.000)),
                    Paths.curve(
                            new Pose(9.806, 9.223),
                            new Pose(33.873, 119.788),
                            new Pose(46.955, 126.906)
                    ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(90), 1.000)),
                    Paths.curve(
                            new Pose(46.955, 126.906),
                            new Pose(63.725, 108.752),
                            new Pose(57.111, 54.528),
                            new Pose(56.841, 76.455),
                            new Pose(57.078, 13.247)
                    ).tangent(),
                    Paths.curve(
                            new Pose(57.078, 13.247),
                            new Pose(21.497, 4.256),
                            new Pose(15.104, 47.677)
                    ).heading(headingInterpolator(Math.toRadians(-90.126), Math.toRadians(180), 2.800)),
                    Paths.line(
                            new Pose(15.104, 47.677),
                            new Pose(48.611, 18.271)
                    ).heading(headingInterpolator(Math.toRadians(180.000), Math.toRadians(90), 1.000)),
                    Paths.curve(
                            new Pose(48.611, 18.271),
                            new Pose(8.480, 56.330),
                            new Pose(11.182, 95.537)
                    ).heading(headingInterpolator(Math.toRadians(90.000), Math.toRadians(90), 1.000))
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