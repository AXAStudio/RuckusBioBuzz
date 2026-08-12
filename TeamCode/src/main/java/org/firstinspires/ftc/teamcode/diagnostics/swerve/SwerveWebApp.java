package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

/**
 * Serves the swerve bring-up dashboard from the Robot Controller's built-in web server, the same
 * mechanism FTC Dashboard uses.
 *
 * <p>Open <b>http://192.168.43.1:8080/swerve</b> from a laptop on the robot's WiFi. On a phone-based
 * Robot Controller substitute that phone's address.
 *
 * <p>Routes are registered individually because the SDK dispatches with an exact map lookup on the
 * request URI - there is no prefix matching.
 */
public class SwerveWebApp {
    private static final String TAG = "SwerveBringUp";

    private static final String ROOT = "/swerve";
    private static final String STATE = "/swerve/state";
    private static final String CMD = "/swerve/cmd";
    private static final String CONFIG = "/swerve/config";
    private static final String RESTART = "/swerve/restart";
    private static final String REC = "/swerve/rec.csv";

    /** Delay before exiting, so the HTTP response reaches the browser first. */
    private static final long RESTART_DELAY_MS = 600;

    /** Defaults follow the wiring diagram: four pods on Control Hub ports 0-3. */
    private static final String DEFAULT_MOTORS = "sm0:0,sm1:1,sm2:2,sm3:3";
    private static final String DEFAULT_SERVOS = "ss0:0,ss1:1,ss2:2,ss3:3";
    private static final String DEFAULT_ANALOGS = "se0:0,se1:1,se2:2,se3:3";
    private static final String DEFAULT_PINPOINT = "pinpoint:0";

    private static final String MIME_HTML = "text/html";
    private static final String MIME_JSON = "application/json";
    private static final String MIME_CSV = "text/csv";

    /** Cached page body; the asset never changes at runtime. */
    private static volatile String cachedPage;

    private SwerveWebApp() {
    }

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        try {
            WebHandler page = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_HTML, page()));
                }
            };

            WebHandler state = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    String body = SwerveBench.INSTANCE.isLive()
                            ? SwerveBench.INSTANCE.state()
                            : "{\"live\":false,\"pods\":[],\"errors\":[],\"notes\":[],"
                                    + "\"message\":\"Run the Swerve Bring-Up OpMode.\"}";
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_JSON, body));
                }
            };

            WebHandler cmd = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    Map<String, String> params = new HashMap<>();
                    for (Map.Entry<String, List<String>> e : session.getParameters().entrySet()) {
                        List<String> values = e.getValue();
                        if (values != null && !values.isEmpty()) {
                            params.put(e.getKey(), values.get(0));
                        }
                    }
                    SwerveBench.INSTANCE.submit(params);
                    String body = "{\"ok\":true,\"live\":"
                            + SwerveBench.INSTANCE.isLive() + "}";
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_JSON, body));
                }
            };

            // Config handling lives here rather than in the OpMode on purpose: with no Driver
            // Station there is no configuration, so no OpMode can run until one is written.
            WebHandler config = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_JSON, handleConfig(session)));
                }
            };

            // Applying a new configuration needs a robot restart. The Robot Controller's own
            // Manage page has no restart control - that lives in the Driver Station app, which is
            // exactly what a team without a DS does not have.
            WebHandler restart = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    scheduleRestart();
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_JSON,
                            "{\"ok\":true,\"message\":\"Restarting. This page reconnects in about "
                                    + "15 seconds.\"}"));
                }
            };

            // Rendered here rather than in the OpMode so a 3000-row CSV build never lands inside
            // the control loop. Safe because the recorder is only read once it has stopped writing.
            WebHandler rec = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    PodRecorder recorder = SwerveBench.INSTANCE.recorder();
                    String body = recorder == null
                            ? "# no recorder; run the Swerve Bring-Up OpMode\n"
                            : recorder.toCsv();
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, MIME_CSV, body));
                }
            };

            manager.register(ROOT, page);
            manager.register(ROOT + "/", page);
            manager.register(STATE, state);
            manager.register(CMD, cmd);
            manager.register(CONFIG, config);
            manager.register(RESTART, restart);
            manager.register(REC, rec);

            RobotLog.ii(TAG, "Swerve bring-up dashboard registered at %s", ROOT);
        } catch (RuntimeException e) {
            // A failure here must never prevent the Robot Controller from starting.
            RobotLog.ee(TAG, e, "Failed to register swerve bring-up dashboard");
        }
    }

    /**
     * Restarts the Robot Controller app, which is what rebuilds the hardware map after a
     * configuration change.
     *
     * <p>{@code AppUtil.restartApp} schedules the relaunch and then calls {@code System.exit},
     * so it must run after this request's response has been written - hence the delay on a
     * separate thread.
     */
    private static void scheduleRestart() {
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Thread.sleep(RESTART_DELAY_MS);
                    RobotLog.ii(TAG, "Restarting Robot Controller app at dashboard request");
                    AppUtil.getInstance().restartApp(0);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (RuntimeException e) {
                    RobotLog.ee(TAG, e, "Restart request failed");
                }
            }
        }, "SwerveBringUp-restart");
        t.setDaemon(true);
        t.start();
    }

    /**
     * Previews or writes a robot configuration. {@code ?write=1} commits it; anything else only
     * renders the XML so it can be reviewed (or copied out for an adb push) before committing.
     */
    private static String handleConfig(NanoHTTPD.IHTTPSession session) {
        Map<String, String> p = firstValues(session);

        String name = orDefault(p.get("name"), SwerveConfigWriter.DEFAULT_CONFIG_NAME);
        String xml = SwerveConfigWriter.buildXml(
                orDefault(p.get("motors"), DEFAULT_MOTORS),
                orDefault(p.get("servos"), DEFAULT_SERVOS),
                orDefault(p.get("analogs"), DEFAULT_ANALOGS),
                orDefault(p.get("pinpoint"), DEFAULT_PINPOINT),
                parseInt(p.get("expansion"), 0));

        boolean write = "1".equals(p.get("write"));
        boolean ok;
        String message;

        if (p.get("builtin") != null) {
            // "1" keeps the original meaning; any other value names a res/xml file,
            // so the positional A/B config is reachable without a Driver Station.
            String which = p.get("builtin");
            SwerveConfigWriter.Result result = "1".equals(which)
                    ? SwerveConfigWriter.activateBuiltIn()
                    : SwerveConfigWriter.activateBuiltIn(which);
            ok = result.ok;
            message = result.message;
        } else if (write) {
            SwerveConfigWriter.Result result = SwerveConfigWriter.writeAndActivate(name, xml);
            ok = result.ok;
            message = result.message;
        } else {
            String problem = SwerveConfigWriter.validate(xml);
            ok = problem == null;
            message = ok ? "Configuration is valid. Nothing written yet." : problem;
        }

        return "{\"ok\":" + ok
                + ",\"active\":\"" + jsonEscape(SwerveConfigWriter.activeConfigName()) + "\""
                + ",\"name\":\"" + jsonEscape(name) + "\""
                + ",\"message\":\"" + jsonEscape(message) + "\""
                + ",\"xml\":\"" + jsonEscape(xml) + "\"}";
    }

    private static Map<String, String> firstValues(NanoHTTPD.IHTTPSession session) {
        Map<String, String> out = new HashMap<>();
        for (Map.Entry<String, List<String>> e : session.getParameters().entrySet()) {
            List<String> values = e.getValue();
            if (values != null && !values.isEmpty()) {
                out.put(e.getKey(), values.get(0));
            }
        }
        return out;
    }

    private static String orDefault(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
    }

    private static int parseInt(String value, int fallback) {
        try {
            return value == null ? fallback : Integer.parseInt(value.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static String jsonEscape(String s) {
        if (s == null) {
            return "";
        }
        StringBuilder out = new StringBuilder(s.length() + 32);
        for (int i = 0; i < s.length(); i++) {
            char ch = s.charAt(i);
            switch (ch) {
                case '"': out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\n': out.append("\\n"); break;
                case '\r': out.append("\\r"); break;
                case '\t': out.append("\\t"); break;
                default:
                    if (ch < 0x20) {
                        out.append(String.format("\\u%04x", (int) ch));
                    } else {
                        out.append(ch);
                    }
                    break;
            }
        }
        return out.toString();
    }

    private static NanoHTTPD.Response noCache(NanoHTTPD.Response response) {
        response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        response.addHeader("Pragma", "no-cache");
        return response;
    }

    /** Loads the dashboard HTML bundled alongside this class. */
    private static String page() {
        String cached = cachedPage;
        if (cached != null) {
            return cached;
        }

        InputStream in = null;
        try {
            in = SwerveWebApp.class.getResourceAsStream("dashboard.html");
            if (in == null) {
                return errorPage("dashboard.html was not bundled into the APK. Confirm "
                        + "TeamCode/build.gradle still adds src/main/java to resources.srcDirs.");
            }
            ByteArrayOutputStream out = new ByteArrayOutputStream(64 * 1024);
            byte[] buffer = new byte[8192];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            cached = out.toString("UTF-8");
            cachedPage = cached;
            return cached;
        } catch (IOException e) {
            return errorPage("Could not read dashboard.html: " + e.getMessage());
        } finally {
            if (in != null) {
                try {
                    in.close();
                } catch (IOException ignored) {
                    // nothing useful to do
                }
            }
        }
    }

    private static String errorPage(String detail) {
        return "<!doctype html><html><body style=\"font-family:sans-serif;padding:2rem\">"
                + "<h1>Swerve Bring-Up</h1><p>" + detail + "</p></body></html>";
    }
}
