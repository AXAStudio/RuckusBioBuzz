package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import android.content.Context;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Creates and activates a Robot Controller configuration without a Driver Station.
 *
 * <p>Normally the hardware configuration is authored in the Driver Station app. A team with no DS
 * has no way to produce one, which blocks everything - no config means an empty
 * {@code hardwareMap}. This writes the configuration directly using the SDK's own
 * {@link RobotConfigFileManager}, driven from the bring-up dashboard.
 *
 * <p>It works because {@link SwerveWebApp} registers on the Robot Controller's web server at
 * <em>app startup</em>, not from an OpMode - so the page is reachable before any configuration
 * exists.
 *
 * <p>The generated XML is parsed back with the SDK's own {@link ReadXMLFileHandler} before it is
 * activated, so a malformed config is rejected rather than left active and broken.
 */
public final class SwerveConfigWriter {

    /** Written as a new file, so an existing configuration is never overwritten. */
    public static final String DEFAULT_CONFIG_NAME = "SwerveBringUp";

    private SwerveConfigWriter() {
    }

    /** One configured device: an XML tag, a name and a port. */
    private static final class Device {
        final String tag;
        final String name;
        final int port;
        final int bus;
        final boolean hasBus;

        Device(String tag, String name, int port) {
            this(tag, name, port, -1, false);
        }

        Device(String tag, String name, int port, int bus, boolean hasBus) {
            this.tag = tag;
            this.name = name;
            this.port = port;
            this.bus = bus;
            this.hasBus = hasBus;
        }
    }

    /** Result of a write attempt. */
    public static final class Result {
        public final boolean ok;
        public final String message;
        public final String xml;

        Result(boolean ok, String message, String xml) {
            this.ok = ok;
            this.message = message;
            this.xml = xml;
        }
    }

    /**
     * Builds configuration XML for the swerve drivetrain.
     *
     * @param motors   comma separated {@code name:port}, e.g. {@code sm0:0,sm1:1}
     * @param servos   comma separated {@code name:port} for continuous rotation turn servos
     * @param analogs  comma separated {@code name:port} for the Axon absolute encoders
     * @param pinpoint {@code name:bus} for the goBILDA Pinpoint, or blank to omit it
     * @param expansionAddress address of an attached Expansion Hub, or &lt;= 0 to omit it
     */
    public static String buildXml(String motors, String servos, String analogs, String pinpoint,
            int expansionAddress) {
        List<Device> devices = new ArrayList<>();
        addAll(devices, "Motor", motors);
        addAll(devices, "ContinuousRotationServo", servos);
        addAll(devices, "AnalogInput", analogs);

        String[] pin = split(pinpoint);
        if (pin != null) {
            // I2C devices carry both a bus and a port; the Pinpoint sits alone on its bus.
            devices.add(new Device("goBILDAPinpoint", pin[0], 0, parseInt(pin[1], 0), true));
        }

        int chAddress = LynxConstants.CH_EMBEDDED_MODULE_ADDRESS;

        StringBuilder sb = new StringBuilder(2048);
        sb.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        sb.append("<Robot type=\"FirstInspires-FTC\">\n");
        sb.append("    <LynxUsbDevice name=\"Control Hub Portal\" serialNumber=\"")
                .append(LynxConstants.SERIAL_NUMBER_EMBEDDED)
                .append("\" parentModuleAddress=\"").append(chAddress).append("\">\n");
        sb.append("        <LynxModule name=\"Control Hub\" port=\"").append(chAddress)
                .append("\">\n");
        for (Device d : devices) {
            sb.append("            <").append(d.tag)
                    .append(" name=\"").append(escape(d.name))
                    .append("\" port=\"").append(d.port).append('"');
            if (d.hasBus) {
                sb.append(" bus=\"").append(d.bus).append('"');
            }
            sb.append(" />\n");
        }
        sb.append("        </LynxModule>\n");

        if (expansionAddress > 0) {
            sb.append("        <LynxModule name=\"Expansion Hub ").append(expansionAddress)
                    .append("\" port=\"").append(expansionAddress).append("\">\n");
            sb.append("        </LynxModule>\n");
        }

        sb.append("    </LynxUsbDevice>\n");
        sb.append("</Robot>\n");
        return sb.toString();
    }

    /**
     * Parses the XML with the SDK's own reader.
     *
     * @return null when the configuration is valid, otherwise a human-readable reason
     */
    public static String validate(String xml) {
        try {
            new ReadXMLFileHandler().parse(new StringReader(xml));
            return null;
        } catch (Exception e) {
            return e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    /** Resource name of the configuration bundled in the APK, under res/xml. */
    public static final String BUILT_IN_RESOURCE = "swerve_bringup";

    /**
     * Activates the configuration shipped inside the APK under {@code res/xml}.
     *
     * <p>The SDK treats any {@code res/xml} file rooted at {@code <Robot type="FirstInspires-FTC">}
     * as a read-only configuration, which makes it the natural recovery path: it cannot be lost by
     * wiping the hub, and it is version controlled alongside the code.
     *
     * <p>The resource id is looked up by name rather than through {@code R} so this keeps working
     * regardless of which module's {@code R} class is on the classpath.
     */
    public static Result activateBuiltIn() {
        return activateBuiltIn(BUILT_IN_RESOURCE);
    }

    /**
     * Activates any configuration shipped in {@code res/xml}, by resource name without extension.
     *
     * <p>Parameterised because there is now more than one: the positional A/B needs pod 0's port
     * declared {@code Servo} while the rest stay continuous rotation, and with no Driver Station
     * this is the only way to select it.
     */
    public static Result activateBuiltIn(String resourceName) {
        try {
            Context context = AppUtil.getInstance().getApplication();
            int resourceId = context.getResources().getIdentifier(
                    resourceName, "xml", context.getPackageName());

            if (resourceId == 0) {
                return new Result(false,
                        "No built-in configuration found. Expected res/xml/"
                                + resourceName + ".xml in the APK.", "");
            }

            RobotConfigFileManager manager = new RobotConfigFileManager();
            RobotConfigFile file = new RobotConfigFile(resourceName, resourceId);
            manager.setActiveConfig(file);

            return new Result(true,
                    "Activated built-in configuration \"" + resourceName
                            + "\". Restart the robot to apply it.", "");
        } catch (Exception e) {
            return new Result(false,
                    "Could not activate the built-in configuration: "
                            + e.getClass().getSimpleName() + ": " + e.getMessage(), "");
        }
    }

    /** Name of the configuration the Robot Controller is currently using. */
    public static String activeConfigName() {
        try {
            RobotConfigFile active = new RobotConfigFileManager().getActiveConfig();
            return active == null ? "(none)" : active.getName();
        } catch (Exception e) {
            return "(unknown)";
        }
    }

    /**
     * Writes the configuration and makes it active. The robot must be restarted afterwards for the
     * new hardware map to take effect.
     */
    public static Result writeAndActivate(String name, String xml) {
        String problem = validate(xml);
        if (problem != null) {
            return new Result(false, "Configuration rejected, nothing was changed. " + problem, xml);
        }

        try {
            RobotConfigFileManager manager = new RobotConfigFileManager();
            manager.createConfigFolder();

            RobotConfigFile file = new RobotConfigFile(manager, name);
            manager.writeToFile(file, false, xml);
            manager.setActiveConfig(file);

            return new Result(true,
                    "Wrote and activated \"" + name + "\". Restart the robot to apply it.", xml);
        } catch (Exception e) {
            return new Result(false,
                    "Could not write the configuration: "
                            + e.getClass().getSimpleName() + ": " + e.getMessage(), xml);
        }
    }

    // ---------------------------------------------------------------- parsing helpers

    private static void addAll(List<Device> out, String tag, String spec) {
        if (spec == null) {
            return;
        }
        for (String entry : spec.split(",")) {
            String[] parts = split(entry);
            if (parts != null) {
                out.add(new Device(tag, parts[0], parseInt(parts[1], 0)));
            }
        }
    }

    /** Splits {@code name:port}; returns null for blank or malformed entries. */
    private static String[] split(String entry) {
        if (entry == null) {
            return null;
        }
        String trimmed = entry.trim();
        if (trimmed.isEmpty()) {
            return null;
        }
        int colon = trimmed.lastIndexOf(':');
        if (colon <= 0 || colon == trimmed.length() - 1) {
            return null;
        }
        return new String[] {trimmed.substring(0, colon).trim(), trimmed.substring(colon + 1).trim()};
    }

    private static int parseInt(String value, int fallback) {
        try {
            return Integer.parseInt(value.trim());
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    private static String escape(String s) {
        return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\"", "&quot;");
    }
}
