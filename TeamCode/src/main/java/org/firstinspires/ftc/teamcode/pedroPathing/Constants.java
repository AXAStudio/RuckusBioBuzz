package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * Picks which drivetrain's Pedro Pathing constants to build the Follower from, based on
 * the "drivetrain" field in config.jsonc (next to this package, at
 * teamcode/config.jsonc). Every OpMode should keep calling Constants.createFollower(...)
 * exactly as before; only this file needs to know that other drivetrains exist.
 */
public class Constants {
    private static final String CONFIG_RESOURCE_PATH = "/org/firstinspires/ftc/teamcode/config.jsonc";
    private static final String DRIVETRAIN_KEY = "drivetrain";

    public static Follower createFollower(HardwareMap hardwareMap) {
        String drivetrain = readDrivetrain();

        switch (drivetrain) {
            case "swerve":
                return SwerveDrivetrainConstants.createFollower(hardwareMap);
            case "mecanum":
                return MecanumDrivetrainConstants.createFollower(hardwareMap);
            default:
                throw new IllegalStateException("Unknown \"" + DRIVETRAIN_KEY + "\" value \""
                        + drivetrain + "\" in config.jsonc. Expected \"swerve\" or \"mecanum\".");
        }
    }

    private static String readDrivetrain() {
        String json = stripLineComments(readConfigResource());

        JSONObject config;
        try {
            config = new JSONObject(json);
        } catch (JSONException e) {
            throw new IllegalStateException("config.jsonc is not valid JSON once comments are "
                    + "stripped. Contents after stripping: " + json, e);
        }

        if (!config.has(DRIVETRAIN_KEY)) {
            throw new IllegalStateException("config.jsonc is missing the \"" + DRIVETRAIN_KEY
                    + "\" field.");
        }

        return config.optString(DRIVETRAIN_KEY, "").trim().toLowerCase();
    }

    private static String readConfigResource() {
        try (InputStream in = Constants.class.getResourceAsStream(CONFIG_RESOURCE_PATH)) {
            if (in == null) {
                throw new IllegalStateException("Could not find " + CONFIG_RESOURCE_PATH
                        + " on the classpath. Check that TeamCode/build.gradle's sourceSets "
                        + "still adds 'src/main/java' as a resources dir, and that config.jsonc "
                        + "is at TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config.jsonc.");
            }

            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            byte[] chunk = new byte[1024];
            int read;
            while ((read = in.read(chunk)) != -1) {
                buffer.write(chunk, 0, read);
            }
            return buffer.toString("UTF-8");
        } catch (IOException e) {
            throw new IllegalStateException("Failed to read " + CONFIG_RESOURCE_PATH, e);
        }
    }

    /**
     * Strips "// ..." line comments (JSONC) so the remainder can be parsed as plain JSON.
     * Ignores "//" that appears inside a quoted string.
     */
    private static String stripLineComments(String jsonc) {
        StringBuilder result = new StringBuilder(jsonc.length());
        boolean inString = false;
        boolean escaped = false;

        for (int i = 0; i < jsonc.length(); i++) {
            char c = jsonc.charAt(i);

            if (inString) {
                result.append(c);
                if (escaped) {
                    escaped = false;
                } else if (c == '\\') {
                    escaped = true;
                } else if (c == '"') {
                    inString = false;
                }
                continue;
            }

            if (c == '"') {
                inString = true;
                result.append(c);
                continue;
            }

            if (c == '/' && i + 1 < jsonc.length() && jsonc.charAt(i + 1) == '/') {
                while (i < jsonc.length() && jsonc.charAt(i) != '\n') {
                    i++;
                }
                result.append('\n');
                continue;
            }

            result.append(c);
        }

        return result.toString();
    }
}
