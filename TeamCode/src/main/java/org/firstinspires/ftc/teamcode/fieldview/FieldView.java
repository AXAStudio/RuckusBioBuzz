package org.firstinspires.ftc.teamcode.fieldview;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.modules.predictiveAiming;
import org.firstinspires.ftc.teamcode.modules.zoneCheck;

public final class FieldView {
    private static final long PUBLISH_INTERVAL_NS = 50_000_000L;

    private static volatile String snapshot;
    private static volatile long publishedAtNs;
    private static long lastPublishNs;

    private FieldView() {}

    public static void publish(Pose robot, Pose predicted) {
        long now = System.nanoTime();
        if (now - lastPublishNs < PUBLISH_INTERVAL_NS) {
            return;
        }
        lastPublishNs = now;

        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"x\":").append(robot.x())
            .append(",\"y\":").append(robot.y())
            .append(",\"h\":").append(robot.heading())
            .append(",\"px\":").append(predicted.x())
            .append(",\"py\":").append(predicted.y())
            .append(",\"ph\":").append(predicted.heading())
            .append(",\"inZone\":").append(zoneCheck.inZone(robot))
            .append(",\"predInZone\":").append(zoneCheck.inZone(predicted))
            .append(",\"predictMs\":").append(predictiveAiming.PREDICT_MS)
            .append(",\"zoneY\":").append(zoneCheck.ZONE_Y)
            .append(",\"field\":").append(zoneCheck.FIELD_SIZE)
            .append(",\"robotL\":").append(zoneCheck.ROBOT_LENGTH)
            .append(",\"robotW\":").append(zoneCheck.ROBOT_WIDTH);
        snapshot = sb.toString();
        publishedAtNs = now;
    }

    static String state() {
        String s = snapshot;
        if (s == null) {
            return "{\"live\":false}";
        }
        long ageMs = (System.nanoTime() - publishedAtNs) / 1_000_000L;
        return s + ",\"live\":" + (ageMs < 1000) + ",\"ageMs\":" + ageMs + "}";
    }
}
