package org.firstinspires.ftc.teamcode.fieldview;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.modules.ZoneCheck;

public final class FieldView {
    private static volatile String snapshot;
    private static volatile long publishedAtNs;
    private static long lastPublishNs;

    private FieldView() {}

    public static void publish(Pose robot, Pose predicted) {
        long now = System.nanoTime();
        if (now - lastPublishNs < SystemConstants.FieldView.PUBLISH_INTERVAL_MS * 1e6) {
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
            .append(",\"inZonePollen\":").append(ZoneCheck.inZone(robot))
            .append(",\"predInZonePollen\":").append(ZoneCheck.inZone(predicted))
            .append(",\"predictMs\":").append(SystemConstants.PredictiveAiming.PREDICT_MS)
            .append(",\"field\":").append(SystemConstants.Zone.FIELD_SIZE)
            .append(",\"robotL\":").append(SystemConstants.Zone.ROBOT_LENGTH)
            .append(",\"robotW\":").append(SystemConstants.Zone.ROBOT_WIDTH);
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
