package org.firstinspires.ftc.teamcode;

public final class Config {
    private Config() {}

    public static int branch = 0; // 0 is dev, 1 is prod, 2 is no telemetry (minimum loop time)

    public static boolean competition() {
        return branch == 1 || branch == 2;
    }

    public static boolean telemetryEnabled() {
        return branch != 2;
    }
}