package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

public final class PoseStorage {
    public static Pose pose = null;

    private PoseStorage() {}

    public static Pose take() {
        Pose p = pose;
        pose = null;
        return p;
    }

    public static void clear() {
        pose = null;
    }
}
