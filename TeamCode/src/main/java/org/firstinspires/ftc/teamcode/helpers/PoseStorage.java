package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

/**
 * One-shot pose handoff from an auto to the TeleOp that follows it.
 *
 * The Pinpoint keeps its own pose across OpModes (ResetMode.NONE), so a stored pose is only
 * worth applying once - right after the auto that saved it. Every auto clears it at init and
 * TeleOp consumes it with {@link #take()}; otherwise a TeleOp restarted mid-match would snap
 * odometry back to the end-of-auto pose.
 */
public final class PoseStorage {
    public static Pose pose = null;

    private PoseStorage() {}

    /** Returns the stored pose (or null) and clears it. */
    public static Pose take() {
        Pose p = pose;
        pose = null;
        return p;
    }

    public static void clear() {
        pose = null;
    }
}
