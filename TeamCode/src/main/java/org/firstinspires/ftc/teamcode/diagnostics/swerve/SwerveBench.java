package org.firstinspires.ftc.teamcode.diagnostics.swerve;

import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Thread-safe hand-off between the {@link SwerveBringUp} OpMode and the web dashboard.
 *
 * <p>The OpMode publishes a state snapshot every loop; the NanoHTTPD worker threads read that
 * snapshot and push commands back. Nothing here touches hardware, so a stale or absent OpMode
 * simply means the dashboard shows "waiting for OpMode" instead of throwing.
 */
public final class SwerveBench {
    public static final SwerveBench INSTANCE = new SwerveBench();

    /** A snapshot is considered live if the OpMode published within this window. */
    private static final long LIVE_WINDOW_MS = 1500;

    /** Bounded so a disconnected OpMode can't let the queue grow without limit. */
    private static final int MAX_PENDING_COMMANDS = 64;

    private final AtomicReference<String> stateJson =
            new AtomicReference<>("{\"live\":false,\"pods\":[]}");
    private final ConcurrentLinkedQueue<Map<String, String>> commands = new ConcurrentLinkedQueue<>();
    private volatile long lastPublishMs = 0;

    /**
     * The running OpMode's loop-rate recorder, so {@code /swerve/rec.csv} can render it directly on
     * the web thread instead of making the control loop pay for a 3000-row string build.
     */
    private final AtomicReference<PodRecorder> recorder = new AtomicReference<>();

    private SwerveBench() {
    }

    /** Called from the OpMode loop with the latest serialized state. */
    public void publish(String json) {
        stateJson.set(json);
        lastPublishMs = System.currentTimeMillis();
    }

    /** Latest snapshot, or a placeholder if the OpMode has never run. */
    public String state() {
        return stateJson.get();
    }

    /** True when an OpMode published recently enough to be considered running. */
    public boolean isLive() {
        return System.currentTimeMillis() - lastPublishMs < LIVE_WINDOW_MS;
    }

    /**
     * Queues a command from the web UI. Dropped silently when the queue is saturated, which only
     * happens if no OpMode is draining it.
     */
    public void submit(Map<String, String> command) {
        if (commands.size() >= MAX_PENDING_COMMANDS) {
            commands.poll();
        }
        commands.add(command);
    }

    /** Removes and returns the next pending command, or null. */
    public Map<String, String> poll() {
        return commands.poll();
    }

    /** Drops anything queued; used when the OpMode starts so stale clicks don't fire. */
    public void clearCommands() {
        commands.clear();
    }

    /** Published by the OpMode at init so the CSV route can find it. */
    public void setRecorder(PodRecorder podRecorder) {
        recorder.set(podRecorder);
    }

    /** The active recorder, or null when no OpMode has run. */
    public PodRecorder recorder() {
        return recorder.get();
    }

    /**
     * Marks the OpMode as gone so the dashboard stops showing live data immediately.
     *
     * <p>The recorder deliberately outlives the OpMode: a run is usually pulled after the OpMode
     * has been stopped, and dropping it here would throw the data away at exactly the wrong moment.
     */
    public void markStopped() {
        lastPublishMs = 0;
        commands.clear();
    }
}
