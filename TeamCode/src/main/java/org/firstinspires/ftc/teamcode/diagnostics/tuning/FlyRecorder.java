package org.firstinspires.ftc.teamcode.diagnostics.tuning;

/**
 * Fixed-size flywheel trace, one sample per loop. Like the swerve PodRecorder it stops when full
 * and never wraps, so a pulled trace is always one contiguous run.
 *
 * <p>Written only from the OpMode thread. The web thread renders the CSV, which is safe once
 * recording has stopped and harmless (a short or torn last row) while it is still running.
 */
public final class FlyRecorder {
    public static final int CAPACITY = 6000;

    static final String HEADER =
            "t,dtMs,pTgt,pVel,nTgt,nVel,pCur,nCur,volts,pMode,nMode,event";

    final double[] t = new double[CAPACITY];
    final double[] dt = new double[CAPACITY];
    final double[] pTgt = new double[CAPACITY];
    final double[] pVel = new double[CAPACITY];
    final double[] nTgt = new double[CAPACITY];
    final double[] nVel = new double[CAPACITY];
    final double[] pCur = new double[CAPACITY];
    final double[] nCur = new double[CAPACITY];
    final double[] volts = new double[CAPACITY];
    final int[] pMode = new int[CAPACITY];
    final int[] nMode = new int[CAPACITY];
    final int[] event = new int[CAPACITY];

    private volatile int size;
    private volatile boolean recording;
    private volatile String label = "";
    private long startNs;
    private int pendingEvent;

    public void start(String newLabel) {
        size = 0;
        label = newLabel == null ? "" : newLabel;
        startNs = System.nanoTime();
        pendingEvent = 0;
        recording = true;
    }

    public void stop() {
        recording = false;
    }

    public boolean isRecording() {
        return recording;
    }

    public boolean isFull() {
        return size >= CAPACITY;
    }

    public int size() {
        return size;
    }

    public String label() {
        return label;
    }

    /** Marks the next sample (1 = step, 2 = kick, 3 = dip detected, 4 = ff level). */
    public void mark(int code) {
        pendingEvent = code;
    }

    public void add(double dtS, double pT, double pV, double nT, double nV,
                    double pC, double nC, double v, int pm, int nm) {
        if (!recording) {
            return;
        }
        int i = size;
        if (i >= CAPACITY) {
            recording = false;
            return;
        }
        t[i] = (System.nanoTime() - startNs) / 1e9;
        dt[i] = dtS;
        pTgt[i] = pT;
        pVel[i] = pV;
        nTgt[i] = nT;
        nVel[i] = nV;
        pCur[i] = pC;
        nCur[i] = nC;
        volts[i] = v;
        pMode[i] = pm;
        nMode[i] = nm;
        event[i] = pendingEvent;
        pendingEvent = 0;
        size = i + 1;
    }

    public String toCsv() {
        int n = size;
        StringBuilder sb = new StringBuilder(64 + n * 80);
        sb.append("# label=").append(label).append(", samples=").append(n)
                .append(", recording=").append(recording).append('\n');
        sb.append(HEADER).append('\n');
        for (int i = 0; i < n; i++) {
            sb.append(r(t[i], 4)).append(',').append(r(dt[i] * 1000, 2)).append(',')
                    .append(r(pTgt[i], 1)).append(',').append(r(pVel[i], 1)).append(',')
                    .append(r(nTgt[i], 1)).append(',').append(r(nVel[i], 1)).append(',')
                    .append(r(pCur[i], 3)).append(',').append(r(nCur[i], 3)).append(',')
                    .append(r(volts[i], 2)).append(',').append(pMode[i]).append(',')
                    .append(nMode[i]).append(',').append(event[i]).append('\n');
        }
        return sb.toString();
    }

    /** Hand-rolled rounding: String.format cost the swerve dashboard 35 ms per publish. */
    static String r(double v, int digits) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return "";
        }
        double scale = Math.pow(10, digits);
        return Double.toString(Math.round(v * scale) / scale);
    }
}
