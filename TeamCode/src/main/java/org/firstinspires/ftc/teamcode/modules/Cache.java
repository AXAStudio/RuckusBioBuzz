package org.firstinspires.ftc.teamcode.modules;

public final class Cache {
    private boolean has;
    private long key;
    private long lastNs;

    public boolean changed(long newKey, boolean enabled, double refreshMs) {
        long now = System.nanoTime();
        if (enabled && has && newKey == key && now - lastNs < refreshMs * 1e6) {
            return false;
        }
        has = true;
        key = newKey;
        lastNs = now;
        return true;
    }

    public boolean due(double intervalMs) {
        long now = System.nanoTime();
        if (has && now - lastNs < intervalMs * 1e6) {
            return false;
        }
        has = true;
        lastNs = now;
        return true;
    }

    public void invalidate() {
        has = false;
    }

    public static long velocityKey(double ticksPerSecond, boolean reversed) {
        return Math.round(reversed ? -ticksPerSecond : ticksPerSecond);
    }

    public static long servoKey(double position, boolean reversed, double pwmLowerUs, double pwmUpperUs) {
        double q = Math.max(0, Math.min(1, position));
        if (reversed) q = 1 - q;
        return (long) (pwmLowerUs + q * (pwmUpperUs - pwmLowerUs));
    }
}
