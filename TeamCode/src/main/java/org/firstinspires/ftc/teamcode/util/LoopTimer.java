package org.firstinspires.ftc.teamcode.util;

/**
 * Loop timing and rate limiting. The clock is passed in.
 *
 * <p>The first {@link #tick} reports {@code dt = 0} rather than a boot-relative nanoTime, and
 * {@link #hz()} returns 0 rather than infinity when {@code dt} is 0.
 */
public final class LoopTimer {

    private static final double NANOS_PER_SECOND = 1e9;

    private boolean started;
    private long lastNs;
    private double dtSeconds;

    private boolean throttleStarted;
    private long lastDueMs;

    /** Call once per loop with {@code System.nanoTime()}. */
    public void tick(long nowNs) {
        if (!started) {
            started = true;
            lastNs = nowNs;
            dtSeconds = 0;
            return;
        }
        dtSeconds = (nowNs - lastNs) / NANOS_PER_SECOND;
        lastNs = nowNs;
    }

    public double dtSeconds() {
        return dtSeconds;
    }

    public double hz() {
        return dtSeconds > 0 ? 1.0 / dtSeconds : 0.0;
    }

    /** True at most once per {@code intervalMs}, and always on the first call. */
    public boolean due(long nowMs, long intervalMs) {
        if (!throttleStarted || nowMs - lastDueMs >= intervalMs) {
            throttleStarted = true;
            lastDueMs = nowMs;
            return true;
        }
        return false;
    }
}
