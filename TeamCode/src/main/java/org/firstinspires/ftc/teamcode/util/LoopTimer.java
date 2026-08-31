package org.firstinspires.ftc.teamcode.util;

/**
 * Loop timing and rate limiting, with the clock passed in so it is testable on a laptop.
 *
 * <p>Fixes two things that bit last season.
 *
 * <ul>
 *   <li><b>Divide by zero.</b> {@code Drivetrain.java:75-79} computed {@code dt} from {@code
 *       System.currentTimeMillis()} and divided by it. A sub-millisecond loop gives {@code dt == 0}
 *       and the angular velocity became {@code ±Infinity}, which was then printed to the driver
 *       station. Here the clock is nanoseconds and {@link #hz()} returns 0 rather than infinity.
 *   <li><b>Bogus first sample.</b> The first {@link #tick} establishes the baseline and reports
 *       {@code dt == 0} instead of "now minus zero", which on Android is boot-relative and enormous.
 *       That is the same class of bug as {@code Debouncer.java:5}.
 * </ul>
 */
public final class LoopTimer {

    private static final double NANOS_PER_SECOND = 1e9;

    private boolean started;
    private long lastNs;
    private double dtSeconds;

    private boolean throttleStarted;
    private long lastDueMs;

    /** Call exactly once per loop, with {@code System.nanoTime()}. */
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

    /** Seconds since the previous loop. Zero on the first loop. */
    public double dtSeconds() {
        return dtSeconds;
    }

    /** Loop rate. Zero rather than infinity when two ticks land on the same nanosecond. */
    public double hz() {
        return dtSeconds > 0 ? 1.0 / dtSeconds : 0.0;
    }

    /**
     * Rate limiter. Returns true at most once per {@code intervalMs}, and always true the first
     * time.
     *
     * <p>Replaces the hand-rolled {@code lastTelemetryUpdate} field that was copy-pasted into three
     * OpModes last season.
     */
    public boolean due(long nowMs, long intervalMs) {
        if (!throttleStarted || nowMs - lastDueMs >= intervalMs) {
            throttleStarted = true;
            lastDueMs = nowMs;
            return true;
        }
        return false;
    }
}
