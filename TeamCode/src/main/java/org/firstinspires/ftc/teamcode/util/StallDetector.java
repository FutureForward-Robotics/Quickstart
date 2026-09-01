package org.firstinspires.ftc.teamcode.util;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/**
 * Reports a mechanism stalled when its current stays at or above a threshold for a period.
 *
 * <p>The period matters: motors draw well above their running current on every direction change and
 * at the start of a move, so a bare threshold reports a stall on normal operation.
 *
 * <p>Takes a current supplier rather than a motor, so it also covers the sum of a motor group.
 */
public final class StallDetector {

    private final DoubleSupplier amps;
    private final double thresholdAmps;
    private final Debouncer debouncer;

    public StallDetector(DoubleSupplier amps, double thresholdAmps, double forSeconds) {
        this(amps, thresholdAmps, forSeconds, System::nanoTime);
    }

    public StallDetector(
            DoubleSupplier amps, double thresholdAmps, double forSeconds, LongSupplier nanos) {
        this.amps = amps;
        this.thresholdAmps = thresholdAmps;
        this.debouncer = new Debouncer(forSeconds, Debouncer.Type.RISING, nanos);
    }

    /** Call once per loop, from the subsystem's sense phase. */
    public boolean update() {
        return debouncer.calculate(amps.getAsDouble() >= thresholdAmps);
    }

    public boolean isStalled() {
        return debouncer.get();
    }
}
