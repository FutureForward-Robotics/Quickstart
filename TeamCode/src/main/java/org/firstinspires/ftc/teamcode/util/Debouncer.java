package org.firstinspires.ftc.teamcode.util;

import java.util.function.LongSupplier;

/**
 * Requires an input to hold a value for a period before reporting the change.
 *
 * <p>{@link Type#RISING} delays a false-to-true transition, {@link Type#FALLING} delays true-to-false,
 * {@link Type#BOTH} delays either. A falling debounce holds the output true for the period after the
 * input drops.
 *
 * <p>The clock is injectable so this can be tested without sleeping.
 */
public final class Debouncer {

    public enum Type {
        RISING,
        FALLING,
        BOTH
    }

    private final long periodNanos;
    private final Type type;
    private final LongSupplier nanos;

    private boolean output;
    private boolean started;
    private long lastChangeNanos;

    public Debouncer(double periodSeconds, Type type) {
        this(periodSeconds, type, System::nanoTime);
    }

    public Debouncer(double periodSeconds, Type type, LongSupplier nanos) {
        if (periodSeconds < 0) {
            throw new IllegalArgumentException("period must not be negative: " + periodSeconds);
        }
        this.periodNanos = (long) (periodSeconds * 1e9);
        this.type = type;
        this.nanos = nanos;
        this.output = type == Type.FALLING;
    }

    /** Feed the raw input; returns the debounced value. Call once per loop. */
    public boolean calculate(boolean input) {
        long now = nanos.getAsLong();
        if (!started) {
            // The window starts at the first sample, not at construction: subsystems are built in
            // init, which runs far longer than any debounce period, so seeding in the constructor
            // let the first transition through with no hold at all.
            started = true;
            lastChangeNanos = now;
        }

        if (input == output) {
            lastChangeNanos = now;
            return output;
        }

        if (debounces(input) && now - lastChangeNanos < periodNanos) {
            return output;
        }

        output = input;
        lastChangeNanos = now;
        return output;
    }

    public boolean get() {
        return output;
    }

    private boolean debounces(boolean input) {
        return type == Type.BOTH || (input ? type == Type.RISING : type == Type.FALLING);
    }
}
