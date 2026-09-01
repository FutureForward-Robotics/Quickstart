package org.firstinspires.ftc.teamcode.util;

/**
 * Schmitt trigger. The output goes high when the input reaches {@code highThreshold} and low again
 * only once it falls to {@code lowThreshold}; between the two it holds.
 *
 * <p>Use it wherever a single comparison would chatter: a distance sensor at the edge of its
 * detection range, a voltage sag, an encoder sitting on a limit. Hysteresis works on amplitude; to
 * reject a brief excursion instead, use {@link Debouncer}.
 */
public final class Hysteresis {

    private final double lowThreshold;
    private final double highThreshold;
    private boolean state;

    public Hysteresis(double lowThreshold, double highThreshold) {
        this(lowThreshold, highThreshold, false);
    }

    public Hysteresis(double lowThreshold, double highThreshold, boolean initialState) {
        if (highThreshold < lowThreshold) {
            throw new IllegalArgumentException(
                    "highThreshold " + highThreshold + " is below lowThreshold " + lowThreshold);
        }
        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
        this.state = initialState;
    }

    /** Feed the current value; returns the latched output. */
    public boolean calculate(double value) {
        if (state) {
            if (value <= lowThreshold) {
                state = false;
            }
        } else if (value >= highThreshold) {
            state = true;
        }
        return state;
    }

    public boolean get() {
        return state;
    }

    public double lowThreshold() {
        return lowThreshold;
    }

    public double highThreshold() {
        return highThreshold;
    }
}
