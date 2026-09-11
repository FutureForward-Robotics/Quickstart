package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;

/**
 * Recent odometry poses against the clock, so a measurement can be compared with where the robot
 * was when the measurement was taken rather than where it is now.
 *
 * <p>Fixed capacity, oldest overwritten. Stores primitives, so an add allocates nothing.
 *
 * <p>Pedro's {@code PoseHistory} is not this: it keeps x and y only, for drawing, with no timestamps
 * and no heading.
 */
public final class PoseBuffer {

    /** One second at 50 Hz. Longer than any camera latency worth compensating. */
    public static final int DEFAULT_CAPACITY = 50;

    private final long[] nanos;
    private final double[] xs;
    private final double[] ys;
    private final double[] headings;

    private int count;
    private int next;

    public PoseBuffer() {
        this(DEFAULT_CAPACITY);
    }

    public PoseBuffer(int capacity) {
        if (capacity < 2) {
            throw new IllegalArgumentException("capacity must be at least 2: " + capacity);
        }
        nanos = new long[capacity];
        xs = new double[capacity];
        ys = new double[capacity];
        headings = new double[capacity];
    }

    /** Records a pose. Call once per loop, with a monotonic clock. */
    public void add(long atNanos, Pose pose) {
        nanos[next] = atNanos;
        xs[next] = pose.x();
        ys[next] = pose.y();
        headings[next] = pose.heading();
        next = (next + 1) % nanos.length;
        if (count < nanos.length) {
            count++;
        }
    }

    public int size() {
        return count;
    }

    public void clear() {
        count = 0;
        next = 0;
    }

    /**
     * Pose at {@code atNanos}, interpolated between the two straddling samples and clamped to the
     * ends. Null when empty. Heading interpolation takes the short way round.
     */
    public Pose at(long atNanos) {
        if (count == 0) {
            return null;
        }
        int oldest = (next - count + nanos.length) % nanos.length;
        if (atNanos <= nanos[oldest]) {
            return pose(oldest);
        }
        int newest = (next - 1 + nanos.length) % nanos.length;
        if (atNanos >= nanos[newest]) {
            return pose(newest);
        }
        for (int i = 1; i < count; i++) {
            int high = (oldest + i) % nanos.length;
            if (nanos[high] < atNanos) {
                continue;
            }
            int low = (high - 1 + nanos.length) % nanos.length;
            long span = nanos[high] - nanos[low];
            double t = span == 0 ? 0 : (double) (atNanos - nanos[low]) / span;
            return new Pose(
                    xs[low] + (xs[high] - xs[low]) * t,
                    ys[low] + (ys[high] - ys[low]) * t,
                    Field.normalize(
                            headings[low]
                                    + Field.normalize(headings[high] - headings[low]) * t));
        }
        return pose(newest);
    }

    private Pose pose(int index) {
        return new Pose(xs[index], ys[index], headings[index]);
    }
}
