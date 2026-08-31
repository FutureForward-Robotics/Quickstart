package org.firstinspires.ftc.teamcode.field;

/**
 * Field geometry and the red-to-blue mirror: {@code x -> WIDTH_IN - x}, {@code heading -> 180deg -
 * heading}, y unchanged.
 *
 * <p>The mirror is a default. Per-side differences are pinned on the individual {@link Waypoint}.
 */
public final class Field {

    private Field() {}

    public static final double WIDTH_IN = 144.0;

    public static double mirrorX(double x) {
        return WIDTH_IN - x;
    }

    public static double mirrorHeading(double headingRad) {
        return normalize(Math.PI - headingRad);
    }

    /** Wraps to [-pi, pi). */
    public static double normalize(double rad) {
        double a = (rad + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }
}
