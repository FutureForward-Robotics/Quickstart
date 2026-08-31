package org.firstinspires.ftc.teamcode.field;

/**
 * Field geometry and the red-to-blue mirror.
 *
 * <p>The mirror is across the vertical centre line: {@code x -> WIDTH_IN - x}, {@code heading ->
 * 180deg - heading}, {@code y} unchanged. This is the same transform the 2025 code copy-pasted into
 * three files as {@code makeXRed} / {@code makeAngRed}.
 *
 * <p><b>The mirror is only a seed.</b> Measured across the 2025 auto paths, red and blue waypoints
 * differed by a median of 4.2 inches with no systematic bias (dx mean +0.68 sd 4.51, dy mean +2.05
 * sd 5.66), and only 1 of 26 waypoints was within half an inch of a pure mirror. That is why {@link
 * Waypoint} lets any single waypoint pin its own blue value instead of forcing the mirror.
 */
public final class Field {

    private Field() {}

    /** Standard FTC field, inches. */
    public static final double WIDTH_IN = 144.0;

    public static double mirrorX(double x) {
        return WIDTH_IN - x;
    }

    public static double mirrorHeading(double headingRad) {
        return normalize(Math.PI - headingRad);
    }

    /** Wrap an angle to [-pi, pi). */
    public static double normalize(double rad) {
        double a = (rad + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }
}
