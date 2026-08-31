package org.firstinspires.ftc.teamcode.field;

/**
 * Field geometry.
 *
 * <p>{@link #SYMMETRY} is a per-season setting: check whether this year's field is a reflection or
 * a 180 degree rotation before writing any waypoints. It only supplies the default blue value;
 * per-side differences are pinned on the individual {@link Waypoint}.
 */
public final class Field {

    private Field() {}

    public static final double WIDTH_IN = 144.0;

    /** Set for the season. 2025-26 DECODE was {@link FieldSymmetry#MIRROR_X}. */
    public static final FieldSymmetry SYMMETRY = FieldSymmetry.MIRROR_X;

    /** Wraps to [-pi, pi). */
    public static double normalize(double rad) {
        double a = (rad + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }
}
