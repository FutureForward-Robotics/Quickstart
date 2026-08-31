package org.firstinspires.ftc.teamcode.field;

/**
 * Which side we are on.
 *
 * <p>Pass this as a value. Do not put it in a mutable static: last season {@code
 * Config.isRedAlliance} was a global, seven files recomputed {@code dir = isRedAlliance ? 1 : -1}
 * from it, and the red test OpMode shipped with it set to {@code false}.
 */
public enum Alliance {
    /** The canonical authoring frame. All waypoints are written in red coordinates. */
    RED,
    /** Derived from red by mirroring, then tuned per waypoint where the robot needs it. */
    BLUE;

    public boolean isRed() {
        return this == RED;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
