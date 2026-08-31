package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/**
 * Named field pose, authored in red coordinates. Blue is derived with {@link Field#SYMMETRY}; call
 * {@link #blue} to pin a different value for one waypoint. Immutable.
 *
 * <pre>{@code
 * static final Waypoint START = Waypoint.red("start", 119.380, 128.800, 225);
 * static final Waypoint SCORE = Waypoint.red("score", 99.533, 98.933, 240)
 *         .blue(53.000, 94.000, 240);
 * }</pre>
 */
public final class Waypoint {

    private final String name;
    private final double redX;
    private final double redY;
    private final double redHeading;
    private final boolean bluePinned;
    private final double blueX;
    private final double blueY;
    private final double blueHeading;

    private Waypoint(
            String name,
            double redX,
            double redY,
            double redHeading,
            boolean bluePinned,
            double blueX,
            double blueY,
            double blueHeading) {
        this.name = name;
        this.redX = redX;
        this.redY = redY;
        this.redHeading = redHeading;
        this.bluePinned = bluePinned;
        this.blueX = blueX;
        this.blueY = blueY;
        this.blueHeading = blueHeading;
    }

    /** Heading in degrees. */
    public static Waypoint red(String name, double x, double y, double headingDeg) {
        return new Waypoint(name, x, y, Math.toRadians(headingDeg), false, 0, 0, 0);
    }

    /** Pins blue instead of using the mirror. Heading in degrees. Returns a new instance. */
    public Waypoint blue(double x, double y, double headingDeg) {
        return new Waypoint(
                name, redX, redY, redHeading, true, x, y, Math.toRadians(headingDeg));
    }

    public String name() {
        return name;
    }

    public boolean isBluePinned() {
        return bluePinned;
    }

    public double x(Alliance alliance) {
        if (alliance.isRed()) {
            return redX;
        }
        return bluePinned ? blueX : Field.SYMMETRY.x(redX, redY);
    }

    public double y(Alliance alliance) {
        if (alliance.isRed()) {
            return redY;
        }
        return bluePinned ? blueY : Field.SYMMETRY.y(redX, redY);
    }

    /** Radians, wrapped to [-pi, pi). */
    public double heading(Alliance alliance) {
        if (alliance.isRed()) {
            return Field.normalize(redHeading);
        }
        return bluePinned ? Field.normalize(blueHeading) : Field.SYMMETRY.heading(redHeading);
    }

    public Pose pose(Alliance alliance) {
        return new Pose(x(alliance), y(alliance), heading(alliance));
    }

    /** Inches to another waypoint, both resolved for {@code alliance}. */
    public double distanceTo(Waypoint other, Alliance alliance) {
        return Math.hypot(x(alliance) - other.x(alliance), y(alliance) - other.y(alliance));
    }

    /** Inches from a robot pose to this waypoint on {@code alliance}. */
    public double distanceTo(Pose pose, Alliance alliance) {
        return Math.hypot(x(alliance) - pose.getX(), y(alliance) - pose.getY());
    }

    /** Inches between the pinned blue value and the one {@link Field#SYMMETRY} would give. */
    public double blueDriftInches() {
        if (!bluePinned) {
            return 0;
        }
        return Math.hypot(
                blueX - Field.SYMMETRY.x(redX, redY), blueY - Field.SYMMETRY.y(redX, redY));
    }

    @Override
    public String toString() {
        return name + (bluePinned ? String.format("[blue %+.1fin]", blueDriftInches()) : "");
    }
}
