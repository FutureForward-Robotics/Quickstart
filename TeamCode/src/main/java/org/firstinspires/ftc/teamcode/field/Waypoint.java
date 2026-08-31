package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/**
 * A named field pose, authored in RED coordinates.
 *
 * <p><b>Why this exists.</b> In the 2025 auto folder, 501 {@code Pose} literals collapsed to 249
 * distinct poses: half of every coordinate in the autos was retyped. Nine of eleven path seams in
 * {@code TopRed12Paths} retyped the previous path's endpoint as the next path's start. Multiply by
 * red and blue and one tuned scoring pose became four hand edits, which is why the two sides drifted
 * apart. Name the pose once and that whole class of drift disappears.
 *
 * <p><b>Blue.</b> Defaults to the mirror of red. When the robot genuinely behaves differently on the
 * other side, call {@link #blue} to pin that one waypoint; the override sits next to the value it
 * replaces. Waypoints are immutable, so {@code blue(...)} returns a new instance.
 *
 * <pre>{@code
 * public final class Waypoints {
 *     public static final Waypoint START = Waypoint.red("start", 119.380, 128.800, 225);
 *
 *     public static final Waypoint SCORE = Waypoint.red("score", 99.533, 98.933, 240)
 *             .blue(97.500, 100.200, 238);   // robot pulls left on blue
 * }
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

    /** Author a waypoint in red coordinates. Heading in degrees. */
    public static Waypoint red(String name, double x, double y, double headingDeg) {
        return new Waypoint(name, x, y, Math.toRadians(headingDeg), false, 0, 0, 0);
    }

    /** Pin this waypoint's blue value instead of using the mirror. Heading in degrees. */
    public Waypoint blue(double x, double y, double headingDeg) {
        return new Waypoint(
                name, redX, redY, redHeading, true, x, y, Math.toRadians(headingDeg));
    }

    public String name() {
        return name;
    }

    /** True if blue was pinned by hand rather than mirrored. */
    public boolean isBluePinned() {
        return bluePinned;
    }

    public double x(Alliance alliance) {
        if (alliance.isRed()) {
            return redX;
        }
        return bluePinned ? blueX : Field.mirrorX(redX);
    }

    public double y(Alliance alliance) {
        if (alliance.isRed()) {
            return redY;
        }
        return bluePinned ? blueY : redY;
    }

    /** Radians, wrapped to [-pi, pi). */
    public double heading(Alliance alliance) {
        if (alliance.isRed()) {
            return Field.normalize(redHeading);
        }
        return bluePinned ? Field.normalize(blueHeading) : Field.mirrorHeading(redHeading);
    }

    /** The one place this class touches Pedro. */
    public Pose pose(Alliance alliance) {
        return new Pose(x(alliance), y(alliance), heading(alliance));
    }

    /**
     * How far a pinned blue value sits from the pure mirror, in inches. Zero when not pinned.
     *
     * <p>Print these during tuning. Last season's median was 4.2 inches; anything much larger is
     * more likely a typo than a real per-side difference.
     */
    public double blueDriftInches() {
        if (!bluePinned) {
            return 0;
        }
        return Math.hypot(blueX - Field.mirrorX(redX), blueY - redY);
    }

    @Override
    public String toString() {
        return name + (bluePinned ? String.format("[blue pinned %+.1fin]", blueDriftInches()) : "");
    }
}
