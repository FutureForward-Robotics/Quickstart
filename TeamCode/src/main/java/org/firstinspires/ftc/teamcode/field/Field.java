package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
    /**
     * Wraps an angle into [-pi, pi). This is the convention for a heading <em>difference</em>: the
     * sign is the direction to turn and the magnitude is the shortest path.
     *
     * <p>Pedro stores an absolute heading in a {@code Pose} as [0, 2pi) instead, so {@code
     * pose.heading()} is never negative. Normalizing a value before handing it to a {@code Pose}
     * does nothing, because the constructor re-wraps it. Pedro's own {@code Angle} class keeps both
     * ranges for the same reason, and its follower computes every error with the signed one.
     */
    public static double normalize(double rad) {
        double a = (rad + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }

    /**
     * Converts an FTC field pose to Pedro's frame: centre origin to corner origin, the position's
     * own {@link DistanceUnit} to inches, and degrees to radians.
     *
     * <p>Assumes FTC +X is Pedro +X and FTC +Y is Pedro +Y. Check that once a season by standing
     * the robot on a known spot and comparing this against odometry; a mismatch means the team's
     * Pedro frame is rotated relative to the field frame.
     */
    public static Pose toPedro(Position position, double yawDegrees) {
        Position inches = position.toUnit(DistanceUnit.INCH);
        return new Pose(
                inches.x + WIDTH_IN / 2,
                inches.y + WIDTH_IN / 2,
                normalize(Math.toRadians(yawDegrees)));
    }
}
