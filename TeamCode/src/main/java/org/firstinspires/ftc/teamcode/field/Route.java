package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Emits path legs from named {@link Waypoint}s, carrying a cursor from one leg to the next.
 *
 * <p><b>The cursor is the point.</b> Each leg starts where the previous one ended, so a seam cannot
 * be retyped and therefore cannot drift. That single rule removes the 50% of coordinate literals
 * that were duplicates in the 2025 autos.
 *
 * <p>Heading interpolation is derived from the waypoints -- constant when the two headings match,
 * linear otherwise -- so headings stop being retyped on every leg too.
 *
 * <pre>{@code
 * Route route = Route.from(follower, alliance, Waypoints.START);
 * Command auto = new SequentialCommandGroup(
 *         drive.follow(route.lineTo(Waypoints.SCORE)),
 *         drive.follow(route.lineTo(Waypoints.PICKUP)),
 *         drive.follow(route.curveTo(Waypoints.GATE_CONTROL, Waypoints.GATE)));
 * }</pre>
 *
 * <p>Legs are geometry, so build them once at OpMode init, not per loop.
 */
public final class Route {

    private static final double HEADING_EPSILON_RAD = 1e-6;

    private final Follower follower;
    private final Alliance alliance;
    private final List<String> visited = new ArrayList<>();
    private Waypoint cursor;

    private Route(Follower follower, Alliance alliance, Waypoint start) {
        this.follower = follower;
        this.alliance = alliance;
        this.cursor = start;
        visited.add(start.name());
    }

    public static Route from(Follower follower, Alliance alliance, Waypoint start) {
        return new Route(follower, alliance, start);
    }

    public Alliance alliance() {
        return alliance;
    }

    public Waypoint cursor() {
        return cursor;
    }

    /** Waypoint names in the order this route visited them. Useful in telemetry. */
    public List<String> visited() {
        return new ArrayList<>(visited);
    }

    /** Straight leg from the cursor to {@code next}. */
    public PathChain lineTo(Waypoint next) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);
        PathBuilder builder = new PathBuilder(follower).addPath(new BezierLine(from, to));
        return finish(builder, from, to, next);
    }

    /** Curved leg from the cursor through the given control points to {@code next}. */
    public PathChain curveTo(Waypoint control, Waypoint next, Waypoint... moreControls) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);

        List<Pose> points = new ArrayList<>();
        points.add(from);
        points.add(control.pose(alliance));
        for (Waypoint extra : moreControls) {
            points.add(extra.pose(alliance));
        }
        points.add(to);

        PathBuilder builder = new PathBuilder(follower).addPath(new BezierCurve(points));
        return finish(builder, from, to, next);
    }

    /**
     * Move the cursor without emitting a leg. Use after the robot is repositioned by something
     * other than a path, so the next leg still starts from the truth.
     */
    public Route jumpTo(Waypoint next) {
        cursor = next;
        visited.add(next.name());
        return this;
    }

    private PathChain finish(PathBuilder builder, Pose from, Pose to, Waypoint next) {
        if (Math.abs(Field.normalize(to.getHeading() - from.getHeading())) < HEADING_EPSILON_RAD) {
            builder.setConstantHeadingInterpolation(from.getHeading());
        } else {
            builder.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        }
        cursor = next;
        visited.add(next.name());
        return builder.build();
    }

    @Override
    public String toString() {
        return alliance + " route " + Arrays.toString(visited.toArray());
    }
}
