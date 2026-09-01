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
 * Builds path legs from named waypoints. Each leg starts where the previous one ended. Heading
 * interpolation comes from the waypoints: constant when the two headings match, linear otherwise.
 *
 * <p>Legs are static geometry. Build them at init, not per loop.
 *
 * <p>Per-leg path constraints are deliberately absent. Pedro 2.0.1 overwrites every path's
 * constraints with the shared static {@code PathConstraints.defaultConstraints} inside
 * {@code PathChain}'s constructor, so a constraint handed to the builder does not survive
 * {@code build()}, and {@code PathBuilder.setBrakingStrength} only appears to work because it
 * mutates that shared instance, changing braking for every path in the process.
 *
 * <pre>{@code
 * Route route = drive.route(alliance, Waypoints.START);
 * Command auto = new SequentialCommandGroup(
 *         drive.follow(route.lineTo(Waypoints.SCORE)),
 *         drive.follow(route.curveTo(Waypoints.GATE, Waypoints.GATE_CONTROL)),
 *         drive.follow(route.lineTo(Waypoints.PICKUP)));
 * }</pre>
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

    /** Waypoint names in visit order. */
    public List<String> visited() {
        return new ArrayList<>(visited);
    }

    public PathChain lineTo(Waypoint next) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);
        PathBuilder builder = newBuilder().addPath(new BezierLine(from, to));
        return finish(builder, from, to, next);
    }

    /**
     * Curve to {@code next} through the given control points. Control points shape the path and are
     * not poses the robot holds, so their headings are unused; they are {@link Waypoint}s so that
     * they mirror with the rest of the route.
     */
    public PathChain curveTo(Waypoint next, Waypoint... via) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);

        List<Pose> points = new ArrayList<>();
        points.add(from);
        for (Waypoint control : via) {
            points.add(control.pose(alliance));
        }
        points.add(to);

        PathBuilder builder = newBuilder().addPath(new BezierCurve(points));
        return finish(builder, from, to, next);
    }

    /**
     * Curve to {@code next} driving backwards, with the heading following the path tangent. Use
     * where the robot should retreat along a curve it just drove, rather than turn around; the
     * waypoint's own heading is then only the starting heading for the following leg.
     */
    public PathChain reversedCurveTo(Waypoint next, Waypoint... via) {
        Pose from = cursor.pose(alliance);
        Pose to = next.pose(alliance);

        List<Pose> points = new ArrayList<>();
        points.add(from);
        for (Waypoint control : via) {
            points.add(control.pose(alliance));
        }
        points.add(to);

        PathBuilder builder =
                newBuilder()
                        .addPath(new BezierCurve(points))
                        .setTangentHeadingInterpolation()
                        .setReversed();
        cursor = next;
        visited.add(next.name());
        return builder.build();
    }

    /** Moves the cursor without emitting a leg, after the robot is repositioned by other means. */
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

    private PathBuilder newBuilder() {
        return new PathBuilder(follower);
    }

    @Override
    public String toString() {
        return alliance + " route " + Arrays.toString(visited.toArray());
    }
}
