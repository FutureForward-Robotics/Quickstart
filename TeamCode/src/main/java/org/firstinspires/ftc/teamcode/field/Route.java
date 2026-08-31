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
 * Builds path legs from named waypoints. Each leg starts where the previous one ended, so seams
 * are not retyped. Heading interpolation is taken from the waypoints: constant when the two
 * headings match, linear otherwise.
 *
 * <p>Legs are static geometry. Build them at init, not per loop.
 *
 * <pre>{@code
 * Route route = drive.route(alliance, Waypoints.START);
 * Command auto = new SequentialCommandGroup(
 *         drive.follow(route.lineTo(Waypoints.SCORE)),
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
        PathBuilder builder = new PathBuilder(follower).addPath(new BezierLine(from, to));
        return finish(builder, from, to, next);
    }

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

    @Override
    public String toString() {
        return alliance + " route " + Arrays.toString(visited.toArray());
    }
}
