package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathChain;

import org.junit.jupiter.api.Test;

/**
 * Route leg geometry and per-leg options.
 *
 * <p>A {@link Route} passes its follower straight to Pedro's builder, which only stores it, so a
 * null follower is enough to build and inspect a whole path off-robot.
 */
class RouteTest {

    private static final double EPS = 1e-9;
    private static final double LOOSE = 1e-6;

    private static final Waypoint START = Waypoint.red("start", 100, 20, 90).blue(30, 20, 90);
    private static final Waypoint MID = Waypoint.red("mid", 100, 60, 90);
    private static final Waypoint TURN = Waypoint.red("turn", 80, 80, 180);
    private static final Waypoint VIA = Waypoint.red("via", 95, 75, 0);
    private static final Waypoint FAR = Waypoint.red("far", 60, 100, 270);

    private static Route route(Alliance alliance) {
        return Route.from(null, alliance, START);
    }

    private static Path only(PathChain chain) {
        assertEquals(1, chain.size(), "a leg is one path");
        return chain.getPath(0);
    }

    @Test
    void lineToRunsFromTheCursorToTheNextWaypoint() {
        Route route = route(Alliance.RED);

        Path leg = only(route.lineTo(MID));

        assertEquals(2, leg.getControlPoints().size(), "a line has two control points");
        assertEquals(100, leg.getFirstControlPoint().getX(), EPS);
        assertEquals(20, leg.getFirstControlPoint().getY(), EPS);
        assertEquals(60, leg.getLastControlPoint().getY(), EPS);
        assertEquals(MID, route.cursor(), "the cursor advances");
    }

    @Test
    void legsChainFromWhereTheLastOneEnded() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path second = only(route.lineTo(TURN));

        assertEquals(100, second.getFirstControlPoint().getX(), EPS);
        assertEquals(60, second.getFirstControlPoint().getY(), EPS);
    }

    @Test
    void aMatchingHeadingIsHeldConstant() {
        Path leg = only(route(Alliance.RED).lineTo(MID));

        assertEquals(Math.toRadians(90), leg.getHeadingGoal(0), LOOSE);
        assertEquals(Math.toRadians(90), leg.getHeadingGoal(0.5), LOOSE);
        assertEquals(Math.toRadians(90), leg.getHeadingGoal(1), LOOSE);
    }

    @Test
    void aChangingHeadingIsInterpolatedAcrossTheLeg() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = only(route.lineTo(TURN));

        assertEquals(Math.toRadians(90), leg.getHeadingGoal(0), LOOSE);
        assertEquals(Math.toRadians(180), leg.getHeadingGoal(1), LOOSE);
        double middle = leg.getHeadingGoal(0.5);
        assertTrue(
                middle > Math.toRadians(90) && middle < Math.toRadians(180),
                "midpoint heading was " + Math.toDegrees(middle));
    }

    @Test
    void curveToAddsItsControlPointsInOrder() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = only(route.curveTo(TURN, VIA));

        assertEquals(3, leg.getControlPoints().size());
        assertEquals(95, leg.getSecondControlPoint().getX(), EPS, "the via point shapes the curve");
        assertEquals(80, leg.getLastControlPoint().getX(), EPS);
    }

    /**
     * Why {@link Route} has no per-leg constraint option. Pedro's {@code PathChain} constructor
     * reassigns every path's constraints to the shared static default, so a constraint given to the
     * builder does not survive {@code build()}. If a later Pedro version fixes this, this test
     * fails and per-leg braking becomes worth adding.
     */
    @Test
    void pedroDiscardsPerPathConstraintsWhenTheChainIsBuilt() {
        PathConstraints own = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 0.65, 10, 1);

        PathChain chain =
                new PathBuilder(null, own)
                        .addPath(new BezierLine(new Pose(0, 0), new Pose(10, 0)))
                        .setConstantHeadingInterpolation(0)
                        .build();

        assertEquals(0.65, own.getBrakingStrength(), EPS, "the constraint object is correct");
        assertEquals(
                PathConstraints.defaultConstraints.getBrakingStrength(),
                chain.getPath(0).getBrakingStrength(),
                EPS,
                "but the built path carries the shared default instead");
    }

    @Test
    void reversedCurveToFollowsThePathTangentBackwards() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = only(route.reversedCurveTo(TURN, VIA));

        // Tangent heading, so the authored 180 degrees at TURN is not commanded on arrival.
        assertNotEquals(Math.toRadians(180), leg.getHeadingGoal(1), LOOSE);

        // Reversed: the heading is the travel direction turned through half a turn.
        double travel =
                Math.atan2(
                        leg.getSecondControlPoint().getY() - leg.getFirstControlPoint().getY(),
                        leg.getSecondControlPoint().getX() - leg.getFirstControlPoint().getX());
        assertEquals(
                0,
                Field.normalize(leg.getHeadingGoal(0) - (travel + Math.PI)),
                1e-3,
                "start heading points away from travel");
    }

    @Test
    void jumpToMovesTheCursorWithoutEmittingALeg() {
        Route route = route(Alliance.RED);

        route.jumpTo(TURN);
        Path leg = only(route.lineTo(FAR));

        assertEquals(80, leg.getFirstControlPoint().getX(), EPS, "starts at the jumped-to pose");
        assertEquals(java.util.Arrays.asList("start", "turn", "far"), route.visited());
    }

    @Test
    void legsResolveForTheAlliance() {
        Path red = only(route(Alliance.RED).lineTo(MID));
        Path blue = only(route(Alliance.BLUE).lineTo(MID));

        assertEquals(100, red.getFirstControlPoint().getX(), EPS);
        assertEquals(30, blue.getFirstControlPoint().getX(), EPS, "blue is pinned on START");
        assertEquals(
                Field.SYMMETRY.x(100, 60),
                blue.getLastControlPoint().getX(),
                EPS,
                "MID has no pin, so blue mirrors");
    }
}
