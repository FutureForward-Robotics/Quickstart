package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.config.Modifier;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.bezier.BezierCurve;

import org.junit.jupiter.api.Test;

/**
 * Route leg geometry and per-leg options.
 *
 * <p>A leg is a {@link Path} and carries no follower, so a whole route can be built and inspected
 * off-robot.
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
        return Route.from(alliance, START);
    }

    @Test
    void lineToRunsFromTheCursorToTheNextWaypoint() {
        Route route = route(Alliance.RED);

        Path leg = route.lineTo(MID);

        assertEquals(100, leg.curve.startPoint().x(), EPS);
        assertEquals(20, leg.curve.startPoint().y(), EPS);
        assertEquals(60, leg.curve.endPoint().y(), EPS);
        assertEquals(MID, route.cursor(), "the cursor advances");
    }

    @Test
    void legsChainFromWhereTheLastOneEnded() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path second = route.lineTo(TURN);

        assertEquals(100, second.curve.startPoint().x(), EPS);
        assertEquals(60, second.curve.startPoint().y(), EPS);
    }

    @Test
    void aMatchingHeadingIsHeldConstant() {
        Path leg = route(Alliance.RED).lineTo(MID);

        assertEquals(Math.toRadians(90), leg.heading(0), LOOSE);
        assertEquals(Math.toRadians(90), leg.heading(0.5), LOOSE);
        assertEquals(Math.toRadians(90), leg.heading(1), LOOSE);
    }

    @Test
    void aChangingHeadingIsInterpolatedAcrossTheLeg() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = route.lineTo(TURN);

        assertEquals(Math.toRadians(90), leg.heading(0), LOOSE);
        assertEquals(Math.toRadians(180), leg.heading(1), LOOSE);
        double middle = leg.heading(0.5);
        assertTrue(
                middle > Math.toRadians(90) && middle < Math.toRadians(180),
                "midpoint heading was " + Math.toDegrees(middle));
    }

    @Test
    void curveToAddsItsControlPointsInOrder() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = route.curveTo(TURN, VIA);
        BezierCurve curve = (BezierCurve) leg.curve;

        assertEquals(3, curve.getControlPoints().size());
        assertEquals(95, curve.getControlPoints().get(1).x(), EPS, "the via point shapes the curve");
        assertEquals(80, curve.endPoint().x(), EPS);
    }

    /**
     * Why {@link Route} can offer per-leg limits: a {@code ConfigVar} hands out a {@link Modifier}
     * that sets the value while a path runs and puts the old one back afterwards, so one leg's cap
     * does not leak into the next. Pedro 2.x had no such mechanism; a per-path constraint object
     * was overwritten by a shared default when the chain was built.
     */
    @Test
    void aPerLegLimitAppliesAndThenReverts() {
        ForesightConfig config = new ForesightConfig(c -> c.maxPathSpeed.set(80.0));

        Modifier slowLeg = config.maxPathSpeed.at(20.0);
        assertEquals(80.0, config.maxPathSpeed.get(), EPS);

        slowLeg.apply();
        assertEquals(20.0, config.maxPathSpeed.get(), EPS);

        slowLeg.revert();
        assertEquals(80.0, config.maxPathSpeed.get(), EPS, "the next leg is unaffected");
    }

    @Test
    void reversedCurveToFollowsThePathTangentBackwards() {
        Route route = route(Alliance.RED);
        route.lineTo(MID);

        Path leg = route.reversedCurveTo(TURN, VIA);

        // Tangent heading, so the authored 180 degrees at TURN is not commanded on arrival.
        assertNotEquals(Math.toRadians(180), leg.heading(1), LOOSE);

        // Reversed: the heading is the travel direction turned through half a turn.
        double travel = leg.curve.tangent(0).theta();
        assertEquals(
                0,
                Field.normalize(leg.heading(0) - (travel + Math.PI)),
                1e-3,
                "start heading points away from travel");
    }

    @Test
    void jumpToMovesTheCursorWithoutEmittingALeg() {
        Route route = route(Alliance.RED);

        route.jumpTo(TURN);
        Path leg = route.lineTo(FAR);

        assertEquals(80, leg.curve.startPoint().x(), EPS, "starts at the jumped-to pose");
        assertEquals(java.util.Arrays.asList("start", "turn", "far"), route.visited());
    }

    @Test
    void legsResolveForTheAlliance() {
        Path red = route(Alliance.RED).lineTo(MID);
        Path blue = route(Alliance.BLUE).lineTo(MID);

        assertEquals(100, red.curve.startPoint().x(), EPS);
        assertEquals(30, blue.curve.startPoint().x(), EPS, "blue is pinned on START");
        assertEquals(
                Field.SYMMETRY.x(100, 60),
                blue.curve.endPoint().x(),
                EPS,
                "MID has no pin, so blue mirrors");
    }

    @Test
    void continuousToKeepsEveryLegsHeadingInOneUnbrokenPath() {
        Route route = route(Alliance.RED);

        Path path = route.continuousTo(MID, TURN);

        assertEquals(2, path.getSegments().size(), "one segment per leg");
        assertEquals(Math.toRadians(90), path.heading(0), LOOSE, "starts on START's heading");
        assertEquals(Math.toRadians(180), path.heading(1), LOOSE, "ends on TURN's heading");
        assertEquals(80, path.endPose().x(), EPS);
        assertEquals(80, path.endPose().y(), EPS);
        assertEquals(java.util.Arrays.asList("start", "mid", "turn"), route.visited());
        assertEquals(TURN, route.cursor());
    }
}
