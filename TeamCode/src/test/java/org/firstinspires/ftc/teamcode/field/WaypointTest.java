package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Waypoint resolution is pure math, so it is tested on a laptop with no robot and no SDK. */
class WaypointTest {

    private static final double EPS = 1e-9;

    @Test
    void redReturnsAuthoredValues() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225);
        assertEquals(119.380, w.x(Alliance.RED), EPS);
        assertEquals(128.800, w.y(Alliance.RED), EPS);
        assertEquals(Math.toRadians(-135), w.heading(Alliance.RED), EPS);
    }

    @Test
    void blueMirrorsRedByDefault() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 30);
        assertEquals(144.0 - 119.380, w.x(Alliance.BLUE), EPS);
        assertEquals(128.800, w.y(Alliance.BLUE), EPS, "mirror is across x only; y is unchanged");
        assertEquals(Math.toRadians(150), w.heading(Alliance.BLUE), EPS);
        assertFalse(w.isBluePinned());
        assertEquals(0, w.blueDriftInches(), EPS);
    }

    @Test
    void mirroringTwiceIsIdentity() {
        for (double x : new double[] {0, 12.5, 72, 119.38, 144}) {
            assertEquals(x, Field.mirrorX(Field.mirrorX(x)), EPS);
        }
        for (double deg : new double[] {-179, -90, 0, 30, 135, 179}) {
            double rad = Math.toRadians(deg);
            assertEquals(rad, Field.mirrorHeading(Field.mirrorHeading(rad)), EPS);
        }
    }

    @Test
    void pinnedBlueWinsOverTheMirror() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(27.0, 130.0, -40);

        assertEquals(27.0, w.x(Alliance.BLUE), EPS);
        assertEquals(130.0, w.y(Alliance.BLUE), EPS);
        assertEquals(Math.toRadians(-40), w.heading(Alliance.BLUE), EPS);
        assertTrue(w.isBluePinned());
    }

    @Test
    void pinningBlueLeavesRedAlone() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(27.0, 130.0, -40);

        assertEquals(119.380, w.x(Alliance.RED), EPS);
        assertEquals(128.800, w.y(Alliance.RED), EPS);
        assertEquals(Math.toRadians(-135), w.heading(Alliance.RED), EPS);
    }

    @Test
    void waypointsAreImmutable() {
        Waypoint red = Waypoint.red("score", 119.380, 128.800, 225);
        Waypoint pinned = red.blue(27.0, 130.0, -40);

        assertNotSame(red, pinned);
        assertFalse(red.isBluePinned(), "blue() must not mutate the original");
        assertEquals(144.0 - 119.380, red.x(Alliance.BLUE), EPS);
    }

    @Test
    void driftMeasuresDistanceFromThePureMirror() {
        // Pure mirror of x=119.380 is 24.620. Pin blue 3 inches further out and 4 up.
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225).blue(21.620, 132.800, 225);
        assertEquals(5.0, w.blueDriftInches(), 1e-9, "3-4-5 triangle");
    }

    @Test
    void headingsAreAlwaysWrapped() {
        Waypoint w = Waypoint.red("spin", 10, 10, 720 + 45);
        assertEquals(Math.toRadians(45), w.heading(Alliance.RED), EPS);
        assertTrue(w.heading(Alliance.BLUE) >= -Math.PI && w.heading(Alliance.BLUE) < Math.PI);
    }

    @Test
    void poseCarriesTheResolvedValues() {
        Waypoint w = Waypoint.red("score", 119.380, 128.800, 225);
        assertEquals(144.0 - 119.380, w.pose(Alliance.BLUE).getX(), EPS);
        assertEquals(119.380, w.pose(Alliance.RED).getX(), EPS);
    }

    /**
     * Regression guard using real numbers from the 2025 TopRed12 / TopBlue12 paths. Those two files
     * were hand-tuned apart, and this is the shape the prefab has to be able to express: mirror by
     * default, one waypoint pinned, red untouched.
     */
    @Test
    void expressesLastSeasonsPerSideTuning() {
        Waypoint scorePreload =
                Waypoint.red("scorePreload", 99.533, 98.933, 240).blue(53.000, 94.000, 240);

        assertEquals(99.533, scorePreload.x(Alliance.RED), EPS);
        assertEquals(53.000, scorePreload.x(Alliance.BLUE), EPS);
        // Pure mirror would have put blue at 44.467; the real robot needed 53.0.
        assertEquals(44.467, Field.mirrorX(99.533), 1e-9);
        assertTrue(scorePreload.blueDriftInches() > 8.0);
    }
}
