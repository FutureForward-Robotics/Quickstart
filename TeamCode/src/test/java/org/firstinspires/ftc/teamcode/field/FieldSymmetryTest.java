package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class FieldSymmetryTest {

    private static final double EPS = 1e-9;

    @ParameterizedTest
    @EnumSource(FieldSymmetry.class)
    void appliedTwiceIsIdentity(FieldSymmetry symmetry) {
        double[][] points = {{0, 0}, {12.5, 40}, {72, 72}, {119.38, 128.8}, {144, 144}};
        for (double[] p : points) {
            double x = symmetry.x(p[0], p[1]);
            double y = symmetry.y(p[0], p[1]);
            assertEquals(p[0], symmetry.x(x, y), EPS);
            assertEquals(p[1], symmetry.y(x, y), EPS);
        }
        for (double deg : new double[] {-179, -90, 0, 30, 135, 179}) {
            double rad = Math.toRadians(deg);
            assertEquals(rad, symmetry.heading(symmetry.heading(rad)), EPS);
        }
    }

    @ParameterizedTest
    @EnumSource(FieldSymmetry.class)
    void headingStaysWrapped(FieldSymmetry symmetry) {
        for (double deg = -180; deg < 180; deg += 7) {
            double out = symmetry.heading(Math.toRadians(deg));
            assertEquals(true, out >= -Math.PI && out < Math.PI, "wrapped for " + deg);
        }
    }

    @Test
    void mirrorXReflectsAcrossTheVerticalCentreLine() {
        FieldSymmetry s = FieldSymmetry.MIRROR_X;
        assertEquals(24.62, s.x(119.38, 128.8), 1e-9);
        assertEquals(128.8, s.y(119.38, 128.8), EPS);
        assertEquals(Math.toRadians(150), s.heading(Math.toRadians(30)), EPS);
    }

    @Test
    void mirrorYReflectsAcrossTheHorizontalCentreLine() {
        FieldSymmetry s = FieldSymmetry.MIRROR_Y;
        assertEquals(119.38, s.x(119.38, 128.8), EPS);
        assertEquals(15.2, s.y(119.38, 128.8), 1e-9);
        assertEquals(Math.toRadians(-30), s.heading(Math.toRadians(30)), EPS);
    }

    @Test
    void rotationalTurnsThePoseThroughHalfATurn() {
        FieldSymmetry s = FieldSymmetry.ROTATIONAL;
        assertEquals(24.62, s.x(119.38, 128.8), 1e-9);
        assertEquals(15.2, s.y(119.38, 128.8), 1e-9);
        assertEquals(Math.toRadians(-150), s.heading(Math.toRadians(30)), EPS);
    }

    /** A reflection swaps handedness; a rotation does not. */
    @Test
    void reflectionsInvertTurnDirection() {
        double straightUp = Math.toRadians(90);
        double slightlyLeft = Math.toRadians(100);

        assertEquals(
                -10,
                Math.toDegrees(
                        Field.normalize(
                                FieldSymmetry.MIRROR_X.heading(slightlyLeft)
                                        - FieldSymmetry.MIRROR_X.heading(straightUp))),
                1e-9);

        assertEquals(
                10,
                Math.toDegrees(
                        Field.normalize(
                                FieldSymmetry.ROTATIONAL.heading(slightlyLeft)
                                        - FieldSymmetry.ROTATIONAL.heading(straightUp))),
                1e-9);
    }
}
