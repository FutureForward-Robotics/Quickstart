package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.junit.jupiter.api.Test;

/** The FTC-to-Pedro frame conversion: centre origin to corner origin, any unit to inches. */
class FieldTest {

    private static final double EPS = 1e-9;

    private static Position meters(double x, double y) {
        return new Position(DistanceUnit.METER, x, y, 0, 0);
    }

    @Test
    void theFieldCentreIsTheMiddleOfPedrosFrame() {
        Pose pose = Field.toPedro(meters(0, 0), 0);

        assertEquals(Field.WIDTH_IN / 2, pose.getX(), EPS);
        assertEquals(Field.WIDTH_IN / 2, pose.getY(), EPS);
    }

    @Test
    void metresAreConvertedToInches() {
        // One metre from the centre along each axis is 39.3701 inches.
        Pose pose = Field.toPedro(meters(1, -1), 0);

        assertEquals(72 + 39.3700787401, pose.getX(), 1e-6);
        assertEquals(72 - 39.3700787401, pose.getY(), 1e-6);
    }

    @Test
    void aPositionAlreadyInInchesIsNotRescaled() {
        Pose pose = Field.toPedro(new Position(DistanceUnit.INCH, 12, -24, 0, 0), 0);

        assertEquals(84, pose.getX(), EPS);
        assertEquals(48, pose.getY(), EPS);
    }

    @Test
    void theFarCornersLandOnPedrosBounds() {
        Position corner = new Position(DistanceUnit.INCH, -72, -72, 0, 0);
        Position opposite = new Position(DistanceUnit.INCH, 72, 72, 0, 0);

        assertEquals(0, Field.toPedro(corner, 0).getX(), EPS);
        assertEquals(0, Field.toPedro(corner, 0).getY(), EPS);
        assertEquals(Field.WIDTH_IN, Field.toPedro(opposite, 0).getX(), EPS);
        assertEquals(Field.WIDTH_IN, Field.toPedro(opposite, 0).getY(), EPS);
    }

    @Test
    void yawIsDegreesInAndRadiansOut() {
        assertEquals(Math.toRadians(90), Field.toPedro(meters(0, 0), 90).getHeading(), EPS);
        assertEquals(Math.toRadians(-45), Field.toPedro(meters(0, 0), -45).getHeading(), EPS);
    }

    @Test
    void yawIsWrappedIntoPedrosRange() {
        // The camera reports (-180, 180], but a wrapped value is what every consumer assumes.
        assertEquals(0, Field.toPedro(meters(0, 0), 360).getHeading(), 1e-9);
        assertEquals(Math.toRadians(-90), Field.toPedro(meters(0, 0), 270).getHeading(), 1e-9);
    }
}
