package org.firstinspires.ftc.teamcode.field;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;

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

        assertEquals(Field.WIDTH_IN / 2, pose.x(), EPS);
        assertEquals(Field.WIDTH_IN / 2, pose.y(), EPS);
    }

    @Test
    void metresAreConvertedToInches() {
        // One metre from the centre along each axis is 39.3701 inches.
        Pose pose = Field.toPedro(meters(1, -1), 0);

        assertEquals(72 + 39.3700787401, pose.x(), 1e-6);
        assertEquals(72 - 39.3700787401, pose.y(), 1e-6);
    }

    @Test
    void aPositionAlreadyInInchesIsNotRescaled() {
        Pose pose = Field.toPedro(new Position(DistanceUnit.INCH, 12, -24, 0, 0), 0);

        assertEquals(84, pose.x(), EPS);
        assertEquals(48, pose.y(), EPS);
    }

    @Test
    void theFarCornersLandOnPedrosBounds() {
        Position corner = new Position(DistanceUnit.INCH, -72, -72, 0, 0);
        Position opposite = new Position(DistanceUnit.INCH, 72, 72, 0, 0);

        assertEquals(0, Field.toPedro(corner, 0).x(), EPS);
        assertEquals(0, Field.toPedro(corner, 0).y(), EPS);
        assertEquals(Field.WIDTH_IN, Field.toPedro(opposite, 0).x(), EPS);
        assertEquals(Field.WIDTH_IN, Field.toPedro(opposite, 0).y(), EPS);
    }

    @Test
    void yawIsDegreesInAndRadiansOut() {
        assertEquals(Math.toRadians(90), Field.toPedro(meters(0, 0), 90).heading(), EPS);

        // Pedro 3 stores a Pose heading in [0, 2pi), so -45 comes back as 315. Code that cares
        // about the sign takes the difference through Field.normalize, which is range-agnostic.
        assertEquals(Math.toRadians(315), Field.toPedro(meters(0, 0), -45).heading(), EPS);
        assertEquals(
                Math.toRadians(-45),
                Field.normalize(Field.toPedro(meters(0, 0), -45).heading()),
                EPS);
    }

    @Test
    void yawIsWrappedIntoPedrosRange() {
        // The camera reports (-180, 180], but a wrapped value is what every consumer assumes.
        assertEquals(0, Field.toPedro(meters(0, 0), 360).heading(), 1e-9);
        assertEquals(Math.toRadians(270), Field.toPedro(meters(0, 0), 270).heading(), 1e-9);
    }
}
