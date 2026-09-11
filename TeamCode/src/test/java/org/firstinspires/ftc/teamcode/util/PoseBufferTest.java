package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.pedropathing.math.Pose;

import org.junit.jupiter.api.Test;

/** PoseBuffer: interpolation, clamping, wrap-aware heading and overwrite. */
class PoseBufferTest {

    private static final long MS = 1_000_000L;
    private static final double EPS = 1e-9;

    @Test
    void anEmptyBufferHasNothingToSay() {
        assertNull(new PoseBuffer().at(0));
    }

    @Test
    void interpolatesBetweenTheStraddlingSamples() {
        PoseBuffer buffer = new PoseBuffer();
        buffer.add(0, new Pose(0, 0, 0));
        buffer.add(20 * MS, new Pose(10, 20, 0));

        Pose middle = buffer.at(10 * MS);

        assertEquals(5, middle.x(), EPS);
        assertEquals(10, middle.y(), EPS);
    }

    @Test
    void clampsToTheEndsRatherThanExtrapolating() {
        PoseBuffer buffer = new PoseBuffer();
        buffer.add(10 * MS, new Pose(1, 1, 0));
        buffer.add(20 * MS, new Pose(2, 2, 0));

        assertEquals(1, buffer.at(0).x(), EPS, "before the oldest sample");
        assertEquals(2, buffer.at(50 * MS).x(), EPS, "after the newest");
    }

    @Test
    void headingInterpolationTakesTheShortWayRound() {
        PoseBuffer buffer = new PoseBuffer();
        buffer.add(0, new Pose(0, 0, Math.toRadians(170)));
        buffer.add(20 * MS, new Pose(0, 0, Math.toRadians(-170)));

        // 170 to -170 is 20 degrees forward through 180, not 340 degrees back.
        assertEquals(Math.toRadians(180), Math.abs(buffer.at(10 * MS).heading()), 1e-9);
    }

    @Test
    void oldSamplesAreOverwrittenAndTheWindowStillReads() {
        PoseBuffer buffer = new PoseBuffer(4);
        for (int i = 0; i < 10; i++) {
            buffer.add(i * 20 * MS, new Pose(i, 0, 0));
        }

        assertEquals(4, buffer.size());
        assertEquals(6, buffer.at(6 * 20 * MS).x(), EPS, "oldest retained sample");
        assertEquals(9, buffer.at(9 * 20 * MS).x(), EPS, "newest");
        assertEquals(6, buffer.at(0).x(), EPS, "a time older than the window clamps");
    }

    @Test
    void aCapacityBelowTwoCannotInterpolate() {
        assertThrows(IllegalArgumentException.class, () -> new PoseBuffer(1));
    }
}
