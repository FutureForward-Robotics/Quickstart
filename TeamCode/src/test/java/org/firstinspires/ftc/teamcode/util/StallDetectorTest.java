package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class StallDetectorTest {

    private long nanos = 500_000_000_000L;
    private double amps;

    private StallDetector detector(double threshold, double forSeconds) {
        return new StallDetector(() -> amps, threshold, forSeconds, () -> nanos);
    }

    private void advance(double seconds) {
        nanos += (long) (seconds * 1e9);
    }

    @Test
    void aCurrentSpikeShorterThanTheWindowIsNotAStall() {
        StallDetector detector = detector(4.0, 0.5);

        amps = 6.0;
        detector.update();
        advance(0.3);
        assertFalse(detector.update(), "0.3s of high current is a direction change");

        amps = 1.0;
        advance(0.1);
        assertFalse(detector.update());
    }

    @Test
    void sustainedCurrentIsAStall() {
        StallDetector detector = detector(4.0, 0.5);

        amps = 6.0;
        detector.update();
        advance(0.6);

        assertTrue(detector.update());
        assertTrue(detector.isStalled(), "state is readable without re-sampling");
    }

    @Test
    void clearsOnceCurrentDrops() {
        StallDetector detector = detector(4.0, 0.5);
        amps = 6.0;
        detector.update();
        advance(0.6);
        assertTrue(detector.update());

        amps = 1.0;
        assertFalse(detector.update(), "falling edge is not delayed");
    }

    @Test
    void triggersExactlyAtTheThreshold() {
        StallDetector detector = detector(4.0, 0.0);
        amps = 3.999;
        assertFalse(detector.update());
        amps = 4.0;
        assertTrue(detector.update());
    }
}
