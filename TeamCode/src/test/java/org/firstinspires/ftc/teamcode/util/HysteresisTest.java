package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HysteresisTest {

    @Test
    void risesAtTheHighThresholdAndFallsAtTheLow() {
        Hysteresis h = new Hysteresis(2.0, 8.0);

        assertFalse(h.calculate(0));
        assertFalse(h.calculate(7.9), "below high, still low");
        assertTrue(h.calculate(8.0), "reaches high");
        assertTrue(h.calculate(2.1), "inside the band, holds high");
        assertFalse(h.calculate(2.0), "reaches low");
    }

    @Test
    void holdsThroughTheBandInBothDirections() {
        Hysteresis h = new Hysteresis(2.0, 8.0);

        h.calculate(9);
        for (double v = 8; v >= 2.5; v -= 0.5) {
            assertTrue(h.calculate(v), "still high at " + v);
        }

        h.calculate(1);
        for (double v = 2; v <= 7.5; v += 0.5) {
            assertFalse(h.calculate(v), "still low at " + v);
        }
    }

    /** The reason to use it: a signal sitting on one threshold must not chatter. */
    @Test
    void doesNotChatterAroundASinglePoint() {
        Hysteresis h = new Hysteresis(2.0, 8.0);
        h.calculate(9);

        int transitions = 0;
        boolean previous = h.get();
        double[] noisy = {5.1, 4.9, 5.2, 4.8, 5.0, 5.3, 4.7};
        for (double v : noisy) {
            boolean now = h.calculate(v);
            if (now != previous) {
                transitions++;
            }
            previous = now;
        }

        assertTrue(h.get(), "noise in the band never crossed either threshold");
        assertFalse(transitions > 0, "no transitions, got " + transitions);
    }

    @Test
    void initialStateIsSettable() {
        assertTrue(new Hysteresis(2.0, 8.0, true).get());
        assertTrue(new Hysteresis(2.0, 8.0, true).calculate(5.0), "holds high through the band");
        assertFalse(new Hysteresis(2.0, 8.0, false).calculate(5.0));
    }

    @Test
    void equalThresholdsBehaveAsAPlainComparison() {
        Hysteresis h = new Hysteresis(5.0, 5.0);

        assertTrue(h.calculate(5.0));
        assertFalse(h.calculate(5.0), "at the threshold it can fall again");
        assertTrue(h.calculate(6.0));
        assertFalse(h.calculate(4.0));
    }

    @Test
    void invertedThresholdsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new Hysteresis(8.0, 2.0));
    }
}
