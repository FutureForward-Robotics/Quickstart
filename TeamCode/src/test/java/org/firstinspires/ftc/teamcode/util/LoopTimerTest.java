package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The clock is a parameter, so no Thread.sleep is needed. */
class LoopTimerTest {

    private static final long MS = 1_000_000L;
    private static final double EPS = 1e-9;

    @Test
    void firstTickReportsZeroNotWallClock() {
        LoopTimer t = new LoopTimer();
        t.tick(1_234_567_890_123L); // boot-relative nanoTime, as on Android
        assertEquals(0.0, t.dtSeconds(), EPS);
    }

    @Test
    void measuresElapsedTimeBetweenTicks() {
        LoopTimer t = new LoopTimer();
        t.tick(0);
        t.tick(20 * MS);
        assertEquals(0.020, t.dtSeconds(), 1e-12);
        t.tick(45 * MS);
        assertEquals(0.025, t.dtSeconds(), 1e-12);
    }

    @Test
    void hzIsZeroRatherThanInfinityWhenTwoTicksCollide() {
        LoopTimer t = new LoopTimer();
        t.tick(500);
        t.tick(500); // same nanosecond
        assertEquals(0.0, t.dtSeconds(), EPS);
        assertEquals(0.0, t.hz(), EPS);
        assertFalse(Double.isInfinite(t.hz()));
    }

    @Test
    void hzIsTheReciprocalOfDt() {
        LoopTimer t = new LoopTimer();
        t.tick(0);
        t.tick(20 * MS);
        assertEquals(50.0, t.hz(), 1e-9);
    }

    @Test
    void throttleFiresImmediatelyThenWaits() {
        LoopTimer t = new LoopTimer();
        assertTrue(t.due(1000, 250));
        assertFalse(t.due(1100, 250));
        assertFalse(t.due(1249, 250));
        assertTrue(t.due(1250, 250));
        assertFalse(t.due(1251, 250));
    }

    @Test
    void throttleIsIndependentOfLoopTiming() {
        LoopTimer t = new LoopTimer();
        t.tick(0);
        t.tick(500 * MS);
        assertTrue(t.due(0, 250), "tick() must not consume the throttle");
    }
}
