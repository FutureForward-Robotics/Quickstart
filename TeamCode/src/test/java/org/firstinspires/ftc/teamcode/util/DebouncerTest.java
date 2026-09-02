package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DebouncerTest {

    private long nanos;

    @BeforeEach
    void setUp() {
        // Boot-relative, as on Android, so a first-sample bug shows up.
        nanos = 987_654_321_000L;
    }

    private void advance(double seconds) {
        nanos += (long) (seconds * 1e9);
    }

    private Debouncer debouncer(double seconds, Debouncer.Type type) {
        return new Debouncer(seconds, type, () -> nanos);
    }

    @Test
    void risingHoldsFalseUntilTheInputIsSteady() {
        Debouncer d = debouncer(0.2, Debouncer.Type.RISING);

        assertFalse(d.calculate(true), "must not pass through on the first sample");
        advance(0.1);
        assertFalse(d.calculate(true));
        advance(0.15);
        assertTrue(d.calculate(true), "0.25s elapsed");
    }

    @Test
    void theWindowStartsAtTheFirstSampleNotAtConstruction() {
        Debouncer d = debouncer(0.2, Debouncer.Type.RISING);

        // A subsystem is built in init; init runs for as long as the team waits for Play.
        advance(30);

        assertFalse(d.calculate(true), "the first sample must still be held for the period");
        advance(0.25);
        assertTrue(d.calculate(true));
    }

    @Test
    void risingRestartsWhenTheInputDrops() {
        Debouncer d = debouncer(0.2, Debouncer.Type.RISING);

        d.calculate(true);
        advance(0.15);
        d.calculate(false);
        advance(0.15);
        assertFalse(d.calculate(true), "the timer restarted at the drop");
        advance(0.25);
        assertTrue(d.calculate(true));
    }

    @Test
    void risingPassesFallingEdgesStraightThrough() {
        Debouncer d = debouncer(0.2, Debouncer.Type.RISING);
        d.calculate(true);
        advance(0.3);
        assertTrue(d.calculate(true));

        assertFalse(d.calculate(false), "only the rising edge is delayed");
    }

    @Test
    void fallingHoldsTrueAfterTheInputDrops() {
        Debouncer d = debouncer(0.2, Debouncer.Type.FALLING);

        assertTrue(d.calculate(false), "falling starts true");
        assertTrue(d.calculate(true));
        assertTrue(d.calculate(false), "held after the drop");
        advance(0.1);
        assertTrue(d.calculate(false));
        advance(0.15);
        assertFalse(d.calculate(false));
    }

    @Test
    void bothDelaysEitherDirection() {
        Debouncer d = debouncer(0.2, Debouncer.Type.BOTH);

        assertFalse(d.calculate(true));
        advance(0.25);
        assertTrue(d.calculate(true));

        assertTrue(d.calculate(false));
        advance(0.25);
        assertFalse(d.calculate(false));
    }

    @Test
    void aZeroPeriodPassesEverythingThrough() {
        Debouncer d = debouncer(0, Debouncer.Type.BOTH);
        assertTrue(d.calculate(true));
        assertFalse(d.calculate(false));
    }

    @Test
    void negativePeriodIsRejected() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new Debouncer(-1, Debouncer.Type.RISING, () -> nanos));
    }
}
