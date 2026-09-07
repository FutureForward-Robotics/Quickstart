package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The write deadband, which decides whether an output change earns a bus transaction. */
class BusWritesTest {

    @Test
    void theFirstWriteAlwaysHappens() {
        assertTrue(BusWrites.worthServo(0.5, Double.NaN));
        assertTrue(BusWrites.worthMotor(0.0, Double.NaN), "zero is still a first write");
    }

    @Test
    void anIdenticalValueIsNotWritten() {
        assertFalse(BusWrites.worthServo(0.5, 0.5));
        assertFalse(BusWrites.worthMotor(-0.25, -0.25));
    }

    @Test
    void aChangeBelowTheServoEpsilonIsNotWritten() {
        assertFalse(BusWrites.worthServo(0.5 + BusWrites.SERVO_EPSILON / 2, 0.5));
        assertTrue(BusWrites.worthServo(0.5 + BusWrites.SERVO_EPSILON, 0.5));
    }

    @Test
    void aChangeBelowTheMotorEpsilonIsNotWritten() {
        assertFalse(BusWrites.worthMotor(0.5 + BusWrites.MOTOR_EPSILON / 2, 0.5));
        assertTrue(BusWrites.worthMotor(0.5 + BusWrites.MOTOR_EPSILON, 0.5));
    }

    @Test
    void theDeadbandIsSymmetric() {
        assertFalse(BusWrites.worthMotor(0.5 - BusWrites.MOTOR_EPSILON / 2, 0.5));
        assertTrue(BusWrites.worthMotor(0.5 - BusWrites.MOTOR_EPSILON, 0.5));
    }

    @Test
    void aNaNValueIsWrittenRatherThanSwallowed() {
        // A broken control loop should reach the hardware and fail visibly.
        assertTrue(BusWrites.worthMotor(Double.NaN, 0.5));
    }
}
