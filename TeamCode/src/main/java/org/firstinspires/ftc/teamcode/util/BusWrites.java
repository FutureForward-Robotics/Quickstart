package org.firstinspires.ftc.teamcode.util;

/**
 * Whether an output change is worth a write to the hub.
 *
 * <p>One Lynx transaction costs roughly 2.5 ms, measured on a Control Hub by timing each
 * subsystem's act phase. An output recomputed every loop by a control loop is almost never bit
 * identical to the last one, so comparing with {@code !=} writes every loop and spends the loop
 * budget sending changes the hardware cannot resolve.
 */
public final class BusWrites {

    private BusWrites() {}

    /**
     * Smallest servo position change worth sending, in the servo's 0 to 1 range. Around four PWM
     * quantisation steps, a third of a degree on a 180 degree servo.
     */
    public static final double SERVO_EPSILON = 0.002;

    /** Smallest motor power change worth sending, as a fraction of full power. */
    public static final double MOTOR_EPSILON = 0.01;

    /**
     * True when {@code value} differs from the last written value by at least {@code epsilon}, and
     * whenever {@code lastWritten} is NaN, so the first write always happens.
     */
    public static boolean worth(double value, double lastWritten, double epsilon) {
        return !(Math.abs(value - lastWritten) < epsilon);
    }

    /** {@link #worth} at {@link #SERVO_EPSILON}. */
    public static boolean worthServo(double position, double lastWritten) {
        return worth(position, lastWritten, SERVO_EPSILON);
    }

    /** {@link #worth} at {@link #MOTOR_EPSILON}. */
    public static boolean worthMotor(double power, double lastWritten) {
        return worth(power, lastWritten, MOTOR_EPSILON);
    }
}
