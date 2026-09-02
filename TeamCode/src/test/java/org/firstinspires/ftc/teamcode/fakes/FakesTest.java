package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.junit.jupiter.api.Test;

/**
 * Template for a subsystem test suite, and coverage of the fakes themselves.
 *
 * <p>To test a new subsystem: annotate the class {@link RobotTest}, take a {@link LoopRunner}
 * parameter, create its devices with {@code runner.motor(...)}, and construct the subsystem from
 * {@code runner.hardwareMap()}.
 */
@RobotTest
class FakesTest {

    /** Stand-in mechanism: proportional position control, built from a HardwareMap. */
    private static final class TestLift extends ForwardSubsystem {
        private static final double KP = 0.002;
        private static final double TOLERANCE = 20;

        private final DcMotorEx motor;
        private double measured;
        private double setpoint;

        TestLift(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "lift");
        }

        @Override
        public void sense() {
            measured = motor.getCurrentPosition();
        }

        @Override
        public void act() {
            motor.setPower(Math.max(-1, Math.min(1, KP * (setpoint - measured))));
        }

        boolean atTarget() {
            return Math.abs(setpoint - measured) < TOLERANCE;
        }

        double measured() {
            return measured;
        }

        Command to(double target) {
            return new FunctionalCommand(
                    () -> setpoint = target, () -> {}, interrupted -> {}, this::atTarget, this);
        }
    }

    @Test
    void closedLoopConvergesOnTarget(LoopRunner runner) {
        runner.motor("lift");
        TestLift lift = new TestLift(runner.hardwareMap());

        lift.to(1500).schedule();
        int loopsUsed = runner.until(lift::atTarget, 500);

        assertEquals(1500, lift.measured(), 20);
        assertTrue(loopsUsed > 1, "must take real time, not converge instantly");
    }

    @Test
    void commandFinishesWhenTheMechanismArrives(LoopRunner runner) {
        runner.motor("lift");
        TestLift lift = new TestLift(runner.hardwareMap());

        Command move = lift.to(1000);
        move.schedule();
        runner.until(() -> !move.isScheduled(), 500);

        assertEquals(1000, lift.measured(), 20);
    }

    @Test
    void senseHappensBeforeActWithinALoop(LoopRunner runner) {
        FakeMotor motor = runner.motor("lift");
        TestLift lift = new TestLift(runner.hardwareMap());
        motor.setEncoder(700);

        runner.loop();

        assertEquals(700, lift.measured(), 1, "sense() read this loop's encoder");
        // The discriminating observable: act() against measured = 700 and setpoint = 0 saturates
        // negative, where act-then-sense would compute 0 against an unread encoder.
        assertEquals(-1.0, motor.getPower(), 1e-9, "act() must run against this loop's reading");
    }

    @Test
    void eachTestGetsAFreshRegistry(LoopRunner runner) {
        runner.motor("lift");
        new TestLift(runner.hardwareMap());
        assertEquals(1, ForwardSubsystem.registeredCount(), "no leakage from earlier tests");
    }

    @Test
    void mistypedHardwareNameFails(LoopRunner runner) {
        runner.motor("lift");

        IllegalArgumentException thrown =
                assertThrows(
                        IllegalArgumentException.class,
                        () -> runner.hardwareMap().get(DcMotorEx.class, "lfit"));

        assertTrue(thrown.getMessage().contains("lift"), "should list the registered names");
    }

    @Test
    void wrongDeviceTypeFails(LoopRunner runner) {
        runner.servo("claw");

        assertThrows(
                IllegalArgumentException.class,
                () -> runner.hardwareMap().get(DcMotorEx.class, "claw"));
        assertNotEquals(null, runner.hardwareMap().get(Servo.class, "claw"));
    }

    @Test
    void encoderIntegratesAppliedPower(LoopRunner runner) {
        FakeMotor motor = runner.motor("lift", 2000);

        motor.setPower(0.5);
        motor.step(1.0);
        assertEquals(1000, motor.getCurrentPosition(), 1);

        motor.step(0.5);
        assertEquals(1500, motor.getCurrentPosition(), 1);
    }

    @Test
    void reverseDirectionFlipsTravel(LoopRunner runner) {
        FakeMotor motor = runner.motor("lift", 2000);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(0.5);
        motor.step(1.0);

        assertEquals(-1000, motor.getCurrentPosition(), 1);
    }

    @Test
    void resetEncoderModeZeroesTheCount(LoopRunner runner) {
        FakeMotor motor = runner.motor("lift");

        motor.setPower(1.0);
        motor.step(1.0);
        assertNotEquals(0, motor.getCurrentPosition());

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, motor.getCurrentPosition());
    }

    @Test
    void servoClampsToTheSdkRange(LoopRunner runner) {
        FakeServo servo = runner.servo("claw");

        servo.setPosition(0.7);
        assertEquals(0.7, servo.getPosition(), 1e-9);

        servo.setPosition(5.0);
        assertEquals(1.0, servo.getPosition(), 1e-9);
    }

    @Test
    void servoReportsTheLogicalPositionUnderAScaledRange(LoopRunner runner) {
        FakeServo servo = runner.servo("claw");
        servo.scaleRange(0.2, 0.8);

        servo.setPosition(0.7);

        assertEquals(0.7, servo.getPosition(), 1e-9, "ServoImpl scales the reading back to [0,1]");
        assertEquals(0.62, servo.commanded(), 1e-9, "the controller receives the scaled value");
    }

    @Test
    void digitalChannelReadsLowWhenPressed(LoopRunner runner) {
        FakeDigitalChannel limit = runner.digitalChannel("liftLimit");

        assertTrue(limit.getState(), "pulled up when released");
        assertFalse(limit.isPressed());

        limit.setPressed(true);
        assertFalse(limit.getState(), "pressed pulls the line low");
        assertTrue(limit.isPressed());
    }

    @Test
    void continuousRotationServoTakesSignedPower(LoopRunner runner) {
        FakeCRServo roller = runner.register("roller", new FakeCRServo("roller"));

        roller.setPower(-1.0);
        assertEquals(-1.0, roller.getPower(), 1e-9);

        roller.setPower(3.0);
        assertEquals(1.0, roller.getPower(), 1e-9, "clamped like the SDK");
    }

    /**
     * Battery voltage hangs off a public field on HardwareMap rather than {@code get(...)}, so the
     * runner installs the mapping. Without it, any subsystem doing voltage compensation would NPE
     * off-robot and be pushed into taking its devices through a test-only constructor.
     */
    @Test
    void voltageIsReachableThroughTheHardwareMapField(LoopRunner runner) {
        FakeVoltageSensor battery = runner.voltageSensor(12.4);

        assertEquals(
                12.4,
                runner.hardwareMap().voltageSensor.iterator().next().getVoltage(),
                1e-9);
        assertEquals(1, battery.reads(), "reads are counted, so a sampling rate can be asserted");

        battery.setVolts(9.0);
        assertEquals(9.0, runner.hardwareMap().voltageSensor.iterator().next().getVoltage(), 1e-9);
    }
}
