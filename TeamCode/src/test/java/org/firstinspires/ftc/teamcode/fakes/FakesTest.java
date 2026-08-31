package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.junit.jupiter.api.Test;

/**
 * Template for a subsystem test suite, and coverage of the fakes themselves.
 *
 * <p>To test a new subsystem: annotate the class {@link RobotTest}, take a {@link LoopRunner}
 * parameter, create devices with {@code runner.motor(...)}, and pass them to a subsystem
 * constructor that accepts devices. {@code HardwareMap.get} calls native code and throws
 * off-robot, so a subsystem needs a device-accepting constructor alongside its HardwareMap one.
 */
@RobotTest
class FakesTest {

    /** Stand-in mechanism: proportional position control. */
    private static final class TestLift extends ForwardSubsystem {
        private static final double KP = 0.002;
        private static final double TOLERANCE = 20;

        private final DcMotorEx motor;
        private double measured;
        private double setpoint;

        TestLift(DcMotorEx motor) {
            this.motor = motor;
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
        TestLift lift = new TestLift(runner.motor("lift"));

        lift.to(1500).schedule();
        int loopsUsed = runner.until(lift::atTarget, 500);

        assertEquals(1500, lift.measured(), 20);
        assertTrue(loopsUsed > 1, "must take real time, not converge instantly");
    }

    @Test
    void commandFinishesWhenTheMechanismArrives(LoopRunner runner) {
        TestLift lift = new TestLift(runner.motor("lift"));

        Command move = lift.to(1000);
        move.schedule();
        runner.until(() -> !move.isScheduled(), 500);

        assertEquals(1000, lift.measured(), 20);
    }

    @Test
    void senseHappensBeforeActWithinALoop(LoopRunner runner) {
        FakeMotor motor = runner.motor("lift");
        TestLift lift = new TestLift(motor);
        motor.setEncoder(700);

        runner.loop();

        assertEquals(700, lift.measured(), 1, "act() must run against this loop's reading");
    }

    @Test
    void eachTestGetsAFreshRegistry(LoopRunner runner) {
        new TestLift(runner.motor("lift"));
        assertEquals(1, ForwardSubsystem.registeredCount(), "no leakage from earlier tests");
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
    void digitalChannelReadsLowWhenPressed(LoopRunner runner) {
        FakeDigitalChannel limit = runner.digitalChannel("liftLimit");

        assertTrue(limit.getState(), "pulled up when released");
        assertFalse(limit.isPressed());

        limit.setPressed(true);
        assertFalse(limit.getState(), "pressed pulls the line low");
        assertTrue(limit.isPressed());
    }
}
