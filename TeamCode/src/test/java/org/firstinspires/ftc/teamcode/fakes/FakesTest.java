package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Exercises the fakes, including a closed loop that a Mockito mock cannot express. */
class FakesTest {

    /**
     * Minimal position-controlled mechanism. Takes the device, not a HardwareMap: {@code
     * HardwareMap.get} calls native code and throws off-robot, so subsystems need a
     * device-accepting constructor to be testable.
     */
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

    private LoopRunner runner;
    private FakeMotor motor;

    @BeforeEach
    void setUp() {
        LoopRunner.reset();
        motor = new FakeMotor("lift", 2000);
        runner = new LoopRunner().stepping(motor);
    }

    @AfterEach
    void tearDown() {
        LoopRunner.reset();
    }

    @Test
    void encoderIntegratesAppliedPower() {
        motor.setPower(0.5);
        motor.step(1.0);
        assertEquals(1000, motor.getCurrentPosition(), 1, "0.5 * 2000 ticks/s for 1s");
        motor.step(0.5);
        assertEquals(1500, motor.getCurrentPosition(), 1);
    }

    @Test
    void reverseDirectionFlipsTravel() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(0.5);
        motor.step(1.0);
        assertEquals(-1000, motor.getCurrentPosition(), 1);
    }

    @Test
    void resetEncoderModeZeroesTheCount() {
        motor.setPower(1.0);
        motor.step(1.0);
        assertNotEquals(0, motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, motor.getCurrentPosition());
    }

    @Test
    void closedLoopConvergesOnTarget() {
        TestLift lift = new TestLift(motor);
        lift.to(1500).schedule();

        int loopsUsed = runner.until(lift::atTarget, 500);

        assertTrue(lift.atTarget());
        assertEquals(1500, lift.measured(), 20);
        assertTrue(loopsUsed > 1, "should take real time, not converge instantly");
        System.out.printf("converged in %d loops (%.2fs)%n", loopsUsed, runner.elapsedSeconds());
    }

    @Test
    void commandFinishesWhenTheMechanismArrives() {
        TestLift lift = new TestLift(motor);
        Command move = lift.to(1000);
        move.schedule();

        runner.until(() -> !move.isScheduled(), 500);

        assertEquals(1000, lift.measured(), 20);
    }

    @Test
    void senseHappensBeforeActWithinALoop() {
        TestLift lift = new TestLift(motor);
        motor.setEncoder(700);

        runner.loop();

        assertEquals(700, lift.measured(), 1, "act() must run against this loop's reading");
    }

    @Test
    void servoRecordsCommandedPosition() {
        FakeServo servo = new FakeServo("claw");
        servo.setPosition(0.7);
        assertEquals(0.7, servo.getPosition(), 1e-9);
        servo.setPosition(5.0);
        assertEquals(1.0, servo.getPosition(), 1e-9, "clamped as the SDK does");
    }

    @Test
    void touchSensorReportsWhatTheTestSets() {
        FakeTouchSensor sensor = new FakeTouchSensor("limit");
        assertEquals(false, sensor.isPressed());
        sensor.setPressed(true);
        assertTrue(sensor.isPressed());
        assertEquals(1.0, sensor.getValue(), 1e-9);
    }
}
