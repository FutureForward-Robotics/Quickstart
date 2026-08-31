package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.fail;

import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Runs the robot loop off-robot: sense, scheduler, act, then advance fake hardware by one tick.
 * Mirrors {@code ForwardOpMode.run()} without gamepads or bulk caching.
 *
 * <p>Call {@link #reset()} in a {@code @BeforeEach} before constructing subsystems.
 */
public final class LoopRunner {

    public static final double DEFAULT_DT_SECONDS = 0.020;

    private final double dtSeconds;
    private final List<FakeMotor> motors = new ArrayList<>();
    private double elapsedSeconds;
    private int loops;

    public LoopRunner() {
        this(DEFAULT_DT_SECONDS);
    }

    public LoopRunner(double dtSeconds) {
        this.dtSeconds = dtSeconds;
    }

    /** Clears the scheduler and subsystem registry, both of which are static. */
    public static void reset() {
        CommandScheduler.getInstance().reset();
        ForwardSubsystem.resetRegistry();
    }

    /** Motors advanced each loop. */
    public LoopRunner stepping(FakeMotor... fakeMotors) {
        motors.addAll(Arrays.asList(fakeMotors));
        return this;
    }

    public void loop() {
        ForwardSubsystem.senseAll();
        CommandScheduler.getInstance().run();
        ForwardSubsystem.actAll();
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).step(dtSeconds);
        }
        elapsedSeconds += dtSeconds;
        loops++;
    }

    public void loops(int count) {
        for (int i = 0; i < count; i++) {
            loop();
        }
    }

    /** Runs until {@code done}, failing the test if {@code maxLoops} is reached first. */
    public int until(BooleanSupplier done, int maxLoops) {
        for (int i = 0; i < maxLoops; i++) {
            if (done.getAsBoolean()) {
                return i;
            }
            loop();
        }
        if (!done.getAsBoolean()) {
            fail("condition not met within " + maxLoops + " loops (" + elapsedSeconds + "s)");
        }
        return maxLoops;
    }

    public double elapsedSeconds() {
        return elapsedSeconds;
    }

    public int loopCount() {
        return loops;
    }
}
