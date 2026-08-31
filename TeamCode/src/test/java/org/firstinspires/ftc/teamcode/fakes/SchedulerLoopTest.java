package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.junit.jupiter.api.Test;

/** The scheduler under the two-phase loop: default commands, preemption, and handback. */
@RobotTest
class SchedulerLoopTest {

    private static final class Counter extends ForwardSubsystem {
        int senses;
        int acts;
        int defaultExecutes;
        int macroExecutes;

        @Override
        public void sense() {
            senses++;
        }

        @Override
        public void act() {
            acts++;
        }

        Command defaultCommand() {
            return new RunCommand(() -> defaultExecutes++, this);
        }

        Command macro(int loopsToRun) {
            int[] remaining = {loopsToRun};
            return new FunctionalCommand(
                    () -> {},
                    () -> {
                        macroExecutes++;
                        remaining[0]--;
                    },
                    interrupted -> {},
                    () -> remaining[0] <= 0,
                    this);
        }
    }

    @Test
    void defaultCommandsRunEveryLoop(LoopRunner runner) {
        Counter subsystem = new Counter();
        subsystem.setDefaultCommand(subsystem.defaultCommand());

        runner.loops(4);

        assertEquals(4, subsystem.senses);
        assertEquals(4, subsystem.acts);
        assertTrue(subsystem.defaultExecutes >= 3, "got " + subsystem.defaultExecutes);
    }

    @Test
    void aScheduledCommandPreemptsTheDefaultAndHandsBack(LoopRunner runner) {
        Counter subsystem = new Counter();
        subsystem.setDefaultCommand(subsystem.defaultCommand());
        runner.loops(3);
        int defaultsBefore = subsystem.defaultExecutes;

        subsystem.macro(2).schedule();
        runner.loops(2);

        assertEquals(2, subsystem.macroExecutes);
        assertEquals(defaultsBefore, subsystem.defaultExecutes, "default is preempted");

        runner.loops(3);
        assertTrue(subsystem.defaultExecutes > defaultsBefore, "default resumes after the macro");
    }

    @Test
    void requiringCommandIsVisibleToTheScheduler(LoopRunner runner) {
        Counter subsystem = new Counter();
        Command macro = subsystem.macro(5);
        macro.schedule();
        runner.loop();

        assertEquals(macro, CommandScheduler.getInstance().requiring(subsystem));
    }
}
