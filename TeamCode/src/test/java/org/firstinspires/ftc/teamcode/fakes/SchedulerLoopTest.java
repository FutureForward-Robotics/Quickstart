package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.firstinspires.ftc.teamcode.util.CommandLogHooks;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.List;

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

    @Test
    void commandStartAndInterruptReachTheEventLog(LoopRunner runner, @TempDir File logDir)
            throws IOException {
        Counter subsystem = new Counter();
        FakeMotor arm = runner.motor("arm");
        RunLog log = RunLog.open(logDir, "SchedulerLoopTest");
        CommandLogHooks.install(log);
        log.addSignal("arm.ticks", () -> arm.getCurrentPosition());

        Command macro = subsystem.macro(50);
        ((CommandBase) macro).setName("LogMacro");
        macro.schedule();
        for (int i = 0; i < 5; i++) {
            runner.loop();
            log.writeLoop(i);
            if (i == 2) {
                macro.cancel();
            }
        }
        log.close();

        List<String> events =
                Files.readAllLines(
                        new File(log.directory(), "events.jsonl").toPath(),
                        Charset.forName("UTF-8"));
        assertTrue(
                events.stream()
                        .anyMatch(
                                line ->
                                        line.contains("\"name\":\"LogMacro\"")
                                                && line.contains("\"event\":\"start\"")),
                events.toString());
        assertTrue(
                events.stream()
                        .anyMatch(
                                line ->
                                        line.contains("\"name\":\"LogMacro\"")
                                                && line.contains("\"event\":\"interrupt\"")),
                events.toString());
        assertEquals(
                6,
                Files.readAllLines(
                                new File(log.directory(), "signals.csv").toPath(),
                                Charset.forName("UTF-8"))
                        .size(),
                "header plus one row per loop");
    }
}
