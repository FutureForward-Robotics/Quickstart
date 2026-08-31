package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/** Asserts the sense-before-act ordering. */
class ForwardSubsystemTest {

    /** Records the phase calls it receives, in order. */
    private static final class Recorder extends ForwardSubsystem {
        private final String id;
        private final List<String> log;

        Recorder(String id, List<String> log) {
            this.id = id;
            this.log = log;
        }

        @Override
        public void sense() {
            log.add(id + ".sense");
        }

        @Override
        public void act() {
            log.add(id + ".act");
        }
    }

    private List<String> log;

    @BeforeEach
    void setUp() {
        // Both registries are static and outlive a single test.
        CommandScheduler.getInstance().reset();
        ForwardSubsystem.resetRegistry();
        log = new ArrayList<>();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().reset();
        ForwardSubsystem.resetRegistry();
    }

    @Test
    void everySubsystemSensesBeforeAnySubsystemActs() {
        new Recorder("a", log);
        new Recorder("b", log);
        new Recorder("c", log);

        ForwardSubsystem.senseAll();
        ForwardSubsystem.actAll();

        assertEquals(
                List.of("a.sense", "b.sense", "c.sense", "a.act", "b.act", "c.act"),
                log,
                "no subsystem may act before every subsystem has sensed");
    }

    @Test
    void constructionRegistersAutomatically() {
        assertEquals(0, ForwardSubsystem.registeredCount());
        new Recorder("a", log);
        new Recorder("b", log);
        assertEquals(2, ForwardSubsystem.registeredCount());
    }

    /** Subsystems declare only parameterised constructors; the implicit super() must still register. */
    private static final class TakesArguments extends ForwardSubsystem {
        private final String hardwareName;
        private int defaultExecutes;

        TakesArguments(String hardwareName) {
            this.hardwareName = hardwareName;
        }

        @Override
        public void sense() {}

        @Override
        public void act() {}
    }

    @Test
    void aSubclassWithNoExplicitSuperCallStillRegisters() {
        TakesArguments subsystem = new TakesArguments("lift");

        assertEquals(1, ForwardSubsystem.registeredCount(), "phase registry");
        assertEquals("lift", subsystem.hardwareName, "own fields assigned after super()");

        // Registration with the scheduler is observable through default-command dispatch, which
        // only happens for registered subsystems.
        subsystem.setDefaultCommand(new RunCommand(() -> subsystem.defaultExecutes++, subsystem));
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();

        assertTrue(subsystem.defaultExecutes > 0, "scheduler registry");
    }

    @Test
    void resetRegistryClearsSubsystemsFromAPreviousOpMode() {
        new Recorder("stale", log);
        assertEquals(1, ForwardSubsystem.registeredCount());

        ForwardSubsystem.resetRegistry();

        assertEquals(0, ForwardSubsystem.registeredCount());
        ForwardSubsystem.senseAll();
        ForwardSubsystem.actAll();
        assertTrue(log.isEmpty(), "a subsystem from a finished OpMode must not tick");
    }

    @Test
    void periodicIsInertSoItCannotBeUsedByMistake() {
        Recorder a = new Recorder("a", log);
        a.periodic();
        assertTrue(log.isEmpty());
    }

    @Test
    void phasesCanRunManyLoopsWithoutReRegistering() {
        new Recorder("a", log);

        for (int loop = 0; loop < 3; loop++) {
            ForwardSubsystem.senseAll();
            ForwardSubsystem.actAll();
        }

        assertEquals(6, log.size());
        assertEquals(1, ForwardSubsystem.registeredCount());
    }
}
