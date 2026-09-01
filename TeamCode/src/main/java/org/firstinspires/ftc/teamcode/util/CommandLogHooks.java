package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

/** Feeds command lifecycle transitions into a {@link RunLog}. */
public final class CommandLogHooks {

    private CommandLogHooks() {}

    /**
     * Hooks start, interrupt and finish on the current scheduler. Hooks live on the scheduler
     * instance, so {@code CommandScheduler.reset()} drops them; install after every reset. Execute
     * is deliberately not hooked, because it fires once per scheduled command per tick.
     *
     * <p>Only {@code interrupt} distinguishes a cancelled command from a completed one.
     */
    public static void install(RunLog log) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> record(log, "start", command));
        scheduler.onCommandInterrupt(command -> record(log, "interrupt", command));
        scheduler.onCommandFinish(command -> record(log, "finish", command));
    }

    private static void record(RunLog log, String event, Command command) {
        log.commandEvent(event, nameOf(command), System.identityHashCode(command));
    }

    /** {@code getName()} is on {@link CommandBase}, not the {@link Command} interface. */
    private static String nameOf(Command command) {
        return command instanceof CommandBase
                ? ((CommandBase) command).getName()
                : command.getClass().getSimpleName();
    }
}
