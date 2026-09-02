package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.firstinspires.ftc.teamcode.util.CommandLogHooks;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Base OpMode. Owns Lynx bulk caching, gamepad polling and loop timing.
 *
 * <p>{@link #initialize()} is final because the scheduler and subsystem registry must be reset
 * before any subsystem is constructed. Put subsystems, default commands and bindings in {@link
 * #configure()}.
 *
 * <p>Bulk caching is MANUAL and cleared once per loop, so every sensor read within a loop returns
 * the same value. Do not busy-wait on a sensor inside a single loop.
 *
 * <p>The scheduler runs in {@link #run()} only, so no command or trigger binding fires before Play.
 * Gamepads are still polled during init, so {@code driver.wasJustPressed(...)} works in an
 * overridden {@link #initialize_loop()} for auto selectors.
 */
public abstract class ForwardOpMode extends CommandOpMode {

    protected GamepadEx driver;
    protected GamepadEx operator;

    private final List<LynxModule> hubs = new ArrayList<>();
    private final LoopTimer loopTimer = new LoopTimer();

    private RunLog log;

    @Override
    public final void initialize() {
        CommandScheduler.getInstance().reset();
        ForwardSubsystem.resetRegistry();

        hubs.clear();
        hubs.addAll(hardwareMap.getAll(LynxModule.class));
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        configure();

        log = RunLog.openDefault(opModeName());
        try {
            ForwardSubsystem.registerSignals(log);
            logSignals(log);
            CommandLogHooks.install(log);
        } catch (RuntimeException e) {
            // end() is unreachable from here, so an unclosed log would leak its writer thread and
            // two open streams for the life of the app.
            log.close();
            throw e;
        }
    }

    /**
     * Register signals that belong to something outside the subsystem registry, such as a pure
     * per-loop computation. Called once, after every subsystem has registered.
     */
    protected void logSignals(RunLog log) {}

    /** Build subsystems, set default commands, bind buttons. */
    protected abstract void configure();

    @Override
    public void initialize_loop() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.initialize_loop();
        // No actAll(): nothing moves during init.
    }

    @Override
    public void run() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.run();
        ForwardSubsystem.actAll();
        log.writeLoop((int) loopTimer.count());
    }

    private void refreshInputs() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
        driver.readButtons();
        operator.readButtons();
        loopTimer.tick(System.nanoTime());
    }

    /**
     * Closes the run log. {@code CommandOpMode.runOpMode()} calls {@code end()} in a finally around
     * the init and run loops only: a throw out of {@link #initialize()} never reaches it.
     */
    @Override
    public void end() {
        if (log != null) {
            log.close();
        }
        super.end();
    }

    /** Run log for this OpMode. Non-null after init; may be a disabled sink. */
    protected RunLog log() {
        return log;
    }

    /** Registered OpMode name, falling back to the class name as the SDK does. */
    private String opModeName() {
        TeleOp teleOp = getClass().getAnnotation(TeleOp.class);
        if (teleOp != null && !teleOp.name().isEmpty()) {
            return teleOp.name();
        }
        Autonomous autonomous = getClass().getAnnotation(Autonomous.class);
        if (autonomous != null && !autonomous.name().isEmpty()) {
            return autonomous.name();
        }
        return getClass().getSimpleName();
    }

    /**
     * Identity of the current loop. Hand this to anything that computes once per loop outside the
     * sense/act phases, so it can cache against the loop instead of being scheduled.
     */
    protected LongSupplier loopId() {
        return loopTimer::count;
    }

    protected double dtSeconds() {
        return loopTimer.dtSeconds();
    }

    protected double loopHz() {
        return loopTimer.hz();
    }

    /** Rate limiter for telemetry. */
    protected boolean telemetryDue(long intervalMs) {
        return loopTimer.due(System.currentTimeMillis(), intervalMs);
    }
}
