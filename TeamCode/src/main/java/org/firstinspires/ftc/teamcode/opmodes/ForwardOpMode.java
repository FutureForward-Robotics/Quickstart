package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for every OpMode. Owns the three things that have no other correct home.
 *
 * <ol>
 *   <li><b>Lynx bulk caching.</b> Set to MANUAL once, cleared at the top of every loop before
 *       anything reads a sensor. Last season this lived inside {@code Drivetrain.periodic()}, which
 *       tied cache correctness to that one subsystem existing and being registered first.
 *   <li><b>{@code GamepadEx.readButtons()}.</b> Without a per-loop call, {@code wasJustPressed},
 *       {@code ButtonReader} and {@code TriggerReader} silently never update.
 *   <li><b>Loop timing.</b> See {@link LoopTimer}.
 * </ol>
 *
 * <p>{@link #initialize()} is final because the ordering matters: reset the scheduler <em>before</em>
 * any {@code SubsystemBase} is constructed (its constructor auto-registers), then configure caching,
 * then hand over to {@link #configure()}. Put your subsystems, default commands and bindings there.
 *
 * <p><b>Bulk caching caveat.</b> In MANUAL mode every sensor read inside one loop returns the same
 * cached value. That is the speedup. It also means a busy-wait for a sensor to change <em>within</em>
 * a single loop will never terminate -- put the wait in a command's {@code isFinished()} instead.
 */
public abstract class ForwardOpMode extends CommandOpMode {

    protected GamepadEx driver;
    protected GamepadEx operator;

    private final List<LynxModule> hubs = new ArrayList<>();
    private final LoopTimer loopTimer = new LoopTimer();

    @Override
    public final void initialize() {
        // Before anything constructs a SubsystemBase, which would register with the old instance.
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
    }

    /** Build subsystems, set default commands, bind buttons. Runs once, at init. */
    protected abstract void configure();

    @Override
    public void initialize_loop() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.initialize_loop();
        // Deliberately no actAll(): nothing on the robot moves before Play.
    }

    /**
     * One loop: clear the cache, read inputs, let every subsystem sense, run the scheduler, then
     * let every subsystem act. Commands therefore read a single consistent snapshot of the robot,
     * and a target a command sets this loop is acted on this loop.
     */
    @Override
    public void run() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.run();
        ForwardSubsystem.actAll();
    }

    /**
     * Cache clear and input read, before the scheduler touches anything. Runs in init as well as
     * during the match so init-time button reads (alliance pickers, auto selectors) work.
     */
    private void refreshInputs() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
        driver.readButtons();
        operator.readButtons();
        loopTimer.tick(System.nanoTime());
    }

    /** Seconds since the previous loop. Zero on the first. */
    protected double dtSeconds() {
        return loopTimer.dtSeconds();
    }

    /** Loop rate, or zero if it cannot be measured yet. Worth putting on the driver station. */
    protected double loopHz() {
        return loopTimer.hz();
    }

    /**
     * Rate limiter for telemetry. Telemetry transmission is one of the more expensive things in the
     * loop, so gate it:
     *
     * <pre>{@code
     * if (telemetryDue(250)) {
     *     telemetry.addData("loop", "%.0f Hz", loopHz());
     *     telemetry.update();
     * }
     * }</pre>
     */
    protected boolean telemetryDue(long intervalMs) {
        return loopTimer.due(System.currentTimeMillis(), intervalMs);
    }
}
