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
 * Base OpMode. Owns Lynx bulk caching, gamepad polling and loop timing.
 *
 * <p>{@link #initialize()} is final because the scheduler and subsystem registry must be reset
 * before any subsystem is constructed. Put subsystems, default commands and bindings in {@link
 * #configure()}.
 *
 * <p>Bulk caching is MANUAL and cleared once per loop, so every sensor read within a loop returns
 * the same value. Do not busy-wait on a sensor inside a single loop.
 */
public abstract class ForwardOpMode extends CommandOpMode {

    protected GamepadEx driver;
    protected GamepadEx operator;

    private final List<LynxModule> hubs = new ArrayList<>();
    private final LoopTimer loopTimer = new LoopTimer();

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
    }

    /** Build subsystems, set default commands, bind buttons. */
    protected abstract void configure();

    @Override
    public void initialize_loop() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.initialize_loop();
        // No actAll(): nothing moves before Play.
    }

    @Override
    public void run() {
        refreshInputs();
        ForwardSubsystem.senseAll();
        super.run();
        ForwardSubsystem.actAll();
    }

    private void refreshInputs() {
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
        driver.readButtons();
        operator.readButtons();
        loopTimer.tick(System.nanoTime());
    }

    protected double dtSeconds() {
        return loopTimer.dtSeconds();
    }

    protected double loopHz() {
        return loopTimer.hz();
    }

    /** Rate limiter for telemetry, which is expensive to transmit every loop. */
    protected boolean telemetryDue(long intervalMs) {
        return loopTimer.due(System.currentTimeMillis(), intervalMs);
    }
}
