package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem with a two-phase loop: every subsystem senses before any subsystem acts.
 *
 * <p>Loop order, driven by {@code ForwardOpMode}:
 *
 * <pre>
 *   clear bulk cache -&gt; senseAll() -&gt; scheduler (buttons, commands) -&gt; actAll()
 * </pre>
 *
 * <p>{@link #sense()} reads hardware into fields and must not write outputs. {@link #act()} runs
 * the control loop and writes outputs, after commands have set their targets.
 *
 * <p>{@code periodic()} is final and empty. Registration is automatic; the registry is static and
 * is cleared by {@code ForwardOpMode.initialize()}.
 */
public abstract class ForwardSubsystem extends SubsystemBase {

    private static final List<ForwardSubsystem> REGISTERED = new ArrayList<>();

    protected ForwardSubsystem() {
        if (!REGISTERED.contains(this)) {
            REGISTERED.add(this);
        }
    }

    public abstract void sense();

    public abstract void act();

    /** Registers signals for the run log. Called once, after {@code configure()}. */
    public void logSignals(RunLog log) {}

    @Override
    public final void periodic() {}

    public static void resetRegistry() {
        REGISTERED.clear();
    }

    public static void senseAll() {
        for (int i = 0; i < REGISTERED.size(); i++) {
            REGISTERED.get(i).sense();
        }
    }

    public static void actAll() {
        for (int i = 0; i < REGISTERED.size(); i++) {
            REGISTERED.get(i).act();
        }
    }

    public static void registerSignals(RunLog log) {
        for (int i = 0; i < REGISTERED.size(); i++) {
            REGISTERED.get(i).logSignals(log);
        }
    }

    public static int registeredCount() {
        return REGISTERED.size();
    }
}
