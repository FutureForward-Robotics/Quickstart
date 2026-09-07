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

    private long senseNanos;
    private long actNanos;

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
            long start = System.nanoTime();
            REGISTERED.get(i).sense();
            REGISTERED.get(i).senseNanos = System.nanoTime() - start;
        }
    }

    public static void actAll() {
        for (int i = 0; i < REGISTERED.size(); i++) {
            long start = System.nanoTime();
            REGISTERED.get(i).act();
            REGISTERED.get(i).actNanos = System.nanoTime() - start;
        }
    }

    public static void registerSignals(RunLog log) {
        for (int i = 0; i < REGISTERED.size(); i++) {
            ForwardSubsystem subsystem = REGISTERED.get(i);
            String name = subsystem.signalPrefix();
            log.addSignal(name + ".senseMs", () -> subsystem.senseNanos / 1e6);
            log.addSignal(name + ".actMs", () -> subsystem.actNanos / 1e6);
            subsystem.logSignals(log);
        }
    }

    /**
     * Prefix for this subsystem's log columns, so timing lands in the same chart group as the
     * subsystem's own signals. Two instances of one class would collide, so the index is appended
     * from the second onwards.
     */
    private String signalPrefix() {
        String base = getClass().getSimpleName();
        base = Character.toLowerCase(base.charAt(0)) + base.substring(1);
        int ordinal = 0;
        for (int i = 0; i < REGISTERED.size(); i++) {
            ForwardSubsystem other = REGISTERED.get(i);
            if (other == this) {
                break;
            }
            if (other.getClass() == getClass()) {
                ordinal++;
            }
        }
        return ordinal == 0 ? base : base + ordinal;
    }

    public static int registeredCount() {
        return REGISTERED.size();
    }
}
