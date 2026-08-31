package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

/**
 * A subsystem with a two-phase loop: every subsystem {@link #sense()}s before any subsystem
 * {@link #act()}s.
 *
 * <p><b>The name.</b> FutureForward, and it is also the invariant: the loop only ever moves
 * forward in time. No subsystem reads backward at a value left over from the previous loop.
 *
 * <p><b>Why.</b> {@code SubsystemBase} gives you one hook, {@code periodic()}, and the scheduler
 * calls it per subsystem in registration order. So subsystem B's control loop can run against
 * subsystem A's sensor values from the previous loop, and which subsystems are stale depends on
 * the order they happened to be constructed in. Splitting the phases removes the ordering
 * dependency entirely: within one loop, every reader sees the same snapshot of the robot.
 *
 * <p><b>The contract.</b>
 *
 * <ul>
 *   <li>{@link #sense()} reads hardware into fields. No motor writes, no decisions, no commanding
 *       other subsystems. It runs with the Lynx bulk cache freshly cleared.
 *   <li>{@link #act()} runs the control loop and writes outputs. By the time it runs, commands have
 *       already executed and set their targets, so a target set this loop is acted on this loop.
 *   <li>Everything in between -- command {@code execute()}, {@code isFinished()}, triggers, assists
 *       -- reads only fields populated by {@code sense()}.
 * </ul>
 *
 * <p>Loop order, driven by {@code ForwardOpMode}:
 *
 * <pre>
 *   clear bulk cache -> senseAll() -> scheduler (buttons, commands) -> actAll()
 * </pre>
 *
 * <p>{@code periodic()} is final and empty on purpose. If you find yourself wanting it, you want
 * {@code sense()} or {@code act()}.
 *
 * <p>Plain {@code SubsystemBase} subsystems still work and still get {@code periodic()}; they just
 * do not get the ordering guarantee. Prefer this class.
 */
public abstract class ForwardSubsystem extends SubsystemBase {

    private static final List<ForwardSubsystem> REGISTERED = new ArrayList<>();

    protected ForwardSubsystem() {
        if (!REGISTERED.contains(this)) {
            REGISTERED.add(this);
        }
    }

    /** Phase 1: read hardware into fields. */
    public abstract void sense();

    /** Phase 2: run the control loop and write outputs. */
    public abstract void act();

    @Override
    public final void periodic() {
        // Intentionally empty. See the class documentation.
    }

    // ---------------------------------------------------------------- registry

    /**
     * Clear the registry. Called by {@code ForwardOpMode.initialize()} before {@code configure()},
     * because these statics outlive an OpMode on the Control Hub.
     */
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

    public static int registeredCount() {
        return REGISTERED.size();
    }
}
