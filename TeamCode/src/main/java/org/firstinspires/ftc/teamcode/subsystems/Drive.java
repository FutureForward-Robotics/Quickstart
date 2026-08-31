package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStore;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Drivetrain wrapping Pedro's {@link Follower}.
 *
 * <p>Every motion command declares this subsystem as a requirement, calls {@code breakFollowing()}
 * when interrupted, and carries a timeout.
 *
 * <p>{@link #sense()} is the only caller of {@code follower.update()}. The {@link #teleop} command's
 * initialize is the only caller of {@code startTeleopDrive()}. Mode handoff follows from the
 * scheduler: teleop is the default command, a path command preempts it, and teleop is rescheduled
 * when the path ends.
 */
public class Drive extends ForwardSubsystem {

    /** Watchdog. Deliberately generous; a short timeout truncates autos silently. */
    public static final long DEFAULT_PATH_TIMEOUT_MS = 8000;

    public static final long DEFAULT_TURN_TIMEOUT_MS = 3000;

    private final Follower follower;
    private final List<DriveAssist> assists = new ArrayList<>();

    public Drive(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    /** Resumes from the stored pose. Use in teleop, after auto. */
    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, PoseStore.loadOr(new Pose()));
    }

    /**
     * Pedro's {@code update()} refreshes localization and writes motor powers in one call and
     * cannot be split. {@code updatePose()} must not be called separately: {@code
     * PoseTracker.update()} shifts {@code previousPoseTime}, so a second call in the same loop
     * measures velocity over roughly zero elapsed time.
     *
     * <p>Running it here keeps the pose read during the command phase current. Drive vectors set by
     * a command this loop are applied by the next loop's update.
     */
    @Override
    public void sense() {
        follower.update();
        PoseStore.save(follower.getPose());
    }

    /** Empty: the follower wrote motor powers during {@link #sense()}. */
    @Override
    public void act() {
        // intentionally empty; see sense()
    }

    // ---------------------------------------------------------------- state

    public Pose pose() {
        return follower.getPose();
    }

    public double headingRad() {
        return follower.getPose().getHeading();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public List<DriveAssist> activeAssists() {
        return assists;
    }

    /** Starts a {@link Route} without exposing the {@code Follower}. */
    public Route route(Alliance alliance, Waypoint start) {
        return Route.from(follower, alliance, start);
    }

    /** For path geometry {@link Route} cannot express. */
    public PathBuilder pathBuilder() {
        return new PathBuilder(follower);
    }

    // ---------------------------------------------------------------- commands

    /**
     * Default command. Stick input passes through the assist stack. Suppliers must already be in
     * {@link DriveInput}'s sign convention. Field-centric.
     */
    public Command teleop(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return teleop(forward, strafe, turn, false);
    }

    public Command teleop(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, boolean robotCentric) {
        return new FunctionalCommand(
                follower::startTeleopDrive,
                () -> {
                    DriveInput input =
                            new DriveInput(
                                    forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
                    for (int i = 0; i < assists.size(); i++) {
                        input = assists.get(i).apply(input, follower.getPose());
                    }
                    follower.setTeleOpDrive(input.forward, input.strafe, input.turn, robotCentric);
                },
                interrupted -> {},
                () -> false,
                this);
    }

    public Command follow(PathChain path) {
        return follow(path, 1.0, DEFAULT_PATH_TIMEOUT_MS);
    }

    public Command follow(PathChain path, double maxPower) {
        return follow(path, maxPower, DEFAULT_PATH_TIMEOUT_MS);
    }

    public Command follow(PathChain path, double maxPower, long timeoutMs) {
        return new FunctionalCommand(
                        () -> follower.followPath(path, maxPower, true),
                        () -> {},
                        interrupted -> {
                            if (interrupted) {
                                follower.breakFollowing();
                            }
                        },
                        () -> !follower.isBusy(),
                        this)
                .withTimeout(timeoutMs);
    }

    public Command turnTo(double headingDeg) {
        return turnTo(headingDeg, DEFAULT_TURN_TIMEOUT_MS);
    }

    public Command turnTo(double headingDeg, long timeoutMs) {
        return new FunctionalCommand(
                        () -> follower.turnTo(Math.toRadians(headingDeg)),
                        () -> {},
                        interrupted -> {
                            if (interrupted) {
                                follower.breakFollowing();
                            }
                        },
                        () -> !follower.isBusy(),
                        this)
                .withTimeout(timeoutMs);
    }

    /** Holds a pose. Never finishes on its own; race it or let the driver take the drivetrain. */
    public Command holdAt(Waypoint waypoint, Alliance alliance) {
        return new FunctionalCommand(
                () -> follower.holdPoint(waypoint.pose(alliance)),
                () -> {},
                interrupted -> follower.breakFollowing(),
                () -> false,
                this);
    }

    /** Stops following. */
    public Command halt() {
        return new FunctionalCommand(
                follower::breakFollowing, () -> {}, interrupted -> {}, () -> true, this);
    }

    /**
     * Enables an assist while this command runs. Requires no subsystems. Bind with {@code
     * toggleWhenPressed} for a latch or {@code whenHeld} for hold-to-use.
     */
    public Command assist(DriveAssist assist) {
        return new StartEndCommand(
                () -> {
                    if (!assists.contains(assist)) {
                        assists.add(assist);
                    }
                },
                () -> assists.remove(assist));
    }
}
