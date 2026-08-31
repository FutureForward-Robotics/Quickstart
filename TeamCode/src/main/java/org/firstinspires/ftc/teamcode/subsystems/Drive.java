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
 * The drivetrain, wrapping Pedro's {@link Follower}.
 *
 * <p><b>Three rules this class exists to enforce.</b>
 *
 * <ol>
 *   <li><b>Every motion command declares {@code this} as a requirement.</b> Neither last season's
 *       {@code commands/FollowPathCommand} nor SolversLib's {@code pedroCommand} package does. Without
 *       a requirement the scheduler cannot preempt the teleop default command, so the driver keeps
 *       feeding {@code setTeleOpDrive} while a path is running, and a stick nudge cannot cancel the
 *       path because nothing owns the drivetrain.
 *   <li><b>Every motion command stops the follower when interrupted.</b> Last season's
 *       {@code FollowPathCommand} has no {@code end(boolean)} at all, so a cancelled, raced or
 *       timed-out path leaves the robot driving. That is a runaway, not a bug.
 *   <li><b>{@code periodic()} is the only caller of {@code follower.update()}, and the teleop
 *       command's {@code initialize()} is the only caller of {@code startTeleopDrive()}.</b> One
 *       owner for the update, one owner for the mode switch.
 * </ol>
 *
 * <p>Mode handoff falls out of the scheduler for free: {@link #teleop} is the default command, any
 * path command preempts it, and when the path ends the scheduler reschedules {@code teleop}, whose
 * {@code initialize()} puts the follower back in teleop mode.
 */
public class Drive extends SubsystemBase {

    /** Watchdog, not a schedule. Generous on purpose; a short timeout truncates autos silently. */
    public static final long DEFAULT_PATH_TIMEOUT_MS = 8000;

    public static final long DEFAULT_TURN_TIMEOUT_MS = 3000;

    private final Follower follower;
    private final List<DriveAssist> assists = new ArrayList<>();

    public Drive(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    /** Resume from wherever the last OpMode left the robot. Use in teleop, after auto. */
    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, PoseStore.loadOr(new Pose()));
    }

    @Override
    public void periodic() {
        follower.update();
        PoseStore.save(follower.getPose());
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

    /**
     * Start a {@link Route} on this drivetrain. Goes through the subsystem so the {@code Follower}
     * itself never escapes.
     */
    public Route route(Alliance alliance, Waypoint start) {
        return Route.from(follower, alliance, start);
    }

    /** Escape hatch for path geometry the {@link Route} vocabulary cannot express yet. */
    public PathBuilder pathBuilder() {
        return new PathBuilder(follower);
    }

    // ---------------------------------------------------------------- commands

    /**
     * Default command. Stick input goes through the assist stack on the way to the follower.
     *
     * <p>Suppliers must already be in {@link DriveInput}'s sign convention; nothing here negates.
     * Field-centric.
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

    /**
     * Hold a pose against contact. Never finishes on its own -- race it, time it out, or let the
     * driver take the drivetrain back.
     */
    public Command holdAt(Waypoint waypoint, Alliance alliance) {
        return new FunctionalCommand(
                () -> follower.holdPoint(waypoint.pose(alliance)),
                () -> {},
                interrupted -> follower.breakFollowing(),
                () -> false,
                this);
    }

    /** Stop following immediately. */
    public Command halt() {
        return new FunctionalCommand(
                follower::breakFollowing, () -> {}, interrupted -> {}, () -> true, this);
    }

    /**
     * Enable an assist for as long as this command runs. Requires no subsystems, so it composes with
     * the driver and with any macro. Bind with {@code toggleWhenPressed} for a latch or {@code
     * whenHeld} for hold-to-use.
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
