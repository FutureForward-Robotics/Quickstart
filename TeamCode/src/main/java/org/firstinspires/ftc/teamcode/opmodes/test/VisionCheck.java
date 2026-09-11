package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.opmodes.teleop.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.PoseFusion;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.LimelightCamera;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Drives normally and compares the camera's field pose against odometry. This is the once-a-season
 * check {@link Field#toPedro} asks for: park on a known spot, read both, and confirm they agree.
 *
 * <p>A constant offset means the camera pose in the Limelight UI is wrong. Swapped or mirrored axes
 * mean the team's Pedro frame is rotated relative to the field frame, which is a change to {@code
 * Field.toPedro}. A delta that grows with speed means latency, so compare at rest.
 *
 * <p>{@link PoseFusion} is running, so the delta should settle toward zero on its own; Y seeds the
 * whole correction at once for when it starts too far out to converge.
 */
@TeleOp(name = "Vision Check", group = "test")
public class VisionCheck extends ForwardOpMode {

    /** Seeding while moving bakes the frame's age into the pose. Inches per second. */
    private static final double SEED_SPEED_LIMIT = 4.0;

    private Robot robot;
    private Vision vision;
    private PoseFusion fusion;

    private String lastSeed = "none yet";

    @Override
    protected void configure() {
        robot = new Robot(hardwareMap, Alliance.RED);
        // After the drivetrain, so it senses after it and orients the camera with this loop's pose.
        vision = new Vision(new LimelightCamera(hardwareMap, "limelight"), robot.drive);
        // After the camera, so a correction acts on this loop's frame.
        fusion = new PoseFusion(vision, robot.drive, robot.drive::setPose, System::nanoTime);

        DriverBindings.configure(robot, driver, operator);
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(this::seed));
    }

    @Override
    protected void logSignals(RunLog log) {
        vision.logSignals(log);
        fusion.logSignals(log);
    }

    private void seed() {
        Pose seen = vision.fieldPose();
        if (seen == null) {
            lastSeed = "refused: no trusted pose";
            return;
        }
        if (speed() > SEED_SPEED_LIMIT) {
            lastSeed = String.format("refused: moving at %.1f in/s", speed());
            return;
        }
        robot.drive.setPose(seen);
        lastSeed = String.format("seeded %s", seen);
    }

    private double speed() {
        return Math.hypot(robot.drive.velocityX(), robot.drive.velocityY());
    }

    @Override
    public void run() {
        super.run();
        if (!telemetryDue(250)) {
            return;
        }
        Pose odometry = robot.drive.pose();
        Pose seen = vision.fieldPose();

        telemetry.addData("loop", "%.0f Hz", loopHz());
        telemetry.addData("speed", "%.1f in/s", speed());
        telemetry.addData("odometry", odometry);
        telemetry.addData("camera", seen == null ? "no trusted pose" : seen.toString());
        if (seen != null) {
            telemetry.addData(
                    "delta now",
                    "x %+.1f  y %+.1f  heading %+.1f deg",
                    seen.x() - odometry.x(),
                    seen.y() - odometry.y(),
                    Math.toDegrees(Field.normalize(seen.heading() - odometry.heading())));
        }
        telemetry.addData("hasTarget", vision.hasTarget());
        telemetry.addData("tags", vision.sample().tagCount());
        telemetry.addData("avgTagDist", "%.1f in", vision.sample().avgTagDistanceIn());
        telemetry.addData("staleness", "%d ms", vision.sample().stalenessMs());
        telemetry.addData(
                "fusion", "%d applied, %d rejected", fusion.corrections(), fusion.rejections());
        telemetry.addData("error at capture", "%.1f in", fusion.lastErrorIn());
        telemetry.addData("Y to seed", lastSeed);
        telemetry.update();
    }
}
