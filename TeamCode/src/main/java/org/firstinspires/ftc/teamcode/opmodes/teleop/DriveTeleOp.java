package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Assists;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Minimum viable teleop, and the reference for how an OpMode should look.
 *
 * <p>Wiring only. Note what is absent: no {@code follower.update()}, no bulk cache handling, no
 * {@code readButtons()}, no sign fiddling beyond mapping the sticks into {@link
 * org.firstinspires.ftc.teamcode.subsystems.DriveInput}'s convention once. If this file grows past
 * about forty lines, logic has leaked upward.
 */
@TeleOp(name = "Drive TeleOp", group = "drive")
public class DriveTeleOp extends ForwardOpMode {

    private static final double PRECISION_SCALE = 0.35;

    private Drive drive;

    @Override
    protected void configure() {
        drive = new Drive(hardwareMap); // resumes the pose auto left behind

        // forward = away from the driver, strafe = left, turn = CCW. Mapped once, here.
        drive.setDefaultCommand(
                drive.teleop(
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()));

        // Hold left bumper for precision mode. The assist requires no subsystems, so it layers
        // on top of whatever else owns the drivetrain instead of fighting it.
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(drive.assist(Assists.speedCap(PRECISION_SCALE)));
    }

    @Override
    public void run() {
        super.run();
        if (telemetryDue(250)) {
            telemetry.addData("loop", "%.0f Hz", loopHz());
            telemetry.addData("pose", drive.pose());
            telemetry.addData("assists", drive.activeAssists().size());
            telemetry.update();
        }
    }
}
