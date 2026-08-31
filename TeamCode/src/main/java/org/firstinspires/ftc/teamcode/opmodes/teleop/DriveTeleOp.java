package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Assists;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/** Drive-only teleop. Reference for OpMode structure: wiring, no logic. */
@TeleOp(name = "Drive TeleOp", group = "drive")
public class DriveTeleOp extends ForwardOpMode {

    private static final double PRECISION_SCALE = 0.35;

    private Drive drive;

    @Override
    protected void configure() {
        drive = new Drive(hardwareMap);

        drive.setDefaultCommand(
                drive.teleop(
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()));

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
