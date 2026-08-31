package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Assists;

/** The only place the driver button map lives. */
public final class DriverBindings {

    private DriverBindings() {}

    private static final double PRECISION_SCALE = 0.35;

    public static void configure(Robot robot, GamepadEx driver, GamepadEx operator) {
        // Sticks mapped into DriveInput's convention here and nowhere else.
        robot.drive.setDefaultCommand(
                robot.drive.teleop(
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(robot.drive.assist(Assists.speedCap(PRECISION_SCALE)));
    }
}
