package org.firstinspires.ftc.teamcode.subsystems;

/**
 * An immutable drive request, in one fixed sign convention.
 *
 * <p><b>The convention lives here and nowhere else.</b> Last season the signs were smeared across
 * three layers -- a {@code dir} multiplier in each OpMode, a negation in {@code ArcadeDrive}, and an
 * argument swap in {@code Drivetrain.updateTeleOpDrive} -- so nobody could say which layer owned it.
 *
 * <ul>
 *   <li>{@code forward} positive drives away from the driver station
 *   <li>{@code strafe} positive drives left
 *   <li>{@code turn} positive rotates counter-clockwise
 * </ul>
 *
 * <p>OpModes are responsible for mapping raw gamepad axes into this convention, e.g. {@code () ->
 * -gamepad1.left_stick_y}. Nothing downstream negates anything.
 */
public final class DriveInput {

    public final double forward;
    public final double strafe;
    public final double turn;

    public DriveInput(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public DriveInput withTurn(double newTurn) {
        return new DriveInput(forward, strafe, newTurn);
    }

    /** Scale translation only. Turn authority is left alone. */
    public DriveInput scaledTranslation(double factor) {
        return new DriveInput(forward * factor, strafe * factor, turn);
    }

    public DriveInput plus(double dForward, double dStrafe, double dTurn) {
        return new DriveInput(forward + dForward, strafe + dStrafe, turn + dTurn);
    }

    public boolean isMoving(double deadband) {
        return Math.abs(forward) > deadband
                || Math.abs(strafe) > deadband
                || Math.abs(turn) > deadband;
    }

    @Override
    public String toString() {
        return String.format("[fwd=%+.2f str=%+.2f turn=%+.2f]", forward, strafe, turn);
    }
}
