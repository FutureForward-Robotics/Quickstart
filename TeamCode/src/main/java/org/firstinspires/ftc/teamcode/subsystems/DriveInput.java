package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Immutable drive request.
 *
 * <p>Sign convention, fixed here and nowhere else: {@code forward} positive drives away from the
 * driver station, {@code strafe} positive drives left, {@code turn} positive rotates
 * counter-clockwise. OpModes map raw gamepad axes into this convention; nothing downstream negates.
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

    /** Scales translation only; turn is unchanged. */
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
