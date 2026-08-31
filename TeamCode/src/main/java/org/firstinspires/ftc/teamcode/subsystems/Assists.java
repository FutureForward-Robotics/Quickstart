package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Waypoint;

import java.util.function.DoubleSupplier;

/**
 * The assist library. Each entry is a hypothesis about why a cycle is slow. Add one, measure the
 * cycle, keep it or delete it.
 *
 * <p>Every assist yields to the driver: if the driver is actively commanding the axis an assist
 * wants to control, the driver wins for that loop.
 */
public final class Assists {

    private Assists() {}

    private static final double DRIVER_TURN_DEADBAND = 0.05;

    /**
     * Hold a heading unless the driver is actively turning. Removes the wiggle-to-line-up tax.
     *
     * @param targetHeadingRad supplier so the target can move (e.g. always face the goal)
     */
    public static DriveAssist headingLock(DoubleSupplier targetHeadingRad, double kP) {
        return (driver, pose) -> {
            if (Math.abs(driver.turn) > DRIVER_TURN_DEADBAND) {
                return driver;
            }
            double error = Field.normalize(targetHeadingRad.getAsDouble() - pose.getHeading());
            return driver.withTurn(clamp(kP * error, -1, 1));
        };
    }

    /** Hold a fixed heading. */
    public static DriveAssist headingLock(double targetHeadingDeg, double kP) {
        final double rad = Math.toRadians(targetHeadingDeg);
        return headingLock(() -> rad, kP);
    }

    /**
     * Scale translation down as the robot closes on a target so the driver stops overshooting.
     * Turn authority is untouched.
     *
     * @param minScale scale applied at zero distance, e.g. 0.35
     */
    public static DriveAssist slowNear(Waypoint target, Alliance alliance, double radiusIn, double minScale) {
        final Pose goal = target.pose(alliance);
        return (driver, pose) -> {
            double distance = Math.hypot(pose.getX() - goal.getX(), pose.getY() - goal.getY());
            if (distance >= radiusIn) {
                return driver;
            }
            return driver.scaledTranslation(minScale + (1 - minScale) * (distance / radiusIn));
        };
    }

    /**
     * Nudge translation toward a pose while the driver keeps authority. This is an assist, not a
     * takeover: {@code maxAuthority} caps how much the robot adds on its own.
     */
    public static DriveAssist pullToward(
            Waypoint target, Alliance alliance, double kP, double radiusIn, double maxAuthority) {
        final Pose goal = target.pose(alliance);
        return (driver, pose) -> {
            double dx = goal.getX() - pose.getX();
            double dy = goal.getY() - pose.getY();
            if (Math.hypot(dx, dy) > radiusIn) {
                return driver;
            }
            return driver.plus(
                    clamp(kP * dx, -maxAuthority, maxAuthority),
                    clamp(kP * dy, -maxAuthority, maxAuthority),
                    0);
        };
    }

    /** Flat speed cap, e.g. a precision-mode toggle. */
    public static DriveAssist speedCap(double factor) {
        return (driver, pose) -> driver.scaledTranslation(factor);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
