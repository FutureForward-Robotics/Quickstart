package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Waypoint;

import java.util.function.DoubleSupplier;

/** Driver assists. Each yields to the driver on the axis it controls. */
public final class Assists {

    private Assists() {}

    private static final double DRIVER_TURN_DEADBAND = 0.05;

    /** Holds a heading unless the driver is turning. */
    public static DriveAssist headingLock(DoubleSupplier targetHeadingRad, double kP) {
        return (driver, pose) -> {
            if (Math.abs(driver.turn) > DRIVER_TURN_DEADBAND) {
                return driver;
            }
            double error = Field.normalize(targetHeadingRad.getAsDouble() - pose.getHeading());
            return driver.withTurn(clamp(kP * error, -1, 1));
        };
    }

    public static DriveAssist headingLock(double targetHeadingDeg, double kP) {
        final double rad = Math.toRadians(targetHeadingDeg);
        return headingLock(() -> rad, kP);
    }

    /** Scales translation down inside {@code radiusIn}, reaching {@code minScale} at the target. */
    public static DriveAssist slowNear(
            Waypoint target, Alliance alliance, double radiusIn, double minScale) {
        final Pose goal = target.pose(alliance);
        return (driver, pose) -> {
            double distance = Math.hypot(pose.getX() - goal.getX(), pose.getY() - goal.getY());
            if (distance >= radiusIn) {
                return driver;
            }
            return driver.scaledTranslation(minScale + (1 - minScale) * (distance / radiusIn));
        };
    }

    /** Adds translation toward a pose, capped at {@code maxAuthority}. */
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

    /** Flat translation cap, e.g. precision mode. */
    public static DriveAssist speedCap(double factor) {
        return (driver, pose) -> driver.scaledTranslation(factor);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
