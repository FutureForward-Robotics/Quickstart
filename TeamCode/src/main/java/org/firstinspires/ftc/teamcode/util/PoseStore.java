package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Carries the robot's pose across an OpMode boundary, so teleop starts field-oriented where auto
 * finished.
 *
 * <p>This holds three doubles and nothing else. That matters: last season the same job was done by
 * caching the whole {@code Drivetrain} in a static, which kept a {@code Follower} built from a
 * {@code HardwareMap} that does not survive OpMode teardown. Every call site then had to pass
 * {@code create=true} to defeat the cache, which is a bug-shaped API. Store data statically;
 * construct hardware fresh every OpMode.
 */
public final class PoseStore {

    private PoseStore() {}

    private static boolean present;
    private static double x;
    private static double y;
    private static double heading;

    public static void save(Pose pose) {
        present = true;
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
    }

    public static boolean isPresent() {
        return present;
    }

    /** The stored pose, or {@code fallback} if nothing has been stored since the app started. */
    public static Pose loadOr(Pose fallback) {
        return present ? new Pose(x, y, heading) : fallback;
    }

    /** Call from auto's init so a stale pose from a previous match cannot leak in. */
    public static void clear() {
        present = false;
    }
}
