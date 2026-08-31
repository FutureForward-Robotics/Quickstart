package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/** Carries the robot pose across an OpMode boundary. Stores primitives only. */
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

    public static Pose loadOr(Pose fallback) {
        return present ? new Pose(x, y, heading) : fallback;
    }

    /** Call from auto init so a pose from a previous match cannot leak in. */
    public static void clear() {
        present = false;
    }
}
