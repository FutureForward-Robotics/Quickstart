package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

/**
 * Continuous correction applied to the driver's request before it reaches the follower. Requires
 * no subsystems.
 */
@FunctionalInterface
public interface DriveAssist {

    DriveInput apply(DriveInput driver, Pose pose);
}
