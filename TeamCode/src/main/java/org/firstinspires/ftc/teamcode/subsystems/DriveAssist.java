package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * Continuous correction applied to the driver's request before it reaches the follower. Assists
 * require no subsystems, so they do not conflict with commands.
 */
@FunctionalInterface
public interface DriveAssist {

    DriveInput apply(DriveInput driver, Pose pose);
}
