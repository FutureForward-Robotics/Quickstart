package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * A continuous correction applied to the driver's request on the way to the follower.
 *
 * <p>An assist is a pure function, so it is trivially testable and trivially composable. Assists
 * require no subsystems, which means enabling one never conflicts with a macro or with the driver --
 * it only changes how stick input is interpreted.
 *
 * <p>Discrete behaviour ("go do this") belongs in a {@link Drive} command factory. Continuous
 * behaviour ("help me while I drive") belongs here.
 */
@FunctionalInterface
public interface DriveAssist {

    /**
     * @param driver what the driver asked for, in {@link DriveInput}'s sign convention
     * @param pose where the robot currently is
     * @return the corrected request
     */
    DriveInput apply(DriveInput driver, Pose pose);
}
