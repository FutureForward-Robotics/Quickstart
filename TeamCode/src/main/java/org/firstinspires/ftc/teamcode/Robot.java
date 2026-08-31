package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.PoseStore;

/**
 * Composition root. Builds every subsystem once and holds the alliance for the match. OpModes
 * construct one and add bindings.
 */
public final class Robot {

    public final Alliance alliance;
    public final Drive drive;

    /** Auto. Starts from a known waypoint and discards any pose left by a previous match. */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Waypoint start) {
        this.alliance = alliance;
        PoseStore.clear();
        this.drive = new Drive(hardwareMap, start.pose(alliance));
    }

    /** Teleop. Resumes the pose auto left behind. */
    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        this.drive = new Drive(hardwareMap);
    }
}
