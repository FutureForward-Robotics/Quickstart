package org.firstinspires.ftc.teamcode.util;

/**
 * Read side of a vision camera, so everything above it is testable without hardware.
 *
 * <p>{@link #orient(double)} must be called before {@link #read()} in the same loop: MegaTag2 fuses
 * the orientation the camera was last given, so a reading taken before the first orientation
 * carries no yaw at all.
 */
public interface VisionCamera {

    /** Robot heading in degrees, counter-clockwise positive. */
    void orient(double headingDegrees);

    /** One reading. Never null; {@link VisionSample#none()} when nothing was seen. */
    VisionSample read();

    void pipeline(int index);
}
