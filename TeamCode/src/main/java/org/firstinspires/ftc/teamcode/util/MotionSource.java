package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Read side of the drivetrain: where the robot is and how fast it is moving, as cached by the
 * drivetrain's sense phase. Inches, radians, inches per second.
 *
 * <p>Pure per-loop computations depend on this rather than on the drivetrain itself, so they carry
 * no hardware and can be tested off-robot against a stub.
 */
public interface MotionSource {

    Pose pose();

    double velocityX();

    double velocityY();
}
