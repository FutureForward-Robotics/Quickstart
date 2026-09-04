package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.fakes.FakeVisionCamera;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.VisionSample;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicLong;

/**
 * Pose fusion driven by an explicit clock. The robot is moved by hand between loops, so a frame's
 * age is exactly known and latency compensation can be checked against arithmetic.
 */
@RobotTest
class PoseFusionTest {

    private static final long MS = 1_000_000L;
    private static final double EPS = 1e-9;

    private final AtomicLong clock = new AtomicLong();
    private final FakeVisionCamera camera = new FakeVisionCamera();
    private final Odometry odometry = new Odometry();

    /** Odometry the test drives directly, standing in for the drivetrain. */
    private static final class Odometry implements MotionSource {
        Pose pose = new Pose();

        @Override
        public Pose pose() {
            return pose;
        }

        @Override
        public double velocityX() {
            return 0;
        }

        @Override
        public double velocityY() {
            return 0;
        }
    }

    private Vision vision;
    private PoseFusion fusion;

    private void build() {
        vision = new Vision(camera, odometry);
        fusion = new PoseFusion(vision, odometry, p -> odometry.pose = p, clock::get);
    }

    /** One loop of the real order: vision senses, fusion records, fusion corrects. */
    private void loop() {
        vision.sense();
        fusion.sense();
        fusion.act();
    }

    private void advance(long ms) {
        clock.addAndGet(ms * MS);
    }

    private static VisionSample seeing(Pose pose, long stalenessMs) {
        return new VisionSample(true, stalenessMs, 0, 0, 0, 1)
                .withPose(pose, 2, 24);
    }

    @Test
    void correctsAFractionOfTheErrorNotAllOfIt() {
        build();
        odometry.pose = new Pose(100, 100, 0);
        camera.set(seeing(new Pose(104, 100, 0), 0));

        loop();

        assertEquals(
                100 + 4 * PoseFusion.TRANSLATION_GAIN,
                odometry.pose.getX(),
                EPS,
                "a snap would read as a velocity spike through Pedro's previousPose");
        assertEquals(1, fusion.corrections());
    }

    @Test
    void measuresTheErrorWhereTheRobotWasWhenTheFrameWasTaken() {
        build();
        // Standing at x=100 when the frame is captured.
        odometry.pose = new Pose(100, 100, 0);
        camera.set(VisionSample.none());
        loop();

        // Drives 40 inches over the next 100 ms, then a 100 ms old frame arrives saying x=100:
        // odometry was right all along and the correction must be zero.
        advance(100);
        odometry.pose = new Pose(140, 100, 0);
        camera.set(seeing(new Pose(100, 100, 0), 100));

        loop();

        assertEquals(140, odometry.pose.getX(), 1e-6, "no correction: the frame agrees with x=100");
        assertEquals(1, fusion.corrections());
    }

    @Test
    void oneFrameIsAppliedOnce() {
        build();
        odometry.pose = new Pose(100, 100, 0);
        camera.set(seeing(new Pose(108, 100, 0), 0));

        loop();
        double afterFirst = odometry.pose.getX();

        // Same frame, one loop later: staleness grew, capture time did not move.
        advance(20);
        camera.set(seeing(new Pose(108, 100, 0), 20));
        loop();

        assertEquals(afterFirst, odometry.pose.getX(), EPS, "counting one measurement twice");
        assertEquals(1, fusion.corrections());
    }

    @Test
    void aWildDisagreementIsRejected() {
        build();
        odometry.pose = new Pose(100, 100, 0);
        camera.set(seeing(new Pose(100 + PoseFusion.MAX_ERROR_IN + 1, 100, 0), 0));

        loop();

        assertEquals(100, odometry.pose.getX(), EPS, "a bad solve must not move the robot");
        assertEquals(0, fusion.corrections());
        assertEquals(1, fusion.rejections());
    }

    @Test
    void repeatedFramesConvergeWithoutOvershooting() {
        build();
        odometry.pose = new Pose(100, 100, 0);

        for (int i = 0; i < 40; i++) {
            advance(20);
            camera.set(seeing(new Pose(110, 100, 0), 0));
            loop();
            assertTrue(odometry.pose.getX() <= 110 + EPS, "never past the measurement");
        }

        assertEquals(110, odometry.pose.getX(), 0.01, "converged on the camera's pose");
    }

    @Test
    void headingIsLeftAloneByDefault() {
        build();
        odometry.pose = new Pose(100, 100, Math.toRadians(30));
        camera.set(seeing(new Pose(100, 100, Math.toRadians(90)), 0));

        loop();

        assertEquals(
                Math.toRadians(30),
                odometry.pose.getHeading(),
                EPS,
                "MegaTag2 returns the yaw it was given, so feeding it back is circular");
    }

    @Test
    void noTrustedPoseMeansNoCorrection() {
        build();
        odometry.pose = new Pose(100, 100, 0);
        camera.set(new VisionSample(true, 0, 0, 3, 0, 1));

        loop();

        assertEquals(100, odometry.pose.getX(), EPS);
        assertEquals(0, fusion.corrections());
    }
}
