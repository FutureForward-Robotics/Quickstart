package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.fakes.FakeVisionCamera;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.util.VisionSample;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;

/** Vision against a fake camera: one read per loop, staleness gating and tag lookup. */
@RobotTest
class VisionTest {

    private static final double EPS = 1e-9;

    private static final class StubMotion implements MotionSource {
        private Pose pose = new Pose();

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

    private static VisionSample fresh() {
        return new VisionSample(true, 20, 0, 3.5, -1.25, 4.0);
    }

    @Test
    void readsTheCameraOncePerLoopHoweverManyQuestionsAreAsked(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(fresh());

        runner.loop();
        vision.hasTarget();
        vision.tx();
        vision.ty();
        vision.sawTag(21);
        vision.fieldPose();

        assertEquals(1, camera.reads(), "five consumers, one network round-trip");
    }

    @Test
    void orientsTheCameraBeforeReadingIt(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        StubMotion motion = new StubMotion();
        motion.pose = new Pose(0, 0, Math.toRadians(90));
        Vision vision = new Vision(camera, motion);

        runner.loop();

        // MegaTag2 fuses the last orientation it was given, so a read that precedes the first
        // orient carries no yaw at all.
        assertEquals(90.0, camera.headingAtRead(), 1e-6, "degrees, converted from Pedro radians");
    }

    @Test
    void aStaleFrameIsNotATarget(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(new VisionSample(true, Vision.MAX_STALENESS_MS + 1, 0, 3.5, 0, 4.0));

        runner.loop();

        assertFalse(vision.hasTarget(), "valid but older than the staleness limit");
        assertTrue(Double.isNaN(vision.tx()), "no fresh target means no angle, not a stale one");
    }

    @Test
    void anInvalidFrameYieldsNoTargetAndNoPose(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(VisionSample.none());

        runner.loop();

        assertFalse(vision.hasTarget());
        assertNull(vision.fieldPose());
        assertTrue(Double.isNaN(vision.txForTag(24)));
    }

    @Test
    void findsTheAngleToOneTagAndIgnoresTheOthers(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(
                fresh().withTags(
                                Arrays.asList(
                                        new VisionSample.Tag(20, -7.5, 1.0, 2.0),
                                        new VisionSample.Tag(24, 12.25, 2.0, 3.0))));

        runner.loop();

        assertTrue(vision.sawTag(24));
        assertFalse(vision.sawTag(21));
        assertEquals(12.25, vision.txForTag(24), EPS);
        assertTrue(Double.isNaN(vision.txForTag(21)), "absent tag is NaN, not zero");
    }

    @Test
    void aPoseFromTooFarAwayIsNotTrusted(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        Pose seen = new Pose(72, 72, 0);

        camera.set(fresh().withPose(seen, 1, Vision.MAX_POSE_DISTANCE_IN - 1));
        runner.loop();
        assertSame(seen, vision.fieldPose(), "within range");

        camera.set(fresh().withPose(seen, 1, Vision.MAX_POSE_DISTANCE_IN + 1));
        runner.loop();
        assertNull(vision.fieldPose(), "beyond the distance limit");
    }

    @Test
    void aPoseWithNoTagsBehindItIsNotTrusted(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(fresh().withPose(new Pose(10, 10, 0), 0, 12));

        runner.loop();
        assertNull(vision.fieldPose());
    }

    @Test
    void signalsSurviveAnEmptyFrame(LoopRunner runner, @TempDir File logDir) throws IOException {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, new StubMotion());
        camera.set(VisionSample.none());

        RunLog log = RunLog.open(logDir, "Vision");
        vision.logSignals(log);
        runner.loop();
        log.writeLoop(0);
        log.close();

        List<String> csv = Files.readAllLines(new File(log.directory(), "signals.csv").toPath());
        assertEquals(
                "t_s,loop,vision.hasTarget,vision.stalenessMs,vision.tx,vision.ty,vision.tags,"
                        + "vision.poseX,vision.poseY",
                csv.get(0));
        // No pose means an empty field, not an NPE swallowed by writeLoop's supplier guard.
        String[] fields = csv.get(1).split(",", -1);
        assertEquals("0", fields[2], "hasTarget is false");
        assertEquals("", fields[4], "tx is NaN with no target");
        assertEquals("", fields[7], "poseX is NaN with no pose");
        assertEquals(0, log.dropped());
    }
}
