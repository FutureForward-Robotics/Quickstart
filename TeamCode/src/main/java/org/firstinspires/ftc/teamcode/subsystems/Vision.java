package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.util.VisionCamera;
import org.firstinspires.ftc.teamcode.util.VisionSample;

/**
 * Camera read once per loop.
 *
 * <p>The camera is a network round-trip, so {@link #sense()} takes exactly one reading and every
 * getter answers from it. Reading the device per question instead is how a targeting value and the
 * pose it was paired with end up describing different frames.
 *
 * <p>{@link #act()} is empty: a camera has no outputs. The heading handed to the camera comes from
 * the drivetrain's sense phase, so this must be constructed after the drivetrain to be refreshed
 * after it.
 */
public final class Vision extends ForwardSubsystem {

    /** A frame older than this is ignored. One control loop at 50 Hz is 20 ms. */
    public static final long MAX_STALENESS_MS = 100;

    /**
     * Tag distance beyond which a pose is not trusted, inches. A per-season guess: tag apparent
     * size sets the noise floor, so check it against odometry on this year's field.
     */
    public static final double MAX_POSE_DISTANCE_IN = 96;

    private final VisionCamera camera;
    private final MotionSource motion;

    private VisionSample sample = VisionSample.none();

    public Vision(VisionCamera camera, MotionSource motion) {
        this.camera = camera;
        this.motion = motion;
    }

    @Override
    public void sense() {
        // Orientation first: MegaTag2 fuses the yaw the camera was last given, and the reading
        // below was captured before this call, so it reflects the previous loop's heading.
        camera.orient(Math.toDegrees(motion.pose().heading()));
        sample = camera.read();
    }

    @Override
    public void act() {}

    /** This loop's reading, whether or not it is fresh. */
    public VisionSample sample() {
        return sample;
    }

    /** A target was seen in a frame new enough to act on. */
    public boolean hasTarget() {
        return sample.isValid() && sample.stalenessMs() <= MAX_STALENESS_MS;
    }

    /** Crosshair offset in degrees, positive to the right, or NaN with no fresh target. */
    public double tx() {
        return hasTarget() ? sample.tx() : Double.NaN;
    }

    public double ty() {
        return hasTarget() ? sample.ty() : Double.NaN;
    }

    public boolean sawTag(int id) {
        return hasTarget() && sample.sawTag(id);
    }

    /** Offset to one tag in degrees, or NaN when it is not in a fresh frame. */
    public double txForTag(int id) {
        return hasTarget() ? sample.txForTag(id) : Double.NaN;
    }

    /**
     * Field pose in Pedro's frame, or null when there is nothing trustworthy. Feed it to {@code
     * Drive.setPose} to re-seed odometry; nothing here writes it for you, because a pose that
     * disagrees with the wheels mid-path is a decision, not a detail.
     */
    public Pose fieldPose() {
        if (!hasTarget() || sample.pose() == null || sample.tagCount() == 0) {
            return null;
        }
        return sample.avgTagDistanceIn() <= MAX_POSE_DISTANCE_IN ? sample.pose() : null;
    }

    public void pipeline(int index) {
        camera.pipeline(index);
    }

    @Override
    public void logSignals(RunLog log) {
        log.addFlag("vision.hasTarget", this::hasTarget);
        log.addSignal("vision.stalenessMs", () -> sample.stalenessMs());
        log.addSignal("vision.tx", this::tx);
        log.addSignal("vision.ty", this::ty);
        log.addSignal("vision.tags", () -> sample.tags().size());
        log.addSignal("vision.poseX", () -> fieldPose() == null ? Double.NaN : fieldPose().x());
        log.addSignal("vision.poseY", () -> fieldPose() == null ? Double.NaN : fieldPose().y());
    }
}
