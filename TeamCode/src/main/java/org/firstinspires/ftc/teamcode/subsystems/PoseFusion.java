package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.PoseBuffer;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * Nudges odometry toward the camera's field pose.
 *
 * <p>Latency first: the camera reports where the robot was when the frame was captured, so the error
 * is measured against the odometry pose from that moment, and the resulting correction is applied to
 * the current pose. Comparing a frame against the present pose is what makes vision correction get
 * worse the faster the robot drives.
 *
 * <p>A fraction of the error is applied per frame rather than the whole of it. {@code
 * PoseTracker.setPose} leaves {@code previousPose} alone, so Pedro's next {@code update()} measures
 * velocity across the jump: a six inch snap at 50 Hz reads as 300 in/s for one loop, and the
 * follower drives on that velocity.
 *
 * <p>Heading is not corrected by default. MegaTag2 is given the robot's yaw and solves position
 * against it, so the yaw it returns is roughly the one it was handed; feeding that back is circular.
 * A heading fix wants MegaTag1, which this does not carry.
 */
public final class PoseFusion extends ForwardSubsystem {

    /** Share of the translation error applied per accepted frame. */
    public static final double TRANSLATION_GAIN = 0.25;

    /** Zero by design: see the class note on MegaTag2 and yaw. */
    public static final double HEADING_GAIN = 0.0;

    /** A frame disagreeing with odometry by more than this is a bad solve, not a correction. */
    public static final double MAX_ERROR_IN = 24.0;

    /** A frame must be this much newer than the last one used, so one frame is applied once. */
    private static final long MIN_FRAME_GAP_NANOS = 1_000_000L;

    private final Vision vision;
    private final MotionSource motion;
    private final Consumer<Pose> apply;
    private final LongSupplier nanos;
    private final PoseBuffer history = new PoseBuffer();

    private boolean applied;
    private long lastCaptureNanos;
    private int corrections;
    private int rejections;
    private double lastErrorIn;

    /**
     * @param apply receives the corrected pose, normally {@code drive::setPose}
     * @param nanos monotonic clock, normally {@code System::nanoTime}
     */
    public PoseFusion(
            Vision vision, MotionSource motion, Consumer<Pose> apply, LongSupplier nanos) {
        this.vision = vision;
        this.motion = motion;
        this.apply = apply;
        this.nanos = nanos;
    }

    @Override
    public void sense() {
        history.add(nanos.getAsLong(), motion.pose());
    }

    @Override
    public void act() {
        Pose seen = vision.fieldPose();
        if (seen == null) {
            return;
        }
        long captureNanos = nanos.getAsLong() - vision.sample().stalenessMs() * 1_000_000L;
        if (applied && captureNanos - lastCaptureNanos < MIN_FRAME_GAP_NANOS) {
            // The same frame read again. Applying it twice counts one measurement twice.
            return;
        }
        Pose then = history.at(captureNanos);
        if (then == null) {
            return;
        }

        double errorX = seen.getX() - then.getX();
        double errorY = seen.getY() - then.getY();
        lastErrorIn = Math.hypot(errorX, errorY);
        if (lastErrorIn > MAX_ERROR_IN) {
            rejections++;
            applied = true;
            lastCaptureNanos = captureNanos;
            return;
        }
        double errorHeading = Field.normalize(seen.getHeading() - then.getHeading());

        Pose current = motion.pose();
        apply.accept(
                new Pose(
                        current.getX() + errorX * TRANSLATION_GAIN,
                        current.getY() + errorY * TRANSLATION_GAIN,
                        Field.normalize(current.getHeading() + errorHeading * HEADING_GAIN)));
        corrections++;
        applied = true;
        lastCaptureNanos = captureNanos;
    }

    /** Frames accepted and applied. */
    public int corrections() {
        return corrections;
    }

    /** Frames dropped for disagreeing with odometry by more than {@link #MAX_ERROR_IN}. */
    public int rejections() {
        return rejections;
    }

    /** Distance between the last frame and odometry at capture time, inches. */
    public double lastErrorIn() {
        return lastErrorIn;
    }

    @Override
    public void logSignals(RunLog log) {
        log.addSignal("fusion.errorIn", this::lastErrorIn);
        log.addSignal("fusion.corrections", () -> corrections);
        log.addSignal("fusion.rejections", () -> rejections);
    }
}
