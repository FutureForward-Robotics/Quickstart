package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One camera reading, immutable. Angles are degrees, distances inches, {@code stalenessMs} the age
 * of the frame at the moment it was read.
 *
 * <p>Built by a {@link VisionCamera} once per loop. Copy methods follow {@code DriveInput}: start
 * from the targeting values, then add what the pipeline actually produced.
 */
public final class VisionSample {

    /** One fiducial in the frame. {@code txDegrees} is positive to the right of the crosshair. */
    public static final class Tag {
        public final int id;
        public final double txDegrees;
        public final double tyDegrees;
        public final double area;

        public Tag(int id, double txDegrees, double tyDegrees, double area) {
            this.id = id;
            this.txDegrees = txDegrees;
            this.tyDegrees = tyDegrees;
            this.area = area;
        }
    }

    private static final VisionSample NONE = new VisionSample(false, 0, -1, 0, 0, 0);

    private final boolean valid;
    private final long stalenessMs;
    private final int pipeline;
    private final double tx;
    private final double ty;
    private final double ta;

    private Pose pose;
    private int tagCount;
    private double avgTagDistanceIn;
    private List<Tag> tags = Collections.emptyList();

    public VisionSample(
            boolean valid, long stalenessMs, int pipeline, double tx, double ty, double ta) {
        this.valid = valid;
        this.stalenessMs = stalenessMs;
        this.pipeline = pipeline;
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
    }

    /** Nothing seen: invalid, no tags, no pose. */
    public static VisionSample none() {
        return NONE;
    }

    /** Attaches a field pose already converted to Pedro's frame. */
    public VisionSample withPose(Pose pose, int tagCount, double avgTagDistanceIn) {
        VisionSample copy = copy();
        copy.pose = pose;
        copy.tagCount = tagCount;
        copy.avgTagDistanceIn = avgTagDistanceIn;
        return copy;
    }

    public VisionSample withTags(List<Tag> tags) {
        VisionSample copy = copy();
        copy.tags = Collections.unmodifiableList(new ArrayList<>(tags));
        return copy;
    }

    private VisionSample copy() {
        VisionSample copy = new VisionSample(valid, stalenessMs, pipeline, tx, ty, ta);
        copy.pose = pose;
        copy.tagCount = tagCount;
        copy.avgTagDistanceIn = avgTagDistanceIn;
        copy.tags = tags;
        return copy;
    }

    /**
     * The camera had a target when the frame was captured. False also covers a camera that has not
     * been reached yet, which is why {@code getLatestResult() != null} is not a usable check.
     */
    public boolean isValid() {
        return valid;
    }

    public long stalenessMs() {
        return stalenessMs;
    }

    public int pipeline() {
        return pipeline;
    }

    /** Crosshair offsets, degrees, and target area as a percentage of the frame. */
    public double tx() {
        return tx;
    }

    public double ty() {
        return ty;
    }

    public double ta() {
        return ta;
    }

    public List<Tag> tags() {
        return tags;
    }

    public int tagCount() {
        return tagCount;
    }

    public double avgTagDistanceIn() {
        return avgTagDistanceIn;
    }

    /** Field pose in Pedro's frame, or null when the pipeline produced none. */
    public Pose pose() {
        return pose;
    }

    public boolean sawTag(int id) {
        return tag(id) != null;
    }

    /** Horizontal offset to one tag in degrees, or NaN when that tag is not in the frame. */
    public double txForTag(int id) {
        Tag found = tag(id);
        return found == null ? Double.NaN : found.txDegrees;
    }

    private Tag tag(int id) {
        for (int i = 0; i < tags.size(); i++) {
            if (tags.get(i).id == id) {
                return tags.get(i);
            }
        }
        return null;
    }
}
