package org.firstinspires.ftc.teamcode.fakes;

import org.firstinspires.ftc.teamcode.util.VisionCamera;
import org.firstinspires.ftc.teamcode.util.VisionSample;

/**
 * Camera whose reading the test sets. Counts reads, because reading once per loop is the contract
 * {@code Vision} exists to enforce, and records the heading it was given at the moment it was read.
 */
public final class FakeVisionCamera implements VisionCamera {

    private VisionSample next = VisionSample.none();
    private int reads;
    private int pipeline;
    private double headingAtRead = Double.NaN;
    private double lastHeading = Double.NaN;

    public void set(VisionSample sample) {
        next = sample;
    }

    @Override
    public void orient(double headingDegrees) {
        lastHeading = headingDegrees;
    }

    @Override
    public VisionSample read() {
        reads++;
        headingAtRead = lastHeading;
        return next;
    }

    @Override
    public void pipeline(int index) {
        pipeline = index;
    }

    public int reads() {
        return reads;
    }

    public int pipelineIndex() {
        return pipeline;
    }

    /** Heading supplied before the most recent read, NaN if orient was never called first. */
    public double headingAtRead() {
        return headingAtRead;
    }
}
