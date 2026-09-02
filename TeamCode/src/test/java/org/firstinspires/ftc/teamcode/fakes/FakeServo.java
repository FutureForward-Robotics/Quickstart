package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Servo that records commanded position. Range is clamped as the SDK does.
 *
 * <p>{@link #getPosition()} returns the logical position, matching {@code ServoImpl}, which scales
 * the controller value back into [0, 1]. The scaled command sent to the hardware is
 * {@link #commanded()}.
 */
public class FakeServo implements Servo {

    private final String name;
    private double position;
    private double commanded;
    private Direction direction = Direction.FORWARD;
    private double scaleMin = 0;
    private double scaleMax = 1;

    public FakeServo(String name) {
        this.name = name;
    }

    @Override
    public void setPosition(double p) {
        position = Math.max(0, Math.min(1, p));
        commanded = scaleMin + position * (scaleMax - scaleMin);
    }

    @Override
    public double getPosition() {
        return position;
    }

    /** Scaled value the controller would receive, which {@link #scaleRange} shifts. */
    public double commanded() {
        return commanded;
    }

    @Override
    public void scaleRange(double min, double max) {
        scaleMin = min;
        scaleMax = max;
    }

    @Override
    public void setDirection(Direction d) {
        direction = d;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public String getDeviceName() {
        return name;
    }

    @Override
    public String getConnectionInfo() {
        return "fake";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {}

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }
}
