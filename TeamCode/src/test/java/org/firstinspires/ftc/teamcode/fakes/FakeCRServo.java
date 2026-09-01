package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Continuous-rotation servo that records commanded power. Power is clamped to [-1, 1] as the SDK
 * does; direction is stored but not applied, so {@link #getPower()} is what was commanded.
 *
 * <p>{@link #writes()} counts {@code setPower} calls, to prove redundant Lynx writes are
 * suppressed.
 */
public class FakeCRServo implements CRServo {

    private final String name;
    private double power;
    private Direction direction = Direction.FORWARD;
    private int writes;

    public FakeCRServo(String name) {
        this.name = name;
    }

    @Override
    public void setPower(double p) {
        writes++;
        power = Math.max(-1, Math.min(1, p));
    }

    @Override
    public double getPower() {
        return power;
    }

    /** Number of {@code setPower} calls since construction. */
    public int writes() {
        return writes;
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
