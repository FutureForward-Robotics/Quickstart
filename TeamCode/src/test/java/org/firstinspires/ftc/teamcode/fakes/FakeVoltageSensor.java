package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Battery voltage the test sets. Counts reads, because a real voltage read is a blocking round trip
 * outside the Lynx bulk cache and subsystems are expected to sample it sparingly.
 */
public class FakeVoltageSensor implements VoltageSensor {

    private final String name;
    private double volts;
    private int reads;

    public FakeVoltageSensor(String name, double volts) {
        this.name = name;
        this.volts = volts;
    }

    public void setVolts(double newVolts) {
        volts = newVolts;
    }

    public int reads() {
        return reads;
    }

    @Override
    public double getVoltage() {
        reads++;
        return volts;
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
}
