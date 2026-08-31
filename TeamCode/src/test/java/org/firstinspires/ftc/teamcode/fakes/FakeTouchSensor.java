package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.TouchSensor;

/** Touch sensor whose state the test sets directly. */
public class FakeTouchSensor implements TouchSensor {

    private final String name;
    private boolean pressed;

    public FakeTouchSensor(String name) {
        this.name = name;
    }

    public void setPressed(boolean p) {
        pressed = p;
    }

    @Override
    public boolean isPressed() {
        return pressed;
    }

    @Override
    public double getValue() {
        return pressed ? 1 : 0;
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
