package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Digital channel whose input state the test sets.
 *
 * <p>A limit switch wired to a digital port reads {@code false} when pressed, because the port is
 * pulled up. {@link #setPressed} models that; {@link #setState} sets the raw line if you need it.
 */
public class FakeDigitalChannel implements DigitalChannel {

    private final String name;
    private Mode mode = Mode.INPUT;
    private boolean state = true;

    public FakeDigitalChannel(String name) {
        this.name = name;
    }

    /** Pressed pulls the line low. */
    public void setPressed(boolean pressed) {
        state = !pressed;
    }

    public boolean isPressed() {
        return !state;
    }

    @Override
    public boolean getState() {
        return state;
    }

    @Override
    public void setState(boolean newState) {
        state = newState;
    }

    @Override
    public Mode getMode() {
        return mode;
    }

    @Override
    public void setMode(Mode newMode) {
        mode = newMode;
    }

    @Override
    public void setMode(DigitalChannelController.Mode newMode) {
        mode = newMode == DigitalChannelController.Mode.OUTPUT ? Mode.OUTPUT : Mode.INPUT;
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
