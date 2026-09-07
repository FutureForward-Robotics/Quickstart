package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * DcMotorEx whose encoder integrates applied power over stepped time.
 *
 * <p>{@code ticks += power * ticksPerSecondAtFullPower * dt}, with direction and zero-power
 * behaviour applied. Call {@link #step(double)} to advance.
 */
public class FakeMotor implements DcMotorEx {

    private final String name;
    private final double ticksPerSecondAtFullPower;

    private double ticks;
    private double power;
    private double velocityTicksPerSecond;
    private Direction direction = Direction.FORWARD;
    private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    private int targetPosition;
    private int targetPositionTolerance = 5;
    private boolean enabled = true;
    private boolean encoderInverted;
    private double currentAmps;
    private double currentAlertAmps;

    public FakeMotor(String name) {
        this(name, 2000);
    }

    public FakeMotor(String name, double ticksPerSecondAtFullPower) {
        this.name = name;
        this.ticksPerSecondAtFullPower = ticksPerSecondAtFullPower;
    }

    /** Advance the simulation by {@code dtSeconds}. */
    public void step(double dtSeconds) {
        double effective = enabled ? power : 0;
        if (effective == 0 && zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE) {
            velocityTicksPerSecond = 0;
            return;
        }
        double sign = direction == Direction.REVERSE ? -1 : 1;
        velocityTicksPerSecond = effective * sign * (encoderInverted ? -1 : 1)
                * ticksPerSecondAtFullPower;
        ticks += velocityTicksPerSecond * dtSeconds;
    }

    /**
     * Models an encoder that counts against the commanded power, from gearing or wiring rather
     * than from {@link #setDirection}. {@code setDirection} cannot express this: the SDK applies it
     * to the power and to the encoder read together, so the two cancel.
     */
    public void setEncoderInverted(boolean inverted) {
        encoderInverted = inverted;
    }

    /**
     * Force the encoder, in the same frame {@link #getCurrentPosition()} reports, so a test reads
     * back what it set.
     */
    public void setEncoder(double newTicks) {
        ticks = adjust(newTicks);
    }

    public void setCurrent(double amps) {
        currentAmps = amps;
    }

    // ------------------------------------------------------------------ DcMotorEx

    @Override
    public void setPower(double p) {
        power = Math.max(-1, Math.min(1, p));
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public int getCurrentPosition() {
        return (int) Math.round(adjust(ticks));
    }

    /** The SDK's adjustPosition and adjustAngularRate: direction is applied on the way out too. */
    private double adjust(double raw) {
        return direction == Direction.REVERSE ? -raw : raw;
    }

    @Override
    public double getVelocity() {
        return adjust(velocityTicksPerSecond);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return adjust(velocityTicksPerSecond);
    }

    @Override
    public void setVelocity(double ticksPerSecond) {
        setPower(ticksPerSecond / ticksPerSecondAtFullPower);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(angularRate);
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
    public void setMode(DcMotor.RunMode m) {
        mode = m;
        if (m == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            ticks = 0;
            velocityTicksPerSecond = 0;
        }
    }

    @Override
    public DcMotor.RunMode getMode() {
        return mode;
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        zeroPowerBehavior = behavior;
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return mode == DcMotor.RunMode.RUN_TO_POSITION
                && Math.abs(getCurrentPosition() - targetPosition) > targetPositionTolerance;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        targetPositionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return targetPositionTolerance;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return unit.convert(currentAmps, CurrentUnit.AMPS);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return unit.convert(currentAlertAmps, CurrentUnit.AMPS);
    }

    @Override
    public void setCurrentAlert(double amps, CurrentUnit unit) {
        currentAlertAmps = CurrentUnit.AMPS.convert(amps, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return currentAlertAmps > 0 && currentAmps >= currentAlertAmps;
    }

    @Override
    public void setMotorEnable() {
        enabled = true;
    }

    @Override
    public void setMotorDisable() {
        enabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return enabled;
    }

    @Override
    public void setPowerFloat() {
        power = 0;
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public boolean getPowerFloat() {
        return zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
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

    // Unused by subsystem code; present to satisfy the interface.

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return MotorConfigurationType.getUnspecifiedMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType type) {}

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode m) {
        return new PIDCoefficients();
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode m, PIDCoefficients c) {}

    @Override
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode m) {
        return new PIDFCoefficients();
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode m, PIDFCoefficients c) {}

    @Override
    public void setPositionPIDFCoefficients(double p) {}

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
}
