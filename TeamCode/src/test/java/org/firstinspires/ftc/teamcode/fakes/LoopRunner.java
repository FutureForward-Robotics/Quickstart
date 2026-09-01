package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.ForwardSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Runs the robot loop off-robot: sense, scheduler, act, then advance fake hardware by one tick.
 * Mirrors {@code ForwardOpMode.run()} without gamepads or bulk caching.
 *
 * <p>Devices created here are registered by name in {@link #hardwareMap()}, a stub, because a real
 * {@code HardwareMap.get} calls native code and throws off-robot. Lookups by an unregistered name
 * or the wrong type throw.
 */
public final class LoopRunner {

    public static final double DEFAULT_DT_SECONDS = 0.020;

    private final double dtSeconds;
    private final List<FakeMotor> motors = new ArrayList<>();
    private final Map<String, HardwareDevice> devices = new LinkedHashMap<>();
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();
    private final HardwareMap hardwareMap = mock(HardwareMap.class);

    private double elapsedSeconds;
    private int loops;

    public LoopRunner() {
        this(DEFAULT_DT_SECONDS);
    }

    public LoopRunner(double dtSeconds) {
        this.dtSeconds = dtSeconds;
        stubHardwareMap();
    }

    /** Clears the scheduler and subsystem registry, both of which are static. */
    public static void reset() {
        CommandScheduler.getInstance().reset();
        ForwardSubsystem.resetRegistry();
    }

    // ------------------------------------------------------------------ hardware

    /** Stubbed map holding the devices created here. */
    public HardwareMap hardwareMap() {
        return hardwareMap;
    }

    /** Creates a motor, registers it under {@code name}, and advances it each loop. */
    public FakeMotor motor(String name) {
        return motor(name, 2000);
    }

    public FakeMotor motor(String name, double ticksPerSecondAtFullPower) {
        FakeMotor fake = new FakeMotor(name, ticksPerSecondAtFullPower);
        motors.add(fake);
        return register(name, fake);
    }

    public FakeServo servo(String name) {
        return register(name, new FakeServo(name));
    }

    public FakeDigitalChannel digitalChannel(String name) {
        return register(name, new FakeDigitalChannel(name));
    }

    /**
     * Registers a battery voltage sensor. Code reaches it through {@code hardwareMap.voltageSensor}
     * rather than by name, which is how the SDK exposes it.
     */
    public FakeVoltageSensor voltageSensor(double volts) {
        FakeVoltageSensor sensor = new FakeVoltageSensor("battery", volts);
        voltageSensors.add(sensor);
        return sensor;
    }

    /** Registers a device built elsewhere. */
    public <T extends HardwareDevice> T register(String name, T device) {
        devices.put(name, device);
        return device;
    }

    /** Motors advanced each loop. Not needed for motors created by {@link #motor}. */
    public LoopRunner stepping(FakeMotor... fakeMotors) {
        motors.addAll(Arrays.asList(fakeMotors));
        return this;
    }

    private void stubHardwareMap() {
        when(hardwareMap.get(any(Class.class), anyString()))
                .thenAnswer(
                        invocation -> {
                            Class<?> type = invocation.getArgument(0);
                            String name = invocation.getArgument(1);
                            HardwareDevice device = devices.get(name);
                            if (device == null) {
                                throw new IllegalArgumentException(
                                        "no device named \""
                                                + name
                                                + "\"; registered: "
                                                + devices.keySet());
                            }
                            if (!type.isInstance(device)) {
                                throw new IllegalArgumentException(
                                        "device \""
                                                + name
                                                + "\" is a "
                                                + device.getClass().getSimpleName()
                                                + ", not a "
                                                + type.getSimpleName());
                            }
                            return device;
                        });

        when(hardwareMap.tryGet(any(Class.class), anyString()))
                .thenAnswer(
                        invocation -> {
                            Class<?> type = invocation.getArgument(0);
                            HardwareDevice device = devices.get(invocation.getArgument(1));
                            return type.isInstance(device) ? device : null;
                        });

        // voltageSensor is a plain public field on HardwareMap, not a get() call, so it has to be
        // installed rather than stubbed. Without it any subsystem reading battery voltage NPEs.
        @SuppressWarnings("unchecked")
        HardwareMap.DeviceMapping<VoltageSensor> mapping =
                mock(HardwareMap.DeviceMapping.class);
        when(mapping.iterator()).thenAnswer(invocation -> voltageSensors.iterator());
        hardwareMap.voltageSensor = mapping;
    }

    // ------------------------------------------------------------------ loop

    public void loop() {
        ForwardSubsystem.senseAll();
        CommandScheduler.getInstance().run();
        ForwardSubsystem.actAll();
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).step(dtSeconds);
        }
        elapsedSeconds += dtSeconds;
        loops++;
    }

    public void loops(int count) {
        for (int i = 0; i < count; i++) {
            loop();
        }
    }

    /** Runs until {@code done}, failing the test if {@code maxLoops} is reached first. */
    public int until(BooleanSupplier done, int maxLoops) {
        for (int i = 0; i < maxLoops; i++) {
            if (done.getAsBoolean()) {
                return i;
            }
            loop();
        }
        if (!done.getAsBoolean()) {
            fail("condition not met within " + maxLoops + " loops (" + elapsedSeconds + "s)");
        }
        return maxLoops;
    }

    public double elapsedSeconds() {
        return elapsedSeconds;
    }

    public int loopCount() {
        return loops;
    }
}
