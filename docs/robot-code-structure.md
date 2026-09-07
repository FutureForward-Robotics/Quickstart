# Robot Code Structure

Every OpMode in this repository follows the same structure.

## OpModes

An OpMode extends `ForwardOpMode`. You write one method, `configure()`, which builds your subsystems,
sets their default commands, and binds buttons.

```java
@TeleOp(name = "TeleOp Red", group = "match")
public class TeleOpRed extends MatchTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
```

`initialize()` is final. It resets the command scheduler, clears the subsystem registry, sets up bulk
caching and the gamepads, opens the run log, and then calls your `configure()`. The order matters,
so you cannot override it.

Methods available inside an OpMode:

| Method | Returns |
| --- | --- |
| `configure()` | You implement this. Build subsystems, set default commands, bind buttons. |
| `logSignals(RunLog log)` | Optional. Register log signals for things that are not subsystems. |
| `dtSeconds()` | Seconds since the previous loop |
| `loopHz()` | Current loop rate |
| `loopId()` | A counter that increases once per loop, as a `LongSupplier` |
| `telemetryDue(long intervalMs)` | True at most once per interval, for rate-limiting telemetry |
| `log()` | The run log for this OpMode |

## The loop

`ForwardOpMode.run()` does the same five things every loop, in this order:

1. Clear the bulk cache on every hub, read the gamepads, tick the loop timer.
2. Call `sense()` on every subsystem.
3. Run the command scheduler, which fires button bindings and runs command bodies.
4. Call `act()` on every subsystem.
5. Write one row to the run log.

During init, before the driver presses Play, the loop runs steps 1 through 3 only. Nothing moves,
because `act()` is not called.

## Subsystems

A subsystem extends `ForwardSubsystem` and implements two methods.

```java
public class Lift extends ForwardSubsystem {

    private final DcMotorEx motor;
    private double position;
    private double target;

    public Lift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
    }

    @Override
    public void sense() {
        position = motor.getCurrentPosition();
    }

    @Override
    public void act() {
        motor.setPower(KP * (target - position));
    }
}
```

`sense()` reads sensors into fields. It must not write to motors or servos.

`act()` writes to motors and servos, using the values `sense()` stored and whatever a command set as
a target.

Write only when the value changed by enough to matter. One write to the hub costs about 2.5 ms,
measured by timing each subsystem's act phase on a Control Hub, so an output recomputed every loop by
a control loop spends the loop budget sending changes the hardware cannot resolve:

```java
if (BusWrites.worthMotor(power, lastWritten)) {
    motor.setPower(power);
    lastWritten = power;
}
```

`BusWrites.worthServo` does the same for a servo position. Comparing with `!=` is not enough: a
control loop output is almost never bit identical to the last one.

Subsystems register themselves when you construct them, so there is no list to keep up to date.

### Sense and act phases

Every subsystem senses before any subsystem acts. This means all of them see the same snapshot of the
robot for a given loop. If a mechanism reacted to a sensor that another mechanism had already changed
partway through the loop, the behavior would depend on the order the subsystems happened to be
constructed in. Splitting the phases removes that ordering problem.

> **Note**
> Bulk caching is set to `MANUAL` and the cache is cleared once at the top of each loop. Every sensor
> read inside one loop returns the same value. Do not write a loop that waits for a sensor to change
> inside a single `sense()` call, because it never will.

### Construction order

Construction order sets the order of `sense()` and `act()` calls. This only matters when one
subsystem reads a value another subsystem produced during its own `sense()`. `Vision` needs the
drivetrain's heading, so it is built after the drivetrain.

```java
robot = new Robot(hardwareMap, alliance());        // builds Drive
vision = new Vision(camera, robot.drive);          // senses after Drive
fusion = new PoseFusion(vision, robot.drive, robot.drive::setPose, System::nanoTime);
```

## Commands

Commands come from SolversLib. A command requires the subsystems it controls, and the scheduler
guarantees only one command controls a subsystem at a time.

A default command runs whenever nothing else has claimed the subsystem. Driver control is a default
command:

```java
robot.drive.setDefaultCommand(
        robot.drive.teleop(
                driver::getLeftY,
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));
```

When you schedule a path-following command, it takes the drivetrain, the default command stops, and
the scheduler restores the default command as soon as the path ends.

Bind buttons in `configure()`:

```java
driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        .whenHeld(robot.drive.assist(Assists.speedCap(0.35)));
```

Assists layer, so more than one can be active. `Assists.steadyShot` is the one with state: it holds
a heading and replaces the stick's magnitude with a fixed speed, which is what makes a shot on the
move predictable. `Drive.steadyShot` latches the heading when the command is scheduled, so bind it
with `whenHeld`, never `whileHeld`.

## Pure computation

Some code reads no hardware and writes no outputs. An aiming solver that turns a robot position into
a turret angle is an example. This does not belong in a subsystem. Make it a plain class, give its
constructor a `LongSupplier loopId`, and have it recompute at most once per loop:

```java
private void solve() {
    long loop = loopId.getAsLong();
    if (solvedLoop == loop) {
        return;
    }
    solvedLoop = loop;
    // compute and store every output field
}
```

Every getter calls `solve()` first. The first caller in a loop pays for the computation and the rest
read the stored answer. Because the check is against the loop counter, the answer can never be stale
and it cannot be recomputed twice with different sensor data.

Register its values for logging from the OpMode's `logSignals`, since it is not in the subsystem
registry.
