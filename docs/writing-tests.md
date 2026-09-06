# Writing Tests

You can run the robot loop on your computer, with fake motors and sensors, and check that a mechanism
behaves the way you expect. This catches sign errors, state machine mistakes, and timing bugs before
you get to the field.

Run all the tests:

```
./gradlew :TeamCode:test
```

Run one class while you work on it:

```
./gradlew :TeamCode:testDebugUnitTest --tests '*LiftTest*'
```

Tests live in `TeamCode/src/test/java/...`, in the same package as the class they test.

## Example test

Annotate the class with `@RobotTest` and take a `LoopRunner` parameter. The annotation clears the
command scheduler and subsystem registry before and after each test, so tests cannot affect each
other.

```java
@RobotTest
class LiftTest {

    @Test
    void raisesToTheCommandedPosition(LoopRunner runner) {
        runner.motor("lift");
        Lift lift = new Lift(runner.hardwareMap());

        lift.to(1500).schedule();
        runner.until(lift::atTarget, 500);

        assertEquals(1500, lift.position(), 20);
    }
}
```

This test does three things:

1. `runner.motor("lift")` creates a fake motor and registers it under that name.
2. `runner.hardwareMap()` returns a stand-in `HardwareMap` that hands out the fakes. Your subsystem
   constructor does not change.
3. `runner.until(...)` runs the loop until the condition is true or the loop limit is reached, and
   fails the test if the limit is hit first.

## Running the loop

| Method | Effect |
| --- | --- |
| `runner.loop()` | One loop: sense, scheduler, act, then advance the fake hardware by one tick |
| `runner.loops(n)` | `n` loops |
| `runner.until(condition, maxLoops)` | Loops until the condition is true, fails at the limit |
| `runner.elapsedSeconds()` | Simulated time since the runner was created |
| `runner.loopCount()` | Loops run so far |

The loop step defaults to 20 ms, matching a 50 Hz robot. Pass a different value to the constructor if
a test needs it.

## Fakes

| Fake | Created with | Models |
| --- | --- | --- |
| `FakeMotor` | `runner.motor("name")` | Encoder that integrates commanded power, current draw, direction |
| `FakeServo` | `runner.servo("name")` | Position, clamped to `[0, 1]`, with scaled range |
| `FakeCRServo` | `runner.register("name", new FakeCRServo("name"))` | Commanded power, and a count of writes |
| `FakeDigitalChannel` | `runner.digitalChannel("name")` | A limit switch or beam break, active low |
| `FakeVoltageSensor` | `runner.voltageSensor(12.0)` | Battery voltage, and a count of reads |

`runner.register(name, device)` adds any device you build yourself.

For a motor to move its encoder, the runner has to step it. `runner.motor(...)` handles that. If you
construct a `FakeMotor` directly, pass it to `runner.stepping(...)`.

## Injecting the clock

Anything that measures time takes a clock in its constructor, so a test can control it instead of
sleeping.

```java
AtomicLong clock = new AtomicLong();
Debouncer beam = new Debouncer(0.050, Debouncer.Type.FALLING, clock::get);

beam.calculate(true);
clock.addAndGet(60_000_000L);   // 60 ms
assertTrue(beam.calculate(true));
```

Use `System::nanoTime` in real robot code and a controlled value in tests. `LoopRunner` advances its
own clock as the loop runs.

## Assertions

A test should fail if the behavior it names is broken. This is easy to get wrong. If a test asserts a
value that some other code path also produces, it passes even when the mechanism is broken.

For example, checking that sense runs before act by asserting the stored sensor reading does not
work, because `sense()` stores that reading in either order. The value that distinguishes the two
orderings is the motor output, because that is computed from the reading:

```java
motor.setEncoder(700);
runner.loop();

assertEquals(700, lift.position(), 1);
assertEquals(-1.0, motor.getPower(), 1e-9);   // this is the assertion that can fail
```

When you write a test for a bug, break the fix on purpose and confirm the test fails. If it still
passes, the test is not testing the fix.

## Limitations

The fakes are simple on purpose. They do not model motor inertia, battery sag under load, encoder
noise, or CAN and I2C timing. `FakeMotor` ignores `RunMode`, so a test cannot catch a missing
`RUN_WITHOUT_ENCODER` after an encoder reset.

A passing test means the logic is right. It does not mean the mechanism works. You still have to test
on the field.
