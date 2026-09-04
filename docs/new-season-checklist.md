# New Season Checklist

Work through this when the new game is released. Most of it is one line per item.

## Field geometry

**1. Set the field symmetry.** Look at the field drawing and decide whether the two alliance halves
are a mirror image or a 180 degree rotation. Set `Field.SYMMETRY` in `field/Field.java`.

```java
public static final FieldSymmetry SYMMETRY = FieldSymmetry.MIRROR_X;
```

This is the default used to derive blue positions from red ones. Getting it wrong makes every blue
autonomous drive to the wrong place, so check it before writing any waypoints.

**2. Check the field width.** `Field.WIDTH_IN` is 144 inches for a standard field. It has not changed
in years, but confirm it.

**3. Write the waypoints.** Author every position in red coordinates and let blue be derived:

```java
static final Waypoint START = Waypoint.red("start", 119.4, 128.8, 225);
static final Waypoint SCORE = Waypoint.red("score", 99.5, 98.9, 240);
```

When a blue position is genuinely not the mirror of red, pin it:

```java
static final Waypoint DEPOT = Waypoint.red("depot", 12, 60, 180).blue(20, 84, 180);
```

Pinning is common. Field elements are often not perfectly symmetric, and measuring both sides is more
reliable than assuming. `Waypoint.blueDriftInches()` tells you how far a pinned value sits from where
the symmetry would have put it, which is a quick way to spot a typo.

## Drivetrain

**4. Tune Pedro Pathing.** Use the tuning OpModes in `pedroPathing/`. Follow the Pedro documentation
and record the values in `pedroPathing/Constants.java`. Do this after the drivetrain is mechanically
finished, because the numbers depend on weight and wheel condition.

**5. Set the starting poses.** Each autonomous OpMode passes a starting waypoint to `Robot`. Measure
where the robot actually sits against the field wall.

## Camera

Skip this section if there is no camera on the robot.

**6. Upload this season's field map** to the Limelight, and set the camera's mounting offsets in its
web interface. See [Limelight](limelight.md).

**7. Check the axis mapping.** Run the Vision Check OpMode, park the robot on a measured spot, and
compare the camera pose against odometry. If x and y are swapped or a sign is flipped, fix
`Field.toPedro`. Do this once and write down the result.

**8. Set the tag IDs.** Tag numbers change every season. Put them somewhere named rather than in the
middle of a method:

```java
private static final int RED_GOAL_TAG = 24;
private static final int BLUE_GOAL_TAG = 20;
```

**9. Set `Vision.MAX_POSE_DISTANCE_IN`** from the `avgTagDist` values you see on this year's field.

## Mechanisms

**10. Write one subsystem per mechanism.** `sense()` reads, `act()` writes. See
[Robot Code Structure](robot-code-structure.md).

**11. Add log signals** for anything you will want to look at later. A target next to its measured
value is the most useful pair. See [Run Logs](run-logs.md).

**12. Write tests** for state machines and anything with a timeout. See
[Writing Tests](writing-tests.md).

## OpModes

**13. Update the button map** in `opmodes/teleop/DriverBindings.java`.

**14. Name the OpModes** so they sort sensibly on the Driver Station. The `group` puts them together
in the list; `match` and `test` are the groups used here.

**15. Delete last season's OpModes.** An old autonomous in the list is something someone will select
by accident at a competition.

## Before the first competition

- [ ] Both alliances of every autonomous run on a real field
- [ ] Teleop drives the correct direction on both alliances
- [ ] `./gradlew :TeamCode:test` passes
- [ ] Run logs pull and plot from a laptop that has never done it before
- [ ] Someone other than the person who wrote it can find and run each OpMode
