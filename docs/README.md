# Documentation

Guides for the code in this repository. If you are new to the codebase, read them in order.

| Guide | Contents |
| --- | --- |
| [Robot Code Structure](robot-code-structure.md) | How OpModes, subsystems, and commands fit together |
| [Writing Tests](writing-tests.md) | Running the robot loop on your computer, without a robot |
| [Run Logs](run-logs.md) | Recording data during a match and reviewing it afterward |
| [Limelight](limelight.md) | Setting up the camera, aiming with it, and correcting odometry |
| [New Season Checklist](new-season-checklist.md) | What to change when the game changes |

## Commands

Run the off-robot tests:

```
./gradlew :TeamCode:test
```

Build the app:

```
./gradlew :TeamCode:assembleDebug
```

Run one test class while you work on it:

```
./gradlew :TeamCode:testDebugUnitTest --tests '*DriveTest*'
```

Pull run logs off the robot and plot the newest one:

```
python3 tools/pull_logs.py
python3 tools/plot_run.py --latest
```

## Repository layout

| Directory | Contents |
| --- | --- |
| `subsystems/` | One class per mechanism. Each reads its sensors and writes its motors. |
| `opmodes/teleop/` | Driver-controlled OpModes and the button map |
| `opmodes/auto/` | Autonomous OpModes |
| `opmodes/test/` | Diagnostic OpModes used to check or tune one thing |
| `field/` | Field geometry: waypoints, alliance mirroring, path building |
| `util/` | Reusable helpers with no hardware of their own |
| `pedroPathing/` | Pedro Pathing constants and tuning OpModes |
