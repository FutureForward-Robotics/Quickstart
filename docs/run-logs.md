# Run Logs

Every OpMode records a log while it runs. After a match or a practice run you copy the logs to your
computer and look at the data. This is how you answer questions like "did the flywheel actually reach
speed before we shot" without guessing.

## Recording

Logging is automatic. `ForwardOpMode` opens a log at init, writes one row per loop, and closes it when
the OpMode stops. You do not have to call anything.

To record a value, add one line to your subsystem's `logSignals` method:

```java
@Override
public void logSignals(RunLog log) {
    log.addSignal("lift.position", () -> position);
    log.addSignal("lift.target", () -> target);
    log.addFlag("lift.atTarget", this::atTarget);
}
```

| Method | Column contents |
| --- | --- |
| `addSignal(name, supplier)` | A number, four decimal places |
| `addFlag(name, supplier)` | `1` or `0` |
| `addSlowSignal(name, supplier, intervalMs)` | A number, sampled at most that often and repeated in between |

Use `addSlowSignal` for anything expensive to read. Battery voltage is the usual case, because reading
it is a round trip to the hub.

Name signals `subsystem.value`. The plotting tool groups columns by the part before the dot and draws
one chart per group.

Register signals in `logSignals`, not in your constructor. Signals must all be registered before the
first row is written, and `logSignals` is called at the right moment.

> **Warning**
> The suppliers run once per loop on the control loop thread. Read a field you already stored during
> `sense()`. Do not read hardware in a supplier.

### Events

Signals are for values that exist every loop. Events are for things that happen at a moment:

```java
log.event("note", "driver took over");
```

Command starts, interrupts, and finishes are recorded automatically, so you can see which command was
running when something went wrong.

## File format

Each run creates a directory on the Robot Controller under `/sdcard/FIRST/logs/`, named with the date,
time, and OpMode:

```
20260901-115434-TeleOp_Red/
    meta.json        column names, OpMode name, start time
    signals.csv      one header row, then one row per loop
    events.jsonl     one JSON object per line
```

The robot keeps the newest 25 runs and deletes older ones. A run stops recording if it reaches 32 MB.

## Retrieving logs

Connect to the Robot Controller by USB, or join the robot's wifi, then:

```
python3 tools/pull_logs.py
```

Runs land in `logs/` and runs you already have are skipped. Useful options:

| Option | Effect |
| --- | --- |
| `--list` | Show what is on the robot without copying anything |
| `--last N` | Copy only the newest N runs |
| `--force` | Copy again even if the run is already local |
| `--delete-remote` | Delete each run from the robot after a verified copy |
| `--hub HOST:PORT` | Address to connect to, default `192.168.43.1:5555` |

## Plotting

```
python3 tools/plot_run.py --latest
```

This prints a summary and writes `plot.html` inside the run directory, then opens it. The page has the
data and the charting library embedded, so it works with no internet connection. You can open it at a
competition, or email it to someone.

| Option | Effect |
| --- | --- |
| `--latest` | Use the newest run in `logs/` |
| `--list-groups` | Print the chart groups and their column counts, then exit |
| `--only drive,lift` | Chart only these groups |
| `--exclude battery` | Chart everything except these groups |
| `--max-points N` | Thin the data to about N rows, default 20000 |
| `--no-open` | Write the file without opening a browser |

## Summary output

```
run 20260901-115434-TeleOp_Red
opMode TeleOp Red
started 2026-09-01T11:54:34 (hub clock, may be wrong)
rows 4760  duration 95.210 s
loop period mean 20.0 ms / median 20.0 / p95 21.0 / max 34.0  (mean 50.0 Hz)
loop numbers contiguous
dropped 0  bytes 812345
```

What to look for:

- **`max` loop period** much larger than the median means something blocked the loop. Compare the time
  it happened against the event list.
- **Loop number gaps** mean rows were dropped because the writer could not keep up.
- **`dropped`** greater than zero means the same thing, counted directly.
- **`no run/end event`** means the OpMode was stopped hard or the log was cut off. The data up to that
  point is still valid.
- **Empty fields** in a column mean the supplier threw an exception or returned a value that could not
  be written, such as NaN.

The start time comes from the Robot Controller's clock, which is wrong when it has been powered up
without a Driver Station connected. Times inside the run are always correct relative to each other,
because they are measured from the start of the log.

## Troubleshooting

| Symptom | Cause |
| --- | --- |
| `no device; plug in USB or join the hub's wifi` | Nothing is connected, or adb is not running |
| `more than one device attached` | A phone or second hub is also connected. Unplug it, or pass `--hub` with the one you want. |
| `no logs at /sdcard/FIRST/logs` | No OpMode has run since this code was installed |
| `adb not found` | Install Android platform-tools, or pass `--adb /path/to/adb` |
| A Driver Station warning about the log | The storage is full or unwritable. Logging stops; the OpMode keeps running. |
| No `meta.json` in a run | The OpMode stopped before its first loop. The other two files are still readable. |
