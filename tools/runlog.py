"""Loader and summariser for RunLog run directories.

A run directory is <yyyyMMdd-HHmmss>-<OpModeName>/ holding signals.csv and events.jsonl (always
created when the log opens) plus meta.json (written at the first logged loop, so it is absent from a
run that was killed before then). Every subsystem's signals land in the one signals.csv as extra
columns, so column count is unbounded and file count is not.

Standard library only: these tools must run on a laptop with nothing installed.
"""

from __future__ import annotations

import csv
import json
import math
import re
import statistics
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

REMOTE_LOGS = "/sdcard/FIRST/logs"
REQUIRED_FILES = ("signals.csv", "events.jsonl")  # always created by RunLog.open
OPTIONAL_FILES = ("meta.json",)  # absent when a run was killed before its first loop
RUN_RE = re.compile(r"^\d{8}-\d{6}-.+$")

TIME_COLUMN = "t_s"
LOOP_COLUMN = "loop"


class RunLogError(Exception):
    """A run could not be located. Callers print the message and exit 2."""


@dataclass
class Run:
    dir: Path
    name: str
    op_mode: str
    started_wall_clock_ms: int
    columns: list[str]
    t: list[float]
    loops: list[int | None]
    series: dict[str, list[float | None]]
    events: list[dict]
    warnings: list[str] = field(default_factory=list)

    @property
    def duration_s(self) -> float:
        return self.t[-1] if self.t else 0.0


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def find_runs(logs_dir: Path) -> list[Path]:
    if not logs_dir.is_dir():
        return []
    runs = [p for p in logs_dir.iterdir() if p.is_dir() and RUN_RE.match(p.name)]
    return sorted(runs, key=lambda p: p.name)


def resolve_run(arg: str | None, logs_dir: Path, latest: bool) -> Path:
    if latest:
        runs = find_runs(logs_dir)
        if not runs:
            raise RunLogError(f"no runs in {logs_dir}; run tools/pull_logs.py first")
        return runs[-1]
    if not arg:
        raise RunLogError("give a run directory or --latest")
    direct = Path(arg)
    if direct.is_dir():
        return direct
    inside = logs_dir / arg
    if inside.is_dir():
        return inside
    raise RunLogError(f"no such run: {arg} (looked in {logs_dir})")


def load_run(run_dir: Path) -> Run:
    if not run_dir.is_dir():
        raise RunLogError(f"not a run directory: {run_dir}")
    warnings: list[str] = []
    meta = _load_meta(run_dir / "meta.json", warnings)
    header, t, loops, series, csv_warnings = _load_signals(run_dir / "signals.csv")
    warnings.extend(csv_warnings)

    columns = header or list(meta.get("columns") or [])
    meta_columns = list(meta.get("columns") or [])
    if header and meta_columns and header != meta_columns:
        warnings.append("meta.json columns disagree with the CSV header; using the header")

    events, event_warnings = _load_events(run_dir / "events.jsonl")
    warnings.extend(event_warnings)

    return Run(
        dir=run_dir,
        name=run_dir.name,
        op_mode=str(meta.get("opMode") or _op_mode_from_events(events) or "?"),
        started_wall_clock_ms=int(meta.get("startedWallClockMs") or 0),
        columns=columns,
        t=t,
        loops=loops,
        series=series,
        events=events,
        warnings=warnings,
    )


def _load_meta(path: Path, warnings: list[str]) -> dict:
    if not path.exists():
        warnings.append("meta.json missing; columns taken from the CSV header")
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except (OSError, ValueError) as exc:
        warnings.append(f"meta.json unreadable ({exc}); columns taken from the CSV header")
        return {}
    if not isinstance(data, dict):
        warnings.append("meta.json is not an object; columns taken from the CSV header")
        return {}
    return data


def _load_signals(
    path: Path,
) -> tuple[list[str], list[float], list[int | None], dict[str, list[float | None]], list[str]]:
    warnings: list[str] = []
    if not path.exists():
        warnings.append("signals.csv missing; plotting events only")
        return [], [], [], {}, warnings

    with path.open(newline="", encoding="utf-8", errors="replace") as handle:
        rows = list(csv.reader(handle))
    if not rows:
        warnings.append("signals.csv is empty; plotting events only")
        return [], [], [], {}, warnings

    header = [name.strip() for name in rows[0]]
    # By index, not by name: a header carrying the same name twice would otherwise collapse into
    # one dict key and append two values per row, mis-shaping the series against t.
    signal_index: dict[str, int] = {}
    for position, name in enumerate(header):
        if name in (TIME_COLUMN, LOOP_COLUMN):
            continue
        if name in signal_index:
            warnings.append(f"duplicate column {name}; keeping the leftmost")
            continue
        signal_index[name] = position
    signal_names = list(signal_index)
    series: dict[str, list[float | None]] = {name: [] for name in signal_names}
    t: list[float] = []
    loops: list[int | None] = []
    bad_columns: set[str] = set()

    data_row = 0
    for raw in rows[1:]:
        if not raw or (len(raw) == 1 and not raw[0].strip()):
            continue
        data_row += 1
        if len(raw) != len(header):
            warnings.append(
                f"dropped ragged row {data_row} (got {len(raw)} fields, want {len(header)})"
            )
            continue
        record = dict(zip(header, raw))
        time_value = _to_float(record.get(TIME_COLUMN, ""))
        if time_value is None:
            warnings.append(f"dropped row {data_row}: unparseable {TIME_COLUMN}")
            continue
        t.append(time_value)
        loops.append(_to_int(record.get(LOOP_COLUMN, "")))
        for name in signal_names:
            text = raw[signal_index[name]]
            value = _to_float(text)
            if value is None and text.strip() != "" and name not in bad_columns:
                bad_columns.add(name)
                warnings.append(f"column {name} has unparseable values, treated as empty")
            series[name].append(value)
    return header, t, loops, series, warnings


def _load_events(path: Path) -> tuple[list[dict], list[str]]:
    warnings: list[str] = []
    events: list[dict] = []
    if not path.exists():
        warnings.append("events.jsonl missing")
        return events, warnings
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                parsed = json.loads(line)
            except ValueError:
                warnings.append(f"skipped unparseable event line {line_number}")
                continue
            if isinstance(parsed, dict):
                events.append(parsed)
            else:
                warnings.append(f"skipped non-object event line {line_number}")
    return events, warnings


def _to_float(text: str) -> float | None:
    text = text.strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    # NaN and infinity have no JSON form; the writer emits those as empty fields anyway.
    return value if math.isfinite(value) else None


def _to_int(text: str) -> int | None:
    value = _to_float(text)
    return None if value is None else int(value)


def _op_mode_from_events(events: list[dict]) -> str:
    for event in events:
        if event.get("type") == "run" and event.get("event") == "start":
            name = event.get("opMode")
            if name:
                return str(name)
    return ""


def event_kind(event: dict) -> str:
    kind = str(event.get("type", "?"))
    name = event.get("event")
    return f"{kind}/{name}" if name else kind


def event_label(event: dict) -> str:
    for key in ("name", "detail"):
        if event.get(key):
            return str(event[key])
    return event_kind(event)


def group_of(column: str) -> str:
    head, dot, _ = column.partition(".")
    return head if dot else "misc"


def group_columns(run: Run) -> dict[str, list[str]]:
    """Signal columns bucketed by name prefix, in first-appearance order."""
    groups: dict[str, list[str]] = {}
    for column in run.series:
        groups.setdefault(group_of(column), []).append(column)
    return groups


def format_summary(run: Run) -> str:
    lines = [f"run {run.name}", f"opMode {run.op_mode}", f"started {_started(run)}"]
    lines.append(f"rows {len(run.t)}  duration {run.duration_s:.3f} s")
    lines.append(_loop_rate(run.t))
    lines.append(_loop_gaps(run.loops))
    lines.extend(_run_end(run.events))
    lines.extend(_empty_fields(run))
    lines.extend(_commands(run.events))
    lines.extend(_notes(run.events))
    lines.extend(f"warning: {w}" for w in run.warnings)
    return "\n".join(lines)


def _started(run: Run) -> str:
    if not run.started_wall_clock_ms:
        return "unknown (no meta.json)"
    stamp = datetime.fromtimestamp(run.started_wall_clock_ms / 1000.0)
    return f"{stamp.isoformat(timespec='seconds')} (hub clock, may be wrong)"


def _loop_rate(t: list[float]) -> str:
    if len(t) < 2:
        return "loop rate: n/a"
    deltas = [(t[i] - t[i - 1]) * 1000.0 for i in range(1, len(t))]
    ordered = sorted(deltas)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
    mean = statistics.fmean(deltas) if hasattr(statistics, "fmean") else sum(deltas) / len(deltas)
    hz = 1000.0 / mean if mean > 0 else 0.0
    return (
        f"loop period mean {mean:.1f} ms / median {statistics.median(deltas):.1f}"
        f" / p95 {p95:.1f} / max {max(deltas):.1f}  (mean {hz:.1f} Hz)"
    )


def _loop_gaps(loops: list[int | None]) -> str:
    gaps = []
    for i in range(1, len(loops)):
        before, after = loops[i - 1], loops[i]
        if before is None or after is None or after - before == 1:
            continue
        gaps.append(f"loop {before} -> {after}")
    if not gaps:
        return "loop numbers contiguous"
    shown = ", ".join(gaps[:3])
    more = f" (+{len(gaps) - 3} more)" if len(gaps) > 3 else ""
    return f"loop gaps {len(gaps)}: {shown}{more}"


def _run_end(events: list[dict]) -> list[str]:
    lines = []
    if any(e.get("type") == "run" and e.get("event") == "truncated" for e in events):
        lines.append("TRUNCATED at 32 MB cap")
    end = next(
        (e for e in reversed(events) if e.get("type") == "run" and e.get("event") == "end"), None
    )
    if end is None:
        lines.append("no run/end event: the OpMode was stopped hard or the log was truncated")
    else:
        lines.append(f"dropped {end.get('dropped', '?')}  bytes {end.get('bytes', '?')}")
    return lines


def _empty_fields(run: Run) -> list[str]:
    counts = [
        (name, sum(1 for v in values if v is None))
        for name, values in run.series.items()
    ]
    reported = [f"{name} {count}" for name, count in counts if count]
    if not reported:
        return []
    joined = ", ".join(reported)
    return [f"empty fields: {joined} (supplier threw or value was NaN)"]


def _commands(events: list[dict]) -> list[str]:
    lines = []
    for index, event in enumerate(events):
        if event.get("type") != "command" or event.get("event") != "start":
            continue
        name = str(event.get("name", "?"))
        start = float(event.get("t", 0.0))
        end_event = _next_command_end(events, index + 1, event.get("id"))
        if end_event is None:
            lines.append(f"{start:.3f} -> (still running at end) {name}")
            continue
        end = float(end_event.get("t", start))
        lines.append(
            f"{start:.3f} -> {end:.3f} ({end - start:6.3f} s) {name} {end_event.get('event')}"
        )
    return lines


def _next_command_end(events: list[dict], start_index: int, command_id) -> dict | None:
    for event in events[start_index:]:
        if event.get("type") != "command" or event.get("id") != command_id:
            continue
        if event.get("event") in ("interrupt", "finish"):
            return event
    return None


def _notes(events: list[dict]) -> list[str]:
    lines = []
    for event in events:
        if event.get("type") in ("run", "command"):
            continue
        detail = event.get("detail")
        body = str(detail) if detail else json.dumps(event, sort_keys=True)
        lines.append(f"{float(event.get('t', 0.0)):.3f} {event.get('type', '?')} {body}")
    return lines
