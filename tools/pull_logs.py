#!/usr/bin/env python3
"""Pulls RunLog run directories off a Control Hub with adb.

USB needs no setup. Over wifi the hub is 192.168.43.1:5555, which this connects to only when no
device is already attached.

    python3 tools/pull_logs.py            # pull every run that is not already local
    python3 tools/pull_logs.py --list     # show what is on the hub
    python3 tools/pull_logs.py --last 3   # newest three runs only

Exit codes: 0 ok, 3 no adb, 4 no device / adb hung, 5 no log folder on the hub.
Standard library only.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

import runlog

DEFAULT_HUB = "192.168.43.1:5555"
SHELL_TIMEOUT_S = 20
PULL_TIMEOUT_S = 180


class AdbError(Exception):
    def __init__(self, message: str, code: int):
        super().__init__(message)
        self.code = code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="pull_logs.py", description="Pull run logs off the Control Hub."
    )
    parser.add_argument("--hub", default=os.environ.get("FTC_HUB", DEFAULT_HUB))
    parser.add_argument("--dest", type=Path, default=runlog.repo_root() / "logs")
    parser.add_argument("--last", type=int, help="pull only the newest N runs")
    parser.add_argument("--force", action="store_true", help="re-pull runs that are already local")
    parser.add_argument(
        "--delete-remote", action="store_true", help="delete each run from the hub after a good pull"
    )
    parser.add_argument("--adb", help="path to adb")
    parser.add_argument("--list", action="store_true", help="list remote runs and exit")
    args = parser.parse_args(argv)

    try:
        adb = find_adb(args.adb)
        ensure_device(adb, args.hub)
        remote_runs = list_remote_runs(adb)
    except AdbError as exc:
        print(str(exc), file=sys.stderr)
        return exc.code

    if args.last is not None and args.last > 0:
        remote_runs = remote_runs[-args.last :]

    dest: Path = args.dest
    if args.list:
        for name in remote_runs:
            mark = "local" if is_complete(dest / name) else "absent"
            print(f"{name} [{mark}]")
        return 0

    dest.mkdir(parents=True, exist_ok=True)
    pulled = 0
    for name in remote_runs:
        local = dest / name
        if is_complete(local) and not args.force:
            print(f"{name} skipped (already local)")
            continue
        try:
            pull_run(adb, name, dest)
        except AdbError as exc:
            print(str(exc), file=sys.stderr)
            return exc.code
        if not is_complete(local):
            print(f"{name} incomplete")
            continue
        pulled += 1
        note = "" if (local / "meta.json").exists() else " (no meta.json)"
        print(f"{name} pulled{note}")
        if args.delete_remote:
            run_adb(adb, ["shell", "rm", "-rf", f"{runlog.REMOTE_LOGS}/{name}"], SHELL_TIMEOUT_S)
            print(f"{name} deleted from hub")

    print(f"{pulled} pulled, {len(remote_runs)} on hub")
    local_runs = runlog.find_runs(dest)
    if local_runs:
        print(f"newest local run: {local_runs[-1]}")
        print("plot it with: python3 tools/plot_run.py --latest")
    return 0


def find_adb(explicit: str | None) -> str:
    # An explicit override that does not resolve is an error: silently using a different adb than
    # the one asked for hides the real problem.
    for label, candidate in (("--adb", explicit), ("$ADB", os.environ.get("ADB"))):
        if not candidate:
            continue
        resolved = resolve_executable(candidate)
        if resolved:
            return resolved
        raise AdbError(f"adb not found: {label}={candidate} is not an executable", 3)
    on_path = shutil.which("adb")
    if on_path:
        return on_path
    android_home = os.environ.get("ANDROID_HOME") or os.environ.get("ANDROID_SDK_ROOT")
    if android_home:
        bundled = Path(android_home) / "platform-tools" / "adb"
        if bundled.is_file():
            return str(bundled)
    raise AdbError("adb not found; install Android platform-tools or pass --adb", 3)


def resolve_executable(candidate: str) -> str | None:
    path = Path(candidate)
    if path.is_file():
        return str(path)
    return shutil.which(candidate)


def run_adb(adb: str, args: list[str], timeout: int) -> tuple[int, str]:
    """adb output is \\r\\n terminated; strip the \\r or every parse breaks on Windows."""
    try:
        done = subprocess.run(
            [adb, *args], capture_output=True, text=True, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        raise AdbError(f"adb {' '.join(args)} timed out after {timeout}s", 4)
    return done.returncode, (done.stdout + done.stderr).replace("\r", "")


def attached_devices(adb: str) -> list[str]:
    _, out = run_adb(adb, ["devices"], SHELL_TIMEOUT_S)
    devices = []
    for line in out.splitlines()[1:]:
        parts = line.split("\t")
        if len(parts) == 2 and parts[1].strip() == "device":
            devices.append(parts[0].strip())
    return devices


def ensure_device(adb: str, hub: str) -> None:
    if attached_devices(adb):
        return
    run_adb(adb, ["connect", hub], SHELL_TIMEOUT_S)
    if attached_devices(adb):
        return
    raise AdbError(
        f"no device; plug in USB or join the hub's wifi (tried {hub})",
        4,
    )


def list_remote_runs(adb: str) -> list[str]:
    code, out = run_adb(adb, ["shell", "ls", "-1", runlog.REMOTE_LOGS], SHELL_TIMEOUT_S)
    if "No such file" in out or code != 0:
        raise AdbError(
            f"no logs at {runlog.REMOTE_LOGS}; has an OpMode run since the logger was deployed?",
            5,
        )
    names = [name for name in out.split() if runlog.RUN_RE.match(name)]
    return sorted(set(names))


def pull_run(adb: str, name: str, dest: Path) -> None:
    code, out = run_adb(
        adb, ["pull", f"{runlog.REMOTE_LOGS}/{name}", str(dest)], PULL_TIMEOUT_S
    )
    if code != 0:
        print(out.strip(), file=sys.stderr)


def is_complete(run_dir: Path) -> bool:
    """meta.json is deliberately not required: a run killed before its first logged loop has none,
    and demanding it would re-pull that run on every invocation forever."""
    return run_dir.is_dir() and all((run_dir / name).exists() for name in runlog.REQUIRED_FILES)


if __name__ == "__main__":
    sys.exit(main())
