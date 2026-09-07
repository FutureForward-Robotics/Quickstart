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
import sys
from pathlib import Path

import adb
import runlog
from adb import AdbError

DEFAULT_HUB = adb.DEFAULT_HUB
SHELL_TIMEOUT_S = adb.SHELL_TIMEOUT_S
PULL_TIMEOUT_S = 180


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
        adb_path = adb.find_adb(args.adb)
        serial = adb.ensure_device(adb_path, args.hub)
        remote_runs = list_remote_runs(adb_path, serial)
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
            pull_run(adb_path, name, dest, serial)
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
            adb.run_adb(
                adb_path,
                ["shell", "rm", "-rf", f"{runlog.REMOTE_LOGS}/{name}"],
                SHELL_TIMEOUT_S,
                serial,
            )
            print(f"{name} deleted from hub")

    print(f"{pulled} pulled, {len(remote_runs)} on hub")
    local_runs = runlog.find_runs(dest)
    if local_runs:
        print(f"newest local run: {local_runs[-1]}")
        print("plot it with: python3 tools/plot_run.py --latest")
    return 0


def list_remote_runs(adb_path: str, serial: str | None) -> list[str]:
    code, out = adb.run_adb(
        adb_path, ["shell", "ls", "-1", runlog.REMOTE_LOGS], SHELL_TIMEOUT_S, serial
    )
    if "No such file" in out:
        raise AdbError(
            f"no logs at {runlog.REMOTE_LOGS}; has an OpMode run since the logger was deployed?",
            5,
        )
    if code != 0:
        raise AdbError(f"adb shell ls {runlog.REMOTE_LOGS} failed: {out.strip()}", 4)
    names = [name for name in out.split() if runlog.RUN_RE.match(name)]
    return sorted(set(names))


def pull_run(adb_path: str, name: str, dest: Path, serial: str | None) -> None:
    code, out = adb.run_adb(
        adb_path, ["pull", f"{runlog.REMOTE_LOGS}/{name}", str(dest)], PULL_TIMEOUT_S, serial
    )
    if code != 0:
        # Must raise: main's completeness check reads the destination, which under --force can
        # still hold an older complete copy, and --delete-remote would then drop the fresh one.
        raise AdbError(f"adb pull {name} failed: {out.strip()}", 4)



def is_complete(run_dir: Path) -> bool:
    """meta.json is deliberately not required: a run killed before its first logged loop has none,
    and demanding it would re-pull that run on every invocation forever."""
    return run_dir.is_dir() and all((run_dir / name).exists() for name in runlog.REQUIRED_FILES)


if __name__ == "__main__":
    sys.exit(main())
