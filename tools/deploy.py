#!/usr/bin/env python3
"""Connects to the Control Hub over wifi and installs the app.

The REV Hardware Client does this on Windows and does not run on macOS. This covers the same ground
with adb and Gradle.

    python3 tools/deploy.py                  # connect over wifi, then install
    python3 tools/deploy.py --connect-only   # connect, then deploy from Android Studio
    python3 tools/deploy.py --from-usb       # switch a USB-connected hub to wifi, then install

Exit codes: 0 success, 2 bad arguments, 3 adb missing, 4 no device or connection failed,
5 the Gradle build failed.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import adb


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Connect to the Control Hub over wifi and install the app."
    )
    parser.add_argument(
        "--hub",
        default=os.environ.get("FTC_HUB", adb.DEFAULT_HUB),
        help=f"hub address, default {adb.DEFAULT_HUB} (its own access point)",
    )
    parser.add_argument("--adb", help="path to adb; defaults to the Android Studio one")
    parser.add_argument(
        "--connect-only", action="store_true", help="connect but do not install"
    )
    parser.add_argument(
        "--from-usb",
        action="store_true",
        help="tell a USB-connected device to listen on wifi first",
    )
    parser.add_argument(
        "--restart", action="store_true", help="restart the adb server before connecting"
    )
    parser.add_argument(
        "--module", default=":TeamCode", help="Gradle module to install, default :TeamCode"
    )
    args = parser.parse_args(argv)

    try:
        adb_path = adb.find_adb(args.adb)
        print(f"adb      {adb_path}")

        if args.restart:
            adb.run_adb(adb_path, ["kill-server"], adb.SHELL_TIMEOUT_S)
            adb.run_adb(adb_path, ["start-server"], adb.SHELL_TIMEOUT_S)
            print("adb      server restarted")

        if args.from_usb:
            enable_wifi_debugging(adb_path)

        serial = adb.connect(adb_path, args.hub, report=print)
        print(f"device   {serial}  {adb.describe(adb_path, serial)}")
    except adb.AdbError as exc:
        print(str(exc), file=sys.stderr)
        return exc.code

    if args.connect_only:
        print("Connected. Deploy from Android Studio, or run this again without --connect-only.")
        return 0

    return install(serial, args.module)


def enable_wifi_debugging(adb_path: str) -> None:
    """Puts a USB-connected device into TCP mode. The Control Hub already listens on 5555."""
    usb = adb.usb_devices(adb_path)
    if not usb:
        raise adb.AdbError("--from-usb given but no device is connected by USB", 4)
    if len(usb) > 1:
        raise adb.AdbError("more than one USB device: " + ", ".join(usb), 4)
    code, out = adb.run_adb(
        adb_path, ["tcpip", str(adb.ADB_PORT)], adb.CONNECT_TIMEOUT_S, usb[0]
    )
    if code != 0:
        raise adb.AdbError(f"adb tcpip failed: {out.strip()}", 4)
    print(f"usb      {usb[0]} now listening on port {adb.ADB_PORT}")


def install(serial: str | None, module: str) -> int:
    """Runs the Gradle install task against one device.

    ANDROID_SERIAL is how Gradle is told which device to use; without it a build with both a USB
    and a wifi connection to the same hub fails as ambiguous.
    """
    root = repo_root()
    gradlew = root / ("gradlew.bat" if os.name == "nt" else "gradlew")
    task = f"{module}:installDebug"

    env = dict(os.environ)
    if serial:
        env["ANDROID_SERIAL"] = serial

    print(f"install  {task}")
    done = subprocess.run([str(gradlew), task], cwd=root, env=env)
    if done.returncode != 0:
        print("the Gradle install task failed", file=sys.stderr)
        return 5
    print("Installed. Select the OpMode on the Driver Station.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
