"""adb discovery and connection, shared by the tools that talk to the robot.

The Control Hub runs adb over TCP on port 5555. In its own access point mode it answers on
192.168.43.1. A hub joined to another network gets an address from that network's DHCP server, and
you have to pass it.
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

DEFAULT_HUB = "192.168.43.1"
ADB_PORT = 5555
SHELL_TIMEOUT_S = 20
CONNECT_TIMEOUT_S = 20
PROBE_TIMEOUT_S = 2.0
CONNECT_ATTEMPTS = 6
CONNECT_BACKOFF_S = 2.0
RESTART_AFTER_ATTEMPT = 3


class AdbError(Exception):
    """Carries the exit code the calling script should return."""

    def __init__(self, message: str, code: int):
        super().__init__(message)
        self.code = code


def sdk_adb() -> str | None:
    """adb from an Android Studio install.

    Preferred over one on PATH. Only one adb server can own port 5037, and a client built from a
    different release refuses to talk to it, so a Homebrew adb and Android Studio's adb fight over
    who started the server. Using the one Android Studio uses avoids that.
    """
    candidates = [
        Path.home() / "Library/Android/sdk/platform-tools/adb",     # macOS
        Path.home() / "Android/Sdk/platform-tools/adb",             # Linux
        Path(os.environ.get("LOCALAPPDATA", "")) / "Android/Sdk/platform-tools/adb.exe",
    ]
    for root in (os.environ.get("ANDROID_HOME"), os.environ.get("ANDROID_SDK_ROOT")):
        if root:
            candidates.insert(0, Path(root) / "platform-tools" / "adb")
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return None


def find_adb(explicit: str | None = None) -> str:
    """An explicit path that does not resolve is an error, so a typo is not silently ignored."""
    for label, candidate in (("--adb", explicit), ("$ADB", os.environ.get("ADB"))):
        if not candidate:
            continue
        resolved = candidate if Path(candidate).is_file() else shutil.which(candidate)
        if resolved:
            return resolved
        raise AdbError(f"adb not found: {label}={candidate} is not an executable", 3)
    from_sdk = sdk_adb()
    if from_sdk:
        return from_sdk
    on_path = shutil.which("adb")
    if on_path:
        return on_path
    raise AdbError("adb not found; install Android platform-tools or pass --adb", 3)


def run_adb(adb: str, args: list[str], timeout: int, serial: str | None = None) -> tuple[int, str]:
    """adb output is \\r\\n terminated; strip the \\r or every parse breaks on Windows."""
    argv = [adb, *(["-s", serial] if serial else []), *args]
    try:
        done = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        raise AdbError(f"adb {' '.join(args)} timed out after {timeout}s", 4)
    return done.returncode, (done.stdout + done.stderr).replace("\r", "")


def device_states(adb: str) -> list[tuple[str, str]]:
    """Every line of `adb devices` as (serial, state). States include device, offline, unauthorized."""
    _, out = run_adb(adb, ["devices"], SHELL_TIMEOUT_S)
    states = []
    for line in out.splitlines()[1:]:
        parts = line.split("\t")
        if len(parts) == 2:
            states.append((parts[0].strip(), parts[1].strip()))
    return states


def attached_devices(adb: str) -> list[str]:
    return [serial for serial, state in device_states(adb) if state == "device"]


def usb_devices(adb: str) -> list[str]:
    """Serials that are not host:port, so they arrived over USB."""
    return [serial for serial in attached_devices(adb) if ":" not in serial]


def with_port(host: str) -> str:
    return host if ":" in host else f"{host}:{ADB_PORT}"


def reachable(target: str, timeout: float = PROBE_TIMEOUT_S) -> bool:
    """TCP probe before calling adb, so an unreachable hub gives a clear message rather than adb's."""
    host, _, port = with_port(target).partition(":")
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except OSError:
        return False


def state_of(adb: str, serial: str) -> str | None:
    return dict(device_states(adb)).get(serial)


def connect(adb: str, target: str, report=lambda message: None) -> str:
    """Connects to host:port and returns the serial once the device reports ready.

    The hub accepts TCP on the adb port before adbd has finished its handshake, so a fresh
    connection reads back as offline for a few seconds after the hub boots or the Robot Controller
    app restarts. Each attempt therefore drops the stale entry and waits before trying again, and
    halfway through it restarts the adb server, which clears a wedged one.
    """
    serial = with_port(target)
    if not reachable(serial):
        raise AdbError(
            f"cannot reach {serial}. Join the Control Hub's wifi network, or pass --hub with the "
            f"address it has on the network you are on.",
            4,
        )

    for attempt in range(1, CONNECT_ATTEMPTS + 1):
        run_adb(adb, ["connect", serial], CONNECT_TIMEOUT_S)
        state = state_of(adb, serial)
        if state == "device":
            if attempt > 1:
                report(f"         ready after {attempt} attempts")
            return serial

        if attempt == CONNECT_ATTEMPTS:
            break
        report(f"         {serial} is {state or 'not listed'}, retrying ({attempt})")
        run_adb(adb, ["disconnect", serial], SHELL_TIMEOUT_S)
        if attempt == RESTART_AFTER_ATTEMPT:
            report("         restarting the adb server")
            run_adb(adb, ["kill-server"], SHELL_TIMEOUT_S)
            run_adb(adb, ["start-server"], CONNECT_TIMEOUT_S)
        time.sleep(CONNECT_BACKOFF_S)

    raise AdbError(
        f"{serial} never became ready.\n\n" + diagnose(adb, serial),
        4,
    )


def diagnose(adb: str, serial: str) -> str:
    """What to check when a connection will not come up. Printed on failure."""
    _, devices = run_adb(adb, ["devices", "-l"], SHELL_TIMEOUT_S)
    _, version = run_adb(adb, ["version"], SHELL_TIMEOUT_S)

    others = []
    seen = {Path(adb).resolve()}
    for candidate in filter(None, [shutil.which("adb"), sdk_adb()]):
        path = Path(candidate).resolve()
        if path in seen:
            continue
        seen.add(path)
        code, out = run_adb(str(path), ["version"], SHELL_TIMEOUT_S)
        line = next((l for l in out.splitlines() if l.startswith("Version")), out.strip())
        others.append(f"  {path}  {line}")

    lines = [
        "adb in use:",
        f"  {adb}",
        "  " + " ".join(version.split("\n")[1:2]),
        "",
        "devices:",
        *["  " + l for l in devices.strip().splitlines()],
    ]
    if others:
        lines += [
            "",
            "another adb is installed, and only one version can own the server:",
            *others,
        ]
    lines += [
        "",
        "things to try, in order:",
        "  1. Restart the Robot Controller app on the hub, wait for the Driver Station to",
        "     reconnect, then run this again.",
        "  2. python3 tools/deploy.py --restart",
        "  3. Power cycle the Control Hub.",
        "  4. Connect by USB and run: python3 tools/deploy.py --from-usb",
    ]
    return "\n".join(lines)


def ensure_device(adb: str, hub: str) -> str | None:
    """A serial to address, or None when a single device makes -s unnecessary."""
    devices = attached_devices(adb)
    if not devices:
        return connect(adb, hub)
    if len(devices) == 1:
        return None
    target = with_port(hub)
    if target in devices:
        return target
    raise AdbError(
        "more than one device attached: "
        + ", ".join(devices)
        + "; unplug the others or pass --hub with the one you want",
        4,
    )


def describe(adb: str, serial: str | None) -> str:
    """Model and manufacturer, to confirm what answered."""
    fields = []
    for prop in ("ro.product.manufacturer", "ro.product.model"):
        code, out = run_adb(adb, ["shell", "getprop", prop], SHELL_TIMEOUT_S, serial)
        if code == 0 and out.strip():
            fields.append(out.strip())
    return " ".join(fields) if fields else "unknown device"
