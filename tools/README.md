# Tools

Dev machine scripts. Standard library only, so there is nothing to install beyond Python 3 and the
Android platform tools.

| Script | Purpose |
| --- | --- |
| `deploy.py` | Connect to the Control Hub over wifi and install the app |
| `pull_logs.py` | Copy run logs off the hub |
| `plot_run.py` | Render a run as a self-contained HTML page |
| `adb.py` | Shared adb discovery and connection code |
| `runlog.py` | Shared run log loader and summariser |

## Uploading code over wifi

The REV Hardware Client manages the adb connection on Windows. It does not run on macOS, so
`deploy.py` does the same work.

1. Connect your computer to the Control Hub's wifi network. The password is on the Driver Station
   under Program and Manage.
2. Run the script from the repository root:

```
python3 tools/deploy.py
```

It locates adb, connects to the hub, confirms what answered, and runs `:TeamCode:installDebug`.

To connect and then deploy from Android Studio instead:

```
python3 tools/deploy.py --connect-only
```

| Option | Effect |
| --- | --- |
| `--hub HOST[:PORT]` | Hub address. Default `192.168.43.1`, which is its own access point. |
| `--connect-only` | Connect without installing |
| `--from-usb` | Tell a USB-connected device to start listening on wifi, then connect to it |
| `--restart` | Restart the adb server first |
| `--adb PATH` | Use a specific adb |
| `--module` | Gradle module, default `:TeamCode` |

Exit codes: 0 success, 2 bad arguments, 3 adb missing, 4 no device or connection failed, 5 the
Gradle build failed.

## Which adb gets used

`adb.py` prefers the copy inside your Android SDK, in this order:

1. `--adb`
2. `$ADB`
3. `$ANDROID_HOME` or `$ANDROID_SDK_ROOT`
4. `~/Library/Android/sdk/platform-tools/adb` on macOS, `~/Android/Sdk/platform-tools/adb` on Linux,
   `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe` on Windows
5. `adb` on `PATH`

The SDK copy comes before `PATH` because only one adb server can hold port 5037, and a client from a
different release refuses to talk to a server started by another one. A Homebrew adb and Android
Studio's adb are usually different versions, so mixing them produces
`adb server version doesn't match this client`. Android Studio uses the SDK copy, so the scripts do
too.

If you hit that error anyway, `python3 tools/deploy.py --restart` kills and restarts the server.

## Troubleshooting

| Message | Cause |
| --- | --- |
| `cannot reach 192.168.43.1:5555` | Not on the hub's wifi network, or the hub is off. If the hub is joined to another network, pass `--hub` with the address it has there. |
| `adb not found` | Install the Android platform tools, or pass `--adb` |
| `more than one device attached` | A phone or second hub is also connected. Unplug it, or pass `--hub` with the one you want. |
| `never became ready` after several `is offline` lines | The hub accepts the connection before adbd is ready, usually after a reboot or a Robot Controller restart. `deploy.py` retries six times over about ten seconds and restarts the adb server halfway through. If it still fails it prints what to try next. |
| `--from-usb given but no device is connected by USB` | Plug the hub in first, or drop the flag |

## Run logs

```
python3 tools/pull_logs.py          # copy new runs into logs/
python3 tools/plot_run.py --latest  # chart the newest run
```

Both accept `--hub` and `--adb` in the same way as `deploy.py`. `pull_logs.py --list` shows what is
on the hub without copying.
