# TrackShot

TrackShot runs **visual-inertial odometry (VIO) topside**, on the ROV pilot's computer (macOS,
Linux or Windows), rather than on
the ROV's Raspberry Pi. It connects straight to the ROV's OAK-D over the tether, runs depthai's VIO
(RTAB-Map or Basalt) on the surface computer, and sends `VISION_POSITION_DELTA` to ArduSub, so
POSHOLD and other position modes work without GPS. It also records survey images (centre camera
and stereo pairs) with position and attitude in their EXIF/XMP.

## Why topside

The BlueOS OAK-D extension runs the same VIO on the Pi, and the Pi's CPU is the limit: Basalt had
to drop to about 3 fps, and VIO competed with everything else BlueOS does. A Mac runs either backend
at 10 fps or more with headroom to spare. The trade is tether bandwidth, because the VIO frames
now travel up the tether. TrackShot estimates that cost live and warns before settings would
exceed the budget; it doesn't stop you applying them (see [Bandwidth](#bandwidth)).

```
OAK-D (PoE, on the ROV) ──tether──► Mac: TrackShot ── VIO ── odometry ──► VISION_POSITION_DELTA
                                                                         │
                         ArduSub ◄── mavlink2rest (BlueOS, WebSocket) ◄──┘   or MAVLink UDP
```

The code layout and module contracts are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Download

Builds for **macOS (Apple silicon)**, **Linux (x86_64)** and **Windows (x64)** are on the
[Releases page](https://github.com/vshie/TrackShot/releases). Every push to `main` builds all three
in GitHub Actions (`.github/workflows/build.yml`); a `v*` tag publishes them as a release.

- **macOS:** unzip and move `TrackShot.app` to Applications. It is ad-hoc signed, so the first
  launch needs right-click → Open (or `xattr -dr com.apple.quarantine TrackShot.app`). Allow
  **Local Network** access when asked.
- **Linux:** `tar xzf TrackShot-*-linux-x86_64.tar.gz` and run `./TrackShot/TrackShot`. Needs
  glibc 2.35 or newer (Ubuntu 22.04+) and the usual X11/Wayland desktop libraries.
- **Windows:** unzip and run `TrackShot\TrackShot.exe`. Allow it through Windows Firewall on
  private networks. depthai's Windows build has RTAB-Map only (no Basalt).

The repo also carries [`ardupilot/gps_depth_gate.lua`](ardupilot/gps_depth_gate.lua), an ArduSub
script that uses GPS only at the surface and ignores it underwater (see the header of the file
for the parameters it needs).

## Requirements

- **macOS on Apple silicon (arm64), Linux x86_64 or Windows x64.** Running from source works
  wherever your Python, PySide6 and depthai wheels do. On macOS, the built app needs at least the macOS its
  bundled binaries were built for: the build sets `LSMinimumSystemVersion` from them and prints
  it. Built from Homebrew's Python, that is the build Mac's own macOS version (Homebrew compiles
  for the running system), and PySide6 6.12's wheels need macOS 14 or later. To build an app for
  older Macs, make the `.venv` from a python.org installer (built for macOS 11) and pin a PySide6
  whose wheels target the oldest macOS you need.
- Python 3.11 or later (developed on 3.14).
- An OAK-D with an IMU (PoE models over the tether), and a BlueOS vehicle running ArduSub.

## Install from source

```sh
python3 -m venv .venv
.venv/bin/pip install -e '.[dev]'
```

This installs depthai 3.12.0 (pinned), PySide6, pyqtgraph, pymavlink, and the test and packaging tools.

## Run

```sh
.venv/bin/python -m trackshot                 # the app, with your saved settings
.venv/bin/python -m trackshot --sim           # simulator instead of a camera, started at once
.venv/bin/python -m trackshot --vehicle-ip 127.0.0.1:8088    # this session only
.venv/bin/python -m trackshot --reset-settings
```

The `--start` flag starts VIO as soon as the window opens. Settings are saved to
`~/Library/Application Support/TrackShot/settings.json` when you apply them in the window. The log
goes to `~/Library/Logs/TrackShot/trackshot.log` (and the Log panel).

**The simulator never sends to a real vehicle.** With the source set to *Simulator*, TrackShot
sends MAVLink only to a loopback address such as the fake autopilot below. Otherwise it logs a
warning and keeps the link closed, so simulated motion can't reach a real EKF.

### When the VIO loses tracking

TrackShot sends `VISION_POSITION_DELTA` only while the VIO is actually tracking. ArduSub fuses
every delta whatever its confidence, so a wrong "not moving" is worse than nothing.

- **Start-up and after a loss:** the Tracking card shows **Acquiring**. Nothing is sent until the
  VIO has produced 10 distinct poses over at least 0.5 s. Starting VIO before the camera can see
  the seabed is therefore safe; it begins sending once it tracks.
- **Frozen VIO:** RTAB-Map, once lost, keeps reporting its last pose unchanged. Real VIO output
  always carries some noise, so a pose repeated exactly is never sent. If it repeats for 1 s, the
  card shows **Lost** with how long, nothing is sent, and Cockpit's `TS_TRACK` drops to 0.
- **Recovery:**
  - RTAB-Map resets itself to its latest pose after 3 s of lost frames, and retries every 3 s
    while still lost (`Odom/ResetCountdown`). That happens on RTAB-Map's own thread.
  - TrackShot never resets or restarts anything automatically.
  - Once poses move again, TrackShot re-acquires and resumes sending. Nothing is sent across the
    gap.
- **Restart VIO** (toolbar, ⌘⇧R): reconnects the camera and starts the VIO afresh (about 15 s),
  for example once the camera sees the seabed.
  - It's available only while VIO is running, and not again within 20 s of the last restart.
  - The tooltip says why it is unavailable.
- **The log:** "lost" and "regained" messages come in pairs, with at most one "lost" message
  every 10 s. Losses in between are counted into the next message.

### Seabed following without a DVL

With RTAB-Map and the camera looking straight down (camera angle 90°), TrackShot measures the
altitude above the seabed from the stereo depth it already receives: the median depth in the
middle of the image, up to 10 m. With *Send altitude as a rangefinder* on, it sends that as a
downward DISTANCE_SENSOR (orientation 25) at 10 Hz. That is all ArduSub needs to follow the
seabed: no script.

1. **TrackShot:** RTAB-Map, Settings → VIO → Camera angle 90°, Settings → MAVLink → *Send
   altitude as a rangefinder*.
2. **Autopilot:** Autopilot panel → Check, then Apply. With the altitude option on, the check
   includes:

   | Parameter | Value | Why |
   |---|---|---|
   | `RNGFND1_TYPE` | 10 (MAVLink; needs a reboot) | |
   | `RNGFND1_ORIENT` | 25 (down) | |
   | `RNGFND1_MIN` / `RNGFND1_MAX` | 0.3 / 10 m | |
   | `WP_RFND_USE` | 1 | waypoints use the rangefinder |
   | `FS_TERRAIN_ENAB` | 1 | Hold Position instead of the default Disarm when the terrain reference is lost in AUTO |

3. **Mission:** plan the survey in Cockpit with Altitude = **Relative to terrain**. The waypoint
   altitude is the height above the seabed.
4. **Fly AUTO.** ArduSub holds each waypoint's height above the seabed from TrackShot's altitude.
   SURFTRAK uses the same rangefinder when flying by hand.

Caveats:
- **Signal quality:** about half the middle of the image must have depth for quality 100. With
  less, the quality falls below a typical `RNGFND_SQ_MIN` of 90, and ArduSub stops trusting the
  reading (with `FS_TERRAIN_ENAB` 1, AUTO then holds position).
- **The DVL:** leave the altitude option off while a DVL feeds the same rangefinder, and turn
  the DVL's DISTANCE_SENSOR off when TrackShot replaces it.
- **Not yet flown in the water.** Compare TrackShot's altitude with the DVL's on a first dive.

### Cockpit widget

TrackShot serves a small page that Cockpit can show in its **IFrame** widget, so the pilot can
watch and control TrackShot without leaving Cockpit. It shows:
- **the VIO camera image** with the tracked features as green dots, and the tracking lock (good,
  acquiring or lost, and whether TrackShot is sending);
- **pressure depth, TrackShot's altitude and the rangefinder** as the autopilot sees it;
- **Start/Stop tracking and Record/Stop recording** buttons, the same as the toolbar's.

**Adding it to Cockpit:**

1. Start TrackShot. The **Cockpit widget** panel (or Help → *Add TrackShot to Cockpit…*) shows
   the URL with a Copy button:
   - `http://localhost:5425/widget` when Cockpit runs on the same computer;
   - `http://<this computer's address>:5425/widget` (the 192.168.2.x one is listed first) when
     Cockpit runs on another computer on the network.
2. In Cockpit, open the main menu and choose **Edit Interface**.
3. In the widget list (*Regular*), drag the **IFrame** card onto the view.
4. Open the new widget's settings, paste the URL into **Iframe Source** and press **✓** (Set).
   Leave *Use vehicle address as base URL* (Advanced options) off.
5. Size and place it, then leave edit mode.

**Notes:**
- **TrackShot must be running** for the widget to show anything; it says *TrackShot not
  reachable* otherwise.
- **Settings → Cockpit widget:** turn the widget on or off, change the port (5425), and choose
  whether it's reachable from the network.
- **Anyone on the network can start and stop tracking and recording** while the widget is
  reachable from the network (the default). There is no login; untick *Reachable from the
  network* if Cockpit runs on the same computer.
- **Cockpit only lists widgets advertised by the vehicle's BlueOS,** so this one is added by URL.
  A small BlueOS "link" extension could make it appear in the list automatically; that's for
  later.

### Logs

- `~/Library/Logs/TrackShot/trackshot.log`: everything in the Log panel (rotating, 5 MB × 3).
- `~/Library/Logs/TrackShot/runs/run_<date>_<time>.csv`: one row per VIO pose for each run (Start →
  Stop), the last 50 runs. Each row holds the camera's raw pose, the vehicle-frame pose, the
  tracking state, the delta and/or north/east estimate sent to the autopilot, and the
  autopilot's heading, roll, pitch and depth at the time.

`TS_TRACK` values for Cockpit:

| Value | Meaning |
|---|---|
| 1 | Good |
| 0.5 | Degraded |
| 0.25 | Acquiring |
| 0 | Lost, or not started |

Below 0.5, TrackShot is not sending.

## Dry runs with the fake autopilot

`tools/fake_autopilot.py` is an in-process fake BlueOS. It provides mavlink2rest (HTTP and
WebSocket), an ArduSub parameter table and telemetry, and a MAVLink UDP endpoint. Once a second it
prints a summary of what TrackShot sends.

```sh
.venv/bin/python tools/fake_autopilot.py                 # mavlink2rest on 127.0.0.1:8088, UDP 14550
.venv/bin/python -m trackshot --sim --vehicle-ip 127.0.0.1:8088
```

Options:

- `--flavor 1.1` emulates newer mavlink2rest JSON.
- `--armed` checks that parameter writes are refused while armed.
- `--skew 3600` simulates a Pi clock that is off by an hour.
- `--hide-viso` hides the `VISO_*` parameters until `VISO_TYPE=1` and a reboot, as ArduPilot does
  (`kill -USR1 <pid>` simulates the reboot).

## Dry runs against ArduSub SITL

The simulator sends simulated poses to an autopilot that isn't on this Mac **only if that autopilot
is an ArduPilot SITL**: TrackShot reads `SIM_SPEEDUP`, a parameter that exists only in SITL builds,
before it opens the MAVLink link. A real vehicle never passes this check, so a simulator session
can't feed a survey that isn't happening into a real EKF.

```sh
.venv/bin/python -m trackshot --sim --sim-pattern hover --vehicle-ip 192.168.1.28
```

`--sim-pattern hover` reports a stationary vehicle, which is what a position-hold test needs.
`survey` (the default) flies a lawnmower pattern. What we learned on a BlueOS SITL (Nov 2026):

- **Reboot the autopilot after setting `VISO_TYPE`.** Until then the deltas arrive but the EKF
  ignores them, so it stays in constant-position mode and POSHOLD is refused ("requires position").
  After a reboot, "EKF3 IMU0 fusing odometry" appeared within 0.3 s of TrackShot starting.
- **ArduSub's GCS failsafe** (`FS_GCS_ENABLE`) disarms immediately unless a ground station sends
  heartbeats. Keep Cockpit open (or send GCS heartbeats) when arming a SITL.
- With the hover pattern, POSHOLD held the EKF's horizontal position within about ±1 cm.
- **When TrackShot stops, ArduSub stays armed in POSHOLD** with the EKF in constant-position mode
  ("EKF3 stopped aiding" after 5 s). With `FS_EKF_ACTION` = 0 no failsafe fires. Decide what the
  vehicle should do on VIO loss before relying on position hold.

## Build the app

GitHub Actions builds all three platforms (`.github/workflows/build.yml`). Locally, on a Mac:

```sh
scripts/build_macos.sh
```

On Linux or Windows, after the install above:

```sh
.venv/bin/python -m PyInstaller --noconfirm --clean packaging/trackshot.spec   # → dist/TrackShot/
QT_QPA_PLATFORM=offscreen dist/TrackShot/TrackShot --self-test
```

On Windows the exe is windowed and has no console output: set `TRACKSHOT_SELFTEST_LOG` to a file
path to get the self-test report there.

What `build_macos.sh` does:

The script runs these steps:

1. Runs the test suite.
2. Builds `dist/TrackShot.app` with PyInstaller from `packaging/trackshot.spec`. The bundle is onedir and windowed, about 250 MB, most of it depthai's core library.
3. Checks that no bundled binary needs a newer macOS than `LSMinimumSystemVersion`.
4. Signs the bundle ad hoc.
5. Runs the bundle's own self-test: `TrackShot.app/Contents/MacOS/TrackShot --self-test` builds both VIO nodes without a camera and pushes simulated poses through the odometry and the MAVLink encoder.

The icon is drawn by `scripts/make_icon.py`. Running it regenerates `src/trackshot/resources/icon.png`, `packaging/TrackShot.ico` and (on macOS) `packaging/TrackShot.icns`.

## Vehicle setup

1. **Stop the BlueOS OAK-D extension** (BlueOS → Extensions). The OAK accepts only one connection.
   If the extension (or any other app) holds it, TrackShot says *"The camera is in use"*.
2. **Autopilot parameters.** Open the Autopilot panel and click *Check*. It compares the vehicle
   against the settings for MAVLink visual odometry:
   - `AHRS_EKF_TYPE=3`
   - `EK3_ENABLE=1`
   - `VISO_TYPE=1`
   - `EK3_SRC1_POSXY=6` and `EK3_SRC1_VELXY=6` (ExternalNav)
   - `EK3_SRC1_POSZ=1` (baro)
   - `EK3_SRC_OPTIONS=0`

   *Apply* writes the parameters that differ. It refuses while the vehicle is armed, and checks
   again before every write: if the vehicle arms part-way (or its state can't be read), it stops
   and the panel lists what wasn't written. Reboot the autopilot when the panel says so (VISO and
   EKF selection only take effect after a reboot). The reminder belongs to the vehicle it was
   written to: changing the vehicle IP shows that vehicle's own reminders.

   Closing TrackShot while it writes parameters (*Apply* or *Revert*) doesn't cut the writes
   short: the window stays open, says it is finishing them, and closes when they are done.

   **Reverting.** Before *Apply* writes a parameter, TrackShot saves the value it had, so
   *Revert to previous…* can put the vehicle back the way it was before TrackShot:
   - Backups are kept per autopilot in
     `~/Library/Application Support/TrackShot/param_backups/<vehicle>.json` (next to
     `settings.json`). The autopilot is identified by the hardware id in its `AUTOPILOT_VERSION`
     (TrackShot requests the message if BlueOS hasn't seen it yet). An autopilot that reports no
     id is identified by its address instead (e.g. `host-192.168.2.2`), so two such vehicles at the
     same address share one backup.
   - **Only the first change is recorded.** Applying again, or applying a new `VISO_DELAY_MS`
     later, never replaces the value from before TrackShot. The value is read from the autopilot
     just before the write and saved to disk first; a parameter whose current value can't be read
     or saved isn't written.
   - The *Before TrackShot* column shows the saved values (blank: TrackShot never changed it).
   - Revert shows what it will change and asks first. It refuses while the vehicle is armed (also
     checked before every write), and restores the **newest change first** (so `VISO_DELAY_MS`
     goes back before `VISO_TYPE`). It **stops at the first parameter it can't restore**, leaving
     the older ones as TrackShot set them: `VISO_TYPE` is never put back while a newer `VISO_*`
     change is still in place (that would hide it after the next reboot). Restored parameters leave
     the backup; the rest stay in it, so Revert can be tried again.
   - **Reboot** the autopilot afterwards when the panel says so: until then it keeps running with
     the setup it started with.
   - If a parameter was changed after TrackShot wrote it (by hand, or by another ground station),
     the confirmation marks it *changed since TrackShot wrote …*. Revert still restores the value
     from before TrackShot, replacing that later change.
   - A parameter that didn't exist before TrackShot changed it can't be removed. Restoring the
     setting that enables it (`VISO_TYPE`) and rebooting hides it again.
   - A backup file that can't be read is renamed aside (`<vehicle>.corrupt-<time>.json`) and
     reported in the panel, never deleted. Restore its values by hand if you need them.
3. **`VISO_DELAY_MS`.** ArduPilot ignores the message's timestamp: it dates each delta at
   arrival minus `VISO_DELAY_MS`, and a delta's velocity is the average over its frame interval.
   TrackShot measures the camera-to-host latency (shown as Latency in the Status panel) and
   suggests `VISO_DELAY_MS` = latency + half the frame interval + about 5 ms to the autopilot,
   rounded to 10 ms. Once VIO is running, *Check* includes it and *Apply* writes it. Expect about
   110–140 ms at 10 fps over the tether.

Messages go out as system 1, component 197 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`). The transport
is either mavlink2rest's WebSocket through BlueOS (the default) or MAVLink UDP (`udpout:…`).
TrackShot also sends `TS_FPS`, `TS_LAT_MS`, `TS_JUMPS` and `TS_TRACK` as `NAMED_VALUE_FLOAT`, so
Cockpit can show VIO health.

## Bandwidth

The tether carries about **80 Mbit/s** usable, and Cockpit's video already uses some of it
(15 Mbit/s allowed for by default). TrackShot's Bandwidth panel shows the estimate for the
settings you are editing next to the measured camera throughput. It warns at 70 % of the
budget and flags 90 %. The table shows the estimates for typical settings, including the
15 Mbit/s of video:

| Stream (on top of the default RTAB-Map 320×200 at 10 fps) | Mbit/s | Total |
|---|---:|---:|
| VIO, RTAB-Map 320×200 at 10 fps (image and 16-bit depth, 3 B/px) | 15.4 | 30 |
| VIO, Basalt 320×200 at 10 fps (left and right, 2 B/px), instead | 10.3 | 25 |
| VIO, RTAB-Map 640×400 at 10 fps, instead | 61.5 | 77 (**too close**) |
| Colour preview, 640×360 MJPEG at 5 fps, q70 | 1.8 | 32 |
| Centre capture, 1920×1080 MJPEG at 1 fps, q90 | 4.1 | 35 |
| Stereo capture, 2 × 1280×800 MJPEG at 2 fps, q90 | 8.2 | 39 |

The VIO preview reuses frames the Mac already receives, so it costs nothing on the tether. Stereo
capture runs at most at the VIO frame rate, because its pairs come from the VIO cameras.

## Frame conventions

- Every timestamp is host wall clock (Unix epoch). depthai's device clock is converted on arrival.
- The VIO world frame is treated as **FLU** (x forward, y left, z up). Quaternions are
  `(qw, qx, qy, qz)`, body to world.
- `VISION_POSITION_DELTA.position_delta` is **body frame FRD**:
  `R(q_k)ᵀ · (p_k − p_{k−1})`, then `(x, −y, −z)`. `angle_delta` is the body-frame rotation vector
  from the previous attitude to the current one.
- `time_delta_usec` comes from the two frames' **capture** times. `time_usec` is the host clock at
  send time; ArduPilot doesn't use it for this message (it dates the delta at arrival minus
  `VISO_DELAY_MS`), so `VISO_DELAY_MS` covers capture to arrival plus half the frame interval.
- A delta is dropped, and differencing restarts from the new pose, when the implied speed is over
  3 m/s, when the pose snaps back to (0,0,0), or when the span is negative, under 1 ms or over 2 s.
  A VIO jump or a dropout therefore never reaches the EKF as motion.

## First wet test checklist

1. **Camera only.** Stop the BlueOS OAK-D extension, then run
   `.venv/bin/python scripts/oak_probe.py --ip 192.168.2.118` (add `--backend basalt` to try Basalt).
   Check that poses arrive at the expected rate, that latency is steady and that the chip
   temperature is sensible. Move the ROV by hand and watch the pose follow.
2. **TrackShot with mavlink2rest.** Source *OAK camera*, transport *mavlink2rest*, vehicle
   `192.168.2.2`. Press Start and check that the Status panel shows the camera, the pose rate, the
   latency and *Sent* climbing.
3. **Confirm the deltas arrive.** In Cockpit's MAVLink inspector, look for `VISION_POSITION_DELTA`
   from component 197 at the VIO rate, and check that `position_delta.x` is positive when the ROV
   moves forward. The `TS_*` named values should also be present.
4. **Parameters.** Run Autopilot → Check, then Apply, then reboot the autopilot if asked. Then set
   `VISO_DELAY_MS` to the suggested value (Check includes it once VIO runs).
5. **POSHOLD.** In the water and disarmed, check that the EKF reports a position. Then arm, switch
   to POSHOLD and hold briefly. Push the ROV gently and check that it returns. Keep a hand on
   MANUAL throughout.

## Known limitations

- **Linux and Windows builds are tested only by CI** (unit tests and the bundle self-test), not
  yet with a real camera or vehicle. Field testing so far has been on macOS. depthai's Windows
  wheel has RTAB-Map but **no Basalt**; TrackShot asks depthai which backends exist instead of
  assuming them.
- **Windows, UDP transport only:** with the default mavlink2rest transport there is no UDP socket
  and nothing to clash with (Cockpit also uses BlueOS's WebSocket). If you switch TrackShot to UDP
  on Windows, pymavlink binds the local port equal to the destination port (e.g. 14550), which
  clashes with a ground station on the same PC listening there (QGroundControl, Mission Planner).
  Point TrackShot at a different BlueOS UDP endpoint port (e.g. 14551) in that case.
- **The app is unsigned** (ad-hoc signed). On another Mac, Gatekeeper blocks it: right-click
  the app and choose Open, then confirm. On first connect, macOS asks for **Local Network** access;
  allow it, or TrackShot can't reach the camera or the vehicle.
- **Untested on hardware.** Everything has been exercised against the simulator and the fake
  autopilot, but not yet against a real OAK-D or ArduSub. In particular:
  - Camera discovery by broadcast from the packaged app is untested. Setting the camera's IP
    address avoids relying on it.
  - In depthai 3.12, Basalt's stereo matching guess type can't be set from Python, so it differs
    from the BlueOS extension's.
  - Basalt can't be stopped safely before its first camera frame (a depthai 3.12 crash).
    TrackShot waits up to 3 s for that frame; if the camera never sends one, the Basalt pipeline
    is left idle (using a CPU core) until you quit TrackShot.
  - Basalt's poses are timed by the newest image it has received, so its latency reads low when
    it falls behind the camera.
- **Camera mounting:** Settings → VIO → Camera angle. Use 0 for a forward-looking camera, and 90
  for one looking straight down with the top of the image towards the bow. TrackShot turns the
  camera's motion into the vehicle's frame (forward, right, down) with it, so `VISO_ORIENT` must be
  0 on the autopilot; the parameter check enforces that. Up to 0.1.1 the angle only rotated the
  on-screen track, as in the BlueOS extension, so a down-looking camera sent its forward motion as
  vertical unless `VISO_ORIENT` rotated it back.
- **The camera can't be booted over a slow, high-latency link** (e.g. a remote camera through
  Tailscale and Starlink). On connect, depthai uploads 40 MB of device firmware in 5 MiB pieces,
  and its network bootloader drops the connection when a keepalive waits more than 6 s. That
  limit is hard-coded in depthai 3.12; no environment variable changes it. At about 7 Mbit/s every
  attempt failed. The tether (~80 Mbit/s, sub-millisecond) boots in a few seconds.

## Development

```sh
QT_QPA_PLATFORM=offscreen .venv/bin/pytest -q
```

The suite runs in about a minute. It includes end-to-end runs from the simulator through the engine
to the fake autopilot, over both mavlink2rest and UDP.
