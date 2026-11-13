TrackShot runs visual-inertial odometry (RTAB-Map or Basalt) on the topside computer against the ROV's
OAK-D over the tether, and sends it to ArduSub through BlueOS, so position modes and AUTO missions
work without GPS or a DVL. It can also act as the vehicle's downward rangefinder and record survey
images tagged with position and attitude.

## New in 0.2.0

**Read before upgrading:** set the camera angle and re-run the parameter check (below). The default
output and the camera-mount handling have changed.

- **The camera mount is applied properly.** With Settings → VIO → Camera angle 90° (looking straight
  down, top of the image towards the bow), forward, sideways and vertical motion come out on the
  vehicle's axes.
  - 0.1.x only rotated the on-screen track, so a down-looking camera sent forward motion as
    vertical unless `VISO_ORIENT` corrected it on the autopilot.
  - The parameter check now requires `VISO_ORIENT` = 0.
- **Horizontal-only output by default.** TrackShot now sends VISION_POSITION_ESTIMATE +
  VISION_SPEED_ESTIMATE, with nothing on the vertical axis.
  - The motion is rotated into north/east by the autopilot's compass heading; nothing is learned.
  - With `EK3_SRC1_VELZ` = 0 (now in the check), the depth sensor alone sets the vertical.
  - The 0.1.x output (VISION_POSITION_DELTA, which ArduSub fuses on all three axes) is still under
    Settings → MAVLink → Send VIO as.
- **TrackShot as the rangefinder: seabed following without a DVL.**
  - With RTAB-Map and the camera looking down, *Send altitude as a rangefinder* sends the stereo
    altitude as a downward DISTANCE_SENSOR.
  - Plan surveys in Cockpit with Altitude = **Relative to terrain** and AUTO follows the seabed at
    that height. No script needed.
  - The parameter check sets the rangefinder (`RNGFND1_*`) and `WP_RFND_USE`, plus
    `FS_TERRAIN_ENAB` = 1, so a lost altitude in AUTO holds position instead of disarming.
- **Cockpit widget.** Add TrackShot to Cockpit with its IFrame widget; Help → *Add TrackShot to
  Cockpit…* gives the URL and steps.
  - It shows the VIO image with the tracked features, the tracking lock, depth, altitude and
    rangefinder.
  - It has Start/Stop tracking and Record buttons.
  - It's reachable from the network by default, with no login.
- **Depth & altitude panel** (pressure depth, TrackShot altitude, the autopilot's rangefinder).
- **North-up map** with the vehicle arrow at the compass heading.
- **Per-run data logs:** `runs/run_<date>_<time>.csv` next to `trackshot.log`, one row per pose:
  the camera and vehicle pose, tracking, what was sent, the autopilot's heading, depth and
  rangefinder.
- **Fixes:**
  - The settings refuse an RTAB-Map width that isn't a multiple of 16 (300 px hung RTAB-Map).
  - DISTANCE_SENSOR and other messages with enum fields now reach BlueOS's mavlink2rest.

## Downloads

| Platform | File | Notes |
|---|---|---|
| macOS, Apple silicon | `TrackShot-*-macos-arm64.zip` | Ad-hoc signed: first launch with right-click → Open (or `xattr -dr com.apple.quarantine TrackShot.app`). Allow Local Network access when asked. |
| Linux x86_64 | `TrackShot-*-linux-x86_64.tar.gz` | Extract and run `./TrackShot/TrackShot`. Ubuntu 22.04 or newer (glibc 2.35). |
| Windows x64 | `TrackShot-*-windows-x64.zip` | Extract and run `TrackShot\TrackShot.exe`. Allow it through Windows Firewall on private networks. RTAB-Map only (depthai's Windows build has no Basalt). |

## Before you connect

- Stop the BlueOS OAK-D extension: the camera accepts only one connection.
- Settings → VIO → Camera angle: 0 for a forward-looking camera, 90 for one looking straight down.
- In TrackShot's Autopilot panel, check the parameters and apply them; every change can be reverted.
  Set `VISO_DELAY_MS` from the suggested value after about 30 s of VIO.
- If a DVL feeds the autopilot's rangefinder, leave *Send altitude as a rangefinder* off.

## Also in the repository

`ardupilot/gps_depth_gate.lua` is an ArduSub 4.7 Lua script. It uses GPS only at the surface (6 or
more satellites, HDOP of 4 or less, shallower than 0.5 m) and ignores it underwater, where position
comes from VIO or DVL odometry. It was tested in ArduSub 4.7.2 SITL. The parameters it needs are in
the header of the file.

## Status

**Not yet tried in the water or with a vehicle:** the horizontal-only output, the camera-mount fix,
TrackShot's altitude as the rangefinder, and the Cockpit widget. All four are tested against
simulators and a fake autopilot. Try POSHOLD and a short terrain-following leg in open water first.
The Linux and Windows builds pass CI's tests and the bundle self-test only.
