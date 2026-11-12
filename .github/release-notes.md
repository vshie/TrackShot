TrackShot runs visual-inertial odometry (RTAB-Map or Basalt) on the topside computer against the ROV's
OAK-D over the tether, and sends `VISION_POSITION_DELTA` to ArduSub via BlueOS, so position modes and
AUTO missions work without GPS. It can also record survey images tagged with position and attitude.

## Downloads

| Platform | File | Notes |
|---|---|---|
| macOS, Apple silicon | `TrackShot-*-macos-arm64.zip` | Ad-hoc signed: first launch with right-click → Open (or `xattr -dr com.apple.quarantine TrackShot.app`). Allow Local Network access when asked. |
| Linux x86_64 | `TrackShot-*-linux-x86_64.tar.gz` | Extract and run `./TrackShot/TrackShot`. Ubuntu 22.04 or newer (glibc 2.35). |
| Windows x64 | `TrackShot-*-windows-x64.zip` | Extract and run `TrackShot\TrackShot.exe`. Allow it through Windows Firewall on private networks. RTAB-Map only (depthai's Windows build has no Basalt). |

## Before you connect

- Stop the BlueOS OAK-D extension: the camera accepts only one connection.
- In TrackShot's Autopilot panel, check the parameters and apply them; every change can be reverted.
  Set `VISO_DELAY_MS` from the suggested value after about 30 s of VIO.

## Also in this release

`ardupilot/gps_depth_gate.lua` is an ArduSub 4.7 Lua script. It uses GPS only at the surface (6 or more
satellites, HDOP of 4 or less, shallower than 0.5 m) and ignores it underwater, where position comes
from VIO or DVL odometry. It was tested in ArduSub 4.7.2 SITL. The parameters it needs are in the
header of the file.

## Status

The macOS app has been used in the water. The Linux and Windows builds pass CI's unit tests and the
bundle self-test, but have not yet been run against a real camera or vehicle.
