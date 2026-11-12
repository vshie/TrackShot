# TrackShot architecture and module contracts

TrackShot runs visual-inertial odometry **topside**. It connects straight to the ROV's OAK-D (PoE,
over the tether), runs depthai's host VIO node (RTAB-Map or Basalt) on the surface computer, and
sends `VISION_POSITION_DELTA` to ArduSub. It replaces running VIO on the Pi in the BlueOS extension
`~/Documents/Blueos-Oakd-Basalt` (branch `tonytest`), whose code is the reference for every ported
module below. **The OAK accepts one connection: the BlueOS extension must be stopped while
TrackShot runs**, and survey capture lives here too.

Target now: macOS arm64, Python 3.14, `.venv` in the repo. Later: Windows (depthai's Windows wheel
has RTAB-Map but **no BasaltVIO**) and Linux. Never assume a backend exists; ask `capabilities`.

## Data flow
```
PoseSource (OakPoseSource | SimPoseSource)        threads owned by the source
   │ on_pose(PoseSample) ─► OdometryProcessor.process() ─► DeltaMessage ─► messages.vision_position_delta() ─► Transport.send()
   │ on_preview(PreviewFrame) ─► Engine (throttle ≤10 Hz) ─► Qt signal ─► PreviewPanel
   │ on_capture(CapturedJpeg) ─► CaptureWriter.submit(jpeg, VehicleTelemetry.latest(), pose, tracking)
   │ on_state / on_log ─► Engine ─► Qt signals
Engine (QObject, GUI thread) polls stats at 4 Hz ─► statusUpdated(dict) ─► panels
```
Nothing blocks the GUI thread. All cross-thread delivery to widgets goes through Qt signals
(queued connections). Source callbacks run on source threads and must return quickly.

## Clock and frame conventions (critical)
- All times in `PoseSample`, `PreviewFrame`, `CapturedJpeg` are **host wall-clock** (Unix epoch).
  OakPoseSource converts depthai's steady clock:
  `t_wall = time.time() - (dai.Clock.now() - ts).total_seconds()`.
- VIO world frame as reported by depthai's VIO nodes is treated as **FLU** (x forward, y left,
  z up), the same as the extension. Quaternions are `(qw, qx, qy, qz)`, body→world.
- `VISION_POSITION_DELTA.position_delta` is **body frame FRD** (x forward, y right, z down):
  `delta_world = p_k − p_{k−1}`, `delta_body_flu = R(q_k)ᵀ · delta_world`, then FLU→FRD
  `(x, −y, −z)`. Camera angle rotation (pitch about y) is applied to pose before differencing
  (port `apply_camera_rotation` exactly).
- `time_delta_usec` comes from **capture times** (`t_capture_us` difference); if a sample lacks a
  capture time, restart the differencing chain (never span two clocks).
- `time_usec` = host wall-clock µs **at send time** (same as the extension). ArduPilot does **not**
  read it for VISION_POSITION_DELTA: it stamps each delta at its own receipt time minus
  `VISO_DELAY_MS`, and the EKF fuses `delPos/delTime` as the velocity at that instant, with no
  half-interval correction. That velocity is the average over the span, true at its midpoint, so
  `VISO_DELAY_MS` must cover capture→host latency (measured, EMA α=0.1) + half the span (EMA of
  `time_delta_usec`) + host→autopilot transit (~5 ms, assumed). TrackShot suggests that sum.
- Reject a delta (and restart from the new pose) when: implied speed > 3 m/s, the pose snaps to
  exactly (0,0,0) after having moved, span < 1 ms, span < 0, or span > 2 s.
- MAVLink header: `source_system` = 1, `source_component` = 197
  (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`; the extension used 195 = PATHPLANNER by mistake).

## Settings — `trackshot/config.py` (DONE, shared)
`Settings` dataclass. JSON at `platformdirs.user_config_dir("TrackShot")/settings.json`.
`Settings.load()`, `.save()`, `.validate() -> list[str]`, `.mavlink2rest_ws_url()`,
`.mavlink2rest_http_url()`, `.effective_stereo_capture_fps()`. `vehicle_ip` may carry a port
(`"127.0.0.1:8088"`) for the fake autopilot — build URLs by string formatting only.

## Bandwidth — `trackshot/bandwidth.py` (DONE, shared)
`estimate(settings) -> BandwidthEstimate` (vio/preview/capture/video/total Mbit/s, budget, level
`ok|warn|bad` at 70 %/90 % of the 80 Mbit tether, `hint`). VIO preview costs 0 tether bandwidth.

## Pose source contract — `trackshot/vio/source.py` (DONE, shared)
`PoseSample`, `PreviewFrame`, `CapturedJpeg`, `SourceStats`, `SourceCallbacks`, `PoseSource`
(`start()` non-blocking, `stop(timeout)`, `is_running()`, `stats()`, `set_preview(vio, color)`),
`STATE_*` constants. Read the file; do not change it without updating this doc.

## Transport contract — `trackshot/mavlink/base.py` (DONE, shared)
`Transport` (`start`, `stop`, `send(msg_type, fields) -> bool` never blocks, `stats()`),
`TransportStats`. Field names are pymavlink names; `char[N]` given as `str`.

---
## Modules to build

### `trackshot/core/odometry.py` (pure, no depthai/Qt)
```python
@dataclass(frozen=True)
class DeltaMessage:
    time_usec: int; time_delta_usec: int
    position_delta: tuple[float, float, float]   # FRD body, m
    angle_delta: tuple[float, float, float]      # FRD body rotation vector prev→current, rad
    confidence: float                            # 0–100

@dataclass
class OdometryHealth:
    fps: float; latency_ms: float | None; suggested_viso_delay_ms: int | None
    poses: int; deltas: int; jumps: int; rejected_spans: int
    last_pose: tuple | None          # (x, y, z, qw, qx, qy, qz) after camera rotation
    last_pose_age_s: float | None; stalled: bool   # no pose for > 1 s while running
    tracking: str                    # "good" | "degraded" | "lost" | "unknown"

class OdometryProcessor:
    def __init__(self, camera_angle_deg=0.0, max_speed_mps=3.0, max_span_s=2.0, clock=time.time): ...
    def process(self, sample: PoseSample) -> DeltaMessage | None   # thread-safe
    def health(self) -> OdometryHealth                             # thread-safe
    def reset(self) -> None
```
`suggested_viso_delay_ms` = latency EMA + ½ × span EMA + 5 ms transit (see the clock conventions),
rounded to the nearest 10 ms, clamped 0–250 (ArduPilot's VISO_DELAY_MS range). `confidence` = 100 while tracking, 0 when the sample says `tracking=False`.
`tracking`: "lost" if stalled or the last sample said False; "degraded" if ≥3 jumps in the last 10 s;
"good" otherwise once poses flow; "unknown" before the first pose.

### `trackshot/mavlink/messages.py`
```python
def vision_position_delta(d: DeltaMessage) -> tuple[str, dict]
def named_value_float(name: str, value: float, time_boot_ms: int) -> tuple[str, dict]   # name ≤ 10 chars
def health_values(h: OdometryHealth, time_boot_ms: int) -> list[tuple[str, dict]]       # TS_FPS, TS_LAT_MS, TS_JUMPS, TS_TRACK (1/0.5/0)
def encode(mav, msg_type: str, fields: dict)   # -> pymavlink MAVLink_message via mav.<type>_encode(**fields)
```

### `trackshot/mavlink/transport.py`
```python
class Mavlink2RestTransport(Transport):   # WebSocket, port of the extension's send_mavlink_message_ws
    def __init__(self, ws_url: str, source_system: int, source_component: int, queue_size: int = 64)
class UdpTransport(Transport):            # pymavlink mavutil connection, e.g. "udpout:192.168.2.2:14550"
    def __init__(self, url: str, source_system: int, source_component: int, queue_size: int = 64)
def make_transport(settings) -> Transport
```
mavlink2rest frame: `{"header": {"system_id", "component_id", "sequence"}, "message": {"type": T, ...fields}}`.
Append `?filter=SEND_ONLY` (a regex matching no message type) so the server doesn't stream the
vehicle's traffic back. Encode `char[N]` the way mavlink2rest expects (verify against mavlink2rest's
source/docs). A sender thread drains a bounded queue (drop-oldest on overflow, counted), reconnects
with backoff (1 s → 5 s), never raises out of `send`.

### `trackshot/autopilot/params.py` — port of `app/autopilot_params.py`
`VISUAL_ODOMETRY_PARAMS`, exceptions `AutopilotUnreachable`, `NoHeartbeat`, `VehicleArmed`.
```python
class AutopilotParams:
    def __init__(self, http_base_url: str, system_id=1, component_id=1, timeout=3.0)
    def read(self, name) -> float | None; def write(self, name, value, param_type) -> float | None
    def is_armed(self) -> bool
    def check(self, suggested_viso_delay_ms: int | None = None) -> list[dict]
        # [{name, current, recommended, needs_reboot, ok}], VISO_DELAY_MS included when suggested
    def apply(self, diffs: list[dict], backup: ParamBackup | None = None) -> list[dict]
        # refuses while armed (VehicleArmed); with a backup, records each original before writing
    def vehicle_identity(self) -> tuple[str, str]   # (key, description): AUTOPILOT_VERSION uid2/uid
        # (requested with MAV_CMD_REQUEST_MESSAGE 148 if not cached), else "host-<address>"
    def vehicle_key(self) -> str
    def preview_revert(self, backup) -> list[dict]  # read-only: {name, current, original, absent,
        # written_by_trackshot, drifted, unconfirmed, needs_reboot, action}
    def revert(self, backup) -> list[dict]          # refuses while armed; newest change first
    def get_origin(self) -> tuple[float, float] | None
def suggest_viso_delay_ms(latency_ms: float | None) -> int | None
```

### `trackshot/autopilot/backup.py` — parameter values from before TrackShot
`ParamBackup(vehicle_key, directory=None)`: one JSON file per autopilot in
`<config dir>/param_backups/<vehicle_key>.json`, written atomically (temporary file, fsync,
`os.replace`). `record_original(name, value | None, param_type, needs_reboot)` keeps only the
first original per parameter and is saved before the write it protects; `mark_written`,
`remove`, `entries()` (application order). An unreadable file is renamed aside
(`.corrupt-<time>.json`), reported in `problem` and by `set_aside_files()`, never deleted.
`open_backup(params)` opens the backup for the autopilot an AutopilotParams talks to (network
I/O: worker threads only). The Autopilot panel loads it after each Check (the *Before TrackShot*
column, *Revert to previous…*), passes it to apply(), and runs preview_revert()/revert() on its
QThreadPool.

### `trackshot/autopilot/telemetry.py` — port of `app/vehicle_telemetry.py`
```python
def read_autopilot_telemetry(http_base_url: str) -> dict
    # keys: lat, lon, heading_deg, roll_deg, pitch_deg, depth_m, temperature_c, altitude_m, mission_seq
class VehicleTelemetry:   # background poller so capture never waits on HTTP
    def __init__(self, http_base_url: str, poll_hz: float = 2.0)
    def start(self); def stop(self)
    def latest(self) -> dict   # the keys above + connected: bool, armed: bool|None, mode: str|None,
                               #   heartbeat_age_s: float|None, age_s: float|None (snapshot age)
```

### `trackshot/testing/fake_autopilot.py` (+ `tools/fake_autopilot.py` CLI wrapper)
In-process fake BlueOS for tests and manual runs: one HTTP+WS server on `127.0.0.1:<port>`
emulating the mavlink2rest subset TrackShot uses (WS `/mavlink2rest/ws/mavlink`, the HTTP GET/POST
endpoints `AutopilotParams`/`VehicleTelemetry` call, with a param table and HEARTBEAT/ATTITUDE/…),
plus a UDP listener decoding with pymavlink. Records every received message:
`fake.received(msg_type=None) -> list[dict]`, `fake.vehicle_ip` ("127.0.0.1:PORT"), `fake.udp_url`,
`fake.set_armed(bool)`, `fake.params` dict, `start()`/`stop()`, context manager.

### `trackshot/capture/meta.py` — copy of `app/photogrammetry_meta.py` (no depthai import)
### `trackshot/capture/writer.py` — port of `app/capture_node.py`, decoupled from depthai
```python
class CaptureWriter:
    def __init__(self, captures_dir, camera_angle_deg=0.0, min_free_gb=2.0, source_tag="tl000000")
    recording: bool (property); def set_recording(on: bool); def set_source_tag(tag); def set_waypoint(wp)
    def submit(self, jpeg: CapturedJpeg, telemetry: dict | None, pose: tuple | None, tracking: bool | None) -> None
        # never blocks; pairs left/right by sequence (fallback: t_capture within 15 ms); thread pool writes
    def snapshot(self) -> dict  # recording, folder, counts {"center","stereo"}, dropped, free_gb, disk_full, last_error, last_file
    def close(self, timeout=5.0)
```
Same folder layout, filenames, CSV header and EXIF/XMP as the extension.

### `trackshot/capabilities.py`
`available_backends() -> list[str]` (by `hasattr(dai.node, ...)`, cached), `depthai_version()`,
`describe() -> dict` (platform, arch, python, depthai, backends), `list_cameras() -> list[dict]`
(name, id/ip, state; never raises).

### `trackshot/vio/oak_pipeline.py` — `OakPoseSource(PoseSource)`
Port of `app/basalt_configurable.py` adapted to **PyPI depthai 3.12**:
- Basalt: `odom.setConfig(cfg)` with `cfg = dai.VioConfig()` fields; no quality output.
- RTAB-Map: `setParams({...})`. Never `setUseFeatures(True)`: in 3.12 it copies the `features`
  input into the internal Sync, so the OAK's FeatureTracker link never reaches it. RTAB-Map detects
  features on the host (as in the extension); the tracker's output feeds only the preview overlay.
- Basalt: a started BasaltVIO must not be stopped or destroyed before its first stereo pair
  (SIGSEGV in 3.12). Stop waits for the first frame (bounded); otherwise the pipeline is parked
  and the process exits with `os._exit` (`oak_pipeline.has_parked_pipelines()`). STATE_RUNNING is
  reported on the first pose, not at `pipeline.start()`.
- Connect to `settings.camera_ip` (or first device) with a 30 s retry; when the device is busy
  say "The camera is in use — stop the BlueOS OAK-D extension (or any other app using it)".
- Check the IMU; fail clearly if none.
- Pose timing: prefer `TransformData.getTimestamp()` if it is set; otherwise pair with the VIO
  node's passthrough frame (as the extension does). Frames are drained before poses so pairing is
  exact; surplus (orphan) frames are dropped from the front, and a batch where a queue was full is
  discarded. Basalt's passthrough is its *newest* received frame: in a burst of poses sharing one
  frame only the last is kept. Basalt's latency is therefore a lower bound when it falls behind.
- Preview: VIO passthrough image (+ features for RTAB-Map) when preview on (no extra stream);
  optional colour preview = CAM_A MJPEG at `color_preview_*`, decoded on the host.
- Capture: centre/stereo MJPEG encoded on the OAK (port `build_capture`, `lens_for`), stereo capped
  at the VIO fps, delivered as `CapturedJpeg`.
- Stats: pose fps, measured rx Mbit/s (sum of received message sizes / time), chip temp at 1 Hz.

### `trackshot/vio/sim.py` — `SimPoseSource(PoseSource)`
Deterministic synthetic ROV: a lawnmower survey path with yaw turns, small noise, simulated
60 ± 15 ms latency, optional scripted jump/dropout (`SimPoseSource(settings, callbacks, seed=0,
realtime=True, script=None)`); synthetic grey preview with moving texture and fake features;
synthetic JPEGs (Pillow) at the capture rates, so the whole app can be exercised without hardware.
`realtime=False` runs as fast as possible with simulated timestamps (for tests).

### `trackshot/engine.py` — `Engine(QObject)` (integration)
```python
stateChanged = Signal(str, str)      # (STATE_*, message)
statusUpdated = Signal(dict)         # 4 Hz: {"state", "source": SourceStats, "odometry": OdometryHealth,
                                     #   "transport": TransportStats, "capture": dict|None,
                                     #   "telemetry": dict|None, "bandwidth": BandwidthEstimate}
poseUpdated = Signal(object)         # (PoseSample, last_pose tuple after camera rotation), ≤ 20 Hz
previewUpdated = Signal(object)      # PreviewFrame, ≤ 10 Hz
logMessage = Signal(str, str)        # (level, text)
def __init__(self, settings: Settings, parent=None)
settings (property); state (property)
def start(self); def stop(self); def apply_settings(self, s: Settings)  # restarts if running and needed
def set_recording(self, on: bool); def set_preview(self, vio: bool, color: bool)
def autopilot_params(self) -> AutopilotParams
```

### `trackshot/ui/` — PySide6 + pyqtgraph
`main_window.py` (`MainWindow(engine)`), panels under `ui/panels/`: settings form, status,
trajectory (top-down XY + depth vs time), preview (toggle), bandwidth (estimate vs measured),
autopilot params (check/apply/suggested VISO_DELAY_MS), capture, log. `__main__.py` wires
`Settings.load()` → `Engine` → `MainWindow`.
