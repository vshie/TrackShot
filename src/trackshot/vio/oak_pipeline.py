"""OakPoseSource: the OAK-D's VIO running topside, on this computer.

Port of the BlueOS extension's `app/basalt_configurable.py` (plus the pose timing of
`mavlink_node.py` and the frame handling of `capture_node.py`) to **PyPI depthai 3.12**:

- The camera streams its stereo pair (Basalt) or rectified image + depth (RTAB-Map, with stereo
  depth still on the OAK) over the tether; depthai's host VIO node turns them into poses on this
  machine.
- One worker thread owns the whole depthai lifecycle: connect (with a retry window, because a PoE
  OAK stays busy for a few seconds after its previous owner lets go), build, start, read the
  output queues, and finally stop the pipeline and close the device so the next start can
  reconnect. Nothing here ever runs on the GUI thread.
- Optional survey capture: centre/stereo MJPEG encoded on the OAK, handed on as CapturedJpeg.

What differs from the extension's custom depthai build (verified against 3.12.0):
- BasaltVIO has no `vioConfig` attribute and no `quality` output; it takes `setConfig(VioConfig)`,
  and that copies *every* field (see `basalt_vio_config`).
- Host nodes are read through output queues on this thread instead of ThreadedHostNode
  subclasses, so stopping never depends on depthai calling back into Python.
- BasaltVIO builds its internal queues only when the first stereo pair arrives, and its stop()
  dereferences them unchecked: stopping (or destroying) a started Basalt pipeline before then
  crashes the process (SIGSEGV). See `OakPoseSource._release`.
- BasaltVIO's poses carry no timestamp, and the left image it sends after each pose is the newest
  one it has received, not necessarily the one the pose was computed from (they differ when Basalt
  falls behind and emits a burst of poses). See `_Session._poll`.
- RTABMapVIO.setUseFeatures(True) copies its `features` input into its internal Sync, so a link
  made to `features` never reaches the Sync (before or after the call). It is not called: as in the
  extension, RTAB-Map detects its own features on the host, and the OAK's tracked features are
  read only for the preview overlay.
"""

from __future__ import annotations

import copy
import io
import math
import threading
import time
from collections import deque
from datetime import timedelta
from typing import Callable, Optional

import depthai as dai
import numpy as np
from loguru import logger

from trackshot import capabilities
from trackshot.core.altitude import range_from_depth
from trackshot.vio.source import (
    RangeSample,
    STATE_CONNECTING,
    STATE_ERROR,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_STOPPED,
    CapturedJpeg,
    PoseSample,
    PoseSource,
    PreviewFrame,
    SourceCallbacks,
    SourceStats,
)

BUSY_MESSAGE = "The camera is in use — stop the BlueOS OAK-D extension (or any other app using it)"
NO_IMU_MESSAGE = ("This camera reports no IMU; VIO needs one. Check the camera model, or its IMU "
                  "in the BlueOS extension's IMU panel.")

# A PoE OAK stays busy for a few seconds after its previous owner disconnects (e.g. the BlueOS
# extension just stopped, or TrackShot is restarting with new settings), so connecting retries
CONNECT_WINDOW_S = 30.0
RETRY_INTERVAL_S = 3.0

PREVIEW_MAX_HZ = 10.0
# A pose's frame is sent by the VIO node right after the pose; this long is plenty to wait for it
FRAME_WAIT_S = 0.2
# Pose and frame output queues: a stall shorter than this many VIO frames (3 s at 10 fps) loses nothing
VIO_QUEUE_SIZE = 32
RANGE_HZ = 10.0           # altitude (DISTANCE_SENSOR) rate; ArduSub times a MAVLink rangefinder out after 0.5 s
# How long a stop waits for a started Basalt pipeline's first frame, so that it can be stopped safely
BASALT_FIRST_FRAME_WAIT_S = 3.0
# Device timestamps older than this (or in the future) are unset or nonsensical
MAX_TIMESTAMP_AGE_S = 5.0
CHIP_TEMP_INTERVAL_S = 1.0
IMU_RATE_HZ = 200

# Native sensor geometry, used to turn calibrated focal lengths into millimetres for EXIF
IMX378_PIXEL_MM, IMX378_NATIVE_WIDTH = 0.00155, 4056
OV9282_PIXEL_MM, OV9282_NATIVE_WIDTH = 0.003, 1280
CAMERA_MAKE = b"MarineSitu C3"

# depthai 3.12's BasaltVIO::setDefaultVIOConfig(), field for field. A Python `dai.VioConfig()`
# starts with every field zeroed (not at these defaults), and `BasaltVIO.setConfig()` copies all of
# them, so a config with only the tuned fields set would zero Basalt's thresholds and noise models.
# The three enum fields (matching guess type, linearization type, keyframe marginalisation
# criteria) are not settable from Python in 3.12 (unregistered pybind types) and stay at value 0:
# SAME_PIXEL / ABS_QR / KF_MARG_DEFAULT. Only the first differs from depthai's default
# (REPROJ_AVG_DEPTH).
DEPTHAI_BASALT_DEFAULTS: dict[str, object] = {
    "optical_flow_type": "frame_to_frame",
    "optical_flow_detection_grid_size": 50,
    "optical_flow_detection_num_points_cell": 1,
    "optical_flow_detection_min_threshold": 5,
    "optical_flow_detection_max_threshold": 40,
    "optical_flow_detection_nonoverlap": True,
    "optical_flow_max_recovered_dist2": 0.04,
    "optical_flow_pattern": 51,
    "optical_flow_max_iterations": 5,
    "optical_flow_epipolar_error": 0.005,
    "optical_flow_levels": 3,
    "optical_flow_skip_frames": 1,
    "optical_flow_matching_default_depth": 2.0,
    "optical_flow_image_safe_radius": 472.0,
    "optical_flow_recall_enable": False,
    "optical_flow_recall_all_cams": False,
    "optical_flow_recall_num_points_cell": True,
    "optical_flow_recall_over_tracking": False,
    "optical_flow_recall_update_patch_viewpoint": False,
    "optical_flow_recall_max_patch_dist": 3.0,
    "optical_flow_recall_max_patch_norms": [1.74, 0.96, 0.99, 0.44],
    "vio_sqrt_marg": True,
    "vio_max_states": 3,
    "vio_max_kfs": 7,
    "vio_min_frames_after_kf": 5,
    "vio_new_kf_keypoints_thresh": 0.7,
    "vio_debug": False,
    "vio_extended_logging": False,
    "vio_obs_std_dev": 0.5,
    "vio_obs_huber_thresh": 1.0,
    "vio_min_triangulation_dist": 0.05,
    "vio_max_iterations": 7,
    "vio_enforce_realtime": False,
    "vio_use_lm": True,
    "vio_lm_lambda_initial": 1e-4,
    "vio_lm_lambda_min": 1e-6,
    "vio_lm_lambda_max": 1e2,
    "vio_scale_jacobian": False,
    "vio_init_pose_weight": 1e8,
    "vio_init_ba_weight": 1e1,
    "vio_init_bg_weight": 1e2,
    "vio_marg_lost_landmarks": True,
    "vio_fix_long_term_keyframes": False,
    "vio_kf_marg_feature_ratio": 0.1,
    "mapper_obs_std_dev": 0.25,
    "mapper_obs_huber_thresh": 1.5,
    "mapper_detection_num_points": 800,
    "mapper_num_frames_to_match": 30.0,
    "mapper_frames_to_match_threshold": 0.04,
    "mapper_min_matches": 20.0,
    "mapper_ransac_threshold": 5e-5,
    "mapper_min_track_length": 5.0,
    "mapper_max_hamming_distance": 70.0,
    "mapper_second_best_test_ratio": 1.2,
    "mapper_bow_num_bits": 16,
    "mapper_min_triangulation_dist": 0.07,
    "mapper_no_factor_weights": False,
    "mapper_use_factors": True,
    "mapper_use_lm": True,
    "mapper_lm_lambda_min": 1e-32,
    "mapper_lm_lambda_max": 1e3,
}

# Settings field -> VioConfig field, as the extension's --grid-size ... --skip-frames mapped them
BASALT_SETTINGS_MAP = {
    "basalt_grid_size": "optical_flow_detection_grid_size",
    "basalt_flow_iterations": "optical_flow_max_iterations",
    "basalt_flow_levels": "optical_flow_levels",
    "basalt_max_states": "vio_max_states",
    "basalt_max_kfs": "vio_max_kfs",
    "basalt_vio_iterations": "vio_max_iterations",
    "basalt_skip_frames": "optical_flow_skip_frames",
}


# --- Pure helpers ------------------------------------------------------------------------------

def basalt_vio_config(settings) -> "dai.VioConfig":
    """A complete VioConfig: depthai's defaults with the user's Basalt settings on top."""
    cfg = dai.VioConfig()
    for name, value in DEPTHAI_BASALT_DEFAULTS.items():
        setattr(cfg, name, value)
    for setting, field_name in BASALT_SETTINGS_MAP.items():
        setattr(cfg, field_name, int(getattr(settings, setting)))
    return cfg


# RTAB-Map's odometry stays lost for good once it loses tracking (Odom/ResetCountdown 0, its
# default) and keeps reporting its last pose. With a countdown of N it resets itself to that latest
# pose after N consecutive lost frames, and again every N while still lost. That runs on RTAB-Map's
# own thread; depthai's RTABMapVIO.reset() would instead race the node's processing thread (it
# takes no lock), so TrackShot never calls it. N is this many seconds of frames: retrying every few
# seconds, not every frame.
RTABMAP_RESET_AFTER_S = 3.0


def rtabmap_reset_countdown(fps: float) -> int:
    return max(1, round(float(fps) * RTABMAP_RESET_AFTER_S))


def rtabmap_params(settings) -> dict[str, str]:
    # The odometry is one host thread and limits the frame rate. Frame-to-map matching cost grows
    # with the local map, so it is kept smaller than RTAB-Map's 2000.
    return {
        "OdomF2M/MaxSize": str(settings.rtabmap_local_map_size),
        "Vis/MaxFeatures": str(settings.rtabmap_features),
        "Odom/ResetCountdown": str(rtabmap_reset_countdown(settings.fps)),
    }


def device_time_to_wall(ts: Optional[timedelta], device_now: timedelta, wall_now: float) -> Optional[float]:
    """Wall-clock seconds of a depthai host-synced timestamp, or None if it is unset/nonsensical.

    depthai stamps messages on its steady clock (`dai.Clock.now()`), so the wall time is "now"
    minus the message's age on that clock.
    """
    if ts is None or ts <= timedelta(0):
        return None  # depthai's VIO nodes leave their TransformData unstamped (zero)
    age = (device_now - ts).total_seconds()
    if not 0.0 <= age < MAX_TIMESTAMP_AGE_S:
        return None
    return wall_now - age


_BUSY_MARKERS = (
    "already_in_use",            # XLinkError_t.X_LINK_DEVICE_ALREADY_IN_USE in error text
    "already in use",
    "not in unbooted",           # "Device not in UNBOOTED, BOOTLOADER or FLASH_BOOTED state"
    "x_link_booted",
    "x_link_gate_booted",
)
_NOT_FOUND_MARKERS = ("cannot find any device", "no available devices", "device_not_found",
                      "failed to find device", "no oak camera found")


def is_busy_error(error) -> bool:
    """Whether a connect error means another host owns the camera."""
    if isinstance(error, CameraBusyError):
        return True
    text = str(error).lower()
    return any(marker in text for marker in _BUSY_MARKERS)


def connect_error_message(error, camera_ip: str) -> str:
    """The message to show for a failed connection attempt."""
    if is_busy_error(error):
        return BUSY_MESSAGE
    text = str(error)
    if any(marker in text.lower() for marker in _NOT_FOUND_MARKERS):
        if camera_ip:
            return f"No camera answered at {camera_ip}. Check the tether, the camera's power and its IP address"
        return "No OAK camera found. Check the tether and the camera's power, or set its IP address"
    where = f" at {camera_ip}" if camera_ip else ""
    return f"Could not connect to the camera{where}: {text}"


class RateMeter:
    """Events (or bytes) per second over a sliding window. Thread-safe."""

    def __init__(self, window_s: float = 2.0, clock: Callable[[], float] = time.monotonic):
        self.window_s = window_s
        self._clock = clock
        self._t0 = clock()
        self._samples: deque[tuple[float, float]] = deque()
        self._lock = threading.Lock()

    def add(self, amount: float = 1.0, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        with self._lock:
            self._samples.append((now, amount))
            self._trim(now)

    def rate(self, now: Optional[float] = None) -> float:
        now = self._clock() if now is None else now
        with self._lock:
            self._trim(now)
            total = sum(amount for _, amount in self._samples)
        # Until a full window has passed, divide by the time actually observed (but not by a
        # sliver of it, which would turn the first sample into a huge spike)
        span = max(min(self.window_s, now - self._t0), min(self.window_s, 0.5))
        return total / span

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._samples and self._samples[0][0] <= cutoff:
            self._samples.popleft()


class PreviewThrottle:
    """Lets frames through at no more than `max_hz` on average.

    Frames are admitted on an ideal schedule (with a little slack) rather than "at least 1/max_hz
    since the last one", so a stream arriving at exactly max_hz isn't halved by timing jitter.
    """

    def __init__(self, max_hz: float = PREVIEW_MAX_HZ):
        self.interval = 1.0 / max_hz
        self._slack = 0.2 * self.interval
        self._next: Optional[float] = None

    def ready(self, now: float) -> bool:
        if self._next is not None and now < self._next - self._slack:
            return False
        if self._next is None or now - self._next >= self.interval:
            self._next = now + self.interval  # first frame, or after a pause: restart the schedule
        else:
            self._next += self.interval
        return True


def grey_image(frame) -> np.ndarray:
    """(H, W) uint8 copy of a frame's first plane (GRAY8/RAW8, or the Y plane of NV12/YUV400p)."""
    width, height = frame.getWidth(), frame.getHeight()
    data = np.asarray(frame.getData(), dtype=np.uint8)
    stride = frame.getStride() or width
    if data.size < stride * height:
        stride = width
    # Copied: getData() is a view into depthai's buffer, which is reused once the frame is gone
    return data[:stride * height].reshape(height, stride)[:, :width].copy()


def decode_jpeg_rgb(data: bytes) -> np.ndarray:
    from PIL import Image
    with Image.open(io.BytesIO(data)) as img:
        return np.asarray(img.convert("RGB"))


def feature_positions(tracked) -> np.ndarray:
    """(N, 2) float32 pixel positions from a dai.TrackedFeatures message."""
    features = tracked.trackedFeatures
    if not features:
        return np.zeros((0, 2), dtype=np.float32)
    return np.array([(f.position.x, f.position.y) for f in features], dtype=np.float32)


def message_nbytes(msg) -> int:
    try:
        return int(msg.getData().nbytes)
    except Exception:
        return 0


def lens_for(calib, socket, make, model, width, height, native_pixel_mm, native_width):
    """Lens from the camera's (in-water) calibration at the captured resolution, or None."""
    try:
        from trackshot.capture.meta import Lens
    except ImportError:
        return None
    try:
        fx = calib.getCameraIntrinsics(socket, width, height)[0][0]
        return Lens.from_intrinsics(make, model, fx, native_pixel_mm, native_width, width)
    except Exception as e:
        logger.warning(f"No calibration for {socket}, images will have no focal length: {e}")
        return None


# --- Connection --------------------------------------------------------------------------------

class CameraBusyError(RuntimeError):
    pass


# Started Basalt pipelines that never received a stereo pair: depthai 3.12 crashes stopping or
# destroying them (see the module docstring), so they are kept here, never released, and the app
# leaves with os._exit while any exist (see `has_parked_pipelines`)
_PARKED_PIPELINES: list = []


def has_parked_pipelines() -> bool:
    """Whether a pipeline had to be left running; the process must then end with os._exit, because
    the interpreter's normal shutdown would destroy it and crash."""
    return bool(_PARKED_PIPELINES)


class _Failure(Exception):
    """A failure with a message meant for the user; ends the run in STATE_ERROR."""


def _enum_name(value) -> str:
    return getattr(value, "name", None) or str(value).rsplit(".", 1)[-1]


def open_device(camera_ip: str) -> "dai.Device":
    """Connects to the camera at `camera_ip` (or the first free one). Raises on failure.

    Discovery (~0.5 s) runs first because it also lists booted devices, which tells a camera held
    by another host apart from one that isn't there.
    """
    try:
        infos = dai.Device.getAllConnectedDevices()
    except Exception as e:
        logger.debug(f"Discovery failed: {e}")
        infos = []
    busy = [i for i in infos if _enum_name(i.state) in capabilities.BUSY_STATES]
    if camera_ip:
        match = next((i for i in infos if camera_ip in (i.name, i.getDeviceId())), None)
        if match is not None and match in busy:
            raise CameraBusyError(BUSY_MESSAGE)
        # Broadcast discovery doesn't cross routers, so an undiscovered address is still tried
        return dai.Device(match if match is not None else dai.DeviceInfo(camera_ip))
    free = [i for i in infos if i not in busy]
    if free:
        return dai.Device(free[0])
    if busy:
        raise CameraBusyError(BUSY_MESSAGE)
    raise RuntimeError("No OAK camera found")


# --- The pose source ---------------------------------------------------------------------------

class OakPoseSource(PoseSource):
    """Runs the chosen VIO backend on a real OAK. See the module docstring."""

    def __init__(self, settings, callbacks: SourceCallbacks,
                 connect_window_s: float = CONNECT_WINDOW_S, retry_interval_s: float = RETRY_INTERVAL_S):
        super().__init__(settings, callbacks)
        self.connect_window_s = connect_window_s
        self.retry_interval_s = retry_interval_s
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._state = STATE_IDLE
        self._preview_vio = bool(settings.preview_enabled)
        self._preview_color = bool(settings.preview_enabled and settings.color_preview_enabled)
        self._color_built = False
        self._stats = SourceStats(backend=settings.backend)
        self._pose_meter = RateMeter()
        self._rx_meter = RateMeter()

    # -- PoseSource API ------------------------------------------------------------------------
    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive() and not self._stop_event.is_set():
                return
            previous = self._thread
            # A fresh event per run: a previous run still closing keeps its own (set) event
            self._stop_event = threading.Event()
            self._thread = threading.Thread(target=self._run, args=(self._stop_event, previous),
                                            name="oak-pose-source", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        with self._lock:
            thread, stop_event = self._thread, self._stop_event
        if thread is None:
            return
        stop_event.set()
        thread.join(timeout)
        if thread.is_alive():
            # Typically blocked in depthai's ~10 s device search; it closes the device itself
            self._log("warning", "The camera is still being released; it will close in the background")

    def is_running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive() and not self._stop_event.is_set()

    def stats(self) -> SourceStats:
        with self._lock:
            stats = copy.copy(self._stats)
            stats.extra = dict(self._stats.extra)
            running = self._state == STATE_RUNNING
        stats.fps = self._pose_meter.rate()
        if running:
            stats.rx_mbit = self._rx_meter.rate() * 8.0 / 1e6
        return stats

    def set_preview(self, vio: bool, color: bool) -> None:
        with self._lock:
            self._preview_vio = bool(vio)
            self._preview_color = bool(color)
            needs_restart = color and not self._color_built and self._state == STATE_RUNNING
        if needs_restart:
            self._log("info", "The colour preview is a separate camera stream: it starts on the next restart")

    # -- Reporting -----------------------------------------------------------------------------
    def _log(self, level: str, message: str) -> None:
        getattr(logger, level)(message)
        self._call(self.callbacks.on_log, level, message)

    def _set_state(self, state: str, message: Optional[str] = None) -> None:
        with self._lock:
            self._state = state
        self._call(self.callbacks.on_state, state, message)

    @staticmethod
    def _call(callback, *args) -> None:
        # A consumer's bug must not kill the camera thread
        try:
            callback(*args)
        except Exception:
            logger.exception("Pose source callback failed")

    # -- Worker thread -------------------------------------------------------------------------
    def _run(self, stop: threading.Event, previous: Optional[threading.Thread]) -> None:
        if previous is not None and previous.is_alive():
            self._log("info", "Waiting for the previous camera connection to close")
            previous.join()
        s = copy.copy(self.settings)
        with self._lock:
            self._stats = SourceStats(backend=s.backend)
            self._color_built = False
        self._pose_meter = RateMeter()
        self._rx_meter = RateMeter()
        device = None
        pipeline = None
        session = None
        try:
            if s.backend not in capabilities.available_backends():
                raise _Failure(f"The {s.backend} VIO backend isn't available in this depthai build "
                               f"({', '.join(capabilities.available_backends()) or 'none'} available)")
            device = self._connect(s, stop)
            if device is None or stop.is_set():
                return
            pipeline = dai.Pipeline(device)
            session = _Session(self, s, device, pipeline)
            session.build()
            if stop.is_set():
                return
            session.started = True   # before start(): if it fails half-way, assume the nodes run
            pipeline.start()
            # RUNNING only once a pose arrives (_Session._on_pose): until then the VIO isn't working
            self._set_state(STATE_CONNECTING, f"Camera started; waiting for the first VIO pose ({s.backend})")
            session.loop(stop)
        except _Failure as e:
            if not stop.is_set():
                self._log("error", str(e))
                self._set_state(STATE_ERROR, str(e))
        except Exception as e:
            if not stop.is_set():
                logger.exception("Camera pipeline failed")
                message = f"Camera pipeline failed: {e}"
                self._call(self.callbacks.on_log, "error", message)
                self._set_state(STATE_ERROR, message)
        finally:
            self._release(pipeline, device, session)
            if stop.is_set():
                self._set_state(STATE_STOPPED, "Camera released")

    def _connect(self, s, stop: threading.Event) -> Optional["dai.Device"]:
        where = f"at {s.camera_ip}" if s.camera_ip else "(first one found)"
        self._set_state(STATE_CONNECTING, f"Connecting to the camera {where}")
        deadline = time.monotonic() + self.connect_window_s
        while not stop.is_set():  # checked before each attempt: one can block ~10 s in depthai's search
            try:
                return open_device(s.camera_ip)
            except Exception as e:
                message = connect_error_message(e, s.camera_ip)
                logger.debug(f"Connect attempt failed: {e}")
            if stop.is_set():
                return None
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise _Failure(message)
            self._log("warning", f"{message}. Retrying…")
            self._set_state(STATE_CONNECTING, f"{message}. Retrying…")
            stop.wait(min(self.retry_interval_s, remaining))
        return None

    def _release(self, pipeline, device, session: Optional["_Session"]) -> None:
        """Stops the pipeline and closes the device, so the next start (or the BlueOS
        extension) can connect. pipeline.stop() closes the device only if the pipeline started.

        A started Basalt pipeline that hasn't had its first stereo pair can't be stopped or
        destroyed without crashing the process (module docstring). Its first frame is waited for
        (bounded); if none comes, the pipeline is parked for the rest of the process and only the
        device is closed.
        """
        if pipeline is not None and session is not None and not session.wait_until_stoppable(BASALT_FIRST_FRAME_WAIT_S):
            _PARKED_PIPELINES.append(pipeline)
            self._log("warning", "Basalt never received a camera frame, and depthai can't stop it safely "
                                 "before then: its pipeline stays idle until TrackShot quits (using a CPU core)")
            pipeline = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception as e:
                logger.warning(f"Pipeline stop failed: {e}")
        if device is not None:
            try:
                device.close()
            except Exception as e:
                logger.warning(f"Device close failed: {e}")


class _Session:
    """One connected run: builds the pipeline and turns its output queues into callbacks.
    Lives entirely on the source's worker thread."""

    def __init__(self, source: OakPoseSource, settings, device, pipeline):
        self.source = source
        self.s = settings
        self.device = device
        self.pipeline = pipeline
        self.q_pose = None
        self.q_frame = None
        self.q_features = None
        self.q_depth = None         # RTAB-Map's passthrough of the depth it received: altitude
        self.next_range_t = 0.0
        self.q_color = None
        self.captures: dict[str, tuple] = {}   # stream -> (queue, lens)
        # Bytes crossing the tether per VIO frame, as a multiple of the frame the host sees
        # (Basalt: left + right; RTAB-Map: rectified 1 B/px + depth 2 B/px)
        self.vio_bytes_factor = 2 if settings.backend == "basalt" else 3
        self.started = False        # pipeline.start() was called
        self.vio_output = False     # the VIO node has sent something (so it has initialised)
        self.running_reported = False
        self.last_frame_seq: Optional[int] = None
        self.last_pose_frame_seq: Optional[int] = None
        self.latest_features: Optional[np.ndarray] = None
        self.vio_throttle = PreviewThrottle()
        self.color_throttle = PreviewThrottle()
        self.next_temp_check = 0.0

    # -- Build (port of basalt_configurable.main / build_* ) ---------------------------------
    def build(self) -> None:
        s, p, device = self.s, self.pipeline, self.device
        imu_type = str(device.getConnectedIMU() or "")
        if not imu_type or imu_type.upper() == "NONE":
            raise _Failure(NO_IMU_MESSAGE)
        self._describe_device(imu_type)
        self.source._log("info", f"Camera IMU: {imu_type}")

        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(s.fps))
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(s.fps))
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], IMU_RATE_HZ)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        if s.backend == "rtabmap":
            odom, frame_out = self._build_rtabmap(left, right, imu)
        else:
            odom, frame_out = self._build_basalt(left, right, imu)
        # Poses and the frames behind them are paired 1:1 by arrival order, so both queues are
        # sized to absorb a stall without dropping either side (see _poll for when they overflow)
        self.q_pose = odom.transform.createOutputQueue(maxSize=VIO_QUEUE_SIZE, blocking=False)
        self.q_frame = frame_out.createOutputQueue(maxSize=VIO_QUEUE_SIZE, blocking=False)
        if s.backend == "rtabmap":
            # The depth frames RTAB-Map already received, passed through on this computer: the
            # altitude costs no tether bandwidth. (Basalt has no stereo depth to tap.)
            self.q_depth = odom.passthroughDepth.createOutputQueue(maxSize=2, blocking=False)

        center = self._build_center_camera()
        if center is not None and s.preview_enabled and s.color_preview_enabled:
            self._build_color_preview(center)
        self._build_capture(left, right, center)

    def _describe_device(self, imu_type: str) -> None:
        name = camera_id = None
        try:
            name = str(self.device.getProductName() or self.device.getDeviceName() or "") or None
            camera_id = self.device.getDeviceId()
        except Exception as e:
            logger.debug(f"Could not read the camera's name: {e}")
        with self.source._lock:
            self.source._stats.camera_name = name
            self.source._stats.camera_id = camera_id
            self.source._stats.imu = imu_type
        self.source._log("info", f"Connected to {name or 'OAK'} ({camera_id})")

    def _build_basalt(self, left, right, imu):
        s = self.s
        odom = self.pipeline.create(dai.node.BasaltVIO)
        odom.setConfig(basalt_vio_config(s))
        left.requestOutput((s.width, s.height)).link(odom.left)
        right.requestOutput((s.width, s.height)).link(odom.right)
        imu.out.link(odom.imu)
        self.source._log("info", "Backend: Basalt (VIO on this computer)")
        # Basalt's poses carry no timestamp; the left image it sends after each one does, but it is
        # the newest image Basalt has received (see _poll)
        return odom, odom.passthrough

    def _build_rtabmap(self, left, right, imu):
        """Stereo depth on the OAK; RTAB-Map odometry (with its own feature detection) on this computer."""
        s, p = self.s, self.pipeline
        stereo = p.create(dai.node.StereoDepth)
        feature_tracker = p.create(dai.node.FeatureTracker)
        odom = p.create(dai.node.RTABMapVIO)

        left.requestOutput((s.width, s.height)).link(stereo.left)
        right.requestOutput((s.width, s.height)).link(stereo.right)
        stereo.setExtendedDisparity(False)
        stereo.setLeftRightCheck(True)
        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)  # the OV9282 lenses on the OAK-D W are wide
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        # On the OAK, so free for the host: less depth noise for RTAB-Map to reject
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

        feature_tracker.setHardwareResources(1, 2)
        feature_tracker.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
        feature_tracker.initialConfig.setNumTargetFeatures(s.rtabmap_features)

        stereo.rectifiedLeft.link(feature_tracker.inputImage)
        feature_tracker.passthroughInputImage.link(odom.rect)
        stereo.depth.link(odom.depth)
        feature_tracker.outputFeatures.link(odom.features)
        imu.out.link(odom.imu)
        # No setUseFeatures(True) (see the module docstring): RTAB-Map detects features on the host,
        # as in the extension. The OAK's tracked features are read directly for the preview overlay.
        self.q_features = feature_tracker.outputFeatures.createOutputQueue(maxSize=2, blocking=False)
        odom.setParams(rtabmap_params(s))
        self.source._log("info", f"Backend: RTAB-Map (stereo depth on the OAK, odometry on this computer, "
                                 f"local map {s.rtabmap_local_map_size})")
        return odom, odom.passthroughRect

    def _build_center_camera(self):
        """The centre colour camera, shared by the colour preview and centre capture."""
        s = self.s
        rates = []
        if s.preview_enabled and s.color_preview_enabled:
            rates.append(float(s.color_preview_fps))
        if s.center_capture_fps > 0:
            rates.append(float(s.center_capture_fps))
        if not rates:
            return None
        # The sensor runs a little faster than the slowest outputs; they are decimated on the OAK
        return self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A,
                                                           sensorFps=max(max(rates), 5.0))

    def _mjpeg(self, out, fps: float, quality: int):
        return self.pipeline.create(dai.node.VideoEncoder).build(
            out, frameRate=fps, profile=dai.VideoEncoderProperties.Profile.MJPEG, quality=quality)

    def _build_color_preview(self, center) -> None:
        s = self.s
        fps = float(s.color_preview_fps)
        out = center.requestOutput((s.color_preview_width, s.color_preview_height), dai.ImgFrame.Type.NV12, fps=fps)
        self.q_color = self._mjpeg(out, fps, s.color_preview_quality).out.createOutputQueue(maxSize=2, blocking=False)
        with self.source._lock:
            self.source._color_built = True
        self.source._log("info", f"Colour preview {s.color_preview_width}x{s.color_preview_height} at {fps:g} fps")

    def _build_capture(self, left, right, center) -> None:
        """Port of build_capture: MJPEG survey images encoded on the OAK."""
        s = self.s
        stereo_fps = s.effective_stereo_capture_fps()
        if s.center_capture_fps <= 0 and stereo_fps <= 0:
            return
        calib = self.device.readCalibration()

        if s.center_capture_fps > 0:
            fps = float(s.center_capture_fps)
            out = center.requestOutput((s.center_width, s.center_height), dai.ImgFrame.Type.NV12, fps=fps)
            queue = self._mjpeg(out, fps, s.jpeg_quality).out.createOutputQueue(maxSize=4, blocking=False)
            lens = lens_for(calib, dai.CameraBoardSocket.CAM_A, CAMERA_MAKE, b"IMX378",
                            s.center_width, s.center_height, IMX378_PIXEL_MM, IMX378_NATIVE_WIDTH)
            self.captures["center"] = (queue, lens)
            self.source._log("info", f"Capturing centre {s.center_width}x{s.center_height} at {fps:g} fps")

        if stereo_fps > 0:
            # Stereo pairs come from the sensors that feed VIO, so they can't run faster than it
            if stereo_fps < s.stereo_capture_fps:
                self.source._log("info", f"Stereo capture capped at the VIO frame rate: {stereo_fps:g} fps "
                                         f"(set to {s.stereo_capture_fps:g})")
            for name, cam, socket in (("left", left, dai.CameraBoardSocket.CAM_B),
                                      ("right", right, dai.CameraBoardSocket.CAM_C)):
                # The OAK's VideoEncoder takes 8-bit grey only as YUV400p; it rejects GRAY8 frames
                out = cam.requestOutput((s.mono_width, s.mono_height), dai.ImgFrame.Type.YUV400p, fps=stereo_fps)
                queue = self._mjpeg(out, stereo_fps, s.jpeg_quality).out.createOutputQueue(maxSize=4, blocking=False)
                lens = lens_for(calib, socket, CAMERA_MAKE, f"OV9282 {name}".encode(),
                                s.mono_width, s.mono_height, OV9282_PIXEL_MM, OV9282_NATIVE_WIDTH)
                self.captures[name] = (queue, lens)
            self.source._log("info", f"Capturing stereo {s.mono_width}x{s.mono_height} at {stereo_fps:g} fps "
                                     "(JPEG on the OAK)")

    # -- Run -----------------------------------------------------------------------------------
    def loop(self, stop: threading.Event) -> None:
        try:
            while not stop.is_set():
                if not self.pipeline.isRunning():
                    raise _Failure("Lost the camera: the pipeline stopped (tether or camera power?)")
                got = self._poll()
                got = self._poll_depth() or got
                self._check_temperature()
                if not got:
                    stop.wait(0.005)
        except dai.MessageQueue.QueueException:
            if not stop.is_set():
                raise _Failure("Lost the camera: its data stream closed (tether or camera power?)")

    def wait_until_stoppable(self, timeout: float) -> bool:
        """Whether pipeline.stop() is safe, waiting up to `timeout` for that to become true.

        Only a started Basalt pipeline whose VIO hasn't initialised (no output yet) is unsafe; its
        first frame normally comes within a second of start, so a stop during warm-up waits for it.
        """
        if self.s.backend != "basalt" or not self.started or self.vio_output:
            return True
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline and self.pipeline.isRunning():
                if self.q_frame.tryGetAll() or self.q_pose.tryGetAll():
                    self.vio_output = True
                    return True
                time.sleep(0.01)
        except Exception as e:   # the queues close with the device: no frame is coming
            logger.debug(f"No Basalt frame before stopping: {e}")
        return False

    def _poll(self) -> bool:
        """Drains every output queue once. Returns whether anything arrived.

        Poses and frames come from two queues. The VIO node sends pose k, then frame k, then pose
        k+1, so with the frames drained *before* the poses, every drained frame's pose has been
        drained too: in this batch, or earlier. A frame whose pose came earlier is an orphan (that
        pose timed out waiting for it, or was discarded below), and orphans can only lead the
        batch. So surplus frames are dropped from the front, and the rest pair with the poses in
        order; a pose sent after the frames were drained waits for its frame.

        A queue found full may have dropped messages on one side only, which would offset the
        pairs; that batch is discarded rather than paired. The next delta then spans the gap (or
        the chain restarts if that is over 2 s).
        """
        got = False
        if self.q_features is not None:
            for tracked in self.q_features.tryGetAll():
                got = True
                self.latest_features = feature_positions(tracked)
        overflowed = (self.q_frame.getSize() >= self.q_frame.getMaxSize()
                      or self.q_pose.getSize() >= self.q_pose.getMaxSize())
        frames = deque(self.q_frame.tryGetAll())
        poses = self.q_pose.tryGetAll()
        if frames or poses:
            got = True
            self.vio_output = True
        if overflowed:
            self._bump("overflow_resyncs")
            logger.warning(f"VIO output backed up (host busy?): skipped {len(poses)} poses to stay in step")
            return True
        orphans = len(frames) - len(poses)
        if orphans > 0:
            for _ in range(orphans):
                frames.popleft()
            self._bump("orphan_frames", orphans)
            logger.debug(f"Dropped {orphans} VIO frame(s) whose pose was already handled")
        pairs = [(pose, frames.popleft() if frames else self._wait_frame()) for pose in poses]
        for i, (pose, frame) in enumerate(pairs):
            following = pairs[i + 1][1] if i + 1 < len(pairs) else None
            if self._stale_basalt_pose(frame, following):
                continue
            self._on_pose(pose, frame)
        if self.q_color is not None:
            got |= self._poll_color()
        for stream, (queue, lens) in self.captures.items():
            for frame in queue.tryGetAll():
                got = True
                self._on_capture(stream, frame, lens)
        return got

    def _stale_basalt_pose(self, frame, following) -> bool:
        """Whether this Basalt pose is one of a burst and must be skipped.

        BasaltVIO sends, after each pose, the newest left image it has received. When it falls
        behind and then catches up, it emits several poses before another image arrives, and they
        all carry that image: only the last one really belongs to it, the earlier ones were
        computed from older images whose times are unknown. Skipping them is exact: the next
        delta runs from the pose before the burst to the burst's last pose, over the right span.
        """
        if self.s.backend != "basalt" or frame is None or following is None:
            return False
        if following.getSequenceNum() != frame.getSequenceNum():
            return False
        self._bump("stale_poses")
        return True

    def _wait_frame(self):
        frame = self.q_frame.get(timedelta(seconds=FRAME_WAIT_S))
        if frame is None:
            logger.debug("No VIO frame followed a pose; its capture time is unknown")
        return frame

    def _on_pose(self, pose, frame) -> None:
        wall_now = time.time()
        device_now = dai.Clock.now()
        t = pose.getTranslation()
        q = pose.getQuaternion()
        values = (t.x, t.y, t.z, q.qw, q.qx, q.qy, q.qz)
        if frame is not None:
            self._count_vio_frame(frame)
        if not all(math.isfinite(v) for v in values):
            self._bump("invalid_poses")
            return

        # Prefer the pose's own stamp if a future depthai sets it; today the frame's is the one
        t_capture = device_time_to_wall(pose.getTimestamp(), device_now, wall_now)
        if t_capture is None and frame is not None:
            t_capture = device_time_to_wall(frame.getTimestamp(), device_now, wall_now)
        if t_capture is None:
            self._bump("untimed_poses")

        if frame is not None and self.s.backend == "basalt":
            seq = frame.getSequenceNum()
            if seq == self.last_pose_frame_seq:
                # A burst split across two polls: the previous pose was sent with this image's time,
                # which belongs to this one. Its delta is lost; the chain restarts here (zero span).
                self._bump("stale_poses")
            self.last_pose_frame_seq = seq

        sample = PoseSample(
            t_capture_us=int(t_capture * 1e6) if t_capture is not None else None,
            t_received=wall_now,
            position=(float(t.x), float(t.y), float(t.z)),
            quaternion=(float(q.qw), float(q.qx), float(q.qy), float(q.qz)),
        )
        self.source._pose_meter.add()
        with self.source._lock:
            extra = self.source._stats.extra
            extra["poses"] = extra.get("poses", 0) + 1
            if t_capture is not None:
                extra["latency_ms"] = round((wall_now - t_capture) * 1000.0, 1)
            preview = self.source._preview_vio
        if not self.running_reported:
            self.running_reported = True
            s = self.s
            self.source._set_state(STATE_RUNNING, f"VIO running ({s.backend}, {s.width}x{s.height} at {s.fps} fps)")
        self.source._call(self.source.callbacks.on_pose, sample)

        if preview and frame is not None and self.vio_throttle.ready(time.monotonic()):
            features = self.latest_features if self.s.backend == "rtabmap" else None
            self.source._call(self.source.callbacks.on_preview,
                              PreviewFrame(kind="vio", image=grey_image(frame), t_received=wall_now, features=features))

    def _count_vio_frame(self, frame) -> None:
        """Adds the VIO input behind this frame to the measured rx rate.

        The VIO node consumes its inputs inside depthai, so only its passthrough frame is seen
        here. Frames the host dropped (VIO busy) still crossed the tether: the sequence-number gap
        since the last frame counts them.
        """
        seq = frame.getSequenceNum()
        if seq == self.last_frame_seq:
            return   # Basalt resent the same image after another pose: nothing new crossed the tether
        sent = 1
        if self.last_frame_seq is not None and 0 < seq - self.last_frame_seq <= 30:
            sent = seq - self.last_frame_seq
        self.last_frame_seq = seq
        self.source._rx_meter.add(sent * self.vio_bytes_factor * message_nbytes(frame))

    def _poll_color(self) -> bool:
        frames = self.q_color.tryGetAll()
        if not frames:
            return False
        for frame in frames:
            self.source._rx_meter.add(message_nbytes(frame))
        with self.source._lock:
            enabled = self.source._preview_color
        now = time.time()
        if enabled and self.color_throttle.ready(time.monotonic()):
            try:
                image = decode_jpeg_rgb(frames[-1].getData().tobytes())
            except Exception as e:
                logger.warning(f"Could not decode a colour preview frame: {e}")
                return True
            self.source._call(self.source.callbacks.on_preview,
                              PreviewFrame(kind="color", image=image, t_received=now))
        return True

    def _on_capture(self, stream: str, frame, lens) -> None:
        data = frame.getData().tobytes()
        self.source._rx_meter.add(len(data))
        wall_now = time.time()
        t_capture = device_time_to_wall(frame.getTimestamp(), dai.Clock.now(), wall_now)
        self.source._call(self.source.callbacks.on_capture, CapturedJpeg(
            stream=stream, data=data, t_capture=t_capture if t_capture is not None else wall_now,
            sequence=frame.getSequenceNum(), lens=lens))

    def _poll_depth(self) -> bool:
        """Altitude from the newest depth frame, at most RANGE_HZ."""
        if self.q_depth is None:
            return False
        frames = self.q_depth.tryGetAll()
        if not frames:
            return False
        now = time.monotonic()
        if now < self.next_range_t:
            return True
        self.next_range_t = now + 1.0 / RANGE_HZ
        frame = frames[-1]
        wall_now = time.time()
        t = device_time_to_wall(frame.getTimestamp(), dai.Clock.now(), wall_now) or wall_now
        estimate = range_from_depth(frame.getFrame(), t)
        self.source._call(self.source.callbacks.on_range,
                          RangeSample(estimate.t, estimate.distance_m, estimate.quality, estimate.valid_fraction))
        return True

    def _check_temperature(self) -> None:
        now = time.monotonic()
        if now < self.next_temp_check:
            return
        self.next_temp_check = now + CHIP_TEMP_INTERVAL_S
        try:
            temp = round(float(self.device.getChipTemperature().average), 1)
        except Exception as e:
            logger.debug(f"Chip temperature unavailable: {e}")
            temp = None
        with self.source._lock:
            self.source._stats.chip_temp_c = temp

    def _bump(self, key: str, amount: int = 1) -> None:
        with self.source._lock:
            extra = self.source._stats.extra
            extra[key] = extra.get(key, 0) + amount
