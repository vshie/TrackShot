"""Engine: wires a pose source, the odometry, the MAVLink transport, telemetry and capture together.

The Engine is a QObject that lives on the GUI thread. The work happens elsewhere:

- The pose source calls back on its own thread. `on_pose` runs the odometry and queues the
  VISION_POSITION_DELTA there (both are quick and never block), so the GUI's load can never
  delay or reorder what ArduPilot receives.
- Anything the UI shows is handed to the GUI thread through private signals connected with
  `Qt.QueuedConnection` to the Engine's own slots, which then emit the public signals. The public
  signals are therefore always emitted on the GUI thread, whatever is connected to them.
- Poses (≤ 20 Hz) and previews (≤ 10 Hz per kind) are throttled on the source thread and handed
  over through a latest-value mailbox: a busy GUI thread gets the newest item, never a backlog.
- Stopping can take seconds (closing a PoE camera, joining sender threads), so teardown runs on a
  daemon thread; a start() requested meanwhile begins once the camera has been released.

Data flow and the signal payloads are documented in docs/ARCHITECTURE.md.
"""

from __future__ import annotations

import copy
import ipaddress
import math
import threading
import time
import weakref
from dataclasses import fields
from typing import Any, Callable, Optional

from loguru import logger
from PySide6.QtCore import QObject, Qt, QTimer, Signal, Slot

from trackshot import bandwidth
from trackshot.autopilot.params import AutopilotParams
from trackshot.autopilot.telemetry import VehicleTelemetry
from trackshot.capture.writer import CaptureWriter
from trackshot.config import Settings
from trackshot.core.odometry import OdometryProcessor, apply_camera_rotation, create_camera_rotation_matrix
from trackshot.mavlink import messages
from trackshot.mavlink.base import Transport
from trackshot.mavlink.transport import make_transport
from trackshot.vio.source import (STATE_CONNECTING, STATE_ERROR, STATE_IDLE, STATE_RUNNING, STATE_STOPPED,
                                  CapturedJpeg, PoseSample, PoseSource, PreviewFrame, SourceCallbacks)

STATUS_INTERVAL_MS = 250          # statusUpdated at 4 Hz
POSE_MAX_HZ = 20.0                # the trajectory can't show more, and Qt's event queue shouldn't carry it
PREVIEW_MAX_HZ = 10.0
TELEMETRY_POLL_HZ = 2.0
# Exists only in ArduPilot SITL builds: the simulator may feed an autopilot that has it
SITL_MARKER_PARAM = "SIM_SPEEDUP"
SOURCE_STOP_TIMEOUT_S = 5.0       # OakPoseSource closes the device itself if this runs out
WRITER_CLOSE_TIMEOUT_S = 5.0
# The pilot's Restart VIO reconnects the camera (~15 s); one at a time, and not again within this
RESTART_VIO_COOLDOWN_S = 20.0

# Settings the engine applies while running. A change to any other field changes the camera
# pipeline, the odometry or the MAVLink link, so it restarts the run.
LIVE_FIELDS = frozenset({
    "preview_enabled", "capture_dir", "min_free_gb", "source_tag", "send_health", "health_rate_hz",
    "tether_budget_mbit", "video_allowance_mbit",
})
# Fields the CaptureWriter was built with: a change replaces the writer
WRITER_FIELDS = frozenset({"capture_dir", "min_free_gb", "camera_angle_deg"})

SourceFactory = Callable[[Settings, SourceCallbacks], PoseSource]


def transport_host(settings: Settings) -> str:
    """The host the MAVLink transport would send to."""
    if settings.transport == "udp":
        # "udpout:HOST:PORT" (or "udp:HOST:PORT")
        parts = settings.udp_url.split(":")
        return parts[1] if len(parts) >= 3 else parts[-1]
    return settings.vehicle_ip.rsplit(":", 1)[0] if settings.vehicle_ip.count(":") == 1 else settings.vehicle_ip


def is_loopback(host: str) -> bool:
    host = host.strip("[]")
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def default_source_factory(settings: Settings, callbacks: SourceCallbacks) -> PoseSource:
    if settings.source == "sim":
        from trackshot.vio.sim import SimPoseSource
        return SimPoseSource(settings, callbacks)
    # Imported here: depthai is only loaded when a camera is wanted, so a broken depthai install
    # still lets the simulator and the rest of the app run
    from trackshot.vio.oak_pipeline import OakPoseSource
    return OakPoseSource(settings, callbacks)


class _Throttle:
    """At most `max_hz` items per second, tolerant of arrival jitter and without a burst after a pause."""

    def __init__(self, max_hz: float):
        self._period = 1.0 / max_hz
        self._next = -math.inf

    def ready(self, now: float) -> bool:
        if now < self._next:
            return False
        # Schedule from the ideal slot rather than the arrival, so a frame arriving a little early
        # isn't dropped; never from further back than half a period, so a pause doesn't bank credit
        self._next = max(self._next + self._period, now + 0.5 * self._period)
        return True


class _Mailbox:
    """Latest-value hand-off to the GUI thread with at most one queued signal in flight."""

    def __init__(self, notify: Callable[[], None]):
        self._notify = notify
        self._lock = threading.Lock()
        self._items: dict[str, Any] = {}
        self._pending = False

    def put(self, key: str, item: Any) -> None:
        with self._lock:
            self._items[key] = item
            if self._pending:
                return
            self._pending = True
        self._notify()

    def take(self) -> dict[str, Any]:
        with self._lock:
            items, self._items = self._items, {}
            self._pending = False
        return items


class _Run:
    """Everything one start() builds. Source callbacks check `active`, so a source thread that is
    still winding down can't touch the next run."""

    def __init__(self, settings: Settings):
        self.settings = settings
        self.active = True
        self.processor = OdometryProcessor(camera_angle_deg=settings.camera_angle_deg)
        self.rotation = (create_camera_rotation_matrix(settings.camera_angle_deg)
                         if settings.camera_angle_deg else None)
        self.source: Optional[PoseSource] = None
        self.transport: Optional[Transport] = None
        self.telemetry: Optional[VehicleTelemetry] = None
        self.started = time.monotonic()
        # Written by the source thread, read by capture: single assignments, so no lock needed
        self.last_pose: Optional[tuple] = None
        self.last_tracking: Optional[bool] = None
        self.pose_throttle = _Throttle(POSE_MAX_HZ)
        self.preview_throttles = {"vio": _Throttle(PREVIEW_MAX_HZ), "color": _Throttle(PREVIEW_MAX_HZ)}

    def time_boot_ms(self) -> int:
        return int((time.monotonic() - self.started) * 1000)


class Engine(QObject):
    stateChanged = Signal(str, str)      # (STATE_*, message)
    statusUpdated = Signal(dict)         # 4 Hz, see status()
    poseUpdated = Signal(object)         # (PoseSample, (x, y, z, qw, qx, qy, qz) after camera rotation)
    previewUpdated = Signal(object)      # PreviewFrame
    logMessage = Signal(str, str)        # (level, text)

    # Private: emitted from any thread, delivered to the slots below on the GUI thread
    _posesReady = Signal()
    _previewsReady = Signal()
    _sourceState = Signal(object, str, str)
    _logReady = Signal(str, str)
    _teardownDone = Signal()

    def __init__(self, settings: Settings, parent: Optional[QObject] = None,
                 source_factory: SourceFactory = default_source_factory):
        super().__init__(parent)
        self._settings = copy.deepcopy(settings)
        self._source_factory = source_factory
        self._state = STATE_IDLE
        self._run: Optional[_Run] = None
        self._start_pending = False
        self._teardowns: list[threading.Thread] = []
        # Teardowns whose _teardownDone the GUI thread hasn't handled yet. Counted here rather than
        # read from Thread.is_alive(): the signal is emitted from the thread's last line, so the
        # thread can still be alive when its own "done" arrives, and the pending start would be lost.
        self._teardowns_outstanding = 0
        self._writer: Optional[CaptureWriter] = None
        # Replaced writers flushing their backlog, and the threads closing them; shutdown waits for them
        self._closing_writers: list[tuple[CaptureWriter, threading.Thread]] = []
        self._recording = False
        self._last_vio_restart = -math.inf
        self._preview_vio = settings.preview_enabled
        self._preview_color = settings.preview_enabled and settings.color_preview_enabled

        queued = Qt.ConnectionType.QueuedConnection
        self._poses = _Mailbox(self._posesReady.emit)
        self._previews = _Mailbox(self._previewsReady.emit)
        self._posesReady.connect(self._deliver_poses, queued)
        self._previewsReady.connect(self._deliver_previews, queued)
        self._sourceState.connect(self._on_source_state, queued)
        self._logReady.connect(self._forward_log, queued)
        self._teardownDone.connect(self._on_teardown_done, queued)

        self._status_timer = QTimer(self)
        self._status_timer.setInterval(STATUS_INTERVAL_MS)
        self._status_timer.timeout.connect(self._emit_status)
        self._status_timer.start()
        self._health_timer = QTimer(self)
        self._health_timer.timeout.connect(self._send_health)

        self._sink_id: Optional[int] = logger.add(_make_log_sink(self), level="INFO", format="{message}",
                                                  catch=True)

    # --- Properties -----------------------------------------------------------------------------
    @property
    def settings(self) -> Settings:
        return copy.deepcopy(self._settings)

    @property
    def state(self) -> str:
        return self._state

    # --- Control (GUI thread) -------------------------------------------------------------------
    def start(self) -> None:
        if self._run is not None:
            return
        problems = self._settings.validate()
        if problems:
            self._set_state(STATE_ERROR, "Can't start: " + "; ".join(problems))
            return
        if self._teardown_busy():
            # The previous run still holds the camera; start as soon as it has let go
            self._start_pending = True
            self._set_state(STATE_CONNECTING, "Waiting for the previous run to close…")
            return
        self._start_run()

    def stop(self) -> None:
        """Returns at once; the source, transport and telemetry close on a background thread."""
        self._start_pending = False
        run, self._run = self._run, None
        if run is not None:
            self._teardown(run)
        if self._state not in (STATE_IDLE, STATE_STOPPED):
            self._set_state(STATE_STOPPED, "Stopped")

    def restart_vio(self) -> bool:
        """The pilot's "Restart VIO": a fresh camera pipeline and VIO, keeping the settings,
        recording and MAVLink link setup. For a VIO that stays lost (RTAB-Map already resets
        itself after a few seconds lost; see oak_pipeline.RTABMAP_RESET_AFTER_S). Refused while
        one is under way or within RESTART_VIO_COOLDOWN_S of the last. Returns whether it restarted."""
        blocked = self.restart_vio_blocked()
        if blocked is not None:
            logger.info(f"Restart VIO not done: {blocked}")
            return False
        self._last_vio_restart = time.monotonic()
        logger.info("Restarting VIO (pilot request): reconnecting the camera")
        self.stop()
        self.start()
        return True

    def restart_vio_blocked(self) -> Optional[str]:
        """Why Restart VIO isn't available now, or None if it is."""
        if self._run is None and not self._start_pending:
            return "VIO isn't running"
        if self._state != STATE_RUNNING:
            return "VIO is still starting"
        wait = RESTART_VIO_COOLDOWN_S - (time.monotonic() - self._last_vio_restart)
        if wait > 0:
            return f"restarted moments ago; available again in {math.ceil(wait)} s"
        return None

    def apply_settings(self, s: Settings) -> None:
        problems = s.validate()
        if problems:
            logger.error("Settings not applied: " + "; ".join(problems))
            return
        old, new = self._settings, copy.deepcopy(s)
        changed = {f.name for f in fields(Settings) if getattr(old, f.name) != getattr(new, f.name)}
        self._settings = new
        if not changed:
            return
        if changed & WRITER_FIELDS and self._writer is not None:
            self._replace_writer()
        if "source_tag" in changed and self._writer is not None:
            self._writer.set_source_tag(new.source_tag)
        if "preview_enabled" in changed:
            self.set_preview(new.preview_enabled, new.preview_enabled and new.color_preview_enabled)
        if changed & {"send_health", "health_rate_hz"} and self._run is not None:
            self._configure_health_timer()
        restart = changed - LIVE_FIELDS
        if restart and (self._run is not None or self._start_pending):
            logger.info(f"Restarting to apply: {', '.join(sorted(restart))}")
            self.stop()
            self.start()

    def set_recording(self, on: bool) -> None:
        self._recording = bool(on)
        writer = self._ensure_writer() if on else self._writer
        if writer is not None:
            writer.set_recording(on)
        logger.info(f"Recording to {self._settings.capture_dir}" if on else "Recording stopped")
        s = self._settings
        if on and s.center_capture_fps <= 0 and s.effective_stereo_capture_fps() <= 0:
            logger.warning("Recording, but both capture rates are 0: no images will be written. "
                           "Set them in Settings → Capture.")

    def set_preview(self, vio: bool, color: bool) -> None:
        self._settings.preview_enabled = bool(vio)
        self._preview_vio, self._preview_color = bool(vio), bool(color)
        run = self._run
        if run is not None and run.source is not None:
            run.source.set_preview(vio, color)

    def autopilot_params(self) -> AutopilotParams:
        """A fresh client for the vehicle in the current settings (no I/O until it is used)."""
        return AutopilotParams(self._settings.mavlink2rest_http_url())

    def status(self) -> dict:
        """The statusUpdated payload; every snapshot it calls is thread-safe and non-blocking."""
        run = self._run
        return {
            "state": self._state,
            "source": run.source.stats() if run and run.source else None,
            "odometry": run.processor.health() if run else None,
            "transport": run.transport.stats() if run and run.transport else None,
            "capture": self._writer.snapshot() if self._writer is not None else None,
            "telemetry": run.telemetry.latest() if run and run.telemetry else None,
            "bandwidth": bandwidth.estimate(self._settings),
        }

    def wait_idle(self, timeout: float = 10.0) -> bool:
        """Blocks until background teardowns finish. For app exit and tests, not the GUI's event loop."""
        deadline = time.monotonic() + timeout
        for thread in list(self._teardowns):
            thread.join(max(0.0, deadline - time.monotonic()))
        return not any(t.is_alive() for t in self._teardowns)

    def shutdown(self, timeout: float = 10.0) -> None:
        """App exit: stop, wait (bounded) for the camera to be released, flush pending images and
        detach the log sink. Safe to call twice."""
        deadline = time.monotonic() + timeout
        self.stop()
        self._status_timer.stop()
        if not self.wait_idle(max(0.0, deadline - time.monotonic() - 1.0)):
            logger.warning("Still releasing the camera at exit")
        writer, self._writer = self._writer, None
        if writer is not None:
            writer.close(max(0.5, min(WRITER_CLOSE_TIMEOUT_S, deadline - time.monotonic())))
        # Writers replaced by a settings change may still be flushing; the interpreter's exit would
        # kill their daemon threads mid-backlog. Each close() gives up (and reports what it
        # abandoned) after WRITER_CLOSE_TIMEOUT_S, so this is bounded.
        for old, thread in self._closing_writers:
            thread.join(max(0.5, deadline - time.monotonic()))
            if thread.is_alive():
                logger.warning(f"Images for {old.snapshot().get('folder')} were still being written at exit")
        self._closing_writers = []
        if self._sink_id is not None:
            logger.remove(self._sink_id)
            self._sink_id = None

    # --- Run lifecycle --------------------------------------------------------------------------
    def _start_run(self) -> None:
        settings = copy.deepcopy(self._settings)
        run = _Run(settings)
        self._run = run
        self._preview_vio = settings.preview_enabled
        self._preview_color = settings.preview_enabled and settings.color_preview_enabled
        self._set_state(STATE_CONNECTING, "Starting…")
        if settings.center_capture_fps > 0 or settings.effective_stereo_capture_fps() > 0:
            self._ensure_writer()
        callbacks = SourceCallbacks(
            on_pose=lambda sample: self._on_pose(run, sample),
            on_preview=lambda frame: self._on_preview(run, frame),
            on_capture=lambda jpeg: self._on_capture(run, jpeg),
            on_state=lambda state, message: self._emit_safely(self._sourceState, run, state, message or ""),
            # Both sources also write every log line to loguru, which the engine's sink forwards to
            # logMessage; forwarding on_log as well would show each line twice
            on_log=lambda level, message: None,
        )
        try:
            if settings.source == "sim" and not is_loopback(transport_host(settings)):
                # Simulated motion must never reach a real autopilot's EKF: a dry run with the
                # vehicle in the water would make position hold chase a survey that isn't happening.
                # An ArduPilot SITL is fine, and proves itself with its SIM_* parameters. The check
                # is HTTP, so it runs off the GUI thread; deltas start once it passes.
                threading.Thread(target=self._connect_if_sitl, args=(run, settings),
                                 name="trackshot-sitl-check", daemon=True).start()
            else:
                run.transport = make_transport(settings)
                run.transport.start()
            run.telemetry = VehicleTelemetry(settings.mavlink2rest_http_url(), TELEMETRY_POLL_HZ)
            run.telemetry.start()
            run.source = self._source_factory(settings, callbacks)
            run.source.start()
        except Exception as e:  # noqa: BLE001 - any failure to start must reach the UI
            logger.exception("Couldn't start")
            self._run = None
            self._teardown(run)
            self._set_state(STATE_ERROR, f"Couldn't start: {e}")
            return
        self._configure_health_timer()
        link = (f"{run.transport.stats().kind} to {run.transport.stats().target}" if run.transport
                else "no MAVLink output")
        logger.info(f"Started: source {settings.source}, backend {settings.backend}, {link}")

    @staticmethod
    def _connect_if_sitl(run: _Run, settings: Settings) -> None:
        """Simulator only: open the MAVLink link to a non-loopback autopilot only if it is SITL.

        SIM_SPEEDUP exists only in ArduPilot's software-in-the-loop builds, so a real vehicle never
        passes. Anything else (unreachable, no heartbeat, parameter missing) keeps the link closed.
        """
        host = transport_host(settings)
        try:
            is_sitl = AutopilotParams(settings.mavlink2rest_http_url(), timeout=2.0).read(SITL_MARKER_PARAM) is not None
        except Exception as e:  # noqa: BLE001 - any failure means "not proven SITL"
            logger.warning(f"Simulator: couldn't check whether {host} is SITL ({e}); not sending VISION_POSITION_DELTA")
            return
        if not is_sitl:
            logger.warning(f"Simulator: {host} is not an ArduPilot SITL (no {SITL_MARKER_PARAM}); not sending "
                           "VISION_POSITION_DELTA. To dry-run the link, run tools/fake_autopilot.py and set the "
                           "vehicle address to 127.0.0.1:8088.")
            return
        if not run.active:
            return
        transport = make_transport(settings)
        transport.start()
        run.transport = transport
        if not run.active:
            # Teardown began while we connected and may have missed this transport
            transport.stop()
            return
        logger.info(f"Simulator: {host} is an ArduPilot SITL ({SITL_MARKER_PARAM} present); sending simulated "
                    f"VISION_POSITION_DELTA via {transport.stats().kind}")

    def _teardown(self, run: _Run) -> None:
        run.active = False
        self._health_timer.stop()
        thread = threading.Thread(target=self._teardown_worker, args=(run,), name="trackshot-teardown",
                                  daemon=True)
        self._teardowns = [t for t in self._teardowns if t.is_alive()] + [thread]
        self._teardowns_outstanding += 1
        thread.start()

    def _teardown_worker(self, run: _Run) -> None:
        # Source first so nothing new is produced, then the link it was feeding, then telemetry
        steps = (("source", lambda: run.source.stop(SOURCE_STOP_TIMEOUT_S)),
                 ("transport", lambda: run.transport.stop()),
                 ("telemetry", lambda: run.telemetry.stop()))
        for name, step in steps:
            if getattr(run, name) is None:
                continue
            try:
                step()
            except Exception:  # noqa: BLE001 - one part failing must not leave the others running
                logger.exception(f"Stopping the {name} failed")
        self._emit_safely(self._teardownDone)

    def _teardown_busy(self) -> bool:
        return self._teardowns_outstanding > 0

    @Slot()
    def _on_teardown_done(self) -> None:
        self._teardowns_outstanding = max(0, self._teardowns_outstanding - 1)
        if self._start_pending and self._run is None and not self._teardown_busy():
            self._start_pending = False
            self._start_run()

    def _set_state(self, state: str, message: str) -> None:
        self._state = state
        if state == STATE_ERROR:
            logger.error(message)
        else:
            logger.info(f"Engine state: {state} ({message})")
        self.stateChanged.emit(state, message)

    @Slot(object, str, str)
    def _on_source_state(self, run: _Run, state: str, message: str) -> None:
        if run is not self._run:
            return   # a previous run winding down
        if state in (STATE_ERROR, STATE_STOPPED):
            # The source ended on its own (camera lost, connect window over, simulation finished):
            # close the rest of the run but keep its message on screen
            self._run = None
            self._teardown(run)
        if state == STATE_STOPPED:
            message = message or "Stopped"
        self._set_state(state, message)

    # --- Source callbacks (source threads: quick, no Qt widgets) ---------------------------------
    def _on_pose(self, run: _Run, sample: PoseSample) -> None:
        if not run.active:
            return
        delta = run.processor.process(sample)
        if delta is not None and run.transport is not None:
            run.transport.send(*messages.vision_position_delta(delta))
        position, quaternion = apply_camera_rotation(sample.position, sample.quaternion, run.rotation)
        pose = (*position, *quaternion)
        if not all(math.isfinite(v) for v in pose):
            return
        run.last_pose, run.last_tracking = pose, sample.tracking
        if run.pose_throttle.ready(time.monotonic()):
            self._poses.put("pose", (sample, pose))

    def _on_preview(self, run: _Run, frame: PreviewFrame) -> None:
        if not run.active:
            return
        wanted = self._preview_color if frame.kind == "color" else self._preview_vio
        throttle = run.preview_throttles.get(frame.kind)
        if wanted and throttle is not None and throttle.ready(time.monotonic()):
            self._previews.put(frame.kind, frame)

    def _on_capture(self, run: _Run, jpeg: CapturedJpeg) -> None:
        writer = self._writer
        if not run.active or writer is None:
            return
        telemetry = run.telemetry.latest() if run.telemetry is not None else None
        writer.submit(jpeg, telemetry, run.last_pose, run.last_tracking)

    def _emit_safely(self, signal, *args) -> None:
        # Background threads can outlive the Engine's C++ object (e.g. at interpreter exit)
        try:
            signal.emit(*args)
        except RuntimeError:
            pass

    # --- GUI-thread delivery ----------------------------------------------------------------------
    @Slot()
    def _deliver_poses(self) -> None:
        item = self._poses.take().get("pose")
        if item is not None:
            self.poseUpdated.emit(item)

    @Slot()
    def _deliver_previews(self) -> None:
        for kind, frame in self._previews.take().items():
            if (self._preview_color if kind == "color" else self._preview_vio):
                self.previewUpdated.emit(frame)

    @Slot(str, str)
    def _forward_log(self, level: str, text: str) -> None:
        self.logMessage.emit(level, text)

    @Slot()
    def _emit_status(self) -> None:
        self.statusUpdated.emit(self.status())

    def _configure_health_timer(self) -> None:
        s = self._settings
        if s.send_health and s.health_rate_hz > 0:
            self._health_timer.start(max(50, int(1000.0 / s.health_rate_hz)))
        else:
            self._health_timer.stop()

    @Slot()
    def _send_health(self) -> None:
        """TS_* NAMED_VALUE_FLOATs for Cockpit, on a timer so a stalled VIO still reports 'lost'."""
        run = self._run
        if run is None or run.transport is None:
            return
        for msg_type, values in messages.health_values(run.processor.health(), run.time_boot_ms()):
            run.transport.send(msg_type, values)

    # --- Capture writer -------------------------------------------------------------------------
    def _ensure_writer(self) -> CaptureWriter:
        if self._writer is None:
            s = self._settings
            self._writer = CaptureWriter(s.capture_dir, s.camera_angle_deg, s.min_free_gb, s.source_tag)
            if self._recording:
                self._writer.set_recording(True)
        return self._writer

    def _replace_writer(self) -> None:
        """Pending images are flushed by the old writer on a background thread (which shutdown()
        waits for); new ones go to the new one."""
        old, self._writer = self._writer, None
        thread = threading.Thread(target=old.close, args=(WRITER_CLOSE_TIMEOUT_S,), name="capture-close",
                                  daemon=True)
        self._closing_writers = [(w, t) for w, t in self._closing_writers if t.is_alive()] + [(old, thread)]
        thread.start()
        self._ensure_writer()


def _make_log_sink(engine: Engine) -> Callable:
    """A loguru sink forwarding INFO and above to Engine.logMessage, from whatever thread logged.
    Holds the engine weakly so a forgotten sink can't keep a deleted Engine alive."""
    ref = weakref.ref(engine)

    def sink(message) -> None:
        engine = ref()
        if engine is None:
            return
        record = message.record
        text = record["message"]
        if record["exception"] is not None and record["exception"].value is not None:
            text += f": {record['exception'].value}"
        engine._emit_safely(engine._logReady, record["level"].name.lower(), text)

    return sink
