"""The contract between a pose source (real OAK or simulator) and the engine.

A PoseSource runs on its own thread(s) and reports through the callbacks in `SourceCallbacks`.
Callbacks are called from the source's threads: they must be quick and must not touch Qt
widgets (the engine marshals to the GUI thread).

Clock convention: every time in these messages is **host wall-clock** (Unix epoch). A source
whose device has its own clock (depthai's steady clock) converts before reporting, as
    t_wall = time.time() - (dai.Clock.now() - frame.getTimestamp()).total_seconds()
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np

# Source lifecycle states, reported through SourceCallbacks.on_state
STATE_IDLE = "idle"
STATE_CONNECTING = "connecting"
STATE_RUNNING = "running"
STATE_ERROR = "error"
STATE_STOPPED = "stopped"


@dataclass(frozen=True)
class PoseSample:
    """One VIO pose, in the VIO's world frame as the backend reports it (x forward, y left, z up)."""
    t_capture_us: Optional[int]                  # wall-clock µs the frame behind this pose was captured; None if unknown
    t_received: float                            # wall-clock s the pose reached the host
    position: tuple[float, float, float]         # metres
    quaternion: tuple[float, float, float, float]  # (qw, qx, qy, qz), body→world
    tracking: Optional[bool] = None              # backend's own tracking flag if it has one


@dataclass(frozen=True)
class RangeSample:
    """Distance along the camera's optical axis to what it sees (the seabed, looking down)."""
    t: float                                     # wall-clock s of the depth frame
    distance_m: Optional[float]                  # None: no usable depth
    quality: int                                 # 1–100
    valid_fraction: float


@dataclass
class PreviewFrame:
    kind: str                                    # "vio" (grey VIO image) or "color" (centre camera)
    image: np.ndarray                            # (H, W) uint8 grey or (H, W, 3) uint8 RGB
    t_received: float
    features: Optional[np.ndarray] = None        # (N, 2) float pixel positions, VIO only


@dataclass
class CapturedJpeg:
    """A survey image already JPEG-encoded on the camera."""
    stream: str                                  # "center", "left" or "right"
    data: bytes
    t_capture: float                             # wall-clock s
    sequence: int                                # device sequence number (pairs left/right)
    lens: Any = None                             # trackshot.capture.meta.Lens or None


@dataclass
class SourceStats:
    fps: float = 0.0                             # poses per second over the last ~2 s
    rx_mbit: Optional[float] = None              # measured camera→host throughput, if known
    chip_temp_c: Optional[float] = None
    camera_name: Optional[str] = None
    camera_id: Optional[str] = None
    imu: Optional[str] = None
    backend: Optional[str] = None
    extra: dict = field(default_factory=dict)


@dataclass
class SourceCallbacks:
    on_pose: Callable[[PoseSample], None] = lambda s: None
    on_preview: Callable[[PreviewFrame], None] = lambda f: None
    on_capture: Callable[[CapturedJpeg], None] = lambda j: None
    on_range: Callable[[RangeSample], None] = lambda r: None                       # ~10 Hz, if the source has depth
    on_state: Callable[[str, Optional[str]], None] = lambda state, message: None   # (STATE_*, human message)
    on_log: Callable[[str, str], None] = lambda level, message: None               # ("info"|"warning"|"error", text)


class PoseSource(ABC):
    """Base class. start() returns at once; the work happens on threads the source owns."""

    def __init__(self, settings, callbacks: SourceCallbacks):
        self.settings = settings
        self.callbacks = callbacks
        self._stop_event = threading.Event()

    @abstractmethod
    def start(self) -> None:
        """Begin connecting and streaming. Non-blocking. Reports progress via on_state."""

    @abstractmethod
    def stop(self, timeout: float = 5.0) -> None:
        """Stop and release the device. Blocks up to `timeout`. Safe to call twice or before start."""

    @abstractmethod
    def is_running(self) -> bool: ...

    @abstractmethod
    def stats(self) -> SourceStats:
        """Thread-safe snapshot."""

    def set_preview(self, vio: bool, color: bool) -> None:
        """Turn preview delivery on/off without restarting. Turning on the colour preview may
        need a restart (it's a separate camera stream); a source may ignore it until then."""
