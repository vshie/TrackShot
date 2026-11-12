"""TrackShot settings: one dataclass, persisted as JSON in the user's config directory.

Every module reads its options from `Settings`; nothing else stores configuration.
Unknown keys in a saved file are ignored and missing keys take their defaults, so the file
survives version changes.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

from platformdirs import user_config_dir, user_documents_dir

APP_NAME = "TrackShot"

# The Pi's CPU forced Basalt down to 3 fps in the BlueOS extension; topside both run at 10.
# The tether, not the CPU, is now the limit (see bandwidth.py).
BACKEND_DEFAULT_FPS = {"rtabmap": 10, "basalt": 10}

SOURCES = ("oak", "sim")
SIM_PATTERNS = ("survey", "hover")
BACKENDS = ("rtabmap", "basalt")
TRANSPORTS = ("mavlink2rest", "udp")
# What the VIO is sent as (see trackshot.core.ned):
#   "position_speed": VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE, horizontal only; the
#                     autopilot keeps its depth sensor for everything vertical (needs
#                     EK3_SRC1_VELZ 0 and the vehicle's heading from telemetry)
#   "position_delta": VISION_POSITION_DELTA (0.1.x); ArduSub fuses all three axes of it
ODOMETRY_OUTPUTS = ("position_speed", "position_delta")
RTABMAP_WIDTH_MULTIPLE = 16


def config_path() -> Path:
    return Path(user_config_dir(APP_NAME, appauthor=False)) / "settings.json"


def default_capture_dir() -> str:
    return str(Path(user_documents_dir()) / "TrackShot Captures")


@dataclass
class Settings:
    # --- Source -----------------------------------------------------------------------------
    source: str = "oak"                     # "oak" (real camera) or "sim" (synthetic, no hardware)
    camera_ip: str = "192.168.2.118"        # empty = first camera found
    sim_pattern: str = "survey"             # simulator motion: "survey" (lawnmower) or "hover" (stationary)
    # --- VIO --------------------------------------------------------------------------------
    backend: str = "rtabmap"                # "rtabmap" or "basalt" (availability: capabilities.py)
    fps: int = BACKEND_DEFAULT_FPS["rtabmap"]
    width: int = 320
    height: int = 200
    camera_angle_deg: float = 0.0           # 0 = looking forward, 90 = straight down (top of image = bow)
    rtabmap_features: int = 500
    rtabmap_local_map_size: int = 1000      # OdomF2M/MaxSize
    basalt_grid_size: int = 80
    basalt_flow_iterations: int = 3
    basalt_flow_levels: int = 2
    basalt_max_states: int = 3
    basalt_max_kfs: int = 5
    basalt_vio_iterations: int = 5
    basalt_skip_frames: int = 1
    # --- MAVLink ----------------------------------------------------------------------------
    transport: str = "mavlink2rest"         # "mavlink2rest" (WebSocket via BlueOS) or "udp" (pymavlink)
    odometry_output: str = "position_speed"  # see ODOMETRY_OUTPUTS
    vehicle_ip: str = "192.168.2.2"
    udp_url: str = "udpout:192.168.2.2:14550"
    source_system: int = 1                  # same system as the autopilot, like an onboard companion
    source_component: int = 197             # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    send_health: bool = True                # NAMED_VALUE_FLOAT TS_* health values for Cockpit
    health_rate_hz: float = 1.0
    # --- Preview ----------------------------------------------------------------------------
    preview_enabled: bool = True            # shows the VIO image the Mac already receives: no tether cost
    color_preview_enabled: bool = False     # extra MJPEG stream from the centre camera
    color_preview_fps: float = 5.0
    color_preview_width: int = 640
    color_preview_height: int = 360
    color_preview_quality: int = 70
    # --- Survey capture ---------------------------------------------------------------------
    capture_dir: str = field(default_factory=default_capture_dir)
    center_capture_fps: float = 0.0         # 0 = off
    stereo_capture_fps: float = 0.0         # 0 = off; runs at min(this, fps)
    center_width: int = 1920
    center_height: int = 1080
    mono_width: int = 1280
    mono_height: int = 800
    jpeg_quality: int = 90
    min_free_gb: float = 2.0
    source_tag: str = "tl000000"
    # --- Bandwidth budget -------------------------------------------------------------------
    tether_budget_mbit: float = 80.0        # usable tether throughput
    video_allowance_mbit: float = 15.0      # Cockpit video already on the tether

    # ----------------------------------------------------------------------------------------
    def mavlink2rest_ws_url(self) -> str:
        return f"ws://{self.vehicle_ip}/mavlink2rest/ws/mavlink"

    def mavlink2rest_http_url(self) -> str:
        return f"http://{self.vehicle_ip}/mavlink2rest"

    def effective_stereo_capture_fps(self) -> float:
        """Stereo pairs come from the VIO sensors, so they can't run faster than VIO."""
        return min(max(self.stereo_capture_fps, 0.0), float(self.fps))

    def validate(self) -> list[str]:
        """Human-readable problems; empty when the settings can be used."""
        problems = []
        if self.source not in SOURCES:
            problems.append(f"Unknown source {self.source!r}")
        if self.sim_pattern not in SIM_PATTERNS:
            problems.append(f"Unknown simulator pattern {self.sim_pattern!r}")
        if self.backend not in BACKENDS:
            problems.append(f"Unknown VIO backend {self.backend!r}")
        if self.transport not in TRANSPORTS:
            problems.append(f"Unknown MAVLink transport {self.transport!r}")
        if self.odometry_output not in ODOMETRY_OUTPUTS:
            problems.append(f"Unknown odometry output {self.odometry_output!r}")
        if not 1 <= self.fps <= 60:
            problems.append("VIO frame rate must be 1–60 fps")
        if self.width < 64 or self.height < 64 or self.width > 1280 or self.height > 800:
            problems.append("VIO resolution must be between 64×64 and 1280×800")
        if self.backend == "rtabmap" and self.width % RTABMAP_WIDTH_MULTIPLE:
            # The OAK's stereo depth (which RTAB-Map needs) takes widths in steps of 16; 300×200
            # started the camera but RTAB-Map never produced a pose (field test, 2026-11-12)
            lower = self.width - self.width % RTABMAP_WIDTH_MULTIPLE
            problems.append(f"RTAB-Map needs a VIO width that is a multiple of {RTABMAP_WIDTH_MULTIPLE} "
                            f"(stereo depth on the OAK): use {lower} or {lower + RTABMAP_WIDTH_MULTIPLE}")
        if not -90.0 <= self.camera_angle_deg <= 90.0:
            problems.append("Camera angle must be between -90° and 90°")
        if self.center_capture_fps < 0 or self.stereo_capture_fps < 0:
            problems.append("Capture rates can't be negative")
        if not 10 <= self.jpeg_quality <= 100 or not 10 <= self.color_preview_quality <= 100:
            problems.append("JPEG quality must be 10–100")
        if self.tether_budget_mbit <= 0:
            problems.append("Tether budget must be positive")
        if not 1 <= self.source_system <= 255 or not 1 <= self.source_component <= 255:
            problems.append("MAVLink system and component IDs must be 1–255")
        return problems

    # --- Persistence ------------------------------------------------------------------------
    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "Settings":
        known = {f.name: f for f in fields(cls)}
        kwargs = {}
        defaults = cls()
        for name, value in (data or {}).items():
            if name not in known:
                continue
            default = getattr(defaults, name)
            try:
                if isinstance(default, bool):
                    value = bool(value)
                elif isinstance(default, int):
                    value = int(value)
                elif isinstance(default, float):
                    value = float(value)
                elif isinstance(default, str):
                    value = str(value)
            except (TypeError, ValueError):
                continue
            kwargs[name] = value
        settings = cls(**kwargs)
        # A file saved without a frame rate gets its backend's default
        if "fps" not in (data or {}):
            settings.fps = BACKEND_DEFAULT_FPS.get(settings.backend, settings.fps)
        return settings

    @classmethod
    def load(cls, path: Path | None = None) -> "Settings":
        path = path or config_path()
        try:
            with open(path) as f:
                return cls.from_dict(json.load(f))
        except (OSError, ValueError):
            return cls()

    def save(self, path: Path | None = None) -> None:
        path = path or config_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        with open(tmp, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)
