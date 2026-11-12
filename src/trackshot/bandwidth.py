"""Tether bandwidth estimate, from the BlueOS extension's `updateEstimate()` model.

Sizes per frame:
- RTAB-Map VIO: rectified image (1 B/px) + 16-bit depth (2 B/px) = 3 B/px
- Basalt VIO:   left + right images (1 B/px each)                = 2 B/px
- MJPEG from the OAK's encoder: ~0.25 B/px at quality 90, scaling with quality
- The VIO preview reuses frames VIO already receives: no extra tether traffic.

The budget is the tether (~80 Mbit/s usable), which also carries Cockpit's video.
"""

from __future__ import annotations

from dataclasses import dataclass

MJPEG_BYTES_PER_PX_AT_Q90 = 0.25
IMU_MBIT = 0.07          # 200 Hz accel+gyro packets, negligible but counted
WARN_FRACTION = 0.70
BAD_FRACTION = 0.90


def mjpeg_bytes_per_frame(width: int, height: int, quality: int) -> float:
    return width * height * MJPEG_BYTES_PER_PX_AT_Q90 * max(10, quality) / 90.0


def _mbit(bytes_per_s: float) -> float:
    return bytes_per_s * 8.0 / 1e6


@dataclass(frozen=True)
class BandwidthEstimate:
    vio_mbit: float
    vio_preview_mbit: float          # always 0: shown so the UI can say so
    color_preview_mbit: float
    center_capture_mbit: float
    stereo_capture_mbit: float
    video_mbit: float
    total_mbit: float
    budget_mbit: float
    stereo_capture_fps: float        # effective (capped at VIO fps)
    level: str                       # "ok", "warn", "bad"
    hint: str

    @property
    def camera_mbit(self) -> float:
        """Everything TrackShot itself pulls from the camera."""
        return self.total_mbit - self.video_mbit

    @property
    def fraction(self) -> float:
        return self.total_mbit / self.budget_mbit if self.budget_mbit > 0 else float("inf")


def estimate(s) -> BandwidthEstimate:
    """`s` is a trackshot.config.Settings (duck-typed so tests can pass a SimpleNamespace)."""
    vio_bpp = 3 if s.backend == "rtabmap" else 2
    vio = _mbit(vio_bpp * s.fps * s.width * s.height) + IMU_MBIT
    if s.source == "sim":
        vio = 0.0

    color = 0.0
    if s.preview_enabled and s.color_preview_enabled and s.source != "sim":
        color = _mbit(s.color_preview_fps * mjpeg_bytes_per_frame(
            s.color_preview_width, s.color_preview_height, s.color_preview_quality))

    center = 0.0
    if s.center_capture_fps > 0 and s.source != "sim":
        center = _mbit(s.center_capture_fps * mjpeg_bytes_per_frame(
            s.center_width, s.center_height, s.jpeg_quality))

    stereo_fps = min(max(s.stereo_capture_fps, 0.0), float(s.fps))
    stereo = 0.0
    if stereo_fps > 0 and s.source != "sim":
        stereo = _mbit(2 * stereo_fps * mjpeg_bytes_per_frame(s.mono_width, s.mono_height, s.jpeg_quality))

    video = max(0.0, s.video_allowance_mbit)
    total = vio + color + center + stereo + video
    budget = s.tether_budget_mbit
    frac = total / budget if budget > 0 else float("inf")
    if frac > BAD_FRACTION:
        level = "bad"
        hint = (f"~{total:.0f} of {budget:.0f} Mbit/s: too close to the tether's limit. "
                "Lower the VIO resolution or frame rate, the capture rates, or the JPEG quality.")
    elif frac > WARN_FRACTION:
        level = "warn"
        hint = f"~{total:.0f} of {budget:.0f} Mbit/s: little headroom left on the tether."
    else:
        level = "ok"
        hint = f"~{total:.0f} of {budget:.0f} Mbit/s, including {video:.0f} for Cockpit video."
    return BandwidthEstimate(
        vio_mbit=vio, vio_preview_mbit=0.0, color_preview_mbit=color,
        center_capture_mbit=center, stereo_capture_mbit=stereo, video_mbit=video,
        total_mbit=total, budget_mbit=budget, stereo_capture_fps=stereo_fps, level=level, hint=hint)
