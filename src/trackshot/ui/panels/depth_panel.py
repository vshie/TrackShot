"""Depth & altitude: the vehicle's pressure depth, TrackShot's stereo altitude and the rangefinder as
the autopilot sees it.

With "Send altitude as a rangefinder" on, TrackShot's altitude *is* the autopilot's rangefinder:
AUTO follows the seabed at the height of waypoints planned "Relative to terrain", and SURFTRAK
holds its target, from it. `depth_readouts()` is shared with the Cockpit widget
(trackshot.web.server), so the two always say the same thing.
"""

from __future__ import annotations

from typing import Any, Optional

from PySide6.QtWidgets import QGridLayout, QLabel, QVBoxLayout, QWidget

from trackshot.ui.theme import css, current_theme, value


def _m(v: Optional[float]) -> str:
    return "—" if v is None else f"{v:.2f} m"


def depth_readouts(status: dict) -> dict[str, Any]:
    """Display values from an engine status: depth, altitude (+ detail), rangefinder."""
    telemetry = status.get("telemetry") or {}
    alt = status.get("altitude")
    distance = value(alt, "distance_m")
    if alt is None:
        detail = "No stereo depth (RTAB-Map with the camera looking down provides it)"
    elif distance is None:
        detail = "No usable depth in the middle of the image"
    else:
        detail = f"quality {value(alt, 'quality')} %"
    if alt is not None:
        detail += (" · sent as the rangefinder" if value(alt, "sending")
                   else " · not sent (Settings → MAVLink, camera angle 90°)")
    return {
        "depth_m": value(telemetry, "depth_m"),
        "altitude_m": distance,
        "altitude_quality": value(alt, "quality"),
        "altitude_sending": bool(value(alt, "sending", False)),
        "altitude_detail": detail,
        "rangefinder_m": value(telemetry, "altitude_m"),
    }


class DepthPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        grid = QGridLayout()
        self.depth = QLabel("—")
        self.altitude = QLabel("—")
        self.altitude_detail = QLabel("")
        self.altitude_detail.setWordWrap(True)
        self.rangefinder = QLabel("—")
        big = self.font()
        big.setPointSizeF(big.pointSizeF() * 1.5)
        big.setBold(True)
        for label in (self.depth, self.altitude, self.rangefinder):
            label.setFont(big)
        rows = [("Depth (pressure sensor)", self.depth, None),
                ("Altitude (TrackShot stereo)", self.altitude, self.altitude_detail),
                ("Rangefinder (as the autopilot sees it)", self.rangefinder, None)]
        for row, (title, val, detail) in enumerate(rows):
            grid.addWidget(QLabel(title), row * 2, 0)
            grid.addWidget(val, row * 2, 1)
            if detail is not None:
                grid.addWidget(detail, row * 2 + 1, 0, 1, 2)
        layout = QVBoxLayout(self)
        layout.addLayout(grid)
        layout.addStretch(1)
        self.update_status({})

    def update_status(self, status: dict) -> None:
        r = depth_readouts(status)
        self.depth.setText(_m(r["depth_m"]))
        self.altitude.setText(_m(r["altitude_m"]))
        self.altitude_detail.setText(r["altitude_detail"])
        self.altitude_detail.setStyleSheet(f"color: {css(current_theme(self.palette()).muted)};")
        self.rangefinder.setText(_m(r["rangefinder_m"]))
