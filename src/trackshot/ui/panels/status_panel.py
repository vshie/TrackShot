"""Status cards: the numbers a pilot glances at, each coloured by whether it needs attention."""

from __future__ import annotations

from typing import Any, Optional
from urllib.parse import urlparse

from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from trackshot.ui.theme import NONE_TEXT, css, current_theme, fmt, value
from trackshot.vio.source import (STATE_CONNECTING, STATE_ERROR, STATE_IDLE, STATE_RUNNING,
                                  STATE_STOPPED)

STATE_LEVEL = {STATE_RUNNING: "ok", STATE_CONNECTING: "warn", STATE_ERROR: "bad",
               STATE_IDLE: "neutral", STATE_STOPPED: "neutral"}
LATENCY_WARN_MS = 100.0
LATENCY_BAD_MS = 200.0
CHIP_TEMP_WARN_C = 95.0      # the OAK's Myriad X throttles near 105 °C
CHIP_TEMP_BAD_C = 105.0
HEARTBEAT_STALE_S = 3.0


class Card(QFrame):
    """Title, one big value, and a few lines of detail."""

    def __init__(self, title: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("card")
        self.title = QLabel(title.upper())
        self.value = QLabel(NONE_TEXT)
        self.detail = QLabel()
        self.detail.setWordWrap(True)
        self.level = "neutral"
        big = self.value.font()
        big.setPointSizeF(big.pointSizeF() * 1.6)
        big.setBold(True)
        self.value.setFont(big)
        small = self.title.font()
        small.setPointSizeF(small.pointSizeF() * 0.8)
        small.setBold(True)
        self.title.setFont(small)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.detail)
        layout.addStretch(1)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        self.apply_theme()

    def set(self, text: str, level: str = "neutral", detail: str = "") -> None:
        self.value.setText(text)
        self.detail.setText(detail)
        self.detail.setVisible(bool(detail))
        if level != self.level:
            self.level = level
            self.apply_theme()

    def apply_theme(self) -> None:
        theme = current_theme(self.palette())
        self.setStyleSheet(
            f"QFrame#card {{ background: {css(theme.base)}; border: 1px solid {css(theme.border)};"
            f" border-left: 4px solid {css(theme.level(self.level))}; border-radius: 6px; }}")
        self.title.setStyleSheet(f"color: {css(theme.muted)};")
        self.value.setStyleSheet(f"color: {css(theme.level(self.level))};")
        self.detail.setStyleSheet(f"color: {css(theme.text)};")


class StatusPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.cards = {key: Card(title) for key, title in (
            ("state", "State"), ("fps", "VIO rate"), ("latency", "Latency"),
            ("viso_delay", "Suggested VISO_DELAY_MS"), ("tracking", "Tracking"), ("jumps", "Jumps"),
            ("mavlink", "MAVLink"), ("camera", "Camera"), ("vehicle", "Vehicle"))}
        self.setMinimumWidth(330)
        grid = QGridLayout(self)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setSpacing(6)
        for i, card in enumerate(self.cards.values()):
            grid.addWidget(card, i // 2, i % 2)
        grid.setRowStretch(len(self.cards) // 2 + 1, 1)
        self.set_state(STATE_IDLE, None)

    def apply_theme(self) -> None:
        for card in self.cards.values():
            card.apply_theme()

    def set_state(self, state: str, message: Optional[str]) -> None:
        self.cards["state"].set(state.capitalize(), STATE_LEVEL.get(state, "neutral"), message or "")

    def update_status(self, status: dict, expected_fps: Optional[float] = None) -> None:
        """`status` is the engine's statusUpdated dict; any member may be missing or None."""
        running = status.get("state") == STATE_RUNNING
        source = status.get("source")
        odo = status.get("odometry")
        transport = status.get("transport")
        telemetry = status.get("telemetry")

        fps = value(odo, "fps", value(source, "fps"))
        if fps is None or not running:
            fps_level = "neutral"
        elif fps <= 0:
            fps_level = "bad"
        elif expected_fps and fps < 0.8 * expected_fps:
            fps_level = "warn"
        else:
            fps_level = "ok"
        stalled = value(odo, "stalled", False)
        self.cards["fps"].set(fmt(fps, ".1f", "fps"), fps_level,
                              ("No pose for over a second" if stalled else "")
                              + (f"Target {expected_fps:g} fps" if expected_fps and not stalled else ""))

        latency = value(odo, "latency_ms")
        lat_level = ("neutral" if latency is None else "ok" if latency < LATENCY_WARN_MS
                     else "warn" if latency < LATENCY_BAD_MS else "bad")
        self.cards["latency"].set(fmt(latency, ".0f", "ms"), lat_level, "Capture → host, averaged")

        suggested = value(odo, "suggested_viso_delay_ms")
        self.cards["viso_delay"].set(fmt(suggested, "d", "ms"), "neutral" if suggested is None else "ok",
                                     "Set on the autopilot from the Autopilot panel")

        tracking = value(odo, "tracking", "unknown") or "unknown"
        age = value(odo, "last_pose_age_s")
        lost_for = value(odo, "lost_for_s")
        if tracking == "lost" and lost_for is not None:
            text = f"Lost {lost_for:.0f} s"
            detail = "Not sending. Point the camera at the seabed; RTAB-Map retries every few seconds, or use Restart VIO"
        elif tracking == "acquiring":
            text, detail = "Acquiring", "Not sending until the VIO tracks steadily"
        else:
            text = tracking.capitalize()
            detail = f"Last pose {fmt(age, '.1f', 's')} ago" if age is not None else ""
        losses = value(odo, "losses") or 0
        if losses:
            detail += f" · lost {losses}× this run"
        self.cards["tracking"].set(text, tracking if tracking != "unknown" else "neutral", detail)

        jumps = value(odo, "jumps")
        rejected = value(odo, "rejected_spans")
        deltas = value(odo, "deltas")
        self.cards["jumps"].set(fmt(jumps, "d"), "neutral" if jumps is None else "ok" if jumps == 0 else "warn",
                                f"Deltas computed {fmt(deltas, 'd')} · bad spans {fmt(rejected, 'd')}")

        self._update_mavlink(transport, telemetry, running)
        self._update_camera(source)
        self._update_vehicle(telemetry)

    def _update_mavlink(self, transport: Any, telemetry: Any, running: bool) -> None:
        card = self.cards["mavlink"]
        if transport is None:
            card.set("Not started", "neutral")
            return
        connected = bool(value(transport, "connected", False))
        hb_age = value(transport, "heartbeat_age_s", value(telemetry, "heartbeat_age_s"))
        if not connected:
            level = "bad" if running else "neutral"
        elif hb_age is not None and hb_age > HEARTBEAT_STALE_S:
            level = "warn"
        else:
            level = "ok"
        target = str(value(transport, "target", "") or "")
        # The full URL is long; the host is what the pilot needs to recognise (full URL in the tooltip)
        host = urlparse(target).netloc or target
        card.setToolTip(target)
        lines = [f"{value(transport, 'kind', '')} · {host}",
                 f"Sent {fmt(value(transport, 'sent'), 'd')} · dropped {fmt(value(transport, 'dropped'), 'd')}",
                 f"Heartbeat age {fmt(hb_age, '.1f', 's')}"]
        if value(transport, "last_error"):
            lines.append(f"Last error: {value(transport, 'last_error')}")
        card.set("Connected" if connected else "Disconnected", level, "\n".join(lines))

    def _update_camera(self, source: Any) -> None:
        card = self.cards["camera"]
        if source is None:
            card.set(NONE_TEXT, "neutral")
            return
        temp = value(source, "chip_temp_c")
        level = ("neutral" if temp is None else "ok" if temp < CHIP_TEMP_WARN_C
                 else "warn" if temp < CHIP_TEMP_BAD_C else "bad")
        lines = [f"IMU {value(source, 'imu') or NONE_TEXT}", f"Chip {fmt(temp, '.0f', '°C')}"]
        if value(source, "camera_id"):
            lines.insert(0, str(value(source, "camera_id")))
        if value(source, "backend"):
            lines.append(f"Backend {value(source, 'backend')}")
        card.set(value(source, "camera_name") or NONE_TEXT, level, "\n".join(lines))

    def _update_vehicle(self, telemetry: Any) -> None:
        card = self.cards["vehicle"]
        if not telemetry or not value(telemetry, "connected", False):
            card.set("No link", "neutral", "Vehicle telemetry from BlueOS")
            return
        armed = value(telemetry, "armed")
        text = "Armed" if armed else "Disarmed" if armed is not None else NONE_TEXT
        lines = [f"Mode {value(telemetry, 'mode') or NONE_TEXT}",
                 f"Depth {fmt(value(telemetry, 'depth_m'), '.2f', 'm')}",
                 f"Heading {fmt(value(telemetry, 'heading_deg'), '.0f', '°')}"]
        # Armed is shown in amber: parameters can't be changed and the pilot should know it's live
        card.set(text, "warn" if armed else "ok", "\n".join(lines))
