"""Colours and small display helpers shared by the panels.

TrackShot follows the system light/dark appearance instead of imposing its own palette, so the
pilot's Mac looks the same as every other app on it. The accent colours below come in a light and
a dark variant, each chosen for at least 4.5:1 contrast against that mode's window background
(they are GitHub Primer's semantic colours, which were designed for exactly this).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPalette, QPixmap
from PySide6.QtWidgets import QApplication

NONE_TEXT = "—"


@dataclass(frozen=True)
class Theme:
    dark: bool
    text: QColor
    muted: QColor
    base: QColor             # background of plots, cards and text views
    border: QColor
    ok: QColor
    warn: QColor
    bad: QColor
    info: QColor
    neutral: QColor
    # Categorical colours for the bandwidth bar segments and plot series
    vio: QColor
    color_preview: QColor
    capture: QColor
    video: QColor

    def level(self, level: str | None) -> QColor:
        """Colour for a status level: ok / warn / bad (anything else is neutral)."""
        return {"ok": self.ok, "good": self.ok, "warn": self.warn, "degraded": self.warn,
                "bad": self.bad, "lost": self.bad, "error": self.bad}.get(level or "", self.neutral)


_LIGHT = dict(ok="#1a7f37", warn="#9a6700", bad="#cf222e", info="#0969da", neutral="#57606a",
              vio="#0969da", color_preview="#8250df", capture="#1b7c83", video="#6e7781")
_DARK = dict(ok="#3fb950", warn="#d29922", bad="#f85149", info="#58a6ff", neutral="#8b949e",
             vio="#58a6ff", color_preview="#bc8cff", capture="#39c5cf", video="#8b949e")


def is_dark(palette: QPalette | None = None) -> bool:
    palette = palette or QApplication.palette()
    return palette.color(QPalette.ColorRole.Window).lightness() < 128


def current_theme(palette: QPalette | None = None) -> Theme:
    palette = palette or QApplication.palette()
    dark = is_dark(palette)
    accents = {k: QColor(v) for k, v in (_DARK if dark else _LIGHT).items()}
    return Theme(
        dark=dark,
        text=palette.color(QPalette.ColorRole.WindowText),
        # Not the palette's disabled text: that's deliberately too faint to read
        muted=QColor("#9da7b3" if dark else "#57606a"),
        base=palette.color(QPalette.ColorRole.Base),
        border=palette.color(QPalette.ColorRole.Mid),
        **accents,
    )


def value(obj: Any, name: str, default: Any = None) -> Any:
    """Reads `name` from a dataclass/object or a dict.

    The status dict's members come from modules written independently (OdometryHealth may arrive
    as a dataclass or a plain dict, capture/telemetry are dicts), so every panel reads through
    this instead of assuming a shape.
    """
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def fmt(v: Any, spec: str = "", unit: str = "") -> str:
    """Formats a possibly-missing number: None → an em dash, otherwise `v` with `spec` and `unit`."""
    if v is None:
        return NONE_TEXT
    try:
        text = format(v, spec)
    except (TypeError, ValueError):
        text = str(v)
    return f"{text} {unit}".rstrip() if unit else text


def css(color: QColor) -> str:
    return color.name(QColor.NameFormat.HexRgb)


def dot_icon(color: QColor, size: int = 14) -> QIcon:
    """A filled circle, e.g. the Record button's red dot."""
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(color)
    painter.drawEllipse(1, 1, size - 2, size - 2)
    painter.end()
    return QIcon(pixmap)
