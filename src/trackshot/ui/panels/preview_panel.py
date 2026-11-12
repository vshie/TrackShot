"""Camera preview: the VIO image (with tracked features) and, optionally, the colour camera.

The VIO preview is the image the VIO node already consumes on this Mac, so it costs no tether
bandwidth; the panel says that, and shows the colour preview's estimated cost when it's on.
Frames arrive at ≤ 10 Hz through the engine's previewUpdated signal.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QLabel, QSplitter, QVBoxLayout, QWidget

from trackshot.ui.theme import css, current_theme
from trackshot.vio.source import PreviewFrame

STALE_FRAME_S = 2.0


class PreviewView(QWidget):
    """One image stream, aspect-locked, with an optional feature overlay."""

    def __init__(self, title: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.title = QLabel(title)
        bold = self.title.font()
        bold.setBold(True)
        self.title.setFont(bold)
        self.meta = QLabel("Waiting for frames…")
        self.graphics = pg.GraphicsLayoutWidget()
        self.view = self.graphics.addViewBox(lockAspect=True, invertY=True, enableMouse=False)
        self.view.setMenuEnabled(False)
        # row-major: numpy (H, W[, 3]) arrays display without a transpose
        self.image = pg.ImageItem(axisOrder="row-major")
        self.view.addItem(self.image)
        self.features = pg.ScatterPlotItem(size=6, symbol="o", pxMode=True)
        self.view.addItem(self.features)
        self.frames = 0
        self.last_frame: Optional[PreviewFrame] = None

        header = QHBoxLayout()
        header.addWidget(self.title)
        header.addStretch(1)
        header.addWidget(self.meta)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(header)
        layout.addWidget(self.graphics, 1)
        self.apply_theme()

    def apply_theme(self) -> None:
        theme = current_theme(self.palette())
        self.graphics.setBackground(theme.base)
        # Feature dots must read on any image: bright fill, dark outline
        self.features.setBrush(pg.mkBrush("#39ff14"))
        self.features.setPen(pg.mkPen("#0d1117", width=1))
        self.meta.setStyleSheet(f"color: {css(theme.muted)};")

    def show_frame(self, frame: PreviewFrame) -> None:
        image = np.asarray(frame.image)
        first = self.last_frame is None or self.last_frame.image.shape != image.shape
        self.image.setImage(image, autoLevels=False, levels=(0, 255))
        if frame.features is not None and len(frame.features):
            pts = np.asarray(frame.features, dtype=float)
            self.features.setData(pts[:, 0], pts[:, 1])
        else:
            self.features.setData([], [])
        if first:
            self.view.autoRange(padding=0)
        self.frames += 1
        self.last_frame = frame
        h, w = image.shape[:2]
        n_feat = 0 if frame.features is None else len(frame.features)
        self.meta.setText(f"{w}×{h}" + (f" · {n_feat} features" if frame.features is not None else ""))

    def clear(self, message: str) -> None:
        self.image.clear()
        self.features.setData([], [])
        self.last_frame = None
        self.meta.setText(message)


class PreviewPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.vio = PreviewView("VIO image")
        self.color = PreviewView("Colour camera")
        self.color.setVisible(False)
        self.note = QLabel()
        self.note.setWordWrap(True)
        self._enabled = True
        self._color_enabled = False
        self._color_mbit = 0.0

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self.vio)
        splitter.addWidget(self.color)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(splitter, 1)
        layout.addWidget(self.note)
        self._update_note()

    def apply_theme(self) -> None:
        self.vio.apply_theme()
        self.color.apply_theme()
        self._update_note()

    def configure(self, preview_enabled: bool, color_enabled: bool, color_mbit: float) -> None:
        """Called when settings are applied or the toolbar's Preview toggle changes."""
        self._enabled = preview_enabled
        self._color_enabled = preview_enabled and color_enabled
        self._color_mbit = color_mbit
        self.color.setVisible(self._color_enabled)
        if not preview_enabled:
            self.vio.clear("Preview off")
            self.color.clear("Preview off")
        self._update_note()

    def show_frame(self, frame: PreviewFrame) -> None:
        if not self._enabled:
            return   # a frame already in flight when the pilot switched preview off
        if frame.kind == "color":
            self.color.setVisible(True)
            self.color.show_frame(frame)
        else:
            self.vio.show_frame(frame)

    def check_stale(self, now: Optional[float] = None) -> None:
        """Marks a view whose frames stopped (camera stopped or link lost)."""
        now = now or time.time()
        for view in (self.vio, self.color):
            if view.last_frame is not None and now - view.last_frame.t_received > STALE_FRAME_S:
                view.meta.setText(f"No new frame for {now - view.last_frame.t_received:.0f} s")

    def _update_note(self) -> None:
        theme = current_theme(self.palette())
        text = "VIO preview — uses frames already on this Mac: 0 Mbit/s extra."
        if self._color_enabled:
            text += f"  Colour preview: ~{self._color_mbit:.1f} Mbit/s over the tether."
        elif not self._enabled:
            text = "Preview is off. Turn it on from the toolbar — the VIO preview costs no tether bandwidth."
        self.note.setText(text)
        self.note.setStyleSheet(f"color: {css(theme.muted)};")
