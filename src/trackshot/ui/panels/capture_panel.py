"""Survey capture: the Record toggle and what the CaptureWriter reports (CaptureWriter.snapshot())."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtCore import QUrl
from PySide6.QtGui import QAction, QDesktopServices
from PySide6.QtWidgets import QFormLayout, QHBoxLayout, QLabel, QPushButton, QToolButton, QVBoxLayout, QWidget

from trackshot.ui.theme import NONE_TEXT, css, current_theme, fmt, value

LOW_SPACE_FACTOR = 2.0   # warn when free space is under twice the stop threshold


class CapturePanel(QWidget):
    def __init__(self, record_action: QAction, parent: QWidget | None = None):
        super().__init__(parent)
        self._folder: Optional[str] = None
        self._min_free_gb = 2.0
        self._rates = (0.0, 0.0)   # (centre, stereo) images per second in the applied settings
        self._snap: Optional[dict] = None

        # Same QAction as the toolbar's Record toggle, so both always agree
        self.record_button = QToolButton()
        self.record_button.setDefaultAction(record_action)
        self.record_button.setMinimumWidth(110)
        self.state = QLabel()
        self.open_button = QPushButton("Open in Finder")
        self.open_button.clicked.connect(self.open_folder)

        self.folder = QLabel(NONE_TEXT)
        self.folder.setWordWrap(True)
        self.counts = QLabel(NONE_TEXT)
        self.free = QLabel(NONE_TEXT)
        self.last_file = QLabel(NONE_TEXT)
        self.last_file.setWordWrap(True)
        self.error = QLabel(NONE_TEXT)
        self.error.setWordWrap(True)

        top = QHBoxLayout()
        top.addWidget(self.record_button)
        top.addWidget(self.state, 1)
        top.addWidget(self.open_button)
        form = QFormLayout()
        form.addRow("Folder", self.folder)
        form.addRow("Images", self.counts)
        form.addRow("Free space", self.free)
        form.addRow("Last file", self.last_file)
        form.addRow("Errors", self.error)
        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(form)
        layout.addStretch(1)
        self.update_snapshot(None)

    def configure(self, capture_dir: str, min_free_gb: float, center_fps: float = 0.0,
                  stereo_fps: float = 0.0) -> None:
        """From the applied settings; the snapshot's own folder wins once recording starts.
        `stereo_fps` is the effective rate (capped at the VIO frame rate)."""
        self._folder = capture_dir
        self._min_free_gb = min_free_gb
        self._rates = (float(center_fps), float(stereo_fps))
        self.folder.setText(capture_dir)
        self.update_snapshot(self._snap)

    def apply_theme(self) -> None:
        self.update_snapshot(self._snap)

    def update_snapshot(self, snap: Optional[dict]) -> None:
        self._snap = snap
        theme = current_theme(self.palette())
        recording = bool(value(snap, "recording", False))
        if snap is None:
            self.state.setText("Capture starts with VIO; set the rates in Settings → Capture.")
            self.state.setStyleSheet(f"color: {css(theme.muted)};")
            return
        if recording and not any(rate > 0 for rate in self._rates):
            # Recording with nothing to record: say so, or the pilot surfaces with an empty folder
            self.state.setText("● Recording, but no images: both capture rates are 0. Set them in Settings → Capture.")
            self.state.setStyleSheet(f"color: {css(theme.warn)}; font-weight: 600;")
        else:
            self.state.setText("● Recording" if recording else "Not recording")
            self.state.setStyleSheet(f"color: {css(theme.bad if recording else theme.muted)}; font-weight: 600;")
        if value(snap, "folder"):
            self._folder = str(value(snap, "folder"))
            self.folder.setText(self._folder)
        counts = value(snap, "counts") or {}
        dropped = value(snap, "dropped", 0) or 0
        self.counts.setText(f"Centre {counts.get('center', 0):,} · stereo pairs {counts.get('stereo', 0):,}"
                            + (f" · dropped {dropped:,}" if dropped else ""))
        free_gb = value(snap, "free_gb")
        disk_full = bool(value(snap, "disk_full", False))
        level = ("bad" if disk_full else "warn" if free_gb is not None and free_gb < self._min_free_gb * LOW_SPACE_FACTOR
                 else "ok" if free_gb is not None else "neutral")
        self.free.setText(fmt(free_gb, ".1f", "GB") + ("  — disk full, recording stopped" if disk_full else ""))
        self.free.setStyleSheet(f"color: {css(theme.level(level))};")
        last = value(snap, "last_file")
        self.last_file.setText(Path(last).name if last else NONE_TEXT)
        self.last_file.setToolTip(str(last or ""))
        err = value(snap, "last_error")
        self.error.setText(str(err) if err else "None")
        self.error.setStyleSheet(f"color: {css(theme.bad if err else theme.text)};")

    def open_folder(self) -> None:
        if not self._folder:
            return
        path = Path(self._folder)
        # The folder may not exist until the first image; open its parent rather than failing silently
        while not path.exists() and path != path.parent:
            path = path.parent
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
