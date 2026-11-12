"""TrackShot's main window: toolbar, settings form, status cards, trajectory, preview and tool docks.

The window only talks to the Engine through its documented interface (signals in, method calls
out), so it runs the same against the real engine and tests' FakeEngine. Everything the engine
delivers arrives through Qt signals on the GUI thread; the only blocking work the UI itself starts
(autopilot HTTP, camera discovery) runs on `self.pool`.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Optional

from loguru import logger
from PySide6.QtCore import QEvent, QSettings, Qt, QThreadPool
from PySide6.QtGui import QAction, QCloseEvent, QGuiApplication
from PySide6.QtWidgets import (QComboBox, QDockWidget, QFrame, QLabel, QMainWindow, QScrollArea, QSizePolicy,
                               QSplitter, QStyle, QToolBar, QWidget)

from trackshot import bandwidth
from trackshot.config import Settings
from trackshot.ui.panels.autopilot_panel import AutopilotPanel
from trackshot.ui.panels.bandwidth_panel import BandwidthPanel
from trackshot.ui.panels.capture_panel import CapturePanel
from trackshot.ui.panels.log_panel import LogPanel
from trackshot.ui.panels.preview_panel import PreviewPanel
from trackshot.ui.panels.settings_panel import SettingsPanel
from trackshot.ui.panels.status_panel import STATE_LEVEL, StatusPanel
from trackshot.ui.panels.trajectory_panel import TrajectoryPanel
from trackshot.ui.theme import css, current_theme, dot_icon, value
from trackshot.vio.source import STATE_CONNECTING, STATE_IDLE, STATE_RUNNING

SOURCE_CHOICES = (("OAK camera", "oak"), ("Simulator", "sim"))
# Fits a 13-inch MacBook with the Dock shown (1440×830 available) with room to spare
MIN_WINDOW_SIZE = (1280, 720)
# Wide enough for two columns of status cards with their longest texts
STATUS_DOCK_WIDTH = 360
ACTIVE_STATES = (STATE_CONNECTING, STATE_RUNNING)


def window_settings() -> QSettings:
    """Where window geometry lives (separate from the app's Settings JSON: it's per-screen layout).
    Uses QSettings' default format so tests can redirect it to a temporary INI file."""
    return QSettings(QSettings.defaultFormat(), QSettings.Scope.UserScope, "TrackShot", "TrackShot")


class MainWindow(QMainWindow):
    def __init__(self, engine, settings_path: Optional[Path] = None, parent: QWidget | None = None,
                 saved_settings: Optional[Settings] = None):
        """`saved_settings`: the settings as on disk, when the engine's carry session-only overrides
        (--sim, --vehicle-ip). Toolbar toggles save onto these, so the overrides stay unsaved
        until the user applies the settings form."""
        super().__init__(parent)
        self.engine = engine
        self.settings_path = settings_path
        self._applied: Settings = copy.deepcopy(engine.settings)
        self._saved: Settings = copy.deepcopy(saved_settings if saved_settings is not None else self._applied)
        self._state = STATE_IDLE
        self._built = False   # palette-change events can arrive before the panels exist
        # Background work started by the UI: autopilot HTTP and camera discovery, never more than a couple
        self.pool = QThreadPool(self)
        self.pool.setMaxThreadCount(2)

        self.setWindowTitle("TrackShot")
        self.setMinimumSize(*MIN_WINDOW_SIZE)
        self.setDockOptions(QMainWindow.DockOption.AnimatedDocks | QMainWindow.DockOption.AllowTabbedDocks)

        self._build_actions()
        self.settings_panel = SettingsPanel(self._applied, self.pool)
        self.status_panel = StatusPanel()
        self.trajectory = TrajectoryPanel()
        self.preview = PreviewPanel()
        self.bandwidth = BandwidthPanel()
        self.autopilot = AutopilotPanel(engine, self.pool)
        # Closing while it writes autopilot parameters waits for the writes (see closeEvent)
        self._close_after_writes = False
        self.autopilot.writesFinished.connect(self._on_autopilot_writes_finished)
        self.capture = CapturePanel(self.record_action)
        self.log = LogPanel()
        self._build_toolbar()
        self._build_layout()

        self.settings_panel.pendingChanged.connect(self._on_pending_changed)
        self.settings_panel.applyRequested.connect(self.apply_settings)
        engine.stateChanged.connect(self._on_state)
        engine.statusUpdated.connect(self._on_status)
        engine.poseUpdated.connect(self.trajectory.add_pose)
        engine.previewUpdated.connect(self.preview.show_frame)
        engine.logMessage.connect(self.log.append)
        QGuiApplication.styleHints().colorSchemeChanged.connect(self.apply_theme)

        self._sync_from_applied()
        self._on_pending_changed(self.settings_panel.pending(), self.settings_panel.estimate, False)
        self._on_state(getattr(engine, "state", STATE_IDLE) or STATE_IDLE, None)
        self._restore_window()
        self._built = True

    # --- Construction ---------------------------------------------------------------------------
    def _build_actions(self) -> None:
        style = self.style()
        self.start_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_MediaPlay), "Start", self)
        self.start_action.setShortcut("Ctrl+R")
        self.start_action.setToolTip("Connect to the source and start sending VISION_POSITION_DELTA (⌘R)")
        self.start_action.triggered.connect(self.engine.start)
        self.stop_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_MediaStop), "Stop", self)
        self.stop_action.setShortcut("Ctrl+.")
        self.stop_action.setToolTip("Stop VIO and release the camera (⌘.)")
        self.stop_action.triggered.connect(self.engine.stop)
        self.restart_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_BrowserReload), "Restart VIO", self)
        self.restart_action.setShortcut("Ctrl+Shift+R")
        self.restart_action.triggered.connect(self.engine.restart_vio)
        self._restart_tooltip(None)
        self.restart_action.setEnabled(False)
        self.preview_action = QAction("Preview", self)
        self.preview_action.setCheckable(True)
        self.preview_action.setToolTip("Show the camera preview. The VIO preview costs no tether bandwidth.")
        self.preview_action.toggled.connect(self._on_preview_toggled)
        self.record_action = QAction(dot_icon(current_theme().bad), "Record", self)
        self.record_action.setCheckable(True)
        self.record_action.setToolTip("Write survey images (rates in Settings → Capture)")
        self.record_action.toggled.connect(self._on_record_toggled)

    def _build_toolbar(self) -> None:
        bar = QToolBar("Main")
        bar.setObjectName("main_toolbar")
        bar.setMovable(False)
        bar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        bar.addAction(self.start_action)
        bar.addAction(self.stop_action)
        bar.addAction(self.restart_action)
        bar.addSeparator()
        bar.addWidget(QLabel(" Source "))
        self.source_combo = QComboBox()
        for label, data in SOURCE_CHOICES:
            self.source_combo.addItem(label, data)
        self.source_combo.setToolTip("Simulator runs the whole app without a camera")
        self.source_combo.currentIndexChanged.connect(self._on_source_selected)
        bar.addWidget(self.source_combo)
        bar.addSeparator()
        bar.addAction(self.preview_action)
        bar.addAction(self.record_action)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(spacer)
        self.state_badge = QLabel()
        bar.addWidget(self.state_badge)
        self.addToolBar(bar)

    def _dock(self, title: str, name: str, widget: QWidget, area: Qt.DockWidgetArea) -> QDockWidget:
        dock = QDockWidget(title, self)
        dock.setObjectName(name)   # saveState() needs stable names
        dock.setWidget(widget)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable
                         | QDockWidget.DockWidgetFeature.DockWidgetFloatable
                         | QDockWidget.DockWidgetFeature.DockWidgetClosable)
        self.addDockWidget(area, dock)
        return dock

    @staticmethod
    def _scrollable(widget: QWidget, min_width: int) -> QScrollArea:
        """`widget` in a scroll area that may be shorter than it (it then scrolls), at least `min_width` wide."""
        area = QScrollArea()
        area.setWidget(widget)
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.Shape.NoFrame)
        area.setMinimumWidth(min_width)
        return area

    def _build_layout(self) -> None:
        central = QSplitter(Qt.Orientation.Horizontal)
        central.addWidget(self.trajectory)
        central.addWidget(self.preview)
        central.setStretchFactor(0, 3)
        central.setStretchFactor(1, 2)
        central.setSizes([600, 380])
        self.setCentralWidget(central)

        left, right, bottom = (Qt.DockWidgetArea.LeftDockWidgetArea, Qt.DockWidgetArea.RightDockWidgetArea,
                               Qt.DockWidgetArea.BottomDockWidgetArea)
        self.docks = {
            "settings": self._dock("Settings", "dock_settings", self.settings_panel, left),
            # Scrollable, so its cards never force the dock taller than the window leaves room for
            # (QMainWindow would otherwise draw it over the bottom docks)
            "status": self._dock("Status", "dock_status", self._scrollable(self.status_panel, STATUS_DOCK_WIDTH), right),
            "bandwidth": self._dock("Bandwidth", "dock_bandwidth", self.bandwidth, bottom),
            "autopilot": self._dock("Autopilot", "dock_autopilot", self.autopilot, bottom),
            "capture": self._dock("Capture", "dock_capture", self.capture, bottom),
            "log": self._dock("Log", "dock_log", self.log, bottom),
        }
        self.tabifyDockWidget(self.docks["bandwidth"], self.docks["autopilot"])
        self.tabifyDockWidget(self.docks["autopilot"], self.docks["capture"])
        self.tabifyDockWidget(self.docks["capture"], self.docks["log"])
        self.docks["bandwidth"].raise_()
        self.resizeDocks([self.docks["settings"], self.docks["status"]], [370, STATUS_DOCK_WIDTH],
                         Qt.Orientation.Horizontal)
        self.resizeDocks([self.docks["bandwidth"]], [200], Qt.Orientation.Vertical)

        view_menu = self.menuBar().addMenu("View")
        for dock in self.docks.values():
            view_menu.addAction(dock.toggleViewAction())

    # --- Settings -------------------------------------------------------------------------------
    def apply_settings(self, settings: Settings) -> None:
        """Saves `settings` and hands a copy to the engine (which restarts if it has to)."""
        try:
            settings.save(self.settings_path)
        except OSError as e:
            self.log.append("error", f"Couldn't save settings: {e}")
            self.statusBar().showMessage(f"Couldn't save settings: {e}", 10_000)
            return
        self._applied = copy.deepcopy(settings)
        self._saved = copy.deepcopy(settings)   # applying the form saves everything, overrides included
        self.engine.apply_settings(copy.deepcopy(settings))
        self.settings_panel.load(settings)
        self._sync_from_applied()
        self.statusBar().showMessage("Settings applied", 4000)

    def _save_fields(self, **changes) -> None:
        """Saves just these fields onto the settings as saved, leaving session overrides unsaved."""
        for name, v in changes.items():
            setattr(self._saved, name, v)
        try:
            self._saved.save(self.settings_path)
        except OSError as e:
            self.log.append("error", f"Couldn't save settings: {e}")

    def _apply_toolbar_change(self, **changes) -> None:
        """A toolbar choice (source) applies at once without taking the form's other edits along."""
        s = copy.deepcopy(self._applied)
        for name, v in changes.items():
            setattr(s, name, v)
        self._save_fields(**changes)
        self._applied = s
        self.engine.apply_settings(copy.deepcopy(s))
        self.settings_panel.update_base(**changes)
        self._sync_from_applied()

    def _sync_from_applied(self) -> None:
        s = self._applied
        self.source_combo.blockSignals(True)
        self.source_combo.setCurrentIndex(max(0, self.source_combo.findData(s.source)))
        self.source_combo.blockSignals(False)
        self.preview_action.blockSignals(True)
        self.preview_action.setChecked(s.preview_enabled)
        self.preview_action.blockSignals(False)
        est = bandwidth.estimate(s)
        self.preview.configure(s.preview_enabled, s.color_preview_enabled, est.color_preview_mbit)
        self.capture.configure(s.capture_dir, s.min_free_gb, s.center_capture_fps, s.effective_stereo_capture_fps())
        self.autopilot.set_vehicle(s.vehicle_ip)

    def _on_pending_changed(self, pending: Settings, est: bandwidth.BandwidthEstimate, dirty: bool) -> None:
        self.bandwidth.set_estimate(est, pending=dirty)

    # --- Toolbar --------------------------------------------------------------------------------
    def _on_source_selected(self, index: int) -> None:
        source = self.source_combo.itemData(index)
        if source and source != self._applied.source:
            self._apply_toolbar_change(source=source)

    def _on_preview_toggled(self, on: bool) -> None:
        # A runtime toggle: the engine switches preview delivery without restarting VIO
        self._applied.preview_enabled = on
        self._save_fields(preview_enabled=on)
        self.engine.set_preview(on, on and self._applied.color_preview_enabled)
        self.settings_panel.update_base(preview_enabled=on)
        self.preview.configure(on, self._applied.color_preview_enabled,
                               bandwidth.estimate(self._applied).color_preview_mbit)

    def _on_record_toggled(self, on: bool) -> None:
        self.engine.set_recording(on)

    # --- Engine signals -------------------------------------------------------------------------
    def _on_state(self, state: str, message: Optional[str]) -> None:
        active = state in ACTIVE_STATES
        self.start_action.setEnabled(not active)
        self.stop_action.setEnabled(active)
        self._update_restart_action()
        self.status_panel.set_state(state, message)
        self._state = state
        self._paint_badge()
        if message:
            self.statusBar().showMessage(message, 0 if state == "error" else 8000)

    def _paint_badge(self) -> None:
        theme = current_theme(self.palette())
        colour = theme.level(STATE_LEVEL.get(self._state, "neutral"))
        self.state_badge.setText(f"  {self._state.upper()}  ")
        self.state_badge.setStyleSheet(
            f"color: {css(colour)}; border: 1.5px solid {css(colour)}; border-radius: 9px;"
            " padding: 2px 6px; font-weight: 700;")

    def _restart_tooltip(self, blocked: Optional[str]) -> None:
        tip = ("Reconnect the camera and start the VIO afresh (⌘⇧R), e.g. once the camera sees the "
               "seabed. RTAB-Map already resets itself after a few seconds lost.")
        self.restart_action.setToolTip(tip if blocked is None else f"{tip}\nNot available: {blocked}")

    def _update_restart_action(self) -> None:
        blocked = self.engine.restart_vio_blocked()
        if self.restart_action.isEnabled() != (blocked is None) or blocked is not None:
            self.restart_action.setEnabled(blocked is None)
            self._restart_tooltip(blocked)

    def _on_status(self, status: dict) -> None:
        self._update_restart_action()
        self.status_panel.update_status(status, expected_fps=float(self._applied.fps))
        source = status.get("source")
        self.bandwidth.set_measured(value(source, "rx_mbit"))
        snap = status.get("capture")
        self.capture.update_snapshot(snap)
        recording = bool(value(snap, "recording", False))
        if recording != self.record_action.isChecked():
            self.record_action.blockSignals(True)
            self.record_action.setChecked(recording)
            self.record_action.blockSignals(False)
        telemetry = status.get("telemetry")
        if value(telemetry, "connected", False):
            self.trajectory.add_vehicle_depth(value(telemetry, "depth_m"))
        self.autopilot.note_telemetry(telemetry)
        self.autopilot.set_suggested_delay(value(status.get("odometry"), "suggested_viso_delay_ms"))
        self.preview.check_stale()

    # --- Window ---------------------------------------------------------------------------------
    def apply_theme(self, *_) -> None:
        if not self._built:
            return
        for panel in (self.settings_panel, self.status_panel, self.trajectory, self.preview, self.bandwidth,
                      self.autopilot, self.capture, self.log):
            panel.apply_theme()
        self._paint_badge()
        self.record_action.setIcon(dot_icon(current_theme(self.palette()).bad))

    def changeEvent(self, event: QEvent) -> None:  # noqa: N802 (Qt API)
        if event.type() in (QEvent.Type.PaletteChange, QEvent.Type.ApplicationPaletteChange):
            self.apply_theme()
        super().changeEvent(event)

    def _restore_window(self) -> None:
        qs = window_settings()
        geometry = qs.value("window/geometry")
        state = qs.value("window/state")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1500, 950)
        if state is not None:
            self.restoreState(state)

    def _on_autopilot_writes_finished(self) -> None:
        if self._close_after_writes:
            self.close()

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (Qt API)
        if self.autopilot.writing:
            # An apply or revert must not be killed halfway (main() abandons background tasks
            # after a few seconds): keep the window until it reports back, then close
            self._close_after_writes = True
            self.autopilot.cancel_background()   # no re-check or dialog once it finishes
            self.autopilot.show_closing_notice()
            self.statusBar().showMessage("Finishing the autopilot parameter writes: TrackShot closes "
                                         "when they are done.", 0)
            event.ignore()
            return
        qs = window_settings()
        qs.setValue("window/geometry", self.saveGeometry())
        qs.setValue("window/state", self.saveState())
        qs.sync()
        self.pool.clear()   # drop queued (not yet started) background tasks…
        self.autopilot.cancel_background()   # …and cut a running check short (main() waits for the pool)
        try:
            self.engine.stop()
        except Exception as e:   # closing must always succeed
            logger.error(f"Engine stop failed while closing: {e}")
        super().closeEvent(event)
