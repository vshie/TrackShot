"""Settings form.

The form edits a *pending* copy of the applied Settings. Every edit re-validates it
(Settings.validate(), shown inline) and re-estimates the tether bandwidth, so the pilot sees the
cost of a change before applying it. Apply emits `applyRequested(Settings)`; the main window saves
and hands the copy to the engine. Revert reloads the applied settings.

Fields the toolbar owns (source, preview on/off) are updated through `update_base()` so a toolbar
click never discards edits in progress.
"""

from __future__ import annotations

import copy
from dataclasses import fields as dataclass_fields
from typing import Any, Callable, Optional

from loguru import logger
from PySide6.QtCore import Qt, QThreadPool, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea, QSpinBox,
                               QVBoxLayout, QWidget)

from trackshot import bandwidth
from trackshot.config import BACKEND_DEFAULT_FPS, BACKENDS, Settings
from trackshot.ui.panels.bandwidth_panel import BandwidthBar
from trackshot.ui.theme import css, current_theme
from trackshot.ui.workers import run_in_pool

BACKEND_LABELS = {
    "rtabmap": "RTAB-Map (depth + features on the OAK)",
    "basalt": "Basalt (stereo VIO on this Mac)",
}
NOT_AVAILABLE_TIP = "not available in this depthai build"
CENTER_RESOLUTIONS = [(4056, 3040, "4056 × 3040 (12 MP)"), (3840, 2160, "3840 × 2160 (4K)"),
                      (1920, 1080, "1920 × 1080")]
MONO_RESOLUTIONS = [(1280, 800, "1280 × 800"), (640, 400, "640 × 400")]
COLOR_PREVIEW_SIZES = [(320, 180, "320 × 180"), (640, 360, "640 × 360"), (960, 540, "960 × 540"),
                       (1280, 720, "1280 × 720")]
RTABMAP_MIN_GOOD_FPS = 8


def available_backends() -> Optional[list[str]]:
    """Backends this depthai build has, or None if that can't be determined.

    trackshot.capabilities imports depthai; it is imported here, lazily, so the UI still opens
    (with every backend enabled) if that module or depthai is missing.
    """
    try:
        from trackshot import capabilities
        return list(capabilities.available_backends())
    except Exception as e:  # ImportError, or depthai failing to load
        logger.warning(f"Can't tell which VIO backends are available: {e}")
        return None


def list_cameras() -> list[dict]:
    """Blocking camera discovery; runs on a pool thread."""
    try:
        from trackshot import capabilities
    except ImportError as e:
        raise RuntimeError("Camera discovery isn't available in this build") from e
    return list(capabilities.list_cameras())


class _Binding:
    """How one form widget maps to Settings fields."""

    def __init__(self, read: Callable[[], dict], write: Callable[[Settings], None]):
        self.read = read
        self.write = write


class SettingsPanel(QWidget):
    pendingChanged = Signal(object, object, bool)   # (pending Settings, BandwidthEstimate, dirty)
    applyRequested = Signal(object)                 # Settings to apply

    def __init__(self, settings: Settings, pool: QThreadPool, parent: QWidget | None = None):
        super().__init__(parent)
        self._pool = pool
        self._base = copy.deepcopy(settings)
        self._bindings: list[_Binding] = []
        # Building widgets fires their change signals; ignore those until load() fills the form
        self._loading = True
        self._available = available_backends()
        self.estimate: Optional[bandwidth.BandwidthEstimate] = None

        form_host = QWidget()
        form_layout = QVBoxLayout(form_host)
        form_layout.setContentsMargins(6, 6, 6, 6)
        for group in (self._source_group(), self._vio_group(), self._mavlink_group(), self._preview_group(),
                      self._capture_group(), self._bandwidth_group()):
            form_layout.addWidget(group)
        form_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setWidget(form_host)

        # Footer: always visible under the scrolling form
        self.estimate_label = QLabel()
        self.estimate_bar = BandwidthBar(compact=True)
        self.problems_label = QLabel()
        self.problems_label.setWordWrap(True)
        self.warnings_label = QLabel()
        self.warnings_label.setWordWrap(True)
        self.apply_button = QPushButton("Apply")
        self.apply_button.setDefault(True)
        self.apply_button.clicked.connect(self._on_apply)
        self.revert_button = QPushButton("Revert")
        self.revert_button.clicked.connect(lambda: self.load(self._base))
        buttons = QHBoxLayout()
        buttons.addStretch(1)
        buttons.addWidget(self.revert_button)
        buttons.addWidget(self.apply_button)

        self.setMinimumWidth(340)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 6)
        layout.addWidget(scroll, 1)
        footer = QVBoxLayout()
        footer.setContentsMargins(8, 0, 8, 0)
        footer.addWidget(self.estimate_label)
        footer.addWidget(self.estimate_bar)
        footer.addWidget(self.problems_label)
        footer.addWidget(self.warnings_label)
        footer.addLayout(buttons)
        layout.addLayout(footer)

        self.load(self._base)

    # --- Public -------------------------------------------------------------------------------
    @property
    def base(self) -> Settings:
        """The applied settings the form compares against."""
        return copy.deepcopy(self._base)

    def pending(self) -> Settings:
        s = copy.deepcopy(self._base)
        valid = {f.name for f in dataclass_fields(Settings)}
        for binding in self._bindings:
            for name, v in binding.read().items():
                if name in valid:
                    setattr(s, name, v)
        return s

    def is_dirty(self) -> bool:
        return self.pending() != self._base

    def load(self, settings: Settings) -> None:
        """Shows `settings` as both the applied baseline and the form contents."""
        self._base = copy.deepcopy(settings)
        self._write_widgets(self._base)

    def update_base(self, **changes: Any) -> None:
        """Changes applied fields from outside the form (toolbar), keeping other edits."""
        pending = self.pending()
        for name, v in changes.items():
            setattr(self._base, name, v)
            setattr(pending, name, v)
        self._write_widgets(pending)

    def problems(self) -> list[str]:
        s = self.pending()
        problems = s.validate()
        if self._available is not None and s.source == "oak" and s.backend not in self._available:
            problems.append(f"{BACKEND_LABELS.get(s.backend, s.backend)} is {NOT_AVAILABLE_TIP}")
        return problems

    def apply_theme(self) -> None:
        self._refresh()

    # --- Widget builders ------------------------------------------------------------------------
    def _bind(self, read: Callable[[], dict], write: Callable[[Settings], None]) -> None:
        self._bindings.append(_Binding(read, write))

    def _spin(self, name: str, lo: int, hi: int, suffix: str = "", tip: str = "") -> QSpinBox:
        w = QSpinBox()
        w.setRange(lo, hi)
        w.setSuffix(suffix)
        w.setToolTip(tip)
        w.valueChanged.connect(self._on_edited)
        self._bind(lambda: {name: w.value()}, lambda s: w.setValue(getattr(s, name)))
        return w

    def _dspin(self, name: str, lo: float, hi: float, step: float, decimals: int = 1,
               suffix: str = "", tip: str = "") -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setDecimals(decimals)
        w.setSuffix(suffix)
        w.setToolTip(tip)
        w.valueChanged.connect(self._on_edited)
        self._bind(lambda: {name: w.value()}, lambda s: w.setValue(getattr(s, name)))
        return w

    def _check(self, name: str, text: str, tip: str = "") -> QCheckBox:
        w = QCheckBox(text)
        w.setToolTip(tip)
        w.toggled.connect(self._on_edited)
        self._bind(lambda: {name: w.isChecked()}, lambda s: w.setChecked(getattr(s, name)))
        return w

    def _line(self, name: str, placeholder: str = "", tip: str = "") -> QLineEdit:
        w = QLineEdit()
        w.setPlaceholderText(placeholder)
        w.setToolTip(tip)
        w.textChanged.connect(self._on_edited)
        self._bind(lambda: {name: w.text().strip()}, lambda s: w.setText(getattr(s, name)))
        return w

    @staticmethod
    def _combo() -> QComboBox:
        # Long item labels must not force the dock wider than the pilot made it
        w = QComboBox()
        w.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        w.setMinimumContentsLength(10)
        return w

    def _choice(self, name: str, items: list[tuple[str, str]]) -> QComboBox:
        w = self._combo()
        for label, data in items:
            w.addItem(label, data)
        w.currentIndexChanged.connect(self._on_edited)

        def write(s: Settings) -> None:
            i = w.findData(getattr(s, name))
            if i >= 0:
                w.setCurrentIndex(i)
        self._bind(lambda: {name: w.currentData()}, write)
        return w

    def _resolution(self, wname: str, hname: str, items: list[tuple[int, int, str]]) -> QComboBox:
        w = self._combo()
        for width, height, label in items:
            w.addItem(label, (width, height))
        w.currentIndexChanged.connect(self._on_edited)

        def write(s: Settings) -> None:
            size = (getattr(s, wname), getattr(s, hname))
            i = w.findData(size)
            if i < 0:   # a size from a hand-edited settings file: keep it selectable
                w.addItem(f"{size[0]} × {size[1]}", size)
                i = w.count() - 1
            w.setCurrentIndex(i)

        def read() -> dict:
            width, height = w.currentData()
            return {wname: width, hname: height}
        self._bind(read, write)
        return w

    @staticmethod
    def _form(group: QGroupBox) -> QFormLayout:
        form = QFormLayout(group)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        return form

    # --- Groups -------------------------------------------------------------------------------
    def _source_group(self) -> QGroupBox:
        group = QGroupBox("Source")
        form = self._form(group)
        self.camera_ip = self._line("camera_ip", "first camera found",
                                    "The OAK's IP address. Blank = first camera found.")
        self.find_button = QPushButton("Find cameras")
        self.find_button.clicked.connect(self._find_cameras)
        row = QHBoxLayout()
        row.addWidget(self.camera_ip, 1)
        row.addWidget(self.find_button)
        form.addRow("Camera IP", row)
        self.found_cameras = self._combo()
        self.found_cameras.setVisible(False)
        self.found_cameras.activated.connect(self._on_camera_chosen)
        form.addRow(self.found_cameras)
        self.find_status = QLabel()
        self.find_status.setWordWrap(True)
        self.find_status.setVisible(False)
        form.addRow(self.find_status)
        note = QLabel("The OAK accepts one connection: stop the BlueOS OAK-D extension while TrackShot runs.")
        note.setWordWrap(True)
        form.addRow(note)
        return group

    def _vio_group(self) -> QGroupBox:
        group = QGroupBox("VIO")
        form = self._form(group)
        self.backend = self._choice("backend", [(BACKEND_LABELS[b], b) for b in BACKENDS])
        model = self.backend.model()
        for i, b in enumerate(BACKENDS):
            if self._available is not None and b not in self._available:
                model.item(i).setEnabled(False)
                self.backend.setItemData(i, NOT_AVAILABLE_TIP, Qt.ItemDataRole.ToolTipRole)
        self.backend.currentIndexChanged.connect(self._on_backend_changed)
        form.addRow("Backend", self.backend)
        self.fps = self._spin("fps", 1, 60, " fps", "Switching backend sets its default frame rate.")
        form.addRow("Frame rate", self.fps)
        self.width_spin = self._spin("width", 64, 1280, " px")
        self.height_spin = self._spin("height", 64, 800, " px")
        size = QHBoxLayout()
        size.addWidget(self.width_spin)
        size.addWidget(QLabel("×"))
        size.addWidget(self.height_spin)
        form.addRow("Resolution", size)
        self.camera_angle = self._dspin("camera_angle_deg", -90, 90, 5, 1, "°", "0 = looking forward, 90 = straight down")
        form.addRow("Camera angle", self.camera_angle)

        self.rtabmap_box = QGroupBox("RTAB-Map")
        rform = self._form(self.rtabmap_box)
        rform.addRow("Tracked features", self._spin("rtabmap_features", 50, 2000, "", "Features tracked on the OAK"))
        rform.addRow("Local map size", self._spin(
            "rtabmap_local_map_size", 200, 5000, "",
            "Descriptors RTAB-Map matches each frame against (RTAB-Map's default is 2000)"))
        form.addRow(self.rtabmap_box)

        self.basalt_box = QGroupBox("Basalt")
        bform = self._form(self.basalt_box)
        bform.addRow("Optical-flow grid", self._spin("basalt_grid_size", 20, 200, " px", "Higher = fewer features, faster"))
        bform.addRow("Flow iterations", self._spin("basalt_flow_iterations", 1, 10))
        bform.addRow("Pyramid levels", self._spin("basalt_flow_levels", 1, 5))
        bform.addRow("Skip frames", self._spin("basalt_skip_frames", 0, 10))
        bform.addRow("Max states", self._spin("basalt_max_states", 1, 10))
        bform.addRow("Max keyframes", self._spin("basalt_max_kfs", 2, 20))
        bform.addRow("VIO iterations", self._spin("basalt_vio_iterations", 1, 10))
        form.addRow(self.basalt_box)
        return group

    def _mavlink_group(self) -> QGroupBox:
        group = QGroupBox("MAVLink")
        form = self._form(group)
        self.transport = self._choice("transport", [("mavlink2rest (BlueOS WebSocket)", "mavlink2rest"),
                                                    ("UDP (pymavlink)", "udp")])
        form.addRow("Transport", self.transport)
        form.addRow("Vehicle IP", self._line(
            "vehicle_ip", "192.168.2.2",
            "The ROV's BlueOS. Parameters and telemetry for the images come from here."))
        self.udp_url = self._line("udp_url", "udpout:192.168.2.2:14550")
        form.addRow("UDP URL", self.udp_url)
        form.addRow(self._check("send_health", "Send health values for Cockpit",
                                "TS_FPS, TS_LAT_MS, TS_JUMPS and TS_TRACK as NAMED_VALUE_FLOAT"))
        return group

    def _preview_group(self) -> QGroupBox:
        group = QGroupBox("Preview")
        form = self._form(group)
        form.addRow(self._check("preview_enabled", "VIO preview",
                                "Shows the image VIO already receives: no extra tether traffic"))
        self.color_preview = self._check("color_preview_enabled", "Colour preview",
                                         "An extra MJPEG stream from the centre camera")
        form.addRow(self.color_preview)
        self.color_fps = self._dspin("color_preview_fps", 0.5, 30, 0.5, 1, " fps")
        self.color_size = self._resolution("color_preview_width", "color_preview_height", COLOR_PREVIEW_SIZES)
        self.color_quality = self._spin("color_preview_quality", 10, 100, "", "JPEG quality")
        form.addRow("Colour rate", self.color_fps)
        form.addRow("Colour size", self.color_size)
        form.addRow("Colour quality", self.color_quality)
        return group

    def _capture_group(self) -> QGroupBox:
        group = QGroupBox("Capture")
        form = self._form(group)
        self.capture_dir = self._line("capture_dir")
        choose = QPushButton("Choose…")
        choose.clicked.connect(self._choose_folder)
        row = QHBoxLayout()
        row.addWidget(self.capture_dir, 1)
        row.addWidget(choose)
        form.addRow("Folder", row)
        form.addRow("Centre rate", self._dspin("center_capture_fps", 0, 10, 0.1, 1, " fps", "IMX378 colour. 0 = off"))
        form.addRow("Centre size", self._resolution("center_width", "center_height", CENTER_RESOLUTIONS))
        form.addRow("Stereo rate", self._dspin("stereo_capture_fps", 0, 30, 0.1, 1, " fps",
                                               "OV9282 left + right pairs. 0 = off. Capped at the VIO frame rate"))
        form.addRow("Stereo size", self._resolution("mono_width", "mono_height", MONO_RESOLUTIONS))
        form.addRow("JPEG quality", self._spin("jpeg_quality", 10, 100))
        form.addRow("Min free space", self._dspin("min_free_gb", 0.5, 1000, 0.5, 1, " GB", "Recording stops below this"))
        form.addRow("Source tag", self._line("source_tag", "tl000000"))
        return group

    def _bandwidth_group(self) -> QGroupBox:
        group = QGroupBox("Bandwidth budget")
        form = self._form(group)
        form.addRow("Tether", self._dspin("tether_budget_mbit", 0, 1000, 5, 0, " Mbit/s", "Usable tether throughput"))
        form.addRow("Video allowance", self._dspin("video_allowance_mbit", 0, 1000, 1, 0, " Mbit/s",
                                                   "Cockpit video already on the tether"))
        return group

    # --- Behaviour ------------------------------------------------------------------------------
    def _write_widgets(self, settings: Settings) -> None:
        self._loading = True
        try:
            for binding in self._bindings:
                binding.write(settings)
        finally:
            self._loading = False
        self._refresh()

    def _on_edited(self, *_: Any) -> None:
        if not self._loading:
            self._refresh()

    def _on_backend_changed(self, *_: Any) -> None:
        if self._loading:
            return
        # Each backend has its own sweet spot; switching resets the rate to it, as the extension did.
        self.fps.setValue(BACKEND_DEFAULT_FPS.get(self.backend.currentData(), self.fps.value()))

    def _refresh(self) -> None:
        """Re-validates and re-estimates the pending settings, and updates dependent widgets."""
        s = self.pending()
        theme = current_theme(self.palette())
        self.rtabmap_box.setVisible(s.backend == "rtabmap")
        self.basalt_box.setVisible(s.backend == "basalt")
        self.udp_url.setEnabled(s.transport == "udp")
        for w in (self.color_fps, self.color_size, self.color_quality):
            w.setEnabled(s.color_preview_enabled)

        problems = self.problems()
        self.problems_label.setText("\n".join(f"✖ {p}" for p in problems))
        self.problems_label.setStyleSheet(f"color: {css(theme.bad)};")
        self.problems_label.setVisible(bool(problems))

        warnings = []
        if s.backend == "rtabmap" and s.fps < RTABMAP_MIN_GOOD_FPS:
            warnings.append(f"RTAB-Map loses tracking below ~{RTABMAP_MIN_GOOD_FPS} fps.")
        if s.stereo_capture_fps > s.fps:
            warnings.append(f"Stereo capture runs at {s.effective_stereo_capture_fps():g} fps (capped at the VIO rate).")
        if s.source == "sim":
            warnings.append("Simulator selected: nothing is pulled over the tether.")
        self.warnings_label.setText("\n".join(f"⚠ {w}" for w in warnings))
        self.warnings_label.setStyleSheet(f"color: {css(theme.warn)};")
        self.warnings_label.setVisible(bool(warnings))

        dirty = s != self._base
        self.apply_button.setEnabled(dirty and not problems)
        self.revert_button.setEnabled(dirty)

        self.estimate = bandwidth.estimate(s)
        self.estimate_bar.set_estimate(self.estimate)
        self.estimate_label.setText(f"Tether: {self.estimate.total_mbit:.1f} of {self.estimate.budget_mbit:.0f} Mbit/s"
                                    + (" (with your edits)" if dirty else ""))
        self.estimate_label.setStyleSheet(f"color: {css(theme.level(self.estimate.level))}; font-weight: 600;")
        self.pendingChanged.emit(s, self.estimate, dirty)

    def _on_apply(self) -> None:
        if self.problems():
            return
        self.applyRequested.emit(self.pending())

    def _choose_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "Capture folder", self.capture_dir.text())
        if folder:
            self.capture_dir.setText(folder)

    def _find_cameras(self) -> None:
        self.find_button.setEnabled(False)
        self.find_status.setText("Searching for cameras…")
        self.find_status.setVisible(True)
        run_in_pool(self._pool, list_cameras, self._on_cameras_found, self._on_find_failed)

    def _on_cameras_found(self, cameras: list[dict]) -> None:
        self.find_button.setEnabled(True)
        self.found_cameras.clear()
        for cam in cameras:
            # PoE cameras are addressed by IP; a USB camera by its device id
            address = cam.get("ip") or cam.get("id") or ""
            label = address if address == cam.get("name") else f"{cam.get('name') or 'OAK'} — {address}"
            if cam.get("in_use"):
                label += " (in use: stop the BlueOS OAK-D extension)"
            elif cam.get("state"):
                label += f" ({cam['state']})"
            self.found_cameras.addItem(label, address)
        self.found_cameras.setVisible(bool(cameras))
        self.find_status.setText(f"{len(cameras)} camera(s) found — pick one to use its address."
                                 if cameras else "No cameras found. Check the tether and the camera's power.")

    def _on_find_failed(self, error: BaseException) -> None:
        self.find_button.setEnabled(True)
        self.find_status.setText(f"Camera search failed: {error}")

    def _on_camera_chosen(self, index: int) -> None:
        address = self.found_cameras.itemData(index)
        if address:
            self.camera_ip.setText(address)
