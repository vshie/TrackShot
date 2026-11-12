"""Trajectory: top-down XY path with the current heading, and z (plus vehicle depth) against time.

Poses arrive at up to 20 Hz from the engine; they go into fixed-size numpy buffers and the plots
redraw on a 10 Hz timer, so a long dive never slows the GUI. The oldest points are dropped beyond
MAX_POINTS — the panel says so.

Once the autopilot's attitude is known, the map is north-up: east to the right, north up, the
track being the VIO's motion rotated by the compass (trackshot.core.ned), and the red arrow is the
vehicle's compass heading. Before that (no telemetry, e.g. the simulator on its own) it falls back
to the VIO's own frame (x forward at start, y left) and the VIO's yaw.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSplitter, QVBoxLayout,
                               QWidget)

from trackshot.ui.theme import css, current_theme

MAX_POINTS = 20_000
REDRAW_MS = 100
FOLLOW_MIN_SPAN_M = 4.0


def yaw_from_quaternion(qw: float, qx: float, qy: float, qz: float) -> float:
    """Yaw (rad) about z of a body→world quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class RingBuffer:
    """Last `capacity` rows of a fixed-width float array, oldest first."""

    def __init__(self, capacity: int, width: int):
        self._data = np.empty((capacity, width), dtype=np.float64)
        self._n = 0

    def __len__(self) -> int:
        return self._n

    def append(self, row) -> None:
        if self._n == len(self._data):
            # Drop the oldest tenth in one copy rather than shifting on every append
            keep = len(self._data) - len(self._data) // 10
            self._data[:keep] = self._data[self._n - keep:self._n]
            self._n = keep
        self._data[self._n] = row
        self._n += 1

    def view(self) -> np.ndarray:
        return self._data[:self._n]

    def clear(self) -> None:
        self._n = 0


class TrajectoryPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._path = RingBuffer(MAX_POINTS, 4)       # t, x, y, z
        self._depth = RingBuffer(MAX_POINTS, 2)      # t, -depth (so it reads the same way as z-up)
        self._t0: Optional[float] = None
        self._heading: Optional[float] = None       # VIO frame: CCW from +x. North-up: clockwise from north
        self._north_up = False
        self._dirty = False

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "x (forward at start)", units="m")
        self.plot.setLabel("left", "y (left at start)", units="m")
        self.path_curve = self.plot.plot([], [])
        self.start_marker = pg.ScatterPlotItem(size=9, symbol="s")
        self.plot.addItem(self.start_marker)
        self.arrow = pg.ArrowItem(angle=0, headLen=20, tailLen=0, pxMode=True)
        self.arrow.setVisible(False)
        self.plot.addItem(self.arrow)

        self.depth_plot = pg.PlotWidget()
        self.depth_plot.showGrid(x=True, y=True, alpha=0.25)
        self.depth_plot.setLabel("bottom", "time", units="s")
        self.depth_plot.setLabel("left", "z", units="m")
        self.legend = self.depth_plot.addLegend(offset=(-10, 10))
        self.z_curve = self.depth_plot.plot([], [], name="VIO z")
        self.depth_curve = self.depth_plot.plot([], [], name="−depth (vehicle)")

        self.follow = QCheckBox("Auto-follow")
        self.follow.setChecked(True)
        self.follow.setToolTip("Keep the vehicle centred at the current zoom")
        self.fit_button = fit = QPushButton("Fit path")
        fit.setToolTip("Show the whole path (turns auto-follow off)")
        fit.clicked.connect(self._fit)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.clear)
        self.info = QLabel()
        cap = QLabel(f"Keeps the last {MAX_POINTS:,} poses")
        self._cap_label = cap
        for label in (self.info, cap):
            # In a narrow window the readouts are cut short instead of squeezing the controls
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        controls = QHBoxLayout()
        controls.addWidget(self.follow)
        controls.addWidget(fit)
        controls.addWidget(clear)
        controls.addStretch(1)
        controls.addWidget(self.info)
        controls.addWidget(cap)

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self.plot)
        splitter.addWidget(self.depth_plot)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 120])

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addLayout(controls)
        layout.addWidget(splitter, 1)

        self._timer = QTimer(self)
        self._timer.setInterval(REDRAW_MS)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        self.apply_theme()
        self._update_info()

    # --- Inputs -------------------------------------------------------------------------------
    def add_pose(self, payload) -> None:
        """poseUpdated payload: (PoseSample, vehicle pose (x, y, z, qw, qx, qy, qz), nav), nav being
        (north, east, compass heading rad) once the autopilot's attitude is known, else None."""
        sample, pose = payload[0], payload[1]
        nav = payload[2] if len(payload) > 2 else None
        if pose is None:
            return
        x, y, z, qw, qx, qy, qz = pose[:7]
        t = getattr(sample, "t_received", None) or time.time()
        if nav is not None and not self._north_up:
            # Switch to the north-up map: the VIO-frame points don't belong on it
            self._path.clear()
            self._north_up = True
            self.plot.setLabel("bottom", "east", units="m")
            self.plot.setLabel("left", "north", units="m")
        if self._t0 is None:
            self._t0 = t
        if self._north_up:
            if nav is None:
                return
            north, east, heading = nav
            self._path.append((t - self._t0, east, north, z))
            self._heading = heading
        else:
            self._path.append((t - self._t0, x, y, z))
            self._heading = yaw_from_quaternion(qw, qx, qy, qz)
        self._dirty = True

    def add_vehicle_depth(self, depth_m: Optional[float], t: Optional[float] = None) -> None:
        if depth_m is None or self._t0 is None:
            return   # only meaningful alongside a VIO track
        self._depth.append(((t or time.time()) - self._t0, -depth_m))
        self._dirty = True

    def point_count(self) -> int:
        return len(self._path)

    def clear(self) -> None:
        self._path.clear()
        self._depth.clear()
        self._t0 = None
        self._heading = None
        self._north_up = False
        self.plot.setLabel("bottom", "x (forward at start)", units="m")
        self.plot.setLabel("left", "y (left at start)", units="m")
        self._dirty = True
        self.refresh()

    # --- Drawing ------------------------------------------------------------------------------
    def apply_theme(self) -> None:
        theme = current_theme(self.palette())
        for plot in (self.plot, self.depth_plot):
            plot.setBackground(theme.base)
            for axis in ("left", "bottom"):
                ax = plot.getAxis(axis)
                ax.setPen(pg.mkPen(theme.border))
                ax.setTextPen(pg.mkPen(theme.text))
        self.path_curve.setPen(pg.mkPen(theme.vio, width=2))
        self.start_marker.setBrush(pg.mkBrush(theme.ok))
        self.start_marker.setPen(pg.mkPen(theme.ok))
        self.arrow.setStyle(brush=pg.mkBrush(theme.bad), pen=pg.mkPen(theme.text, width=1))
        self.z_curve.setPen(pg.mkPen(theme.vio, width=2))
        self.depth_curve.setPen(pg.mkPen(theme.color_preview, width=2, style=Qt.PenStyle.DashLine))
        self.legend.setLabelTextColor(theme.text)
        self._cap_label.setStyleSheet(f"color: {css(theme.muted)};")
        self.info.setStyleSheet(f"color: {css(theme.muted)};")

    def refresh(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        path = self._path.view()
        self.path_curve.setData(path[:, 1], path[:, 2])
        self.z_curve.setData(path[:, 0], path[:, 3])
        depth = self._depth.view()
        self.depth_curve.setData(depth[:, 0], depth[:, 1])
        if len(path):
            self.start_marker.setData([path[0, 1]], [path[0, 2]])
            x, y = path[-1, 1], path[-1, 2]
            self.arrow.setPos(x, y)
            # ArrowItem's angle 0 points to -x on screen and turns clockwise (screen y is down).
            # North-up: a compass heading ψ points (east, north) = (sin ψ, cos ψ), i.e. 90° − ψ
            # counter-clockwise from +x on screen.
            heading = math.degrees(self._heading or 0.0)
            ccw_from_x = 90.0 - heading if self._north_up else heading
            self.arrow.setStyle(angle=180.0 - ccw_from_x)
            self.arrow.setVisible(True)
            if self.follow.isChecked():
                self._centre_on(x, y)
        else:
            self.start_marker.setData([], [])
            self.arrow.setVisible(False)
        self._update_info()

    def _centre_on(self, x: float, y: float) -> None:
        (x0, x1), (y0, y1) = self.plot.viewRange()
        half_w = max((x1 - x0) / 2, FOLLOW_MIN_SPAN_M / 2)
        half_h = max((y1 - y0) / 2, FOLLOW_MIN_SPAN_M / 2)
        self.plot.setRange(xRange=(x - half_w, x + half_w), yRange=(y - half_h, y + half_h), padding=0)

    def _fit(self) -> None:
        self.follow.setChecked(False)
        self.plot.autoRange()

    def _update_info(self) -> None:
        path = self._path.view()
        if not len(path):
            self.info.setText("No poses yet")
            return
        x, y, z = path[-1, 1:4]
        heading = math.degrees(self._heading or 0.0)
        if self._north_up:
            self.info.setText(f"N {y:+.2f}  E {x:+.2f} m  ·  heading {heading % 360:.0f}° (compass)  ·  "
                              f"{len(path):,} pts")
        else:
            self.info.setText(f"x {x:+.2f}  y {y:+.2f}  z {z:+.2f} m  ·  yaw {heading:+.0f}° (VIO)  ·  "
                              f"{len(path):,} pts")
