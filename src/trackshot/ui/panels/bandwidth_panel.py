"""Tether bandwidth: what the current settings will pull over the tether, against its budget.

The estimate comes from trackshot.bandwidth (the extension's model). It is drawn as a stacked bar
— VIO, colour preview, survey capture, Cockpit video — with the 70 % and 90 % marks where the
estimate turns amber and red, plus the camera→Mac throughput actually measured while running.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from trackshot.bandwidth import BAD_FRACTION, WARN_FRACTION, BandwidthEstimate
from trackshot.ui.theme import Theme, css, current_theme, fmt

LEVEL_TEXT = {"ok": "OK", "warn": "Little headroom", "bad": "Over the safe limit"}


def segments(est: BandwidthEstimate, theme: Theme) -> list[tuple[str, float, QColor]]:
    """(label, Mbit/s, colour) for each part of the bar, in drawing order."""
    return [
        ("VIO", est.vio_mbit, theme.vio),
        ("Colour preview", est.color_preview_mbit, theme.color_preview),
        ("Capture", est.center_capture_mbit + est.stereo_capture_mbit, theme.capture),
        ("Video", est.video_mbit, theme.video),
    ]


class BandwidthBar(QWidget):
    """Stacked horizontal bar. `compact` drops the in-bar labels and the scale for the settings form."""

    def __init__(self, compact: bool = False, parent: QWidget | None = None):
        super().__init__(parent)
        self.compact = compact
        self.estimate: Optional[BandwidthEstimate] = None
        self.measured_mbit: Optional[float] = None
        self.setMinimumHeight(14 if compact else 54)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def sizeHint(self):
        size = super().sizeHint()
        size.setHeight(14 if self.compact else 54)
        return size

    def set_estimate(self, est: BandwidthEstimate, measured_mbit: Optional[float] = None) -> None:
        self.estimate = est
        self.measured_mbit = measured_mbit
        self.setToolTip(est.hint)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt API)
        if self.estimate is None:
            return
        est, theme = self.estimate, current_theme(self.palette())
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        small = QFont(self.font())
        small.setPointSizeF(max(8.0, small.pointSizeF() * 0.8))
        p.setFont(small)

        label_h = 0 if self.compact else 16
        bar = QRectF(1, 1, self.width() - 2, self.height() - 2 - label_h)
        # Leave room beyond the budget so an over-budget estimate is visibly over it
        scale_max = max(est.budget_mbit * 1.15, est.total_mbit * 1.05, (self.measured_mbit or 0) * 1.05, 1.0)

        def x_at(mbit: float) -> float:
            return bar.left() + bar.width() * min(mbit, scale_max) / scale_max

        p.setPen(QPen(theme.border, 1))
        p.setBrush(theme.base)
        p.drawRoundedRect(bar, 3, 3)

        x = bar.left()
        for label, mbit, colour in segments(est, theme):
            if mbit <= 0:
                continue
            w = x_at(mbit) - bar.left()
            seg = QRectF(x, bar.top(), w, bar.height())
            p.fillRect(seg, colour)
            if not self.compact and w > 70:
                p.setPen(QColor("white") if not theme.dark else QColor("#0d1117"))
                p.drawText(seg.adjusted(6, 0, -4, 0), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                           f"{label} {mbit:.1f}")
            x += w

        # 70 % / 90 % marks and the budget itself
        for frac, colour in ((WARN_FRACTION, theme.warn), (BAD_FRACTION, theme.bad), (1.0, theme.text)):
            mx = x_at(est.budget_mbit * frac)
            pen = QPen(colour, 2 if frac == 1.0 else 1.5)
            if frac != 1.0:
                pen.setStyle(Qt.PenStyle.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(mx, bar.top() - 1), QPointF(mx, bar.bottom() + 1))
            if not self.compact:
                text = f"{est.budget_mbit:.0f} Mbit/s" if frac == 1.0 else f"{frac:.0%}"
                p.setPen(colour)
                p.drawText(QRectF(mx - 60, bar.bottom() + 2, 120, label_h),
                           Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, text)

        if self.measured_mbit is not None and not self.compact:
            # Downward triangle above the bar: what the camera link is really carrying
            mx = x_at(self.measured_mbit)
            p.setPen(QPen(theme.text, 1))
            p.setBrush(theme.text)
            p.drawPolygon(QPolygonF([QPointF(mx - 5, bar.top()), QPointF(mx + 5, bar.top()),
                                     QPointF(mx, bar.top() + 7)]))
            p.drawLine(QPointF(mx, bar.top()), QPointF(mx, bar.bottom()))
        p.end()


class BandwidthPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.estimate: Optional[BandwidthEstimate] = None
        self.measured_mbit: Optional[float] = None
        self._pending = False

        self.caption = QLabel()
        self.total = QLabel()
        bold = self.total.font()
        bold.setBold(True)
        bold.setPointSizeF(bold.pointSizeF() * 1.2)
        self.total.setFont(bold)
        self.bar = BandwidthBar()
        self.hint = QLabel()
        self.hint.setWordWrap(True)
        self.measured = QLabel()
        self.measured.setWordWrap(True)

        self.legend = QGridLayout()
        self.legend.setHorizontalSpacing(10)
        self._legend_rows: list[tuple[QFrame, QLabel, QLabel]] = []
        for _ in range(4):
            swatch, name, val = QFrame(), QLabel(), QLabel()
            swatch.setFixedSize(12, 12)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row = len(self._legend_rows)
            self.legend.addWidget(swatch, row, 0)
            self.legend.addWidget(name, row, 1)
            self.legend.addWidget(val, row, 2)
            self._legend_rows.append((swatch, name, val))
        self.vio_note = QLabel("VIO preview uses frames already on this Mac: 0 Mbit/s extra.")
        self.vio_note.setWordWrap(True)

        heading = QHBoxLayout()
        heading.addWidget(self.total)
        heading.addStretch(1)
        heading.addWidget(self.caption)
        notes = QVBoxLayout()
        notes.addWidget(self.hint)
        notes.addWidget(self.vio_note)
        notes.addWidget(self.measured)
        notes.addStretch(1)
        details = QHBoxLayout()
        details.addLayout(self.legend)
        details.addSpacing(24)
        details.addLayout(notes, 1)
        layout = QVBoxLayout(self)
        layout.addLayout(heading)
        layout.addWidget(self.bar)
        layout.addLayout(details)
        layout.addStretch(1)

    def set_estimate(self, est: BandwidthEstimate, pending: bool = False) -> None:
        """`pending`: the estimate is for form edits not yet applied."""
        self.estimate = est
        self._pending = pending
        self._render()

    def set_measured(self, rx_mbit: Optional[float]) -> None:
        if rx_mbit != self.measured_mbit:
            self.measured_mbit = rx_mbit
            self._render()

    def apply_theme(self) -> None:
        self._render()

    def _render(self) -> None:
        est = self.estimate
        if est is None:
            return
        theme = current_theme(self.palette())
        self.caption.setText("Estimate for the edited settings (not applied yet)" if self._pending
                             else "Estimate for the applied settings")
        self.caption.setStyleSheet(f"color: {css(theme.warn if self._pending else theme.muted)};")
        colour = css(theme.level(est.level))
        self.total.setText(f"{est.total_mbit:.1f} of {est.budget_mbit:.0f} Mbit/s "
                           f"({est.fraction:.0%}) — {LEVEL_TEXT.get(est.level, est.level)}")
        self.total.setStyleSheet(f"color: {colour};")
        self.hint.setText(est.hint)
        self.bar.set_estimate(est, self.measured_mbit)
        for (swatch, name, val), (label, mbit, seg_colour) in zip(self._legend_rows, segments(est, theme)):
            swatch.setStyleSheet(f"background: {css(seg_colour)}; border-radius: 2px;")
            name.setText(label)
            val.setText(f"{mbit:.1f} Mbit/s")
        if self.measured_mbit is None:
            self.measured.setText("Measured camera → Mac: — (shown while the OAK camera runs)")
        else:
            self.measured.setText(f"Measured camera → Mac: {fmt(self.measured_mbit, '.1f', 'Mbit/s')} "
                                  f"(▼ on the bar; excludes Cockpit video)")
