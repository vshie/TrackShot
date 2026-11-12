"""Log: the engine's logMessage stream, newest at the bottom, coloured by level, capped in length."""

from __future__ import annotations

import html
import time

from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget

from trackshot.ui.theme import css, current_theme

MAX_LINES = 2000
LEVEL_LABELS = {"debug": "DEBUG", "info": "INFO", "success": "OK", "warning": "WARN", "error": "ERROR",
                "critical": "ERROR"}


class LogPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(MAX_LINES)   # Qt drops the oldest lines itself
        mono = self.view.font()
        mono.setFamilies(["Menlo", "SF Mono", "Monaco", "monospace"])
        self.view.setFont(mono)
        self.follow = QCheckBox("Follow")
        self.follow.setChecked(True)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.view.clear)
        bar = QHBoxLayout()
        bar.addWidget(self.follow)
        bar.addStretch(1)
        bar.addWidget(clear)
        layout = QVBoxLayout(self)
        layout.addLayout(bar)
        layout.addWidget(self.view, 1)

    def apply_theme(self) -> None:
        """Lines already shown keep their colours; new ones use the new palette."""

    def append(self, level: str, text: str) -> None:
        theme = current_theme(self.palette())
        level = (level or "info").lower()
        colour = {"warning": theme.warn, "error": theme.bad, "critical": theme.bad,
                  "success": theme.ok, "debug": theme.muted}.get(level, theme.text)
        stamp = time.strftime("%H:%M:%S")
        label = LEVEL_LABELS.get(level, level.upper())
        self.view.appendHtml(
            f'<span style="color:{css(theme.muted)}">{stamp}</span> '
            f'<span style="color:{css(colour)}"><b>{label:<5}</b> {html.escape(text)}</span>')
        if self.follow.isChecked():
            bar = self.view.verticalScrollBar()
            bar.setValue(bar.maximum())

    def line_count(self) -> int:
        return self.view.document().blockCount() if self.view.toPlainText() else 0
