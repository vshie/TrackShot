"""Cockpit widget: how to put TrackShot into Cockpit, and the URL to paste.

The steps follow Cockpit's own UI (Edit Interface → the IFrame widget → its "Iframe Source"),
checked against the Cockpit source.
"""

from __future__ import annotations

from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget)

from trackshot.ui.theme import css, current_theme

INSTRUCTIONS = """
<ol style="margin-left:-20px">
<li>In Cockpit, open the main menu and choose <b>Edit Interface</b>.</li>
<li>In the widget list (<i>Regular</i>), drag the <b>IFrame</b> card onto the view.</li>
<li>Open the new widget's settings (its cog in edit mode), paste the URL below into
    <b>Iframe Source</b> and press <b>✓</b> (Set). Leave <i>Use vehicle address as base URL</i>
    (Advanced options) off.</li>
<li>Size and place it, then leave edit mode.</li>
</ol>
<p>Use the <b>localhost</b> URL when Cockpit runs on this computer, and this computer's address
when Cockpit runs on another one on the same network. TrackShot must be running for the widget to
show anything; it says <i>TrackShot not reachable</i> otherwise.</p>
"""


class WidgetPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.state_label = QLabel()
        self.state_label.setWordWrap(True)
        self.instructions = QLabel(INSTRUCTIONS)
        self.instructions.setWordWrap(True)
        self.urls_box = QVBoxLayout()
        self.warning = QLabel()
        self.warning.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.state_label)
        layout.addLayout(self.urls_box)
        layout.addWidget(self.instructions)
        layout.addWidget(self.warning)
        layout.addStretch(1)
        self.url_fields: list[QLineEdit] = []
        self.show_server(False, [], None, lan=True)

    def show_server(self, running: bool, urls: list[str], error: str | None, lan: bool,
                    enabled: bool = True) -> None:
        theme = current_theme(self.palette())
        if not enabled:
            self.state_label.setText("The Cockpit widget is off (Settings → Cockpit widget).")
        elif running:
            self.state_label.setText("Serving the Cockpit widget. URL to paste into Cockpit:")
        else:
            self.state_label.setText(f"The Cockpit widget isn't being served: {error or 'not started'}. "
                                     "Change the port in Settings → Cockpit widget.")
        self.state_label.setStyleSheet("" if running or not enabled else f"color: {css(theme.bad)};")
        while self.urls_box.count():
            item = self.urls_box.takeAt(0)
            row = item.layout()
            if row is not None:
                while row.count():
                    w = row.takeAt(0).widget()
                    if w is not None:
                        w.deleteLater()
        self.url_fields = []
        for url in urls:
            field = QLineEdit(url)
            field.setReadOnly(True)
            copy = QPushButton("Copy")
            copy.setToolTip("Copy this URL for Cockpit's IFrame widget")
            copy.clicked.connect(lambda _=False, u=url: QGuiApplication.clipboard().setText(u))
            row = QHBoxLayout()
            row.addWidget(field, 1)
            row.addWidget(copy)
            self.urls_box.addLayout(row)
            self.url_fields.append(field)
        self.warning.setText("Anyone on the network can start and stop tracking and recording through the "
                             "widget while it is reachable from the network." if running and lan else "")
        self.warning.setStyleSheet(f"color: {css(theme.muted)};")
