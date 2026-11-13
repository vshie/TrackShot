"""The Cockpit widget server (trackshot.web.server) driven through a real MainWindow + FakeEngine."""

from __future__ import annotations

import io
import json
import os
import socket
import urllib.error
import urllib.request

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from PIL import Image  # noqa: E402
from PySide6.QtCore import QSettings  # noqa: E402

from fakes import FakeEngine, make_status  # noqa: E402
from trackshot.config import Settings  # noqa: E402
from trackshot.ui.main_window import MainWindow  # noqa: E402
from trackshot.ui.panels import settings_panel  # noqa: E402
from trackshot.vio.source import PreviewFrame  # noqa: E402
from trackshot.web.server import WidgetCommands, WidgetServer, WidgetState, widget_status  # noqa: E402


@pytest.fixture
def window(qtbot, tmp_path, monkeypatch):
    QSettings.setDefaultFormat(QSettings.Format.IniFormat)
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope, str(tmp_path / "qsettings"))
    monkeypatch.setattr(settings_panel, "available_backends", lambda: ["rtabmap", "basalt"])
    engine = FakeEngine(Settings(capture_dir=str(tmp_path / "captures")))
    win = MainWindow(engine, settings_path=tmp_path / "settings.json")
    qtbot.addWidget(win)
    win.show()
    yield win
    QSettings.setDefaultFormat(QSettings.Format.NativeFormat)


def base(win) -> str:
    assert win._widget_server is not None and win._widget_server.running
    return f"http://127.0.0.1:{win._widget_server.port}"


def get(url: str):
    with urllib.request.urlopen(url, timeout=3) as r:
        return r.status, dict(r.headers), r.read()


def post(url: str, body: dict):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, json.loads(r.read())


def test_the_page_is_served_with_cors(window):
    code, headers, body = get(base(window) + "/widget")
    assert code == 200 and b"TrackShot" in body and b"preview.jpg" in body
    assert headers["Access-Control-Allow-Origin"] == "*" and headers["Cache-Control"] == "no-store"
    assert get(base(window) + "/")[0] == 200


def test_status_reflects_the_engine(window, qtbot):
    status = make_status()
    status["telemetry"] = dict(status["telemetry"] or {}, depth_m=6.2, altitude_m=2.51, mode="AUTO", armed=True)
    status["altitude"] = {"distance_m": 2.48, "quality": 96, "age_s": 0.05, "sending": True}
    window.engine.statusUpdated.emit(status)
    code, _, body = get(base(window) + "/status")
    s = json.loads(body)
    assert code == 200
    assert (s["state"], s["tracking"], s["sending"]) == ("running", "good", True)
    assert (s["depth_m"], s["altitude_m"], s["altitude_quality"], s["rangefinder_m"]) == (6.2, 2.48, 96, 2.51)
    assert s["altitude_sending"] and (s["mode"], s["armed"]) == ("AUTO", True)


def test_status_before_anything_happened():
    s = widget_status({})
    assert s["state"] == "idle" and s["altitude_m"] is None and s["recording"] is False


def test_preview_has_the_green_feature_dots(window, qtbot):
    assert get(base(window) + "/preview.jpg")[0] == 204              # nothing yet
    image = np.full((200, 320), 60, dtype=np.uint8)
    window.engine.previewUpdated.emit(PreviewFrame(kind="vio", image=image, t_received=0.0,
                                                   features=np.array([[100.0, 50.0], [250.0, 150.0]])))
    code, headers, body = get(base(window) + "/preview.jpg")
    assert code == 200 and headers["Content-Type"] == "image/jpeg"
    img = np.asarray(Image.open(io.BytesIO(body)).convert("RGB")).astype(int)
    assert img.shape == (200, 320, 3)
    r, g, b = img[50, 100]
    assert g > 180 and g - r > 60 and g - b > 60                       # a green dot at the feature
    r, g, b = img[10, 10]
    assert abs(r - g) < 20 and abs(g - b) < 20                         # grey elsewhere


def test_colour_frames_are_not_the_vio_view(window):
    window.engine.previewUpdated.emit(PreviewFrame(kind="color", image=np.zeros((10, 10, 3), np.uint8),
                                                   t_received=0.0))
    assert get(base(window) + "/preview.jpg")[0] == 204


def test_start_stop_and_record_press_the_window_controls(window, qtbot):
    engine = window.engine
    assert post(base(window) + "/vio", {"run": True}) == (200, {"ok": True})
    qtbot.waitUntil(lambda: bool(engine.called("start")), timeout=3000)
    assert post(base(window) + "/record", {"on": True})[0] == 200
    qtbot.waitUntil(lambda: window.record_action.isChecked(), timeout=3000)
    assert ("set_recording", True) in engine.calls
    post(base(window) + "/record", {"on": False})
    qtbot.waitUntil(lambda: not window.record_action.isChecked(), timeout=3000)
    post(base(window) + "/vio", {"run": False})
    qtbot.waitUntil(lambda: bool(engine.called("stop")), timeout=3000)


def test_bad_commands_are_refused(window):
    for path, body in (("/vio", {"run": "yes"}), ("/record", {}), ("/nothing", {"on": True})):
        with pytest.raises(urllib.error.HTTPError) as e:
            post(base(window) + path, body)
        assert e.value.code == 400


def test_the_panel_lists_the_url_to_paste(window):
    urls = [f.text() for f in window.widget_panel.url_fields]
    assert urls and urls[0] == f"http://localhost:{window._widget_server.port}/widget"
    assert "Iframe Source" in window.widget_panel.instructions.text()
    window.widget_help_action.trigger()
    assert window.docks["widget"].isVisible()


def test_turning_the_widget_off_stops_the_server(window):
    url = base(window) + "/status"
    s = Settings(**{**vars(window._applied), "widget_enabled": False})
    window.apply_settings(s)
    assert window._widget_server is None
    with pytest.raises(OSError):
        get(url)
    assert "off" in window.widget_panel.state_label.text()


def test_a_busy_port_is_reported_not_raised():
    holder = socket.socket()
    holder.bind(("127.0.0.1", 0))
    holder.listen()
    port = holder.getsockname()[1]
    try:
        server = WidgetServer.__new__(WidgetServer)
        # bypass the test fixture's free-port override: bind exactly the busy port
        server._state, server._commands, server._page = WidgetState(), WidgetCommands(), b""
        server.requested_port, server.lan, server._httpd, server._thread, server.error = port, False, None, None, None
        assert server.start() is False
        assert not server.running and "unavailable" in server.error and server.urls() == []
    finally:
        holder.close()
