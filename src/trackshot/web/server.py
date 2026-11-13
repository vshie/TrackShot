"""The Cockpit widget: a small HTTP server so Cockpit's IFrame widget can show and control TrackShot.

    GET  /widget        the page (trackshot/resources/widget.html)
    GET  /status        JSON: engine and tracking state, recording, depth, altitude, rangefinder
    GET  /preview.jpg   the latest VIO image with the tracked features drawn as green dots
    POST /vio           {"run": true|false}   start / stop tracking
    POST /record        {"on": true|false}    start / stop recording

Every reply carries `Access-Control-Allow-Origin: *` (Cockpit's page has another origin) and
`Cache-Control: no-store`. The server runs on its own threads and never touches the engine: the
main window copies status and preview frames into `WidgetState` on the GUI thread, and commands go
back to the GUI thread through `WidgetCommands`' queued signals, which press the same toolbar
actions a click would (so the window and the widget always agree).

There is no authentication: anyone who can reach the port can start/stop tracking and recording.
It listens on all interfaces by default (Settings → Cockpit widget) so Cockpit on another computer
can load it; on this computer only, untick "Reachable from the network".
"""

from __future__ import annotations

import io
import ipaddress
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import numpy as np
from loguru import logger
from PIL import Image, ImageDraw
from PySide6.QtCore import QObject, Signal

from trackshot.ui.panels.depth_panel import depth_readouts
from trackshot.ui.theme import value

FEATURE_COLOUR = (0x39, 0xFF, 0x14)        # the desktop preview's green
JPEG_QUALITY = 75
MAX_BODY = 4096


def widget_status(status: dict, connected: bool = True) -> dict[str, Any]:
    """JSON-safe view of an engine status for the widget."""
    odo = status.get("odometry")
    source = status.get("source")
    capture = status.get("capture") or {}
    telemetry = status.get("telemetry") or {}
    counts = value(capture, "counts", {}) or {}
    return {
        "state": status.get("state", "idle"),
        "tracking": value(odo, "tracking", "unknown"),
        "sending": bool(value(odo, "sending", False)),
        "lost_for_s": value(odo, "lost_for_s"),
        "vio_fps": value(source, "fps"),
        "recording": bool(value(capture, "recording", False)),
        "images": int(sum(counts.values())) if counts else 0,
        "armed": value(telemetry, "armed"),
        "mode": value(telemetry, "mode"),
        "vehicle_connected": bool(value(telemetry, "connected", False)),
        **depth_readouts(status),
    }


def preview_jpeg(image: np.ndarray, features: Optional[np.ndarray]) -> bytes:
    """The VIO image as JPEG with the tracked features as green dots."""
    img = Image.fromarray(np.ascontiguousarray(image)).convert("RGB")
    if features is not None and len(features):
        draw = ImageDraw.Draw(img)
        r = max(3, img.width // 100)       # big enough to survive JPEG's colour subsampling
        for x, y in np.asarray(features, dtype=float)[:, :2]:
            draw.ellipse((x - r, y - r, x + r, y + r), fill=FEATURE_COLOUR, outline=(13, 17, 23))
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


class WidgetState:
    """What the widget shows, written on the GUI thread, read by the server's threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._status: dict = widget_status({})
        self._frame = None
        self._frame_id = 0
        self._jpeg: Optional[bytes] = None
        self._jpeg_id = -1

    def set_status(self, status: dict) -> None:
        snapshot = widget_status(status)
        with self._lock:
            self._status = snapshot

    def set_preview(self, frame) -> None:
        if getattr(frame, "kind", None) != "vio":
            return
        with self._lock:
            self._frame = frame
            self._frame_id += 1

    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def jpeg(self) -> Optional[bytes]:
        """The latest preview as JPEG, encoded once per frame (on the first request for it)."""
        with self._lock:
            frame, frame_id = self._frame, self._frame_id
            if frame is None:
                return None
            if self._jpeg_id == frame_id:
                return self._jpeg
        data = preview_jpeg(frame.image, frame.features)
        with self._lock:
            if self._frame_id == frame_id:
                self._jpeg, self._jpeg_id = data, frame_id
        return data


class WidgetCommands(QObject):
    """Widget → GUI thread. Connect with Qt.QueuedConnection."""
    vioRequested = Signal(bool)        # True: start tracking, False: stop
    recordRequested = Signal(bool)


def _handler(state: WidgetState, commands: WidgetCommands, page: bytes):
    class Handler(BaseHTTPRequestHandler):
        server_version = "TrackShot"

        def log_message(self, fmt, *args):  # noqa: D401 - quiet; failures are logged below
            pass

        def _send(self, code: int, body: bytes = b"", ctype: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.end_headers()
            if body:
                self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj).encode())

        def do_OPTIONS(self):  # noqa: N802 - CORS preflight
            self._send(204)

        def do_GET(self):  # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/widget"):
                self._send(200, page, "text/html; charset=utf-8")
            elif path == "/status":
                self._json(state.status())
            elif path == "/preview.jpg":
                data = state.jpeg()
                self._send(200, data, "image/jpeg") if data else self._send(204)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):  # noqa: N802
            path = self.path.split("?")[0]
            try:
                length = min(int(self.headers.get("Content-Length") or 0), MAX_BODY)
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                self._json({"error": "body must be JSON"}, 400)
                return
            if path == "/vio" and isinstance(body.get("run"), bool):
                commands.vioRequested.emit(body["run"])
            elif path == "/record" and isinstance(body.get("on"), bool):
                commands.recordRequested.emit(body["on"])
            else:
                self._json({"error": "expected POST /vio {\"run\": bool} or /record {\"on\": bool}"}, 400)
                return
            self._json({"ok": True})

    return Handler


class WidgetServer:
    def __init__(self, state: WidgetState, commands: WidgetCommands, page: bytes,
                 port: int, lan: bool = True):
        self._state, self._commands, self._page = state, commands, page
        self.requested_port = int(port)
        self.lan = bool(lan)
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[str] = None

    @property
    def running(self) -> bool:
        return self._httpd is not None

    @property
    def port(self) -> Optional[int]:
        return self._httpd.server_address[1] if self._httpd else None

    def start(self) -> bool:
        host = "0.0.0.0" if self.lan else "127.0.0.1"
        try:
            httpd = ThreadingHTTPServer((host, self.requested_port), _handler(self._state, self._commands, self._page))
        except OSError as e:
            self.error = f"port {self.requested_port} unavailable ({e.strerror or e})"
            logger.error(f"Cockpit widget not served: {self.error}")
            return False
        httpd.daemon_threads = True
        self._httpd = httpd
        self.error = None
        # A short poll so stop() (window close, settings change) doesn't wait half a second
        self._thread = threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.05},
                                        name="trackshot-widget", daemon=True)
        self._thread.start()
        logger.info(f"Cockpit widget: {', '.join(self.urls())}")
        return True

    def stop(self) -> None:
        httpd, self._httpd = self._httpd, None
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None

    def urls(self) -> list[str]:
        """The URLs to paste into Cockpit: this computer first, then its network addresses
        (the tether's 192.168.2.x first). Empty when not running."""
        if self.port is None:
            return []
        urls = [f"http://localhost:{self.port}/widget"]
        if self.lan:
            urls += [f"http://{ip}:{self.port}/widget" for ip in local_ipv4_addresses()]
        return urls


def local_ipv4_addresses() -> list[str]:
    """This computer's IPv4 addresses (not loopback), the BlueOS tether network's first."""
    found: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            found.add(info[4][0])
    except OSError:
        pass
    for probe in ("192.168.2.2", "10.255.255.255"):
        # The address the OS would use to reach these: no packet is sent for a UDP connect
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((probe, 9))
            found.add(s.getsockname()[0])
        except OSError:
            pass
        finally:
            s.close()
    addrs = [a for a in found if not ipaddress.ip_address(a).is_loopback]
    return sorted(addrs, key=lambda a: (not a.startswith("192.168.2."), a))
