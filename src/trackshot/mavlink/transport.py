"""Transports that carry TrackShot's MAVLink messages to the autopilot.

`Mavlink2RestTransport` is the BlueOS route (the extension's `create_websocket_connection` /
`send_mavlink_message_ws`), `UdpTransport` talks MAVLink directly with pymavlink (e.g. to a
mavlink-router endpoint). Both share the same non-blocking shape: `send()` only appends to a
bounded queue; a daemon sender thread drains it, and a daemon connection thread keeps the link up.
The extension blocked its pose loop for up to 5 s reconnecting; here nothing the caller does ever
waits on the network.
"""

from __future__ import annotations

import json
import select
import socket
import threading
import time
from abc import abstractmethod
from collections import deque
from typing import Callable, Optional
from urllib.parse import urlsplit

import websocket
from loguru import logger
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega

from trackshot.mavlink.base import Transport, TransportStats
from trackshot.mavlink.messages import char_fields, encode

# A message that waited longer than this (link down) is dropped instead of sent: a burst of
# seconds-old deltas, stamped with their old time_usec, would be fused at the wrong time.
MAX_QUEUE_AGE_S = 1.0
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 5.0
# TCP connect timeout: an unplugged tether would otherwise hang a connect for the OS default (~75 s)
CONNECT_TIMEOUT_S = 3.0
# WebSocket keep-alive, so a dead tether is noticed even while nothing comes back from the server
WS_PING_INTERVAL_S = 5.0
WS_PING_TIMEOUT_S = 3.0
THREAD_JOIN_S = 2.0

AUTOPILOT_COMPONENT = 1   # MAV_COMP_ID_AUTOPILOT1

# mavlink2rest's websocket streams every vehicle message whose name matches the `filter` regex,
# and the filter defaults to ".*" (src/endpoints.rs). "SEND_ONLY" matches no message name, so the
# server doesn't push the vehicle's whole traffic down the tether to us. Same URL as the extension.
SEND_ONLY_QUERY = "filter=SEND_ONLY"

# How char[N] fields go into mavlink2rest JSON. BlueOS ships mavlink2rest 0.11.x, built on
# rust-mavlink 0.10 where char[N] is `[char; N]`: serde wants an array of exactly N one-character
# strings (the extension's `_encode_id` pads PARAM_SET.param_id that way). mavlink2rest ≥ 1.0
# (rust-mavlink ≥ 0.14, `CharArray<N>`) wants a plain string and rejects the array.
CHARS_AS_LIST = "list"
CHARS_AS_STRING = "string"
# mavlink2rest's reply to a frame it couldn't deserialize (both versions)
M2R_PARSE_ERROR = "Could not convert input message"


def with_send_only_filter(ws_url: str) -> str:
    """Appends the SEND_ONLY filter unless the URL already has a query (an explicit choice)."""
    return ws_url if urlsplit(ws_url).query else f"{ws_url}?{SEND_ONLY_QUERY}"


def encode_char_arrays(msg_type: str, fields: dict, mode: str) -> dict:
    """Copy of `fields` with each char[N] string in mavlink2rest's form for `mode`."""
    chars = char_fields(msg_type)
    if not chars or mode == CHARS_AS_STRING:
        return dict(fields)
    out = dict(fields)
    for name, length in chars.items():
        value = out.get(name)
        if isinstance(value, str):
            out[name] = list(value[:length].ljust(length, "\u0000"))
    return out


def classify_m2r_reply(text: str) -> Optional[str]:
    """mavlink2rest answers every frame we send. Returns None if `text` isn't such an answer
    (vehicle traffic when a filter lets it through), "" for success, else the error text.

    0.11.x answers with Rust Debug text ("Ok(())", "Err(...)" or the bare parse error);
    1.x with JSON ({"result": "ok"} or {"error": "..."}).
    """
    try:
        data = json.loads(text)
    except ValueError:
        return "" if text.startswith("Ok") else text
    if not isinstance(data, dict) or "header" in data:
        return None
    return str(data["error"]) if "error" in data else ""


class _OutQueue:
    """Bounded FIFO of (enqueued_at, msg_type, fields); full → the oldest entry goes."""

    def __init__(self, size: int):
        self._items: deque = deque()
        self._size = max(1, size)
        self._cond = threading.Condition()

    def put(self, item) -> bool:
        """Adds `item`; True if the oldest entry had to be dropped to make room."""
        with self._cond:
            overflow = len(self._items) >= self._size
            if overflow:
                self._items.popleft()
            self._items.append(item)
            self._cond.notify()
            return overflow

    def get(self, timeout: float):
        with self._cond:
            if not self._items:
                self._cond.wait(timeout)
            return self._items.popleft() if self._items else None

    def drop_older_than(self, cutoff: float) -> int:
        with self._cond:
            dropped = 0
            while self._items and self._items[0][0] < cutoff:
                self._items.popleft()
                dropped += 1
            return dropped

    def wake(self) -> None:
        with self._cond:
            self._cond.notify_all()


class _QueuedTransport(Transport):
    """Queue, sender thread, stats and stop handling shared by both transports."""

    kind = ""

    def __init__(self, target: str, source_system: int, source_component: int, queue_size: int):
        super().__init__(source_system, source_component)
        self.target = target
        self._queue = _OutQueue(queue_size)
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._stats_lock = threading.Lock()
        self._sent = 0
        self._dropped = 0
        self._errors = 0
        self._last_error: Optional[str] = None
        self._threads: list[threading.Thread] = []

    # --- Transport API ----------------------------------------------------------------------
    def start(self) -> None:
        if any(t.is_alive() for t in self._threads):
            return
        self._stop.clear()
        self._threads = [threading.Thread(target=fn, name=f"trackshot-{self.kind}-{name}", daemon=True)
                         for name, fn in self._thread_targets()]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._connected.clear()
        self._close_link()
        self._queue.wake()
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(THREAD_JOIN_S)
        self._threads = []

    def send(self, msg_type: str, fields: dict) -> bool:
        if self._queue.put((time.monotonic(), msg_type, fields)):
            with self._stats_lock:
                self._dropped += 1
        return True

    def stats(self) -> TransportStats:
        with self._stats_lock:
            return TransportStats(
                kind=self.kind, target=self.target, connected=self._connected.is_set(),
                sent=self._sent, dropped=self._dropped, errors=self._errors,
                last_error=self._last_error, heartbeat_age_s=self._heartbeat_age_s(),
            )

    # --- For subclasses ---------------------------------------------------------------------
    @abstractmethod
    def _thread_targets(self) -> list[tuple[str, Callable[[], None]]]:
        """(name, function) of the threads to run; the sender is one of them."""

    @abstractmethod
    def _deliver(self, msg_type: str, fields: dict) -> None:
        """Puts one message on the wire. Raises on failure."""

    @abstractmethod
    def _close_link(self) -> None:
        """Closes the link so blocked I/O returns; called from stop()."""

    def _on_link_failure(self) -> None:
        """A send failed for a reason other than the message: drop the link to reconnect."""
        self._connected.clear()
        self._close_link()

    def _heartbeat_age_s(self) -> Optional[float]:
        return None

    def _record_error(self, message: str) -> None:
        with self._stats_lock:
            self._errors += 1
            self._last_error = message

    def _sender_loop(self) -> None:
        while not self._stop.is_set():
            if not self._connected.wait(0.2):
                self._drop_stale()
                continue
            item = self._queue.get(timeout=0.2)
            if item is None:
                continue
            enqueued_at, msg_type, fields = item
            if time.monotonic() - enqueued_at > MAX_QUEUE_AGE_S:
                with self._stats_lock:
                    self._dropped += 1
                continue
            try:
                self._deliver(msg_type, fields)
            except (ValueError, TypeError, KeyError, AttributeError) as e:
                # The message itself is bad (e.g. a NaN or a wrong field): the link is fine
                self._record_error(f"{msg_type}: {e}")
                logger.warning(f"Couldn't encode {msg_type}: {e}")
            except Exception as e:
                self._record_error(f"{msg_type}: {e}")
                logger.warning(f"Sending {msg_type} over {self.kind} failed: {e}")
                self._on_link_failure()
            else:
                with self._stats_lock:
                    self._sent += 1

    def _drop_stale(self) -> None:
        dropped = self._queue.drop_older_than(time.monotonic() - MAX_QUEUE_AGE_S)
        if dropped:
            with self._stats_lock:
                self._dropped += dropped

    def _backoff_waits(self):
        """1, 2, 4, 5, 5, … s between connection attempts."""
        delay = RECONNECT_MIN_S
        while True:
            yield delay
            delay = min(delay * 2, RECONNECT_MAX_S)


class Mavlink2RestTransport(_QueuedTransport):
    """MAVLink as JSON frames over mavlink2rest's websocket (BlueOS's `/mavlink2rest/ws/mavlink`)."""

    kind = "mavlink2rest"

    def __init__(self, ws_url: str, source_system: int, source_component: int, queue_size: int = 64):
        super().__init__(with_send_only_filter(ws_url), source_system, source_component, queue_size)
        self._app: Optional[websocket.WebSocketApp] = None
        self._app_lock = threading.Lock()
        self._sequence = 0
        self._char_mode = CHARS_AS_LIST
        # (msg_type, char mode used or None) of frames awaiting mavlink2rest's in-order reply
        self._awaiting_reply: deque = deque(maxlen=256)

    def _thread_targets(self):
        return [("ws", self._connection_loop), ("sender", self._sender_loop)]

    def _connection_loop(self) -> None:
        waits = self._backoff_waits()
        while not self._stop.is_set():
            opened = threading.Event()

            def on_open(ws):
                if self._stop.is_set():   # stop() ran while we were connecting
                    ws.close()
                    return
                logger.info(f"Connected to mavlink2rest at {self.target}")
                self._awaiting_reply.clear()
                opened.set()
                self._connected.set()

            def on_close(ws, close_status_code, close_msg):
                self._connected.clear()

            def on_error(ws, error):
                self._connected.clear()
                if not self._stop.is_set():
                    self._record_error(str(error) or type(error).__name__)

            try:
                sock = self._open_socket()
            except OSError as e:
                self._record_error(f"connecting: {e}")
                logger.debug(f"mavlink2rest at {self.target} not reachable: {e}")
                self._stop.wait(next(waits))
                continue
            app = websocket.WebSocketApp(self.target, on_open=on_open, on_close=on_close,
                                         on_error=on_error, on_message=self._on_message,
                                         socket=sock)
            with self._app_lock:
                if self._stop.is_set():
                    if sock is not None:
                        sock.close()
                    break
                self._app = app
            try:
                # reconnect=0: this loop does the reconnecting, with backoff
                app.run_forever(ping_interval=WS_PING_INTERVAL_S, ping_timeout=WS_PING_TIMEOUT_S,
                                reconnect=0)
            except Exception as e:   # run_forever reports through on_error; this is a last resort
                self._record_error(str(e))
            finally:
                # websocket-client skips closing the TCP socket when the server closed first, which
                # leaves the server waiting on a half-closed connection: close it ourselves
                if sock is not None:
                    sock.close()
            self._connected.clear()
            if self._stop.is_set():
                break
            if opened.is_set():
                logger.warning("mavlink2rest connection closed; reconnecting")
                waits = self._backoff_waits()
            self._stop.wait(next(waits))

    def _open_socket(self) -> Optional[socket.socket]:
        """Pre-connects ws:// with a timeout (websocket-client would wait the OS default).
        Returns None for other schemes (wss), which websocket-client connects itself."""
        parts = urlsplit(self.target)
        if parts.scheme != "ws" or not parts.hostname:
            return None
        return socket.create_connection((parts.hostname, parts.port or 80), timeout=CONNECT_TIMEOUT_S)

    def _close_link(self) -> None:
        with self._app_lock:
            app = self._app
        if app is not None:
            try:
                app.close()
            except Exception as e:
                logger.debug(f"Closing the mavlink2rest websocket: {e}")

    def _deliver(self, msg_type: str, fields: dict) -> None:
        mode = self._char_mode
        frame = {
            "header": {
                "system_id": self.source_system,
                "component_id": self.source_component,
                "sequence": self._sequence,
            },
            "message": {"type": msg_type, **encode_char_arrays(msg_type, fields, mode)},
        }
        text = json.dumps(frame, allow_nan=False)   # mavlink2rest can't take NaN/inf as a float
        with self._app_lock:
            app = self._app
        if app is None:
            raise ConnectionError("not connected")
        self._awaiting_reply.append((msg_type, mode if char_fields(msg_type) else None))
        app.send(text)
        self._sequence = (self._sequence + 1) % 256

    def _on_message(self, ws, text) -> None:
        error = classify_m2r_reply(text)
        if error is None:
            return
        msg_type, mode = self._awaiting_reply.popleft() if self._awaiting_reply else ("?", None)
        if not error:
            return
        self._record_error(f"mavlink2rest rejected {msg_type}: {error}")
        if M2R_PARSE_ERROR in error and mode == self._char_mode:
            # The only encoding that differs between mavlink2rest versions is char[N]
            self._char_mode = CHARS_AS_STRING if mode == CHARS_AS_LIST else CHARS_AS_LIST
            logger.warning(f"mavlink2rest couldn't parse {msg_type}; sending char arrays as "
                           f"{self._char_mode} from now on")
        else:
            logger.debug(f"mavlink2rest rejected {msg_type}: {error}")


class UdpTransport(_QueuedTransport):
    """MAVLink 2 over UDP with pymavlink, e.g. "udpout:192.168.2.2:14550"."""

    kind = "udp"

    def __init__(self, url: str, source_system: int, source_component: int, queue_size: int = 64):
        super().__init__(url, source_system, source_component, queue_size)
        self._conn = None
        self._io_lock = threading.Lock()   # pymavlink's MAVLink object isn't thread-safe
        self._last_heartbeat: Optional[float] = None

    def _thread_targets(self):
        return [("rx", self._receive_loop), ("sender", self._sender_loop)]

    def _open(self):
        conn = mavutil.mavlink_connection(self.target, source_system=self.source_system,
                                          source_component=self.source_component)
        # mavutil defaults to MAVLink 1 unless $MAVLINK20 is set, and VISION_POSITION_DELTA (id
        # 11011) only exists in MAVLink 2. Give this connection its own MAVLink 2 codec rather
        # than flipping pymavlink's process-wide dialect.
        conn.mav = ardupilotmega.MAVLink(conn, srcSystem=self.source_system,
                                         srcComponent=self.source_component)
        conn.mav.robust_parsing = True
        conn.WIRE_PROTOCOL_VERSION = "2.0"
        return conn

    def _receive_loop(self) -> None:
        """Opens the connection (retrying with backoff), then reads, tracking autopilot HEARTBEATs."""
        waits = self._backoff_waits()
        while not self._stop.is_set():
            if self._conn is None:
                try:
                    conn = self._open()
                except Exception as e:
                    self._record_error(f"opening {self.target}: {e}")
                    logger.warning(f"Couldn't open {self.target}: {e}")
                    self._stop.wait(next(waits))
                    continue
                with self._io_lock:
                    self._conn = conn
                self._connected.set()
                logger.info(f"MAVLink UDP open: {self.target}")
            self._read_available()

    def _read_available(self) -> None:
        conn = self._conn
        if conn is None:
            return
        try:
            readable, _, _ = select.select([conn.fd], [], [], 0.2)
        except (OSError, ValueError):   # closed by stop()
            self._stop.wait(0.2)
            return
        if not readable:
            return
        while not self._stop.is_set():
            with self._io_lock:
                if self._conn is not conn:
                    return
                try:
                    msg = conn.recv_msg()
                except OSError as e:
                    self._record_error(f"receive: {e}")
                    return
            if msg is None:
                return
            if (msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == self.source_system
                    and msg.get_srcComponent() == AUTOPILOT_COMPONENT):
                with self._stats_lock:
                    self._last_heartbeat = time.monotonic()

    def _deliver(self, msg_type: str, fields: dict) -> None:
        with self._io_lock:
            if self._conn is None:
                raise ConnectionError("not open")
            self._conn.mav.send(encode(self._conn.mav, msg_type, fields))

    def _on_link_failure(self) -> None:
        """A UDP socket has no connection to lose (pymavlink even swallows sendto errors): the
        failure is counted and the socket kept."""

    def _close_link(self) -> None:
        with self._io_lock:
            conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except OSError as e:
                logger.debug(f"Closing {self.target}: {e}")

    def _heartbeat_age_s(self) -> Optional[float]:
        return None if self._last_heartbeat is None else time.monotonic() - self._last_heartbeat


def make_transport(settings) -> Transport:
    if settings.transport == "udp":
        return UdpTransport(settings.udp_url, settings.source_system, settings.source_component)
    return Mavlink2RestTransport(settings.mavlink2rest_ws_url(), settings.source_system,
                                 settings.source_component)
