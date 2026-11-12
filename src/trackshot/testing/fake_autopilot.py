"""An in-process fake of the BlueOS side TrackShot talks to: mavlink2rest (HTTP + WebSocket) in
front of an ArduSub autopilot, plus a MAVLink UDP endpoint. For tests and for running the app with
no ROV.

One HTTP server on 127.0.0.1 serves, under `/mavlink2rest` (and `/mavlink2rest/v1`):
- `GET  /mavlink/vehicles/<sys>/components/<comp>/messages/<NAME>` → the latest message as
  `{"message": {...,"type"}, "status": {"time": {first_update, last_update, counter, frequency}}}`,
  or HTTP 200 with the body `None` when there is none, exactly as mavlink2rest does.
- `POST /mavlink` → a `{"header", "message"}` frame sent "to the vehicle" (HTTP 200, empty body),
  or HTTP 404 with a reason when real mavlink2rest couldn't parse it.
- WebSocket `/ws/mavlink?filter=<regex>&system_id=&component_id=` → frames in, vehicle messages
  matching the filter out, and a reply to every frame (mavlink2rest answers each one).

`flavor` picks which mavlink2rest JSON dialect to speak, since the two differ in ways clients trip on:
- "0.11" (default; what BlueOS ships): char arrays are lists of characters, bitmasks are
  `{"bits": N}`, a WS frame is answered with `Ok(N)` (Rust's Debug of the send result).
- "1.1": char arrays are strings, bitmasks are `"FLAG_A | FLAG_B"`, `status.time` also carries
  `last_update_seconds_ago`, a WS frame is answered `{"result": "ok"}`, and posted messages are
  forwarded to WS clients.
Frames with char arrays or enum fields in the wrong form are rejected as real mavlink2rest would.

The fake autopilot answers PARAM_REQUEST_READ / PARAM_SET from `params` with PARAM_VALUE,
MAV_CMD_REQUEST_MESSAGE for GPS_GLOBAL_ORIGIN and AUTOPILOT_VERSION (which, as on the SITL, is not
in mavlink2rest's store until requested unless `autopilot_version_cached`; `uid`/`uid2` give its
hardware id), and publishes HEARTBEAT (1 Hz), ATTITUDE,
GLOBAL_POSITION_INT, VFR_HUD, SCALED_PRESSURE2, DISTANCE_SENSOR and MISSION_CURRENT (5 Hz) from
`vehicle`. `clock_skew_s` offsets its `last_update` stamps, like a Pi whose clock is wrong.
With `hide_viso_until_reboot`, VISO_* parameters other than VISO_TYPE are hidden (unreadable and
unwritable) unless VISO_TYPE was 1 at the last `reboot()`, as ArduPilot does.

A UDP socket decodes MAVLink with pymavlink and sends a HEARTBEAT to the last peer at 1 Hz, so a
pymavlink `udpout:` client sees the autopilot.

Every message received over any path is recorded; `received(msg_type)` returns them.
"""

from __future__ import annotations

import json
import math
import re
import select
import socket
import threading
import time
import urllib.parse
from dataclasses import dataclass
from datetime import datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

from loguru import logger
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from websockets.frames import Opcode
from websockets.http11 import Request
from websockets.protocol import State
from websockets.server import ServerProtocol

FLAVORS = ("0.11", "1.1")

AUTOPILOT_SYSTEM_ID = 1
AUTOPILOT_COMPONENT_ID = 1

ARDUSUB_MANUAL = 19

# ArduSub's base_mode, disarmed: CUSTOM_MODE_ENABLED | STABILIZE_ENABLED | MANUAL_INPUT_ENABLED
BASE_MODE_DISARMED = 0x51
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MODE_FLAG_NAMES = [(0x01, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"), (0x02, "MAV_MODE_FLAG_TEST_ENABLED"),
                   (0x04, "MAV_MODE_FLAG_AUTO_ENABLED"), (0x08, "MAV_MODE_FLAG_GUIDED_ENABLED"),
                   (0x10, "MAV_MODE_FLAG_STABILIZE_ENABLED"), (0x20, "MAV_MODE_FLAG_HIL_ENABLED"),
                   (0x40, "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED"), (0x80, "MAV_MODE_FLAG_SAFETY_ARMED")]

# Stock ArduSub values for the parameters TrackShot checks, and their MAVLink types
DEFAULT_PARAMS = {
    "AHRS_EKF_TYPE": 3.0,
    "EK3_ENABLE": 1.0,
    "VISO_TYPE": 0.0,
    "EK3_SRC1_POSXY": 3.0,     # GPS
    "EK3_SRC1_VELXY": 3.0,
    "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELZ": 0.0,      # TrackShot wants 0 (the depth sensor alone sets the vertical)
    "EK3_SRC_OPTIONS": 1.0,
    "VISO_DELAY_MS": 10.0,
    "VISO_ORIENT": 0.0,
    "SURFACE_DEPTH": -10.0,
}
DEFAULT_PARAM_TYPES = {
    "EK3_SRC_OPTIONS": "MAV_PARAM_TYPE_INT16",
    "VISO_DELAY_MS": "MAV_PARAM_TYPE_INT16",
    "SURFACE_DEPTH": "MAV_PARAM_TYPE_REAL32",
}
DEFAULT_PARAM_TYPE = "MAV_PARAM_TYPE_INT8"
INTEGER_PARAM_TYPES = {"MAV_PARAM_TYPE_INT8", "MAV_PARAM_TYPE_UINT8", "MAV_PARAM_TYPE_INT16",
                       "MAV_PARAM_TYPE_UINT16", "MAV_PARAM_TYPE_INT32", "MAV_PARAM_TYPE_UINT32"}

MAV_CMD_REQUEST_MESSAGE_NAME = "MAV_CMD_REQUEST_MESSAGE"
GPS_GLOBAL_ORIGIN_MSG_ID = 49
AUTOPILOT_VERSION_MSG_ID = 148
DEFAULT_UID = 0x0042_4C55_4552_4F56   # any non-zero 64-bit id

TELEMETRY_PERIOD_S = 0.2
HEARTBEAT_PERIOD_S = 1.0
TICK_S = 0.05


@dataclass
class VehicleState:
    """What the fake autopilot reports. Change it with FakeAutopilot.update_vehicle()."""
    armed: bool = False
    custom_mode: int = ARDUSUB_MANUAL
    lat: float = -33.8568           # 0, 0 = no EKF origin
    lon: float = 151.2153
    depth_m: float = 4.2
    heading_deg: float = 87.5
    roll_deg: float = 1.5
    pitch_deg: float = -2.0
    temperature_c: float = 18.25
    altitude_m: Optional[float] = 1.5   # rangefinder; None = no DISTANCE_SENSOR
    mission_seq: int = 0
    origin: Optional[tuple[float, float]] = (-33.8570, 151.2150)   # EKF origin, None = not set
    local_ne: Optional[tuple[float, float]] = (12.5, -3.25)          # EKF local position, None = none


def _message_class(name: str):
    return getattr(mavlink2, f"MAVLink_{name.lower()}_message", None)


def _char_fields(cls) -> set[str]:
    lengths = dict(zip(cls.ordered_fieldnames, cls.array_lengths))
    return {f for f, t in zip(cls.fieldnames, cls.fieldtypes) if t == "char" and lengths.get(f, 0) > 0}


def _normalise(value: Any) -> Any:
    """char arrays (lists of 1-character strings, or NUL-padded strings) → str."""
    if isinstance(value, list) and value and all(isinstance(c, str) and len(c) == 1 for c in value):
        return "".join(value).rstrip("\u0000")
    if isinstance(value, str):
        return value.rstrip("\u0000")
    return value


def _chrono_timestamp(t: float) -> str:
    """A timestamp as mavlink2rest writes it: chrono's RFC 3339, local offset, nanoseconds."""
    dt = datetime.fromtimestamp(t).astimezone()
    nanos = int(round((t % 1.0) * 1e9)) % 1_000_000_000
    offset = dt.utcoffset() or timedelta(0)
    sign = "-" if offset < timedelta(0) else "+"
    minutes = abs(int(offset.total_seconds())) // 60
    return f"{dt:%Y-%m-%dT%H:%M:%S}.{nanos:09d}{sign}{minutes // 60:02d}:{minutes % 60:02d}"


class _StoredMessage:
    """One slot in mavlink2rest's store: the latest message plus its `status.time`."""

    def __init__(self, message: dict, now: float):
        self.message = message
        self.first_update = now
        self.last_update = now
        self.counter = 2        # mavlink2rest's Temporal starts at 1 and is updated on insert

    def update(self, message: dict, now: float):
        self.message = message
        self.last_update = now
        self.counter += 1


class _WsClient:
    def __init__(self, sock: socket.socket, protocol: ServerProtocol, regex: Optional[re.Pattern],
                 system_id: Optional[int], component_id: Optional[int]):
        self.sock = sock
        self.protocol = protocol
        self.regex = regex
        self.system_id = system_id
        self.component_id = component_id
        self.lock = threading.Lock()    # the sans-I/O protocol and the socket are shared with senders

    def wants(self, name: str, header: dict) -> bool:
        return (self.regex is not None and self.regex.search(name) is not None
                and self.system_id in (None, header["system_id"])
                and self.component_id in (None, header["component_id"]))

    def send_text(self, text: str) -> None:
        with self.lock:
            if self.protocol.state is not State.OPEN:
                return
            self.protocol.send_text(text.encode())
            self._flush()

    def _flush(self) -> None:
        for data in self.protocol.data_to_send():
            if data:
                self.sock.sendall(data)


class _HttpServer(ThreadingHTTPServer):
    daemon_threads = True
    # socketserver's default backlog of 5 overflows when a poller fetches 7 messages at once;
    # macOS then drops the SYN and the client retries a second later, past its timeout
    request_queue_size = 128


class FakeAutopilot:
    def __init__(self, port: int = 0, udp_port: int = 0, flavor: str = "0.11",
                 params: Optional[dict[str, float]] = None, heartbeat: bool = True,
                 clock_skew_s: float = 0.0, vehicle: Optional[VehicleState] = None,
                 host: str = "127.0.0.1", uid: int = DEFAULT_UID, uid2: Optional[bytes] = None,
                 autopilot_version_cached: bool = False, hide_viso_until_reboot: bool = False):
        if flavor not in FLAVORS:
            raise ValueError(f"flavor must be one of {FLAVORS}")
        self.flavor = flavor
        self.host = host
        self._requested_port = port
        self._requested_udp_port = udp_port
        self.params: dict[str, float] = dict(DEFAULT_PARAMS if params is None else params)
        self.param_types: dict[str, str] = dict(DEFAULT_PARAM_TYPES)
        self.vehicle = vehicle or VehicleState()
        self.heartbeat_enabled = heartbeat
        self.clock_skew_s = clock_skew_s
        self.uid = uid
        self.uid2 = bytes(uid2 or b"")[:18]
        self.autopilot_version_cached = autopilot_version_cached
        self.hide_viso_until_reboot = hide_viso_until_reboot
        self._viso_enabled_at_boot = self.params.get("VISO_TYPE", 0.0) >= 1
        self.reboots = 0

        self._lock = threading.RLock()
        self._store: dict[tuple[int, int, str], _StoredMessage] = {}
        self._received: list[dict] = []
        self._ws_clients: list[_WsClient] = []
        self._boot = time.monotonic()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._http: Optional[_HttpServer] = None
        self._udp: Optional[socket.socket] = None
        self._udp_peer: Optional[tuple[str, int]] = None
        self._mav = mavlink2.MAVLink(None, srcSystem=AUTOPILOT_SYSTEM_ID, srcComponent=AUTOPILOT_COMPONENT_ID)
        self.port = 0
        self.udp_port = 0

    # --- Lifecycle --------------------------------------------------------------------------
    def start(self) -> "FakeAutopilot":
        if self._http is not None:
            return self
        self._stop.clear()
        self._http = _HttpServer((self.host, self._requested_port), self._handler_class())
        self.port = self._http.server_address[1]
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp.bind((self.host, self._requested_udp_port))
        self.udp_port = self._udp.getsockname()[1]
        self._publish_vehicle(heartbeat=True)
        if self.autopilot_version_cached:
            self._publish(self._autopilot_version_message())
        for target, name in ((lambda: self._http.serve_forever(poll_interval=0.05), "fake-m2r-http"),
                             (self._udp_loop, "fake-m2r-udp"),
                             (self._vehicle_loop, "fake-m2r-vehicle")):
            thread = threading.Thread(target=target, daemon=True, name=name)
            thread.start()
            self._threads.append(thread)
        return self

    def stop(self) -> None:
        if self._http is None:
            return
        self._stop.set()
        self._http.shutdown()
        self._http.server_close()
        with self._lock:
            clients, self._ws_clients = self._ws_clients, []
        for client in clients:
            try:
                client.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        for thread in self._threads:
            thread.join(2.0)
        self._threads = []
        self._udp.close()
        self._http = None
        self._udp = None

    def __enter__(self) -> "FakeAutopilot":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # --- Addresses --------------------------------------------------------------------------
    @property
    def vehicle_ip(self) -> str:
        return f"{self.host}:{self.port}"

    @property
    def http_base(self) -> str:
        return f"http://{self.vehicle_ip}/mavlink2rest"

    @property
    def ws_url(self) -> str:
        return f"ws://{self.vehicle_ip}/mavlink2rest/ws/mavlink"

    @property
    def udp_url(self) -> str:
        return f"udpout:{self.host}:{self.udp_port}"

    # --- Control and inspection -------------------------------------------------------------
    def set_armed(self, armed: bool) -> None:
        self.update_vehicle(armed=armed)

    def update_vehicle(self, **changes) -> None:
        """Changes VehicleState fields and publishes the result at once."""
        with self._lock:
            for name, value in changes.items():
                if not hasattr(self.vehicle, name):
                    raise AttributeError(f"VehicleState has no field {name!r}")
                setattr(self.vehicle, name, value)
        self._publish_vehicle(heartbeat=True)

    def reboot(self) -> None:
        """Restarts the fake autopilot: parameters that take effect at boot (here, whether the
        VISO_* group exists) are re-read."""
        with self._lock:
            self._viso_enabled_at_boot = self.params.get("VISO_TYPE", 0.0) >= 1
            self._boot = time.monotonic()
            self.reboots += 1

    def visible(self, name: str) -> bool:
        """Whether the autopilot reports parameter `name` (it is in `params` and not hidden)."""
        if name not in self.params:
            return False
        if (self.hide_viso_until_reboot and name.startswith("VISO_") and name != "VISO_TYPE"
                and not self._viso_enabled_at_boot):
            return False
        return True

    def received(self, msg_type: Optional[str] = None) -> list[dict]:
        """Messages received over HTTP, WebSocket or UDP, oldest first.

        Each is the message's fields with char arrays as str, plus `type`, `_via`
        ("http" | "ws" | "udp"), `_header` ({system_id, component_id}), `_time` (host time.time())
        and `_raw` (the JSON frame as sent, or the MAVLink packet bytes for UDP).
        """
        with self._lock:
            return [m for m in self._received if msg_type is None or m["type"] == msg_type]

    def clear_received(self) -> list[dict]:
        """Forgets the recorded messages; returns them, so nothing slips between read and clear."""
        with self._lock:
            taken, self._received = self._received, []
        return taken

    # --- mavlink2rest store -----------------------------------------------------------------
    def _publish(self, message: dict, system_id: int = AUTOPILOT_SYSTEM_ID,
                 component_id: int = AUTOPILOT_COMPONENT_ID, to_ws: bool = True) -> None:
        name = message["type"]
        now = time.time()
        with self._lock:
            key = (system_id, component_id, name)
            if key in self._store:
                self._store[key].update(message, now)
            else:
                self._store[key] = _StoredMessage(message, now)
            clients = list(self._ws_clients)
        header = {"system_id": system_id, "component_id": component_id, "sequence": 0}
        wanted = [c for c in clients if c.wants(name, header)] if to_ws else []
        if wanted:
            text = json.dumps({"header": header, "message": message}, indent=2)
            for client in wanted:
                try:
                    client.send_text(text)
                except OSError:
                    pass    # the client's own loop notices and removes it

    def _stored_json(self, system_id: int, component_id: int, name: str) -> Optional[str]:
        with self._lock:
            stored = self._store.get((system_id, component_id, name))
            if stored is None:
                return None
            now = time.time()
            # Rust: counter / whole seconds since first update; f32 division by 0 → inf → null
            whole_s = int(stored.last_update - stored.first_update)
            frequency = stored.counter / whole_s if whole_s > 0 else None
            status_time = {
                "first_update": _chrono_timestamp(stored.first_update + self.clock_skew_s),
                "last_update": _chrono_timestamp(stored.last_update + self.clock_skew_s),
            }
            if self.flavor == "1.1":
                status_time["last_update_seconds_ago"] = round(now - stored.last_update, 3)
            status_time.update(counter=stored.counter, frequency=frequency)
            return json.dumps({"message": stored.message, "status": {"time": status_time}}, indent=2)

    # --- Message encoding in the flavour's dialect ------------------------------------------
    def _chars(self, text: str, length: int) -> Any:
        if self.flavor == "1.1":
            return text
        return list(text.ljust(length, "\u0000")[:length])

    def _bits(self, value: int) -> Any:
        if self.flavor == "1.1":
            return " | ".join(name for bit, name in MODE_FLAG_NAMES if value & bit)
        return {"bits": value}

    def _validate(self, message: dict) -> Optional[str]:
        """Why real mavlink2rest would fail to parse `message`, or None if it would parse."""
        name = message.get("type")
        cls = _message_class(name) if isinstance(name, str) else None
        if cls is None:
            return f"unknown message type {name!r}"
        for field in _char_fields(cls):
            value = message.get(field)
            if value is None:
                continue
            if self.flavor == "1.1" and not isinstance(value, str):
                return f"{name}.{field}: invalid type: sequence, expected a string"
            if self.flavor == "0.11" and not isinstance(value, list):
                return f"{name}.{field}: invalid type: string, expected an array"
        for field in getattr(cls, "fieldenums_by_name", {}):
            if isinstance(message.get(field), (int, float)):
                return f"{name}.{field}: expected an enum object, got a number"
        return None

    # --- The fake autopilot -----------------------------------------------------------------
    def _boot_ms(self) -> int:
        return int((time.monotonic() - self._boot) * 1000)

    def _heartbeat_message(self) -> dict:
        base_mode = BASE_MODE_DISARMED | (MAV_MODE_FLAG_SAFETY_ARMED if self.vehicle.armed else 0)
        return {"type": "HEARTBEAT", "custom_mode": self.vehicle.custom_mode,
                "mavtype": {"type": "MAV_TYPE_SUBMARINE"},
                "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                "base_mode": self._bits(base_mode),
                "system_status": {"type": "MAV_STATE_ACTIVE"}, "mavlink_version": 3}

    def _telemetry_messages(self) -> list[dict]:
        v = self.vehicle
        boot_ms = self._boot_ms()
        messages = [
            {"type": "ATTITUDE", "time_boot_ms": boot_ms, "roll": math.radians(v.roll_deg),
             "pitch": math.radians(v.pitch_deg),
             # MAVLink yaw is -pi..pi
             "yaw": math.radians((v.heading_deg + 180.0) % 360.0 - 180.0),
             "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0},
            {"type": "GLOBAL_POSITION_INT", "time_boot_ms": boot_ms,
             "lat": int(round(v.lat * 1e7)), "lon": int(round(v.lon * 1e7)),
             "alt": int(round(-v.depth_m * 1000)), "relative_alt": int(round(-v.depth_m * 1000)),
             "vx": 0, "vy": 0, "vz": 0, "hdg": int(round(v.heading_deg * 100)) % 36000},
            {"type": "VFR_HUD", "airspeed": 0.0, "groundspeed": 0.0,
             "heading": int(round(v.heading_deg)) % 360, "throttle": 0, "alt": -v.depth_m, "climb": 0.0},
            {"type": "SCALED_PRESSURE2", "time_boot_ms": boot_ms,
             "press_abs": 1013.25 + v.depth_m * 100.5, "press_diff": 0.0,
             "temperature": int(round(v.temperature_c * 100)), "temperature_press_diff": 0},
            {"type": "MISSION_CURRENT", "seq": v.mission_seq, "total": 0,
             "mission_state": {"type": "MISSION_STATE_UNKNOWN"},
             "mission_mode": 0, "mission_id": 0, "fence_id": 0, "rally_points_id": 0},
        ]
        if v.local_ne is not None:
            messages.append({"type": "LOCAL_POSITION_NED", "time_boot_ms": boot_ms, "x": v.local_ne[0],
                             "y": v.local_ne[1], "z": v.depth_m, "vx": 0.0, "vy": 0.0, "vz": 0.0})
        if v.altitude_m is not None:
            messages.append(
                {"type": "DISTANCE_SENSOR", "time_boot_ms": boot_ms, "min_distance": 20,
                 "max_distance": 5000, "current_distance": int(round(v.altitude_m * 100)),
                 "mavtype": {"type": "MAV_DISTANCE_SENSOR_ULTRASOUND"}, "id": 1,
                 "orientation": {"type": "MAV_SENSOR_ROTATION_PITCH_270"}, "covariance": 0,
                 "horizontal_fov": 0.0, "vertical_fov": 0.0, "quaternion": [0.0, 0.0, 0.0, 0.0],
                 "signal_quality": 0})
        return messages

    def _publish_vehicle(self, heartbeat: bool) -> None:
        with self._lock:
            messages = self._telemetry_messages()
            if heartbeat and self.heartbeat_enabled:
                messages.insert(0, self._heartbeat_message())
        for message in messages:
            self._publish(message)

    def _vehicle_loop(self) -> None:
        next_telemetry = next_heartbeat = time.monotonic()
        while not self._stop.wait(TICK_S):
            now = time.monotonic()
            heartbeat = now >= next_heartbeat
            if heartbeat:
                next_heartbeat = now + HEARTBEAT_PERIOD_S
                self._udp_heartbeat()
            if heartbeat or now >= next_telemetry:
                next_telemetry = now + TELEMETRY_PERIOD_S
                self._publish_vehicle(heartbeat)

    def _autopilot_version_message(self) -> dict:
        capabilities = 0x0000_E1EF    # roughly ArduPilot's MAV_PROTOCOL_CAPABILITY bits
        return {"type": "AUTOPILOT_VERSION",
                "capabilities": ({"bits": capabilities} if self.flavor == "0.11"
                                 else "MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT"),
                "flight_sw_version": 0x0405_00FF, "middleware_sw_version": 0, "os_sw_version": 0,
                "board_version": 0, "flight_custom_version": [0] * 8,
                "middleware_custom_version": [0] * 8, "os_custom_version": [0] * 8,
                "vendor_id": 0, "product_id": 0, "uid": int(self.uid),
                "uid2": list(self.uid2.ljust(18, b"\0"))}

    def _param_names(self) -> list[str]:
        return [name for name in self.params if self.visible(name)]

    def _param_value(self, name: str) -> dict:
        names = self._param_names()
        return {"type": "PARAM_VALUE", "param_value": float(self.params[name]),
                "param_count": len(names), "param_index": names.index(name),
                "param_id": self._chars(name, 16),
                "param_type": {"type": self.param_types.get(name, DEFAULT_PARAM_TYPE)}}

    def _act_on(self, message: dict) -> None:
        """The autopilot's reaction to a message addressed to it."""
        kind = message["type"]
        reply = None
        with self._lock:
            if kind == "PARAM_REQUEST_READ":
                name = _normalise(message.get("param_id"))
                index = message.get("param_index", -1)
                if isinstance(index, int) and index >= 0:
                    names = self._param_names()
                    name = names[index] if index < len(names) else None
                # ArduPilot ignores requests for parameters it doesn't have
                if self.visible(name):
                    reply = self._param_value(name)
            elif kind == "PARAM_SET":
                name = _normalise(message.get("param_id"))
                if self.visible(name):
                    value = float(message.get("param_value", 0.0))
                    if self.param_types.get(name, DEFAULT_PARAM_TYPE) in INTEGER_PARAM_TYPES:
                        value = float(int(round(value)))
                    self.params[name] = value
                    reply = self._param_value(name)
            elif kind == "COMMAND_LONG":
                command = message.get("command")
                command = command.get("type") if isinstance(command, dict) else command
                requested = int(message.get("param1", 0))
                if command != MAV_CMD_REQUEST_MESSAGE_NAME:
                    pass
                elif requested == GPS_GLOBAL_ORIGIN_MSG_ID and self.vehicle.origin is not None:
                    lat, lon = self.vehicle.origin
                    reply = {"type": "GPS_GLOBAL_ORIGIN", "latitude": int(round(lat * 1e7)),
                             "longitude": int(round(lon * 1e7)), "altitude": 0, "time_usec": 0}
                elif requested == AUTOPILOT_VERSION_MSG_ID:
                    reply = self._autopilot_version_message()
        if reply is not None:
            self._publish(reply)

    def _record(self, message: dict, header: dict, via: str, raw: Any) -> None:
        entry = {key: _normalise(value) for key, value in message.items()}
        entry.update(_via=via, _header=header, _time=time.time(), _raw=raw)
        with self._lock:
            self._received.append(entry)

    def _receive_frame(self, text: str, via: str) -> Optional[str]:
        """Handles one JSON frame from HTTP or WS. Returns the parse error, or None if accepted."""
        try:
            frame = json.loads(text)
            header, message = frame["header"], frame["message"]
            header = {"system_id": int(header["system_id"]), "component_id": int(header["component_id"])}
        except (ValueError, KeyError, TypeError) as e:
            return f"Failed to parse message, not a valid MAVLinkMessage: {e!r}"
        if not isinstance(message, dict):
            return "Failed to parse message, not a valid MAVLinkMessage: message is not an object"
        error = self._validate(message)
        if error:
            return f"Failed to parse message, not a valid MAVLinkMessage: {error}"
        self._record(message, header, via, frame)
        # mavlink2rest files what it sends under the sender's ids; 1.x also tells WS clients
        self._publish(message, header["system_id"], header["component_id"], to_ws=self.flavor == "1.1")
        self._act_on(message)
        return None

    # --- UDP --------------------------------------------------------------------------------
    def _udp_loop(self) -> None:
        parser = mavlink2.MAVLink(None)
        parser.robust_parsing = True
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select([self._udp], [], [], 0.1)
                if not readable:
                    continue
                data, peer = self._udp.recvfrom(65535)
            except (OSError, ValueError):
                return  # socket closed by stop()
            self._udp_peer = peer
            try:
                messages = parser.parse_buffer(data) or []
            except mavlink2.MAVError as e:
                logger.debug(f"fake autopilot: bad UDP packet: {e}")
                continue
            for msg in messages:
                if msg.get_type() == "BAD_DATA":
                    continue
                fields = msg.to_dict()
                fields["type"] = fields.pop("mavpackettype")
                header = {"system_id": msg.get_srcSystem(), "component_id": msg.get_srcComponent()}
                self._record(fields, header, "udp", bytes(msg.get_msgbuf()))

    def _udp_heartbeat(self) -> None:
        peer = self._udp_peer
        if peer is None or not self.heartbeat_enabled or self._udp is None:
            return
        base_mode = BASE_MODE_DISARMED | (MAV_MODE_FLAG_SAFETY_ARMED if self.vehicle.armed else 0)
        msg = self._mav.heartbeat_encode(mavlink2.MAV_TYPE_SUBMARINE, mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                         base_mode, self.vehicle.custom_mode, mavlink2.MAV_STATE_ACTIVE)
        try:
            self._udp.sendto(msg.pack(self._mav), peer)
        except OSError:
            pass

    # --- HTTP / WebSocket -------------------------------------------------------------------
    def _handler_class(self):
        fake = self

        class Handler(BaseHTTPRequestHandler):
            # Unbuffered, so no WebSocket bytes can hide in a read buffer after the handshake
            rbufsize = 0
            server_version = "FakeMavlink2Rest"

            def log_message(self, format, *args):
                pass

            def _route(self) -> tuple[Optional[str], dict]:
                url = urllib.parse.urlsplit(self.path)
                path = url.path
                if not path.startswith("/mavlink2rest"):
                    return None, {}
                path = path[len("/mavlink2rest"):]
                if path.startswith("/v1/"):
                    path = path[len("/v1"):]
                return path.rstrip("/"), urllib.parse.parse_qs(url.query)

            def _reply(self, status: int, body: str, content_type: str = "application/json"):
                data = body.encode()
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path, query = self._route()
                if path == "/ws/mavlink" and self.headers.get("Upgrade", "").lower() == "websocket":
                    self._websocket(query)
                    return
                match = re.fullmatch(r"/mavlink/vehicles/(\d+)/components/(\d+)/messages/(\w+)", path or "")
                if match is None:
                    self._reply(404, "File does not exist", "text/plain")
                    return
                body = fake._stored_json(int(match[1]), int(match[2]), match[3])
                self._reply(200, "None" if body is None else body)

            def do_POST(self):
                path, _ = self._route()
                if path != "/mavlink":
                    self._reply(404, "File does not exist", "text/plain")
                    return
                length = int(self.headers.get("Content-Length", 0))
                text = self.rfile.read(length).decode(errors="replace")
                error = fake._receive_frame(text, "http")
                if error:
                    self._reply(404, error)
                else:
                    self._reply(200, "")

            def _websocket(self, query: dict):
                # http.server already consumed the request; replay it into websockets' sans-I/O
                # protocol, whose parser starts by reading the handshake request
                lines = [self.requestline] + [f"{k}: {v}" for k, v in self.headers.items()]
                protocol = ServerProtocol()
                protocol.receive_data(("\r\n".join(lines) + "\r\n\r\n").encode("latin-1"))
                requests = [e for e in protocol.events_received() if isinstance(e, Request)]
                if not requests:
                    self._reply(400, "Bad WebSocket handshake", "text/plain")
                    return
                response = protocol.accept(requests[0])
                protocol.send_response(response)
                for data in protocol.data_to_send():
                    self.connection.sendall(data)
                if response.status_code != 101:
                    return
                filter_ = query.get("filter", [".*"])[0]
                try:
                    regex = re.compile(filter_)
                except re.error:
                    regex = None    # mavlink2rest: an invalid filter matches nothing
                ids = [int(query[k][0]) if k in query else None for k in ("system_id", "component_id")]
                client = _WsClient(self.connection, protocol, regex, *ids)
                with fake._lock:
                    fake._ws_clients.append(client)
                try:
                    self._websocket_loop(client)
                finally:
                    with fake._lock:
                        if client in fake._ws_clients:
                            fake._ws_clients.remove(client)
                    self.close_connection = True

            def _websocket_loop(self, client: _WsClient):
                fragments: list[bytes] = []
                sock = client.sock
                while not fake._stop.is_set():
                    try:
                        readable, _, _ = select.select([sock], [], [], 0.1)
                        if not readable:
                            continue
                        data = sock.recv(65536)
                    except (OSError, ValueError):
                        return
                    with client.lock:
                        if data:
                            client.protocol.receive_data(data)
                        else:
                            client.protocol.receive_eof()
                        events = client.protocol.events_received()
                        try:
                            client._flush()     # pongs and the closing handshake
                        except OSError:
                            return
                    for frame in events:
                        if frame.opcode in (Opcode.TEXT, Opcode.CONT):
                            fragments.append(frame.data)
                            if frame.fin:
                                text = b"".join(fragments).decode(errors="replace")
                                fragments = []
                                client.send_text(fake._ws_answer(text))
                    if not data or client.protocol.state is State.CLOSED:
                        return

        return Handler

    def _ws_answer(self, text: str) -> str:
        """mavlink2rest's reply to one WS frame, in this flavour."""
        error = self._receive_frame(text, "ws")
        if self.flavor == "1.1":
            if error:
                return json.dumps({"error": "Could not convert input message."})
            return json.dumps({"result": "ok"})
        if error:
            return "Could not convert input message."
        return f"Ok({len(text) // 4})"   # Rust's Debug of Ok(bytes sent); the count is not meaningful
