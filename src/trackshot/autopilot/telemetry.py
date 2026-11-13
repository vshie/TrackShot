"""Reads the ROV's autopilot state from BlueOS's mavlink2rest, for labelling survey images.

Port of the BlueOS extension's `app/vehicle_telemetry.py`. Two things change because TrackShot
runs topside instead of on the Pi:

- **Clocks differ.** mavlink2rest stamps each message with the *Pi's* clock (`last_update`), and
  a Pi in the field often has no NTP, so comparing that stamp with the Mac's clock can be off by
  hours and would make every message look stale. mavlink2rest ≥ 1.1 also serves
  `last_update_seconds_ago`, computed on the Pi, which is used when present. For older servers
  (BlueOS ships 0.11.x) `VehicleTelemetry` estimates the clock offset from the HEARTBEAT stream.
- **HTTP over the tether can stall**, so capture never waits on it: `VehicleTelemetry` polls in
  the background and `latest()` returns the last snapshot immediately.

mavlink2rest's JSON differs between versions: 0.11.x serialises bitmasks as `{"bits": N}`,
1.x as `"FLAG_A | FLAG_B"`. Helpers here accept both.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime, timezone
from typing import Any, Optional

from loguru import logger

# A message older than this is treated as missing
STALE_AFTER_S = 5.0
# Per-request timeout for the poller. mavlink2rest has been measured stalling 1–1.7 s every
# ~40 s (BlueOS SITL, Nov 2026); the poller runs on its own thread, so waiting costs nothing.
REQUEST_TIMEOUT_S = 1.5

MAV_MODE_FLAG_SAFETY_ARMED = 128

# Messages read_autopilot_telemetry() needs, fetched in parallel
TELEMETRY_MESSAGES = ("GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD",
                      "SCALED_PRESSURE2", "DISTANCE_SENSOR", "MISSION_CURRENT", "LOCAL_POSITION_NED")

TELEMETRY_KEYS = ("lat", "lon", "heading_deg", "roll_deg", "pitch_deg",
                  "depth_m", "temperature_c", "altitude_m", "mission_seq",
                  # EKF heading and local position with the wall time each was valid at
                  "yaw_rad", "yaw_time", "local_ne", "local_ne_time")

try:
    from pymavlink.mavutil import mode_mapping_sub as _PYMAVLINK_SUB_MODES
except ImportError:  # pymavlink is a dependency, but a missing table only costs mode names
    _PYMAVLINK_SUB_MODES = {}
# pymavlink's table predates SURFTRAK (ArduSub 4.5, mode 21)
_ARDUSUB_MODES = {**_PYMAVLINK_SUB_MODES, 21: "SURFTRAK"}


def get_message(base: str, name: str, system_id: int = 1, component_id: int = 1,
                timeout: float = 0.5) -> Optional[dict]:
    """Latest `name` message as mavlink2rest returns it ({"message":..., "status":...}), or None.

    mavlink2rest answers an unknown path with HTTP 200 and the body `None`, which isn't JSON:
    that, like any network error, means "no message".
    """
    url = f"{base}/mavlink/vehicles/{system_id}/components/{component_id}/messages/{name}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            message = json.loads(response.read())
    except Exception:
        return None
    return message if isinstance(message, dict) else None


def raw_message_age_s(message: Optional[dict]) -> Optional[float]:
    """Seconds between the server's `last_update` stamp and this host's clock, or None.

    Includes any offset between the two clocks; see `message_age_s`.
    """
    try:
        last = datetime.fromisoformat(message["status"]["time"]["last_update"])
    except (KeyError, TypeError, ValueError):
        return None
    if last.tzinfo is None:
        last = last.replace(tzinfo=timezone.utc)
    return (datetime.now(timezone.utc) - last).total_seconds()


def message_age_s(message: Optional[dict], clock_skew_s: float = 0.0) -> Optional[float]:
    """Seconds since mavlink2rest last updated `message`, or None if it can't be told.

    Uses the server-computed `last_update_seconds_ago` (mavlink2rest ≥ 1.1) when present, which
    no clock offset can disturb; otherwise the `last_update` stamp minus `clock_skew_s`, this
    host's clock minus the server's.
    """
    try:
        seconds_ago = message["status"]["time"].get("last_update_seconds_ago")
    except (KeyError, TypeError, AttributeError):
        seconds_ago = None
    if isinstance(seconds_ago, (int, float)):
        return float(seconds_ago)
    age = raw_message_age_s(message)
    return None if age is None else age - clock_skew_s


def fresh(message: Optional[dict], clock_skew_s: float = 0.0) -> Optional[dict]:
    """The message body if it exists and isn't stale, else None."""
    if message is None:
        return None
    age = message_age_s(message, clock_skew_s)
    if age is not None and age > STALE_AFTER_S:
        return None
    return message.get("message")


def counter(message: Optional[dict]) -> Optional[int]:
    try:
        return message["status"]["time"]["counter"]
    except (KeyError, TypeError):
        return None


def get_messages(base: str, names, timeout: float = 0.5) -> dict[str, Optional[dict]]:
    """Fetches several messages in parallel. Returns {name: raw mavlink2rest reply or None}.

    One short-lived daemon thread per message instead of the extension's shared executor: pool
    workers aren't daemonic and would hold the app open at exit while a request hangs.
    """
    results: dict[str, Optional[dict]] = {name: None for name in names}

    def fetch(name):
        results[name] = get_message(base, name, timeout=timeout)

    threads = [threading.Thread(target=fetch, args=(name,), daemon=True, name=f"m2r-{name}")
               for name in names]
    for thread in threads:
        thread.start()
    deadline = time.monotonic() + timeout + 0.5
    for thread in threads:
        thread.join(max(0.0, deadline - time.monotonic()))
    return dict(results)


def is_armed_heartbeat(heartbeat: dict) -> bool:
    """Armed flag of a HEARTBEAT body, in either mavlink2rest bitmask format."""
    base_mode = heartbeat.get("base_mode")
    if isinstance(base_mode, dict):          # 0.11.x: {"bits": 209}
        base_mode = base_mode.get("bits", 0)
    if isinstance(base_mode, str):           # 1.x: "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED"
        return "MAV_MODE_FLAG_SAFETY_ARMED" in (flag.strip() for flag in base_mode.split("|"))
    return bool(int(base_mode or 0) & MAV_MODE_FLAG_SAFETY_ARMED)


def ardusub_mode_name(custom_mode: Any) -> Optional[str]:
    """ArduSub flight mode name for HEARTBEAT.custom_mode (e.g. 19 → "MANUAL")."""
    if not isinstance(custom_mode, int):
        return None
    return _ARDUSUB_MODES.get(custom_mode, f"MODE_{custom_mode}")


def telemetry_from_messages(raw: dict[str, Optional[dict]], clock_skew_s: float = 0.0) -> dict:
    """The extension's telemetry dict from already-fetched mavlink2rest replies."""
    pos = fresh(raw.get("GLOBAL_POSITION_INT"), clock_skew_s)
    att = fresh(raw.get("ATTITUDE"), clock_skew_s)
    vfr = fresh(raw.get("VFR_HUD"), clock_skew_s)
    press = fresh(raw.get("SCALED_PRESSURE2"), clock_skew_s)
    dist = fresh(raw.get("DISTANCE_SENSOR"), clock_skew_s)
    mission = fresh(raw.get("MISSION_CURRENT"), clock_skew_s)
    local = fresh(raw.get("LOCAL_POSITION_NED"), clock_skew_s)

    t: dict[str, Any] = {key: None for key in TELEMETRY_KEYS}
    if pos:
        lat, lon = pos["lat"] / 1e7, pos["lon"] / 1e7
        # 0,0 means the EKF has no origin yet (Cockpit sets it)
        if lat != 0 or lon != 0:
            t["lat"], t["lon"] = lat, lon
        if pos.get("hdg", 65535) != 65535:
            t["heading_deg"] = pos["hdg"] / 100.0
        t["depth_m"] = -pos["relative_alt"] / 1000.0
    if att:
        t["roll_deg"] = math.degrees(att["roll"])
        t["pitch_deg"] = math.degrees(att["pitch"])
        if t["heading_deg"] is None:
            t["heading_deg"] = math.degrees(att["yaw"]) % 360
    if vfr:
        # ArduSub reports depth as negative altitude
        t["depth_m"] = -vfr["alt"]
    if press:
        t["temperature_c"] = press["temperature"] / 100.0
    if dist and dist.get("current_distance", 0) > 0:
        t["altitude_m"] = dist["current_distance"] / 100.0
    if mission:
        t["mission_seq"] = mission.get("seq")
    # For the horizontal-only VIO output (trackshot.core.ned): the EKF's heading and local position,
    # each with the host wall time it was valid at (None when that can't be told)
    now = time.time()
    if att and math.isfinite(att.get("yaw", math.nan)):
        age = message_age_s(raw.get("ATTITUDE"), clock_skew_s)
        if age is not None:
            t["yaw_rad"], t["yaw_time"] = float(att["yaw"]), now - max(0.0, age)
    if local:
        age = message_age_s(raw.get("LOCAL_POSITION_NED"), clock_skew_s)
        t["local_ne"] = (float(local["x"]), float(local["y"]))
        t["local_ne_time"] = None if age is None else now - max(0.0, age)
    return t


def read_autopilot_telemetry(http_base_url: str, clock_skew_s: float = 0.0) -> dict:
    """Position, attitude, depth and environment for labelling one image. Missing values are None.

    `http_base_url` is mavlink2rest's base, e.g. "http://192.168.2.2/mavlink2rest".
    """
    return telemetry_from_messages(get_messages(http_base_url, TELEMETRY_MESSAGES), clock_skew_s)


class _ClockSkewEstimator:
    """Estimates (host clock − server clock) from a message the server refreshes every second.

    When a message's counter has advanced since the previous poll it is at most ~1 s old, so its
    raw age is the clock offset plus a small positive delay; the minimum over a sliding window is
    the offset. Polls where the counter didn't move are ignored, so a stopped stream can't drag
    the estimate along, and the window lets the estimate follow the Pi's clock being corrected.
    """

    WINDOW_S = 30.0

    def __init__(self):
        self._samples: deque[tuple[float, float]] = deque()
        self._last_counter: Optional[int] = None

    def update(self, message: Optional[dict], now: float) -> None:
        count = counter(message)
        raw_age = raw_message_age_s(message)
        advanced = count is not None and self._last_counter is not None and count != self._last_counter
        if count is not None:
            self._last_counter = count
        if advanced and raw_age is not None:
            self._samples.append((now, raw_age))
        while self._samples and now - self._samples[0][0] > self.WINDOW_S:
            self._samples.popleft()

    @property
    def skew_s(self) -> float:
        # Unknown until two polls have seen the stream move; assume synchronised clocks till then
        return min(age for _, age in self._samples) if self._samples else 0.0


def _empty_snapshot() -> dict:
    snapshot: dict[str, Any] = {key: None for key in TELEMETRY_KEYS}
    snapshot.update(connected=False, armed=None, mode=None, heartbeat_age_s=None)
    return snapshot


class VehicleTelemetry:
    """Polls mavlink2rest in a daemon thread so capture never waits on HTTP.

    `latest()` returns the telemetry keys plus:
      connected        a fresh autopilot HEARTBEAT was seen in the last poll
      armed, mode      from that HEARTBEAT (None when not connected)
      heartbeat_age_s  seconds since the autopilot's last HEARTBEAT, clock offset removed
      age_s            seconds since this snapshot was taken (None before the first poll)
    """

    def __init__(self, http_base_url: str, poll_hz: float = 2.0):
        self.base_url = http_base_url.rstrip("/")
        self.poll_hz = max(0.1, float(poll_hz))
        self._lock = threading.Lock()
        self._snapshot = _empty_snapshot()
        self._snapshot_time: Optional[float] = None   # time.monotonic() of the snapshot
        self._skew = _ClockSkewEstimator()
        self._last_raw: dict[str, dict] = {}         # last successful reply per message name
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="vehicle-telemetry")
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)
            self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def latest(self) -> dict:
        with self._lock:
            snapshot = dict(self._snapshot)
            taken = self._snapshot_time
        snapshot["age_s"] = None if taken is None else time.monotonic() - taken
        return snapshot

    def poll_once(self) -> dict:
        """One poll, run on the caller's thread. Public so tests need no timing."""
        names = (*TELEMETRY_MESSAGES, "HEARTBEAT")
        raw = get_messages(self.base_url, names, timeout=REQUEST_TIMEOUT_S)
        # mavlink2rest on a busy Pi (and BlueOS SITL) stalls for 1–2 s every so often. A request
        # that times out says nothing about the vehicle, so keep the last reply: its last_update
        # stamp keeps ageing, and fresh() declares it stale only once the message really is.
        for name in names:
            if raw[name] is None:
                raw[name] = self._last_raw.get(name)
            else:
                self._last_raw[name] = raw[name]
        self._skew.update(raw["HEARTBEAT"], time.time())
        skew = self._skew.skew_s

        snapshot = _empty_snapshot()
        snapshot.update(telemetry_from_messages(raw, skew))
        heartbeat = fresh(raw["HEARTBEAT"], skew)
        if heartbeat is not None:
            age = message_age_s(raw["HEARTBEAT"], skew)
            snapshot.update(
                connected=True,
                armed=is_armed_heartbeat(heartbeat),
                mode=ardusub_mode_name(heartbeat.get("custom_mode")),
                heartbeat_age_s=None if age is None else max(0.0, age),
            )
        with self._lock:
            was_connected = self._snapshot["connected"]
            self._snapshot = snapshot
            self._snapshot_time = time.monotonic()
        if snapshot["connected"] != was_connected:
            if snapshot["connected"]:
                logger.info(f"Autopilot telemetry connected ({self.base_url})")
            else:
                logger.warning(f"Autopilot telemetry lost: no fresh HEARTBEAT from {self.base_url}")
        return snapshot

    def _run(self) -> None:
        period = 1.0 / self.poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.poll_once()
            except Exception:
                # A malformed reply must not kill the poller; the next poll may be fine
                logger.exception("Telemetry poll failed")
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))
