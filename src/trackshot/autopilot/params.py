"""Checks and sets the autopilot's EKF parameters for TrackShot's VISION_POSITION_DELTA, and reads
the EKF origin. Port of the BlueOS extension's `app/autopilot_params.py`, talking to BlueOS's
mavlink2rest over HTTP.

The origin itself comes from the GPS's first 3D fix at the surface (or is set by hand from the
GCS); TrackShot only reads it.

Differences from the extension, all forced by real mavlink2rest behaviour:
- mavlink2rest answers a message it hasn't seen with HTTP 200 and the body `None` (not 404), so a
  missing HEARTBEAT is NoHeartbeat here, not "unreachable".
- `char[N]` fields: mavlink2rest 0.11.x (what BlueOS ships) takes a list of characters,
  1.x takes a string. The list form is tried first and the other on a parse rejection (HTTP 404);
  the working form is remembered.
- `check()` returns every parameter as a row and can include VISO_DELAY_MS, the delay TrackShot
  suggests from its measurements (trackshot.core.odometry.viso_delay_estimate_ms: capture→host
  latency + half the frame interval + transit), since that delay is now the Mac's.

Undo: `apply(diffs, backup)` records each parameter's value from before TrackShot first changed it
in a `trackshot.autopilot.backup.ParamBackup` (keyed by `vehicle_key()`), and `revert(backup)`
puts those values back, newest change first. `preview_revert(backup)` shows what that would do.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional

from loguru import logger

from trackshot.autopilot.backup import BackupError, ParamBackup, safe_key
from trackshot.autopilot.telemetry import is_armed_heartbeat

# (name, value, takes effect only after an autopilot reboot)
VISUAL_ODOMETRY_PARAMS = [
    ("AHRS_EKF_TYPE", 3, True),   # EKF3
    ("EK3_ENABLE", 1, True),
    ("VISO_TYPE", 1, True),       # MAVLink visual odometry
    ("EK3_SRC1_POSXY", 6, False),  # ExternalNav
    ("EK3_SRC1_VELXY", 6, False),  # ExternalNav
    ("EK3_SRC1_POSZ", 1, False),   # Baro
    # No vertical velocity source: the depth sensor alone sets the vertical. TrackShot's default
    # output (VISION_SPEED_ESTIMATE) is then fused horizontally only; see trackshot.core.ned.
    ("EK3_SRC1_VELZ", 0, False),
    # TrackShot sends motion already in the vehicle's frame (the camera's mounting angle is
    # applied in the app), so the autopilot must not rotate it again
    ("VISO_ORIENT", 0, False),
    # Not checked: VISO_QUAL_MIN must stay at its default 0, since VISION_POSITION/SPEED_ESTIMATE
    # carry no quality (ArduSub reads 0). VISO_* parameters only exist after VISO_TYPE is set and
    # the autopilot rebooted, so they can't be written in the same apply.
    # Bit 0 ("fuse all velocities") would blend GPS velocity in with the VIO's. GPS stays enabled
    # so a surface fix still sets the EKF origin.
    ("EK3_SRC_OPTIONS", 0, False),
]

VISO_DELAY_PARAM = "VISO_DELAY_MS"
VISO_DELAY_RANGE_MS = (0, 250)     # ArduPilot's documented range for VISO_DELAY_MS
VISO_DELAY_TOLERANCE_MS = 10       # the suggestion is rounded to 10 ms; closer isn't worth a write

GPS_GLOBAL_ORIGIN_MSG_ID = 49
AUTOPILOT_VERSION_MSG_ID = 148

# Sender identity for parameter traffic: a ground station's, as Mission Planner/QGC use.
# The extension used component 0, but 0 is MAV_COMP_ID_ALL, a target address that is not meant
# as a source. ArduPilot doesn't filter the parameter protocol by sender and broadcasts its
# PARAM_VALUE replies, and mavlink2rest forwards any header, so both work; this one is valid.
# It must not be the autopilot's own (1/1): mavlink2rest files a posted message under its
# header's ids, so a PARAM_SET would overwrite the autopilot's PARAM_VALUE slot in its store.
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190             # MAV_COMP_ID_MISSIONPLANNER

PARAM_ID_LEN = 16

CHARS_AS_LIST = "list"             # mavlink2rest 0.11.x: ["V","I","S","O",...,"\u0000"]
CHARS_AS_STR = "str"               # mavlink2rest 1.x:    "VISO_TYPE"


class AutopilotUnreachable(Exception):
    """mavlink2rest itself could not be reached."""


class NoHeartbeat(Exception):
    """mavlink2rest is up, but no autopilot HEARTBEAT has been seen."""


class VehicleArmed(Exception):
    """Parameters are never changed while armed."""


class Cancelled(Exception):
    """A read was abandoned because `AutopilotParams.cancel` was set (e.g. the app is closing)."""


def suggest_viso_delay_ms(latency_ms: Optional[float]) -> Optional[int]:
    """VISO_DELAY_MS for a delay estimate in ms (see trackshot.core.odometry.viso_delay_estimate_ms):
    nearest 10 ms, within ArduPilot's range."""
    if latency_ms is None or not math.isfinite(latency_ms):
        return None
    rounded = int(math.floor(latency_ms / 10.0 + 0.5)) * 10   # half up, not banker's rounding
    low, high = VISO_DELAY_RANGE_MS
    return max(low, min(high, rounded))


def _same(a: Optional[float], b: Optional[float]) -> bool:
    """Parameter values equal as the autopilot stores them (REAL32: about 7 significant digits)."""
    if a is None or b is None:
        return a is None and b is None
    return math.isclose(float(a), float(b), rel_tol=1e-6, abs_tol=1e-9)


def _fmt(value: Optional[float]) -> str:
    return "?" if value is None else f"{value:g}"


def _decode_id(param_id) -> str:
    if isinstance(param_id, list):
        param_id = "".join(c for c in param_id if isinstance(c, str))
    return (param_id or "").rstrip("\u0000")


class AutopilotParams:
    def __init__(self, http_base_url: str, system_id: int = 1, component_id: int = 1,
                 timeout: float = 3.0):
        self.base_url = http_base_url.rstrip("/")
        self.system_id = system_id
        self.component_id = component_id
        self.timeout = timeout
        # Set to abandon reads (check()) between requests: a check against an autopilot that
        # doesn't answer takes a minute, and must not hold the app open after its window closes.
        # apply() and revert() ignore it (they stop only when the vehicle arms or a restore
        # fails); the app waits for them before it exits.
        self.cancel = threading.Event()
        self._char_style = CHARS_AS_LIST

    # --- mavlink2rest -----------------------------------------------------------------------
    def _get(self, path: str):
        with urllib.request.urlopen(f"{self.base_url}{path}", timeout=self.timeout) as response:
            return json.loads(response.read())

    def _send(self, message: dict) -> None:
        payload = {
            "header": {"system_id": GCS_SYSTEM_ID, "component_id": GCS_COMPONENT_ID, "sequence": 0},
            "message": message,
        }
        request = urllib.request.Request(
            f"{self.base_url}/mavlink",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            response.read()

    def _post(self, message: dict) -> None:
        """POSTs `message`, whose `param_id` (if any) is a plain str.

        Raises AutopilotUnreachable on network errors, RuntimeError if mavlink2rest rejects it.
        """
        if "param_id" not in message:
            styles = [None]
        else:
            other = CHARS_AS_STR if self._char_style == CHARS_AS_LIST else CHARS_AS_LIST
            styles = [self._char_style, other]
        rejection = None
        for style in styles:
            encoded = dict(message)
            if style == CHARS_AS_LIST:
                encoded["param_id"] = list(message["param_id"].ljust(PARAM_ID_LEN, "\u0000"))
            try:
                self._send(encoded)
            except urllib.error.HTTPError as e:
                # 404 is how mavlink2rest says "could not parse that message"
                rejection = f"HTTP {e.code}: {e.read()[:200].decode(errors='replace')}"
                if e.code != 404:
                    break
                continue
            except (urllib.error.URLError, OSError) as e:
                raise AutopilotUnreachable(f"Could not reach mavlink2rest at {self.base_url}: {e}") from e
            if style is not None and style != self._char_style:
                logger.info(f"mavlink2rest takes char arrays as {style}")
                self._char_style = style
            return
        raise RuntimeError(f"mavlink2rest rejected {message['type']}: {rejection}")

    def _vehicle_message(self, name: str) -> Optional[dict]:
        try:
            message = self._get(f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/{name}")
        except Exception:
            return None
        return message if isinstance(message, dict) else None

    def _message_counter(self, name: str) -> int:
        message = self._vehicle_message(name)
        return message["status"]["time"]["counter"] if message else 0

    def _wait_param_value(self, name: str, previous_counter: int,
                          cancellable: bool = False) -> Optional[tuple[float, str]]:
        """Waits for a PARAM_VALUE for `name` newer than `previous_counter`. Returns (value, type) or None."""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            message = self._vehicle_message("PARAM_VALUE")
            if message and message["status"]["time"]["counter"] > previous_counter:
                content = message["message"]
                if _decode_id(content["param_id"]) == name:
                    return content["param_value"], content["param_type"]["type"]
            if not cancellable:
                time.sleep(0.05)
            elif self.cancel.wait(0.05):
                raise Cancelled(f"reading {name}")
        return None

    def _request_param(self, name: str, message: dict, attempts: int = 3,
                       cancellable: bool = False) -> Optional[tuple[float, str]]:
        # mavlink2rest only keeps the latest PARAM_VALUE, so our reply can be overwritten by another one
        for _ in range(attempts):
            if cancellable and self.cancel.is_set():
                raise Cancelled(f"reading {name}")
            counter = self._message_counter("PARAM_VALUE")
            self._post(message)
            result = self._wait_param_value(name, counter, cancellable)
            if result is not None:
                return result
        return None

    def _read(self, name: str, cancellable: bool = True) -> Optional[tuple[float, str]]:
        return self._request_param(name, {
            "type": "PARAM_REQUEST_READ",
            "param_index": -1,
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": name,
        }, cancellable=cancellable)

    def _write(self, name: str, value: float, param_type: str) -> Optional[tuple[float, str]]:
        return self._request_param(name, {
            "type": "PARAM_SET",
            "param_value": float(value),
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": name,
            "param_type": {"type": param_type},
        })

    # --- Public API -------------------------------------------------------------------------
    def read(self, name: str) -> Optional[float]:
        """The parameter's value, or None if the autopilot didn't report it."""
        result = self._read(name)
        return None if result is None else result[0]

    def write(self, name: str, value: float, param_type: str) -> Optional[float]:
        """Sets a parameter; returns the value the autopilot read back, or None if unconfirmed.

        `param_type` is mavlink2rest's MAV_PARAM_TYPE name, e.g. "MAV_PARAM_TYPE_INT8".
        """
        result = self._write(name, value, param_type)
        return None if result is None else result[0]

    def is_armed(self) -> bool:
        """Raises AutopilotUnreachable or NoHeartbeat instead of guessing."""
        path = f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/HEARTBEAT"
        try:
            heartbeat = self._get(path)
        except urllib.error.HTTPError as e:
            if e.code >= 500:
                # BlueOS's proxy answered for a mavlink2rest that is down
                raise AutopilotUnreachable(f"mavlink2rest at {self.base_url} is not answering (HTTP {e.code})") from e
            raise NoHeartbeat(f"no autopilot HEARTBEAT in mavlink2rest (HTTP {e.code})") from e
        except json.JSONDecodeError as e:
            # mavlink2rest answered, with "None": it has no HEARTBEAT from the autopilot yet
            raise NoHeartbeat("no autopilot HEARTBEAT in mavlink2rest") from e
        except (urllib.error.URLError, OSError) as e:
            raise AutopilotUnreachable(f"Could not reach mavlink2rest at {self.base_url}: {e}") from e
        if not isinstance(heartbeat, dict) or "message" not in heartbeat:
            raise NoHeartbeat("no autopilot HEARTBEAT in mavlink2rest")
        return is_armed_heartbeat(heartbeat["message"])

    def check(self, suggested_viso_delay_ms: Optional[int] = None) -> list[dict]:
        """Compares the autopilot with what VIO needs. Read-only.

        Returns one row per parameter: {name, current, recommended, needs_reboot, ok, param_type}.
        `current` and `param_type` are None when the autopilot didn't report the parameter.
        VISO_DELAY_MS is included only when a suggestion is given, and is ok within 10 ms.
        """
        self.is_armed()  # surfaces AutopilotUnreachable / NoHeartbeat before reading anything
        wanted = list(VISUAL_ODOMETRY_PARAMS)
        if suggested_viso_delay_ms is not None:
            wanted.append((VISO_DELAY_PARAM, int(suggested_viso_delay_ms), False))
        rows = []
        for name, recommended, needs_reboot in wanted:
            result = self._read(name)
            current, param_type = result if result is not None else (None, None)
            if current is None:
                logger.warning(f"{name}: not reported by the autopilot")
                ok = False
            elif name == VISO_DELAY_PARAM:
                ok = abs(current - recommended) <= VISO_DELAY_TOLERANCE_MS
            else:
                ok = current == recommended
            if ok:
                logger.info(f"{name} = {current:g} (already set)")
            rows.append({"name": name, "current": current, "recommended": recommended,
                         "needs_reboot": needs_reboot, "ok": ok, "param_type": param_type})
        return rows

    def apply(self, diffs: list[dict], backup: Optional[ParamBackup] = None) -> list[dict]:
        """Writes the rows of `diffs` (from check()) that aren't ok. Raises VehicleArmed if armed.

        Returns one result per row written: {name, ok, needs_reboot, error}, plus
        `not_attempted: True` for rows left alone because the run stopped. `needs_reboot` is
        True when the write succeeded and only takes effect after an autopilot reboot.

        The vehicle is checked before *every* write: if it arms meanwhile (or its state can't be
        read), the rest isn't written. A read that fails mid-run is reported in that parameter's
        result, never raised, so what was already written is always reported.

        With `backup` (this autopilot's, see vehicle_key()), the value each parameter had before
        TrackShot first changed it is saved there *before* it is written, so revert() can put it
        back. That value is read from the autopilot just before the write; a parameter whose
        original can't be read or saved is not written. An original recorded by an earlier apply
        is never replaced.
        """
        if self.is_armed():
            raise VehicleArmed("Vehicle is armed, not changing autopilot parameters")
        results = []
        needs_reboot = []
        stop: Optional[str] = None      # why the remaining rows aren't written
        for diff in diffs:
            if diff.get("ok"):
                continue
            name = diff["name"]
            result = {"name": name, "ok": False, "needs_reboot": False, "error": None}
            results.append(result)
            if stop is not None:
                result.update(error=f"not written: {stop}", not_attempted=True)
                continue
            try:
                stop = self._apply_one(diff, result, backup)
            except Exception as e:   # e.g. the link dropped: report it and carry on with the rest
                logger.error(f"{name}: {e}")
                result["ok"] = False
                result["error"] = result["error"] or str(e)
            if result["ok"] and result["needs_reboot"]:
                needs_reboot.append(name)
        if stop is not None:
            logger.error(f"Apply stopped: {stop}")
        if needs_reboot:
            logger.warning(f"Reboot the autopilot for {', '.join(needs_reboot)} to take effect")
        return results

    def _not_writable(self) -> Optional[str]:
        """None when the vehicle is confirmed disarmed right now; otherwise why nothing may be written."""
        try:
            if self.is_armed():
                return "the vehicle was armed"
        except Exception as e:   # AutopilotUnreachable, NoHeartbeat…: don't guess
            return f"couldn't confirm the vehicle is disarmed ({e})"
        return None

    def _apply_one(self, diff: dict, result: dict, backup: Optional[ParamBackup]) -> Optional[str]:
        """apply() for one row, filling in `result`. Returns why the rows after it mustn't be
        written either (the vehicle armed), else None."""
        name, wanted = diff["name"], diff["recommended"]
        if diff.get("param_type") is None:
            result["error"] = "not reported by the autopilot"
            if name.startswith("VISO_") and name != "VISO_TYPE":
                # ArduPilot hides the VISO_* group until VISO_TYPE is set and it has rebooted
                result["error"] += " (set VISO_TYPE=1 and reboot first)"
            return None
        param_type = diff["param_type"]
        original = diff["current"]
        record = backup is not None and name not in backup
        if record:
            # The check may be minutes old: record what the autopilot holds right now. Without
            # that value there's nothing safe to record, so the parameter isn't written.
            try:
                fresh = self._read(name, cancellable=False)
                problem = None
            except Exception as e:
                fresh, problem = None, str(e)
            if fresh is None:
                logger.error(f"{name}: not written, its current value couldn't be read to back it up"
                             + (f" ({problem})" if problem else ""))
                result["error"] = ("not written: couldn't read its current value to back it up"
                                   + (f" ({problem})" if problem else ""))
                return None
            original, param_type = fresh[0], fresh[1] or param_type
            if _same(original, wanted):
                logger.info(f"{name} = {wanted:g} (already set)")
                result["ok"] = True
                return None
        stop = self._not_writable()
        if stop is not None:
            logger.error(f"{name}: not written: {stop}")
            result.update(error=f"not written: {stop}", not_attempted=True)
            return stop
        newly_recorded = False
        if record:
            try:
                newly_recorded = backup.record_original(name, original, param_type,
                                                        bool(diff.get("needs_reboot")))
            except BackupError as e:
                logger.error(f"{name}: not written, its current value couldn't be backed up: {e}")
                result["error"] = f"not written: couldn't back up its current value ({e})"
                return None
        try:
            confirmed = self.write(name, wanted, param_type)
        except Exception as e:
            logger.error(f"{name}: {e}")
            result["error"] = str(e)
            self._forget_unchanged(backup, name, original, newly_recorded)
            return None
        if confirmed is None or not _same(confirmed, wanted):
            logger.error(f"{name}: failed to set to {wanted:g} (was {_fmt(original)})")
            result["error"] = "write not confirmed"
            self._forget_unchanged(backup, name, original, newly_recorded)
            return None
        logger.info(f"{name}: {_fmt(original)} -> {wanted:g}")
        result["ok"] = True
        if backup is not None:
            try:
                backup.mark_written(name, confirmed, param_type)
            except BackupError as e:
                # The original is already on disk (recorded before the write); only the
                # "written" mark is missing, which revert() copes with
                logger.warning(f"{name}: written, but the backup couldn't be updated: {e}")
        if diff.get("needs_reboot"):
            result["needs_reboot"] = True
        return None

    def _forget_unchanged(self, backup: Optional[ParamBackup], name: str, original: float,
                          newly_recorded: bool) -> None:
        """After a failed write: drops the original just recorded, but only once the autopilot
        confirms it still holds it. An unconfirmed write may still have landed (only its reply was
        lost), and then the original is what revert() needs."""
        if backup is None or not newly_recorded:
            return
        try:
            now = self._read(name, cancellable=False)
        except Exception as e:   # can't tell: keep the original (revert() copes with an unwritten one)
            logger.warning(f"{name}: couldn't re-read it after the failed write ({e})")
            now = None
        if now is not None and _same(now[0], original):
            try:
                backup.remove(name)
            except BackupError as e:
                logger.warning(f"{name}: couldn't drop its unused backup entry: {e}")
        else:
            logger.warning(f"{name}: write not confirmed and the autopilot reports "
                           f"{'nothing' if now is None else f'{now[0]:g}'}; keeping its original "
                           f"({_fmt(original)}) in the backup")

    # --- Vehicle identity and undo -----------------------------------------------------------
    def vehicle_identity(self, request_timeout: float = 2.0) -> tuple[str, str]:
        """(key, description) naming this autopilot, for its parameter backup file.

        The key comes from AUTOPILOT_VERSION's hardware id (`uid2`, else `uid`), requested with
        MAV_CMD_REQUEST_MESSAGE when mavlink2rest hasn't seen one. When the autopilot reports no
        id (0) or doesn't answer, it falls back to the mavlink2rest address ("host-…").
        Raises AutopilotUnreachable when mavlink2rest can't be reached, Cancelled when `cancel`
        is set while waiting.
        """
        host = urllib.parse.urlsplit(self.base_url).netloc or self.base_url
        message = self._request_message("AUTOPILOT_VERSION", AUTOPILOT_VERSION_MSG_ID, request_timeout)
        if message is not None:
            content = message.get("message") or {}
            uid2 = content.get("uid2")
            if isinstance(uid2, list):
                raw = bytes(int(b) & 0xFF for b in uid2 if isinstance(b, (int, float)))
                if any(raw):
                    hex_id = raw.rstrip(b"\0").hex()
                    return safe_key(f"ap-{hex_id}"), f"autopilot {hex_id} (uid2), seen at {host}"
            uid = content.get("uid")
            if isinstance(uid, (int, float)) and int(uid) != 0:
                hex_id = f"{int(uid) & 0xFFFFFFFFFFFFFFFF:016x}"
                return safe_key(f"ap-{hex_id}"), f"autopilot {hex_id} (uid), seen at {host}"
            reason = "it reports no hardware id"
        else:
            reason = "it didn't send AUTOPILOT_VERSION"
        logger.info(f"Autopilot at {host}: {reason}; its parameter backup is named after the address")
        return safe_key(f"host-{host}"), f"the autopilot at {host} ({reason})"

    def vehicle_key(self, request_timeout: float = 2.0) -> str:
        """The file name for this autopilot's parameter backup (see vehicle_identity)."""
        return self.vehicle_identity(request_timeout)[0]

    def _request_message(self, name: str, message_id: int, timeout: float) -> Optional[dict]:
        """The vehicle's latest `name` from mavlink2rest; if it has none, asks the autopilot for
        it with MAV_CMD_REQUEST_MESSAGE and waits up to `timeout`."""
        message = self._vehicle_message(name)
        if message is not None:
            return message
        counter = self._message_counter(name)
        try:
            self._post({
                "type": "COMMAND_LONG",
                "target_system": self.system_id,
                "target_component": self.component_id,
                "command": {"type": "MAV_CMD_REQUEST_MESSAGE"},
                "confirmation": 0,
                "param1": float(message_id),
                "param2": 0.0, "param3": 0.0, "param4": 0.0,
                "param5": 0.0, "param6": 0.0, "param7": 0.0,
            })
        except RuntimeError as e:     # mavlink2rest rejected the command
            logger.warning(f"Couldn't request {name}: {e}")
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            message = self._vehicle_message(name)
            if message and message["status"]["time"]["counter"] > counter:
                return message
            if self.cancel.wait(0.1):
                raise Cancelled(f"waiting for {name}")
        return None

    @staticmethod
    def _revert_row(entry: dict, current: Optional[float]) -> dict:
        written = entry.get("written")
        confirmed = entry.get("status") == "written" and written is not None
        if entry.get("absent"):
            action = "skip"            # can't be un-created
        elif current is None:
            action = "missing"
        elif _same(current, entry["original"]):
            action = "none"            # already back at the original
        else:
            action = "restore"
        return {"name": entry["name"], "current": current, "original": entry.get("original"),
                "absent": bool(entry.get("absent")), "written_by_trackshot": written,
                # Changed since TrackShot wrote it (by hand, another GCS…): still revertible
                "drifted": bool(confirmed and current is not None and not _same(current, written)),
                # Recorded, but TrackShot's write was never confirmed (e.g. it crashed mid-apply)
                "unconfirmed": not confirmed,
                "needs_reboot": bool(entry.get("needs_reboot")), "action": action}

    def preview_revert(self, backup: ParamBackup) -> list[dict]:
        """What revert() would do, in the order it would do it. Read-only (and cancellable).

        One row per backed-up parameter: {name, current, original, absent, written_by_trackshot,
        drifted, unconfirmed, needs_reboot, action}; `action` is "restore", "none" (already at
        the original), "skip" (didn't exist before TrackShot) or "missing" (not reported now).
        """
        rows = []
        for entry in reversed(backup.entries()):
            result = None if entry.get("absent") else self._read(entry["name"])
            rows.append(self._revert_row(entry, None if result is None else result[0]))
        return rows

    def revert(self, backup: ParamBackup) -> list[dict]:
        """Puts back the values the parameters in `backup` had before TrackShot changed them,
        newest change first. Raises VehicleArmed if armed. It ignores `cancel` (like apply()).

        Returns one result per backed-up parameter: {name, ok, needs_reboot, error, skipped,
        drifted, current, original}, plus `not_attempted: True` for ones left alone because the
        run stopped. Restored parameters (and ones found already at their original) leave the
        backup; the rest stay for another try. Parameters that didn't exist before TrackShot are
        skipped (`skipped`): they can't be removed, and they leave the backup only when everything
        else was restored.

        It stops at the first parameter it can't restore (not reported, write not confirmed, a
        dropped link): older changes are only undone after newer ones, so VISO_TYPE never goes
        back (hiding VISO_DELAY_MS after the next reboot) while a newer VISO_* change is still in
        place, and the vehicle is only ever left in a state an apply went through. It also stops
        when the vehicle arms (checked before every write) or its state can't be read.
        """
        if self.is_armed():
            raise VehicleArmed("Vehicle is armed, not changing autopilot parameters")
        results = []
        absent = []
        all_restored = True
        stop: Optional[str] = None      # why the remaining entries aren't attempted
        for entry in reversed(backup.entries()):
            name, original = entry["name"], entry.get("original")
            result = {"name": name, "ok": False, "needs_reboot": False, "error": None,
                      "skipped": False, "drifted": False, "current": None, "original": original}
            results.append(result)
            if entry.get("absent"):
                result["skipped"] = True
                result["error"] = ("didn't exist before TrackShot, so it can't be removed: restoring "
                                   "the setting that enables it (e.g. VISO_TYPE) and rebooting hides it again")
                absent.append(name)
                continue
            if stop is not None:
                result.update(error=f"not attempted: {stop}", not_attempted=True)
                continue
            try:
                armed = self._revert_one(entry, result, backup)
            except Exception as e:   # e.g. the link dropped on the read: stop, but report what was done
                logger.error(f"{name}: {e}")
                result["ok"] = False
                result["error"] = result["error"] or str(e)
                armed = None
            if not result["ok"]:
                all_restored = False
                stop = armed or f"{name} wasn't restored first (older changes are undone only after newer ones)"
        if stop is not None:
            logger.error(f"Revert stopped: {stop}")
        if all_restored:
            for name in absent:
                try:
                    backup.remove(name)
                except BackupError as e:
                    logger.warning(f"{name}: couldn't drop it from the backup: {e}")
        rebooted = [r["name"] for r in results if r["needs_reboot"]]
        if rebooted:
            logger.warning(f"Reboot the autopilot for {', '.join(rebooted)} to take effect")
        return results

    def _revert_one(self, entry: dict, result: dict, backup: ParamBackup) -> Optional[str]:
        """revert() for one backup entry, filling in `result`. Returns why nothing more may be
        written (the vehicle armed), else None."""
        name, original = entry["name"], entry.get("original")
        read = self._read(name, cancellable=False)
        if read is None:
            result["error"] = "not reported by the autopilot"
            if name.startswith("VISO_") and name != "VISO_TYPE":
                result["error"] += (" (ArduPilot hides it unless VISO_TYPE was 1 when the autopilot "
                                    "started: set VISO_TYPE=1, reboot, then revert)")
            return None
        current, reported_type = read
        row = self._revert_row(entry, current)
        result.update(current=current, drifted=row["drifted"])
        if row["drifted"]:
            logger.warning(f"{name}: is {current:g}, not the {row['written_by_trackshot']:g} TrackShot "
                           f"wrote; restoring {original:g} anyway")
        if row["action"] == "none":
            logger.info(f"{name} = {original:g} (already at its value from before TrackShot)")
            result["ok"] = True
        else:
            stop = self._not_writable()
            if stop is not None:
                logger.error(f"{name}: not restored: {stop}")
                result.update(error=f"not restored: {stop}", not_attempted=True)
                return stop
            try:
                confirmed = self.write(name, original, reported_type or entry.get("param_type"))
            except Exception as e:
                logger.error(f"{name}: {e}")
                result["error"] = str(e)
                return None
            if confirmed is None or not _same(confirmed, original):
                logger.error(f"{name}: failed to restore {original:g} (is {current:g})")
                result["error"] = "write not confirmed"
                return None
            logger.info(f"{name}: {current:g} -> {original:g} (restored)")
            result["ok"] = True
            result["needs_reboot"] = bool(entry.get("needs_reboot"))
        try:
            backup.remove(name)
        except BackupError as e:
            logger.warning(f"{name}: restored, but the backup couldn't be updated: {e}")
        return None

    def get_origin(self, request_timeout: float = 2.0) -> Optional[tuple[float, float]]:
        """Returns (lat, lon) of the EKF origin, or None if the autopilot has none."""
        message = self._vehicle_message("GPS_GLOBAL_ORIGIN")
        if message is None:
            counter = self._message_counter("GPS_GLOBAL_ORIGIN")
            self._post({
                "type": "COMMAND_LONG",
                "target_system": self.system_id,
                "target_component": self.component_id,
                "command": {"type": "MAV_CMD_REQUEST_MESSAGE"},
                "confirmation": 0,
                "param1": float(GPS_GLOBAL_ORIGIN_MSG_ID),
                "param2": 0.0, "param3": 0.0, "param4": 0.0,
                "param5": 0.0, "param6": 0.0, "param7": 0.0,
            })
            deadline = time.time() + request_timeout
            while time.time() < deadline:
                message = self._vehicle_message("GPS_GLOBAL_ORIGIN")
                if message and message["status"]["time"]["counter"] > counter:
                    break
                message = None
                time.sleep(0.1)
        if message is None:
            return None
        content = message["message"]
        return content["latitude"] / 1e7, content["longitude"] / 1e7
