"""What the autopilot's parameters were before TrackShot changed them, so they can be put back.

One JSON file per autopilot, `param_backups/<vehicle_key>.json` next to settings.json (see
`trackshot.autopilot.params.AutopilotParams.vehicle_key`). For each parameter TrackShot changed:

    {"original": 0.0,              # the value before TrackShot first changed it; null if absent
     "absent": false,              # true: the autopilot didn't report the parameter at all (e.g.
                                   #   VISO_DELAY_MS is hidden until VISO_TYPE=1 and a reboot)
     "param_type": "MAV_PARAM_TYPE_INT8", "needs_reboot": true,
     "written": 1.0,               # what TrackShot's last write set; null until a write is confirmed
     "status": "written",          # "pending": recorded, write not (yet) confirmed | "written"
     "order": 3,                   # when it was first recorded: revert goes in reverse order
     "recorded_at": 1700000000.0, "written_at": 1700000001.0}

Rules that make it safe:
- **First change only.** `record_original` never replaces an existing entry, so applying again (or
  applying a new VISO_DELAY_MS) keeps the value from before TrackShot's first change.
- **Recorded before writing.** Every change is on disk (written to a temporary file, then renamed
  over the old one) before the parameter is written, so a crash part way through an apply still
  leaves a backup to revert from.
- **Nothing is discarded silently.** A file that can't be read is renamed aside
  (`<key>.corrupt-<time>.json`), logged and reported in `problem`; the store then starts empty.
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
from pathlib import Path
from typing import Optional

from loguru import logger

from trackshot.config import config_path

FORMAT_VERSION = 1
STATUS_PENDING = "pending"
STATUS_WRITTEN = "written"


def backups_dir() -> Path:
    """`param_backups` in the same directory as settings.json."""
    return config_path().parent / "param_backups"


def safe_key(text: str) -> str:
    """`text` as a file name: letters, digits, '.', '_' and '-' only."""
    key = re.sub(r"[^A-Za-z0-9._-]+", "_", text).strip("._") or "unknown"
    return key[:120]


class BackupError(Exception):
    """The backup file couldn't be written, so the parameter must not be changed."""


def _valid_entry(entry) -> bool:
    if not isinstance(entry, dict):
        return False
    absent = entry.get("absent")
    original = entry.get("original")
    if not isinstance(absent, bool):
        return False
    if not absent and not isinstance(original, (int, float)):
        return False
    return isinstance(entry.get("order"), int)


class ParamBackup:
    """The pre-TrackShot values of one autopilot's parameters. Thread-safe; every change is saved
    at once."""

    def __init__(self, vehicle_key: str, directory: Optional[Path] = None, *,
                 description: Optional[str] = None):
        self.vehicle_key = safe_key(vehicle_key)
        self.directory = Path(directory) if directory is not None else backups_dir()
        self.path = self.directory / f"{self.vehicle_key}.json"
        self.description = description
        # Why the file on disk couldn't be used (it was set aside), or None
        self.problem: Optional[str] = None
        self._lock = threading.RLock()
        self._entries: dict[str, dict] = {}
        self._load()

    # --- Reading ----------------------------------------------------------------------------
    def entries(self) -> list[dict]:
        """Copies of the entries in application order, each with its `name`."""
        with self._lock:
            ordered = sorted(self._entries.items(), key=lambda item: item[1]["order"])
            return [{"name": name, **entry} for name, entry in ordered]

    def get(self, name: str) -> Optional[dict]:
        with self._lock:
            entry = self._entries.get(name)
            return None if entry is None else {"name": name, **entry}

    def __contains__(self, name: str) -> bool:
        with self._lock:
            return name in self._entries

    def __len__(self) -> int:
        with self._lock:
            return len(self._entries)

    def set_aside_files(self) -> list[Path]:
        """Earlier backups of this autopilot that couldn't be read and were moved aside. They may
        still hold pre-TrackShot values, so the UI keeps mentioning them until they are dealt with."""
        try:
            return sorted(self.directory.glob(f"{self.vehicle_key}.corrupt-*.json"))
        except OSError:
            return []

    # --- Changing ---------------------------------------------------------------------------
    def record_original(self, name: str, original: Optional[float], param_type: Optional[str],
                        needs_reboot: bool) -> bool:
        """Remembers `original` (None: the autopilot didn't have the parameter) as the value from
        before TrackShot, unless one is already recorded. Saved before returning.

        Returns True if it was recorded now, False if an earlier original was kept.
        Raises BackupError if it couldn't be saved (and then nothing is remembered).
        """
        with self._lock:
            if name in self._entries:
                return False
            order = max((e["order"] for e in self._entries.values()), default=0) + 1
            self._entries[name] = {
                "original": None if original is None else float(original),
                "absent": original is None,
                "param_type": param_type,
                "needs_reboot": bool(needs_reboot),
                "written": None,
                "status": STATUS_PENDING,
                "order": order,
                "recorded_at": time.time(),
                "written_at": None,
            }
            try:
                self._save()
            except BackupError:
                del self._entries[name]
                raise
            return True

    def mark_written(self, name: str, value: float, param_type: Optional[str] = None) -> None:
        """TrackShot's write of `name` was confirmed with `value`."""
        with self._lock:
            entry = self._entries.get(name)
            if entry is None:
                return
            entry.update(written=float(value), status=STATUS_WRITTEN, written_at=time.time())
            if param_type and not entry.get("param_type"):
                entry["param_type"] = param_type
            self._save()

    def remove(self, name: str) -> None:
        """Forgets `name` (restored, or never actually changed)."""
        with self._lock:
            if self._entries.pop(name, None) is not None:
                self._save()

    # --- Persistence ------------------------------------------------------------------------
    def _load(self) -> None:
        try:
            text = self.path.read_text()
        except FileNotFoundError:
            return
        except OSError as e:
            # Unreadable but maybe intact: don't move it, and don't overwrite it either
            self.problem = f"Couldn't read the parameter backup {self.path}: {e}"
            logger.error(self.problem)
            raise BackupError(self.problem) from e
        try:
            data = json.loads(text)
            params = data["params"]
            if not isinstance(params, dict) or not all(_valid_entry(e) for e in params.values()):
                raise ValueError("unexpected contents")
        except (ValueError, KeyError, TypeError) as e:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            aside = self.path.with_name(f"{self.path.stem}.corrupt-{stamp}.json")
            n = 1
            while aside.exists():     # never replace an earlier set-aside file
                n += 1
                aside = self.path.with_name(f"{self.path.stem}.corrupt-{stamp}-{n}.json")
            try:
                os.replace(self.path, aside)
                where = f"it was moved to {aside}"
            except OSError as move_error:
                raise BackupError(f"The parameter backup {self.path} is unreadable ({e}) "
                                  f"and couldn't be set aside: {move_error}") from e
            self.problem = (f"The parameter backup for this autopilot was unreadable ({e}); {where}. "
                            "It may hold values from before TrackShot that need restoring by hand.")
            logger.error(self.problem)
            return
        self._entries = {str(name): dict(entry) for name, entry in params.items()}

    def _save(self) -> None:
        data = {"format": FORMAT_VERSION, "vehicle_key": self.vehicle_key,
                "description": self.description, "saved_at": time.time(),
                "params": self._entries}
        tmp = self.path.with_name(f".{self.path.name}.tmp")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            with open(tmp, "w") as f:
                json.dump(data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except (OSError, TypeError, ValueError) as e:
            try:
                tmp.unlink()
            except OSError:
                pass
            raise BackupError(f"Couldn't save the parameter backup {self.path}: {e}") from e


def open_backup(params, directory: Optional[Path] = None) -> ParamBackup:
    """The backup for the autopilot `params` (an AutopilotParams) talks to. Does network I/O
    (AutopilotParams.vehicle_identity), so call it off the GUI thread. Raises BackupError if the
    file exists but can't be read."""
    key, description = params.vehicle_identity()
    return ParamBackup(key, directory, description=description)
