"""Autopilot parameters: check the EKF/VISO settings VIO needs, apply the recommended ones, and
revert to the values from before TrackShot changed them.

Reading and writing parameters is blocking HTTP to BlueOS's mavlink2rest (seconds when the vehicle
is slow or offline), so all of it, and the parameter backup's file I/O, runs on a QThreadPool
thread and reports back through Qt signals. Applying and reverting ask for confirmation first, and
AutopilotParams itself refuses while the vehicle is armed.

Apply records each parameter's pre-TrackShot value in the vehicle's ParamBackup before writing it
(trackshot.autopilot.backup). Check also loads that backup, which fills the "Before TrackShot"
column and enables "Revert to previous…". Revert first reads the current values (in the worker)
for its confirmation dialog, then restores them, newest change first.

After an apply or a revert that needs a reboot, the reminder stays in the message (and the table
marks those parameters "Pending") through the automatic re-check, until the vehicle telemetry
shows the autopilot up, then drop out and come back. Reminders belong to the vehicle (address)
they were written to: changing the vehicle in Settings shows that vehicle's own, and switching
back brings the old ones back.

Results of an apply or revert are tied to the vehicle they were started on: one that finishes
after the vehicle changed is reported as the previous vehicle's and never shown as the new one's.
While one is writing, `writing` is True and closing the window waits for it (MainWindow); its end
emits `writesFinished`.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

from PySide6.QtCore import QThreadPool, Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QMessageBox,
                               QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from trackshot.ui.theme import NONE_TEXT, css, current_theme
from trackshot.ui.workers import run_in_pool

# The reboot-pending reminder for a parameter TrackShot wrote ("apply") or restored ("revert")
REBOOT_APPLY, REBOOT_REVERT = "apply", "revert"
VISO_DELAY_PARAM = "VISO_DELAY_MS"
VISO_DELAY_TOLERANCE_MS = 10      # as trackshot.autopilot.params (imported lazily there)

COLUMNS = ("Parameter", "Current", "Recommended", "Before TrackShot", "Reboot?", "OK")
COL_CURRENT, COL_RECOMMENDED, COL_BEFORE, COL_REBOOT, COL_OK = 1, 2, 3, 4, 5
ABSENT_TEXT = "(absent)"
# A heartbeat older than this means the autopilot went away (e.g. it is rebooting)
AUTOPILOT_GONE_S = 3.0

FRIENDLY_ERRORS = {
    "VehicleArmed": "The vehicle is armed. Parameters are never changed while armed — disarm and try again.",
    "AutopilotUnreachable": "Can't reach BlueOS (mavlink2rest). Check the tether and the vehicle IP in Settings.",
    "NoHeartbeat": "BlueOS answered, but it has no heartbeat from the autopilot yet. "
                   "Check the autopilot is powered and connected to BlueOS.",
    "BackupError": "TrackShot couldn't save or read its backup of the autopilot's current parameter "
                   "values, so it didn't change them.",
}


def error_kind(error: BaseException) -> Optional[str]:
    """Which of trackshot.autopilot.params' exceptions `error` is, if any.

    That module is imported lazily (and may be missing in a partial build); the class name is the
    fallback so the message is still friendly.
    """
    try:
        from trackshot.autopilot import backup, params
        for name in FRIENDLY_ERRORS:
            cls = getattr(params, name, None) or getattr(backup, name, None)
            if cls is not None and isinstance(error, cls):
                return name
    except ImportError:
        pass
    name = type(error).__name__
    return name if name in FRIENDLY_ERRORS else None


def describe_error(error: BaseException) -> str:
    kind = error_kind(error)
    if kind is None:
        return f"{type(error).__name__}: {error}"
    detail = str(error)
    return FRIENDLY_ERRORS[kind] + (f"\n({detail})" if detail else "")


def normalise_row(row: dict) -> dict:
    """One check() row with the contract's keys, accepting the extension's older names too."""
    recommended = row.get("recommended", row.get("wanted"))
    current = row.get("current")
    ok = row.get("ok")
    if ok is None:
        ok = current is not None and current == recommended
    # Keep the original keys too (e.g. param_type): apply() is handed these rows back
    return {**row, "name": row.get("name", "?"), "current": current, "recommended": recommended,
            "needs_reboot": bool(row.get("needs_reboot", row.get("reboot_required", False))), "ok": bool(ok)}


def value_ok(name: str, current: Optional[float], recommended: Any) -> bool:
    """A row's "OK", as AutopilotParams.check() decides it."""
    if current is None or recommended is None:
        return False
    if name == VISO_DELAY_PARAM:
        return abs(float(current) - float(recommended)) <= VISO_DELAY_TOLERANCE_MS
    return float(current) == float(recommended)


def _num(v: Any) -> str:
    if v is None:
        return NONE_TEXT
    return f"{v:g}" if isinstance(v, (int, float)) else str(v)


def before_text(entry: Optional[dict]) -> str:
    """The "Before TrackShot" cell: blank when TrackShot never changed the parameter."""
    if entry is None:
        return ""
    return ABSENT_TEXT if entry.get("absent") else _num(entry.get("original"))


def _drifted(entry: dict, current: Optional[float]) -> bool:
    written = entry.get("written")
    return (entry.get("status") == "written" and written is not None and current is not None
            and abs(float(current) - float(written)) > 1e-6 * max(1.0, abs(float(written))))


def before_tooltip(entry: dict, current: Optional[float]) -> tuple[str, bool]:
    when = entry.get("recorded_at")
    stamp = time.strftime("%Y-%m-%d %H:%M", time.localtime(when)) if when else "?"
    if entry.get("absent"):
        tip = f"Didn't exist before TrackShot first changed it ({stamp})"
    else:
        tip = f"Value before TrackShot first changed it ({stamp})"
    written = entry.get("written")
    if written is not None:
        tip += f"; TrackShot wrote {_num(written)}"
    else:
        tip += "; TrackShot's write was never confirmed"
    drifted = _drifted(entry, current)
    if drifted:
        tip += f". Changed since then (now {_num(current)}): Revert still restores {before_text(entry)}"
    return tip, drifted


def revert_summary(rows: list[dict]) -> str:
    """The revert confirmation text for preview_revert() rows (in the order they'll be restored)."""
    lines = []
    for r in rows:
        action = r.get("action")
        if action == "skip":
            lines.append(f"{r['name']}: didn't exist before TrackShot — left as {_num(r.get('current'))} "
                         "(it can't be removed; restoring VISO_TYPE and rebooting hides it)")
            continue
        if action == "missing":
            lines.append(f"{r['name']}: not reported by the autopilot now — will be tried, "
                         f"restoring {_num(r.get('original'))}")
            continue
        if action == "none":
            lines.append(f"{r['name']}: already {_num(r.get('original'))} (its value before TrackShot)")
            continue
        flags = []
        if r.get("drifted"):
            flags.append(f"changed since TrackShot wrote {_num(r.get('written_by_trackshot'))}")
        if r.get("unconfirmed"):
            flags.append("TrackShot's write was never confirmed")
        if r.get("needs_reboot"):
            flags.append("needs reboot")
        lines.append(f"{r['name']}: {_num(r.get('current'))} → {_num(r.get('original'))}"
                     + (f"  ({'; '.join(flags)})" if flags else ""))
    text = ("Put these autopilot parameters back to their values from before TrackShot changed them?\n"
            "(Restored newest change first.)\n\n" + "\n".join(lines)
            + "\n\nThe vehicle must be disarmed. Parameters marked 'needs reboot' take effect after "
              "rebooting the autopilot.")
    if any(r.get("drifted") for r in rows):
        text += ("\n\nWarning: some parameters were changed by something other than TrackShot after it "
                 "wrote them. Reverting replaces those changes too.")
    return text


def backup_snapshot(backup) -> dict:
    """What the GUI shows of a ParamBackup (plain data, safe to hand across threads)."""
    aside = backup.set_aside_files()
    problem = backup.problem
    if problem is None and aside:
        problem = ("An earlier parameter backup for this autopilot couldn't be read and was set aside: "
                   + ", ".join(str(p) for p in aside)
                   + ". It may hold values from before TrackShot that need restoring by hand.")
    return {"key": backup.vehicle_key, "description": backup.description, "path": str(backup.path),
            "entries": backup.entries(), "problem": problem}


def _open_backup(params):
    from trackshot.autopilot.backup import open_backup
    return open_backup(params)


# --- Worker tasks (QThreadPool threads) ----------------------------------------------------------

def check_task(params, suggested: Optional[int]) -> dict:
    rows = params.check(suggested)
    from trackshot.autopilot.backup import BackupError
    try:
        snapshot = backup_snapshot(_open_backup(params))
    except BackupError as e:
        snapshot = {"key": None, "description": None, "path": None, "entries": [], "problem": str(e)}
    return {"rows": rows, "backup": snapshot}


def apply_task(params, diffs: list[dict]) -> dict:
    # Without a working backup nothing is written: BackupError is raised before any write
    backup = _open_backup(params)
    results = params.apply(diffs, backup=backup)
    return {"results": results, "backup": backup_snapshot(backup)}


def preview_revert_task(params) -> dict:
    from trackshot.autopilot.params import VehicleArmed
    if params.is_armed():
        raise VehicleArmed("Vehicle is armed, not changing autopilot parameters")
    backup = _open_backup(params)
    return {"rows": params.preview_revert(backup), "backup": backup_snapshot(backup)}


def revert_task(params, expected_key: str) -> dict:
    backup = _open_backup(params)
    if backup.vehicle_key != expected_key:
        raise RuntimeError("The autopilot changed after the revert was confirmed; nothing was changed. "
                           "Check, then revert again.")
    results = params.revert(backup)
    return {"results": results, "backup": backup_snapshot(backup)}


class AutopilotPanel(QWidget):
    # An apply or revert finished (or failed): `writing` is False again
    writesFinished = Signal()

    def __init__(self, engine, pool: QThreadPool, parent: QWidget | None = None):
        super().__init__(parent)
        self._engine = engine
        self._pool = pool
        self._suggested: Optional[int] = None
        self.rows: list[dict] = []
        self.busy = False
        # Abandons a running check when the window closes (see cancel_background)
        self._cancel = threading.Event()
        # Per vehicle: parameters written ("apply") or restored ("revert") that only take effect
        # after a reboot, until one is seen: {vehicle: {name: kind}}. See reboot_pending.
        self._reboot: dict[str, dict[str, str]] = {}
        # The current vehicle's restart, as seen in telemetry: up, then gone, then back
        self._autopilot_seen_up = False
        self._autopilot_went_away = False
        # An apply or revert is running (it isn't cut short: closing waits for it)
        self.writing = False
        # What the last apply/revert reported, kept for the message after its automatic re-check
        self._apply_note: Optional[tuple[str, str]] = None
        # This vehicle's parameter backup as last loaded by a worker: {name: entry}
        self.backup_entries: dict[str, dict] = {}
        self.backup_info: Optional[dict] = None
        self._vehicle: Optional[str] = None
        # Bumped when the vehicle changes: a read started before then is about the old vehicle
        self._generation = 0

        self.check_button = QPushButton("Check")
        self.check_button.setToolTip("Read the autopilot's parameters (doesn't change anything)")
        self.check_button.clicked.connect(self.check)
        self.apply_button = QPushButton("Apply recommended…")
        self.apply_button.setEnabled(False)
        self.apply_button.clicked.connect(self.apply_recommended)
        self.revert_button = QPushButton("Revert to previous…")
        self.revert_button.setToolTip("Put back the values these parameters had before TrackShot changed them")
        self.revert_button.setEnabled(False)
        self.revert_button.clicked.connect(self.revert_previous)
        self.suggested_label = QLabel()
        self.message = QLabel("Check reads the EKF and visual-odometry parameters VIO needs.")
        self.message.setWordWrap(True)
        self.message.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.table.setAlternatingRowColors(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col in range(1, len(COLUMNS)):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)

        buttons = QHBoxLayout()
        buttons.addWidget(self.check_button)
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.revert_button)
        buttons.addStretch(1)
        buttons.addWidget(self.suggested_label)
        layout = QVBoxLayout(self)
        layout.addLayout(buttons)
        layout.addWidget(self.message)
        layout.addWidget(self.table, 1)
        self.set_suggested_delay(None)

    # --- Reboot reminders -----------------------------------------------------------------------
    def _vehicle_key(self) -> str:
        return self._vehicle or ""

    @property
    def reboot_pending(self) -> list[str]:
        """The current vehicle's parameters waiting for an autopilot reboot."""
        return list(self._reboot.get(self._vehicle_key(), {}))

    @reboot_pending.setter
    def reboot_pending(self, names: list[str]) -> None:
        self._reboot[self._vehicle_key()] = {name: REBOOT_APPLY for name in names}

    def _note_reboot(self, vehicle: str, names: list[str], kind: str) -> None:
        if not names:
            return
        pending = self._reboot.setdefault(vehicle, {})
        for name in names:
            pending[name] = kind     # a revert's reminder replaces the apply's, and vice versa
        if vehicle == self._vehicle_key():
            # Only a restart seen after this write counts
            self._autopilot_seen_up = False
            self._autopilot_went_away = False

    def _reboot_text(self, vehicle: Optional[str] = None) -> str:
        pending = self._reboot.get(self._vehicle_key() if vehicle is None else vehicle, {})
        applied = [n for n, kind in pending.items() if kind == REBOOT_APPLY]
        reverted = [n for n, kind in pending.items() if kind == REBOOT_REVERT]
        parts = []
        if applied:
            parts.append("Reboot the autopilot for " + ", ".join(applied)
                         + " to take effect (until then the EKF can't use visual odometry).")
        if reverted:
            parts.append("Reboot the autopilot for " + ", ".join(reverted)
                         + " to put the setup from before TrackShot back in effect (until then the "
                           "autopilot keeps the setup it started with).")
        return " ".join(parts)

    # --- Inputs ---------------------------------------------------------------------------------
    def set_suggested_delay(self, ms: Optional[int]) -> None:
        """From the odometry health: included in the next check as VISO_DELAY_MS."""
        self._suggested = ms
        self.suggested_label.setText(
            f"Suggested VISO_DELAY_MS: {ms} ms (measured)" if ms is not None
            else "VISO_DELAY_MS: measured once VIO runs")

    def note_telemetry(self, telemetry: Any) -> None:
        """From each status update: clears the reboot reminder once the autopilot has restarted
        (its heartbeat stopped, then came back)."""
        if not self.reboot_pending or telemetry is None:
            return
        get = telemetry.get if isinstance(telemetry, dict) else lambda k, d=None: getattr(telemetry, k, d)
        if get("age_s") is None:
            return   # the poller hasn't polled yet (e.g. VIO just restarted): no news either way
        age = get("heartbeat_age_s")
        alive = bool(get("connected", False)) and age is not None and age < AUTOPILOT_GONE_S
        if not alive:
            # Only a drop after the autopilot was seen up counts: telemetry that is still
            # connecting (VIO just started, or the vehicle just changed) is not a restart
            if self._autopilot_seen_up:
                self._autopilot_went_away = True
        elif not self._autopilot_went_away:
            self._autopilot_seen_up = True
        else:
            self._reboot.pop(self._vehicle_key(), None)
            self._autopilot_went_away = False
            self._fill_table()
            if not self.busy:
                self._set_message("The autopilot restarted: the rebooted parameters are in effect. "
                                  "Check again to confirm.", "ok")

    def set_vehicle(self, vehicle_ip: str) -> None:
        """The vehicle in the applied settings. A different one makes the table and the backup
        shown stale: they are cleared until the next check."""
        if self._vehicle is not None and vehicle_ip != self._vehicle:
            self._generation += 1
            self.rows = []
            self.backup_entries = {}
            self.backup_info = None
            self._apply_note = None
            # Telemetry now describes the new vehicle; its reboot reminders (if any) are its own
            self._autopilot_seen_up = False
            self._autopilot_went_away = False
            self._vehicle = vehicle_ip
            self._fill_table()
            if not self.busy:
                self._set_busy(False, "The vehicle changed: Check reads its parameters.")
        self._vehicle = vehicle_ip

    def cancel_background(self) -> None:
        """Abandons a running check or revert preview (the app is closing), and starts no new
        one. A running apply or revert still finishes (MainWindow waits for it)."""
        self._cancel.set()

    def show_closing_notice(self) -> None:
        self._set_message("Finishing the autopilot parameter writes: TrackShot closes when they are done.",
                          "warn")

    def apply_theme(self) -> None:
        self._fill_table()

    # --- Actions --------------------------------------------------------------------------------
    def check(self) -> None:
        if self.busy or self._cancel.is_set():   # never start reading while the app closes
            return
        try:
            params = self._engine.autopilot_params()
        except Exception as e:
            self._show_error(e)
            return
        params.cancel = self._cancel
        suggested = self._suggested
        self._set_busy(True, "Reading parameters from the autopilot…")
        generation = self._generation
        run_in_pool(self._pool, lambda: check_task(params, suggested),
                    lambda r: self._on_checked(r, generation), lambda e: self._on_failed(e, generation))

    def apply_recommended(self) -> None:
        diffs = [r for r in self.rows if not r["ok"]]
        if self.busy or not diffs:
            return
        if not self.confirm_apply(diffs):
            return
        try:
            params = self._engine.autopilot_params()
        except Exception as e:
            self._show_error(e)
            return
        params.cancel = self._cancel   # only reads before the first write can be cut short
        self._apply_note = None
        self._start_writing("Writing parameters…")
        generation, vehicle = self._generation, self._vehicle_key()
        run_in_pool(self._pool, lambda: apply_task(params, diffs),
                    lambda r: self._on_applied(diffs, r, generation, vehicle),
                    lambda e: self._on_write_failed(e, generation, vehicle))

    def revert_previous(self) -> None:
        """Reads what a revert would change (worker), asks, then reverts (worker)."""
        if self.busy or not self.backup_entries:
            return
        try:
            params = self._engine.autopilot_params()
        except Exception as e:
            self._show_error(e)
            return
        params.cancel = self._cancel
        self._set_busy(True, "Reading the current values of the parameters TrackShot changed…")
        generation = self._generation
        run_in_pool(self._pool, lambda: preview_revert_task(params),
                    lambda r: self._on_revert_preview(params, r, generation),
                    lambda e: self._on_failed(e, generation))

    def confirm_apply(self, diffs: list[dict]) -> bool:
        """Asks the pilot. Separate so tests can answer without a modal dialog."""
        lines = [f"{d['name']}: {_num(d['current'])} → {_num(d['recommended'])}"
                 + ("  (needs reboot)" if d["needs_reboot"] else "") for d in diffs]
        answer = QMessageBox.question(
            self, "Apply recommended parameters",
            "Write these autopilot parameters?\n\n" + "\n".join(lines)
            + "\n\nThe vehicle must be disarmed. Parameters marked 'needs reboot' take effect after "
              "rebooting the autopilot. The current values are saved first, so 'Revert to previous…' "
              "can put them back.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel)
        return answer == QMessageBox.StandardButton.Yes

    def confirm_revert(self, rows: list[dict]) -> bool:
        """Asks the pilot, listing what the revert will do. Separate so tests can answer."""
        answer = QMessageBox.question(
            self, "Revert to previous parameters", revert_summary(rows),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel)
        return answer == QMessageBox.StandardButton.Yes

    # --- Results (GUI thread) -------------------------------------------------------------------
    def _take_backup(self, snapshot: Optional[dict]) -> None:
        if snapshot is None:
            return
        self.backup_info = snapshot
        self.backup_entries = {e["name"]: e for e in snapshot.get("entries") or []}

    def _stale(self, generation: Optional[int]) -> bool:
        if generation is None or generation == self._generation:
            return False
        self._set_busy(False, "The vehicle changed while reading the old one: Check reads the new one.", "warn")
        return True

    def _on_checked(self, result: Any, generation: Optional[int] = None) -> None:
        if self._stale(generation):
            return
        # check_task returns {"rows", "backup"}; a bare list of rows is accepted too
        if isinstance(result, dict):
            rows = result.get("rows")
            self._take_backup(result.get("backup"))
        else:
            rows = result
        self.rows = [normalise_row(r) for r in rows or []]
        self._fill_table()
        bad = [r for r in self.rows if not r["ok"]]
        parts = [f"{len(bad)} parameter(s) differ from the recommended values." if bad
                 else "All parameters are set as recommended."]
        level = "warn" if bad else "ok"
        if self.backup_entries:
            parts.append(f"{len(self.backup_entries)} parameter(s) changed by TrackShot can be reverted.")
        note, self._apply_note = self._apply_note, None
        if note is not None:
            parts.insert(0, note[0])
            level = "bad" if note[1] == "bad" else "warn" if note[1] == "warn" and level == "ok" else level
        problem = (self.backup_info or {}).get("problem")
        if problem:
            parts.append(problem)
            level = "bad" if level == "bad" else "warn"
        if self.reboot_pending:
            parts.append(self._reboot_text())
            level = "bad" if level == "bad" else "warn"
        self._set_busy(False, " ".join(parts), level)

    def _start_writing(self, message: str) -> None:
        self.writing = True
        self._set_busy(True, message)

    def _finish_writing(self) -> None:
        self.writing = False
        self.writesFinished.emit()

    def _write_was_stale(self, generation: Optional[int]) -> bool:
        return generation is not None and generation != self._generation

    def _after_stale_write(self, vehicle: str, text: str, level: str) -> None:
        """An apply/revert finished after the vehicle changed: it is about `vehicle`, not the one
        shown now. Its reboot reminder was filed under `vehicle`; nothing else here changes."""
        reminder = self._reboot_text(vehicle)
        self._set_busy(False, f"Previous vehicle ({vehicle or '?'}): {text}"
                       + (f" {reminder}" if reminder else "")
                       + " The vehicle changed: Check reads the new one.", "bad" if level == "bad" else "warn")

    def _on_applied(self, diffs: list[dict], result: Any, generation: Optional[int] = None,
                    vehicle: Optional[str] = None) -> None:
        # apply_task returns {"results", "backup"}; AutopilotParams.apply a list of {name, ok, error?};
        # the extension returned (results, needs_reboot).
        vehicle = self._vehicle_key() if vehicle is None else vehicle
        stale = self._write_was_stale(generation)
        snapshot = result.get("backup") if isinstance(result, dict) else None
        if isinstance(result, dict):
            result = result.get("results")
        results = result[0] if isinstance(result, tuple) else (result or [])
        not_attempted = [r for r in results if r.get("not_attempted")]
        failed = [r for r in results if not r.get("ok", False) and not r.get("not_attempted")]
        written = {r.get("name") for r in results if r.get("ok", False)}
        reboot = [r.get("name") for r in results if r.get("ok") and r.get("needs_reboot")] or \
            [d["name"] for d in diffs if d["needs_reboot"] and d["name"] in written]
        self._note_reboot(vehicle, reboot, REBOOT_APPLY)
        parts = [f"Wrote {len(written)} parameter(s)."]
        if failed:
            parts.append("Failed: " + ", ".join(f"{r.get('name')} ({r.get('error') or 'not confirmed'})" for r in failed))
        if not_attempted:
            parts.append("Not written: " + ", ".join(r.get("name") for r in not_attempted)
                         + f" ({not_attempted[0].get('error')}).")
        level = "bad" if failed or not_attempted else "ok"
        if stale:
            self._after_stale_write(vehicle, " ".join(parts), level)
            self._finish_writing()
            return
        self._take_backup(snapshot)
        # Show what was confirmed now: if the re-check below fails, the table isn't left stale
        for row in self.rows:
            if row["name"] in written:
                row["current"] = row["recommended"]
                row["ok"] = True
        self._fill_table()
        # Kept for the re-check's message, which would otherwise replace this one at once
        self._apply_note = (" ".join(parts), level)
        if self.reboot_pending:
            parts.append(self._reboot_text())
        self._set_busy(False, " ".join(parts), level if level == "bad" else "warn" if self.reboot_pending else "ok")
        self._finish_writing()
        self.check()   # show what the autopilot now reports (not when the window is closing)

    def _on_revert_preview(self, params, result: dict, generation: Optional[int] = None) -> None:
        if self._stale(generation):
            return
        if self._cancel.is_set():   # the window is closing: never open a dialog now
            self._set_busy(False)
            return
        self._take_backup(result.get("backup"))
        rows = result.get("rows") or []
        self._fill_table()
        if not rows:
            self._set_busy(False, "Nothing to revert: TrackShot has no recorded changes for this autopilot.", "ok")
            return
        self._set_busy(False)
        if not self.confirm_revert(rows):
            self._set_message("Revert cancelled; nothing was changed.", "neutral")
            return
        if self.busy:     # a check started while the dialog was open
            return
        key = (self.backup_info or {}).get("key")
        self._start_writing("Restoring the parameters' previous values…")
        vehicle = self._vehicle_key()
        run_in_pool(self._pool, lambda: revert_task(params, key),
                    lambda r: self._on_reverted(r, generation, vehicle),
                    lambda e: self._on_write_failed(e, generation, vehicle))

    def _on_reverted(self, result: dict, generation: Optional[int] = None,
                     vehicle: Optional[str] = None) -> None:
        vehicle = self._vehicle_key() if vehicle is None else vehicle
        stale = self._write_was_stale(generation)
        results = result.get("results") or []
        restored = [r for r in results if r.get("ok")]
        skipped = [r for r in results if r.get("skipped")]
        not_attempted = [r for r in results if r.get("not_attempted")]
        failed = [r for r in results if not r.get("ok") and not r.get("skipped") and not r.get("not_attempted")]
        drifted = [r["name"] for r in restored if r.get("drifted")]
        reboot = [r["name"] for r in restored if r.get("needs_reboot")]
        self._note_reboot(vehicle, reboot, REBOOT_REVERT)
        parts = [f"Restored {len(restored)} parameter(s) to their values from before TrackShot."]
        if drifted:
            parts.append("Note: " + ", ".join(drifted) + " had been changed since TrackShot wrote "
                         "them; the value from before TrackShot was restored anyway.")
        if skipped:
            parts.append("Not restorable: " + ", ".join(f"{r['name']} ({r.get('error')})" for r in skipped) + ".")
        if failed:
            parts.append("Failed: " + ", ".join(f"{r.get('name')} ({r.get('error') or 'not confirmed'})"
                                                for r in failed) + " (kept in the backup: try again).")
        if not_attempted:
            parts.append("Not restored yet: " + ", ".join(r["name"] for r in not_attempted)
                         + f" ({not_attempted[0].get('error')}; kept in the backup).")
        level = "bad" if failed or not_attempted else "warn" if (drifted or skipped) else "ok"
        if stale:
            self._after_stale_write(vehicle, " ".join(parts), level)
            self._finish_writing()
            return
        self._take_backup(result.get("backup"))
        # Show what was confirmed now: if the re-check below fails, the table isn't left stale
        originals = {r["name"]: r.get("original") for r in restored if not r.get("skipped")}
        for row in self.rows:
            if row["name"] in originals and originals[row["name"]] is not None:
                row["current"] = originals[row["name"]]
                row["ok"] = value_ok(row["name"], row["current"], row["recommended"])
        self._fill_table()
        self._apply_note = (" ".join(parts), level)
        if self.reboot_pending:
            parts.append(self._reboot_text())
        self._set_busy(False, " ".join(parts), "bad" if level == "bad" else "warn" if self.reboot_pending else level)
        self._finish_writing()
        self.check()   # show what the autopilot now reports (not when the window is closing)

    def _on_failed(self, error: BaseException, generation: Optional[int] = None) -> None:
        """A check or revert preview failed (or an apply/revert, via _on_write_failed)."""
        if self._stale(generation):
            return
        note, self._apply_note = self._apply_note, None
        if note is None:
            self._set_busy(False)
            self._show_error(error)
            return
        # The re-check after an apply/revert failed: what was written still has to be said
        text = f"{note[0]} Re-reading the parameters afterwards failed: {describe_error(error)}"
        if self.reboot_pending:
            text += " " + self._reboot_text()
        self._set_busy(False, text, "bad")

    def _on_write_failed(self, error: BaseException, generation: Optional[int], vehicle: str) -> None:
        """An apply or revert raised before writing anything (armed, unreachable, backup unusable…)."""
        if self._write_was_stale(generation):
            self._after_stale_write(vehicle, describe_error(error), "bad")
        else:
            self._apply_note = None
            self._set_busy(False)
            self._show_error(error)
        self._finish_writing()

    # --- Display --------------------------------------------------------------------------------
    def _show_error(self, error: BaseException) -> None:
        self._set_message(describe_error(error), "bad")

    def _set_busy(self, busy: bool, message: Optional[str] = None, level: str = "neutral") -> None:
        self.busy = busy
        self.check_button.setEnabled(not busy)
        self.apply_button.setEnabled(not busy and any(not r["ok"] for r in self.rows))
        self.revert_button.setEnabled(not busy and bool(self.backup_entries))
        if message is not None:
            self._set_message(message, level)

    def _set_message(self, text: str, level: str) -> None:
        theme = current_theme(self.palette())
        self.message.setText(text)
        self.message.setStyleSheet(f"color: {css(theme.level(level) if level != 'neutral' else theme.text)};")

    def _fill_table(self) -> None:
        theme = current_theme(self.palette())
        self.table.setRowCount(len(self.rows))
        for i, row in enumerate(self.rows):
            kind = self._reboot.get(self._vehicle_key(), {}).get(row["name"])
            pending = kind is not None
            entry = self.backup_entries.get(row["name"])
            cells = (row["name"], _num(row["current"]), _num(row["recommended"]), before_text(entry),
                     "Pending" if pending else "Yes" if row["needs_reboot"] else "No", "✔" if row["ok"] else "✖")
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == COL_REBOOT and pending:
                    item.setForeground(theme.warn)
                    item.setToolTip("Restored; the autopilot keeps its current setup until it reboots"
                                    if kind == REBOOT_REVERT else
                                    "Written; takes effect when the autopilot reboots")
                elif col == COL_OK:
                    item.setForeground(theme.ok if row["ok"] else theme.bad)
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                elif col in (COL_CURRENT, COL_RECOMMENDED, COL_BEFORE):
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                if col == COL_BEFORE and entry is not None:
                    tip, drifted = before_tooltip(entry, row["current"])
                    item.setToolTip(tip)
                    if drifted:
                        item.setForeground(theme.warn)
                self.table.setItem(i, col, item)
