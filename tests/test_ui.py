"""UI tests against FakeEngine (pytest-qt, offscreen)."""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from PySide6.QtCore import QSettings, Qt  # noqa: E402

from fakes import FakeAutopilotParams, FakeEngine, lawnmower_pose, make_preview, make_status, odometry_health  # noqa: E402
from trackshot.config import BACKEND_DEFAULT_FPS, Settings  # noqa: E402
from trackshot.ui import main_window as main_window_module  # noqa: E402
from trackshot.ui.main_window import MainWindow  # noqa: E402
from trackshot.ui.panels import settings_panel  # noqa: E402
from trackshot.ui.panels.autopilot_panel import COL_BEFORE, COL_REBOOT  # noqa: E402
from trackshot.ui.panels.settings_panel import NOT_AVAILABLE_TIP  # noqa: E402
from trackshot.vio.source import STATE_RUNNING, STATE_STOPPED  # noqa: E402

SCREENSHOT = Path(__file__).resolve().parents[1] / "docs" / "screenshot-ui-fake.png"


@pytest.fixture(autouse=True)
def isolated_qsettings(tmp_path):
    """Window geometry goes to a temporary INI file, not the user's preferences."""
    QSettings.setDefaultFormat(QSettings.Format.IniFormat)
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope, str(tmp_path / "qsettings"))
    yield
    QSettings.setDefaultFormat(QSettings.Format.NativeFormat)


@pytest.fixture
def backends(monkeypatch):
    available = ["rtabmap", "basalt"]
    monkeypatch.setattr(settings_panel, "available_backends", lambda: list(available))
    return available


@pytest.fixture
def make_window(qtbot, tmp_path, backends):
    def make(**settings_overrides) -> MainWindow:
        settings = Settings(capture_dir=str(tmp_path / "captures"), **settings_overrides)
        engine = FakeEngine(settings)
        win = MainWindow(engine, settings_path=tmp_path / "settings.json")
        qtbot.addWidget(win)
        win.show()
        return win
    return make


@pytest.fixture
def window(make_window) -> MainWindow:
    return make_window()


def card_text(win: MainWindow, key: str) -> str:
    return win.status_panel.cards[key].value.text()


# --- Structure and toolbar ---------------------------------------------------------------------

def test_window_builds(window):
    assert window.windowTitle() == "TrackShot"
    assert set(window.docks) == {"settings", "status", "bandwidth", "autopilot", "capture", "log"}
    assert window.start_action.isEnabled() and not window.stop_action.isEnabled()
    assert window.source_combo.currentData() == "oak"
    assert window.preview_action.isChecked()


def test_start_stop_follow_state(window):
    engine = window.engine
    window.start_action.trigger()
    assert engine.called("start")
    assert not window.start_action.isEnabled() and window.stop_action.isEnabled()
    assert card_text(window, "state") == "Running"
    assert "RUNNING" in window.state_badge.text()

    window.stop_action.trigger()
    assert engine.called("stop")
    assert window.start_action.isEnabled() and not window.stop_action.isEnabled()
    assert card_text(window, "state") == "Stopped"


def test_source_selector_applies_without_taking_form_edits(window, tmp_path):
    window.settings_panel.fps.setValue(17)   # an edit in progress in the form
    window.source_combo.setCurrentIndex(window.source_combo.findData("sim"))
    applied = window.engine.applied[-1]
    assert applied.source == "sim"
    assert applied.fps == BACKEND_DEFAULT_FPS["rtabmap"]          # the form edit wasn't applied…
    assert window.settings_panel.pending().fps == 17               # …and wasn't lost
    assert json.loads((tmp_path / "settings.json").read_text())["source"] == "sim"


def test_preview_and_record_toggles(window):
    window.preview_action.setChecked(False)
    assert window.engine.called("set_preview")[-1] == ("set_preview", False, False)
    assert window.preview.vio.meta.text() == "Preview off"

    window.record_action.setChecked(True)
    assert window.engine.called("set_recording")[-1] == ("set_recording", True)
    # The engine reports recording stopped (e.g. disk full): the toggle follows without re-calling it
    window.engine.statusUpdated.emit(make_status(recording=False))
    assert not window.record_action.isChecked()
    assert len(window.engine.called("set_recording")) == 1


# --- Engine signals ------------------------------------------------------------------------------

@pytest.mark.parametrize("as_dict", [False, True])
def test_status_updates_cards(window, as_dict):
    odo = odometry_health()
    if as_dict and not isinstance(odo, dict):
        odo = dict(vars(odo))
    window.engine.stateChanged.emit(STATE_RUNNING, "VIO running")
    window.engine.statusUpdated.emit(make_status(odometry=odo))
    assert card_text(window, "fps") == "9.8 fps"
    assert card_text(window, "latency") == "64 ms"
    assert card_text(window, "viso_delay") == "60 ms"
    assert card_text(window, "tracking") == "Good"
    assert card_text(window, "jumps") == "1"
    assert card_text(window, "mavlink") == "Connected"
    assert "dropped 3" in window.status_panel.cards["mavlink"].detail.text()
    assert card_text(window, "camera") == "OAK-D-W-PoE"
    assert "BNO086" in window.status_panel.cards["camera"].detail.text()
    assert card_text(window, "vehicle") == "Disarmed"
    assert "4.37 m" in window.status_panel.cards["vehicle"].detail.text()
    assert "18.4" in window.bandwidth.measured.text()
    assert "412" in window.capture.counts.text()
    assert "60 ms" in window.autopilot.suggested_label.text()


def test_status_tolerates_missing_parts(window):
    window.engine.statusUpdated.emit({"state": "idle", "source": None, "odometry": None, "transport": None,
                                      "capture": None, "telemetry": None, "bandwidth": None})
    assert card_text(window, "fps") == "—"
    assert card_text(window, "vehicle") == "No link"


def test_lost_tracking_is_red(window):
    window.engine.statusUpdated.emit(make_status(odometry=odometry_health(tracking="lost", stalled=True)))
    assert card_text(window, "tracking") == "Lost"
    assert window.status_panel.cards["tracking"].level == "lost"


def test_frozen_vio_card_says_not_sending_and_how_long(window):
    odo = odometry_health(tracking="lost", sending=False, lost_for_s=12.4, losses=2)
    window.engine.statusUpdated.emit(make_status(odometry=odo))
    card = window.status_panel.cards["tracking"]
    assert card_text(window, "tracking") == "Lost 12 s"
    assert card.level == "lost"
    assert "Not sending" in card.detail.text() and "Restart VIO" in card.detail.text()
    assert "lost 2× this run" in card.detail.text()


def test_acquiring_card(window):
    window.engine.statusUpdated.emit(make_status(odometry=odometry_health(tracking="acquiring", sending=False)))
    assert card_text(window, "tracking") == "Acquiring"
    assert window.status_panel.cards["tracking"].level == "acquiring"
    assert "Not sending" in window.status_panel.cards["tracking"].detail.text()


def test_restart_vio_button_follows_the_engine(window):
    engine = window.engine
    assert not window.restart_action.isEnabled()          # not running
    window.start_action.trigger()
    assert window.restart_action.isEnabled()
    window.restart_action.trigger()
    assert engine.called("restart_vio")
    # Cooling down: disabled, and the tooltip says why
    engine.restart_blocked = "restarted moments ago; available again in 18 s"
    engine.statusUpdated.emit(make_status())
    assert not window.restart_action.isEnabled()
    assert "available again in 18 s" in window.restart_action.toolTip()
    engine.restart_blocked = None
    engine.statusUpdated.emit(make_status())
    assert window.restart_action.isEnabled()
    assert "Not available" not in window.restart_action.toolTip()


def test_pose_updates_add_trajectory_points(window):
    for i in range(50):
        window.engine.poseUpdated.emit(lawnmower_pose(i))
    assert window.trajectory.point_count() == 50
    window.trajectory.refresh()
    xs, _ = window.trajectory.path_curve.getData()
    assert len(xs) == 50
    assert window.trajectory.arrow.isVisible()
    window.trajectory.clear()
    assert window.trajectory.point_count() == 0


def test_trajectory_caps_points():
    from trackshot.ui.panels import trajectory_panel
    buffer = trajectory_panel.RingBuffer(100, 2)
    for i in range(250):
        buffer.append((i, i))
    assert len(buffer) <= 100
    assert buffer.view()[-1, 0] == 249   # newest kept, oldest dropped


def test_preview_shows_images(window):
    frame = make_preview("vio")
    window.engine.previewUpdated.emit(frame)
    assert window.preview.vio.image.image.shape == (200, 320)
    assert len(window.preview.vio.features.data) == 60
    window.engine.previewUpdated.emit(make_preview("color", 640, 360))
    assert window.preview.color.isVisible()
    assert window.preview.color.image.image.shape == (360, 640, 3)
    assert "0 Mbit/s extra" in window.preview.note.text()


def test_log_messages(window):
    window.engine.logMessage.emit("warning", "Tracking degraded <3 features>")
    window.engine.logMessage.emit("error", "Camera disconnected")
    text = window.log.view.toPlainText()
    assert "Tracking degraded <3 features>" in text and "Camera disconnected" in text
    assert window.log.view.maximumBlockCount() > 0


# --- Settings form -------------------------------------------------------------------------------

def test_backend_switch_sets_fps(window):
    panel = window.settings_panel
    panel.fps.setValue(15)
    panel.backend.setCurrentIndex(panel.backend.findData("basalt"))
    assert panel.fps.value() == BACKEND_DEFAULT_FPS["basalt"]
    assert panel.basalt_box.isVisibleTo(panel) and not panel.rtabmap_box.isVisibleTo(panel)


def test_unavailable_backend_disabled(make_window, backends):
    backends[:] = ["rtabmap"]
    win = make_window()
    panel = win.settings_panel
    i = panel.backend.findData("basalt")
    assert not panel.backend.model().item(i).isEnabled()
    assert panel.backend.itemData(i, Qt.ItemDataRole.ToolTipRole) == NOT_AVAILABLE_TIP


def test_bandwidth_estimate_follows_pending_edits(window):
    panel, bw = window.settings_panel, window.bandwidth
    before = bw.estimate
    panel.color_preview.setChecked(True)
    assert bw.estimate.color_preview_mbit > 0
    assert bw.estimate.total_mbit > before.total_mbit
    assert "not applied" in bw.caption.text()

    vio_before = bw.estimate.vio_mbit
    panel.fps.setValue(20)
    assert bw.estimate.vio_mbit > vio_before
    assert window.engine.applied == []   # nothing applied yet


def test_apply_saves_and_hands_copy_to_engine(window, tmp_path):
    panel = window.settings_panel
    assert not panel.apply_button.isEnabled()
    panel.fps.setValue(12)
    assert panel.apply_button.isEnabled()
    panel.apply_button.click()

    applied = window.engine.applied[-1]
    assert applied.fps == 12
    assert applied is not window.settings_panel.base
    assert json.loads((tmp_path / "settings.json").read_text())["fps"] == 12
    assert Settings.load(tmp_path / "settings.json").fps == 12
    assert not panel.apply_button.isEnabled()
    assert "applied settings" in window.bandwidth.caption.text()


def test_validation_blocks_apply(window):
    panel = window.settings_panel
    tether = [w for w in panel.findChildren(type(panel.camera_angle)) if w.suffix() == " Mbit/s"][0]
    tether.setValue(0)
    assert panel.problems_label.isVisibleTo(panel)
    assert "Tether budget" in panel.problems_label.text()
    assert not panel.apply_button.isEnabled()


def test_revert_restores_applied(window):
    panel = window.settings_panel
    panel.fps.setValue(25)
    panel.revert_button.click()
    assert panel.fps.value() == BACKEND_DEFAULT_FPS["rtabmap"]
    assert not panel.is_dirty()


def test_find_cameras_runs_in_background(window, qtbot, monkeypatch):
    threads = []

    def fake_list():
        threads.append(threading.get_ident())
        return [{"name": "OAK-D-W-PoE", "ip": "192.168.2.118", "state": "available"}]
    monkeypatch.setattr(settings_panel, "list_cameras", fake_list)
    panel = window.settings_panel
    panel.find_button.click()
    qtbot.waitUntil(lambda: panel.found_cameras.count() == 1, timeout=3000)
    assert threads[0] != threading.main_thread().ident
    panel.camera_ip.setText("")
    panel._on_camera_chosen(0)
    assert panel.camera_ip.text() == "192.168.2.118"


# --- Autopilot -----------------------------------------------------------------------------------

def test_autopilot_check_runs_off_gui_thread(window, qtbot):
    params = window.engine.params = FakeAutopilotParams(delay_s=0.05)
    window.engine.statusUpdated.emit(make_status())          # suggested VISO_DELAY_MS = 60
    window.autopilot.check_button.click()
    assert window.autopilot.busy                              # returned at once; work is elsewhere
    qtbot.waitUntil(lambda: window.autopilot.table.rowCount() == 5, timeout=3000)
    assert params.threads and all(t != threading.main_thread().ident for t in params.threads)
    assert params.check_calls == [60]
    assert window.autopilot.apply_button.isEnabled()


def test_autopilot_apply_confirms_then_writes(window, qtbot, monkeypatch):
    params = window.engine.params
    window.autopilot.check()
    qtbot.waitUntil(lambda: window.autopilot.table.rowCount() == 4, timeout=3000)

    monkeypatch.setattr(window.autopilot, "confirm_apply", lambda diffs: False)
    window.autopilot.apply_button.click()
    assert params.apply_calls == []

    monkeypatch.setattr(window.autopilot, "confirm_apply", lambda diffs: True)
    window.autopilot.apply_button.click()
    qtbot.waitUntil(lambda: bool(params.apply_calls) and not window.autopilot.busy
                    and all(r["ok"] for r in window.autopilot.rows), timeout=3000)
    assert [d["name"] for d in params.apply_calls[0]] == ["VISO_TYPE", "EK3_SRC1_POSXY"]
    assert all(t != threading.main_thread().ident for t in params.threads)


def test_reboot_reminder_survives_the_recheck_until_the_autopilot_restarts(window, qtbot, monkeypatch):
    panel = window.autopilot
    panel.check()
    qtbot.waitUntil(lambda: panel.table.rowCount() == 4 and not panel.busy, timeout=3000)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    # The apply's automatic re-check finishes with everything ok…
    qtbot.waitUntil(lambda: not panel.busy and all(r["ok"] for r in panel.rows), timeout=3000)
    # …and the reboot instruction is still there, with the written parameter marked
    assert "Wrote 2 parameter(s)." in panel.message.text()
    assert "Reboot the autopilot for VISO_TYPE" in panel.message.text()
    visotype = [r["name"] for r in panel.rows].index("VISO_TYPE")
    assert panel.table.item(visotype, COL_REBOOT).text() == "Pending"
    panel.check()                                            # a later check still says it
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    assert "Reboot the autopilot for VISO_TYPE" in panel.message.text()

    # The autopilot goes quiet (rebooting), then comes back: the reminder clears
    window.engine.statusUpdated.emit(make_status(armed=False))
    assert panel.reboot_pending == ["VISO_TYPE"]
    status = make_status()
    status["telemetry"] = dict(status["telemetry"], connected=False, heartbeat_age_s=None)
    window.engine.statusUpdated.emit(status)
    window.engine.statusUpdated.emit(make_status())
    assert panel.reboot_pending == []
    assert "restarted" in panel.message.text()
    assert panel.table.item(visotype, COL_REBOOT).text() == "Yes"


def test_a_failed_write_is_still_reported_after_the_recheck(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: panel.table.rowCount() == 4 and not panel.busy, timeout=3000)
    monkeypatch.setattr(params, "apply", lambda diffs, backup=None: [{"name": "VISO_TYPE", "ok": False,
                                                         "error": "write not confirmed"}])
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and len(params.check_calls) == 2, timeout=3000)
    assert "Failed: VISO_TYPE (write not confirmed)" in panel.message.text()


def test_apply_then_revert_with_fake_params(window, qtbot, monkeypatch, isolated_param_backups):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: panel.table.rowCount() == 4 and not panel.busy, timeout=3000)
    assert not panel.revert_button.isEnabled()                      # nothing recorded yet
    assert all(panel.table.item(i, COL_BEFORE).text() == "" for i in range(4))

    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and len(params.check_calls) == 2, timeout=3000)
    names = [r["name"] for r in panel.rows]
    assert panel.table.item(names.index("VISO_TYPE"), COL_BEFORE).text() == "0"
    assert panel.table.item(names.index("EK3_SRC1_POSXY"), COL_BEFORE).text() == "3"
    assert panel.table.item(names.index("AHRS_EKF_TYPE"), COL_BEFORE).text() == ""
    assert panel.revert_button.isEnabled()
    assert (isolated_param_backups / "ap-fake0001.json").exists()

    shown = []
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: shown.append(rows) or False)
    panel.revert_button.click()
    qtbot.waitUntil(lambda: bool(shown) and not panel.busy, timeout=3000)
    assert [r["name"] for r in shown[0]] == ["EK3_SRC1_POSXY", "VISO_TYPE"]   # newest change first
    assert params.revert_calls == 0 and "cancelled" in panel.message.text()

    monkeypatch.setattr(panel, "confirm_revert", lambda rows: True)
    panel.reboot_pending = []
    panel.revert_button.click()
    qtbot.waitUntil(lambda: params.revert_calls == 1 and not panel.busy and len(params.check_calls) == 3,
                    timeout=3000)
    assert "Restored 2 parameter(s)" in panel.message.text()
    assert "Reboot the autopilot for VISO_TYPE" in panel.message.text()
    assert panel.reboot_pending == ["VISO_TYPE"]
    assert panel.table.item(names.index("VISO_TYPE"), COL_REBOOT).text() == "Pending"
    assert {r["name"]: r["current"] for r in panel.rows}["VISO_TYPE"] == 0.0
    assert not panel.revert_button.isEnabled()                      # the backup is empty again
    assert panel.table.item(names.index("VISO_TYPE"), COL_BEFORE).text() == ""
    assert all(t != threading.main_thread().ident for t in params.threads)


def test_revert_while_armed_is_explained(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and len(params.check_calls) == 2, timeout=3000)
    params.armed = True
    asked = []
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: asked.append(rows) or True)
    panel.revert_button.click()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    assert asked == [] and params.revert_calls == 0
    assert "disarm" in panel.message.text()
    assert panel.revert_button.isEnabled()                          # still revertible once disarmed


def test_revert_summary_flags_drift_reboot_and_disarm():
    from trackshot.ui.panels.autopilot_panel import revert_summary
    text = revert_summary([
        {"name": "VISO_DELAY_MS", "current": 90.0, "original": 10.0, "written_by_trackshot": 120.0,
         "drifted": True, "unconfirmed": False, "needs_reboot": False, "action": "restore"},
        {"name": "VISO_TYPE", "current": 1.0, "original": 0.0, "written_by_trackshot": 1.0,
         "drifted": False, "unconfirmed": False, "needs_reboot": True, "action": "restore"},
        {"name": "VISO_SCALE", "current": 1.0, "original": None, "absent": True, "action": "skip"},
    ])
    assert "VISO_DELAY_MS: 90 → 10  (changed since TrackShot wrote 120)" in text
    assert "VISO_TYPE: 1 → 0  (needs reboot)" in text
    assert "VISO_SCALE: didn't exist before TrackShot" in text
    assert "disarmed" in text and "Warning" in text


def test_changing_the_vehicle_clears_its_backup_view(window, qtbot, monkeypatch, tmp_path):
    panel = window.autopilot
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and panel.revert_button.isEnabled(), timeout=3000)
    window.apply_settings(Settings(capture_dir=str(tmp_path / "captures"), vehicle_ip="192.168.2.3"))
    assert panel.rows == [] and panel.backup_entries == {}
    assert not panel.revert_button.isEnabled() and not panel.apply_button.isEnabled()


def test_panel_against_the_fake_autopilot_apply_then_revert(window, qtbot, monkeypatch, isolated_param_backups):
    """The real AutopilotParams and ParamBackup behind the panel, against the fake mavlink2rest."""
    from trackshot.autopilot.params import AutopilotParams
    from trackshot.testing.fake_autopilot import FakeAutopilot
    panel = window.autopilot
    with FakeAutopilot() as fake:
        before = dict(fake.params)
        window.engine.autopilot_params = lambda: AutopilotParams(fake.http_base, timeout=1.0)
        panel.check()
        qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=10000)
        assert not panel.revert_button.isEnabled()
        monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
        panel.apply_button.click()
        qtbot.waitUntil(lambda: not panel.busy and all(r["ok"] for r in panel.rows), timeout=10000)
        assert panel.revert_button.isEnabled()
        names = [r["name"] for r in panel.rows]
        assert panel.table.item(names.index("EK3_SRC_OPTIONS"), COL_BEFORE).text() == "1"
        [backup_file] = isolated_param_backups.glob("*.json")
        assert backup_file.name == f"ap-{fake.uid:016x}.json"

        fake.params["EK3_SRC1_POSXY"] = 2.0                         # changed outside TrackShot
        shown = []
        monkeypatch.setattr(panel, "confirm_revert", lambda rows: shown.append(rows) or True)
        panel.revert_button.click()
        qtbot.waitUntil(lambda: not panel.busy and not panel.revert_button.isEnabled() and bool(shown), timeout=10000)
        assert {r["name"]: r["drifted"] for r in shown[0]}["EK3_SRC1_POSXY"] is True
        assert fake.params == before
        assert "EK3_SRC1_POSXY had been changed" in panel.message.text()
        assert "VISO_TYPE" in panel.reboot_pending


def checked_and_applied(panel, qtbot, monkeypatch):
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and panel.revert_button.isEnabled(), timeout=3000)


def blocking(params, method: str, monkeypatch) -> threading.Event:
    """Makes params.<method> wait (on its worker thread) until the returned event is set."""
    release = threading.Event()
    real = getattr(params, method)

    def wait_then_run(*args, **kwargs):
        assert release.wait(5.0)
        return real(*args, **kwargs)
    monkeypatch.setattr(params, method, wait_then_run)
    return release


def other_vehicle(tmp_path, ip="192.168.2.3") -> Settings:
    return Settings(capture_dir=str(tmp_path / "captures"), vehicle_ip=ip)


def test_reboot_reminders_belong_to_their_vehicle(window, qtbot, monkeypatch, tmp_path):
    panel = window.autopilot
    vehicle_a = window._applied.vehicle_ip
    checked_and_applied(panel, qtbot, monkeypatch)
    assert panel.reboot_pending == ["VISO_TYPE"]

    window.apply_settings(other_vehicle(tmp_path))                 # vehicle B: nothing written there
    assert panel.reboot_pending == []
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    assert "Reboot" not in panel.message.text()
    visotype = [r["name"] for r in panel.rows].index("VISO_TYPE")
    assert panel.table.item(visotype, COL_REBOOT).text() == "Yes"
    # B's telemetry connecting (down, then up) is not a restart of A
    status = make_status()
    status["telemetry"] = dict(status["telemetry"], connected=False, heartbeat_age_s=None)
    window.engine.statusUpdated.emit(status)
    window.engine.statusUpdated.emit(make_status())
    assert "restarted" not in panel.message.text()

    window.apply_settings(other_vehicle(tmp_path, vehicle_a))      # back to A: its reminder is back
    assert panel.reboot_pending == ["VISO_TYPE"]


def test_telemetry_still_connecting_after_an_apply_is_not_a_restart(window, qtbot, monkeypatch):
    panel = window.autopilot
    checked_and_applied(panel, qtbot, monkeypatch)
    status = make_status()
    status["telemetry"] = dict(status["telemetry"], connected=False, heartbeat_age_s=None)
    window.engine.statusUpdated.emit(status)                       # VIO just started: not up yet
    window.engine.statusUpdated.emit(make_status())
    assert panel.reboot_pending == ["VISO_TYPE"]


def test_an_apply_that_finishes_after_the_vehicle_changed_is_not_shown_as_the_new_ones(
        window, qtbot, monkeypatch, tmp_path):
    panel = window.autopilot
    params = window.engine.params
    vehicle_a = window._applied.vehicle_ip
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    release = blocking(params, "apply", monkeypatch)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    assert panel.writing
    window.apply_settings(other_vehicle(tmp_path))                 # changed while it writes
    release.set()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    text = panel.message.text()
    assert f"Previous vehicle ({vehicle_a}): Wrote 2 parameter(s)." in text
    assert "Check reads the new one" in text
    assert panel.reboot_pending == [] and panel.rows == [] and panel.backup_entries == {}
    assert len(params.check_calls) == 1                            # no automatic re-check of B
    window.apply_settings(other_vehicle(tmp_path, vehicle_a))
    assert panel.reboot_pending == ["VISO_TYPE"]                   # filed under A


def test_a_revert_that_finishes_after_the_vehicle_changed_is_not_shown_as_the_new_ones(
        window, qtbot, monkeypatch, tmp_path):
    panel = window.autopilot
    params = window.engine.params
    checked_and_applied(panel, qtbot, monkeypatch)
    panel.reboot_pending = []
    release = blocking(params, "revert", monkeypatch)
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: True)
    panel.revert_button.click()
    qtbot.waitUntil(lambda: panel.writing, timeout=3000)
    checks = len(params.check_calls)
    window.apply_settings(other_vehicle(tmp_path))
    release.set()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    assert "Previous vehicle" in panel.message.text() and "Restored 2 parameter(s)" in panel.message.text()
    assert panel.reboot_pending == [] and panel.backup_entries == {} and len(params.check_calls) == checks


def test_a_failed_recheck_after_a_revert_keeps_its_outcome(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    checked_and_applied(panel, qtbot, monkeypatch)
    panel.reboot_pending = []
    monkeypatch.setattr(params, "check", lambda suggested=None: (_ for _ in ()).throw(TimeoutError("timed out")))
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: True)
    panel.revert_button.click()
    qtbot.waitUntil(lambda: params.revert_calls == 1 and not panel.busy, timeout=3000)
    text = panel.message.text()
    assert "Restored 2 parameter(s)" in text and "timed out" in text
    assert "Reboot the autopilot for VISO_TYPE" in text
    names = [r["name"] for r in panel.rows]
    # The table shows the restored values, not the ones from before the revert
    assert panel.table.item(names.index("VISO_TYPE"), 1).text() == "0"
    assert panel.table.item(names.index("EK3_SRC1_POSXY"), 1).text() == "3"
    assert panel.table.item(names.index("VISO_TYPE"), COL_BEFORE).text() == ""
    assert panel.apply_button.isEnabled() and not panel.revert_button.isEnabled()


def test_a_failed_recheck_after_an_apply_keeps_its_outcome(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    real_apply = params.apply

    def apply_then_link_drops(diffs, backup=None):
        results = real_apply(diffs, backup)
        params.error = TimeoutError("timed out")                   # the re-check fails
        return results
    monkeypatch.setattr(params, "apply", apply_then_link_drops)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and len(params.check_calls) == 2, timeout=3000)
    text = panel.message.text()
    assert "Wrote 2 parameter(s)" in text and "timed out" in text and "Reboot the autopilot for VISO_TYPE" in text
    names = [r["name"] for r in panel.rows]
    assert panel.table.item(names.index("VISO_TYPE"), 1).text() == "1"
    assert panel.table.item(names.index("VISO_TYPE"), COL_REBOOT).text() == "Pending"
    assert panel.table.item(names.index("VISO_TYPE"), COL_BEFORE).text() == "0"
    assert panel.revert_button.isEnabled()


def test_a_stopped_apply_says_what_wasnt_written(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    monkeypatch.setattr(params, "apply", lambda diffs, backup=None: [
        {"name": "VISO_TYPE", "ok": True, "needs_reboot": True, "error": None},
        {"name": "EK3_SRC1_POSXY", "ok": False, "needs_reboot": False,
         "error": "not written: the vehicle was armed", "not_attempted": True}])
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and len(params.check_calls) == 2, timeout=3000)
    text = panel.message.text()
    assert "Wrote 1 parameter(s)." in text
    assert "Not written: EK3_SRC1_POSXY (not written: the vehicle was armed)." in text
    assert "Failed" not in text


def test_reboot_reminder_after_a_revert_says_what_the_reboot_does(window, qtbot, monkeypatch):
    panel = window.autopilot
    checked_and_applied(panel, qtbot, monkeypatch)
    assert "can't use visual odometry" in panel.message.text()
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: True)
    panel.revert_button.click()
    qtbot.waitUntil(lambda: not panel.busy and not panel.revert_button.isEnabled(), timeout=3000)
    text = panel.message.text()
    assert "Reboot the autopilot for VISO_TYPE to put the setup from before TrackShot back in effect" in text
    assert "can't use visual odometry" not in text
    visotype = [r["name"] for r in panel.rows].index("VISO_TYPE")
    item = panel.table.item(visotype, COL_REBOOT)
    assert item.text() == "Pending" and "Restored" in item.toolTip()


def test_closing_while_writing_waits_for_the_writes(window, qtbot, monkeypatch):
    from trackshot.__main__ import POOL_EXIT_WAIT_MS, exit_wait_ms
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy and bool(panel.rows), timeout=3000)
    release = blocking(params, "apply", monkeypatch)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    assert exit_wait_ms(window) == -1                               # main() would wait it out
    window.close()
    assert window.isVisible() and not window.engine.called("stop")  # the close waits
    assert "closes when they are done" in panel.message.text()
    release.set()
    qtbot.waitUntil(lambda: not window.isVisible(), timeout=3000)   # …then happens by itself
    assert window.engine.called("stop")
    assert all(r["ok"] for r in params.rows if r["name"] in ("VISO_TYPE", "EK3_SRC1_POSXY"))
    assert len(params.check_calls) == 1                             # no re-check while closing
    assert exit_wait_ms(window) == POOL_EXIT_WAIT_MS


def test_closing_cancels_a_running_check(make_window):
    win = make_window()
    params = win.engine.params
    win.autopilot.check()
    win.close()
    assert params.cancel.is_set()


def test_closing_cancels_a_revert_preview(window, qtbot, monkeypatch):
    panel = window.autopilot
    params = window.engine.params
    panel.check()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    monkeypatch.setattr(panel, "confirm_apply", lambda diffs: True)
    panel.apply_button.click()
    qtbot.waitUntil(lambda: not panel.busy and panel.revert_button.isEnabled(), timeout=3000)
    params.cancel = None
    panel.revert_previous()
    assert panel.busy and params.cancel is not None and not params.cancel.is_set()
    asked = []
    monkeypatch.setattr(panel, "confirm_revert", lambda rows: asked.append(rows) or True)
    window.close()
    assert params.cancel.is_set()
    qtbot.waitUntil(lambda: not panel.busy, timeout=3000)
    assert asked == [] and params.revert_calls == 0     # no dialog, no revert after closing


def test_toolbar_toggles_never_save_session_overrides(qtbot, tmp_path, backends):
    """--sim / --vehicle-ip apply to this session only: Preview and Source save just their own field."""
    path = tmp_path / "settings.json"
    on_disk = Settings(capture_dir=str(tmp_path / "captures"))
    on_disk.save(path)
    session = Settings.load(path)
    session.source, session.vehicle_ip = "sim", "127.0.0.1:8088"     # as __main__ applies the flags
    win = MainWindow(FakeEngine(session), settings_path=path, saved_settings=Settings.load(path))
    qtbot.addWidget(win)
    win.preview_action.setChecked(False)
    saved = json.loads(path.read_text())
    assert (saved["source"], saved["vehicle_ip"], saved["preview_enabled"]) == ("oak", "192.168.2.2", False)
    assert win.engine.called("set_preview")                          # the toggle still acts at once

    win.source_combo.setCurrentIndex(win.source_combo.findData("oak"))
    win.source_combo.setCurrentIndex(win.source_combo.findData("sim"))
    saved = json.loads(path.read_text())
    assert (saved["source"], saved["vehicle_ip"]) == ("sim", "192.168.2.2")
    assert win.engine.applied[-1].vehicle_ip == "127.0.0.1:8088"    # the session keeps its override

    # Applying the form saves what it shows, overrides included (as documented)
    win.settings_panel.apply_button.setEnabled(True)
    win.apply_settings(win.settings_panel.pending())
    assert json.loads(path.read_text())["vehicle_ip"] == "127.0.0.1:8088"


@pytest.mark.parametrize("size", [main_window_module.MIN_WINDOW_SIZE, (1280, 750), (1440, 790), (1440, 830)])
def test_status_dock_never_covers_the_bottom_docks(make_window, qtbot, size):
    """Down to the window's minimum size (which fits a 13-inch MacBook with the Dock shown), the
    status cards scroll rather than being drawn over the Bandwidth dock."""
    assert main_window_module.MIN_WINDOW_SIZE <= (1440, 830)
    win = make_window()
    win.resize(*size)
    win.engine.start()
    win.engine.statusUpdated.emit(make_status(win.settings_panel.base))
    qtbot.wait(50)
    status = win.docks["status"].geometry()
    bottom = win.docks["bandwidth"].geometry()
    assert win.size().height() <= size[1] + 1
    assert not status.intersects(bottom), (status, bottom)


def test_recording_with_no_capture_rates_says_nothing_is_written(make_window):
    win = make_window()                                           # default rates: 0 and 0
    win.record_action.setChecked(True)
    win.engine.statusUpdated.emit(make_status(win.settings_panel.base, recording=True))
    assert "both capture rates are 0" in win.capture.state.text()
    win2 = make_window(center_capture_fps=1.0)
    win2.engine.statusUpdated.emit(make_status(win2.settings_panel.base, recording=True))
    assert win2.capture.state.text() == "● Recording"


def test_jumps_card_counts_deltas_computed_not_sent(window):
    window.engine.statusUpdated.emit(make_status())
    assert "Deltas computed" in window.status_panel.cards["jumps"].detail.text()


class VehicleArmed(Exception):
    """Same name as trackshot.autopilot.params.VehicleArmed: the UI must still explain it."""


def test_real_autopilot_exceptions_are_explained():
    params = pytest.importorskip("trackshot.autopilot.params")
    from trackshot.ui.panels.autopilot_panel import describe_error
    assert "disarm" in describe_error(params.VehicleArmed("armed"))
    assert "Can't reach BlueOS" in describe_error(params.AutopilotUnreachable("timed out"))
    assert "no heartbeat" in describe_error(params.NoHeartbeat("404"))


def test_real_capabilities_backends(qtbot, tmp_path):
    """Without the fixture's stand-in: the form asks trackshot.capabilities (depthai) itself."""
    win = MainWindow(FakeEngine(Settings()), settings_path=tmp_path / "s.json")
    qtbot.addWidget(win)
    model = win.settings_panel.backend.model()
    assert model.item(win.settings_panel.backend.findData("rtabmap")).isEnabled()


def test_autopilot_errors_are_explained(window, qtbot):
    window.engine.params = FakeAutopilotParams(error=VehicleArmed("armed"))
    window.autopilot.check()
    qtbot.waitUntil(lambda: not window.autopilot.busy, timeout=3000)
    assert "disarm" in window.autopilot.message.text()


# --- Window lifecycle ----------------------------------------------------------------------------

def test_close_stops_engine_and_saves_geometry(make_window):
    win = make_window()
    win.close()
    assert win.engine.called("stop")
    assert main_window_module.window_settings().value("window/geometry") is not None


# --- Screenshot ----------------------------------------------------------------------------------

def test_screenshot_with_fake_data(make_window, qtbot):
    win = make_window(color_preview_enabled=True, center_capture_fps=1.0, stereo_capture_fps=2.0)
    win.resize(1600, 1000)
    engine = win.engine
    engine.start()
    for i in range(700):
        engine.poseUpdated.emit(lawnmower_pose(i))
    for _ in range(8):
        engine.statusUpdated.emit(make_status(win.settings_panel.base, recording=True))
    engine.previewUpdated.emit(make_preview("vio"))
    engine.previewUpdated.emit(make_preview("color", 640, 360, seed=1))
    for level, text in [("info", "Connected to OAK-D-W-PoE at 192.168.2.118 (BNO086 IMU)"),
                        ("info", "RTAB-Map VIO running at 10 fps, 320×200"),
                        ("warning", "Pose jump rejected: 3.4 m/s implied speed"),
                        ("info", "mavlink2rest connected: ws://192.168.2.2/mavlink2rest/ws/mavlink")]:
        engine.logMessage.emit(level, text)
    win.autopilot.check()
    qtbot.waitUntil(lambda: win.autopilot.table.rowCount() > 0, timeout=3000)
    win.trajectory.refresh()
    win.trajectory.fit_button.click()
    qtbot.wait(50)
    SCREENSHOT.parent.mkdir(parents=True, exist_ok=True)
    assert win.grab().save(str(SCREENSHOT))
    assert SCREENSHOT.stat().st_size > 10_000
    assert engine.state == STATE_RUNNING
    engine.stop()
    assert engine.state == STATE_STOPPED
