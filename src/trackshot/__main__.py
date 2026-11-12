"""TrackShot entry point: `python -m trackshot`, the `trackshot` script, and the PyInstaller app.

    python -m trackshot                     the app, with the saved settings
    python -m trackshot --sim               the simulator instead of a camera, started at once
    python -m trackshot --vehicle-ip 127.0.0.1:8088   e.g. the fake autopilot (tools/fake_autopilot.py)
    python -m trackshot --reset-settings    back to the defaults (and save them)

`--self-test` (hidden) checks a build without opening a window: it prints the platform summary,
builds each available VIO node in a device-less depthai pipeline and runs the simulator through
the odometry and MAVLink encoder. The packaging script runs it against the bundled binary.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import signal
import sys
import threading
from pathlib import Path

# depthai 3 reports usage events to Luxonis and checks for notifications over the network. A
# topside app on a dive-site laptop shouldn't phone home unasked, nor wait on a network that
# isn't there. Set before depthai is first imported; an explicit setting in the environment wins.
for _name, _value in (("DEPTHAI_TELEMETRY", "0"), ("DEPTHAI_DISABLE_FEEDBACK", "1"),
                      ("DEPTHAI_DISABLE_STARTUP_NOTIFICATIONS", "1")):
    os.environ.setdefault(_name, _value)

# urllib/http.client encode host names with the "idna" codec the first time they connect, which
# here is in a background worker (autopilot checks, telemetry). Load the codec now, on the main
# thread, so the first request never depends on a lazy import from the app bundle.
import encodings.idna  # noqa: E402,F401

from loguru import logger  # noqa: E402

APP_NAME = "TrackShot"
# At exit, how long to wait for the UI's background tasks (autopilot HTTP, camera discovery). A
# cancelled check stops within one HTTP timeout (3 s); anything longer is abandoned with os._exit.
# Autopilot parameter writes are never abandoned: the window doesn't close while one runs, and if
# the event loop ends anyway, exit_wait_ms() waits for it without a limit.
POOL_EXIT_WAIT_MS = 4000


def exit_wait_ms(window) -> int:
    """How long main() waits for the window's background tasks: without a limit (-1) while
    autopilot parameters are being written, so they're never left half applied or reverted."""
    autopilot = getattr(window, "autopilot", None)
    return -1 if getattr(autopilot, "writing", False) else POOL_EXIT_WAIT_MS
# The VIO node behind each backend name (as in Settings.backend)
VIO_NODES = {"rtabmap": "RTABMapVIO", "basalt": "BasaltVIO"}


def resource_path(name: str) -> Path:
    """Files under trackshot/resources, from a source checkout or inside the PyInstaller bundle
    (where the spec copies them next to the frozen package)."""
    import trackshot
    return Path(trackshot.__file__).resolve().parent / "resources" / name


def app_version() -> str:
    from importlib.metadata import PackageNotFoundError, version
    try:
        return version("trackshot")
    except PackageNotFoundError:
        return "dev"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="trackshot", description="Topside OAK-D VIO for ArduSub.")
    parser.add_argument("--sim", action="store_true",
                        help="use the simulator instead of a camera, and start it at once")
    parser.add_argument("--sim-pattern", choices=("survey", "hover"),
                        help="simulator motion for this session: a lawnmower survey or a stationary hover")
    parser.add_argument("--start", action="store_true", help="start VIO as soon as the window opens")
    parser.add_argument("--vehicle-ip", metavar="HOST[:PORT]",
                        help="vehicle (BlueOS) address for this session, e.g. 127.0.0.1:8088")
    parser.add_argument("--reset-settings", action="store_true", help="discard the saved settings")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    # Finder used to pass -psn_0_NNN to apps; Qt also takes its own options. Ignore what isn't ours.
    args, _unknown = parser.parse_known_args(argv)
    return args


def configure_logging() -> Path | None:
    """stderr (a terminal launch) plus a rotating file, since a Finder launch has no terminal."""
    from platformdirs import user_log_dir
    log_dir = Path(user_log_dir(APP_NAME, appauthor=False))
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / "trackshot.log"
        logger.add(path, level="INFO", rotation="5 MB", retention=3, enqueue=False)
        return path
    except OSError as e:
        logger.warning(f"No log file: {e}")
        return None


def self_test() -> int:
    """Exercises everything a broken bundle would lose (native libraries, data files, dynamically
    imported modules), without a window, a camera or a vehicle. Exit 0 when all is well.

    With TRACKSHOT_SELFTEST_LOG set, the report is also written to that file: a windowed Windows
    exe has no stdout to print to."""
    log_file = os.environ.get("TRACKSHOT_SELFTEST_LOG")
    if log_file:
        sys.stdout = _Tee(sys.stdout, open(log_file, "w", encoding="utf-8"))
    try:
        return _self_test()
    finally:
        if isinstance(sys.stdout, _Tee):
            sys.stdout.close_file()
            sys.stdout = sys.stdout.console


class _Tee:
    """stdout that also writes to a file; the console side may be None (windowed Windows exe)."""

    def __init__(self, console, file):
        self.console, self.file = console, file

    def write(self, text: str) -> int:
        if self.console is not None:
            self.console.write(text)
        self.file.write(text)
        return len(text)

    def flush(self) -> None:
        if self.console is not None:
            self.console.flush()
        self.file.flush()

    def close_file(self) -> None:
        self.file.close()


def _self_test() -> int:
    ok = True
    logger.remove()   # just the results on stdout; warnings and errors still reach stderr
    if sys.stderr is not None:   # a windowed Windows exe has no stderr (loguru rejects None)
        logger.add(sys.stderr, level="WARNING")

    def check(label: str, fn) -> None:
        nonlocal ok
        try:
            detail = fn()
            print(f"ok    {label}" + (f": {detail}" if detail else ""))
        except Exception as e:  # noqa: BLE001 - report every failure, not just the first
            ok = False
            print(f"FAIL  {label}: {type(e).__name__}: {e}")

    from trackshot import capabilities
    info = capabilities.describe()
    print(json.dumps({"trackshot": app_version(), **info}))
    if not info["backends"]:
        ok = False
        print("FAIL  no VIO backend available in this depthai build")

    def build_node(backend: str):
        import depthai as dai
        pipeline = dai.Pipeline(False)   # host-only: no device needed to build the VIO nodes
        node = pipeline.create(getattr(dai.node, VIO_NODES[backend]))
        node.transform.createOutputQueue(maxSize=2, blocking=False)
        return type(node).__name__

    for backend in info["backends"]:
        check(f"{backend} node", lambda b=backend: build_node(b))

    def sim_to_mavlink():
        from pymavlink.dialects.v20 import ardupilotmega
        from trackshot.config import Settings
        from trackshot.core.odometry import OdometryProcessor
        from trackshot.mavlink import messages
        from trackshot.vio.sim import SimPoseSource
        from trackshot.vio.source import STATE_ERROR, STATE_STOPPED, SourceCallbacks

        processor = OdometryProcessor()
        mav = ardupilotmega.MAVLink(None, srcSystem=1, srcComponent=197)
        packets = []

        def on_pose(sample):
            delta = processor.process(sample)
            if delta is not None:
                packets.append(messages.encode(mav, *messages.vision_position_delta(delta)).pack(mav))

        finished = threading.Event()

        def on_state(state, _message):
            if state in (STATE_STOPPED, STATE_ERROR):
                finished.set()

        callbacks = SourceCallbacks(on_pose=on_pose, on_state=on_state)
        source = SimPoseSource(Settings(source="sim", center_capture_fps=1.0), callbacks,
                               realtime=False, duration_s=2.0)
        source.start()
        finished.wait(30.0)
        source.stop()
        if len(packets) < 10:
            raise RuntimeError(f"only {len(packets)} VISION_POSITION_DELTA packets")
        return f"{len(packets)} VISION_POSITION_DELTA packets"

    check("simulator → odometry → MAVLink", sim_to_mavlink)

    def param_backup():
        import tempfile
        from pathlib import Path
        from trackshot.autopilot.backup import ParamBackup
        from trackshot.ui.panels.autopilot_panel import revert_summary  # noqa: F401
        with tempfile.TemporaryDirectory() as tmp:
            backup = ParamBackup("self-test", Path(tmp))
            backup.record_original("VISO_TYPE", 0.0, "MAV_PARAM_TYPE_INT8", True)
            backup.mark_written("VISO_TYPE", 1.0)
            backup.record_original("VISO_TYPE", 1.0, "MAV_PARAM_TYPE_INT8", True)
            entry = ParamBackup("self-test", Path(tmp)).get("VISO_TYPE")
            if entry is None or entry["original"] != 0.0 or entry["written"] != 1.0:
                raise RuntimeError(f"unexpected backup entry {entry}")
        return "record, reload, first change kept"

    check("parameter backup", param_backup)

    def http_from_worker():
        # What the Autopilot panel does: urllib to the vehicle by name, from a worker thread. A
        # bundle missing the idna codec (or its stringprep/unicodedata deps) fails right here.
        import http.server
        import urllib.request
        if "bücher.example".encode("idna") != b"xn--bcher-kva.example":
            raise RuntimeError("idna codec gives the wrong result")

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"None")

            def log_message(self, *args):
                pass

        server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        result = {}

        def fetch():
            try:
                url = f"http://localhost:{server.server_address[1]}/mavlink2rest/mavlink/vehicles"
                with urllib.request.urlopen(url, timeout=5) as response:
                    result["body"] = response.read()
            except Exception as e:  # noqa: BLE001 - surfaced below
                result["error"] = e

        worker = threading.Thread(target=fetch)
        worker.start()
        worker.join(10)
        server.shutdown()
        if "error" in result:
            raise result["error"]
        if result.get("body") != b"None":
            raise RuntimeError(f"unexpected reply {result!r}")
        return "idna codec, urllib to localhost from a worker thread"

    check("network stack", http_from_worker)

    def gui_modules():
        import pyqtgraph  # noqa: F401
        from PySide6 import QtWidgets  # noqa: F401
        import trackshot.ui.main_window  # noqa: F401
        return f"pyqtgraph {pyqtgraph.__version__}"

    check("GUI modules", gui_modules)
    check("app icon", lambda: f"{resource_path('icon.png').stat().st_size} bytes")
    print("self-test passed" if ok else "self-test FAILED")
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.self_test:
        return self_test()

    log_path = configure_logging()

    from PySide6.QtCore import QTimer
    from PySide6.QtGui import QIcon
    from PySide6.QtWidgets import QApplication

    from trackshot import capabilities
    from trackshot.config import Settings, config_path
    from trackshot.engine import Engine
    from trackshot.ui.main_window import MainWindow

    app = QApplication(sys.argv[:1])
    app.setApplicationName(APP_NAME)
    app.setOrganizationName(APP_NAME)
    app.setApplicationDisplayName(APP_NAME)
    app.setApplicationVersion(app_version())
    icon = resource_path("icon.png")
    if icon.exists():
        app.setWindowIcon(QIcon(str(icon)))

    path = config_path()
    if args.reset_settings:
        settings = Settings()
        settings.save(path)
        logger.info(f"Settings reset to the defaults ({path})")
    else:
        settings = Settings.load(path)
    saved = copy.deepcopy(settings)
    # Session overrides: not saved unless the user applies settings in the window
    if args.sim:
        settings.source = "sim"
    if args.vehicle_ip:
        settings.vehicle_ip = args.vehicle_ip
    if args.sim_pattern:
        settings.sim_pattern = args.sim_pattern

    logger.info(f"TrackShot {app_version()} on {capabilities.describe()}; settings {path}"
                + (f"; log {log_path}" if log_path else ""))
    engine = Engine(settings)
    window = MainWindow(engine, settings_path=path, saved_settings=saved)
    window.show()
    if args.sim or args.start:
        QTimer.singleShot(0, engine.start)

    # Ctrl-C in the terminal closes the window like ⌘Q. Qt's event loop doesn't return to Python
    # on its own, so a timer gives the interpreter a moment to run the signal handler.
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    tick = QTimer()
    tick.start(250)
    tick.timeout.connect(lambda: None)

    code = app.exec()
    engine.shutdown()
    # The window's QThreadPool waits for running tasks when it is destroyed (on return from
    # here), which would keep a closed app alive in the Dock: wait a bounded time instead
    wait_ms = exit_wait_ms(window)
    if wait_ms < 0:
        logger.warning("Waiting for the autopilot parameter writes to finish before exiting")
    pool_done = window.pool.waitForDone(wait_ms)
    logger.info("TrackShot closed")
    if not pool_done or _camera_pipeline_parked():
        # A task that won't finish, or a camera pipeline that can't be destroyed without crashing
        # (see trackshot.vio.oak_pipeline): leave without the interpreter's teardown
        if not pool_done:
            logger.warning("Leaving with a background task still running")
        logger.complete()
        for stream in (sys.stdout, sys.stderr):
            if stream is not None:   # None in a windowed Windows exe
                stream.flush()
        os._exit(code)
    return code


def _camera_pipeline_parked() -> bool:
    # Only if the camera module was ever loaded: importing it here would load depthai for nothing
    oak = sys.modules.get("trackshot.vio.oak_pipeline")
    return oak is not None and oak.has_parked_pipelines()


if __name__ == "__main__":
    sys.exit(main())
