#!/usr/bin/env python3
"""Hardware-day check of the OAK over the tether, without MAVLink or the GUI.

Lists the cameras this computer can see, connects through OakPoseSource exactly as the app does,
reports the IMU, then prints pose rate, capture→host latency and measured rx Mbit/s once a
second. Stop the BlueOS OAK-D extension first: the camera accepts one connection.

    .venv/bin/python scripts/oak_probe.py --ip 192.168.2.118 --backend rtabmap --seconds 60
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

from trackshot import capabilities
from trackshot.config import BACKEND_DEFAULT_FPS, Settings
from trackshot.vio.oak_pipeline import OakPoseSource, has_parked_pipelines
from trackshot.vio.source import STATE_ERROR, STATE_RUNNING, SourceCallbacks


def parse_args() -> argparse.Namespace:
    backends = capabilities.available_backends()
    parser = argparse.ArgumentParser(description="OAK VIO probe (no MAVLink)")
    parser.add_argument("--ip", default=Settings().camera_ip, help="camera IP; empty = first camera found")
    parser.add_argument("--backend", choices=backends, default=backends[0] if backends else None)
    parser.add_argument("--seconds", type=float, default=30.0, help="how long to stream after connecting")
    parser.add_argument("--fps", type=int, help="VIO frame rate (default: the backend's default)")
    return parser.parse_args()


def list_cameras() -> None:
    cameras = capabilities.list_cameras()
    if not cameras:
        print("No cameras discovered (a camera on another subnet can still be reached by IP)")
    for cam in cameras:
        print(f"  {cam['name']:<16} id={cam['id']:<22} {cam['protocol']:<14} {cam['state']}"
              f"{'  <- IN USE (stop the BlueOS extension)' if cam['in_use'] else ''}")


def main() -> int:
    args = parse_args()
    if args.backend is None:
        print("This depthai build has no VIO backend", file=sys.stderr)
        return 2
    print(f"depthai {capabilities.depthai_version()} | backends: {', '.join(capabilities.available_backends())}")
    list_cameras()

    settings = Settings(camera_ip=args.ip, backend=args.backend,
                        fps=args.fps or BACKEND_DEFAULT_FPS[args.backend],
                        preview_enabled=False, color_preview_enabled=False)
    running = threading.Event()
    finished = threading.Event()
    outcome = {"error": None}

    def on_state(state, message):
        print(f"[{state}] {message or ''}")
        if state == STATE_RUNNING:
            running.set()
        elif state == STATE_ERROR:
            outcome["error"] = message
            finished.set()

    source = OakPoseSource(settings, SourceCallbacks(on_state=on_state))
    source.start()
    try:
        # Connecting may take up to the 30 s retry window, plus depthai's own device search
        while not running.is_set() and not finished.is_set():
            time.sleep(0.1)
        if outcome["error"]:
            return 1
        stats = source.stats()
        print(f"Camera: {stats.camera_name} ({stats.camera_id}) | IMU: {stats.imu} | backend: {stats.backend}")
        print(f"{'t (s)':>6} {'poses/s':>8} {'latency ms':>11} {'rx Mbit/s':>10} {'chip °C':>8} {'poses':>6}")
        start = time.monotonic()
        while time.monotonic() - start < args.seconds and not finished.wait(1.0):
            stats = source.stats()
            latency = stats.extra.get("latency_ms")
            print(f"{time.monotonic() - start:6.0f} {stats.fps:8.1f} "
                  f"{latency if latency is not None else '-':>11} "
                  f"{stats.rx_mbit if stats.rx_mbit is not None else 0.0:10.1f} "
                  f"{stats.chip_temp_c if stats.chip_temp_c is not None else '-':>8} "
                  f"{stats.extra.get('poses', 0):6d}")
        return 1 if outcome["error"] else 0
    except KeyboardInterrupt:
        return 130
    finally:
        source.stop(timeout=10.0)


if __name__ == "__main__":
    code = main()
    if has_parked_pipelines():
        # A Basalt pipeline stopped before its first frame can't be destroyed without crashing
        # (see oak_pipeline): leave without running the interpreter's teardown
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
    sys.exit(code)
