"""Shared fixtures for every test module."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def isolated_run_logs(tmp_path, monkeypatch):
    """Run logs go to a temporary directory, never the user's real log folder."""
    from trackshot import runlog
    directory = tmp_path / "runs"
    monkeypatch.setattr(runlog, "runs_dir", lambda: directory)
    return directory


@pytest.fixture(autouse=True)
def isolated_param_backups(tmp_path, monkeypatch):
    """Parameter backups go to a temporary directory, never the user's real one."""
    from trackshot.autopilot import backup
    directory = tmp_path / "param_backups"
    monkeypatch.setattr(backup, "backups_dir", lambda: directory)
    return directory


class _NoRealCamera:
    """Stands in for depthai.Device in every test: nothing may reach a real camera.

    A test once connected to 192.168.2.118 believing no camera would answer, and on the vehicle's
    network it took the camera away from the running app. Discovery finds nothing and connecting
    fails like an absent camera. Tests that need a device mock it themselves; real-hardware tests
    run only with TRACKSHOT_HW_TESTS=1.
    """

    def __init__(self, *args, **kwargs):
        raise RuntimeError("Cannot find any device with given deviceInfo "
                           "(tests never connect to a real camera; TRACKSHOT_HW_TESTS=1 allows it)")

    @staticmethod
    def getAllConnectedDevices():
        return []

    @staticmethod
    def getAllAvailableDevices():
        return []


@pytest.fixture(autouse=True)
def no_real_camera(monkeypatch):
    import os
    if os.environ.get("TRACKSHOT_HW_TESTS") == "1":
        return
    import depthai
    monkeypatch.setattr(depthai, "Device", _NoRealCamera)


VEHICLE_NETWORK_PREFIX = "192.168.2."   # BlueOS's tether network: the real vehicle and camera


@pytest.fixture(autouse=True)
def no_vehicle_network(monkeypatch):
    """No test may open a connection or send a datagram to the vehicle's network: the developer's
    computer is often on the tether with a real ROV in the water."""
    import socket

    def refuse(address):
        host = address[0] if isinstance(address, tuple) else str(address)
        if isinstance(host, str) and host.startswith(VEHICLE_NETWORK_PREFIX):
            raise ConnectionRefusedError(f"tests never contact the vehicle network ({host})")

    real_connect, real_connect_ex, real_sendto = socket.socket.connect, socket.socket.connect_ex, socket.socket.sendto

    def connect(self, address):
        refuse(address)
        return real_connect(self, address)

    def connect_ex(self, address):
        refuse(address)
        return real_connect_ex(self, address)

    def sendto(self, data, *args):
        refuse(args[-1])
        return real_sendto(self, data, *args)

    monkeypatch.setattr(socket.socket, "connect", connect)
    monkeypatch.setattr(socket.socket, "connect_ex", connect_ex)
    monkeypatch.setattr(socket.socket, "sendto", sendto)
