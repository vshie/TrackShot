"""What this computer's depthai build can do, and which OAK cameras it can see.

depthai's wheels differ by platform: the Windows wheel ships RTAB-Map but **no BasaltVIO**, so the
UI and the pose source ask here instead of assuming a backend exists. Everything degrades to
"nothing available" rather than raising, so a broken depthai install shows up as a clear message
in the app instead of a crash at import time.
"""

from __future__ import annotations

import functools
import platform

from loguru import logger

# Backend name (as in Settings.backend) -> depthai host node class that implements it.
# Order is the UI's order of preference: RTAB-Map first, as it is on every platform.
_BACKEND_NODES = {"rtabmap": "RTABMapVIO", "basalt": "BasaltVIO"}

# XLink states in which another process (usually the BlueOS extension) owns the camera.
# RVC2 PoE devices report X_LINK_BOOTED while a host is connected; RVC4 reports the GATE_ variant.
BUSY_STATES = ("X_LINK_BOOTED", "X_LINK_GATE_BOOTED")


def _import_depthai():
    try:
        import depthai
        return depthai
    except Exception as e:  # ImportError, or a broken native library
        logger.warning(f"depthai is not usable: {e}")
        return None


@functools.cache
def _backends() -> tuple[str, ...]:
    dai = _import_depthai()
    if dai is None:
        return ()
    return tuple(name for name, node in _BACKEND_NODES.items() if hasattr(dai.node, node))


def available_backends() -> list[str]:
    """VIO backends this depthai build provides, e.g. ['rtabmap', 'basalt'] (cached)."""
    return list(_backends())


def depthai_version() -> str | None:
    dai = _import_depthai()
    return getattr(dai, "__version__", None) if dai else None


def describe() -> dict:
    """Platform summary for the log and the About box."""
    return {
        "platform": platform.system(),
        "arch": platform.machine(),
        "python": platform.python_version(),
        "depthai": depthai_version(),
        "backends": available_backends(),
    }


def _enum_name(value) -> str:
    return getattr(value, "name", None) or str(value).rsplit(".", 1)[-1]


def list_cameras() -> list[dict]:
    """Every OAK the host can discover, including ones another host is using.

    Each entry: name (IP for PoE, USB path otherwise), id, ip (None over USB), protocol,
    state (XLink state name) and in_use. Never raises: a discovery failure returns [].
    Discovery takes about half a second, so call it off the GUI thread.
    """
    dai = _import_depthai()
    if dai is None:
        return []
    try:
        # getAllConnectedDevices (unlike getAllAvailableDevices) also lists booted devices,
        # which is how a camera held by the BlueOS extension shows up
        infos = dai.Device.getAllConnectedDevices()
    except Exception as e:
        logger.warning(f"Camera discovery failed: {e}")
        return []
    cameras = []
    for info in infos:
        try:
            protocol = _enum_name(info.protocol)
            state = _enum_name(info.state)
            cameras.append({
                "name": info.name,
                "id": info.getDeviceId(),
                "ip": info.name if protocol == "X_LINK_TCP_IP" else None,
                "protocol": protocol,
                "state": state,
                "in_use": state in BUSY_STATES,
            })
        except Exception as e:
            logger.warning(f"Skipping a camera that could not be described: {e}")
    return cameras

