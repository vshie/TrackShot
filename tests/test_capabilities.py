"""capabilities: backends this depthai build has, and camera discovery that never raises."""

import platform

import depthai as dai
import pytest

from trackshot import capabilities

on_mac_arm = platform.system() == "Darwin" and platform.machine() == "arm64"


@pytest.mark.skipif(not on_mac_arm, reason="the macOS arm64 wheel is the one known to ship both backends")
def test_mac_has_both_backends():
    assert capabilities.available_backends() == ["rtabmap", "basalt"]


def test_available_backends_is_cached_and_safe_to_mutate():
    first = capabilities.available_backends()
    first.append("bogus")
    assert "bogus" not in capabilities.available_backends()
    assert capabilities._backends.cache_info().hits >= 1


def test_describe():
    info = capabilities.describe()
    assert set(info) == {"platform", "arch", "python", "depthai", "backends"}
    assert info["depthai"] == dai.__version__
    assert info["backends"] == capabilities.available_backends()


def test_list_cameras_returns_a_list_without_hardware():
    cameras = capabilities.list_cameras()
    assert isinstance(cameras, list)


def test_list_cameras_never_raises(monkeypatch):
    def boom():
        raise RuntimeError("XLink exploded")
    monkeypatch.setattr(dai.Device, "getAllConnectedDevices", staticmethod(boom))
    assert capabilities.list_cameras() == []


def _info(name, state, protocol=dai.XLinkProtocol.X_LINK_TCP_IP, device_id="18443010B1"):
    return dai.DeviceInfo(name, device_id, state, protocol, dai.XLinkPlatform.X_LINK_MYRIAD_X,
                          dai.XLinkError_t.X_LINK_SUCCESS)


def test_list_cameras_describes_free_and_busy_cameras(monkeypatch):
    infos = [
        _info("192.168.2.118", dai.XLinkDeviceState.X_LINK_BOOTED),
        _info("192.168.2.119", dai.XLinkDeviceState.X_LINK_BOOTLOADER, device_id="18443010B2"),
        _info("1.1.2", dai.XLinkDeviceState.X_LINK_UNBOOTED, protocol=dai.XLinkProtocol.X_LINK_USB_VSC,
              device_id="18443010B3"),
    ]
    monkeypatch.setattr(dai.Device, "getAllConnectedDevices", staticmethod(lambda: infos))
    cameras = capabilities.list_cameras()
    assert cameras == [
        {"name": "192.168.2.118", "id": "18443010B1", "ip": "192.168.2.118", "protocol": "X_LINK_TCP_IP",
         "state": "X_LINK_BOOTED", "in_use": True},
        {"name": "192.168.2.119", "id": "18443010B2", "ip": "192.168.2.119", "protocol": "X_LINK_TCP_IP",
         "state": "X_LINK_BOOTLOADER", "in_use": False},
        {"name": "1.1.2", "id": "18443010B3", "ip": None, "protocol": "X_LINK_USB_VSC",
         "state": "X_LINK_UNBOOTED", "in_use": False},
    ]
