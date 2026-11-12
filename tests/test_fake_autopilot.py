"""The fake BlueOS itself: mavlink2rest HTTP/WS shapes, the autopilot emulation, and UDP."""

import json
import socket
import time
import urllib.error
import urllib.request
from datetime import datetime

import pytest
import websocket
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from trackshot.testing.fake_autopilot import FakeAutopilot


def get(fake, name, system_id=1, component_id=1):
    url = f"{fake.http_base}/mavlink/vehicles/{system_id}/components/{component_id}/messages/{name}"
    with urllib.request.urlopen(url, timeout=2) as response:
        return response.status, response.read().decode()


def post(fake, frame):
    request = urllib.request.Request(f"{fake.http_base}/mavlink", data=json.dumps(frame).encode(),
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=2) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.01)
    return predicate()


NAMED_VALUE = {"type": "NAMED_VALUE_FLOAT", "time_boot_ms": 1234, "value": 9.5}


def test_start_stop_and_addresses():
    started = time.monotonic()
    fake = FakeAutopilot().start()
    try:
        host, port = fake.vehicle_ip.split(":")
        assert host == "127.0.0.1" and int(port) == fake.port > 0
        assert fake.http_base == f"http://127.0.0.1:{fake.port}/mavlink2rest"
        assert fake.ws_url == f"ws://127.0.0.1:{fake.port}/mavlink2rest/ws/mavlink"
        assert fake.udp_url == f"udpout:127.0.0.1:{fake.udp_port}"
    finally:
        fake.stop()
        fake.stop()     # safe twice
    assert time.monotonic() - started < 1.0
    with pytest.raises(OSError):
        get(fake, "HEARTBEAT")


@pytest.mark.parametrize("flavor", ["0.11", "1.1"])
def test_message_shape(flavor):
    with FakeAutopilot(flavor=flavor) as fake:
        status, body = get(fake, "HEARTBEAT")
        reply = json.loads(body)
        _, again = get(fake, "HEARTBEAT")
    assert status == 200
    assert set(reply) == {"message", "status"}
    message, times = reply["message"], reply["status"]["time"]
    assert message["type"] == "HEARTBEAT" and message["custom_mode"] == 19
    assert message["mavtype"] == {"type": "MAV_TYPE_SUBMARINE"}
    expected_keys = {"first_update", "last_update", "counter", "frequency"}
    if flavor == "1.1":
        expected_keys.add("last_update_seconds_ago")
        assert isinstance(message["base_mode"], str)
    else:
        assert message["base_mode"] == {"bits": 0x51}
    assert set(times) == expected_keys
    assert times["counter"] >= 2
    # chrono's RFC 3339 with nanoseconds and an offset
    assert len(times["last_update"].split(".")[1]) == 9 + 6
    assert abs(datetime.fromisoformat(times["last_update"]).timestamp() - time.time()) < 5


def test_unknown_message_is_none_with_200():
    with FakeAutopilot() as fake:
        assert get(fake, "BATTERY_STATUS") == (200, "None")
        assert get(fake, "HEARTBEAT", system_id=7) == (200, "None")


def test_counter_and_set_armed():
    with FakeAutopilot() as fake:
        first = json.loads(get(fake, "HEARTBEAT")[1])
        fake.set_armed(True)
        second = json.loads(get(fake, "HEARTBEAT")[1])
    assert second["status"]["time"]["counter"] > first["status"]["time"]["counter"]
    assert second["message"]["base_mode"]["bits"] & 0x80


@pytest.mark.parametrize("flavor, good, bad", [
    ("0.11", list("TS_FPS".ljust(10, "\u0000")), "TS_FPS"),
    ("1.1", "TS_FPS", list("TS_FPS".ljust(10, "\u0000"))),
])
def test_post_char_arrays(flavor, good, bad):
    with FakeAutopilot(flavor=flavor) as fake:
        header = {"system_id": 1, "component_id": 197, "sequence": 0}
        assert post(fake, {"header": header, "message": {**NAMED_VALUE, "name": good}}) == (200, "")
        status, error = post(fake, {"header": header, "message": {**NAMED_VALUE, "name": bad}})
        assert status == 404 and "not a valid MAVLinkMessage" in error
        assert post(fake, {"header": header, "message": {"type": "NOT_A_MESSAGE"}})[0] == 404
        assert post(fake, {"message": NAMED_VALUE})[0] == 404
        # an enum given as a number is rejected, as rust-mavlink would
        assert post(fake, {"header": header, "message": {
            "type": "PARAM_SET", "param_id": good, "param_value": 1.0, "param_type": 9,
            "target_system": 1, "target_component": 1}})[0] == 404

        [received] = fake.received("NAMED_VALUE_FLOAT")
        assert received["name"] == "TS_FPS" and received["value"] == 9.5
        assert received["_via"] == "http" and received["_header"] == {"system_id": 1, "component_id": 197}
        assert received["_raw"]["message"]["name"] == good
        # mavlink2rest files a sent message under its sender's ids
        assert json.loads(get(fake, "NAMED_VALUE_FLOAT", 1, 197)[1])["message"]["value"] == 9.5


def test_params_emulation_over_http():
    with FakeAutopilot() as fake:
        header = {"system_id": 255, "component_id": 190, "sequence": 0}
        post(fake, {"header": header, "message": {
            "type": "PARAM_SET", "param_id": list("EK3_SRC1_POSXY".ljust(16, "\u0000")),
            "param_value": 6.4, "param_type": {"type": "MAV_PARAM_TYPE_INT8"},
            "target_system": 1, "target_component": 1}})
        reply = json.loads(get(fake, "PARAM_VALUE")[1])["message"]
    assert fake.params["EK3_SRC1_POSXY"] == 6.0          # integer parameter
    assert "".join(reply["param_id"]).rstrip("\u0000") == "EK3_SRC1_POSXY"
    assert reply["param_value"] == 6.0 and reply["param_type"] == {"type": "MAV_PARAM_TYPE_INT8"}
    assert reply["param_count"] == len(fake.params)


@pytest.mark.parametrize("flavor, ok_reply", [("0.11", "Ok("), ("1.1", '{"result": "ok"}')])
def test_websocket_send_only(flavor, ok_reply):
    with FakeAutopilot(flavor=flavor) as fake:
        ws = websocket.create_connection(f"{fake.ws_url}?filter=SEND_ONLY", timeout=2)
        try:
            name = "TS_LAT_MS" if flavor == "1.1" else list("TS_LAT_MS".ljust(10, "\u0000"))
            frame = {"header": {"system_id": 1, "component_id": 197, "sequence": 0},
                     "message": {**NAMED_VALUE, "name": name}}
            ws.send(json.dumps(frame))
            assert ws.recv().startswith(ok_reply)       # mavlink2rest answers every frame
            ws.send("not json")
            assert "Could not convert input message." in ws.recv()
        finally:
            ws.close()
        [received] = fake.received("NAMED_VALUE_FLOAT")
    assert received["name"] == "TS_LAT_MS" and received["_via"] == "ws"
    assert received["_raw"] == frame


def test_websocket_streams_matching_messages():
    with FakeAutopilot() as fake:
        ws = websocket.create_connection(f"{fake.ws_url}?filter=^HEARTBEAT$", timeout=3)
        try:
            frame = json.loads(ws.recv())
        finally:
            ws.close()
    assert frame["message"]["type"] == "HEARTBEAT"
    assert frame["header"] == {"system_id": 1, "component_id": 1, "sequence": 0}


def test_stop_closes_websockets():
    fake = FakeAutopilot().start()
    ws = websocket.create_connection(f"{fake.ws_url}?filter=SEND_ONLY", timeout=2)
    started = time.monotonic()
    fake.stop()
    assert time.monotonic() - started < 1.0
    with pytest.raises(Exception):
        ws.send(json.dumps({"header": {"system_id": 1, "component_id": 197},
                            "message": NAMED_VALUE}))
        ws.recv()
    ws.close()


def test_udp_records_messages():
    sender = mavlink2.MAVLink(None, srcSystem=1, srcComponent=197)
    with FakeAutopilot() as fake, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        delta = sender.vision_position_delta_encode(123456789, 33000, [0.0, 0.0, 0.01], [0.1, 0.2, 0.3], 100.0)
        named = sender.named_value_float_encode(5, b"TS_FPS", 9.5)
        sock.sendto(delta.pack(sender) + named.pack(sender), ("127.0.0.1", fake.udp_port))
        assert wait_for(lambda: len(fake.received()) >= 2)
        [delta] = fake.received("VISION_POSITION_DELTA")
        [named] = fake.received("NAMED_VALUE_FLOAT")
    assert delta["time_usec"] == 123456789 and delta["time_delta_usec"] == 33000
    assert delta["position_delta"] == pytest.approx([0.1, 0.2, 0.3])
    assert delta["_via"] == "udp" and delta["_header"] == {"system_id": 1, "component_id": 197}
    assert isinstance(delta["_raw"], bytes)
    assert named["name"] == "TS_FPS"


def test_udp_heartbeats_reach_a_pymavlink_client():
    with FakeAutopilot() as fake:
        conn = mavutil.mavlink_connection(fake.udp_url, source_system=1, source_component=197)
        try:
            # the fake learns its peer from the first packet, as ArduPilot's udpin would
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            heartbeat = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            fake.set_armed(True)
            armed = None
            deadline = time.monotonic() + 3.0
            while armed is None and time.monotonic() < deadline:
                message = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if message is not None and message.base_mode & 0x80:
                    armed = message
        finally:
            conn.close()
    assert heartbeat is not None and heartbeat.type == mavutil.mavlink.MAV_TYPE_SUBMARINE
    assert heartbeat.get_srcSystem() == 1 and heartbeat.get_srcComponent() == 1
    assert heartbeat.custom_mode == 19 and not heartbeat.base_mode & 0x80
    assert armed is not None


def test_clear_received_returns_messages():
    with FakeAutopilot() as fake:
        post(fake, {"header": {"system_id": 1, "component_id": 197}, "message": {**NAMED_VALUE, "name": list("A")}})
        taken = fake.clear_received()
        assert [m["type"] for m in taken] == ["NAMED_VALUE_FLOAT"]
        assert fake.received() == []
