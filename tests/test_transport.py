"""Transports against local stand-ins: a `websockets` server playing mavlink2rest, and a UDP socket
decoding with pymavlink. Nothing here touches the (offline) vehicle."""

import asyncio
import json
import socket
import threading
import sys
import time

import pytest
from pymavlink.dialects.v20 import ardupilotmega
from websockets.asyncio.server import serve

from trackshot.config import Settings
from trackshot.core.odometry import DeltaMessage
from trackshot.mavlink import transport as transport_module
from trackshot.mavlink.messages import named_value_float, vision_position_delta
from trackshot.mavlink.transport import (
    Mavlink2RestTransport,
    UdpTransport,
    classify_m2r_reply,
    make_transport,
    with_send_only_filter,
)

DELTA = DeltaMessage(time_usec=1_700_000_000_000_000, time_delta_usec=100_000,
                     position_delta=(0.1, -0.2, 0.3), angle_delta=(0.0, 0.0, -0.1), confidence=100.0)


# pymavlink binds a udpout socket to 0.0.0.0:<destination port> on Windows, which collides with
# the test's vehicle socket on localhost (and, for real, with a GCS on the same PC using that port)
udp_loopback = pytest.mark.skipif(sys.platform == "win32", reason="pymavlink udpout binds the destination port on Windows")


def wait_until(condition, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


def reply_like_m2r_0_11(frame):
    """BlueOS's mavlink2rest 0.11 (rust-mavlink 0.10, `[char; N]`): char arrays must be lists."""
    if isinstance(frame["message"].get("name", []), str):
        return "Could not convert input message."
    return "Ok(())"


def reply_like_m2r_1_x(frame):
    """mavlink2rest ≥ 1.0 (rust-mavlink `CharArray`): char arrays must be strings."""
    if isinstance(frame["message"].get("name", ""), list):
        return json.dumps({"error": "Could not convert input message."})
    return json.dumps({"result": "ok"})


class FakeMavlink2Rest:
    """A websocket server in its own asyncio thread that records frames and answers like mavlink2rest."""

    def __init__(self, reply=reply_like_m2r_0_11):
        self.reply = reply
        self.frames: list[dict] = []
        self.paths: list[str] = []
        self.port = 0
        self._loop = None
        self._thread = None
        self._stopped = None

    def start(self):
        ready = threading.Event()

        async def handler(ws):
            self.paths.append(ws.request.path)
            async for text in ws:
                frame = json.loads(text)
                self.frames.append(frame)
                await ws.send(self.reply(frame))

        async def main():
            self._stopped = asyncio.Event()
            async with serve(handler, "127.0.0.1", self.port) as server:
                self.port = server.sockets[0].getsockname()[1]
                ready.set()
                await self._stopped.wait()

        def run():
            self._loop = asyncio.new_event_loop()
            self._loop.run_until_complete(main())
            self._loop.close()

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        assert ready.wait(3)
        return self

    def stop(self):
        self._loop.call_soon_threadsafe(self._stopped.set)
        self._thread.join(3)

    def url(self):
        return f"ws://127.0.0.1:{self.port}/mavlink2rest/ws/mavlink"


@pytest.fixture
def m2r():
    server = FakeMavlink2Rest().start()
    yield server
    server.stop()


@pytest.fixture
def stoppable():
    started = []

    def track(transport):
        started.append(transport)
        return transport

    yield track
    for t in started:
        t.stop()


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# --- mavlink2rest ---------------------------------------------------------------------------------

def test_send_only_filter():
    # mavlink2rest's ws filter defaults to ".*" (stream everything); SEND_ONLY matches no message
    assert with_send_only_filter("ws://h/mavlink2rest/ws/mavlink") == "ws://h/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
    assert with_send_only_filter("ws://h/ws/mavlink?filter=HEARTBEAT") == "ws://h/ws/mavlink?filter=HEARTBEAT"


def test_classify_replies():
    assert classify_m2r_reply("Ok(())") == ""
    assert classify_m2r_reply('{"result": "ok"}') == ""
    assert classify_m2r_reply("Could not convert input message.") == "Could not convert input message."
    assert classify_m2r_reply('{"error": "boom"}') == "boom"
    assert classify_m2r_reply('{"header": {}, "message": {"type": "HEARTBEAT"}}') is None


def test_frames_have_mavlink2rest_shape(m2r, stoppable):
    t = stoppable(Mavlink2RestTransport(m2r.url(), 1, 197))
    t.start()
    assert wait_until(lambda: t.stats().connected)
    assert t.send(*vision_position_delta(DELTA))
    assert t.send(*named_value_float("TS_FPS", 9.5, 1234))
    assert wait_until(lambda: len(m2r.frames) == 2)

    assert m2r.paths == ["/mavlink2rest/ws/mavlink?filter=SEND_ONLY"]
    vpd, nvf = m2r.frames
    assert vpd["header"] == {"system_id": 1, "component_id": 197, "sequence": 0}
    assert vpd["message"] == {
        "type": "VISION_POSITION_DELTA", "time_usec": 1_700_000_000_000_000, "time_delta_usec": 100_000,
        "angle_delta": [0.0, 0.0, -0.1], "position_delta": [0.1, -0.2, 0.3], "confidence": 100.0,
    }
    assert nvf["header"]["sequence"] == 1
    # char[10] as BlueOS's mavlink2rest 0.11 wants it: exactly 10 one-character strings, NUL padded
    assert nvf["message"] == {"type": "NAMED_VALUE_FLOAT", "time_boot_ms": 1234, "value": 9.5,
                              "name": ["T", "S", "_", "F", "P", "S", "\u0000", "\u0000", "\u0000", "\u0000"]}
    assert wait_until(lambda: t.stats().sent == 2)
    s = t.stats()
    assert (s.kind, s.errors, s.dropped, s.heartbeat_age_s) == ("mavlink2rest", 0, 0, None)


def test_switches_char_arrays_to_strings_for_newer_mavlink2rest(stoppable):
    server = FakeMavlink2Rest(reply=reply_like_m2r_1_x).start()
    try:
        t = stoppable(Mavlink2RestTransport(server.url(), 1, 197))
        t.start()
        assert wait_until(lambda: t.stats().connected)
        t.send(*named_value_float("TS_FPS", 1.0, 0))
        assert wait_until(lambda: t.stats().errors == 1)
        assert "Could not convert" in t.stats().last_error
        t.send(*named_value_float("TS_FPS", 2.0, 0))
        assert wait_until(lambda: len(server.frames) == 2)
        assert server.frames[0]["message"]["name"][0] == "T"
        assert server.frames[1]["message"]["name"] == "TS_FPS"
        time.sleep(0.1)
        assert t.stats().errors == 1   # the string form was accepted
    finally:
        server.stop()


def test_reconnects_after_server_restart(stoppable):
    server = FakeMavlink2Rest().start()
    t = stoppable(Mavlink2RestTransport(server.url(), 1, 197))
    t.start()
    assert wait_until(lambda: t.stats().connected)
    server.stop()
    assert wait_until(lambda: not t.stats().connected)

    restarted = FakeMavlink2Rest()
    restarted.port = server.port
    restarted.start()
    try:
        assert wait_until(lambda: t.stats().connected, timeout=5)   # first retry is after 1 s
        t.send(*vision_position_delta(DELTA))
        assert wait_until(lambda: len(restarted.frames) == 1)
    finally:
        restarted.stop()


def test_queue_overflow_drops_oldest(m2r, stoppable):
    t = stoppable(Mavlink2RestTransport(m2r.url(), 1, 197, queue_size=4))
    for i in range(10):   # not started: everything waits in the queue
        assert t.send(*named_value_float("N", float(i), i))
    assert t.stats().dropped == 6
    t.start()
    assert wait_until(lambda: len(m2r.frames) == 4)
    assert [f["message"]["value"] for f in m2r.frames] == [6.0, 7.0, 8.0, 9.0]


def test_stale_messages_are_dropped_while_disconnected(monkeypatch, stoppable):
    monkeypatch.setattr(transport_module, "MAX_QUEUE_AGE_S", 0.1)
    t = stoppable(Mavlink2RestTransport(f"ws://127.0.0.1:{free_port()}/ws", 1, 197))
    t.start()
    for _ in range(3):
        t.send(*vision_position_delta(DELTA))
    assert wait_until(lambda: t.stats().dropped == 3, timeout=2)
    # the refused connect is reported once it fails (about 2 s on Windows, which retries it)
    assert wait_until(lambda: t.stats().errors >= 1, timeout=5)
    s = t.stats()
    assert not s.connected and s.sent == 0 and s.errors >= 1 and s.last_error


def test_send_never_blocks_and_stop_is_quick_when_unreachable():
    t = Mavlink2RestTransport(f"ws://127.0.0.1:{free_port()}/ws", 1, 197)
    t.start()
    started = time.monotonic()
    for _ in range(1000):
        t.send(*vision_position_delta(DELTA))
    assert time.monotonic() - started < 0.5
    started = time.monotonic()
    t.stop()
    t.stop()   # twice is fine
    assert time.monotonic() - started < 2.5


def test_stop_before_start():
    Mavlink2RestTransport("ws://127.0.0.1:1/ws", 1, 197).stop()
    UdpTransport("udpout:127.0.0.1:1", 1, 197).stop()


# --- UDP ------------------------------------------------------------------------------------------

@pytest.fixture
def vehicle_socket():
    """Stands in for the autopilot's UDP endpoint."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(2.0)
    yield sock
    sock.close()


def receive(sock):
    data, addr = sock.recvfrom(65535)
    msgs = ardupilotmega.MAVLink(None).parse_buffer(data)
    assert len(msgs) == 1
    return msgs[0], addr


def test_udp_sends_mavlink2_from_1_197(vehicle_socket, stoppable):
    port = vehicle_socket.getsockname()[1]
    t = stoppable(UdpTransport(f"udpout:127.0.0.1:{port}", 1, 197))
    t.start()
    assert wait_until(lambda: t.stats().connected)
    t.send(*vision_position_delta(DELTA))
    t.send(*named_value_float("TS_TRACK", 1.0, 42))

    vpd, _ = receive(vehicle_socket)
    assert vpd.get_type() == "VISION_POSITION_DELTA"
    assert (vpd.get_srcSystem(), vpd.get_srcComponent()) == (1, 197)
    assert vpd.time_usec == DELTA.time_usec and vpd.time_delta_usec == 100_000
    assert vpd.position_delta == pytest.approx([0.1, -0.2, 0.3], rel=1e-6)
    assert vpd.angle_delta == pytest.approx([0.0, 0.0, -0.1], rel=1e-6)
    nvf, _ = receive(vehicle_socket)
    assert (nvf.name, nvf.value, nvf.time_boot_ms) == ("TS_TRACK", 1.0, 42)
    assert nvf.get_srcComponent() == 197
    assert wait_until(lambda: t.stats().sent == 2)
    assert t.stats().kind == "udp"


@udp_loopback
def test_udp_tracks_autopilot_heartbeat(vehicle_socket, stoppable):
    port = vehicle_socket.getsockname()[1]
    t = stoppable(UdpTransport(f"udpout:127.0.0.1:{port}", 1, 197))
    t.start()
    assert wait_until(lambda: t.stats().connected)
    t.send(*named_value_float("TS_FPS", 0.0, 0))
    _, transport_addr = receive(vehicle_socket)   # where to answer

    def heartbeat_from(sysid, compid):
        mav = ardupilotmega.MAVLink(None, srcSystem=sysid, srcComponent=compid)
        msg = mav.heartbeat_encode(ardupilotmega.MAV_TYPE_SUBMARINE, ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        vehicle_socket.sendto(msg.pack(mav), transport_addr)

    heartbeat_from(1, 197)   # another component's heartbeat isn't the autopilot's
    heartbeat_from(2, 1)     # nor another system's
    time.sleep(0.3)
    assert t.stats().heartbeat_age_s is None
    heartbeat_from(1, 1)
    assert wait_until(lambda: t.stats().heartbeat_age_s is not None)
    assert t.stats().heartbeat_age_s < 1.0


def test_make_transport():
    s = Settings(vehicle_ip="127.0.0.1:8088", source_system=1, source_component=197)
    t = make_transport(s)
    assert isinstance(t, Mavlink2RestTransport)
    assert t.target == "ws://127.0.0.1:8088/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
    assert (t.source_system, t.source_component) == (1, 197)
    s.transport = "udp"
    s.udp_url = "udpout:127.0.0.1:14550"
    t = make_transport(s)
    assert isinstance(t, UdpTransport) and t.target == "udpout:127.0.0.1:14550"


def test_enums_go_to_mavlink2rest_as_objects_and_type_as_mavtype():
    from trackshot.mavlink import messages
    from trackshot.mavlink.transport import encode_enums
    msg_type, fields = messages.distance_sensor(2.5, 100, 1000, 0.3, 10.0)
    out = encode_enums(msg_type, fields)
    assert out["orientation"] == {"type": "MAV_SENSOR_ROTATION_PITCH_270"}
    assert out["mavtype"] == {"type": "MAV_DISTANCE_SENSOR_UNKNOWN"} and "type" not in out
    assert out["current_distance"] == 250 and out["signal_quality"] == 100
    # messages without enums pass through untouched
    msg_type, fields = vision_position_delta(DELTA)
    assert encode_enums(msg_type, fields) == fields
