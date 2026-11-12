"""OakPoseSource without hardware: pure helpers, the real depthai config objects, the connect/stop
lifecycle against no camera, and the output-queue handling fed with real depthai messages.
Tests that need a camera run only on request: TRACKSHOT_HW_TESTS=1 pytest. They take the camera
over, so a routine run (and every app build) must never grab a camera a pilot is using."""

import os

import io
import subprocess
import sys
import textwrap
import threading
import time
from datetime import timedelta
from types import SimpleNamespace
from unittest import mock

import depthai as dai
import numpy as np
import pytest
from PIL import Image

from trackshot import capabilities
from trackshot.config import Settings
from trackshot.vio import oak_pipeline as op
from trackshot.vio.source import (
    STATE_CONNECTING, STATE_ERROR, STATE_RUNNING, STATE_STOPPED, SourceCallbacks,
)

HW_TESTS = os.environ.get("TRACKSHOT_HW_TESTS") == "1"
has_camera = HW_TESTS and bool(capabilities.list_cameras())
needs_camera = pytest.mark.skipif(not has_camera,
                                  reason="camera tests are opt-in: TRACKSHOT_HW_TESTS=1 with an OAK on the network")


# --- Pure helpers --------------------------------------------------------------------------------

def test_device_time_to_wall():
    now_dev = timedelta(hours=30)
    assert op.device_time_to_wall(now_dev - timedelta(milliseconds=80), now_dev, 1000.0) == pytest.approx(999.92)
    assert op.device_time_to_wall(timedelta(0), now_dev, 1000.0) is None                 # unset (VIO poses)
    assert op.device_time_to_wall(None, now_dev, 1000.0) is None
    assert op.device_time_to_wall(now_dev + timedelta(seconds=1), now_dev, 1000.0) is None  # future
    assert op.device_time_to_wall(now_dev - timedelta(seconds=6), now_dev, 1000.0) is None  # stale


def test_device_time_to_wall_with_the_real_clock():
    ts = dai.Clock.now() - timedelta(milliseconds=50)
    wall = op.device_time_to_wall(ts, dai.Clock.now(), time.time())
    assert wall == pytest.approx(time.time() - 0.05, abs=0.01)


@pytest.mark.parametrize("text", [
    "Failed to connect to device with name \"192.168.2.118\". Error: X_LINK_DEVICE_ALREADY_IN_USE",
    "Failed to find device (192.168.2.118), error message: X_LINK_DEVICE_ALREADY_IN_USE",
    "Device not in UNBOOTED, BOOTLOADER or FLASH_BOOTED state",
    "DeviceInfo(name=192.168.2.118, deviceId=x, X_LINK_BOOTED, X_LINK_TCP_IP, ...)",
])
def test_busy_errors(text):
    assert op.is_busy_error(RuntimeError(text))
    assert op.connect_error_message(RuntimeError(text), "192.168.2.118") == op.BUSY_MESSAGE


def test_busy_exception_type():
    assert op.is_busy_error(op.CameraBusyError("anything"))


@pytest.mark.parametrize("text", ["Cannot find any device with given deviceInfo", "No available devices",
                                  "X_LINK_TIMEOUT"])
def test_not_busy_errors(text):
    assert not op.is_busy_error(RuntimeError(text))


def test_connect_error_messages():
    msg = op.connect_error_message(RuntimeError("Cannot find any device with given deviceInfo"), "192.168.2.118")
    assert "No camera answered at 192.168.2.118" in msg
    assert "No OAK camera found" in op.connect_error_message(RuntimeError("No OAK camera found"), "")
    assert op.connect_error_message(RuntimeError("X_LINK_TIMEOUT"), "10.0.0.5") == \
        "Could not connect to the camera at 10.0.0.5: X_LINK_TIMEOUT"


def test_rate_meter():
    clock = SimpleNamespace(t=0.0)
    meter = op.RateMeter(window_s=2.0, clock=lambda: clock.t)
    assert meter.rate() == 0.0
    for i in range(40):                      # 20 Hz for 2 s
        clock.t = 0.05 * (i + 1)
        meter.add()
    assert meter.rate() == pytest.approx(20.0, rel=0.05)
    clock.t = 3.0                            # 1 s of silence: half the window is still full
    assert meter.rate() == pytest.approx(10.0, rel=0.1)
    clock.t = 10.0
    assert meter.rate() == 0.0


def test_rate_meter_bytes_and_startup():
    clock = SimpleNamespace(t=0.0)
    meter = op.RateMeter(window_s=2.0, clock=lambda: clock.t)
    clock.t = 0.01
    meter.add(1000)                          # a first sample can't become a 100 kB/s spike
    assert meter.rate() == pytest.approx(2000.0)
    clock.t = 1.0
    meter.add(1000)
    assert meter.rate() == pytest.approx(2000.0)


def test_preview_throttle_caps_a_fast_stream():
    throttle = op.PreviewThrottle(max_hz=10.0)
    passed = sum(throttle.ready(i / 30.0) for i in range(300))   # 30 fps for 10 s
    assert 95 <= passed <= 101


def test_preview_throttle_keeps_a_jittery_stream_at_the_limit():
    rng = np.random.default_rng(0)
    throttle = op.PreviewThrottle(max_hz=10.0)
    times = np.arange(100) * 0.1 + rng.uniform(-0.003, 0.003, 100)
    assert all(throttle.ready(t) for t in times)


def test_preview_throttle_restarts_after_a_pause():
    throttle = op.PreviewThrottle(max_hz=10.0)
    assert throttle.ready(0.0)
    assert throttle.ready(5.0)
    assert not throttle.ready(5.01)          # no burst to "catch up" the pause
    assert throttle.ready(5.1)


# --- depthai configuration objects ---------------------------------------------------------------

ENUM_FIELDS = {"optical_flow_matching_guess_type", "vio_linearization_type", "vio_kf_marg_criteria"}


def test_basalt_defaults_cover_every_settable_field():
    fields = {f for f in dir(dai.VioConfig()) if not f.startswith("_")}
    assert set(op.DEPTHAI_BASALT_DEFAULTS) == fields - ENUM_FIELDS


def test_basalt_config_maps_settings_onto_depthai_defaults():
    s = Settings(basalt_grid_size=64, basalt_flow_iterations=4, basalt_flow_levels=2, basalt_max_states=2,
                 basalt_max_kfs=6, basalt_vio_iterations=5, basalt_skip_frames=2)
    cfg = op.basalt_vio_config(s)
    assert isinstance(cfg, dai.VioConfig)
    assert cfg.optical_flow_detection_grid_size == 64
    assert cfg.optical_flow_max_iterations == 4
    assert cfg.optical_flow_levels == 2
    assert cfg.vio_max_states == 2
    assert cfg.vio_max_kfs == 6
    assert cfg.vio_max_iterations == 5
    assert cfg.optical_flow_skip_frames == 2
    # Everything else keeps depthai's default instead of VioConfig()'s zeros
    assert cfg.optical_flow_type == "frame_to_frame"
    assert cfg.vio_obs_std_dev == pytest.approx(0.5)
    assert cfg.optical_flow_pattern == 51
    assert cfg.optical_flow_recall_max_patch_norms == pytest.approx([1.74, 0.96, 0.99, 0.44])
    assert cfg.mapper_lm_lambda_max == pytest.approx(1e3)


def test_basalt_node_accepts_the_config():
    pipeline = dai.Pipeline(False)            # host-only: the VIO nodes can be built without a camera
    odom = pipeline.create(dai.node.BasaltVIO)
    odom.setConfig(op.basalt_vio_config(Settings()))
    assert odom.transform.createOutputQueue(maxSize=8, blocking=False) is not None
    assert odom.passthrough.createOutputQueue(maxSize=8, blocking=False) is not None


def test_rtabmap_params_on_the_real_node():
    params = op.rtabmap_params(Settings(rtabmap_features=400, rtabmap_local_map_size=800))
    assert params == {"OdomF2M/MaxSize": "800", "Vis/MaxFeatures": "400"}
    pipeline = dai.Pipeline(False)
    odom = pipeline.create(dai.node.RTABMapVIO)
    odom.setParams(params)
    for out in (odom.transform, odom.passthroughRect):
        assert out.createOutputQueue(maxSize=2, blocking=False) is not None


# --- Frame helpers -------------------------------------------------------------------------------

def make_frame(width=8, height=4, stride=None, seq=0, age_s=0.05, fill=None):
    stride = stride or width
    frame = dai.ImgFrame()
    frame.setWidth(width)
    frame.setHeight(height)
    frame.setStride(stride)
    frame.setType(dai.ImgFrame.Type.GRAY8)
    data = fill if fill is not None else (np.arange(stride * height) % 251).astype(np.uint8)
    frame.setData(data)
    frame.setSequenceNum(seq)
    frame.setTimestamp(dai.Clock.now() - timedelta(seconds=age_s))
    return frame


def test_grey_image_drops_the_row_padding():
    frame = make_frame(width=8, height=4, stride=10)
    image = op.grey_image(frame)
    expected = (np.arange(40) % 251).astype(np.uint8).reshape(4, 10)[:, :8]
    assert image.shape == (4, 8) and image.dtype == np.uint8
    np.testing.assert_array_equal(image, expected)


def test_decode_jpeg_rgb():
    buf = io.BytesIO()
    Image.new("RGB", (64, 36), (200, 30, 30)).save(buf, format="JPEG")
    image = op.decode_jpeg_rgb(buf.getvalue())
    assert image.shape == (36, 64, 3)
    assert image[0, 0, 0] > 150


def test_feature_positions():
    tracked = dai.TrackedFeatures()
    features = []
    for x, y in ((3.0, 4.0), (10.5, 20.25)):
        f = dai.TrackedFeature()
        f.position = dai.Point2f(x, y)
        features.append(f)
    tracked.trackedFeatures = features
    np.testing.assert_allclose(op.feature_positions(tracked), [[3.0, 4.0], [10.5, 20.25]])
    assert op.feature_positions(dai.TrackedFeatures()).shape == (0, 2)


def test_lens_for():
    calib = mock.Mock()
    calib.getCameraIntrinsics.return_value = [[1500.0, 0, 960], [0, 1500.0, 540], [0, 0, 1]]
    lens = op.lens_for(calib, dai.CameraBoardSocket.CAM_A, op.CAMERA_MAKE, b"IMX378", 1920, 1080,
                       op.IMX378_PIXEL_MM, op.IMX378_NATIVE_WIDTH)
    calib.getCameraIntrinsics.assert_called_once_with(dai.CameraBoardSocket.CAM_A, 1920, 1080)
    pixel_mm = op.IMX378_PIXEL_MM * op.IMX378_NATIVE_WIDTH / 1920
    assert lens.pixel_mm == pytest.approx(pixel_mm)
    assert lens.focal_mm == pytest.approx(1500.0 * pixel_mm)


def test_lens_for_without_calibration():
    assert op.lens_for(dai.CalibrationHandler(), dai.CameraBoardSocket.CAM_A, op.CAMERA_MAKE, b"IMX378",
                       1920, 1080, op.IMX378_PIXEL_MM, op.IMX378_NATIVE_WIDTH) is None


# --- Lifecycle against no camera -----------------------------------------------------------------

class Recorder:
    """SourceCallbacks that record everything, thread-safely."""

    def __init__(self):
        self.lock = threading.Lock()
        self.states, self.logs, self.poses, self.previews, self.captures = [], [], [], [], []
        self.callbacks = SourceCallbacks(
            on_pose=lambda s: self._add(self.poses, s),
            on_preview=lambda f: self._add(self.previews, f),
            on_capture=lambda j: self._add(self.captures, j),
            on_state=lambda state, msg: self._add(self.states, (state, msg)),
            on_log=lambda level, msg: self._add(self.logs, (level, msg)),
        )

    def _add(self, target, item):
        with self.lock:
            target.append(item)

    def wait_for_state(self, state, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for s, msg in self.states:
                    if s == state:
                        return msg
            time.sleep(0.02)
        raise AssertionError(f"no {state} state within {timeout} s; got {self.states}")


@pytest.fixture
def no_cameras(monkeypatch):
    monkeypatch.setattr(dai.Device, "getAllConnectedDevices", staticmethod(lambda: []))
    # depthai's direct-address search otherwise takes 10 s per attempt
    monkeypatch.setenv("DEPTHAI_SEARCH_TIMEOUT", "300")


def test_no_camera_ends_in_error_within_the_window(no_cameras):
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip=""), rec.callbacks, connect_window_s=0.5, retry_interval_s=0.1)
    t0 = time.monotonic()
    source.start()
    message = rec.wait_for_state(STATE_ERROR, timeout=5.0)
    assert time.monotonic() - t0 < 3.0
    assert "No OAK camera found" in message
    assert rec.states[0][0] == STATE_CONNECTING
    source._thread.join(2.0)
    assert not source.is_running()
    source.stop()
    source.stop()


def test_no_camera_at_an_address(no_cameras):
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip="192.168.2.118"), rec.callbacks,
                              connect_window_s=0.5, retry_interval_s=0.1)
    source.start()
    message = rec.wait_for_state(STATE_ERROR, timeout=8.0)
    assert "192.168.2.118" in message
    source.stop()


def test_busy_camera_says_to_stop_the_extension(monkeypatch):
    booted = dai.DeviceInfo("192.168.2.118", "18443010B1", dai.XLinkDeviceState.X_LINK_BOOTED,
                            dai.XLinkProtocol.X_LINK_TCP_IP, dai.XLinkPlatform.X_LINK_MYRIAD_X,
                            dai.XLinkError_t.X_LINK_SUCCESS)
    # Discovery reports the camera as booted (owned by the extension); connecting must not be tried
    connect = mock.Mock(side_effect=AssertionError("must not connect to a booted camera"),
                        getAllConnectedDevices=lambda: [booted])
    monkeypatch.setattr(op.dai, "Device", connect)
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip="192.168.2.118"), rec.callbacks,
                              connect_window_s=0.4, retry_interval_s=0.1)
    source.start()
    assert rec.wait_for_state(STATE_ERROR, timeout=5.0) == op.BUSY_MESSAGE
    assert any(s == STATE_CONNECTING and op.BUSY_MESSAGE in (m or "") for s, m in rec.states)
    connect.assert_not_called()
    source.stop()


def test_first_free_camera_is_preferred_over_a_busy_one(monkeypatch):
    def info(name, state):
        return dai.DeviceInfo(name, name, state, dai.XLinkProtocol.X_LINK_TCP_IP,
                              dai.XLinkPlatform.X_LINK_MYRIAD_X, dai.XLinkError_t.X_LINK_SUCCESS)
    busy = info("10.0.0.1", dai.XLinkDeviceState.X_LINK_BOOTED)
    free = info("10.0.0.2", dai.XLinkDeviceState.X_LINK_BOOTLOADER)
    fake_device = mock.Mock(getAllConnectedDevices=lambda: [busy, free])
    monkeypatch.setattr(op.dai, "Device", fake_device)
    op.open_device("")
    assert fake_device.call_args.args[0].name == "10.0.0.2"
    fake_device.getAllConnectedDevices = lambda: [busy]
    with pytest.raises(op.CameraBusyError):
        op.open_device("")


def test_unavailable_backend_fails_clearly(monkeypatch):
    monkeypatch.setattr(capabilities, "available_backends", lambda: ["rtabmap"])
    rec = Recorder()
    source = op.OakPoseSource(Settings(backend="basalt"), rec.callbacks)
    source.start()
    message = rec.wait_for_state(STATE_ERROR, timeout=2.0)
    assert "basalt" in message and "rtabmap" in message
    source.stop()


def test_stop_is_idempotent_and_interrupts_the_retry_wait(no_cameras):
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip=""), rec.callbacks, connect_window_s=30.0, retry_interval_s=5.0)
    source.stop()                             # before start: nothing happens
    assert rec.states == []
    source.start()
    source.start()                            # already running: no second thread
    rec.wait_for_state(STATE_CONNECTING, timeout=2.0)
    time.sleep(0.8)                           # into the retry wait
    t0 = time.monotonic()
    source.stop()
    assert time.monotonic() - t0 < 2.0
    assert not source.is_running()
    source.stop()
    assert [s for s, _ in rec.states].count(STATE_STOPPED) == 1
    assert STATE_ERROR not in [s for s, _ in rec.states]


def test_restart_after_stop(no_cameras):
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip=""), rec.callbacks, connect_window_s=0.3, retry_interval_s=0.1)
    source.start()
    rec.wait_for_state(STATE_ERROR, timeout=5.0)
    source.stop()
    rec.states.clear()
    source.start()
    rec.wait_for_state(STATE_ERROR, timeout=5.0)
    source.stop()


# --- Output queue handling, fed with real depthai messages ---------------------------------------

class FakeQueue:
    def __init__(self, items=(), max_size=op.VIO_QUEUE_SIZE):
        self.items = list(items)
        self.max_size = max_size

    def getSize(self):
        return len(self.items)

    def getMaxSize(self):
        return self.max_size

    def tryGetAll(self):
        items, self.items = self.items, []
        return items

    def get(self, timeout):
        return self.items.pop(0) if self.items else None


def make_session(settings, rec):
    source = op.OakPoseSource(settings, rec.callbacks)
    source._state = STATE_RUNNING
    return source, op._Session(source, settings, device=mock.Mock(), pipeline=mock.Mock())


def test_poses_take_their_capture_time_from_the_passthrough_frame():
    rec = Recorder()
    source, session = make_session(Settings(backend="basalt", preview_enabled=True), rec)
    poses = [dai.TransformData(0.1 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) for i in range(3)]
    frames = [make_frame(width=32, height=20, seq=10 + i, age_s=0.06) for i in range(3)]
    session.q_pose, session.q_frame = FakeQueue(poses), FakeQueue(frames)
    assert session._poll()
    assert [p.position[0] for p in rec.poses] == pytest.approx([0.0, 0.1, 0.2])
    for sample in rec.poses:
        assert sample.quaternion == (1.0, 0.0, 0.0, 0.0)
        assert sample.t_received - sample.t_capture_us / 1e6 == pytest.approx(0.06, abs=0.01)
    stats = source.stats()
    assert stats.extra["poses"] == 3
    assert stats.extra["latency_ms"] == pytest.approx(60, abs=10)
    assert stats.rx_mbit > 0
    # Three frames back to back: the preview throttle lets only the first through
    assert len(rec.previews) == 1 and rec.previews[0].kind == "vio"
    assert rec.previews[0].image.shape == (20, 32) and rec.previews[0].features is None


def test_running_is_reported_with_the_first_pose():
    rec = Recorder()
    source, session = make_session(Settings(backend="basalt"), rec)
    session.q_pose, session.q_frame = FakeQueue(), FakeQueue()
    session._poll()
    assert rec.states == []                                   # camera started, no VIO output yet
    session.q_pose.items = [dai.TransformData(0, 0, 0, 0, 0, 0, 1) for _ in range(2)]
    session.q_frame.items = [make_frame(seq=1), make_frame(seq=2)]
    session._poll()
    assert [s for s, _ in rec.states] == [STATE_RUNNING]      # once, on the first pose
    assert session.vio_output


def test_a_basalt_burst_keeps_only_the_pose_its_frame_belongs_to():
    """Basalt sends the newest left image after each pose: three poses popped in a burst all carry
    image 13. Only the last is really image 13's; the earlier two are skipped, so the next delta
    runs from the pose before the burst, over the right span."""
    rec = Recorder()
    source, session = make_session(Settings(backend="basalt"), rec)
    poses = [dai.TransformData(0.05 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) for i in range(5)]
    frames = [make_frame(seq=10, age_s=0.30), make_frame(seq=13, age_s=0.10), make_frame(seq=13, age_s=0.10),
              make_frame(seq=13, age_s=0.10), make_frame(seq=14, age_s=0.05)]
    session.q_pose, session.q_frame = FakeQueue(poses), FakeQueue(frames)
    session._poll()
    assert [round(p.position[0], 2) for p in rec.poses] == [0.0, 0.15, 0.2]
    assert source.stats().extra["stale_poses"] == 2
    # The resent image was counted once for the rx rate
    total = sum(amount for _, amount in source._rx_meter._samples)
    assert total == (1 + 3 + 1) * 2 * 8 * 4


def test_a_burst_split_across_polls_is_counted_and_differenced_from_the_true_frame():
    rec = Recorder()
    source, session = make_session(Settings(backend="basalt"), rec)
    session.q_pose = FakeQueue([dai.TransformData(0.1, 0, 0, 0, 0, 0, 1)])
    session.q_frame = FakeQueue([make_frame(seq=5)])
    session._poll()
    session.q_pose.items = [dai.TransformData(0.2, 0, 0, 0, 0, 0, 1)]
    session.q_frame.items = [make_frame(seq=5)]
    session._poll()
    assert len(rec.poses) == 2 and source.stats().extra["stale_poses"] == 1


def test_rtabmap_frames_with_one_sequence_number_are_not_bursts():
    rec = Recorder()
    _, session = make_session(Settings(backend="rtabmap"), rec)
    session.q_pose = FakeQueue([dai.TransformData(0, 0, 0, 0, 0, 0, 1) for _ in range(2)])
    session.q_frame = FakeQueue([make_frame(seq=3), make_frame(seq=3)])
    session._poll()
    assert len(rec.poses) == 2


def test_orphan_frames_are_dropped_not_paired_with_later_poses():
    """Frame 9's pose was already handled (it timed out waiting, or was discarded): frame 9 must not
    become pose 10's frame."""
    rec = Recorder()
    source, session = make_session(Settings(backend="rtabmap"), rec)
    session.q_frame = FakeQueue([make_frame(seq=9, age_s=0.2), make_frame(seq=10, age_s=0.05)])
    session.q_pose = FakeQueue([dai.TransformData(0, 0, 0, 0, 0, 0, 1)])
    session._poll()
    assert rec.poses[0].t_received - rec.poses[0].t_capture_us / 1e6 == pytest.approx(0.05, abs=0.01)
    assert source.stats().extra["orphan_frames"] == 1


def test_a_full_queue_discards_the_batch_and_pairing_recovers():
    """The worker stalled long enough for both queues to overflow, q_pose having dropped one more
    than q_frame. Pairing that batch would offset every pose by a frame: it is discarded, and the
    frame left in flight is dropped as an orphan on the next poll."""
    rec = Recorder()
    source, session = make_session(Settings(backend="rtabmap"), rec)
    size = 4
    session.q_pose = FakeQueue([dai.TransformData(0.1 * k, 0, 0, 0, 0, 0, 1) for k in range(2, 6)], max_size=size)
    session.q_frame = FakeQueue([make_frame(seq=k) for k in range(1, 5)], max_size=size)
    session._poll()
    assert rec.poses == [] and source.stats().extra["overflow_resyncs"] == 1
    session.q_frame.items = [make_frame(seq=5, age_s=0.3)]            # pose 5's frame, still in flight
    session._poll()
    session.q_frame.items = [make_frame(seq=6, age_s=0.05)]
    session.q_pose.items = [dai.TransformData(0.6, 0, 0, 0, 0, 0, 1)]
    session._poll()
    assert len(rec.poses) == 1
    assert rec.poses[0].t_received - rec.poses[0].t_capture_us / 1e6 == pytest.approx(0.05, abs=0.01)


def test_a_pose_waits_briefly_for_its_frame():
    rec = Recorder()
    _, session = make_session(Settings(backend="basalt"), rec)
    session.q_pose = FakeQueue([dai.TransformData(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)])
    late = make_frame(seq=1)

    class LateQueue(FakeQueue):
        def tryGetAll(self):
            return []                          # not there yet when the queues are drained...

        def get(self, timeout):
            return late                        # ...but arrives right after the pose
    session.q_frame = LateQueue()
    session._poll()
    assert rec.poses[0].t_capture_us is not None


def test_pose_without_frame_has_no_capture_time():
    rec = Recorder()
    source, session = make_session(Settings(backend="basalt"), rec)
    session.q_pose, session.q_frame = FakeQueue([dai.TransformData(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)]), FakeQueue()
    session._poll()
    assert rec.poses[0].t_capture_us is None
    assert source.stats().extra["untimed_poses"] == 1


def test_non_finite_poses_are_dropped():
    rec = Recorder()
    source, session = make_session(Settings(backend="rtabmap"), rec)
    session.q_pose = FakeQueue([dai.TransformData(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)])
    session.q_frame = FakeQueue([make_frame()])
    session._poll()
    assert rec.poses == []
    assert source.stats().extra["invalid_poses"] == 1


def test_rtabmap_preview_carries_features_and_honours_set_preview():
    rec = Recorder()
    source, session = make_session(Settings(backend="rtabmap", preview_enabled=True), rec)
    tracked = dai.TrackedFeatures()
    feature = dai.TrackedFeature()
    feature.position = dai.Point2f(5.0, 6.0)
    tracked.trackedFeatures = [feature]
    session.q_features = FakeQueue([tracked])
    session.q_pose = FakeQueue([dai.TransformData(0, 0, 0, 0, 0, 0, 1)])
    session.q_frame = FakeQueue([make_frame()])
    session._poll()
    np.testing.assert_allclose(rec.previews[0].features, [[5.0, 6.0]])

    source.set_preview(vio=False, color=False)
    session.vio_throttle = op.PreviewThrottle()
    session.q_pose.items = [dai.TransformData(0, 0, 0, 0, 0, 0, 1)]
    session.q_frame.items = [make_frame(seq=1)]
    session._poll()
    assert len(rec.previews) == 1 and len(rec.poses) == 2


def test_rx_counts_frames_the_host_dropped():
    rec = Recorder()
    source, session = make_session(Settings(backend="rtabmap"), rec)
    frame_bytes = 8 * 4
    session._count_vio_frame(make_frame(seq=100))
    session._count_vio_frame(make_frame(seq=103))      # 2 frames dropped in between
    total = sum(amount for _, amount in source._rx_meter._samples)
    assert total == (1 + 3) * 3 * frame_bytes           # RTAB-Map: rect + 16-bit depth = 3x


def test_colour_preview_and_capture_frames():
    rec = Recorder()
    source, session = make_session(Settings(preview_enabled=True, color_preview_enabled=True), rec)
    source._preview_color = True
    buf = io.BytesIO()
    Image.new("RGB", (64, 36), (10, 200, 10)).save(buf, format="JPEG")
    jpeg = np.frombuffer(buf.getvalue(), dtype=np.uint8)

    def encoded(seq):
        frame = make_frame(seq=seq, age_s=0.1)
        frame.setData(jpeg)
        return frame
    session.q_pose, session.q_frame = FakeQueue(), FakeQueue()
    session.q_color = FakeQueue([encoded(1), encoded(2)])
    session.captures = {"center": (FakeQueue([encoded(7)]), "lens-c"), "left": (FakeQueue([encoded(8)]), None)}
    assert session._poll()
    colour = [p for p in rec.previews if p.kind == "color"]
    assert len(colour) == 1 and colour[0].image.shape == (36, 64, 3)
    by_stream = {c.stream: c for c in rec.captures}
    assert by_stream["center"].data == buf.getvalue()
    assert by_stream["center"].sequence == 7 and by_stream["center"].lens == "lens-c"
    assert by_stream["center"].t_capture == pytest.approx(time.time() - 0.1, abs=0.02)
    assert by_stream["left"].lens is None


def test_build_wires_backend_preview_and_capture(monkeypatch):
    """Pipeline construction with the device nodes mocked (they need a camera to exist)."""
    rec = Recorder()
    s = Settings(backend="basalt", fps=10, preview_enabled=True, color_preview_enabled=True, color_preview_fps=5.0,
                 center_capture_fps=2.0, stereo_capture_fps=15.0)
    source = op.OakPoseSource(s, rec.callbacks)
    device = mock.Mock()
    device.getConnectedIMU.return_value = "BMI270"
    device.getProductName.return_value = "OAK-D-W-PoE"
    device.getDeviceId.return_value = "18443010B1"
    device.readCalibration.return_value = dai.CalibrationHandler()
    pipeline = mock.Mock()
    created = []

    def create(node_type):
        node = mock.Mock(name=node_type.__name__)
        node.build.return_value = node
        created.append((node_type, node))
        return node
    pipeline.create.side_effect = create
    session = op._Session(source, s, device, pipeline)
    session.build()

    by_type = {}
    for node_type, node in created:
        by_type.setdefault(node_type, []).append(node)
    cameras = by_type[dai.node.Camera]
    sockets = [call.args[0] for cam in cameras for call in cam.build.call_args_list]
    assert sockets == [dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_A]
    assert cameras[2].build.call_args.kwargs["sensorFps"] == 5.0        # max(colour 5, centre 2, 5)
    basalt = by_type[dai.node.BasaltVIO][0]
    assert isinstance(basalt.setConfig.call_args.args[0], dai.VioConfig)
    # Stereo capture is capped at the VIO frame rate
    stereo_requests = [c for c in cameras[0].requestOutput.call_args_list if c.args[0] == (s.mono_width, s.mono_height)]
    assert stereo_requests[0].kwargs["fps"] == 10.0
    assert set(session.captures) == {"center", "left", "right"}
    assert session.q_color is not None and source._color_built
    stats = source.stats()
    assert (stats.camera_name, stats.camera_id, stats.imu) == ("OAK-D-W-PoE", "18443010B1", "BMI270")


def test_rtabmap_build_leaves_feature_detection_to_rtabmap():
    """setUseFeatures(True) would copy RTABMapVIO's features input into its Sync after (or before)
    the link, so the OAK's features would never reach it: it isn't called, as in the extension, and
    the overlay reads the FeatureTracker directly."""
    rec = Recorder()
    s = Settings(backend="rtabmap")
    device = mock.Mock()
    device.getConnectedIMU.return_value = "BMI270"
    pipeline = mock.Mock()
    created = {}

    def create(node_type):
        node = mock.Mock(name=node_type.__name__)
        node.build.return_value = node
        created.setdefault(node_type, []).append(node)
        return node
    pipeline.create.side_effect = create
    session = op._Session(op.OakPoseSource(s, rec.callbacks), s, device, pipeline)
    session.build()
    odom = created[dai.node.RTABMapVIO][0]
    tracker = created[dai.node.FeatureTracker][0]
    odom.setUseFeatures.assert_not_called()
    tracker.outputFeatures.link.assert_called_once_with(odom.features)
    assert session.q_features is tracker.outputFeatures.createOutputQueue.return_value
    odom.passthroughFeatures.createOutputQueue.assert_not_called()


# --- Stopping Basalt before its first frame -------------------------------------------------------

def basalt_session(rec, frames=()):
    source, session = make_session(Settings(backend="basalt"), rec)
    session.started = True
    session.pipeline.isRunning.return_value = True
    session.q_pose, session.q_frame = FakeQueue(), FakeQueue(frames)
    return source, session


def test_a_basalt_pipeline_with_output_is_stopped_normally(monkeypatch):
    monkeypatch.setattr(op, "_PARKED_PIPELINES", [])
    rec = Recorder()
    source, session = basalt_session(rec)
    session.vio_output = True
    device = mock.Mock()
    source._release(session.pipeline, device, session)
    session.pipeline.stop.assert_called_once()
    device.close.assert_called_once()
    assert not op.has_parked_pipelines()


def test_stopping_basalt_during_warm_up_waits_for_its_first_frame(monkeypatch):
    monkeypatch.setattr(op, "_PARKED_PIPELINES", [])
    rec = Recorder()
    source, session = basalt_session(rec)

    class FirstFrameLater(FakeQueue):
        calls = 0

        def tryGetAll(self):
            self.calls += 1
            return [make_frame()] if self.calls > 3 else []
    session.q_frame = FirstFrameLater()
    source._release(session.pipeline, mock.Mock(), session)
    session.pipeline.stop.assert_called_once()
    assert not op.has_parked_pipelines()


def test_a_basalt_pipeline_that_never_got_a_frame_is_parked_not_stopped(monkeypatch):
    monkeypatch.setattr(op, "_PARKED_PIPELINES", [])
    monkeypatch.setattr(op, "BASALT_FIRST_FRAME_WAIT_S", 0.1)
    rec = Recorder()
    source, session = basalt_session(rec)
    device = mock.Mock()
    t0 = time.monotonic()
    source._release(session.pipeline, device, session)
    assert time.monotonic() - t0 < 1.0
    session.pipeline.stop.assert_not_called()
    device.close.assert_called_once()                           # the camera is still released
    assert op.has_parked_pipelines()
    assert any("Basalt never received" in m for _, m in rec.logs)


def test_parking_avoids_the_depthai_crash_end_to_end(tmp_path):
    """The real depthai: a started Basalt pipeline with no frames crashes (SIGSEGV) on stop() or
    when destroyed. Released through OakPoseSource and left with os._exit, the process exits 0."""
    script = tmp_path / "park.py"
    script.write_text(textwrap.dedent("""
        import os, sys
        import depthai as dai
        from unittest import mock
        from trackshot.config import Settings
        from trackshot.vio import oak_pipeline as op
        from trackshot.vio.source import SourceCallbacks
        op.BASALT_FIRST_FRAME_WAIT_S = 0.3
        p = dai.Pipeline(False)
        p.setCalibrationData(dai.CalibrationHandler())
        odom = p.create(dai.node.BasaltVIO)
        for i in (odom.left, odom.right, odom.imu):
            i.createInputQueue()
        for n in p.getAllNodes():
            if n.getName() == "Sync":
                n.setRunOnHost(True)
        source = op.OakPoseSource(Settings(backend="basalt"), SourceCallbacks())
        session = op._Session(source, Settings(backend="basalt"), mock.Mock(), p)
        session.q_pose = odom.transform.createOutputQueue(maxSize=8, blocking=False)
        session.q_frame = odom.passthrough.createOutputQueue(maxSize=8, blocking=False)
        session.started = True
        p.start()
        source._release(p, None, session)
        del p, odom, session
        print("parked", op.has_parked_pipelines(), flush=True)
        os._exit(0 if op.has_parked_pipelines() else 3)
    """))
    result = subprocess.run([sys.executable, str(script)], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "parked True" in result.stdout


def test_build_refuses_a_camera_without_imu():
    rec = Recorder()
    source = op.OakPoseSource(Settings(), rec.callbacks)
    device = mock.Mock()
    device.getConnectedIMU.return_value = "NONE"
    with pytest.raises(op._Failure, match="no IMU"):
        op._Session(source, Settings(), device, mock.Mock()).build()


# --- With a camera -------------------------------------------------------------------------------

@needs_camera
@pytest.mark.parametrize("backend", capabilities.available_backends())
def test_real_camera_streams_poses(backend):
    camera = capabilities.list_cameras()[0]
    rec = Recorder()
    source = op.OakPoseSource(Settings(camera_ip=camera["name"], backend=backend), rec.callbacks)
    source.start()
    try:
        rec.wait_for_state(STATE_RUNNING, timeout=40.0)
        time.sleep(8.0)
        assert rec.poses, "no poses in 8 s"
        assert source.stats().imu
    finally:
        source.stop(timeout=10.0)
    assert not source.is_running()
