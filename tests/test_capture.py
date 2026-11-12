"""CaptureWriter and photogrammetry metadata: files, EXIF/XMP, CSV, pairing and failure paths."""

from __future__ import annotations

import csv
import errno
import io
import os
import threading
from datetime import datetime
from pathlib import Path

import piexif
import pytest
from PIL import Image

from trackshot.capture import writer as writer_mod
from trackshot.capture.meta import Lens
from trackshot.capture.writer import CSV_HEADER, CaptureWriter, survey_day_folder
from trackshot.vio.source import CapturedJpeg

T0 = 1_760_000_000.0  # a fixed wall-clock capture time, so the day folder is predictable

TELEMETRY = {
    "lat": -33.8567844, "lon": 151.2152967, "heading_deg": 271.5, "roll_deg": 1.2,
    "pitch_deg": -3.4, "depth_m": 12.345, "temperature_c": 18.25, "altitude_m": 2.5,
    "mission_seq": None, "connected": True, "armed": True, "mode": "POSHOLD",
    "heartbeat_age_s": 0.1, "age_s": 0.25,
}
POSE = (1.0, -2.0, 0.5, 1.0, 0.0, 0.0, 0.0)


class FakeClock:
    def __init__(self, now: float):
        self.now = now

    def __call__(self) -> float:
        return self.now


def _jpeg_bytes(shade: int = 128) -> bytes:
    buf = io.BytesIO()
    Image.new("L", (32, 24), shade).save(buf, format="JPEG", quality=80)
    return buf.getvalue()


JPEG = _jpeg_bytes()


def frame(stream: str, t: float = T0, seq: int = 1, lens=None) -> CapturedJpeg:
    return CapturedJpeg(stream=stream, data=JPEG, t_capture=t, sequence=seq, lens=lens)


@pytest.fixture
def clock():
    return FakeClock(T0 + 0.1)


@pytest.fixture
def make_writer(tmp_path, clock):
    made = []

    def make(**kwargs):
        kwargs.setdefault("min_free_gb", 0.0)
        w = CaptureWriter(tmp_path, clock=clock, **kwargs)
        made.append(w)
        return w

    yield make
    for w in made:
        w.close(timeout=2.0)


@pytest.fixture
def hold_writes(monkeypatch):
    """Writer threads wait in embed_metadata until the returned event is set."""
    gate = threading.Event()
    real_embed = writer_mod.meta.embed_metadata

    def held_embed(*args, **kwargs):
        gate.wait(5.0)
        return real_embed(*args, **kwargs)

    monkeypatch.setattr(writer_mod.meta, "embed_metadata", held_embed)
    yield gate
    gate.set()


def day(tmp_path) -> str:
    return survey_day_folder(str(tmp_path), datetime.fromtimestamp(T0))


def read_csv(path) -> list[list[str]]:
    with open(path, newline="") as f:
        return list(csv.reader(f))


def jpgs(folder) -> list[str]:
    return sorted(n for n in os.listdir(folder) if n.endswith(".jpg")) if os.path.isdir(folder) else []


def test_center_write_with_exif_and_xmp(tmp_path, make_writer):
    lens = Lens.from_intrinsics("Luxonis", "OAK-D CAM_A", fx_px=1000.0, native_pixel_mm=0.00155,
                                native_width_px=4056, width_px=1920)
    w = make_writer(camera_angle_deg=45.0)
    w.set_recording(True)
    w.submit(frame("center", lens=lens), TELEMETRY, POSE, True)
    w.close()

    path = os.path.join(day(tmp_path), "center", "000001_tl000000_00001.jpg")
    assert os.path.exists(path)
    exif = piexif.load(Path(path).read_bytes())
    gps = exif["GPS"]
    assert gps[piexif.GPSIFD.GPSLatitudeRef] == b"S"
    assert gps[piexif.GPSIFD.GPSLongitudeRef] == b"E"
    assert gps[piexif.GPSIFD.GPSLatitude][0] == (33, 1)
    assert gps[piexif.GPSIFD.GPSAltitudeRef] == 1  # below sea level = depth
    assert gps[piexif.GPSIFD.GPSAltitude] == (12345, 1000)
    focal = exif["Exif"][piexif.ExifIFD.FocalLength]
    assert focal[0] / focal[1] == pytest.approx(lens.focal_mm, abs=1e-3)
    assert exif["0th"][piexif.ImageIFD.Make] == b"Luxonis"
    assert exif["0th"][piexif.ImageIFD.Software] == b"TrackShot"

    data = Path(path).read_bytes()
    assert b"<Camera:Yaw>271.50</Camera:Yaw>" in data
    assert b"<Camera:Pitch>41.60</Camera:Pitch>" in data  # 90 - 45 mount + (-3.4) body pitch
    assert b"<ROV:VioTracking>1.00</ROV:VioTracking>" in data

    snap = w.snapshot()
    assert snap["counts"] == {"center": 1, "stereo": 0}
    assert snap["last_file"] == path
    assert snap["folder"] == day(tmp_path)


def test_csv_header_and_row(tmp_path, make_writer):
    w = make_writer()
    w.set_recording(True)
    w.submit(frame("center"), TELEMETRY, POSE, True)
    w.submit(frame("center", t=T0 + 0.05, seq=2), None, None, None)
    w.close()

    rows = read_csv(os.path.join(day(tmp_path), "telemetry_center.csv"))
    assert rows[0] == CSV_HEADER
    rows = sorted(rows[1:], key=lambda r: int(r[1]))
    assert len(rows) == 2
    row = dict(zip(CSV_HEADER, rows[0]))
    assert row["timestamp"] == datetime.fromtimestamp(T0).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    assert row["seq"] == "1" and row["frame"] == "1"
    assert row["filename"] == "000001_tl000000_00001.jpg"
    assert row["source_tag"] == "tl000000" and row["wp"] == ""
    assert int(row["size_bytes"]) > len(JPEG)  # metadata was added
    assert row["lat"] == "-33.8567844" and row["lon"] == "151.2152967"
    assert row["heading_deg"] == "271.5" and row["depth_m"] == "12.35"
    assert row["temperature_c"] == "18.25" and row["altitude_above_bottom_m"] == "2.50"
    assert row["telem_ms"] == "250.0"
    assert row["sync_skew_ms"] == "100.0"  # clock is 100 ms after capture
    assert [row[k] for k in ("vio_x", "vio_y", "vio_z", "vio_qw")] == ["1.0000", "-2.0000", "0.5000", "1.0000"]
    assert row["vio_tracking"] == "1"

    # No telemetry, pose or tracking: the columns are empty, never an error
    empty = dict(zip(CSV_HEADER, rows[1]))
    for key in ("lat", "lon", "heading_deg", "depth_m", "telem_ms", "vio_x", "vio_qz", "vio_tracking"):
        assert empty[key] == ""
    path = os.path.join(day(tmp_path), "center", empty["filename"])
    assert piexif.GPSIFD.GPSLatitude not in piexif.load(Path(path).read_bytes())["GPS"]


def test_stereo_pairs_by_sequence(tmp_path, make_writer):
    w = make_writer()
    w.set_recording(True)
    # 50 ms apart in time, but the same device sequence: still a pair
    w.submit(frame("left", t=T0, seq=7), TELEMETRY, POSE, True)
    w.submit(frame("right", t=T0 + 0.05, seq=7), TELEMETRY, POSE, True)
    w.close()

    left = jpgs(os.path.join(day(tmp_path), "stereo", "left"))
    right = jpgs(os.path.join(day(tmp_path), "stereo", "right"))
    assert left == right == ["000001_tl000000_00001.jpg"]
    rows = read_csv(os.path.join(day(tmp_path), "telemetry_stereo.csv"))
    assert len(rows) == 2 and rows[1][2] == left[0]
    assert w.snapshot()["counts"]["stereo"] == 1


def test_stereo_pairs_by_time_when_sequences_differ(tmp_path, make_writer):
    w = make_writer()
    w.set_recording(True)
    w.submit(frame("right", t=T0 + 0.010, seq=40), None, None, None)
    w.submit(frame("left", t=T0 + 0.200, seq=11), None, None, None)  # far in time: waits
    w.submit(frame("left", t=T0, seq=12), None, None, None)          # 10 ms from the right frame
    w.close()

    assert jpgs(os.path.join(day(tmp_path), "stereo", "left")) == ["000001_tl000000_00001.jpg"]
    assert jpgs(os.path.join(day(tmp_path), "stereo", "right")) == ["000001_tl000000_00001.jpg"]


def test_unpaired_frames_time_out(tmp_path, make_writer, clock):
    w = make_writer()
    w.set_recording(True)
    w.submit(frame("left", t=T0, seq=1), None, None, None)
    w.submit(frame("right", t=T0 + 0.05, seq=2), None, None, None)  # neither seq nor time match
    assert w.snapshot()["unpaired"] == 0

    clock.now = T0 + 2.5
    w.submit(frame("left", t=T0 + 2.4, seq=30), None, None, None)  # triggers the sweep
    snap = w.snapshot()
    assert snap["unpaired"] == 2
    assert snap["dropped"] == 2
    w.close()
    assert not os.path.exists(os.path.join(day(tmp_path), "stereo"))


def test_seq_continues_across_writer_instances(tmp_path, make_writer):
    first = make_writer()
    first.set_recording(True)
    first.submit(frame("center", seq=1), None, None, None)
    first.submit(frame("center", seq=2), None, None, None)
    first.close()

    second = make_writer(source_tag="tl123456")
    second.set_recording(True)
    second.set_waypoint(3)
    second.submit(frame("center", seq=1), None, None, None)
    second.close()

    names = jpgs(os.path.join(day(tmp_path), "center"))
    assert names == ["000001_tl000000_00001.jpg", "000002_tl000000_00002.jpg",
                     "000003_tl123456_3_00001.jpg"]


def test_labels_are_made_safe_for_file_names(tmp_path, make_writer):
    w = make_writer()
    w.set_source_tag("../dive 1")
    w.set_recording(True)
    w.submit(frame("center"), None, None, None)
    w.close()
    assert jpgs(os.path.join(day(tmp_path), "center")) == ["000001_---dive-1_00001.jpg"]


def test_recording_off_discards(tmp_path, make_writer):
    w = make_writer()
    w.submit(frame("center"), TELEMETRY, POSE, True)
    w.submit(frame("left"), TELEMETRY, POSE, True)
    w.submit(frame("right"), TELEMETRY, POSE, True)
    w.close()
    assert not os.path.exists(day(tmp_path))
    snap = w.snapshot()
    assert snap["recording"] is False and snap["dropped"] == 0
    assert snap["counts"] == {"center": 0, "stereo": 0}


def test_stopping_recording_mid_pair_discards_the_half_pair(tmp_path, make_writer):
    w = make_writer()
    w.set_recording(True)
    w.submit(frame("left", seq=1), None, None, None)
    w.set_recording(False)
    w.set_recording(True)
    w.submit(frame("right", seq=1), None, None, None)
    w.close()
    assert not os.path.exists(os.path.join(day(tmp_path), "stereo"))


def test_low_free_space_stops_writing(tmp_path, make_writer):
    w = make_writer(min_free_gb=1e9)  # more than any disk has
    w.set_recording(True)
    w.submit(frame("center"), TELEMETRY, POSE, True)
    w.close()

    assert jpgs(os.path.join(day(tmp_path), "center")) == []
    snap = w.snapshot()
    assert snap["disk_full"] is True
    assert snap["recording"] is False
    assert snap["free_gb"] is not None
    assert "disk nearly full" in snap["last_error"]


def test_enospc_removes_partial_pair(tmp_path, make_writer, monkeypatch):
    real_open = open

    def full_disk_open(path, mode="r", *args, **kwargs):
        if "right" in str(path) and "w" in mode:
            raise OSError(errno.ENOSPC, "No space left on device")
        return real_open(path, mode, *args, **kwargs)

    monkeypatch.setattr(writer_mod, "open", full_disk_open, raising=False)
    w = make_writer()
    w.set_recording(True)
    w.submit(frame("left", seq=1), None, None, None)
    w.submit(frame("right", seq=1), None, None, None)
    w.close()

    assert jpgs(os.path.join(day(tmp_path), "stereo", "left")) == []  # no half pair left behind
    snap = w.snapshot()
    assert snap["disk_full"] is True and snap["dropped"] == 1
    assert "No space left" in snap["last_error"]


def test_submit_never_blocks_and_close_drains(tmp_path, make_writer, hold_writes):
    w = make_writer()
    w.set_recording(True)
    for i in range(20):
        w.submit(frame("center", t=T0 + i * 0.1, seq=i), TELEMETRY, POSE, True)
    assert w.snapshot()["backlog"] > 0
    hold_writes.set()
    w.close(timeout=10.0)

    assert len(jpgs(os.path.join(day(tmp_path), "center"))) == 20
    assert w.snapshot()["counts"]["center"] == 20
    assert not any(t.is_alive() for t in w._threads)
    w.submit(frame("center", seq=99), None, None, None)  # after close: ignored
    assert len(jpgs(os.path.join(day(tmp_path), "center"))) == 20


def test_backlog_overflow_drops_instead_of_blocking(tmp_path, make_writer, hold_writes):
    w = make_writer()
    w.set_recording(True)
    extra = 10
    # One centre frame is held by the writer thread, MAX_BACKLOG wait, the rest are dropped
    for i in range(writer_mod.MAX_BACKLOG + 1 + extra):
        w.submit(frame("center", seq=i), None, None, None)
    snap = w.snapshot()
    assert snap["dropped"] >= extra
    assert "backlog" in snap["last_error"]


def test_close_times_out_and_counts_unwritten(tmp_path, make_writer, hold_writes):
    w = make_writer()
    w.set_recording(True)
    for i in range(5):
        w.submit(frame("center", seq=i), None, None, None)
    w.close(timeout=0.2)
    snap = w.snapshot()
    assert snap["dropped"] >= 1
    assert "not written" in snap["last_error"]
