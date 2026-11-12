"""Survey image capture alongside VIO.

Port of the BlueOS extension's ``app/capture_node.py``, decoupled from depthai: images arrive
as `CapturedJpeg` (already JPEG-encoded on the OAK, stamped with host wall-clock capture time),
so this module only labels, tags and writes them. Every image is tagged from the autopilot
telemetry snapshot and the VIO pose handed in with it.

Layout (naming follows SubReels TowFish's survey layout, same as the extension):

    captures_dir/survey_YYYYMMDD/
        center/{seq:06d}_{source_tag}[_{wp}]_{frame:05d}.jpg
        stereo/left/...   stereo/right/...   (identical names for each synced pair)
        telemetry_center.csv
        telemetry_stereo.csv

`submit()` is called from the pose source's threads, so it only pairs frames and queues a job;
the disk work (EXIF/XMP embedding, file and CSV writes) happens on the writer's own threads, one
per stream. (The extension used a shared 2-thread pool, which could number two centre frames out
of capture order; a FIFO per stream keeps the numbering in order and still writes centre and
stereo in parallel.)
"""

from __future__ import annotations

import csv
import errno
import os
import queue
import re
import shutil
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Optional

from loguru import logger

from trackshot.capture import meta
from trackshot.vio.source import CapturedJpeg

CSV_HEADER = [
    'timestamp', 'seq', 'filename', 'source_tag', 'wp', 'frame', 'size_bytes',
    'lat', 'lon', 'heading_deg', 'roll_deg', 'pitch_deg', 'depth_m', 'temperature_c',
    'telem_ms', 'sync_skew_ms', 'altitude_above_bottom_m',
    'vio_x', 'vio_y', 'vio_z', 'vio_qw', 'vio_qx', 'vio_qy', 'vio_qz', 'vio_tracking',
]

DEFAULT_SOURCE_TAG = "tl000000"

# Left/right frames from the synced OV9282 pair land within a few ms of each other
STEREO_PAIR_TOLERANCE_S = 0.015
STEREO_PENDING_MAX_AGE_S = 2.0

# Jobs waiting per stream. A disk that can't keep up must cost frames, not memory:
# 64 jobs is several seconds of capture at the rates the tether allows.
MAX_BACKLOG = 64

_SEQ_RE = re.compile(r"^(\d{6})_")
# Labels end up in file names: anything but letters, digits and '-' could change the path
_UNSAFE_LABEL_RE = re.compile(r"[^A-Za-z0-9-]")


def survey_day_folder(captures_dir, now=None):
    return os.path.join(captures_dir, f"survey_{(now or datetime.now()):%Y%m%d}")


def scan_max_seq(folder):
    """Highest NNNNNN_ prefix in `folder`, so numbering continues across restarts within a day."""
    highest = 0
    try:
        for name in os.listdir(folder):
            match = _SEQ_RE.match(name)
            if match:
                highest = max(highest, int(match.group(1)))
    except FileNotFoundError:
        pass
    return highest


def make_filename(seq, source_tag, wp, frame):
    if wp:
        return f"{seq:06d}_{source_tag}_{wp}_{frame:05d}.jpg"
    return f"{seq:06d}_{source_tag}_{frame:05d}.jpg"


def safe_label(value) -> str:
    """`value` made safe for a file name ('/' or spaces from a typed tag become '-')."""
    return _UNSAFE_LABEL_RE.sub("-", str(value))


def _fmt(value, spec):
    return format(value, spec) if value is not None else ""


class StreamWriter:
    """Numbering for one stream ("center" or "stereo"). Used only by that stream's writer thread."""

    def __init__(self, name, subdirs):
        self.name = name
        self.subdirs = subdirs  # e.g. ["center"] or ["stereo/left", "stereo/right"]
        self.day_folder = None
        self.seq = 0
        self.frame = 0
        self.label = None  # (source_tag, wp) the frame counter belongs to
        self.session_count = 0

    def claim(self, day_folder, source_tag, wp):
        """Reserves (seq, frame) for the next image."""
        if day_folder != self.day_folder:
            self.day_folder = day_folder
            self.seq = scan_max_seq(os.path.join(day_folder, self.subdirs[0]))
        if (source_tag, wp) != self.label:
            if self.label is None or self.label[0] != source_tag:
                self.session_count = 0
            self.label = (source_tag, wp)
            self.frame = 0
        self.seq += 1
        self.frame += 1
        self.session_count += 1
        return self.seq, self.frame


class CaptureWriter:
    """Writes survey images and their telemetry CSVs on its own daemon threads.

    Thread-safe: `submit` may be called from any thread and never blocks; the setters and
    `snapshot` are cheap enough for the GUI thread.
    """

    def __init__(self, captures_dir, camera_angle_deg=0.0, min_free_gb=2.0,
                 source_tag=DEFAULT_SOURCE_TAG, *, clock: Callable[[], float] = time.time):
        self.captures_dir = str(captures_dir)
        self.camera_angle_deg = camera_angle_deg
        self.min_free_bytes = min_free_gb * 1e9
        self._clock = clock  # wall clock, injectable so tests can age frames

        self._writers = {
            "center": StreamWriter("center", ["center"]),
            "stereo": StreamWriter("stereo", ["stereo/left", "stereo/right"]),
        }

        # Guards everything below: control state, unpaired stereo frames and stats
        self._lock = threading.Lock()
        self._recording = False
        self._closed = False
        self._disk_full = False
        self._source_tag = DEFAULT_SOURCE_TAG
        self._wp: Optional[str] = None
        self._pending: dict[str, list[CapturedJpeg]] = {"left": [], "right": []}
        self._dropped = 0
        self._unpaired = 0
        self._free_gb: Optional[float] = None
        self._last_error: Optional[str] = None
        self._last_file: Optional[str] = None
        self.set_source_tag(source_tag)

        # Jobs queued or running; close() waits for this to reach zero
        self._idle = threading.Condition()
        self._outstanding = 0
        self._jobs = {name: queue.Queue(maxsize=MAX_BACKLOG) for name in self._writers}
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._worker, args=(jobs,), name=f"capture-{name}",
                                          daemon=True)
                         for name, jobs in self._jobs.items()]
        for thread in self._threads:
            thread.start()

    # -- control ------------------------------------------------------------
    @property
    def recording(self) -> bool:
        with self._lock:
            return self._recording and not self._disk_full

    def set_recording(self, on: bool) -> None:
        with self._lock:
            if self._closed:
                return
            self._recording = bool(on)
            # Either way the disk-full latch is cleared, as the extension did on stop: pressing
            # record again re-checks the space (the user may have freed some).
            self._disk_full = False
            if not on:
                # Half-pairs from the moment recording stopped aren't lost survey data
                self._pending = {"left": [], "right": []}
        if on:
            # Check the disk now, off the caller's thread, so the UI learns at once if it is full
            self._enqueue("center", self._check_space)

    def set_source_tag(self, tag) -> None:
        with self._lock:
            self._source_tag = safe_label(tag) if tag else DEFAULT_SOURCE_TAG

    def set_waypoint(self, wp) -> None:
        with self._lock:
            self._wp = safe_label(wp) if wp not in (None, "") else None

    # -- intake -------------------------------------------------------------
    def submit(self, jpeg: CapturedJpeg, telemetry: dict | None, pose: tuple | None,
               tracking: bool | None) -> None:
        """Queue one image for writing. Never blocks; discarded at once while not recording.

        `telemetry` is a `VehicleTelemetry.latest()` dict (None = no telemetry), `pose` is
        (x, y, z, qw, qx, qy, qz), `tracking` the VIO's tracking flag.
        """
        with self._lock:
            if self._closed or not self._recording or self._disk_full:
                return
            # Snapshot the labels now, so a change can't land half-way through this frame
            label = (self._source_tag, self._wp)
            if jpeg.stream == "center":
                stream, frames = "center", [jpeg]
            elif jpeg.stream in ("left", "right"):
                stream, frames = "stereo", self._pair(jpeg)
                if frames is None:
                    return
            else:
                logger.warning(f"Capture: ignoring frame from unknown stream {jpeg.stream!r}")
                return
        self._enqueue(stream, self._write_job, stream, frames, telemetry, pose, tracking, label)

    def _pair(self, frame: CapturedJpeg) -> Optional[list[CapturedJpeg]]:
        """[left, right] once `frame`'s partner is here, else None (the frame waits). Lock held.

        Partners share the device sequence number; capture time within 15 ms is the fallback for
        when the two streams' numbering drifts apart. Frames waiting longer than 2 s have lost
        their partner (dropped by the camera or the tether) and are counted as unpaired.
        """
        now = self._clock()
        for side, frames in self._pending.items():
            fresh = [f for f in frames if now - f.t_capture < STEREO_PENDING_MAX_AGE_S]
            stale = len(frames) - len(fresh)
            if stale:
                self._unpaired += stale
                self._dropped += stale
                self._pending[side] = fresh

        others = self._pending["right" if frame.stream == "left" else "left"]
        match = next((f for f in others if f.sequence == frame.sequence), None)
        if match is None:
            near = [f for f in others if abs(f.t_capture - frame.t_capture) <= STEREO_PAIR_TOLERANCE_S]
            match = min(near, key=lambda f: abs(f.t_capture - frame.t_capture), default=None)
        if match is None:
            self._pending[frame.stream].append(frame)
            return None
        others.remove(match)
        return [frame, match] if frame.stream == "left" else [match, frame]

    # -- writer threads -----------------------------------------------------
    def _enqueue(self, stream, fn, *args) -> None:
        with self._idle:
            self._outstanding += 1
        try:
            self._jobs[stream].put_nowait((fn, args))
        except queue.Full:
            self._job_done()
            self._count_drop("writer backlog full: the disk can't keep up")

    def _job_done(self) -> None:
        with self._idle:
            self._outstanding -= 1
            self._idle.notify_all()

    def _worker(self, jobs: queue.Queue) -> None:
        while not self._stop.is_set():
            try:
                job = jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            if job is None:  # close()'s wake-up
                break
            fn, args = job
            try:
                fn(*args)  # jobs report their own errors
            finally:
                self._job_done()

    def _count_drop(self, error: Optional[str]) -> None:
        with self._lock:
            self._dropped += 1
            if error:
                self._last_error = error

    def _check_space(self) -> bool:
        try:
            os.makedirs(self.captures_dir, exist_ok=True)
            free = shutil.disk_usage(self.captures_dir).free
        except OSError:
            return True  # can't tell: let the write itself report any problem
        with self._lock:
            self._free_gb = free / 1e9
            if free < self.min_free_bytes:
                if not self._disk_full:
                    logger.error(f"Capture stopped: only {self._free_gb:.1f} GB free")
                    self._last_error = f"disk nearly full ({self._free_gb:.1f} GB free)"
                self._disk_full = True
                return False
        return True

    def _write_job(self, stream, frames, telemetry, pose, tracking, label) -> None:
        """Runs on a writer thread: label, stamp and write one image (or stereo pair)."""
        try:
            if not self._check_space():
                self._count_drop(None)  # the space check already said why
                return
            first = frames[0]
            ts_local = datetime.fromtimestamp(first.t_capture)
            ts_utc = datetime.fromtimestamp(first.t_capture, timezone.utc)
            t = telemetry or {}
            age_s = t.get("age_s")
            telem_ms = age_s * 1000.0 if age_s is not None else None
            vio_tracking = (1.0 if tracking else 0.0) if tracking is not None else None

            source_tag, wp = label
            day_folder = survey_day_folder(self.captures_dir, ts_local)
            writer = self._writers[stream]
            seq, frame_no = writer.claim(day_folder, source_tag, wp)
            filename = make_filename(seq, source_tag, wp, frame_no)
            sync_skew_ms = (self._clock() - first.t_capture) * 1000.0

            total_bytes = 0
            written = []
            for subdir, frame in zip(writer.subdirs, frames):
                jpeg = meta.embed_metadata(
                    frame.data, telemetry, ts_local, ts_utc, lens=frame.lens,
                    camera_angle_deg=self.camera_angle_deg, wp=wp, vio_tracking=vio_tracking)
                folder = os.path.join(day_folder, subdir)
                os.makedirs(folder, exist_ok=True)
                path = os.path.join(folder, filename)
                try:
                    with open(path, "wb") as f:
                        f.write(jpeg)
                except OSError as e:
                    # A frame cut short by a full disk would trip up the photogrammetry run
                    if e.errno == errno.ENOSPC:
                        with self._lock:
                            self._disk_full = True
                        for p in written + [path]:
                            try:
                                os.unlink(p)
                            except OSError:
                                pass
                    raise
                written.append(path)
                total_bytes += len(jpeg)

            csv_path = os.path.join(day_folder, f"telemetry_{stream}.csv")
            p = pose or (None,) * 7
            new_file = not os.path.exists(csv_path)
            with open(csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(CSV_HEADER)
                w.writerow([
                    ts_local.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], seq, filename, source_tag,
                    wp or "", frame_no, total_bytes,
                    _fmt(t.get("lat"), ".7f"), _fmt(t.get("lon"), ".7f"),
                    _fmt(t.get("heading_deg"), ".1f"), _fmt(t.get("roll_deg"), ".1f"),
                    _fmt(t.get("pitch_deg"), ".1f"), _fmt(t.get("depth_m"), ".2f"),
                    _fmt(t.get("temperature_c"), ".2f"), _fmt(telem_ms, ".1f"),
                    f"{sync_skew_ms:.1f}", _fmt(t.get("altitude_m"), ".2f"),
                    *[_fmt(v, ".4f") for v in p],
                    _fmt(vio_tracking, ".0f"),
                ])
            with self._lock:
                self._last_file = written[0]
        except Exception as e:
            logger.exception(f"Capture write failed ({stream})")
            self._count_drop(str(e))

    # -- status / shutdown --------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            folder = (self._writers["center"].day_folder or self._writers["stereo"].day_folder
                      or survey_day_folder(self.captures_dir))
            return {
                "recording": self._recording and not self._disk_full,
                "folder": folder,
                "counts": {name: w.session_count for name, w in self._writers.items()},
                "dropped": self._dropped,
                "unpaired": self._unpaired,
                "backlog": sum(jobs.qsize() for jobs in self._jobs.values()),
                "free_gb": round(self._free_gb, 2) if self._free_gb is not None else None,
                "disk_full": self._disk_full,
                "last_error": self._last_error,
                "last_file": self._last_file,
                "source_tag": self._source_tag,
                "wp": self._wp,
            }

    def close(self, timeout: float = 5.0) -> None:
        """Stop taking frames, write what is queued (up to `timeout` s), stop the threads.

        Safe to call twice: the second call returns at once.
        """
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._recording = False
            self._pending = {"left": [], "right": []}
        deadline = time.monotonic() + timeout
        with self._idle:
            drained = self._idle.wait_for(lambda: self._outstanding == 0, timeout)
            abandoned = self._outstanding
        self._stop.set()
        for jobs in self._jobs.values():
            try:
                jobs.put_nowait(None)  # wake an idle thread now rather than at its next poll
            except queue.Full:
                pass  # timed out with a full queue: the thread exits after its current job
        for thread in self._threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        if not drained:
            logger.warning(f"Capture: {abandoned} image(s) not written before close timed out")
            with self._lock:
                self._dropped += abandoned
                self._last_error = f"{abandoned} image(s) not written before shutdown"
