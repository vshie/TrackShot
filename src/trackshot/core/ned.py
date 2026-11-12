"""Horizontal-only VIO output: VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE in the EKF's NED frame.

Why not VISION_POSITION_DELTA: ArduSub's EKF3 fuses a delta as a body-frame velocity on all three
axes (NavEKF3_core::FuseBodyVel loops over x, y and z, one noise for all), whatever
EK3_SRC1_VELZ says. So the camera's vertical motion competes with the depth sensor. There is no
way to leave an axis out of a delta, and a zero there would assert "no vertical motion".

ExtNav velocity (VISION_SPEED_ESTIMATE) is fused per axis: north/east when EK3_SRC1_VELXY is
ExtNav (6), down only when EK3_SRC1_VELZ is ExtNav too. With VELZ 0 the camera never touches
the vertical. Velocity alone doesn't let the EKF start horizontal aiding, though (relative aiding
needs body odometry or optical flow, absolute aiding needs a position source), so the VIO's
position goes too as VISION_POSITION_ESTIMATE with EK3_SRC1_POSXY 6; its z is ignored while
EK3_SRC1_POSZ is the barometer (1). All of this is ArduSub 4.7.2 (NavEKF3_PosVelFusion.cpp,
NavEKF3_Control.cpp).

For VISO_TYPE 1 (MAVLink) ArduSub applies no rotation or heading alignment to either message: they
must already be NED in the EKF's frame. The camera is fixed to the vehicle, so the odometry's
deltas are already in the vehicle's body frame (see odometry.apply_camera_rotation); this module
rotates each one into NED by the autopilot's own attitude (ATTITUDE roll, pitch and compass
heading). Nothing is learned. Telemetry arrives at ~2 Hz, so between samples the heading is
carried forward by how much the VIO says the vehicle has turned since that sample.

Position is integrated here from the horizontal motion. After a gap (tracking lost, a stall, the
start) it is re-anchored on the EKF's own local position, so the EKF sees no jump.
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional

from trackshot.core.odometry import DeltaMessage

Vec3 = tuple[float, float, float]

# The autopilot's attitude is used up to this old (mavlink2rest stalls 1–2 s now and then); older
# than that, nothing is sent
ATTITUDE_MAX_AGE_S = 3.0
# How far apart the attitude sample and a VIO yaw may be to carry the heading forward with the VIO
HEADING_MATCH_S = 0.3
# VIO yaw kept for carrying the heading forward
YAW_HISTORY_S = 5.0
# A gap between deltas longer than this re-anchors the position on the EKF's
ANCHOR_AFTER_GAP_S = 1.0


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def body_to_ned(v: Vec3, roll: float, pitch: float, yaw: float) -> Vec3:
    """FRD body vector → NED, by the ZYX (yaw, pitch, roll) Euler angles ArduPilot reports."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = v
    return (
        cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        -sp * x + cp * sr * y + cp * cr * z,
    )


@dataclass(frozen=True)
class NedEstimate:
    usec: int                    # the pose's capture time, µs (ArduPilot jitter-corrects it)
    position: Vec3               # NED from the EKF origin, m; z is 0 (the EKF takes depth from the baro)
    velocity: Vec3               # NED, m/s; down is always 0 (not fused with EK3_SRC1_VELZ 0)
    yaw: float                   # the compass heading used, rad (unused unless EK3_SRC1_YAW is 6)
    anchored: bool               # this estimate re-anchored on the EKF's position


class NedTracker:
    """Turns the odometry's vehicle-frame deltas into NED position and velocity. Thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._attitude: Optional[tuple[float, float, float, float]] = None   # roll, pitch, yaw, t
        self._yaw_history: deque[tuple[float, float]] = deque()   # (t s, VIO yaw FRD, unwrapped)
        self._north = 0.0
        self._east = 0.0
        self._last_t: Optional[float] = None
        self.last_heading: Optional[float] = None       # rad, the heading the last estimate used

    def observe_attitude(self, roll: Optional[float], pitch: Optional[float], yaw: Optional[float],
                         t: Optional[float]) -> None:
        """The autopilot's attitude (ATTITUDE, rad) valid at wall time `t`."""
        if None in (roll, pitch, yaw, t) or not all(math.isfinite(v) for v in (roll, pitch, yaw)):
            return
        with self._lock:
            if self._attitude is None or t > self._attitude[3]:
                self._attitude = (float(roll), float(pitch), float(yaw), float(t))

    def _vio_yaw_at(self, t: float) -> Optional[float]:
        h = self._yaw_history
        if not h:
            return None
        nearest = min(h, key=lambda item: abs(item[0] - t))
        return nearest[1] if abs(nearest[0] - t) <= HEADING_MATCH_S else None

    def update(self, d: DeltaMessage, ekf_north_east: Optional[tuple[float, float]] = None) -> Optional[NedEstimate]:
        """One odometry delta → the NED estimate to send, or None without a recent attitude.

        `ekf_north_east`: the EKF's current local position (LOCAL_POSITION_NED x, y), used to
        re-anchor after a gap; None if not known."""
        if d.stamp_us is None:
            return None
        t = d.stamp_us / 1e6
        with self._lock:
            vio_yaw = None
            if d.vio_yaw is not None:
                vio_yaw = -d.vio_yaw              # FLU yaw is counter-clockwise; NED/FRD clockwise
                h = self._yaw_history
                if h:
                    vio_yaw = h[-1][1] + wrap_pi(vio_yaw - h[-1][1])    # unwrapped
                h.append((t, vio_yaw))
                while h and t - h[0][0] > YAW_HISTORY_S:
                    h.popleft()
            if self._attitude is None or t - self._attitude[3] > ATTITUDE_MAX_AGE_S:
                return None
            roll, pitch, heading, t_att = self._attitude
            # Carry the compass heading forward by the VIO's turn since the sample (not an offset:
            # only the change over the last fraction of a second)
            then = self._vio_yaw_at(t_att)
            if vio_yaw is not None and then is not None:
                heading = wrap_pi(heading + (vio_yaw - then))
            self.last_heading = heading

            anchored = self._last_t is None or t - self._last_t > ANCHOR_AFTER_GAP_S
            if anchored and ekf_north_east is not None:
                self._north, self._east = float(ekf_north_east[0]), float(ekf_north_east[1])
            self._last_t = t
            dn, de, _ = body_to_ned(d.position_delta, roll, pitch, heading)
            self._north += dn
            self._east += de
            span_s = d.time_delta_usec / 1e6
            return NedEstimate(
                usec=int(d.stamp_us),
                position=(self._north, self._east, 0.0),
                velocity=(dn / span_s, de / span_s, 0.0),
                yaw=heading,
                anchored=anchored,
            )
