"""Transport contract: how TrackShot hands MAVLink messages to the autopilot.

Messages are given as (type name, fields dict) using **pymavlink field names** — mavlink2rest's
JSON uses the same names. Arrays are Python lists of numbers. `char[N]` fields (e.g.
NAMED_VALUE_FLOAT.name) are given as a plain Python `str`; each transport encodes it the way its
wire format needs.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass
class TransportStats:
    kind: str                       # "mavlink2rest" or "udp"
    target: str                     # URL / endpoint string
    connected: bool = False
    sent: int = 0
    dropped: int = 0                # queue overflow or not connected
    errors: int = 0
    last_error: Optional[str] = None
    heartbeat_age_s: Optional[float] = None   # time since the autopilot's last HEARTBEAT, if the transport sees it


class Transport(ABC):
    def __init__(self, source_system: int, source_component: int):
        self.source_system = source_system
        self.source_component = source_component

    @abstractmethod
    def start(self) -> None:
        """Connect in the background (non-blocking) and keep reconnecting until stop()."""

    @abstractmethod
    def stop(self) -> None:
        """Close. Safe to call twice."""

    @abstractmethod
    def send(self, msg_type: str, fields: dict) -> bool:
        """Queue one message. Never blocks. Returns False if it was dropped."""

    @abstractmethod
    def stats(self) -> TransportStats: ...
