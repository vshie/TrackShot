"""Run blocking calls (HTTP to the autopilot, camera discovery) off the GUI thread.

`run_in_pool(pool, fn, on_success, on_error)` runs `fn()` on a QThreadPool thread and calls one of
the callbacks back **on the GUI thread** with its result or exception. The hand-off goes through a
small QObject created on the GUI thread: its signals are connected to its own slots with a queued
connection, so delivery always happens in the thread that object lives in, whatever kind of
callable the caller passed.
"""

from __future__ import annotations

from typing import Any, Callable

from loguru import logger
from PySide6.QtCore import QObject, Qt, QThreadPool, Signal, Slot

# Relays waiting for their task; holding them here keeps Python from collecting them meanwhile.
_PENDING: set["_Relay"] = set()


class _Relay(QObject):
    _succeeded = Signal(object)
    _failed = Signal(object)

    def __init__(self, on_success: Callable[[Any], None], on_error: Callable[[BaseException], None]):
        super().__init__()
        self._on_success = on_success
        self._on_error = on_error
        self._succeeded.connect(self._deliver_success, Qt.ConnectionType.QueuedConnection)
        self._failed.connect(self._deliver_error, Qt.ConnectionType.QueuedConnection)

    @Slot(object)
    def _deliver_success(self, result: Any) -> None:
        self._finish(self._on_success, result)

    @Slot(object)
    def _deliver_error(self, error: BaseException) -> None:
        self._finish(self._on_error, error)

    def _finish(self, callback: Callable[[Any], None], arg: Any) -> None:
        _PENDING.discard(self)
        self.deleteLater()
        try:
            callback(arg)
        except RuntimeError as e:
            # The widget that asked was closed while the task ran: nothing left to update.
            logger.debug(f"Dropped a background result: {e}")


def run_in_pool(pool: QThreadPool, fn: Callable[[], Any],
                on_success: Callable[[Any], None],
                on_error: Callable[[BaseException], None]) -> None:
    relay = _Relay(on_success, on_error)
    _PENDING.add(relay)

    def task() -> None:
        try:
            result = fn()
        except Exception as e:  # handed to the GUI to display, never raised on a pool thread
            relay._failed.emit(e)
        else:
            relay._succeeded.emit(result)

    pool.start(task)
