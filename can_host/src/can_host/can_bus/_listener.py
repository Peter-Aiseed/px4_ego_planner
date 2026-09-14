"""CAN bus listeners for python-can Notifier-based backends."""

from __future__ import annotations

import time
from typing import Callable, Optional

import can

from ..events import CANFrame


class FrameListener(can.Listener):
    """Turns ``can.Message`` objects into :class:`CANFrame` events.

    Construct with a callable that accepts the published frame, then pass
    an instance to ``can.Notifier``::

        notifier = can.Notifier(bus, [FrameListener(self._publish_frame)])

    An optional ``status_cb`` receives reader-error notifications so the
    backend can re-emit them as ``StatusEvent``::

        FrameListener(self._publish_frame, status_cb=self._emit_status)
    """

    def __init__(
        self,
        publish: Callable[[CANFrame], None],
        status_cb: Optional[Callable[[str, str], None]] = None,
    ) -> None:
        self._publish = publish
        self._status = status_cb

    def on_message_received(self, msg: can.Message) -> None:
        frame = CANFrame(
            arbitration_id=msg.arbitration_id,
            dlc=msg.dlc,
            data=bytes(msg.data),
            is_extended_id=bool(msg.is_extended_id),
            is_remote_frame=bool(msg.is_remote_frame),
            timestamp=msg.timestamp or time.time(),
        )
        try:
            self._publish(frame)
        except Exception as exc:  # noqa: BLE001
            if self._status:
                self._status("error", f"publish callback raised: {exc!r}")

    def on_error(self, exc: Exception) -> None:
        if self._status:
            self._status("error", f"reader error: {exc!r}")
