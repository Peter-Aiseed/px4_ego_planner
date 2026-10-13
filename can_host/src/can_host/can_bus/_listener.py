"""CAN bus listener that feeds python-can messages onto the EventBus."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable

import can

from ..events import CANFrame

logger = logging.getLogger(__name__)


class FrameListener(can.Listener):
    """Adapts ``can.Message`` objects into :class:`CANFrame` events.

    Pass an instance to ``can.Notifier``::

        notifier = can.Notifier(bus, [FrameListener(bus._publish_frame)])

    ``publish`` is called with each converted frame. Reader errors are logged
    here; publish errors don't need guarding because :class:`EventBus` already
    catches subscriber exceptions.
    """

    def __init__(self, publish: Callable[[CANFrame], None]) -> None:
        self._publish = publish

    def on_message_received(self, msg: can.Message) -> None:
        self._publish(
            CANFrame(
                arbitration_id=msg.arbitration_id,
                dlc=msg.dlc,
                data=bytes(msg.data),
                is_extended_id=bool(msg.is_extended_id),
                is_remote_frame=bool(msg.is_remote_frame),
                timestamp=msg.timestamp or time.time(),
            )
        )

    def on_error(self, exc: Exception) -> None:
        logger.error("reader error: %r", exc)