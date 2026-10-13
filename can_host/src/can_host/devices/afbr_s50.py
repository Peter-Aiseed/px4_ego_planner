"""AFBR-S50 ToF device.

Parses the 8-byte 1D measurement frames (default IDs 0x18-0x1F) the sensor
streams on CAN and emits :class:`AFBRMeasurement` events.  The Start / Stop
Remote Frames at 0x08 / 0x09 control whether the sensor measures at all.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from apscheduler.jobstores.base import JobLookupError

from ..events import CANFrame, EventBus
from ..scheduler import schedule_interval
from ._afbr_parser import AFBRParser
from .base import Device
from .factory import register_device

if TYPE_CHECKING:
    from apscheduler.schedulers.base import BaseScheduler

logger = logging.getLogger(__name__)

#: Seconds between the Remote Frames of a Start / Stop burst.
REMOTE_FRAME_GAP = 0.1

#: Default seconds between Start Remote Frame keepalive re-sends.  Positive by
#: default so a sensor that resets or loses its stream starts streaming again
#: on its own; 0 disables the keepalive and sends Start only at startup.
DEFAULT_START_INTERVAL = 5.0

#: Logical name of the keepalive job, in log lines and on the scheduler.
START_KEEPALIVE = "start-keepalive"


@register_device("afbr-s50")
class AFBR_S50(Device):
    """AFBR-S50 ToF sensor driver.

    Decodes the 1D measurement frames arriving on *can_ids* and emits one
    :class:`~can_host.devices.measurements.AFBRMeasurement` per frame, tagged
    with the CAN ID it came from.

    :meth:`start` sends a burst of ``send_start`` Start Remote Frames (0x08)
    and, when ``start_interval`` is positive, keeps re-sending a single Start
    frame on that interval.  The keepalive restarts the measurement cycle on a
    sensor that rebooted or dropped its stream, so the host does not have to be
    cycled.  :meth:`stop` tears the keepalive down first and then sends
    ``send_stop`` Stop Remote Frames (0x09), which always all go out.
    """

    KIND = "afbr"

    def __init__(
        self,
        device_id: str = "afbr-s50",
        can_ids: list[int] | None = None,
        auto_install_filters: bool = True,
        event_bus: EventBus | None = None,
        send_start: int = 1,
        send_stop: int = 1,
        start_interval: float = DEFAULT_START_INTERVAL,
        scheduler: BaseScheduler | None = None,
        **kwargs: Any,
    ) -> None:
        """
        Args:
            can_ids: Arbitration IDs accepted as 1D measurement data.  Defaults
                to the sensor's built-in range (0x18-0x1F), which
                :func:`can_host.main` narrows to ``--can-ids``.
            auto_install_filters: Ask the bus for hardware acceptance filters so
                only these IDs reach this process.  Turn it off when several
                devices share one bus and each would otherwise overwrite the
                other's filters.
            send_start: Start Remote Frames sent by :meth:`start`; 0 disables.
            send_stop: Stop Remote Frames sent by :meth:`stop`; 0 disables.
            start_interval: Seconds between Start keepalive re-sends; 0 disables
                the keepalive.
            scheduler: Scheduler for the keepalive; the host passes its own.
        """
        super().__init__(
            device_id=device_id, event_bus=event_bus, scheduler=scheduler
        )
        self._can_ids = (
            list(can_ids) if can_ids is not None else list(AFBRParser.FRAME_ID_1D)
        )
        self._auto_install_filters = auto_install_filters
        self._send_start = send_start
        self._send_stop = send_stop
        self._start_interval = start_interval

    # ---- wiring --------------------------------------------------------

    def _on_attached(self) -> None:
        """Narrow the bus to this sensor's measurement IDs, when asked to."""
        if not self._auto_install_filters or self._bus is None:
            return
        filters = [{"can_id": cid, "can_mask": 0x7FF} for cid in self._can_ids]
        try:
            self._bus.set_filters(filters)
        except RuntimeError as exc:
            logger.warning("filter failed: %s", exc)
            return
        logger.info(
            "filters installed: can_ids=%s",
            [f"0x{c:02X}" for c in self._can_ids],
        )

    # ---- lifecycle -----------------------------------------------------

    @property
    def _keepalive_job_id(self) -> str:
        """Job id of this device's Start keepalive, unique per device instance."""
        return f"{self.device_id}:{START_KEEPALIVE}"

    def start(self) -> None:
        """Send the Start burst, then arm the Start keepalive."""
        super().start()
        if self._send_start > 0:
            self._send_remote_burst(
                AFBRParser.FRAME_ID_START,
                self._send_start,
                gap=REMOTE_FRAME_GAP,
                label="start",
            )
        if self._start_interval > 0:
            schedule_interval(
                self.scheduler,
                self._send_start_frame,
                job_id=self._keepalive_job_id,
                seconds=self._start_interval,
            )
            logger.info(
                "device %s: %s every %.3fs",
                self.device_id,
                START_KEEPALIVE,
                self._start_interval,
            )

    def stop(self) -> None:
        """Drop the keepalive, then send the Stop burst."""
        self._remove_keepalive()
        super().stop()
        if self._send_stop > 0:
            self._send_remote_burst(
                AFBRParser.FRAME_ID_STOP,
                self._send_stop,
                gap=REMOTE_FRAME_GAP,
                label="stop",
                interruptible=False,
            )

    def detach(self) -> None:
        """Remove the keepalive even when :meth:`stop` was skipped, then unwire."""
        self._remove_keepalive()
        super().detach()

    def _remove_keepalive(self) -> None:
        """Remove the Start keepalive job; absent jobs are a no-op."""
        scheduler = self._scheduler
        if scheduler is None:
            return
        try:
            scheduler.remove_job(self._keepalive_job_id)
        except JobLookupError:
            logger.debug("device %s: %s already removed", self.device_id, START_KEEPALIVE)

    def _send_start_frame(self) -> None:
        """Re-send a single Start Remote Frame; one keepalive tick."""
        self._send_remote_frame(AFBRParser.FRAME_ID_START, label=START_KEEPALIVE)

    # ---- frame handling ------------------------------------------------

    def _is_measurement_frame(self, frame: CANFrame) -> bool:
        """True when *frame* is a 1D measurement this sensor is configured for."""
        return (
            not frame.is_remote_frame
            and frame.arbitration_id in self._can_ids
            and frame.dlc == AFBRParser.EXPECTED_DLC
        )

    def _on_frame(self, frame: CANFrame) -> None:
        if not self._is_measurement_frame(frame):
            return
        meas = AFBRParser.parse_1d_frame(frame.data)
        if meas is None:
            return
        meas.id = frame.arbitration_id
        self._emit_measurement(meas, frame)
