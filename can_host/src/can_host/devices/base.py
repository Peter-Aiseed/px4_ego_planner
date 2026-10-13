"""Abstract device base class.

A device:

* subscribes to ``frame`` events on the shared event bus,
* parses the frames that match its protocol into measurements,
* emits ``measurement`` and ``status`` events.

Lifecycle
---------

``attach()`` wires the device to a bus, ``start()`` / ``stop()`` bracket the
run phase in which the device talks to its hardware, and ``detach()`` unwires
it again::

    device.attach(bus)     # subscribe to frames
    device.start()         # tell the sensor to start; arm its own jobs
    ...
    device.stop()          # tell the sensor to stop; drop its own jobs
    device.detach()        # unsubscribe and release owned resources

``start()`` and ``stop()`` are concrete template methods: an override must
call ``super()`` so the run flag and the shutdown latch stay consistent, and
``stop()`` should call ``super()`` *first* so the device's own shutdown
traffic cannot race.

Periodic work is the device's own business, not the base class's: the base
only injects a shared :attr:`scheduler` (the host's ``BackgroundScheduler``)
so every actor shares one thread pool, and each device schedules the jobs it
needs directly through ``schedule_interval(self.scheduler, ...)`` and is
responsible for removing them again.  The base class also owns the one
convenience most CAN devices need: Remote Frame bursts
(:meth:`_send_remote_burst`).
"""

from __future__ import annotations

import abc
import logging
import threading
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, ClassVar

from ..events import CANFrame, DeviceMeasurement, EventBus, StatusEvent
from ..scheduler import create_scheduler
from .measurements import Measurement

if TYPE_CHECKING:
    from apscheduler.schedulers.base import BaseScheduler

    from ..can_bus.base import CANBus

logger = logging.getLogger(__name__)


class Device(abc.ABC):
    EVENT_MEASUREMENT = "measurement"
    EVENT_STATUS = "status"

    #: Family discriminator stamped on every measurement this device emits
    #: ("afbr", "uavcan", …) so consumers dispatch without isinstance checks.
    KIND: ClassVar[str] = "device"

    def __init__(
        self,
        device_id: str,
        event_bus: EventBus | None = None,
        scheduler: BaseScheduler | None = None,
    ) -> None:
        """
        Args:
            device_id: Identifier reported with every event from this device.
            event_bus: Shared event bus; the host always supplies one.
            scheduler: Scheduler for this device's periodic jobs. The host
                passes its own so every job shares one thread pool; when
                omitted the device creates — and later shuts down — a private
                one, which is what standalone use and tests want.
        """
        self.device_id = device_id
        self.events: EventBus = event_bus  # type: ignore[assignment]
        self._bus: CANBus | None = None
        self._unsubscribe: Callable[[], None] | None = None
        self._shutdown = threading.Event()
        self._running = False
        self._scheduler = scheduler
        self._owns_scheduler = scheduler is None

    @property
    def is_running(self) -> bool:
        """True between :meth:`start` and :meth:`stop`."""
        return self._running

    # ---- wiring --------------------------------------------------------

    def attach(self, bus: CANBus) -> None:
        """Subscribe to the bus's frame events.  Idempotent."""
        if self._unsubscribe is not None:
            return
        self._bus = bus
        self._unsubscribe = bus.events.subscribe(bus.EVENT_FRAME, self._on_frame)
        self._on_attached()

    def detach(self) -> None:
        """Unsubscribe and release everything the device owns.

        Shuts down a privately owned scheduler so no device work can reach the
        bus after this returns, and sets the shutdown latch.  Sends no protocol
        traffic — call :meth:`stop` first for that.  Each device removes its
        own periodic jobs before detaching.  Safe to call without a matching
        :meth:`stop` and safe to call more than once.
        """
        self._release_scheduler()
        self._shutdown.set()
        self._running = False
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception as exc:  # noqa: BLE001 - teardown must not abort
                logger.warning("device %s: unsubscribe failed: %s", self.device_id, exc)
            self._unsubscribe = None
        self._bus = None

    # ---- lifecycle hooks ----------------------------------------------

    def _on_attached(self) -> None:
        """Hook for subclasses (e.g. install bus filters)."""

    def start(self) -> None:
        """Enter the run phase.

        Clears the shutdown latch so interruptible work can proceed, marks the
        device running and makes sure a privately owned scheduler is up.
        Overrides call this first, then do their protocol work and schedule
        their own periodic jobs on :attr:`scheduler`.
        """
        self._shutdown.clear()
        self._running = True
        if self._owns_scheduler:
            scheduler = self.scheduler
            if not scheduler.running:
                scheduler.start()

    def stop(self) -> None:
        """Leave the run phase.

        Sets the shutdown latch (so an in-flight burst gives up waiting) and
        clears the running flag.  Overrides call this first — a device removes
        its own periodic jobs and then sends its shutdown traffic, so nothing
        scheduled can interleave with that traffic.
        """
        self._shutdown.set()
        self._running = False

    # ---- frame handling -----------------------------------------------

    @abc.abstractmethod
    def _on_frame(self, frame: CANFrame) -> None:
        """Parse a frame and emit a measurement if applicable."""

    # ---- scheduler dependency -----------------------------------------

    @property
    def scheduler(self) -> BaseScheduler:
        """The scheduler this device schedules its jobs on, created on first use."""
        if self._scheduler is None:
            self._scheduler = create_scheduler(f"device-{self.device_id}")
        return self._scheduler

    def _release_scheduler(self) -> None:
        """Shut down a privately owned scheduler; a shared one is left alone."""
        if not self._owns_scheduler or self._scheduler is None:
            return
        if self._scheduler.running:
            self._scheduler.shutdown(wait=False)
        self._scheduler = None

    # ---- helpers -------------------------------------------------------

    def stats(self) -> dict[str, int]:
        """Return per-device counters for the periodic status line.

        Subclasses that track bus/driver statistics override this; the base
        implementation reports nothing (empty dict).
        """
        return {}

    def _send_remote_frame(self, can_id: int, *, label: str) -> bool:
        """Send one Remote Frame, reporting rather than raising on failure.

        Returns True when the frame reached the bus.  A detached device or a
        closed bus is a warning, not an error: senders run on lifecycle and
        scheduler paths that must still finish cleanly.
        """
        bus = self._bus
        if bus is None:
            logger.warning(
                "%s: not attached to a bus, dropped Remote Frame 0x%03X", label, can_id
            )
            return False
        try:
            bus.send_remote_frame(can_id)
        except RuntimeError as exc:
            # Bus backends wrap every send failure in CANBusError, a RuntimeError.
            logger.error(
                "%s: failed to send Remote Frame 0x%03X: %s", label, can_id, exc
            )
            return False
        logger.debug("%s: sent Remote Frame 0x%03X", label, can_id)
        return True

    def _send_remote_burst(
        self,
        can_id: int,
        count: int,
        *,
        gap: float,
        label: str,
        interruptible: bool = True,
    ) -> int:
        """Send *count* Remote Frames for *can_id*, *gap* seconds apart.

        Failures are logged per frame and never abort the burst — a burst is a
        best-effort nudge, not a transaction.  Returns how many frames reached
        the bus.

        Args:
            can_id: Arbitration ID of the Remote Frame to repeat.
            count: Number of frames to send; <= 0 sends nothing.
            gap: Seconds to wait between frames (never after the last one).
            label: Name of the burst in log lines, e.g. ``"start"``.
            interruptible: When True the wait between frames is cut short by
                :meth:`stop` and the remaining frames are skipped, so shutdown
                is never delayed.  Pass False for traffic that must land,
                such as the Stop frames sent during teardown.
        """
        if count <= 0:
            return 0
        sent = 0
        for i in range(count):
            if self._send_remote_frame(can_id, label=label):
                sent += 1
            last = i >= count - 1
            if last or gap <= 0:
                continue
            if not self._wait(gap, interruptible):
                logger.info("%s: burst cut short after %d/%d frames", label, sent, count)
                return sent
        logger.info("%s: sent %d/%d Remote Frames for 0x%03X", label, sent, count, can_id)
        return sent

    def _wait(self, timeout: float, interruptible: bool) -> bool:
        """Wait up to *timeout* seconds; return True when the wait completed.

        An *interruptible* wait ends early once :meth:`stop` has been called
        and returns False, so the caller can abandon its work instead of
        holding up shutdown.  A non-interruptible wait always completes.
        """
        if not interruptible:
            time.sleep(timeout)
            return True
        return not self._shutdown.wait(timeout)

    def _emit_measurement(self, measurement: Measurement, frame: CANFrame) -> None:
        self.events.emit(
            self.EVENT_MEASUREMENT,
            DeviceMeasurement(self.device_id, self.KIND, measurement, frame),
        )

    def _emit_status(self, event: StatusEvent) -> None:
        self.events.emit(self.EVENT_STATUS, event)
