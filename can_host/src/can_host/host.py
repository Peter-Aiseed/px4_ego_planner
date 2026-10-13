"""CAN host runtime: owns the bus + device lifecycle and the monitor loop.

Use :class:`Host` to run a CAN bus with one or more devices attached.
"""

from __future__ import annotations

import logging
import signal
import sys
import threading
from enum import IntEnum
from typing import TYPE_CHECKING, Any

from . import create_can_bus
from .can_bus.base import CANBus, CANBusError
from .devices._afbr_parser import AFBRParser
from .devices.base import Device
from .devices.factory import create_all_devices, create_device, list_registered_devices
from .devices.measurements import UAVCANRangeMeasurement
from .events import EventBus
from .scheduler import create_scheduler, schedule_interval
from .usb_scan import resolve_baudrate

if TYPE_CHECKING:
    from apscheduler.schedulers.base import BaseScheduler

logger = logging.getLogger(__name__)


class ExitCode(IntEnum):
    """Process exit codes returned by :meth:`Host.setup` / :meth:`Host.run`."""

    OK = 0
    SETUP_ERROR = 2


class Host:
    """Owns the CAN bus + devices and runs the monitor loop.

    ``main()`` constructs ``Host(args)``, calls :meth:`setup`, registers
    renderers on ``host.event_bus``, then calls :meth:`run`.  ``Host`` tracks
    its own ``frame_count`` via an internal measurement subscription for
    status reporting — it never references renderer classes.

    All periodic work — the host's status block and every device's keepalive —
    runs as an APScheduler job on ``host.scheduler``, so :meth:`run` only has
    to block until a shutdown signal arrives.
    """

    #: Job id of the periodic status block.
    STATUS_JOB = "host-status"

    def __init__(self, args: Any) -> None:
        self.args = args
        self.event_bus: EventBus = EventBus()
        self.bus: CANBus | None = None
        self.devices: list[Device] = []
        self.frame_count: int = 0
        self._range_counts: dict[str, int] = {}
        self._stop_event = threading.Event()
        self.scheduler: BaseScheduler = create_scheduler()

    # -- setup --------------------------------------------------------------

    def _resolve_channel_if_needed(self) -> str | None:
        """Auto-resolve a channel when ``--channel`` was not passed.

        Delegates the per-bus policy to :func:`usb_scan.resolve_channel_for_bus`
        (SocketCAN interfaces, WaveShare serial ports, …). ``--index`` selects
        the Nth candidate of that bus, and it is ordered so ``--bus <name>
        --index N`` picks the same adapter as ``--bus auto --index N``.

        Returns the explicit or resolved channel, or None when nothing could
        be resolved (the caller decides how to report/handle that).
        """
        if self.args.channel not in (None, ""):
            return str(self.args.channel)
        from .usb_scan import resolve_channel_for_bus

        return resolve_channel_for_bus(str(self.args.bus), index=int(self.args.index))

    def setup(self) -> int:
        """Resolve args, create bus + devices + internal listeners."""
        resolved = self._resolve_channel_if_needed()
        if self.args.bus != "auto" and resolved is None:
            logger.error(
                "No auto-detectable channel for --bus %s. "
                "Pass --channel explicitly "
                "(e.g. 'can0' for socketcan; /dev/ttyXXX for serial).",
                self.args.bus,
            )
            return ExitCode.SETUP_ERROR
        if resolved is not None:
            self.args.channel = resolved

        rc = self._create_bus()
        if rc != ExitCode.OK:
            return rc

        rc = self._create_devices()
        if rc != ExitCode.OK:
            return rc

        rc = self._open_bus()
        if rc != ExitCode.OK:
            return rc

        self._on_setup_complete()
        return ExitCode.OK

    def _resolve_baudrate(self) -> int | None:
        override: int | None = self.args.baudrate
        if override is not None:
            return override
        if self.args.bus == "auto":
            return None
        return resolve_baudrate(channel=self.args.channel)

    def _create_bus(self) -> int:
        try:
            baudrate = self._resolve_baudrate()
            self.bus = create_can_bus(
                self.args.bus,
                channel=self.args.channel,
                event_bus=self.event_bus,
                bitrate=self.args.bitrate,
                baudrate=baudrate,
                no_config=self.args.no_config,
                no_filter=self.args.no_filter,
                raw_debug=self.args.raw,
                index=self.args.index,
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to create CAN bus: %s", exc)
            return ExitCode.SETUP_ERROR
        return ExitCode.OK

    def _create_devices(self) -> int:
        try:
            # All kwargs are passed; factory filters per-class via inspect.signature
            kwargs: dict[str, Any] = {
                "can_ids": getattr(self.args, "can_ids_list", AFBRParser.FRAME_ID_1D),
                "send_start": self.args.send_start,
                "send_stop": self.args.send_stop,
                "start_interval": self.args.start_interval,
                "auto_alloc": getattr(self.args, "auto_alloc", True),
                "scheduler": self.scheduler,
            }

            if self.args.device == "all":
                extra: dict[str, Any] = {}
                if not self.args.no_filter and len(list_registered_devices()) > 1:
                    extra["auto_install_filters"] = False
                self.devices = create_all_devices(self.event_bus, **kwargs, **extra)
            else:
                self.devices = [
                    create_device(self.args.device, self.event_bus, **kwargs)
                ]
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to create device(s): %s", exc)
            return ExitCode.SETUP_ERROR
        return ExitCode.OK

    def _open_bus(self) -> int:
        assert self.bus is not None
        try:
            self.bus.open()
        except CANBusError as exc:
            logger.error("Failed to open CAN bus: %s", exc)
            return ExitCode.SETUP_ERROR
        return ExitCode.OK

    def _on_setup_complete(self) -> None:
        """Subscribe internal listeners and attach all devices to bus."""
        assert self.bus is not None and self.devices
        self.event_bus.subscribe("measurement", self._on_measurement)
        self._announce()
        for device in self.devices:
            device.attach(self.bus)
        self._install_signal_handlers()

    # -- public: run --------------------------------------------------------

    def run(self) -> int:
        """Enter the monitor loop until signalled to stop."""
        self.scheduler.start()
        self._start_devices()
        try:
            self._print_listening_banner()
            self._run_monitor()
            self._stop_event.wait()
        finally:
            self._cleanup()
        return 0

    # -- internal: measurement tracking ------------------------------------

    def _on_measurement(self, event: Any) -> None:
        self.frame_count += 1
        if not getattr(self.args, "show_hz", False):
            return

        show_ids = getattr(self.args, "show_ids_list", None)
        kind = getattr(event, "kind", "")

        if kind == "afbr":
            if show_ids is not None and event.frame.arbitration_id not in show_ids:
                return
            self._increment_hz(f"0x{event.frame.arbitration_id:02X}")
        elif kind == "uavcan":
            meas = event.measurement
            if show_ids is not None and meas.id not in show_ids:
                return
            if isinstance(meas, UAVCANRangeMeasurement):
                self._increment_hz(f"0x{meas.id:02X}")

    def _increment_hz(self, key: str) -> None:
        """Increment Hz counter for a given device ID key."""
        self._range_counts[key] = self._range_counts.get(key, 0) + 1

    # -- internal: setup helpers -------------------------------------------

    def _announce(self) -> None:
        assert self.bus is not None
        selected = getattr(self.bus, "selected", None)
        if selected is not None and self.args.bus == "auto":
            logger.info(
                "Auto-detected CAN adapter: backend=%s channel=%s (%s)",
                selected.backend,
                selected.channel,
                selected.label,
            )

    def _install_signal_handlers(self) -> None:
        def stop(_signum: int, _frame: Any) -> None:
            self._stop_event.set()
            logger.info("Shutdown requested")

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

    def _start_devices(self) -> None:
        assert self.bus is not None and self.devices
        for device in self.devices:
            device.start()

    # -- internal: run loop ------------------------------------------------

    def _run_monitor(self) -> None:
        """Schedule the status block, then block until shutdown is requested.

        Nothing periodic happens on this thread: measurements arrive on the
        bus's reader thread and the status block runs on the scheduler, so the
        main thread only has to wait for a signal to land.
        """
        schedule_interval(
            self.scheduler,
            self._log_status_block,
            job_id=self.STATUS_JOB,
            seconds=self._status_interval(),
        )

    def _status_interval(self) -> int:
        """Status block period in whole seconds; APScheduler needs an int here."""
        return max(1, int(self.args.status_interval))

    def _log_status_block(self) -> None:
        """One periodic status block: totals, per-device stats, per-ID Hz."""
        self._log_blank()          # separate the status/HZ block visually
        self._print_status()
        if getattr(self.args, "show_hz", False) and self._range_counts:
            self._log_blank()  # blank line between status and HZ
            self._print_hz()
        self._log_blank()

    def _log_blank(self) -> None:
        """Write a blank line to the log stream so periodic summary blocks pop."""
        print(file=sys.stderr)

    def _print_status(self) -> None:
        """Log the periodic status line including per-device decode stats."""
        line = f"[status] waiting... frames_received={self.frame_count}"
        extras = []
        for device in self.devices:
            stats = device.stats()
            if not stats:
                continue
            ok = stats.get("ok", 0)
            fail = stats.get("fail", 0)
            total = ok + fail
            pct = f"{fail / total * 100:.1f}%" if total else "-"
            extras.append(
                f"{device.device_id}: rx={stats.get('rx_frames', 0)} "
                f"ok={ok} fail={fail}({pct}) active={stats.get('active', 0)}"
            )
        if extras:
            line += " | " + "; ".join(extras)
        logger.info(line)

    def _print_hz(self) -> None:
        if not self._range_counts:
            return
        interval = self._status_interval()
        parts = [
            f"{k}={count / interval:.1f}Hz"
            for k, count in sorted(self._range_counts.items())
        ]
        logger.info("[hz] " + " ".join(parts))
        self._range_counts.clear()

    def _print_listening_banner(self) -> None:
        if not self.args.json and not self.args.csv:
            device_types = ", ".join(d.device_id for d in self.devices)
            selected = getattr(self.bus, "selected", None)
            channel = selected.channel if selected is not None else self.args.channel
            logger.info(
                "Listening on %s via %s @ %d bps; devices=[%s], "
                "accepted IDs=%s. show IDs=%s. Press Ctrl+C to stop.",
                channel,
                self.args.bus,
                self.args.bitrate,
                device_types,
                [f"0x{c:02X}" for c in getattr(self.args, "can_ids_list", AFBRParser.FRAME_ID_1D)],
                self.args.show_ids or "all",
            )

    # -- internal: teardown -------------------------------------------------

    def _cleanup(self) -> None:
        assert self.bus is not None and self.devices
        try:
            self.scheduler.remove_all_jobs()
            if self.scheduler.running:
                self.scheduler.shutdown(wait=False)
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to stop scheduler: %s", exc)
        for device in self.devices:
            try:
                device.stop()
            except Exception as exc:  # noqa: BLE001
                logger.error("Failed to stop device %r: %s", device.device_id, exc)
        for device in self.devices:
            try:
                device.detach()
            except Exception as exc:  # noqa: BLE001
                logger.error("Failed to detach device %r: %s", device.device_id, exc)
        self.bus.close()
