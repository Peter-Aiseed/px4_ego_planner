"""CAN host runtime: owns the bus + device lifecycle and the monitor loop.

Use :class:`Host` to run a CAN bus with one or more devices attached.
"""

from __future__ import annotations

import logging
import signal
import sys
import threading
import time
from typing import Any, Optional

from . import create_can_bus, create_device
from .devices.factory import create_all_devices, list_registered_devices
from .devices._afbr_parser import AFBRParser
from .can_bus.base import CANBus, CANBusError
from .devices.base import Device
from .events import EventBus

logger = logging.getLogger(__name__)

_DEFAULT_GAP = 0.1


def _ts() -> str:
    now = time.time()
    return time.strftime("%H:%M:%S", time.localtime(now)) + f".{now % 1:.3f}".split(".")[1]


class Host:
    """Owns the CAN bus + devices and runs the monitor loop.

    ``main()`` constructs ``Host(args)``, calls :meth:`setup`, registers
    renderers on ``host.event_bus``, then calls :meth:`run`.  ``Host`` tracks
    its own ``frame_count`` via an internal measurement subscription for
    heartbeat reporting — it never references renderer classes.
    """

    def __init__(self, args: Any) -> None:
        self.args = args
        self.event_bus: EventBus = EventBus()
        self.bus: Optional[CANBus] = None
        self.devices: list[Device] = []
        self.frame_count: int = 0
        self._range_counts: dict[str, int] = {}
        self._stop_event = threading.Event()

    # -- remote frames -------------------------------------------------------

    def _send_remote_frames(
        self,
        can_id: int,
        count: int,
        gap: float = _DEFAULT_GAP,
        interruptible: bool = True,
    ) -> None:
        """Send *count* remote frames at *can_id*.

        When *interruptible* is True the method exits early if a shutdown
        signal arrives (appropriate during start-up).  When False it sends
        all frames regardless (appropriate during cleanup, e.g. the final
        Stop frame that tells the sensor to stop).
        """
        for i in range(count):
            if interruptible and self._stop_event.is_set():
                return
            try:
                assert self.bus is not None
                self.bus.send_remote_frame(can_id)
                print(f"{_ts()} Sent Remote Frame ID=0x{can_id:03X} ({i+1}/{count})", flush=True)
            except CANBusError as exc:
                print(f"{_ts()} Failed to send Remote Frame: {exc}", file=sys.stderr, flush=True)
            if i < count - 1 and gap > 0:
                if interruptible and self._stop_event.wait(gap):
                    return
                if not interruptible:
                    time.sleep(gap)

    # -- setup --------------------------------------------------------------

    def _resolve_uart_channel_if_needed(self) -> str:
        """Auto-resolve a serial port for uart/slcan when not explicitly set."""
        if self.args.bus in ("uart", "slcan") and self.args.channel == "can0":
            from .usb_scan import resolve_uart_channel
            resolved = resolve_uart_channel()
            if resolved is None:
                print(
                    f"{_ts()} No UART port found for --bus " + self.args.bus + ". "
                    "Pass --channel /dev/ttyXXX explicitly "
                    "(e.g. /dev/ttyTHS1 or /dev/ttyUSB0).",
                    file=sys.stderr,
                )
                raise SystemExit(2)
            logger.info("Auto-resolved UART port: %s", resolved)
            return resolved
        return str(self.args.channel)

    def setup(self) -> int:
        """Resolve args, create bus + devices + internal listeners."""
        self.args.channel = self._resolve_uart_channel_if_needed()

        rc = self._create_bus()
        if rc != 0:
            return rc

        rc = self._create_devices()
        if rc != 0:
            return rc

        rc = self._open_bus()
        if rc != 0:
            return rc

        self._on_setup_complete()
        return 0

    def _create_bus(self) -> int:
        try:
            self.bus = create_can_bus(
                self.args.bus,
                channel=self.args.channel,
                event_bus=self.event_bus,
                bitrate=self.args.bitrate,
                no_config=self.args.no_config,
                no_filter=self.args.no_filter,
                raw_debug=self.args.raw,
                index=self.args.index,
                baudrate=self.args.baudrate,
            )
        except Exception as exc:
            print(f"{_ts()} Failed to create CAN bus: {exc}", file=sys.stderr)
            return 2
        return 0

    def _create_devices(self) -> int:
        try:
            kwargs: dict[str, Any] = {"can_ids": AFBRParser.FRAME_ID_1D}
            if self.args.device == "all":
                extra: dict[str, Any] = {}
                if not self.args.no_filter and len(list_registered_devices()) > 1:
                    extra["auto_install_filters"] = False
                self.devices = create_all_devices(
                    self.event_bus,
                    auto_alloc=getattr(self.args, "auto_alloc", True),
                    **kwargs,
                    **extra,
                )
            else:
                if self.args.device == "uavcan" and hasattr(self.args, "auto_alloc"):
                    kwargs["auto_alloc"] = self.args.auto_alloc
                self.devices = [create_device(
                    self.args.device,
                    event_bus=self.event_bus,
                    **kwargs,
                )]
        except Exception as exc:
            print(f"{_ts()} Failed to create device(s): {exc}", file=sys.stderr)
            return 2
        return 0

    def _open_bus(self) -> int:
        assert self.bus is not None
        try:
            self.bus.open()
        except CANBusError as exc:
            print(f"{_ts()} Failed to open CAN bus: {exc}", file=sys.stderr)
            return 2
        return 0

    def _on_setup_complete(self) -> None:
        """Subscribe internal listeners and attach all devices to bus."""
        assert self.bus is not None and self.devices
        self.event_bus.on("measurement", self._on_measurement)
        self._announce()
        for device in self.devices:
            device.attach(self.bus)
        self._install_signal_handlers()

    # -- public: run --------------------------------------------------------

    def run(self) -> int:
        """Enter the monitor loop until signalled to stop."""
        self._start_devices()
        try:
            self._print_listening_banner()
            self._run_monitor()
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
            key = f"0x{event.frame.arbitration_id:02X}"
            self._range_counts[key] = self._range_counts.get(key, 0) + 1
        elif kind == "uavcan":
            if show_ids is not None:
                node_id = event.measurement.get("id", 0)
                if node_id not in show_ids:
                    return
            if event.measurement.get("measurement_type") == "range":
                node_id = event.measurement.get("id", 0)
                key = f"0x{node_id:02X}"
                self._range_counts[key] = self._range_counts.get(key, 0) + 1

    # -- internal: setup helpers -------------------------------------------

    def _announce(self) -> None:
        assert self.bus is not None
        selected = getattr(self.bus, "selected", None)
        if selected is not None and self.args.bus == "auto":
            print(
                f"{_ts()} Auto-detected CAN adapter: backend={selected.backend} "
                f"channel={selected.channel} ({selected.label})",
                flush=True,
            )

    def _install_signal_handlers(self) -> None:
        def stop(_signum: int, _frame: Any) -> None:
            self._stop_event.set()
            print(f"{_ts()} Shutdown requested", flush=True)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

    def _start_devices(self) -> None:
        assert self.bus is not None and self.devices
        if self.args.send_start > 0:
            self._send_remote_frames(int(self.args.start_id, 0), self.args.send_start)
        for device in self.devices:
            device.start()

    # -- internal: run loop ------------------------------------------------

    def _run_monitor(self) -> None:
        last_heartbeat = time.time()
        last_hz_print = time.time()
        while not self._stop_event.is_set():
            wait_time = min(self.args.timeout, 1.0)
            if self._stop_event.wait(wait_time):
                break
            now = time.time()
            if now - last_heartbeat >= self.args.heartbeat:
                print(
                    f"{_ts()} [heartbeat] waiting... frames_received={self.frame_count}",
                    flush=True,
                )
                last_heartbeat = now
            if getattr(self.args, "show_hz", False) and now - last_hz_print >= 1.0:
                self._print_hz()
                last_hz_print = now

    def _print_hz(self) -> None:
        if not self._range_counts:
            return
        parts = [f"{k}={v}Hz" for k, v in sorted(self._range_counts.items())]
        print(f"{_ts()} [hz] " + " ".join(parts), flush=True)
        self._range_counts.clear()

    def _print_listening_banner(self) -> None:
        if not self.args.json and not self.args.csv:
            device_types = ", ".join(d.device_id for d in self.devices)
            print(
                f"{_ts()} Listening on {self.args.channel} via {self.args.bus} @ {self.args.bitrate} bps; "
                f"devices=[{device_types}], "
                f"accepted IDs={[f'0x{c:02X}' for c in AFBRParser.FRAME_ID_1D]}. "
                f"show IDs={self.args.show_ids or 'all'}. "
                "Press Ctrl+C to stop.",
                flush=True,
            )

    # -- internal: teardown -------------------------------------------------

    def _cleanup(self) -> None:
        assert self.bus is not None and self.devices
        for device in self.devices:
            try:
                device.stop()
            except Exception as exc:
                print(f"{_ts()} Failed to stop device {device.device_id!r}: {exc}", file=sys.stderr)
        if self.args.send_stop > 0:
            try:
                self._send_remote_frames(
                    int(self.args.stop_id, 0), self.args.send_stop, interruptible=False,
                )
            except Exception as exc:
                print(f"{_ts()} Failed to send Stop frames: {exc}", file=sys.stderr)
        for device in self.devices:
            try:
                device.detach()
            except Exception as exc:
                print(f"{_ts()} Failed to detach device {device.device_id!r}: {exc}", file=sys.stderr)
        self.bus.close()