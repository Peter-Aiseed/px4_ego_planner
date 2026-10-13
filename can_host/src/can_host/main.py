"""
AFBR-S50 ToF CAN host.

Pluggable event-driven architecture:

  CAN bus (socketcan / waveshare) -- publishes raw CANFrame events
        |
        v
  Device (e.g. AFBR_S50)          -- parses frames, publishes measurements
        |
        v
  Host (can_host.runner.Host)     -- runs event loop, manages bus/device lifecycle
        |
        v
  Renderer (Printer / CsvRecorder, registered by main.py) -- prints or records

main() parses arguments, creates Host, registers renderers, and runs.
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
from pathlib import Path
from typing import Any, TextIO

from .devices._afbr_parser import parse_can_id_list
from .devices.afbr_s50 import DEFAULT_START_INTERVAL
from .devices.measurements import AFBRMeasurement, UAVCANFlowMeasurement
from .events import DeviceMeasurement, StatusEvent
from .host import ExitCode, Host
from .key_debug import KeyListener

logger = logging.getLogger(__name__)

DEFAULT_BUS = "auto"
DEFAULT_DEVICE = "all"
DEFAULT_BITRATE = 1_000_000
DEFAULT_TIMEOUT = 1.0
DEFAULT_CAN_IDS = "0x18-0x1F"
DEFAULT_CSV_FIELDS = (
    "host_time_iso,host_time_unix,device_id,can_id,range_mm,"
    "amplitude_lsb,signal_quality,status,status_desc,valid"
)


class _MsFormatter(logging.Formatter):
    _RED = "\x1b[31m"
    _RESET = "\x1b[0m"

    def __init__(self, *, color: bool = False, fmt: str, datefmt: str) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt)
        self._color = color

    def format(self, record: logging.LogRecord) -> str:
        line = super().format(record)
        if self._color and record.levelno >= logging.WARNING:
            return f"{self._RED}{line}{self._RESET}"
        return line

    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = time.strftime(datefmt, ct)
            return f"{s}.{int(record.msecs):03d}"
        return super().formatTime(record, datefmt)


# ---------------------------------------------------------------------------
# Renderers — subscribe to events and write output
# ---------------------------------------------------------------------------


def _measurement_fields(event: DeviceMeasurement) -> dict[str, Any]:
    """Extract common measurement fields used by Printer and CsvRecorder."""
    meas = event.measurement
    if not isinstance(meas, AFBRMeasurement):
        return {}  # UAVCAN measurements handled by the Printer's _render_uavcan
    frame = event.frame
    now = time.time()
    return {
        "host_time_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(now)),
        "host_time_unix": f"{now:.6f}",
        "device_id": event.device_id,
        "id": f"0x{meas.id:03X}",
        "can_id": f"0x{frame.arbitration_id:03X}",
        "range_mm": meas.range_mm,
        "amplitude_lsb": meas.amplitude_lsb,
        "signal_quality": meas.signal_quality,
        "status": meas.status,
        "status_desc": meas.status_desc,
        "valid": meas.is_valid,
        "raw": frame.data.hex(),
    }


class Printer:
    """Subscribes to measurement/status events and prints them.

    Handles both AFBR-S50 range measurements and UAVCAN/DroneCAN messages
    (range, flow, allocation) in a single handler.
    """

    def __init__(self, as_json: bool, show_ids: list[int] | None = None, show_flow: bool = False) -> None:
        self.as_json = as_json
        self.show_ids = show_ids
        self.show_flow = show_flow
        self.frame_count: int = 0

    def on_measurement(self, event: DeviceMeasurement) -> None:
        meas = event.measurement
        if self.show_ids is not None and meas.id not in self.show_ids:
            return
        if isinstance(meas, UAVCANFlowMeasurement) and not self.show_flow:
            return
        frame_num = self._next_frame()
        if self.as_json:
            logger.info(meas.to_json(frame_num=frame_num, can_id=event.frame.arbitration_id))
        else:
            logger.info(f"[{frame_num}] {meas.to_string()}")

    def on_status(self, event: StatusEvent) -> None:
        if self.show_ids is not None and event.id not in self.show_ids:
            return
        if self.as_json:
            logger.info(event.to_json())
        else:
            logger.info(f"[status] {event.to_string()}")

    def _next_frame(self) -> int:
        """Increment and return the frame counter."""
        self.frame_count += 1
        return self.frame_count


class CsvRecorder:
    def __init__(self, path: Path, fieldnames: list[str], show_ids: list[int] | None = None) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not path.exists() or path.stat().st_size == 0
        self._fh: TextIO = path.open("a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fh, fieldnames=fieldnames, extrasaction="ignore")
        if is_new:
            self._writer.writeheader()
            self._fh.flush()
        self._fields = fieldnames
        self._show_ids = show_ids

    def on_measurement(self, event: DeviceMeasurement) -> None:
        if self._show_ids is not None and event.measurement.id not in self._show_ids:
            return
        row = _measurement_fields(event)
        self._writer.writerow(row)

    def close(self) -> None:
        self._fh.close()


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def setup_logging(verbose: bool, show_lineno: bool = False) -> None:
    handler = logging.StreamHandler()
    fmt = "%(asctime)s [%(levelname)s] %(name)s"
    if show_lineno:
        fmt += ":%(lineno)d"
    fmt += ": %(message)s"
    handler.setFormatter(_MsFormatter(
        fmt=fmt,
        datefmt="%Y-%m-%d %H:%M:%S",
        color=sys.stderr.isatty(),
    ))
    root = logging.getLogger()
    root.handlers = [handler]
    root.setLevel(logging.DEBUG if verbose else logging.INFO)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="AFBR-S50 ToF CAN host (event-driven, pluggable).",
    )
    p.add_argument("--bus", default=DEFAULT_BUS,
                   choices=["auto", "socketcan", "waveshare", "gs_usb", "slcan_serial"],
                   help="CAN bus backend. 'auto' detects via USB + SocketCAN.")
    p.add_argument("--index", type=int, default=0,
                   help="When multiple adapters match, pick this index "
                        "(0-based). Same adapter for every --bus.")
    p.add_argument("--channel", default=None,
                   help="Bus channel. Auto-detected when omitted: active "
                        "SocketCAN interface (can0/can1) for --bus socketcan; "
                        "USB serial port for waveshare and slcan_serial. "
                        "Ignored when --bus=auto.")
    p.add_argument("--device", default=DEFAULT_DEVICE,
                    help=f"Device driver name (default: {DEFAULT_DEVICE} = all registered devices)")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bitrate in bps (default: {DEFAULT_BITRATE})")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                   help="Deprecated and ignored: the monitor loop now blocks until "
                        "shutdown instead of polling. Kept so existing scripts still run.")
    p.add_argument("--can-ids", default=DEFAULT_CAN_IDS,
                   help=f"Comma-separated CAN IDs and/or ranges (e.g. '0x1C' or '0x18-0x1F') "
                        f"accepted as 1D measurement data (default: {DEFAULT_CAN_IDS})")
    p.add_argument("--no-filter", action="store_true",
                   help="Don't install hardware CAN filters; pass everything through")
    p.add_argument("--show-ids", default=None,
                    help="Comma-separated CAN IDs and/or ranges (e.g. '0x1C' or '0x18-0x1F') "
                         "to print; others are hidden. Default: show all.")
    p.add_argument("--show-flow", action="store_true", default=False,
                    help="[uavcan] Show UAVCAN flow measurement info (default: off)")
    p.add_argument("--show-hz", action="store_true", default=True,
                    help="Show range message Hz per device ID (default: on)")
    p.add_argument("--no-config", action="store_true",
                   help="[waveshare] Skip sending adapter config packet")
    p.add_argument("--raw", action="store_true",
                   help="[waveshare] Dump all raw serial bytes for debugging")
    p.add_argument(
        "--baudrate",
        type=int,
        default=None,
        help="UART line baud rate. When omitted, each serial backend resolves "
        "it from the adapter's USB ids (WaveShare CH340 → 2000000, "
        "FT232R → 1500000), falling back to 115200 for unrecognized or "
        "non-USB ports. With --bus auto it is resolved for the adapter "
        "--index selects.",
    )
    p.add_argument("--json", action="store_true", help="Print JSON instead of human-readable")
    p.add_argument("--csv", type=Path, help="Append measurements to this CSV file")
    p.add_argument("--csv-fields", default=DEFAULT_CSV_FIELDS,
                   help=f"CSV columns (default: {DEFAULT_CSV_FIELDS})")
    p.add_argument("--status-interval", type=int, default=5,
                   help="Status log interval in seconds when idle (default: 5)")
    p.add_argument("--send-start", type=int, default=3, metavar="N",
                   help="Send Start Remote Frame N times before monitoring (default: 3)")
    p.add_argument("--send-stop", type=int, default=3, metavar="N",
                   help="Send Stop Remote Frame N times after monitoring ends (default: 3)")
    p.add_argument(
        "--start-interval", type=float, default=DEFAULT_START_INTERVAL, metavar="SEC",
        help="[afbr] Re-send the Start Remote Frame every SEC seconds to keep the "
             "sensor streaming; a sensor that rebooted restarts on its own. "
             f"0 sends it only at startup (default: {DEFAULT_START_INTERVAL})",
    )
    p.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging")
    p.add_argument(
        "--debug-keys", action="store_true", default=False,
        help="Print keystrokes to the log for debugging (interactive terminal only). "
             "Uses standard library only — no external deps.",
    )
    p.add_argument("--log-lines", action="store_true", default=False,
                   help="Include source line numbers in log output")
    p.add_argument("--no-auto-alloc", action="store_false", dest="auto_alloc", default=True,
                   help="[uavcan] Disable dynamic node ID allocation (default: on)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Entry point — main() just wires things up and runs Host
# ---------------------------------------------------------------------------


def main() -> int:
    """Parse args, register renderers, and run the host."""
    args = parse_args()
    setup_logging(args.verbose, args.log_lines)

    if args.can_ids:
        args.can_ids_list = parse_can_id_list(args.can_ids)

    show_ids: list[int] | None = None
    if args.show_ids:
        show_ids = parse_can_id_list(args.show_ids)
    args.show_ids_list = show_ids

    host = Host(args)

    rc = host.setup()
    if rc != ExitCode.OK:
        return rc

    printer = Printer(as_json=args.json, show_ids=show_ids, show_flow=args.show_flow)
    host.event_bus.subscribe("measurement", printer.on_measurement)
    host.event_bus.subscribe("status", printer.on_status)

    csv_recorder: CsvRecorder | None = None
    if args.csv:
        csv_fields = [f.strip() for f in args.csv_fields.split(",") if f.strip()]
        csv_recorder = CsvRecorder(args.csv, csv_fields, show_ids=show_ids)
        host.event_bus.subscribe("measurement", csv_recorder.on_measurement)

    key_listener = KeyListener(enabled=args.debug_keys)
    key_listener.start()

    try:
        return host.run()
    finally:
        key_listener.stop()
        if csv_recorder is not None:
            csv_recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
