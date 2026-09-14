#!/usr/bin/env python3
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
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Optional, TextIO

from .devices._afbr_parser import AFBRParser, parse_can_id_list
from .events import DeviceMeasurement, StatusEvent
from .host import Host

logger = logging.getLogger(__name__)

DEFAULT_BUS = "auto"
DEFAULT_CHANNEL = "can0"
DEFAULT_DEVICE = "all"
DEFAULT_BITRATE = 1_000_000
DEFAULT_TIMEOUT = 1.0
DEFAULT_CAN_IDS = "0x18-0x1F"
DEFAULT_CSV_FIELDS = (
    "host_time_iso,host_time_unix,device_id,can_id,range_mm,"
    "amplitude_lsb,signal_quality,status,status_desc,valid"
)


def _ts() -> str:
    now = time.time()
    return time.strftime("%H:%M:%S", time.localtime(now)) + f".{now % 1:.3f}".split(".")[1]


# ---------------------------------------------------------------------------
# Renderers — subscribe to events and write output
# ---------------------------------------------------------------------------


def _measurement_fields(event: DeviceMeasurement) -> dict[str, Any]:
    """Extract common measurement fields used by Printer and CsvRecorder."""
    meas = event.measurement
    if event.kind != "afbr":
        return {}  # UAVCAN measurements handled by UAVCANPrinter
    frame = event.frame
    return {
        "host_time_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        "host_time_unix": f"{time.time():.6f}",
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
    """Subscribes to AFBR measurement/status events and prints them."""

    def __init__(self, as_json: bool, show_ids: Optional[list[int]] = None) -> None:
        self.as_json = as_json
        self.show_ids = show_ids
        self.frame_count: int = 0

    def on_measurement(self, event: DeviceMeasurement) -> None:
        if event.kind != "afbr":
            return
        if self.show_ids is not None and event.frame.arbitration_id not in self.show_ids:
            return
        self.frame_count += 1
        meas = event.measurement
        dev_id = f"0x{meas.id:03X}"
        status_ok = meas.status == 0
        if self.as_json:
            entry = {
                "frame": self.frame_count,
                "id": dev_id,
                "range_mm": meas.range_mm,
                "amplitude_lsb": meas.amplitude_lsb,
                "signal_quality": meas.signal_quality,
                "valid": meas.is_valid,
            }
            if not status_ok:
                entry["status"] = meas.status
                entry["status_desc"] = meas.status_desc
            print(json.dumps(entry, ensure_ascii=False), flush=True)
        else:
            valid = "OK" if meas.is_valid else "INVALID"
            if status_ok:
                print(
                    f"{_ts()} [{self.frame_count}] id={dev_id} "
                    f"range_mm={meas.range_mm} amplitude_lsb={meas.amplitude_lsb} "
                    f"signal_quality={meas.signal_quality} valid={valid}",
                    flush=True,
                )
            else:
                print(
                    f"{_ts()} [{self.frame_count}] id={dev_id} "
                    f"range_mm={meas.range_mm} amplitude_lsb={meas.amplitude_lsb} "
                    f"signal_quality={meas.signal_quality} status={meas.status} "
                    f"status_desc={meas.status_desc} valid={valid}",
                    flush=True,
                )

    def on_status(self, event: StatusEvent) -> None:
        stream = sys.stderr if event.level in ("warning", "error") else sys.stdout
        print(f"{_ts()} [{event.level}] {event.source}: {event.message}", file=stream, flush=True)


class UAVCANPrinter:
    """Subscribes to UAVCAN measurement events and prints them."""

    def __init__(self, as_json: bool, show_ids: Optional[list[int]] = None, show_flow: bool = False) -> None:
        self.as_json = as_json
        self.show_ids = show_ids
        self.show_flow = show_flow
        self.frame_count: int = 0

    def on_measurement(self, event: DeviceMeasurement) -> None:
        if event.kind != "uavcan":
            return
        meas = event.measurement
        mtype = meas.get("measurement_type", "")
        if self.show_ids is not None:
            node_id = meas.get("id", 0)
            if node_id not in self.show_ids:
                return
        if mtype == "flow" and not self.show_flow:
            return
        self.frame_count += 1
        frame = event.frame
        data = meas.get("data", {})
        dev_id = meas.get("id", 0)
        if self.as_json:
            print(json.dumps({
                "frame": self.frame_count,
                "id": dev_id,
                "can_id": f"0x{frame.arbitration_id:03X}",
                "type": mtype,
                "data": data,
            }, ensure_ascii=False), flush=True)
        else:
            if mtype == "range":
                range_mm = data.get("range_mm", 0)
                if isinstance(range_mm, float):
                    range_mm = round(range_mm, 1)
                print(f"{_ts()} [{self.frame_count}] id=0x{dev_id:02X} RANGE "
                      f"sensor={data.get('sensor_id', 0)} "
                      f"range={range_mm}mm "
                      f"quality={data.get('reading_type', 0)}", flush=True)
            elif mtype == "flow":
                print(f"{_ts()} [{self.frame_count}] id=0x{dev_id:02X} FLOW "
                      f"quality={data.get('quality', 0)}/255", flush=True)
            elif mtype == "node_status":
                print(f"{_ts()} [{self.frame_count}] id=0x{dev_id:02X} STATUS "
                      f"health={data.get('health', 0)} mode={data.get('mode', 0)}", flush=True)
            elif mtype == "allocation":
                print(f"{_ts()} [{self.frame_count}] id=0x{dev_id:02X} ALLOC "
                      f"granted={data.get('granted_node_id', 0)}", flush=True)
            else:
                print(f"{_ts()} [{self.frame_count}] id=0x{dev_id:02X} {mtype} {data}", flush=True)

    def on_status(self, event: StatusEvent) -> None:
        stream = sys.stderr if event.level in ("warning", "error") else sys.stdout
        print(f"{_ts()} [{event.level}] {event.source}: {event.message}", file=stream, flush=True)


class CsvRecorder:
    def __init__(self, path: Path, fieldnames: list[str], show_ids: Optional[list[int]] = None) -> None:
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
        if self._show_ids is not None and event.frame.arbitration_id not in self._show_ids:
            return
        row = _measurement_fields(event)
        self._writer.writerow(row)
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S.%f",
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="AFBR-S50 ToF CAN host (event-driven, pluggable).",
    )
    p.add_argument("--bus", default=DEFAULT_BUS,
                   choices=["auto", "socketcan", "waveshare", "gs_usb", "slcan", "uart"],
                   help="CAN bus backend. 'auto' detects via USB + SocketCAN.")
    p.add_argument("--index", type=int, default=0,
                   help="When multiple adapters match, pick this index (0-based).")
    p.add_argument("--channel", default=DEFAULT_CHANNEL,
                   help=f"Bus channel. Ignored when --bus=auto. "
                        f"Default for socketcan: {DEFAULT_CHANNEL}; for waveshare: COM port.")
    p.add_argument("--device", default=DEFAULT_DEVICE,
                    help=f"Device driver name (default: {DEFAULT_DEVICE} = all registered devices)")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bitrate in bps (default: {DEFAULT_BITRATE})")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                   help=f"Heartbeat/recv timeout in seconds (default: {DEFAULT_TIMEOUT})")
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
        help="UART line baud rate when --bus uart/slcan. Defaults to 2000000 "
         "(waveshare) or 115200 (uart/slcan) if not given.",
     )
    p.add_argument("--json", action="store_true", help="Print JSON instead of human-readable")
    p.add_argument("--csv", type=Path, help="Append measurements to this CSV file")
    p.add_argument("--csv-fields", default=DEFAULT_CSV_FIELDS,
                   help=f"CSV columns (default: {DEFAULT_CSV_FIELDS})")
    p.add_argument("--heartbeat", type=int, default=5,
                   help="Heartbeat interval in seconds when idle (default: 5)")
    p.add_argument("--send-start", type=int, default=3, metavar="N",
                   help="Send Start Remote Frame N times before monitoring (default: 3)")
    p.add_argument("--send-stop", type=int, default=3, metavar="N",
                   help="Send Stop Remote Frame N times after monitoring ends (default: 3)")
    p.add_argument("--start-id", default=f"0x{AFBRParser.FRAME_ID_START:02X}",
                   help=f"Start CAN ID (default: 0x{AFBRParser.FRAME_ID_START:02X})")
    p.add_argument("--stop-id", default=f"0x{AFBRParser.FRAME_ID_STOP:02X}",
                   help=f"Stop CAN ID (default: 0x{AFBRParser.FRAME_ID_STOP:02X})")
    p.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging")
    p.add_argument("--auto-alloc", action="store_true", default=True,
                   help="[uavcan] Perform dynamic node ID allocation on startup (default: on)")
    p.add_argument("--no-auto-alloc", action="store_false", dest="auto_alloc",
                   help="[uavcan] Disable dynamic node ID allocation")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Entry point — main() just wires things up and runs Host
# ---------------------------------------------------------------------------


def resolve_baudrate_if_needed(args: argparse.Namespace) -> int:
    """Pick a default UART baud rate when the user did not pass --baudrate."""
    if args.baudrate is not None:
        return int(args.baudrate)
    return 2_000_000 if args.bus == "waveshare" else 115_200


def main() -> int:
    """Parse args, register renderers, and run the host."""
    args = parse_args()
    setup_logging(args.verbose)

    if args.can_ids:
        AFBRParser.FRAME_ID_1D = parse_can_id_list(args.can_ids)

    show_ids: Optional[list[int]] = None
    if args.show_ids:
        show_ids = parse_can_id_list(args.show_ids)
    args.show_ids_list = show_ids

    args.baudrate = resolve_baudrate_if_needed(args)

    host = Host(args)

    rc = host.setup()
    if rc != 0:
        return rc

    printer = Printer(as_json=args.json, show_ids=show_ids)
    host.event_bus.on("measurement", printer.on_measurement)
    host.event_bus.on("status", printer.on_status)

    uavcan_printer = UAVCANPrinter(as_json=args.json, show_ids=show_ids, show_flow=args.show_flow)
    host.event_bus.on("measurement", uavcan_printer.on_measurement)
    host.event_bus.on("status", uavcan_printer.on_status)

    csv_recorder: Optional[CsvRecorder] = None
    if args.csv:
        csv_fields = [f.strip() for f in args.csv_fields.split(",") if f.strip()]
        csv_recorder = CsvRecorder(args.csv, csv_fields, show_ids=show_ids)
        host.event_bus.on("measurement", csv_recorder.on_measurement)

    try:
        return host.run()
    finally:
        if csv_recorder is not None:
            csv_recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
