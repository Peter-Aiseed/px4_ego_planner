"""USB device enumeration for CAN adapters.

Pure info layer: list every USB device with its VID/PID and, when known,
a suggested bus backend + channel. The CLI decides what to do with the
result.

Adding a new adapter family:

1. Add an entry to :data:`KNOWN_DEVICES` below (VID, PID range, backend,
   channel-template, label).
2. That's it. The autodetect path will pick it up automatically.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Module-level path roots so tests (and alternate roots) can override them.
SYS_BUS_USB_DEVICES = "/sys/bus/usb/devices"
SYS_CLASS_TTY = "/sys/class/tty"
DEV_ROOT = "/dev"


@dataclass(frozen=True)
class USBDevice:
    bus: int
    address: int
    vid: int
    pid: int
    manufacturer: str
    product: str
    serial: str
    port: str  # physical USB port path, e.g. "1-2.3"

    @property
    def vid_pid(self) -> str:
        return f"{self.vid:04x}:{self.pid:04x}"


@dataclass(frozen=True)
class AdapterMatch:
    device: USBDevice
    backend: str         # e.g. "gs_usb", "slcan", "socketcan"
    channel: str         # e.g. "can0", "/dev/ttyUSB0"
    label: str           # human-readable


# (vid, pid_or_None, backend, channel_template, label)
# channel_template may use {port} / {serial} or be a literal.
# If pid is None, the entry matches the whole vendor.
# For gs_usb, channel_template should produce an int index ({index});
# for slcan, it should produce a /dev/ttyUSB* path; for socketcan,
# any value is overridden at runtime.
_KNOWN: List[Tuple[int, Optional[int], str, str, str]] = [
    # CANable / candleLight / gs_usb family (used by CANable 2 in native mode).
    (0x16D0, None, "gs_usb", "{index}", "CANable/candleLight (gs_usb)"),
    # CH340-based adapters. 1a86:7523 is the WaveShare USB-CAN-A's PID; it
    # speaks the WaveShare binary protocol. Generic CH340 slcan modules exist
    # at other PIDs and can still be used with --bus slcan explicitly.
    (0x1A86, 0x7523, "waveshare", "{port}", "WaveShare USB-CAN-A (binary protocol)"),
    # PEAK PCAN-USB
    (0x0C72, None, "pcan", "PCAN_USBBUS1", "PEAK PCAN-USB"),
    # Kvaser Leaf
    (0x0BF8, None, "kvaser", "0", "Kvaser Leaf"),
    # CANtact
    (0x1D50, 0x6018, "cantact", "{port}", "CANtact"),
]


def _vid_pid_known(vid: int, pid: int) -> Optional[Tuple[str, str, str]]:
    for v, p, backend, tmpl, label in _KNOWN:
        if v == vid and (p is None or p == pid):
            return backend, tmpl, label
    return None


def list_usb_devices() -> List[USBDevice]:
    """Enumerate USB devices using pyusb. Empty list if unavailable / no perms."""
    try:
        import usb.core
        import usb.util
    except ImportError:
        logger.warning("pyusb not installed; USB enumeration disabled")
        return []

    devices: List[USBDevice] = []
    try:
        devs = usb.core.find(find_all=True)
    except Exception as exc:  # noqa: BLE001
        logger.warning("USB enumeration failed: %s", exc)
        return []

    for d in devs:
        try:
            vid = d.idVendor
            pid = d.idProduct
            manufacturer = ""
            product = ""
            serial = ""
            try:
                manufacturer = usb.util.get_string(d, d.iManufacturer) or ""
            except Exception:
                pass
            try:
                product = usb.util.get_string(d, d.iProduct) or ""
            except Exception:
                pass
            try:
                serial = usb.util.get_string(d, d.iSerialNumber) or ""
            except Exception:
                pass
            port = _format_port(d.bus, d.port_numbers) if d.port_numbers else ""
            devices.append(
                USBDevice(
                    bus=d.bus,
                    address=d.address,
                    vid=vid,
                    pid=pid,
                    manufacturer=manufacturer,
                    product=product,
                    serial=serial,
                    port=port,
                )
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("skipping USB device: %s", exc)
            continue
    return devices


def _format_port(bus: int, numbers: Iterable[int]) -> str:
    """Build the sysfs device path for a USB device, e.g. ``3-1.3``.

    ``lsusb`` reports a bus number + port path; sysfs encodes the same as
    ``<bus>-<port>.<subport>...`` (note the dots between levels), so we must
    include the bus number or the paths won't match anything under
    ``/sys/bus/usb/devices/``.
    """
    nums = [str(n) for n in numbers] if numbers else []
    return f"{bus}" + ("-" + ".".join(nums) if nums else "")


def resolve_serial_port(usb_port: str) -> Optional[str]:
    """Find the ``/dev/tty*`` node attached to a USB device (by port path).

    ``usb_port`` looks like ``3-1.3``. The kernel exposes the tty in one of
    (at least) two layouts depending on the bound driver, so instead of
    guessing the symlink layout we scan every ``/sys/class/tty/*/device``
    symlink and see which one points back to an interface of ``usb_port``.

    This handles cdc_acm (``3-1.3:1.0/tty/ttyACM0``), ch341
    (``3-1.3:1.0/ttyUSB0``) and the generic ``ch341``/``ftdi``/``pl2303``
    drivers all uniformly.
    """

    target_patterns = [f"{SYS_BUS_USB_DEVICES}/{usb_port}:1.0"]
    # Also accept deeper interfaces (e.g. 3-1.3:1.1).
    for i in range(16):
        target_patterns.append(f"{SYS_BUS_USB_DEVICES}/{usb_port}:1.{i}")
    # Normalize patterns to their real (PCI) form so the comparison works no
    # matter which bus layout (sysfs vs devices/pci…) the kernel reports.
    target_reals = {
        os.path.realpath(p) for p in target_patterns if os.path.exists(p)
    }
    # Also accept a match whose realpath *ends with* one of the short patterns,
    # e.g. .../3-1.3:1.0/ttyUSB0 starts-with the realpath of 3-1.3:1.0.
    target_short = f"{usb_port}:1.0"

    if not os.path.isdir(SYS_CLASS_TTY):
        return None

    for tty_name in os.listdir(SYS_CLASS_TTY):
        if not tty_name.startswith("tty"):
            continue
        device_link = f"{SYS_CLASS_TTY}/{tty_name}/device"
        if not os.path.islink(device_link):
            continue
        real = os.path.realpath(device_link)
        # Exact match (rare) OR prefix match: the tty's device path is a
        # descendant of one of the USB interface directories.
        if real in target_reals or any(
            real.startswith(rp + "/") or real.startswith(target_short)
            for rp in target_reals
        ):
            dev_path = f"{DEV_ROOT}/{tty_name}"
            if os.path.exists(dev_path):
                return dev_path
    return None


def detect_adapters() -> List[AdapterMatch]:
    """Return the list of detected CAN adapters, in enumeration order.

    Backend-specific notes:

    * ``gs_usb`` — channel is the integer index into ``gs_usb``'s device
      list. We assign indices in the order the matching USB devices are
      found (first match = index 0).
    * ``slcan`` — channel is the resolved ``/dev/ttyUSB*`` (or ``ttyACM*``)
      path of the device's USB port. If no tty is bound (driver not loaded
      or no permission), the match is skipped.
    """
    out: List[AdapterMatch] = []
    gs_usb_index = 0
    for d in list_usb_devices():
        hit = _vid_pid_known(d.vid, d.pid)
        if not hit:
            continue
        backend, tmpl, label = hit

        if backend == "gs_usb":
            # CANable/candleLight devices can run in two modes:
            #   * native gs_usb  → driver "gs_usb",  channel = integer index
            #   * serial/slcan   → driver "cdc_acm", channel = resolved /dev/ttyACM*
            driver = _interface_driver(d.port)
            if driver == "cdc_acm":
                tty = resolve_serial_port(d.port)
                if tty is None:
                    logger.debug("CANable %s is cdc_acm but no tty bound; skipping", d.vid_pid)
                    continue
                backend, channel, label = (
                    "slcan",
                    tty,
                    "CANable2 in serial/slcan mode (cdc_acm)",
                )
            else:
                channel = str(gs_usb_index)
                gs_usb_index += 1
        elif backend in ("slcan", "waveshare"):
            # Both CH340-based devices (WaveShare binary protocol + generic
            # slcan) are reached through a serial tty bound to the USB port.
            tty = resolve_serial_port(d.port)
            if tty is None:
                logger.debug("%s %s has no /dev/tty*; skipping", d.vid_pid, backend)
                continue
            channel = tty
            if backend == "slcan":
                label = label or f"CH340 slcan ({d.vid_pid})"
        else:
            channel = tmpl.format(port=d.port, serial=d.serial)
        out.append(AdapterMatch(device=d, backend=backend, channel=channel, label=label))
    return out


def _interface_driver(usb_port: str) -> Optional[str]:
    """Return the kernel driver bound to the device's first interface, or None.

    ``usb_port`` looks like ``3-1.3``. We look at ``<port>:1.0`` and read
    the ``driver`` symlink, which points at
    ``/sys/bus/usb/drivers/<driver>``.
    """

    link = f"{SYS_BUS_USB_DEVICES}/{usb_port}:1.0/driver"
    if not os.path.exists(link):
        return None
    try:
        target = os.path.realpath(link)
    except (FileNotFoundError, OSError):
        return None
    return os.path.basename(target) if target else None


# ---------------------------------------------------------------------------
# UART CAN adapter enumeration (e.g. MCP2515-on-UART on a Jetson).
# ---------------------------------------------------------------------------


def list_uart_can_devices() -> List[AdapterMatch]:
    """Return UART-based CAN adapters (e.g. MCP2515-on-UART on a Jetson/TI).

    A UART CAN adapter is a serial port wired to an MCP2515-style CAN
    controller running the slcan protocol. We only enumerate ports that are
    commonly dedicated to peripheral use on embedded boards (Jetson
    ``ttyTHS*``, TI ``ttyO*``), since generic x86 ``ttyS*`` ports are almost
    never CAN devices.
    """
    import re

    out: List[AdapterMatch] = []
    if not os.path.isdir(DEV_ROOT):
        return out
    for base in sorted(os.listdir(DEV_ROOT)):
        m = re.match(r"^tty(THS|O)(\d+)$", base)
        if not m:
            continue
        full = os.path.join(DEV_ROOT, base)
        if os.path.exists(full):
            out.append(
                AdapterMatch(
                    device=None,  # type: ignore[arg-type]
                    backend="uart",
                    channel=full,
                    label=f"UART CAN ({full})",
                )
            )
    return out


# ---------------------------------------------------------------------------
# SocketCAN interface enumeration (separate from USB; uses ``ip link``).
# ---------------------------------------------------------------------------


def list_socketcan_interfaces() -> List[str]:
    """Return names of all active SocketCAN interfaces, e.g. ['can0', 'vcan1'].

    We parse ``ip -o link show`` and keep only lines whose ``link/`` field
    is a CAN device (``link/can``). This avoids accidentally matching
    unrelated interface names like ``lo`` or ``wlp1s0``.
    """
    import subprocess

    try:
        out = subprocess.run(
            ["ip", "-o", "link", "show"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except Exception as exc:  # noqa: BLE001
        logger.warning("ip link failed: %s", exc)
        return []
    if out.returncode != 0:
        return []

    names: List[str] = []
    seen: set[str] = set()
    for line in out.stdout.splitlines():
        # Line format: "2: can0: <...> ... link/can ..."
        parts = line.split(":", 2)
        if len(parts) < 2:
            continue
        name = parts[1].strip()
        if not name or name in seen:
            continue
        # Only keep it if the line actually contains a CAN link layer.
        if "link/can" in line:
            seen.add(name)
            names.append(name)
    return names


def print_summary() -> str:
    """Build a human-readable summary used by ``--list``."""
    lines: List[str] = []
    lines.append("USB devices:")
    devs = list_usb_devices()
    if not devs:
        lines.append("  (none / pyusb missing or no permission — try sudo)")
    for d in devs:
        known = _vid_pid_known(d.vid, d.pid)
        tag = f"  -> {known[2]}" if known else ""
        lines.append(
            f"  {d.vid_pid} bus={d.bus} addr={d.address} port={d.port} "
            f"{d.manufacturer} {d.product} (serial={d.serial or '-'}){tag}"
        )

    lines.append("")
    lines.append("Detected CAN adapters (USB + UART):")
    for m in detect_adapters() + list_uart_can_devices():
        vid_pid = m.device.vid_pid if m.device is not None else "(uart)"
        lines.append(
            f"  {vid_pid} backend={m.backend} channel={m.channel}  ({m.label})"
        )

    sc = list_socketcan_interfaces()
    lines.append("")
    lines.append("Active SocketCAN interfaces:")
    lines.append("  " + (", ".join(sc) if sc else "(none)"))
    return "\n".join(lines)


def resolve_uart_channel() -> Optional[str]:
    """Pick a serial port for --bus uart/slcan when the user did not set --channel.

    Preference order:
    1. A /dev/ttyTHS* (Jetson dedicated UART CAN pin).
    2. A /dev/ttyACM* / /dev/ttyUSB* reported by the USB scanner (CANable2
       in cdc_acm mode, CH340, …).
    3. Direct filesystem scan for /dev/ttyACM* and /dev/ttyUSB* (fallback
       when pyusb is not installed).
    4. Any other /dev/tty{THS,O,S}<N> that is not the system console.
    """
    for tty in sorted(os.listdir("/dev")) if os.path.isdir("/dev") else []:
        if tty.startswith("ttyTHS"):
            return os.path.join("/dev", tty)

    for m in detect_adapters():
        if m.backend in ("slcan", "uart"):
            return m.channel

    # Direct filesystem scan for ACM/USB serial (works even without pyusb)
    for tty in sorted(os.listdir("/dev")) if os.path.isdir("/dev") else []:
        if tty.startswith("ttyACM") or tty.startswith("ttyUSB"):
            return os.path.join("/dev", tty)

    for m in list_uart_can_devices():
        return m.channel

    for tty in sorted(os.listdir("/dev")) if os.path.isdir("/dev") else []:
        full = os.path.join("/dev", tty)
        if tty.startswith("ttyO") and os.path.exists(full):
            return full
        if tty.startswith("ttyS") and not tty.startswith("ttyS0") and os.path.exists(full):
            return full
    return None


def main() -> None:
    """CLI entry point: print a human-readable USB / CAN adapter summary."""
    print(print_summary())


if __name__ == "__main__":
    main()
