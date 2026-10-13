"""Enumeration of the CAN adapters this host can talk to.

One rule, one list: :func:`list_candidates` returns every adapter that could
be opened, in the order they should be tried — USB adapters first, then
SocketCAN interfaces, then on-board UART CAN ports. Both selection paths
derive from that single list: ``--bus auto`` walks it, and ``--bus <name>``
filters it, so ``--index N`` names the same physical adapter whichever
spelling is used. (They used to be built from two separate enumeration paths
and drifted apart — ``--index`` was silently ignored for concrete buses.)

Adapter knowledge lives in :data:`KNOWN_ADAPTERS` and nowhere else: adding a
new adapter family means adding one row there. This module is a pure info
layer — it never opens a bus.
"""

from __future__ import annotations

import logging
import os
import re
import subprocess
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)

# Module-level path roots so tests (and alternate roots) can override them.
SYS_BUS_USB_DEVICES = "/sys/bus/usb/devices"
SYS_CLASS_TTY = "/sys/class/tty"
DEV_ROOT = "/dev"

# Line rate for serial adapters we have no known rate for.
DEFAULT_SERIAL_BAUDRATE = 115_200

# Embedded boards wire CAN controllers to peripheral UARTs. Generic x86
# ttyS* ports are almost never CAN, so only these prefixes are considered.
_UART_CAN_RE = re.compile(r"^tty(?:THS|O)\d+$")

# Adapter backend names that differ from the ``--bus`` name that opens them.
# A UART CAN port is an MCP2515 running slcan over a serial line, so it is
# opened by the same backend as a USB slcan adapter.
BACKEND_BUS_NAMES: dict[str, str] = {"slcan": "slcan_serial", "uart": "slcan_serial"}


class ChannelKind(Enum):
    """How a known adapter's :attr:`AdapterMatch.channel` is derived."""

    INDEX = "index"
    """A position in the backend's own device list (``gs_usb``)."""
    TTY = "tty"
    """The ``/dev/tty*`` node bound to the device's USB interface."""
    LITERAL = "literal"
    """A fixed string, for devices addressed by name or number."""


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
    """One CAN adapter that could be opened.

    ``device`` is the USB device it was found on, or None when the source
    carries no USB metadata: SocketCAN interfaces, on-board UART CAN ports,
    and adapters recovered from sysfs without pyusb.
    """

    device: USBDevice | None
    backend: str  # adapter backend, e.g. "gs_usb", "waveshare", "slcan"
    channel: str  # e.g. "can0", "0", "/dev/ttyUSB0"
    label: str  # human-readable


@dataclass(frozen=True)
class KnownAdapter:
    """One row of the adapter knowledge table.

    ``pid=None`` matches every product of the vendor. ``backend=None`` marks a
    device we can identify but have no bus implementation for: it is reported
    by ``--list`` but never offered as a candidate. ``baudrate`` is the UART
    line rate, and None both for native (non-serial) adapters and for serial
    adapters with no recorded rate — :func:`resolve_baudrate` turns either
    into :data:`DEFAULT_SERIAL_BAUDRATE`.
    """

    vid: int
    pid: int | None
    backend: str | None
    label: str
    channel_kind: ChannelKind
    channel: str = ""  # used when channel_kind is LITERAL
    baudrate: int | None = None

    def matches(self, vid: int, pid: int) -> bool:
        """Whether this row covers the given USB ``vid``/``pid``."""
        return self.vid == vid and (self.pid is None or self.pid == pid)


KNOWN_ADAPTERS: tuple[KnownAdapter, ...] = (
    # CANable/candleLight (the gs_usb family) runs in one of two modes: the
    # kernel gs_usb driver claims it and it is addressed by list index, or
    # cdc_acm claims it and the same board speaks slcan over a tty.
    KnownAdapter(
        0x16D0, None, "gs_usb", "CANable/candleLight", ChannelKind.INDEX
    ),
    # CH340-based WaveShare USB-CAN-A: WaveShare binary protocol over UART at
    # 2 Mbps.
    KnownAdapter(
        0x1A86,
        0x7523,
        "waveshare",
        "WaveShare USB-CAN-A (CH340)",
        ChannelKind.TTY,
        baudrate=2_000_000,
    ),
    # FT232R-based revision of the same board: same protocol, 1.5 Mbps.
    KnownAdapter(
        0x0403,
        0x6001,
        "waveshare",
        "WaveShare USB-CAN-A (FT232R)",
        ChannelKind.TTY,
        baudrate=1_500_000,
    ),
    # CANtact: an MCP2515 running the slcan protocol over a cdc_acm tty.
    KnownAdapter(
        0x1D50,
        0x6018,
        "slcan",
        "CANtact",
        ChannelKind.TTY,
        baudrate=115_200,
    ),
    # Identified and reported by --list, but no bus implementation here.
    KnownAdapter(0x0C72, None, None, "PEAK PCAN-USB", ChannelKind.LITERAL, "PCAN_USBBUS1"),
    KnownAdapter(0x0BF8, None, None, "Kvaser Leaf", ChannelKind.LITERAL, "0"),
)


def lookup_known_adapter(vid: int, pid: int) -> KnownAdapter | None:
    """The first :data:`KNOWN_ADAPTERS` row covering ``vid``/``pid``, else None."""
    for known in KNOWN_ADAPTERS:
        if known.matches(vid, pid):
            return known
    return None


def known_serial_baudrate(vid: int, pid: int) -> int | None:
    """UART line rate recorded for a known adapter VID/PID, else None.

    Vendor-wide rows (``pid=None``) match any PID. Returns None for native
    (non-serial) adapters and for devices we don't recognize, so callers fall
    back to :data:`DEFAULT_SERIAL_BAUDRATE`.
    """
    known = lookup_known_adapter(vid, pid)
    return known.baudrate if known is not None else None


def bus_name_for_backend(backend: str) -> str:
    """The ``--bus`` name that opens ``backend``'s adapters."""
    return BACKEND_BUS_NAMES.get(backend, backend)


# ---------------------------------------------------------------------------
# USB enumeration
# ---------------------------------------------------------------------------


def _format_port(bus: int, numbers: Iterable[int]) -> str:
    """Build the sysfs device path for a USB device, e.g. ``3-1.3``.

    ``lsusb`` reports a bus number + port path; sysfs encodes the same as
    ``<bus>-<port>.<subport>...`` (note the dots between levels), so we must
    include the bus number or the paths won't match anything under
    ``/sys/bus/usb/devices/``.
    """
    nums = [str(n) for n in numbers] if numbers else []
    return f"{bus}" + ("-" + ".".join(nums) if nums else "")


def _usb_string(dev: Any, index: int) -> str:
    """One USB string descriptor, or ``""`` when absent or unreadable.

    Reading a descriptor opens the device, which fails for adapters the
    current user has no permission for. A missing vendor string must not hide
    the device from detection, so failures are swallowed here rather than
    losing the whole match.
    """
    try:
        import usb.util

        return usb.util.get_string(dev, index) or ""
    except Exception as exc:  # noqa: BLE001
        logger.debug("USB string descriptor %d unreadable: %s", index, exc)
        return ""


def list_usb_devices() -> list[USBDevice]:
    """Enumerate USB devices using pyusb. Empty if unavailable / no perms.

    Needs access to the raw USB device nodes, so it reports nothing for an
    unprivileged user on a host without udev rules. Callers that must work
    anyway use the sysfs path in :func:`_adapters_from_sysfs`.
    """
    try:
        import usb.core
    except ImportError:
        logger.warning("pyusb not installed; USB detection falls back to sysfs")
        return []

    try:
        devs = usb.core.find(find_all=True)
    except Exception as exc:  # noqa: BLE001
        logger.warning("USB enumeration failed: %s", exc)
        return []

    devices: list[USBDevice] = []
    for d in devs:
        try:
            devices.append(
                USBDevice(
                    bus=d.bus,
                    address=d.address,
                    vid=d.idVendor,
                    pid=d.idProduct,
                    manufacturer=_usb_string(d, d.iManufacturer),
                    product=_usb_string(d, d.iProduct),
                    serial=_usb_string(d, d.iSerialNumber),
                    port=_format_port(d.bus, d.port_numbers) if d.port_numbers else "",
                )
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("skipping USB device: %s", exc)
    return devices


def _interface_driver(usb_port: str) -> str | None:
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


def _iter_class_ttys(prefixes: tuple[str, ...]) -> Iterator[str]:
    """Kernel tty names under :data:`SYS_CLASS_TTY` starting with ``prefixes``.

    Sorted, so every caller walks them in the same order and a detection
    result never depends on directory-listing order.
    """
    if not os.path.isdir(SYS_CLASS_TTY):
        return
    for name in sorted(os.listdir(SYS_CLASS_TTY)):
        if name.startswith(prefixes):
            yield name


def resolve_serial_port(usb_port: str) -> str | None:
    """Find the ``/dev/tty*`` node attached to a USB device (by port path).

    ``usb_port`` looks like ``3-1.3``. The kernel exposes the tty in one of
    (at least) two layouts depending on the bound driver, so instead of
    guessing the symlink layout we scan every ``/sys/class/tty/*/device``
    symlink and see which one points back to an interface of ``usb_port``.

    This handles cdc_acm (``3-1.3:1.0/tty/ttyACM0``), ch341
    (``3-1.3:1.0/ttyUSB0``) and the generic ``ch341``/``ftdi``/``pl2303``
    drivers all uniformly. Returns None when no tty is bound or no ``/dev``
    node exists for it.
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
    if not target_reals:
        return None
    # Also accept a match whose realpath *ends with* one of the short patterns,
    # e.g. .../3-1.3:1.0/ttyUSB0 starts-with the realpath of 3-1.3:1.0.
    target_short = f"{usb_port}:1.0"

    for tty_name in _iter_class_ttys(("tty",)):
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


def _read_hex_file(path: str) -> int | None:
    """Return the value of a sysfs hex attribute, or None when missing/malformed."""
    try:
        with open(path, encoding="ascii") as f:
            return int(f.read().strip(), 16)
    except (OSError, ValueError):
        return None


def _usb_ids_for_tty(tty_name: str) -> tuple[int, int] | None:
    """Return the ``(vid, pid)`` of the USB device owning a serial tty.

    ``tty_name`` is a bare kernel tty name (``ttyUSB2``).  Walks up from
    ``/sys/class/tty/<name>/device`` and reads ``idVendor`` / ``idProduct``
    from the first ancestor directory that has both.  Handles two sysfs
    layouts:

    * PCI-tree layout: ``.../usb2/2-1.3/2-1.3:1.0/ttyUSB0`` — the device dir
      is an ancestor of the tty node.
    * ``/sys/bus/usb/devices`` layout: ``.../devices/2-1.3:1.0`` — the
      device dir ``2-1.3`` is a *sibling* of the interface dir, reached by
      trimming the ``:1.0`` suffix.

    Requires no pyusb and no USB device permissions — only readable sysfs.
    Returns None when the tty is not backed by a USB device (e.g. a UART
    port) or the ids are unreadable.
    """
    device_link = os.path.join(SYS_CLASS_TTY, tty_name, "device")
    if not os.path.islink(device_link):
        return None
    probe = os.path.realpath(device_link)
    while probe and probe != os.path.dirname(probe):
        basename = os.path.basename(probe)
        candidates = [probe]
        if ":" in basename:
            candidates.append(
                os.path.join(os.path.dirname(probe), basename.split(":", 1)[0])
            )
        for cand in candidates:
            vid = _read_hex_file(os.path.join(cand, "idVendor"))
            pid = _read_hex_file(os.path.join(cand, "idProduct"))
            if vid is not None and pid is not None:
                return vid, pid
        probe = os.path.dirname(probe)
    return None


def _tty_by_usb_ids(vid: int, pid: int) -> str | None:
    """Return the ``/dev/tty*`` node owned by a USB device with these VID/PID.

    Walks ``/sys/class/tty`` and compares each serial tty's owning USB device
    (see :func:`_usb_ids_for_tty`).  Returns None when no tty is bound to a
    matching device, or when the matching ``/dev`` node does not exist.
    """
    for tty_name in _iter_class_ttys(("ttyUSB", "ttyACM")):
        if _usb_ids_for_tty(tty_name) != (vid, pid):
            continue
        full = os.path.join(DEV_ROOT, tty_name)
        return full if os.path.exists(full) else None
    return None


# ---------------------------------------------------------------------------
# Adapter detection
# ---------------------------------------------------------------------------


def _canable_serial_channel(device: USBDevice) -> str | None:
    """The tty of a CANable that is in serial mode, else None.

    The CANable row is addressed by gs_usb list index, but the same board can
    be claimed by cdc_acm instead and then speaks slcan over a tty, so in that
    mode its channel is the tty rather than the index.
    """
    if _interface_driver(device.port) != "cdc_acm":
        return None
    return resolve_serial_port(device.port)


def _adapters_from_pyusb(devices: list[USBDevice]) -> list[AdapterMatch]:
    """Match enumerated USB devices against the known-adapter table."""
    matches: list[AdapterMatch] = []
    gs_usb_index = 0
    for device in devices:
        known = lookup_known_adapter(device.vid, device.pid)
        if known is None or known.backend is None:
            continue
        backend = known.backend
        label = known.label
        channel: str | None
        if known.channel_kind is ChannelKind.TTY:
            channel = resolve_serial_port(device.port)
            if channel is None:
                logger.debug("%s %s has no /dev/tty*; skipping", device.vid_pid, label)
                continue
        elif known.channel_kind is ChannelKind.INDEX:
            serial = _canable_serial_channel(device)
            if serial is None:
                channel = str(gs_usb_index)
                gs_usb_index += 1
                label = f"{label} (gs_usb native)"
            else:
                backend, channel = "slcan", serial
                label = f"{label} (serial/slcan mode)"
        else:
            channel = known.channel
        matches.append(
            AdapterMatch(device=device, backend=backend, channel=channel, label=label)
        )
    return matches


def _adapters_from_sysfs() -> list[AdapterMatch]:
    """Find serial adapters by walking sysfs for known VID/PIDs.

    Needs no pyusb and no USB device permissions, only readable sysfs, so it
    still finds the adapters when :func:`list_usb_devices` cannot. It cannot
    see native USB CAN adapters — no tty is bound, and the gs_usb device order
    is a userspace list, not a sysfs walk — so only :attr:`ChannelKind.TTY`
    rows are recoverable, in table order.
    """
    matches: list[AdapterMatch] = []
    for known in KNOWN_ADAPTERS:
        if known.backend is None or known.pid is None:
            continue
        if known.channel_kind is not ChannelKind.TTY:
            continue
        channel = _tty_by_usb_ids(known.vid, known.pid)
        if channel is None:
            continue
        matches.append(
            AdapterMatch(
                device=None, backend=known.backend, channel=channel, label=known.label
            )
        )
    return matches


def detect_adapters(devices: list[USBDevice] | None = None) -> list[AdapterMatch]:
    """Detected USB CAN adapters, in enumeration order.

    Uses pyusb when it can see the bus and falls back to a pure-sysfs
    VID/PID walk when it cannot, so both paths find the same adapters. The
    fallback only runs when pyusb reported no device at all: a working pyusb
    that recognises nothing means the table is out of date, not that the
    devices are hidden.

    ``devices`` passes an already-enumerated list so a caller that needs the
    raw devices too does not ask the bus twice; None enumerates now.

    A serial adapter with no tty bound (no driver loaded, or no permission)
    is skipped rather than offered as a channel that cannot be opened.
    """
    devs = list_usb_devices() if devices is None else devices
    matches = _adapters_from_pyusb(devs)
    if not matches and not devs:
        matches = _adapters_from_sysfs()
    return matches


# ---------------------------------------------------------------------------
# SocketCAN interface enumeration (separate from USB; uses ``ip link``).
# ---------------------------------------------------------------------------


def list_socketcan_interfaces() -> list[str]:
    """Return names of all active SocketCAN interfaces, e.g. ['can0', 'vcan1'].

    We parse ``ip -o link show`` and keep only lines whose ``link/`` field
    is a CAN device (``link/can``). This avoids accidentally matching
    unrelated interface names like ``lo`` or ``wlp1s0``.
    """
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

    names: list[str] = []
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


# ---------------------------------------------------------------------------
# UART CAN adapter enumeration (e.g. MCP2515-on-UART on a Jetson).
# ---------------------------------------------------------------------------


def list_uart_can_devices() -> list[AdapterMatch]:
    """Return UART-based CAN adapters (e.g. MCP2515-on-UART on a Jetson/TI).

    A UART CAN adapter is a serial port wired to an MCP2515-style CAN
    controller running the slcan protocol, so these are opened by the
    ``slcan_serial`` backend. Only ports commonly dedicated to peripheral use
    on embedded boards are enumerated, since generic x86 ``ttyS*`` ports are
    almost never CAN devices.
    """
    if not os.path.isdir(DEV_ROOT):
        return []
    matches: list[AdapterMatch] = []
    for name in sorted(os.listdir(DEV_ROOT)):
        if not _UART_CAN_RE.match(name):
            continue
        port = os.path.join(DEV_ROOT, name)
        if not os.path.exists(port):
            continue
        matches.append(
            AdapterMatch(
                device=None,
                backend="uart",
                channel=port,
                label=f"UART CAN ({port})",
            )
        )
    return matches


# ---------------------------------------------------------------------------
# The candidate pipeline both selection paths share
# ---------------------------------------------------------------------------


def list_candidates(devices: list[USBDevice] | None = None) -> list[AdapterMatch]:
    """Every CAN adapter this host could open, in the order to try them.

    USB adapters, then SocketCAN interfaces, then on-board UART CAN ports.
    ``--bus auto`` walks this list (see :class:`can_host.can_bus.auto.AutoCANBus`)
    and ``--bus <name>`` filters it, so ``--index N`` selects the same adapter
    either way. ``devices`` passes an already-enumerated USB list to avoid
    asking the bus twice.
    """
    return [
        *detect_adapters(devices),
        *(
            AdapterMatch(
                device=None,
                backend="socketcan",
                channel=name,
                label=f"SocketCAN ({name})",
            )
            for name in list_socketcan_interfaces()
        ),
        *list_uart_can_devices(),
    ]


def channels_for_bus(bus: str) -> list[str]:
    """Channels ``--bus <bus>`` would try, in :func:`list_candidates` order.

    De-duplicated, because one port can be reachable by more than one rule
    (an on-board UART CAN port is a UART candidate and a ``slcan_serial``
    candidate at once).
    """
    seen: set[str] = set()
    channels: list[str] = []
    for match in list_candidates():
        if bus_name_for_backend(match.backend) != bus:
            continue
        if match.channel in seen:
            continue
        seen.add(match.channel)
        channels.append(match.channel)
    return channels


def resolve_channel_for_bus(bus: str, index: int = 0) -> str | None:
    """The channel ``--bus <bus> --index N`` would open, or None.

    ``index`` is ``--index`` (0-based) and counts that bus's own candidates in
    :func:`list_candidates` order, which is the order ``--bus auto`` tries
    them in, so the two select the same adapter.

    Returns None — leaving it to the caller to report — when ``bus`` is
    ``auto`` (it picks its own adapter while opening), when nothing is
    detected for the bus, or when ``index`` is out of range.
    """
    if bus == "auto":
        return None
    channels = channels_for_bus(bus)
    if not 0 <= index < len(channels):
        if channels:
            logger.warning(
                "--index %d is out of range: %d %s candidate(s) detected (%s).",
                index,
                len(channels),
                bus,
                ", ".join(channels),
            )
        return None
    channel = channels[index]
    logger.info("Auto-resolved %s channel: %s", bus, channel)
    return channel


def resolve_baudrate(
    channel: str | None,
    override: int | None = None,
    vid_pid: tuple[int, int] | None = None,
) -> int:
    """The single place the UART line rate for ``channel`` is decided.

    Precedence:

    1. ``override`` — an explicit ``--baudrate``; never second-guessed.
    2. ``vid_pid`` — the USB device the caller already has, from pyusb
       enumeration.
    3. ``channel`` — the USB ids behind the port, read from sysfs. This is the
       per-adapter lookup, so two revisions of the same adapter plugged in
       together each get their own rate.
    4. :data:`DEFAULT_SERIAL_BAUDRATE` for anything unrecognised, for a
       non-USB port (an embedded UART CAN port) or no port at all — ``--bus
       auto`` has not chosen its adapter yet.

    The rate follows the *device*, never "whichever adapter was found first".
    Never raises; an undetectable port falls back to the default.
    """
    if override is not None:
        return override
    known: int | None = None
    if vid_pid is not None:
        known = known_serial_baudrate(*vid_pid)
    if known is None and channel:
        ids = _usb_ids_for_tty(os.path.basename(channel))
        if ids is not None:
            known = known_serial_baudrate(*ids)
    return known if known is not None else DEFAULT_SERIAL_BAUDRATE


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def print_summary() -> str:
    """Build the human-readable summary used by ``--list``."""
    devices = list_usb_devices()
    candidates = list_candidates(devices)

    lines = ["USB devices:"]
    if not devices:
        lines.append("  (none via pyusb — missing or no permission; trying sysfs)")
    for d in devices:
        known = lookup_known_adapter(d.vid, d.pid)
        tag = f" -> {known.label}" if known is not None else ""
        lines.append(
            f"  {d.vid_pid} bus={d.bus} addr={d.address} port={d.port} "
            f"{d.manufacturer} {d.product} (serial={d.serial or '-'}){tag}"
        )

    lines.append("")
    lines.append("CAN adapters (in --bus auto order; --index counts this list):")
    if not candidates:
        lines.append("  (none)")
    for m in candidates:
        ids = m.device.vid_pid if m.device is not None else "-"
        lines.append(
            f"  {ids:>9}  backend={m.backend} channel={m.channel}  ({m.label})"
        )
    return "\n".join(lines)


def main() -> None:
    """CLI entry point: print a human-readable USB / CAN adapter summary."""
    print(print_summary())


if __name__ == "__main__":
    main()
