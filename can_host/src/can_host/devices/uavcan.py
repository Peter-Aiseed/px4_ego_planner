"""UAVCAN/DroneCAN device.

Parses UAVCAN messages (NodeStatus, Range Sensor Measurement, Flow Measurement)
using the DroneCAN library's TransferManager and Transfer.from_frames().
Emits DeviceMeasurement events on the shared EventBus.

Optionally performs dynamic node ID allocation at startup using the
persistent :mod:`uavcan_node_ids` store so the same device always gets
the same node ID across power cycles.
"""

from __future__ import annotations

import logging
import sys
import time
from typing import Any, Optional

from ..can_bus.base import CANBus
from ..events import CANFrame, DeviceMeasurement, EventBus, StatusEvent
from .base import Device
from ._uavcan_node_ids import NodeIDStore

logger = logging.getLogger(__name__)


class UAVCANDevice(Device):
    """UAVCAN/DroneCAN device driver.

    Subscribes to all frames on the bus, reassembles multi-frame transfers
    using the DroneCAN library, and dispatches known message types.

    When ``auto_alloc=True`` (default) the device will, on attach, listen
    for anonymous Allocation requests from unassigned nodes, collect their
    unique IDs, and reply with a Grant carrying the node ID from the
    persistent store (or a new one if the UID is unknown).  This keeps
    node IDs stable across power cycles.
    """

    ALLOCATOR_NODE_ID = 127
    ALLOC_TIMEOUT = 15.0

    def __init__(
        self,
        device_id: str = "uavcan",
        event_bus: Optional[EventBus] = None,
        auto_alloc: bool = True,
        node_id_store: Optional[NodeIDStore] = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(device_id=device_id, event_bus=event_bus)
        self._transfer_manager = None
        self._dronecan = None
        self._auto_alloc = auto_alloc
        self._store = node_id_store if node_id_store is not None else NodeIDStore()
        self._alloc_state: Optional[dict] = None
        self._init_dronecan()

    def _init_dronecan(self) -> None:
        """Import DroneCAN library lazily."""
        sys.path.insert(0, '.venv/lib/python3.12/site-packages')
        import dronecan
        from dronecan.transport import TransferManager
        self._dronecan = dronecan
        self._transfer_manager = TransferManager()

    def _on_attached(self) -> None:
        """If auto_alloc is enabled, start listening for anonymous Allocation requests."""
        if not self._auto_alloc:
            return
        self._alloc_state = {
            "received_uid": bytearray(),
            "transfer_id": 0,
            "start_time": time.time(),
            "active": True,
        }
        used = self._store.used_ids
        logger.info("UAVCAN dynamic node ID allocation enabled (allocator node %d)",
                    self.ALLOCATOR_NODE_ID)
        if used:
            logger.info("Previously used node IDs (will be skipped): %s",
                        sorted(used))
        else:
            logger.info("No previously used node IDs")

    def _on_frame(self, frame: CANFrame) -> None:
        """Process a CAN frame: reassemble multi-frame transfers and dispatch."""
        logger.debug("UAVCAN _on_frame: can_id=0x%08X dlc=%d data=%s",
                     frame.arbitration_id, frame.dlc, frame.data.hex())
        if self._transfer_manager is None:
            logger.debug("UAVCAN _on_frame: transfer_manager is None, skipping")
            return

        # If allocation is active, check for anonymous Allocation requests first
        if self._alloc_state is not None:
            consumed = self._try_allocation(frame)
            if consumed:
                return  # frame was handled by allocation logic

        # Create DroneCAN Frame object
        from dronecan.transport import Frame as DroneFrame
        drone_frame = DroneFrame(frame.arbitration_id, frame.data)

        # Reassemble multi-frame transfers
        transfer_frames = self._transfer_manager.receive_frame(drone_frame)
        if not transfer_frames:
            logger.debug("UAVCAN _on_frame: transfer_manager returned None (incomplete transfer)")
            return

        # Decode the message
        try:
            from dronecan.transport import Transfer, get_dronecan_data_type
            transfer = Transfer()
            transfer.from_frames(transfer_frames)
        except Exception as exc:
            logger.debug("Transfer decode failed: %s", exc)
            return

        source_node = transfer.source_node_id
        payload = transfer.payload
        payload_type = get_dronecan_data_type(payload)
        logger.debug("UAVCAN _on_frame: decoded payload_type=%s source=%d",
                     payload_type.full_name, source_node)

        # Dispatch by message type
        if payload_type == self._dronecan.uavcan.protocol.dynamic_node_id.Allocation:
            self._emit_allocation(payload, frame)
        elif payload_type == self._dronecan.uavcan.equipment.range_sensor.Measurement:
            self._emit_range_measurement(payload, frame)
        elif payload_type == self._dronecan.uavcan.protocol.NodeStatus:
            self._emit_node_status(payload, frame)
        elif payload_type == self._dronecan.com.hex.equipment.flow.Measurement:
            self._emit_flow_measurement(payload, frame)
        else:
            logger.debug("Unhandled UAVCAN message type: %s", payload_type.full_name)

    # ------------------------------------------------------------------
    # Dynamic node ID allocation
    # ------------------------------------------------------------------

    def _try_allocation(self, frame: CANFrame) -> bool:
        """Inspect a raw frame for an anonymous Allocation request.

        Returns True if the frame was an Allocation request and was handled.
        """
        state = self._alloc_state
        if state is None or not state["active"]:
            return False

        # Timeout?
        if time.time() - state["start_time"] > self.ALLOC_TIMEOUT:
            logger.info("Allocation timeout — giving up on %s", self.device_id)
            state["active"] = False
            self._alloc_state = None
            return False

        can_id = frame.arbitration_id
        data = frame.data
        if len(data) < 2:
            return False

        # Anonymous Allocation request: source=0, message type=1
        # Same filter as set_node_id.py line 71
        if (can_id & 0xFF) != 0 or ((can_id >> 8) & 0x03) != 1:
            return False

        # First byte = node_id + Q flag; tail byte = transfer flags + transfer_id
        uid_part = data[1:-1]  # strip first byte and tail byte
        if not uid_part:
            return False

        if len(state["received_uid"]) == 0:
            logger.info("Received first-stage Allocation request from anonymous node")
            state["received_uid"].extend(uid_part)
        else:
            if uid_part not in state["received_uid"]:
                state["received_uid"].extend(uid_part)
                logger.info("Received UID continuation, length now %d", len(state["received_uid"]))

        # Build the reply
        from dronecan.transport import Transfer
        from dronecan import uavcan

        if len(state["received_uid"]) >= 16:
            uid_hex = bytes(state["received_uid"][:16]).hex()
            node_id = self._store.get(uid_hex)
            if node_id is None:
                node_id = self._store.find_unused_node_id()
                self._store.set(uid_hex, node_id)
                logger.info("[%s] Allocated NEW node ID %d for UID %s",
                            time.strftime("%H:%M:%S"), node_id, uid_hex)
            else:
                logger.info("[%s] Reusing existing node ID %d for UID %s",
                            time.strftime("%H:%M:%S"), node_id, uid_hex)

            msg = uavcan.protocol.dynamic_node_id.Allocation()
            msg.node_id = node_id
            msg.first_part_of_unique_id = False
            msg.unique_id = bytes(state["received_uid"][:16])

            transfer = Transfer(
                payload=msg,
                source_node_id=self.ALLOCATOR_NODE_ID,
                transfer_id=state["transfer_id"],
                transfer_priority=20,
                service_not_message=False,
            )
            frames = transfer.to_frames()
            logger.info("Sending %d grant frame(s) for node_id=%d", len(frames), node_id)
            for f in frames:
                logger.info("  → Grant frame: can_id=0x%08X dlc=%d data=%s",
                            f.message_id, len(f.bytes), f.bytes.hex())
                self._send_can_frame(f.message_id, f.bytes)

            self._emit_measurement(
                {
                    "measurement_type": "allocation",
                    "data": {
                        "node_id": node_id,
                        "unique_id": uid_hex,
                        "action": "granted",
                    },
                },
                frame,
                kind="uavcan",
            )
            state["active"] = False
            self._alloc_state = None
            return True
        else:
            # Reply with query (node_id=0, Q=0, partial UID)
            logger.info("Sending query reply (UID collected: %d bytes)",
                        len(state["received_uid"]))
            msg = uavcan.protocol.dynamic_node_id.Allocation()
            msg.node_id = 0
            msg.first_part_of_unique_id = False
            msg.unique_id = bytes(state["received_uid"])

            transfer = Transfer(
                payload=msg,
                source_node_id=self.ALLOCATOR_NODE_ID,
                transfer_id=state["transfer_id"],
                transfer_priority=20,
                service_not_message=False,
            )
            frames = transfer.to_frames()
            logger.info("Sending %d query frame(s)", len(frames))
            for f in frames:
                logger.info("  → Query frame: can_id=0x%08X dlc=%d data=%s",
                            f.message_id, len(f.bytes), f.bytes.hex())
                self._send_can_frame(f.message_id, f.bytes)

            state["start_time"] = time.time()
            state["transfer_id"] = (state["transfer_id"] + 1) & 0x1F
            return True

    def _send_can_frame(self, can_id: int, data: bytes) -> None:
        """Send a raw CAN data frame on the attached bus."""
        if self._bus is None:
            logger.warning("Cannot send CAN frame 0x%X: no bus attached", can_id)
            return
        try:
            logger.debug("Sending CAN frame: can_id=0x%08X dlc=%d data=%s",
                         can_id, len(data), data.hex())
            self._bus.send_data_frame(can_id, bytes(data))
        except Exception as exc:
            logger.warning("Failed to send CAN frame 0x%X: %s", can_id, exc)

    def _emit_allocation(self, payload, frame):
        node_id = frame.arbitration_id & 0x7F
        self._emit_measurement(
            {
                "measurement_type": "allocation",
                "id": node_id,
                "data": {
                    "granted_node_id": payload.node_id,
                    "first_part_of_unique_id": payload.first_part_of_unique_id,
                    "unique_id": bytes(payload.unique_id).hex(),
                },
            },
            frame,
            kind="uavcan",
        )

    def _emit_range_measurement(self, payload, frame):
        orient = payload.beam_orientation_in_body_frame
        node_id = frame.arbitration_id & 0x7F
        self._emit_measurement(
            {
                "measurement_type": "range",
                "id": node_id,
                "data": {
                    "sensor_id": payload.sensor_id,
                    "range_m": payload.range,
                    "range_mm": payload.range * 1000,
                    "sensor_type": payload.sensor_type,
                    "reading_type": payload.reading_type,
                    "field_of_view": payload.field_of_view,
                    "roll": orient.fixed_axis_roll_pitch_yaw[0],
                    "pitch": orient.fixed_axis_roll_pitch_yaw[1],
                    "yaw": orient.fixed_axis_roll_pitch_yaw[2],
                    "timestamp_us": payload.timestamp.usec,
                },
            },
            frame,
            kind="uavcan",
        )

    def _emit_node_status(self, payload, frame):
        # Record this node as seen so its node_id stays reserved
        source_node = frame.arbitration_id & 0x7F
        if source_node > 0:
            self._store.mark_seen_by_node_id(source_node)
            logger.info("[%s] NodeStatus from node %d (health=%d mode=%d)",
                        time.strftime("%H:%M:%S"), source_node,
                        payload.health, payload.mode)
        self._emit_measurement(
            {
                "measurement_type": "node_status",
                "id": source_node,
                "data": {
                    "uptime_sec": payload.uptime_sec,
                    "health": payload.health,
                    "mode": payload.mode,
                    "sub_mode": payload.sub_mode,
                    "vendor_status": payload.vendor_specific_status_code,
                },
            },
            frame,
            kind="uavcan",
        )

    def _emit_flow_measurement(self, payload, frame):
        node_id = frame.arbitration_id & 0x7F
        self._emit_measurement(
            {
                "measurement_type": "flow",
                "id": node_id,
                "data": {
                    "integration_interval": payload.integration_interval,
                    "rate_gyro_integral": (
                        payload.rate_gyro_integral[0],
                        payload.rate_gyro_integral[1],
                    ),
                    "flow_integral": (
                        payload.flow_integral[0],
                        payload.flow_integral[1],
                    ),
                    "quality": payload.quality,
                },
            },
            frame,
            kind="uavcan",
        )

    def start(self) -> None:
        """UAVCAN devices don't need explicit start."""
        pass

    def stop(self) -> None:
        """Flush the node ID store so final timestamps are persisted."""
        if self._store is not None:
            self._store.flush()