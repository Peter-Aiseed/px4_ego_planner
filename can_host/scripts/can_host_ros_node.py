#!/usr/bin/env python
"""ROS 1 wrapper around can_host — publishes each ToF sensor's range as a
single-point PointCloud2 (range, 0, 0) in that sensor's own frame, on its
own topic. Does not modify any of the original can_host code.
"""
from __future__ import annotations

import sys
from typing import Dict, Optional

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from can_host.main import parse_args, resolve_baudrate_if_needed
from can_host.host import Host
from can_host.events import DeviceMeasurement
from can_host.devices._afbr_parser import AFBRParser, parse_can_id_list


class RosPublisher:
    """Publishes each sensor's range as a single-point PointCloud2, keyed
    by (namespace, id) so AFBR and UAVCAN sensors don't collide even if
    their numeric IDs overlap.
    """

    def __init__(self, afbr_topic: str, uavcan_topic: str,
                 frame_id_prefix: str = "afbr_s50", uavcan_frame_id_prefix: str = "uavcan",
                 queue_size: int = 10, show_ids: Optional[list] = None) -> None:
        self.afbr_topic = afbr_topic.rstrip("/")
        self.uavcan_topic = uavcan_topic.rstrip("/")
        self.frame_id_prefix = frame_id_prefix
        self.uavcan_frame_id_prefix = uavcan_frame_id_prefix
        self.queue_size = queue_size
        self.show_ids = show_ids
        self._pubs: Dict[str, rospy.Publisher] = {}

    def _get_publisher(self, base: str, sensor_id: int) -> rospy.Publisher:
        topic = f"{base}/id_0x{sensor_id:02x}"
        pub = self._pubs.get(topic)
        if pub is None:
            pub = rospy.Publisher(topic, PointCloud2, queue_size=self.queue_size)
            self._pubs[topic] = pub
            rospy.loginfo(f"RosPublisher: new sensor, publishing on {topic}")
        return pub

    @staticmethod
    def _make_cloud(frame_id: str, range_m: float) -> PointCloud2:
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return point_cloud2.create_cloud_xyz32(header, [(range_m, 0.0, 0.0)])

    def on_measurement(self, event: DeviceMeasurement) -> None:
        if event.kind == "afbr":
            self._publish_afbr(event)
        elif event.kind == "uavcan":
            self._publish_uavcan(event)

    def _publish_afbr(self, event: DeviceMeasurement) -> None:
        can_id = event.frame.arbitration_id
        if self.show_ids is not None and can_id not in self.show_ids:
            return
        meas = event.measurement
        range_m = meas.range_mm / 1000.0
        frame_id = f"{self.frame_id_prefix}_0x{can_id:02x}_link"
        cloud = self._make_cloud(frame_id, range_m)
        self._get_publisher(self.afbr_topic, can_id).publish(cloud)

    def _publish_uavcan(self, event: DeviceMeasurement) -> None:
        meas = event.measurement
        if meas.get("measurement_type") != "range":
            return  # skip node_status/flow/allocation
        node_id = meas.get("id", 0)
        if self.show_ids is not None and node_id not in self.show_ids:
            return
        data = meas.get("data", {})
        range_m = data.get("range_mm", 0) / 1000.0
        frame_id = f"{self.uavcan_frame_id_prefix}_0x{node_id:02x}_link"
        cloud = self._make_cloud(frame_id, range_m)
        self._get_publisher(self.uavcan_topic, node_id).publish(cloud)


def main() -> int:
    sys.argv = rospy.myargv(sys.argv)   # strip roslaunch's __name:=, __log:= etc.
    args = parse_args()                 # reuses can_host's original CLI parser as-is

    if args.can_ids:
        AFBRParser.FRAME_ID_1D = parse_can_id_list(args.can_ids)
    show_ids = parse_can_id_list(args.show_ids) if args.show_ids else None
    args.show_ids_list = show_ids
    args.baudrate = resolve_baudrate_if_needed(args)

    rospy.init_node("can_host_ros", disable_signals=True)

    afbr_topic = rospy.get_param("~afbr_topic", "/pointcloud/tof")
    uavcan_topic = rospy.get_param("~uavcan_topic", "/pointcloud/uavcan_tof")
    frame_id_prefix = rospy.get_param("~frame_id_prefix", "tof")
    uavcan_frame_id_prefix = rospy.get_param("~uavcan_frame_id_prefix", "uavcan_tof")

    host = Host(args)
    rc = host.setup()
    if rc != 0:
        return rc

    ros_pub = RosPublisher(
        afbr_topic=afbr_topic,
        uavcan_topic=uavcan_topic,
        frame_id_prefix=frame_id_prefix,
        uavcan_frame_id_prefix=uavcan_frame_id_prefix,
        show_ids=show_ids,
    )
    host.event_bus.on("measurement", ros_pub.on_measurement)

    return host.run()


if __name__ == "__main__":
    sys.exit(main())