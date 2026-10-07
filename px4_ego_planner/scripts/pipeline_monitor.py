#!/usr/bin/env python3
"""
One-file monitor for the TOF -> costmap -> 72-bin -> mavros pipeline,
plus the TF edges that gate the rolling-window recenter.

Run:
    rosrun px4_ego_planner pipeline_monitor.py
    # or:  python3 pipeline_monitor.py

Everything here is READ-ONLY: it subscribes and measures, it never
publishes a topic or a transform, so it cannot change any rate.
"""
import rospy
from collections import deque, defaultdict
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from tf2_msgs.msg import TFMessage
import tf2_ros

# Occupied-bin detection auto-tracks the publisher via each message's
# range_max (empty bins are sent as range_max + 1), so there is no
# max_detection_dist constant here to drift out of sync with scan_to_mavros.
WARN_AGE = 0.5           # seconds; flag data older than this
REPORT_PERIOD = 1.0      # seconds between table prints

# Multi-hop TF chains to resolve via a buffer (parent, child).
# These are not single /tf edges, so we look them up to get composite delay.
TF_CHAINS = [
    ("odom_drone_z", "base_link"),   # the chain costmap uses to recenter
    ("odom", "base_link"),
]


class Rate:
    """Sliding-window rate estimator + last payload note."""
    def __init__(self):
        self.t = deque(maxlen=60)
        self.note = "-"

    def tick(self, note="-"):
        self.t.append(rospy.get_time())
        self.note = note

    def hz(self):
        if len(self.t) < 2:
            return 0.0
        dt = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / dt if dt > 0 else 0.0

    def stale(self):
        # no message for > 2 report periods
        return self.t and (rospy.get_time() - self.t[-1]) > 2 * REPORT_PERIOD


class Monitor:
    def __init__(self):
        rospy.init_node("pipeline_monitor", anonymous=True)

        # ---- topic hops ----
        self.topics = defaultdict(Rate)        # label -> Rate
        self.tf_edges = defaultdict(Rate)      # "parent->child" -> Rate

        subs = [
            ("rs_cloud",   "/camera/depth/color/points",      PointCloud2,        self.cb_pc),
            ("rs_scan",    "/pointcloud/realsense/laserscan", LaserScan,          self.cb_scan),
            ("tof_left",   "/tof_sensor_left/pointcloud",     PointCloud2,        self.cb_pc),
            ("tof_right",  "/tof_sensor_right/pointcloud",    PointCloud2,        self.cb_pc),
            ("tof_back",   "/tof_sensor_back/pointcloud",     PointCloud2,        self.cb_pc),
            ("tof_up",     "/tof_sensor_up/pointcloud",       PointCloud2,        self.cb_pc),
            ("odom",       "/mavros/local_position/odom",     Odometry,           self.cb_odom),
            ("costmap",    "/costmap_node/costmap/costmap",   OccupancyGrid,      self.cb_costmap),
            ("costmap_upd","/costmap_node/costmap/costmap_updates", OccupancyGridUpdate, self.cb_upd),
            ("mavros_out", "/mavros/obstacle/send",           LaserScan,          self.cb_mavros),
        ]
        # fixed order for the printed table
        self.order = [s[0] for s in subs]
        for label, topic, typ, cb in subs:
            self.topics[label]  # create entry so it shows even before first msg
            rospy.Subscriber(topic, typ, cb, callback_args=label, queue_size=20)

        # ---- raw /tf and /tf_static edge rates (mirrors tf_monitor) ----
        rospy.Subscriber("/tf", TFMessage, self.cb_tf, queue_size=200)
        rospy.Subscriber("/tf_static", TFMessage, self.cb_tf, queue_size=50)

        # ---- buffer for composite multi-hop chain latency ----
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)

        rospy.Timer(rospy.Duration(REPORT_PERIOD), self.report)
        rospy.loginfo("pipeline_monitor started (read-only).")

    # ---------- topic callbacks ----------
    def _age(self, stamp):
        return (rospy.Time.now() - stamp).to_sec()

    def cb_pc(self, msg, label):
        self.topics[label].tick("pts=%d age=%.2f" % (msg.width * msg.height, self._age(msg.header.stamp)))

    def cb_scan(self, msg, label):
        n = sum(1 for r in msg.ranges if msg.range_min < r < msg.range_max)
        self.topics[label].tick("beams=%d age=%.2f" % (n, self._age(msg.header.stamp)))

    def cb_odom(self, msg, label):
        v = msg.twist.twist.linear
        spd = (v.x**2 + v.y**2 + v.z**2) ** 0.5
        self.topics[label].tick("spd=%.2fm/s" % spd)

    def cb_costmap(self, msg, label):
        occ = sum(1 for c in msg.data if c > 50)
        self.topics[label].tick("occ=%d age=%.2f" % (occ, self._age(msg.header.stamp)))

    def cb_upd(self, msg, label):
        self.topics[label].tick("-")

    def cb_mavros(self, msg, label):
        # empty bins are published as range_max + 1, so occupied == below range_max
        occ = [r for r in msg.ranges if r < msg.range_max]
        mn = min(occ) if occ else -1.0
        self.topics[label].tick("bins=%d/72 min=%.2f rmax=%.1f age=%.2f"
                                % (len(occ), mn, msg.range_max, self._age(msg.header.stamp)))

    def cb_tf(self, msg, _=None):
        for tr in msg.transforms:
            key = "%s->%s" % (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))
            self.tf_edges[key].tick("delay=%.2f" % self._age(tr.header.stamp))

    # ---------- reporting ----------
    def report(self, _):
        lines = ["", "================ PIPELINE ================"]
        lines.append("%-12s %7s   %s" % ("TOPIC", "Hz", "payload"))
        for label in self.order:
            r = self.topics[label]
            flag = "  <-- SILENT" if r.stale() else ""
            lines.append("%-12s %7.1f   %s%s" % (label, r.hz(), r.note, flag))

        lines.append("---------------- TF EDGES ----------------")
        lines.append("%-26s %7s   %s" % ("parent->child", "Hz", ""))
        for key in sorted(self.tf_edges):
            r = self.tf_edges[key]
            lines.append("%-26s %7.1f   %s" % (key, r.hz(), r.note))

        lines.append("------------- TF CHAINS (lookup) ---------")
        for parent, child in TF_CHAINS:
            try:
                t = self.buf.lookup_transform(parent, child, rospy.Time(0))
                lines.append("%-26s   delay=%.3fs" % ("%s->%s" % (parent, child), self._age(t.header.stamp)))
            except Exception as e:
                lines.append("%-26s   FAIL: %s" % ("%s->%s" % (parent, child), type(e).__name__))

        rospy.loginfo("\n".join(lines))


if __name__ == "__main__":
    try:
        Monitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
