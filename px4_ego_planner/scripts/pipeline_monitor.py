#!/usr/bin/env python3
"""
One-file monitor for the TOF -> costmap -> 72-bin -> mavros pipeline,
plus the TF edges that gate the rolling-window recenter.

Run:
    rosrun px4_ego_planner pipeline_monitor.py
    # or:  python3 pipeline_monitor.py

Everything here is READ-ONLY: it subscribes and measures, it never
publishes a topic or a transform, so it cannot change any rate.

TOF FUNNEL
----------
For each TOF sensor it counts how many of the scanned points survive each
stage of the pipeline, so you can see *where* points are dropped:

    raw      points the sensor published            (width * height)
    valid    finite points (NaN / inf removed)
    inrange  valid points within the source's obstacle_range (sensor-frame
             3D distance, mirroring costmap_2d's marking range check)
    marked   inrange points that also pass the source's height band AND fall
             inside the current costmap window  -> would be marked as obstacles

Stages past the costmap merge every source into one grid, so per-sensor
identity ends at `marked`; the merged tail (occupied cells -> 72 bins ->
mavros) is reported as single totals.

The stage rules below are read straight from local_costmap_params.yaml and
scan_to_mavros.py; if you retune those, update TOF_SENSORS / COSTMAP_FRAME to
keep the funnel honest. Note tof_up is NOT an obstacle source, so its points
never reach the costmap -- the funnel makes that visible.
"""
import rospy
import numpy as np
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

# Costmap global frame (local_costmap_params.yaml: global_frame). TOF points
# are transformed into this frame to test the height band and map window.
COSTMAP_FRAME = "odom_drone_z"

# Per-TOF-sensor stage rules, mirrored from the obstacle_layer sources in
# local_costmap_params.yaml. `source=False` means the sensor is not listed as
# an observation_source, so its points never reach the costmap.
#   label (monitor topic name) -> obstacle_range, min_h, max_h, source
TOF_SENSORS = {
    "tof_left":  dict(obstacle_range=14.8, min_h=-0.5, max_h=2.0, source=True),
    "tof_right": dict(obstacle_range=14.8, min_h=-0.5, max_h=2.0, source=True),
    "tof_back":  dict(obstacle_range=14.8, min_h=-1.0, max_h=2.5, source=True),
    "tof_up":    dict(obstacle_range=15.0, min_h=-1e9, max_h=1e9, source=False),
}

# Multi-hop TF chains to resolve via a buffer (parent, child).
# These are not single /tf edges, so we look them up to get composite delay.
TF_CHAINS = [
    ("odom_drone_z", "base_link"),   # the chain costmap uses to recenter
    ("odom", "base_link"),
]


def cloud_xyz(msg):
    """Fast numpy view of an xyz float32 PointCloud2 -> (N,3) array, or None.

    Matches create_cloud_xyz32 clouds (the TOF publisher's format): x/y/z are
    the first three contiguous float32 fields. Returns None if the layout is
    not float32 xyz, so callers can fall back gracefully.
    """
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    FLOAT32 = 7
    if not all(k in off and off[k][1] == FLOAT32 for k in ("x", "y", "z")):
        return None
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < n * step:
        return None
    raw = raw[:n * step].reshape(n, step)

    def comp(name):
        o = off[name][0]
        return raw[:, o:o + 4].copy().view(np.float32).reshape(n)

    return np.stack([comp("x"), comp("y"), comp("z")], axis=1)


def quat_trans_matrix(q, t):
    """Build a 4x4 homogeneous transform from a geometry_msgs rotation+translation."""
    x, y, z, w = q.x, q.y, q.z, q.w
    nrm = x * x + y * y + z * z + w * w
    s = 2.0 / nrm if nrm > 0.0 else 0.0
    X, Y, Z = x * s, y * s, z * s
    wX, wY, wZ = w * X, w * Y, w * Z
    xX, xY, xZ = x * X, x * Y, x * Z
    yY, yZ, zZ = y * Y, y * Z, z * Z
    T = np.eye(4)
    T[:3, :3] = [[1.0 - (yY + zZ), xY - wZ,        xZ + wY],
                 [xY + wZ,        1.0 - (xX + zZ), yZ - wX],
                 [xZ - wY,        yZ + wX,         1.0 - (xX + yY)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


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

        # ---- per-sensor TOF funnel counts (label -> stage dict) ----
        self.funnel = defaultdict(dict)
        # ---- merged downstream totals (shared across all sources) ----
        self.cm = None            # current costmap window geometry
        self.occ_cells = "-"      # occupied costmap cells (data > 50)
        self.mavros_bins = "-"    # occupied 72-bins forwarded to mavros

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

        # ---- buffer for composite multi-hop chain latency + TOF transforms ----
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)

        rospy.Timer(rospy.Duration(REPORT_PERIOD), self.report)
        rospy.loginfo("pipeline_monitor started (read-only).")

    # ---------- topic callbacks ----------
    def _age(self, stamp):
        return (rospy.Time.now() - stamp).to_sec()

    def _lookup_T(self, source_frame):
        """4x4 transform from source_frame into COSTMAP_FRAME, or None."""
        try:
            tr = self.buf.lookup_transform(COSTMAP_FRAME, source_frame, rospy.Time(0))
        except Exception:
            return None
        return quat_trans_matrix(tr.transform.rotation, tr.transform.translation)

    def cb_pc(self, msg, label):
        age = self._age(msg.header.stamp)
        if label in TOF_SENSORS:
            self._tof_funnel(msg, label, age)
        else:
            self.topics[label].tick("pts=%d age=%.2f" % (msg.width * msg.height, age))

    def _tof_funnel(self, msg, label, age):
        """Count how many TOF points survive each pipeline stage for this sensor."""
        cfg = TOF_SENSORS[label]
        f = {"raw": msg.width * msg.height, "valid": "-",
             "inrange": "-", "marked": "-", "age": age, "note": "-"}

        pts = cloud_xyz(msg)
        if pts is None:
            f["note"] = "unparsed(non-xyz32)"
            self.funnel[label] = f
            self.topics[label].tick("raw=%d %s" % (f["raw"], f["note"]))
            return

        finite = np.isfinite(pts).all(axis=1)
        pts_v = pts[finite]
        f["valid"] = int(pts_v.shape[0])

        # Stage 2: range gate -- sensor-frame 3D distance vs obstacle_range.
        dist = np.linalg.norm(pts_v, axis=1)
        inrange = dist <= cfg["obstacle_range"]
        f["inrange"] = int(inrange.sum())

        if not cfg["source"]:
            f["marked"] = 0
            f["note"] = "not a costmap source"
            self.funnel[label] = f
            self.topics[label].tick("valid=%d (not a source)" % f["valid"])
            return

        # Stage 3: height band + costmap window, evaluated in the global frame.
        sub = pts_v[inrange]
        T = self._lookup_T(msg.header.frame_id)
        if T is None:
            f["note"] = "noTF->%s" % COSTMAP_FRAME
        elif self.cm is None:
            f["note"] = "no costmap yet"
        else:
            g = sub @ T[:3, :3].T + T[:3, 3]
            cm = self.cm
            hmask = (g[:, 2] >= cfg["min_h"]) & (g[:, 2] <= cfg["max_h"])
            bmask = ((g[:, 0] >= cm["x0"]) & (g[:, 0] <= cm["x1"]) &
                     (g[:, 1] >= cm["y0"]) & (g[:, 1] <= cm["y1"]))
            f["marked"] = int((hmask & bmask).sum())
            f["note"] = "ok"

        self.funnel[label] = f
        self.topics[label].tick("valid=%d inrange=%d marked=%s" %
                                (f["valid"], f["inrange"], f["marked"]))

    def cb_scan(self, msg, label):
        n = sum(1 for r in msg.ranges if msg.range_min < r < msg.range_max)
        self.topics[label].tick("beams=%d age=%.2f" % (n, self._age(msg.header.stamp)))

    def cb_odom(self, msg, label):
        v = msg.twist.twist.linear
        spd = (v.x**2 + v.y**2 + v.z**2) ** 0.5
        self.topics[label].tick("spd=%.2fm/s" % spd)

    def cb_costmap(self, msg, label):
        data = np.asarray(msg.data, dtype=np.int16)
        occ = int(np.count_nonzero(data > 50))
        self.occ_cells = occ
        info = msg.info
        self.cm = dict(
            x0=info.origin.position.x,
            y0=info.origin.position.y,
            x1=info.origin.position.x + info.width * info.resolution,
            y1=info.origin.position.y + info.height * info.resolution,
        )
        self.topics[label].tick("occ=%d age=%.2f" % (occ, self._age(msg.header.stamp)))

    def cb_upd(self, msg, label):
        self.topics[label].tick("-")

    def cb_mavros(self, msg, label):
        # empty bins are published as range_max + 1, so occupied == below range_max
        occ = [r for r in msg.ranges if r < msg.range_max]
        self.mavros_bins = len(occ)
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

        lines.append("----------- TOF FUNNEL (per sensor) ------")
        lines.append("%-10s %6s %6s %8s %7s  %s"
                     % ("sensor", "raw", "valid", "inrange", "marked", "note"))
        for s in ("tof_left", "tof_right", "tof_back", "tof_up"):
            f = self.funnel.get(s)
            if not f:
                lines.append("%-10s %6s %6s %8s %7s  %s"
                             % (s, "-", "-", "-", "-", "(no data)"))
                continue
            lines.append("%-10s %6s %6s %8s %7s  %s"
                         % (s, f["raw"], f["valid"], f["inrange"], f["marked"], f["note"]))
        lines.append("  merged downstream: occ_cells=%s  ->  mavros_bins=%s/72"
                     % (self.occ_cells, self.mavros_bins))

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
