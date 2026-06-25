#!/usr/bin/env python3
"""
Log detected objects (class + map-frame position) to a CSV file, with temporal
and spatial filtering so each physical object is written exactly once.

Pipeline
--------
1. Subscribe to /detection_info (object_detection_msgs/ObjectDetectionInfoArray),
   published by the object_detection node in the camera optical frame.
2. Transform each detection's position into the map frame via TF.
3. TEMPORAL filter: detections of the same class that arrive close together in
   time *and* space are treated as the same object and their positions are
   averaged (a running mean) into a single "pending" track. A track is
   finalized once no new detection has been added to it for `temporal_window`.
4. SPATIAL filter: when a track finalizes, it is compared against everything
   already stored. If a same-class object already exists within `dedup_radius`
   (default 1.0 m), the new one is discarded (we already have it); otherwise it
   is appended to the CSV.

CSV format (one header row, then one row per stored object):
    class, x, y, z

Run
---
    source /opt/ros/jazzy/setup.bash
    source /a2_ros_ws/install/setup.bash          # for object_detection_msgs
    python3 scripts/object_logger.py --ros-args \
        -p use_sim_time:=true \
        -p csv_path:=/a2_ros/detected_objects.csv

Parameters (all optional, shown with defaults):
    detection_topic   /detection_info
    target_frame      map
    csv_path          ~/detected_objects.csv
    dedup_radius      1.0    # m, spatial dedup against already-stored objects
    merge_radius      1.0    # m, max distance to treat detections as same object
    temporal_window   2.0    # s, idle time before a pending track is finalized
    min_confidence    0.0    # ignore detections below this confidence
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from object_detection_msgs.msg import ObjectDetectionInfoArray


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class ObjectLogger(Node):
    def __init__(self):
        super().__init__("object_logger")

        self.detection_topic = self.declare_parameter("detection_topic", "/detection_info").value
        self.target_frame = self.declare_parameter("target_frame", "map").value
        default_csv = os.path.join(os.path.expanduser("~"), "detected_objects.csv")
        self.csv_path = self.declare_parameter("csv_path", default_csv).value
        self.dedup_radius = float(self.declare_parameter("dedup_radius", 2.0).value)
        self.merge_radius = float(self.declare_parameter("merge_radius", 2.0).value)
        self.temporal_window = float(self.declare_parameter("temporal_window", 2.0).value)
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.0).value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pending tracks (being temporally averaged) and confirmed objects.
        #   pending:   [{class, sum:[x,y,z], count, last_stamp(rclpy.Time)}]
        #   confirmed: [{class, pos:[x,y,z]}]
        self.pending = []
        self.confirmed = []
        self._latest_stamp = None  # newest detection stamp seen, used as the clock

        # CSV — create with header if new/empty, otherwise append (resume).
        self._init_csv()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            ObjectDetectionInfoArray, self.detection_topic, self.on_detections, qos
        )
        # Periodically finalize stale pending tracks.
        self.create_timer(0.5, self.finalize_stale)

        self.get_logger().info(
            f"object_logger: topic={self.detection_topic} frame={self.target_frame} "
            f"csv={self.csv_path} dedup={self.dedup_radius}m window={self.temporal_window}s"
        )

    # ----------------------------------------------------------------- CSV ---
    def _init_csv(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.csv_path)), exist_ok=True)
        is_new = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        # Pre-load any rows already in the file so we don't re-add them and we
        # keep deduplicating across restarts.
        if not is_new:
            try:
                with open(self.csv_path, newline="") as f:
                    for row in csv.reader(f):
                        if len(row) == 4 and row[0] != "class":
                            self.confirmed.append(
                                {"class": row[0],
                                 "pos": [float(row[1]), float(row[2]), float(row[3])]}
                            )
            except (OSError, ValueError):
                pass
        self._csv = open(self.csv_path, "a", newline="")
        self._writer = csv.writer(self._csv)
        if is_new:
            self._writer.writerow(["class", "x", "y", "z"])
            self._csv.flush()
        if self.confirmed:
            self.get_logger().info(f"Loaded {len(self.confirmed)} existing objects from CSV")

    def _write_row(self, cls, pos):
        self._writer.writerow([cls, f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}"])
        self._csv.flush()  # persist immediately so a kill doesn't lose data

    # ------------------------------------------------------------- callbacks ---
    def on_detections(self, msg: ObjectDetectionInfoArray):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self._latest_stamp is None or stamp > self._latest_stamp:
            self._latest_stamp = stamp

        for det in msg.info:
            if det.confidence < self.min_confidence:
                continue
            pos = self._to_map(det.position, msg.header)
            if pos is None:
                continue
            self._accumulate(det.class_id, pos, stamp)

    def _to_map(self, point, header):
        """Transform a Point in header.frame_id into target_frame. Non-blocking:
        tries the detection stamp first, then latest available; returns None on
        failure (so we never stall the executor / the TF listener)."""
        ps = PointStamped()
        ps.header = header
        ps.point = point
        for t in (rclpy.time.Time.from_msg(header.stamp), rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, header.frame_id, t, timeout=Duration(seconds=0.0)
                )
                out = do_transform_point(ps, tf)
                return [out.point.x, out.point.y, out.point.z]
            except TransformException:
                continue
        self.get_logger().warn(
            f"No TF {header.frame_id} -> {self.target_frame}; skipping detection",
            throttle_duration_sec=2.0,
        )
        return None

    def _accumulate(self, cls, pos, stamp):
        """Add a detection to a matching pending track (same class, within
        merge_radius of its current mean), or start a new track."""
        for tr in self.pending:
            if tr["class"] != cls:
                continue
            mean = [s / tr["count"] for s in tr["sum"]]
            if _dist(mean, pos) <= self.merge_radius:
                tr["sum"] = [tr["sum"][i] + pos[i] for i in range(3)]
                tr["count"] += 1
                tr["last_stamp"] = stamp
                return
        self.pending.append(
            {"class": cls, "sum": list(pos), "count": 1, "last_stamp": stamp}
        )

    def finalize_stale(self):
        """Finalize tracks idle for longer than temporal_window, applying the
        spatial dedup filter against already-stored objects."""
        if self._latest_stamp is None:
            return
        still_pending = []
        for tr in self.pending:
            idle = (self._latest_stamp - tr["last_stamp"]).nanoseconds * 1e-9
            if idle < self.temporal_window:
                still_pending.append(tr)
                continue
            self._commit(tr)
        self.pending = still_pending

    def _commit(self, tr):
        mean = [s / tr["count"] for s in tr["sum"]]
        cls = tr["class"]
        for obj in self.confirmed:
            if obj["class"] == cls and _dist(obj["pos"], mean) < self.dedup_radius:
                self.get_logger().info(
                    f"Discard duplicate '{cls}' at "
                    f"({mean[0]:.2f},{mean[1]:.2f},{mean[2]:.2f}) — "
                    f"within {self.dedup_radius}m of existing"
                )
                return
        self.confirmed.append({"class": cls, "pos": mean})
        self._write_row(cls, mean)
        self.get_logger().info(
            f"Stored '{cls}' at ({mean[0]:.2f},{mean[1]:.2f},{mean[2]:.2f}) "
            f"[{tr['count']} detections averaged] -> total {len(self.confirmed)}"
        )

    def flush_all(self):
        """Finalize every remaining pending track (e.g. on shutdown)."""
        for tr in self.pending:
            self._commit(tr)
        self.pending = []
        try:
            self._csv.flush()
            self._csv.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
