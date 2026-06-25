#!/usr/bin/env python3
"""
Minimal image rectifier — a drop-in replacement for `image_proc rectify_node`
that uses only OpenCV + cv_bridge (already available via the object_detection
package), so image_proc does NOT need to be installed.

Subscribes (remap these, same as image_proc):
    image        sensor_msgs/Image       distorted input
    camera_info  sensor_msgs/CameraInfo   intrinsics + distortion
Publishes:
    image_rect   sensor_msgs/Image       rectified output

Run (identical interface to the image_proc command):
    python3 scripts/rectify_node.py --ros-args \
        -r image:=/camera/image_raw \
        -r camera_info:=/camera/camera_info \
        -r image_rect:=/camera/image_rect \
        -p use_sim_time:=true
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RectifyNode(Node):
    def __init__(self):
        super().__init__("rectify_node")
        self.bridge = CvBridge()
        self.map1 = None
        self.map2 = None
        self.map_size = None  # (w, h) the maps were built for

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Image, "image_rect", qos)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos)
        self.create_subscription(Image, "image", self.on_image, qos)
        self.get_logger().info("rectify_node (OpenCV) ready; waiting for camera_info...")

    def on_info(self, msg: CameraInfo):
        size = (msg.width, msg.height)
        if self.map1 is not None and self.map_size == size:
            return  # maps already built for this resolution

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        R = np.array(msg.r, dtype=np.float64).reshape(3, 3)
        P = np.array(msg.p, dtype=np.float64).reshape(3, 4)
        new_K = P[:3, :3]  # rectified projection's intrinsics
        if not np.any(R):   # all-zero R (uncalibrated) -> identity
            R = np.eye(3)

        model = (msg.distortion_model or "plumb_bob").lower()
        if model in ("equidistant", "fisheye"):
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D[:4], R, new_K, size, cv2.CV_16SC2
            )
        else:  # plumb_bob / rational_polynomial / etc.
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                K, D, R, new_K, size, cv2.CV_16SC2
            )
        self.map_size = size
        self.get_logger().info(
            f"Built rectify maps for {size[0]}x{size[1]} (model={model})"
        )

    def on_image(self, msg: Image):
        if self.map1 is None:
            self.get_logger().warn("No camera_info yet; dropping image",
                                   throttle_duration_sec=2.0)
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        rect = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        out = self.bridge.cv2_to_imgmsg(rect, encoding=msg.encoding)
        out.header = msg.header  # preserve stamp + frame_id
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
