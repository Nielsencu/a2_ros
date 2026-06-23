# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TwistStamped


# def yaw_from_quaternion(q):
#     return math.atan2(
#         2.0 * (q.w * q.z + q.x * q.y),
#         1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     )


# def wrap_angle(angle):
#     while angle > math.pi:
#         angle -= 2.0 * math.pi
#     while angle < -math.pi:
#         angle += 2.0 * math.pi
#     return angle


# class GoToPoint(Node):
#     def __init__(self):
#         super().__init__("go_to_point")

#         # Change these to your desired goal in the map frame
#         self.goal_x = 2.0
#         self.goal_y = 0.0

#         self.x = None
#         self.y = None
#         self.yaw = None
#         self.goal_reached = False

#         self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
#         self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

#         self.timer = self.create_timer(0.1, self.control_loop)

#         self.get_logger().info(
#             f"Controller started. Goal: x={self.goal_x}, y={self.goal_y}"
#         )

#     def odom_callback(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

#     def publish_cmd(self, linear_x, angular_z):
#         cmd = TwistStamped()
#         cmd.header.stamp = self.get_clock().now().to_msg()
#         cmd.header.frame_id = "base_link"

#         cmd.twist.linear.x = linear_x
#         cmd.twist.linear.y = 0.0
#         cmd.twist.linear.z = 0.0

#         cmd.twist.angular.x = 0.0
#         cmd.twist.angular.y = 0.0
#         cmd.twist.angular.z = angular_z

#         self.cmd_pub.publish(cmd)

#     def control_loop(self):
#         if self.x is None or self.y is None or self.yaw is None:
#             self.get_logger().info("Waiting for odometry...")
#             return

#         if self.goal_reached:
#             self.publish_cmd(0.0, 0.0)
#             return

#         dx = self.goal_x - self.x
#         dy = self.goal_y - self.y

#         distance = math.sqrt(dx * dx + dy * dy)
#         desired_yaw = math.atan2(dy, dx)
#         yaw_error = wrap_angle(desired_yaw - self.yaw)

#         self.get_logger().info(
#             f"x={self.x:.2f}, y={self.y:.2f}, dist={distance:.2f}, yaw_err={yaw_error:.2f}"
#         )

#         if distance < 0.25:
#             self.get_logger().info("Goal reached. Stopping robot.")
#             self.goal_reached = True
#             self.publish_cmd(0.0, 0.0)
#             return

#         max_linear = 0.5
#         max_angular = 0.8

#         k_linear = 0.4
#         k_angular = 1.2

#         angular_z = k_angular * yaw_error
#         angular_z = max(min(angular_z, max_angular), -max_angular)

#         if abs(yaw_error) < 0.5:
#             linear_x = k_linear * distance
#             linear_x = min(linear_x, max_linear)
#         else:
#             linear_x = 0.0

#         self.publish_cmd(linear_x, angular_z)


# def main():
#     rclpy.init()
#     node = GoToPoint()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.publish_cmd(0.0, 0.0)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()



# #!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoToPoint(Node):
    def __init__(self):
        super().__init__("go_to_point")

        self.x = None
        self.y = None
        self.yaw = None

        self.goal_x = None
        self.goal_y = None
        self.goal_active = False

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Point, "/goal_point", self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("GoToPoint node started. Waiting for /goal_point...")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_active = True

        self.get_logger().info(
            f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}"
        )

    def publish_velocity(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        cmd.twist.linear.x = v
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = w

        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        if not self.goal_active:
            self.publish_velocity(0.0, 0.0)
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.sqrt(dx * dx + dy * dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_angle(desired_yaw - self.yaw)

        if distance < 0.25:
            self.publish_velocity(0.0, 0.0)
            self.goal_active = False
            self.get_logger().info("Goal reached. Waiting for next /goal_point.")
            return

        kv = 0.5
        kw = 1.2

        max_linear = 0.5
        max_angular = 1.0

        v = kv * distance
        w = kw * yaw_error

        v = min(v, max_linear)
        w = max(min(w, max_angular), -max_angular)

        if abs(yaw_error) > 0.5:
            v = 0.0

        self.publish_velocity(v, w)

        self.get_logger().info(
            f"x={self.x:.2f}, y={self.y:.2f}, "
            f"goal=({self.goal_x:.2f},{self.goal_y:.2f}), "
            f"dist={distance:.2f}, yaw_err={yaw_error:.2f}, "
            f"v={v:.2f}, w={w:.2f}"
        )


def main():
    rclpy.init()
    node = GoToPoint()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_velocity(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()