# #!/usr/bin/env python3

# import math
# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import TwistStamped
# from nav_msgs.msg import Odometry


# OBSTACLES = [
#     (2.5,  1.5),
#     (2.5, -1.5),
#     (3.5,  0.0),
#     (6.0,  2.0),
#     (6.0, -2.0),
#     (5.0, -3.5),
#     (7.0,  3.5),
#     (8.0, -1.0),
#     (9.0,  2.0),
#     (12.0,  1.5),
#     (12.0, -1.5),
#     (13.0,  0.0),
# ]

# GOAL = (14.0, 0.0)


# def yaw_from_quaternion(q):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# def wrap_angle(a):
#     while a > math.pi:
#         a -= 2.0 * math.pi
#     while a < -math.pi:
#         a += 2.0 * math.pi
#     return a


# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__("obstacle_avoider")

#         self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             "/odom",
#             self.odom_callback,
#             10,
#         )

#         self.get_logger().info("Obstacle avoider started")

#     def odom_callback(self, msg):
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         yaw = yaw_from_quaternion(msg.pose.pose.orientation)

#         gx, gy = GOAL

#         # Attractive vector toward goal
#         att_x = gx - x
#         att_y = gy - y

#         att_norm = math.hypot(att_x, att_y)
#         if att_norm > 0.001:
#             att_x /= att_norm
#             att_y /= att_norm

#         # Repulsive vector away from nearby obstacles
#         rep_x = 0.0
#         rep_y = 0.0
#         influence_radius = 1.5

#         for ox, oy in OBSTACLES:
#             dx = x - ox
#             dy = y - oy
#             dist = math.hypot(dx, dy)

#             if 0.001 < dist < influence_radius:
#                 strength = (1.0 / dist - 1.0 / influence_radius) / (dist * dist)
#                 rep_x += strength * dx / dist
#                 rep_y += strength * dy / dist

#         # Combine goal attraction and obstacle repulsion
#         k_att = 1.0
#         k_rep = 1.5

#         desired_x = k_att * att_x + k_rep * rep_x
#         desired_y = k_att * att_y + k_rep * rep_y

#         desired_yaw = math.atan2(desired_y, desired_x)
#         yaw_error = wrap_angle(desired_yaw - yaw)

#         distance_to_goal = math.hypot(gx - x, gy - y)

#         cmd = TwistStamped()
#         cmd.header.stamp = self.get_clock().now().to_msg()
#         cmd.header.frame_id = "base_link"

#         if distance_to_goal < 0.4:
#             cmd.twist.linear.x = 0.0
#             cmd.twist.angular.z = 0.0
#             self.get_logger().info("Goal reached")
#         else:
#             cmd.twist.linear.x = 0.35
#             cmd.twist.angular.z = max(-1.0, min(1.0, 1.5 * yaw_error))

#         self.cmd_pub.publish(cmd)


# def main():
#     rclpy.init()
#     node = ObstacleAvoider()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


ROBOT_RADIUS = 0.45
SAFETY_MARGIN = 0.25
INFLUENCE_RADIUS = 1.2

GOAL = (14.0, 0.0)

OBSTACLES = [
    {"type": "box", "x": 2.5, "y": 1.5, "hx": 0.3, "hy": 0.3},
    {"type": "box", "x": 2.5, "y": -1.5, "hx": 0.3, "hy": 0.3},
    {"type": "cylinder", "x": 3.5, "y": 0.0, "r": 0.3},
    {"type": "box", "x": 6.0, "y": 2.0, "hx": 0.2, "hy": 1.5},
    {"type": "box", "x": 6.0, "y": -2.0, "hx": 0.2, "hy": 1.5},
    {"type": "cylinder", "x": 5.0, "y": -3.5, "r": 0.25},
    {"type": "box", "x": 7.0, "y": 3.5, "hx": 0.35, "hy": 0.35},
    {"type": "box", "x": 8.0, "y": -1.0, "hx": 0.3, "hy": 0.3},
    {"type": "cylinder", "x": 9.0, "y": 2.0, "r": 0.3},
    {"type": "box", "x": 12.0, "y": 1.5, "hx": 0.3, "hy": 0.3},
    {"type": "box", "x": 12.0, "y": -1.5, "hx": 0.3, "hy": 0.3},
    {"type": "cylinder", "x": 13.0, "y": 0.0, "r": 0.35},
]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def nearest_point_on_expanded_obstacle(px, py, obs):
    margin = ROBOT_RADIUS + SAFETY_MARGIN

    if obs["type"] == "cylinder":
        ox, oy = obs["x"], obs["y"]
        dx = px - ox
        dy = py - oy
        dist = math.hypot(dx, dy)

        effective_radius = obs["r"] + margin

        if dist < 1e-6:
            return ox + effective_radius, oy, -1.0

        nearest_x = ox + effective_radius * dx / dist
        nearest_y = oy + effective_radius * dy / dist
        surface_dist = dist - effective_radius

        return nearest_x, nearest_y, surface_dist

    if obs["type"] == "box":
        ox, oy = obs["x"], obs["y"]

        hx = obs["hx"] + margin
        hy = obs["hy"] + margin

        min_x = ox - hx
        max_x = ox + hx
        min_y = oy - hy
        max_y = oy + hy

        nearest_x = min(max(px, min_x), max_x)
        nearest_y = min(max(py, min_y), max_y)

        dx = px - nearest_x
        dy = py - nearest_y
        outside_dist = math.hypot(dx, dy)

        inside = min_x <= px <= max_x and min_y <= py <= max_y

        if inside:
            distances = [
                (abs(px - min_x), -1.0, 0.0),
                (abs(px - max_x), 1.0, 0.0),
                (abs(py - min_y), 0.0, -1.0),
                (abs(py - max_y), 0.0, 1.0),
            ]
            min_dist, nx, ny = min(distances, key=lambda v: v[0])
            return px - nx * min_dist, py - ny * min_dist, -min_dist

        return nearest_x, nearest_y, outside_dist

    raise ValueError(f"Unknown obstacle type: {obs['type']}")


class BoundaryAvoider(Node):
    def __init__(self):
        super().__init__("boundary_avoider")

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info("Boundary-based obstacle avoider started")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        gx, gy = GOAL

        att_x = gx - x
        att_y = gy - y
        att_norm = math.hypot(att_x, att_y)

        if att_norm > 1e-6:
            att_x /= att_norm
            att_y /= att_norm

        rep_x = 0.0
        rep_y = 0.0

        for obs in OBSTACLES:
            nearest_x, nearest_y, surface_dist = nearest_point_on_expanded_obstacle(
                x, y, obs
            )

            dx = x - nearest_x
            dy = y - nearest_y
            norm = math.hypot(dx, dy)

            if norm < 1e-6:
                continue

            if surface_dist < INFLUENCE_RADIUS:
                d = max(surface_dist, 0.05)
                strength = (1.0 / d - 1.0 / INFLUENCE_RADIUS) / (d * d)

                rep_x += strength * dx / norm
                rep_y += strength * dy / norm

        k_att = 1.0
        k_rep = 0.8

        desired_x = k_att * att_x + k_rep * rep_x
        desired_y = k_att * att_y + k_rep * rep_y

        desired_yaw = math.atan2(desired_y, desired_x)
        yaw_error = wrap_angle(desired_yaw - yaw)

        distance_to_goal = math.hypot(gx - x, gy - y)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        if distance_to_goal < 0.4:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
        else:
            cmd.twist.linear.x = 0.35
            cmd.twist.angular.z = max(-1.0, min(1.0, 1.5 * yaw_error))

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = BoundaryAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()