#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


DURATION = 10.0  # seconds (wall clock)


class VelPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.pub = self.create_publisher(TwistStamped, 'nav_vel', 10)
        self.start_time = None
        self.create_timer(0.1, self.publish)  # 10 Hz

    def publish(self):
        if self.start_time is None:
            self.start_time = time.monotonic()
        elapsed = time.monotonic() - self.start_time
        if elapsed >= DURATION:
            self.get_logger().info('10 seconds elapsed, shutting down.')
            raise SystemExit

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.5   # m/s forward
        msg.twist.angular.z = 0.0  # rad/s turn
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
