#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class VelPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.pub = self.create_publisher(TwistStamped, 'nav_vel', 10)
        self.create_timer(0.1, self.publish)  # 10 Hz

    def publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.3   # m/s forward
        msg.twist.angular.z = 0.0  # rad/s turn
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
