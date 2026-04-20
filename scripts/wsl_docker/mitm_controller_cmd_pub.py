#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class MitmCommandPublisher(Node):
    def __init__(self):
        super().__init__('mitm_controller_cmd_pub')
        self.topic = os.environ.get('MITM_CMD_TOPIC', '/mitm_cmd')
        self.rate_hz = float(os.environ.get('MITM_CMD_RATE_HZ', '20'))
        self.linear_x = float(os.environ.get('MITM_CMD_LINEAR_X', '0.2'))
        self.angular_z = float(os.environ.get('MITM_CMD_ANGULAR_Z', '0.0'))
        self.pub = self.create_publisher(TwistStamped, self.topic, 10)
        self.count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.get_logger().info(
            f'MITM controller publisher started: {self.topic} @ {self.rate_hz:.1f}Hz '
            f'linear_x={self.linear_x} angular_z={self.angular_z}'
        )

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mitm_controller'
        msg.twist.linear.x = self.linear_x
        msg.twist.angular.z = self.angular_z
        self.pub.publish(msg)
        self.count += 1

    def log_status(self):
        self.get_logger().info(f'published={self.count}')


def main():
    rclpy.init()
    node = MitmCommandPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
