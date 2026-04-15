#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel_in', self.cb, 50)
        self.count = 0
        self.last_input_time = None
        self.stopped_after_timeout = True
        self.timeout_seconds = 0.5
        self.watchdog = self.create_timer(0.1, self.check_input_timeout)
        self.get_logger().info('Relay started: /cmd_vel_in -> /cmd_vel')
        self.get_logger().info(f'Input watchdog enabled: stop robot after {self.timeout_seconds:.1f}s without /cmd_vel_in')

    def cb(self, msg: Twist):
        self.pub.publish(msg)
        self.count += 1
        self.last_input_time = self.get_clock().now()
        self.stopped_after_timeout = False
        if self.count % 100 == 0:
            self.get_logger().info(f'relayed={self.count}')

    def check_input_timeout(self):
        if self.last_input_time is None or self.stopped_after_timeout:
            return

        elapsed = (self.get_clock().now() - self.last_input_time).nanoseconds / 1_000_000_000
        if elapsed >= self.timeout_seconds:
            self.pub.publish(Twist())
            self.stopped_after_timeout = True
            self.get_logger().warn(f'No /cmd_vel_in for {elapsed:.2f}s; published zero /cmd_vel')


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
