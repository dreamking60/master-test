#!/usr/bin/env python3
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AttackerPublisher(Node):
    def __init__(self):
        super().__init__('attacker')
        self.pub = self.create_publisher(Twist, '/cmd_vel_in', 10)
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.8
        self.count = 0
        self.timer = self.create_timer(0.02, self.tick)
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.get_logger().info('Attacker publisher started: /cmd_vel_in @ 50Hz (wall timer)')

    def tick(self):
        self.pub.publish(self.msg)
        self.count += 1

    def log_status(self):
        self.get_logger().warn(f'published={self.count}')


def main():
    rclpy.init()
    try:
        node = AttackerPublisher()
    except RCLError as exc:
        print('SROS2 policy blocked attacker publisher creation on /cmd_vel_in.')
        print('This is the expected result in Experiment 02 secure mode.')
        print(f'Original rclpy error: {exc}')
        rclpy.shutdown()
        return 13

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
