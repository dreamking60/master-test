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
        self.get_logger().info('Relay started: /cmd_vel_in -> /cmd_vel')

    def cb(self, msg: Twist):
        self.pub.publish(msg)
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'relayed={self.count}')


def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
