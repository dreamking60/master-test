#!/usr/bin/env python3
import rclpy
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
        self.get_logger().info('Attacker publisher started: /cmd_vel @ 50Hz (wall timer)')

    def tick(self):
        self.pub.publish(self.msg)
        self.count += 1

    def log_status(self):
        self.get_logger().warn(f'published={self.count}')


def main():
    rclpy.init()
    node = AttackerPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
