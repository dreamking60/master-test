#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ForwardPublisher(Node):
    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel_in', 10)
        self.pattern = os.environ.get('CONTROLLER_PATTERN', 'forward').strip().lower()
        self.linear_speed = float(os.environ.get('CONTROLLER_LINEAR_SPEED', '0.2'))
        self.angular_speed = float(os.environ.get('CONTROLLER_ANGULAR_SPEED', '0.8'))
        self.square_side_seconds = float(os.environ.get('CONTROLLER_SQUARE_SIDE_SECONDS', '4.0'))
        self.square_turn_seconds = float(os.environ.get('CONTROLLER_SQUARE_TURN_SECONDS', '1.96'))
        self.zigzag_period_seconds = float(os.environ.get('CONTROLLER_ZIGZAG_PERIOD_SECONDS', '1.0'))
        self.started_at = time.monotonic()
        self.current_phase = None
        self.count = 0
        self.timer = self.create_timer(0.1, self.tick)
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.get_logger().info(
            'Controller publisher started: /cmd_vel_in @ 10Hz '
            f'pattern={self.pattern} linear={self.linear_speed} angular={self.angular_speed}'
        )

    def build_forward_msg(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        return msg

    def build_square_msg(self):
        msg = Twist()
        segment_seconds = self.square_side_seconds + self.square_turn_seconds
        elapsed = (time.monotonic() - self.started_at) % segment_seconds

        if elapsed < self.square_side_seconds:
            phase = 'forward-side'
            msg.linear.x = self.linear_speed
        else:
            phase = 'left-turn'
            msg.angular.z = self.angular_speed

        if phase != self.current_phase:
            self.current_phase = phase
            self.get_logger().info(f'square phase={phase}')

        return msg

    def build_zigzag_msg(self):
        msg = Twist()
        elapsed = time.monotonic() - self.started_at
        half_period = max(self.zigzag_period_seconds / 2.0, 0.1)
        turn_left = int(elapsed / half_period) % 2 == 0
        phase = 'zig-left' if turn_left else 'zig-right'

        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed if turn_left else -self.angular_speed

        if phase != self.current_phase:
            self.current_phase = phase
            self.get_logger().info(f'zigzag phase={phase}')

        return msg

    def build_msg(self):
        if self.pattern == 'square':
            return self.build_square_msg()
        if self.pattern in ('zigzag', 'weave'):
            return self.build_zigzag_msg()
        return self.build_forward_msg()

    def tick(self):
        self.pub.publish(self.build_msg())
        self.count += 1

    def log_status(self):
        self.get_logger().info(f'published={self.count} pattern={self.pattern} phase={self.current_phase or "forward"}')


def main():
    rclpy.init()
    node = ForwardPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.get_logger().info('Published zero stop command before shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
