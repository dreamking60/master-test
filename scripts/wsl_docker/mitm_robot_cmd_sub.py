#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class MitmCommandSubscriber(Node):
    def __init__(self):
        super().__init__('mitm_robot_cmd_sub')
        self.topic = os.environ.get('MITM_CMD_TOPIC', '/mitm_cmd')
        self.sub = self.create_subscription(TwistStamped, self.topic, self.cb, 50)
        self.count = 0
        self.last_count = 0
        self.last_msg_time = None
        self.max_gap_ms = 0.0
        self.max_latency_ms = 0.0
        self.sum_latency_ms = 0.0
        self.last_latency_ms = 0.0
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.get_logger().info(f'MITM robot subscriber started: {self.topic}')

    def cb(self, msg: TwistStamped):
        now = self.get_clock().now()
        sent_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        latency_ms = max((now.nanoseconds - sent_ns) / 1_000_000.0, 0.0)

        if self.last_msg_time is not None:
            gap_ms = (now - self.last_msg_time).nanoseconds / 1_000_000.0
            self.max_gap_ms = max(self.max_gap_ms, gap_ms)
        self.last_msg_time = now

        self.count += 1
        self.last_latency_ms = latency_ms
        self.sum_latency_ms += latency_ms
        self.max_latency_ms = max(self.max_latency_ms, latency_ms)

    def log_status(self):
        delta = self.count - self.last_count
        self.last_count = self.count
        avg_latency = self.sum_latency_ms / self.count if self.count else 0.0
        self.get_logger().info(
            f'received={self.count} rate={delta}/s '
            f'last_latency_ms={self.last_latency_ms:.1f} '
            f'avg_latency_ms={avg_latency:.1f} '
            f'max_latency_ms={self.max_latency_ms:.1f} '
            f'max_gap_ms={self.max_gap_ms:.1f}'
        )


def main():
    rclpy.init()
    node = MitmCommandSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
