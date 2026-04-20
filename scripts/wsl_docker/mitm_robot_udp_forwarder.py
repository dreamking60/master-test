#!/usr/bin/env python3
import os
import socket
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class MitmRobotUdpForwarder(Node):
    def __init__(self):
        super().__init__('mitm_robot_udp_forwarder')
        self.topic = os.environ.get('MITM_CMD_TOPIC', '/mitm_cmd')
        self.udp_host = os.environ.get('GAZEBO_UDP_HOST', '172.28.0.1')
        self.udp_port = int(os.environ.get('GAZEBO_UDP_PORT', '15000'))
        self.tamper_enabled = os.environ.get('MITM_TAMPER_ENABLE', '0').lower() in ('1', 'true', 'yes', 'on')
        self.tamper_flag_file = os.environ.get(
            'MITM_TAMPER_FLAG_FILE',
            '/workspace/project/logs/experiments/03_network_mitm/mitm_tamper_active',
        )
        self.tamper_angular_z = float(os.environ.get('MITM_TAMPER_ANGULAR_Z', '1.2'))
        self.tamper_linear_x = os.environ.get('MITM_TAMPER_LINEAR_X', '').strip()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(TwistStamped, self.topic, self.cb, 50)
        self.count = 0
        self.last_count = 0
        self.tampered_count = 0
        self.max_latency_ms = 0.0
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.get_logger().info(
            f'Robot UDP forwarder started: {self.topic} -> udp://{self.udp_host}:{self.udp_port}'
        )
        if self.tamper_enabled:
            self.get_logger().warn(
                'MITM tamper mode enabled immediately: overriding forwarded angular.z '
                f'to {self.tamper_angular_z}'
            )
        else:
            self.get_logger().info(
                'MITM tamper waits for flag file: '
                f'{self.tamper_flag_file}'
            )

    def cb(self, msg: TwistStamped):
        now = self.get_clock().now()
        sent_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        latency_ms = max((now.nanoseconds - sent_ns) / 1_000_000.0, 0.0)
        self.max_latency_ms = max(self.max_latency_ms, latency_ms)
        self.count += 1
        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z

        tamper_active = self.tamper_enabled or os.path.exists(self.tamper_flag_file)
        if tamper_active:
            angular_z = self.tamper_angular_z
            if self.tamper_linear_x:
                linear_x = float(self.tamper_linear_x)
            self.tampered_count += 1

        payload = (
            f'{time.time():.6f} {linear_x:.6f} {angular_z:.6f} '
            f'{latency_ms:.3f} {self.count}\n'
        ).encode('ascii')
        self.sock.sendto(payload, (self.udp_host, self.udp_port))

    def log_status(self):
        delta = self.count - self.last_count
        self.last_count = self.count
        self.get_logger().info(
            f'forwarded={self.count} rate={delta}/s tampered={self.tampered_count} '
            f'max_ros_latency_ms={self.max_latency_ms:.1f}'
        )


def main():
    rclpy.init()
    node = MitmRobotUdpForwarder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
