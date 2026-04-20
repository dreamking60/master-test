#!/usr/bin/env python3
import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class UdpCmdVelReceiver(Node):
    def __init__(self):
        super().__init__('udp_cmd_vel_receiver')
        self.bind_host = os.environ.get('GAZEBO_UDP_BIND_HOST', '0.0.0.0')
        self.bind_port = int(os.environ.get('GAZEBO_UDP_PORT', '15000'))
        self.topic = os.environ.get('GAZEBO_CMD_TOPIC', '/cmd_vel_in')
        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_host, self.bind_port))
        self.count = 0
        self.last_count = 0
        self.last_udp_latency_ms = 0.0
        self.last_ros_latency_ms = 0.0
        self.max_udp_latency_ms = 0.0
        self.max_ros_latency_ms = 0.0
        self.running = True
        self.thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.thread.start()
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.watchdog = self.create_timer(0.2, self.check_timeout)
        self.last_receive_time = time.monotonic()
        self.stopped = True
        self.get_logger().info(f'UDP receiver started: udp://{self.bind_host}:{self.bind_port} -> {self.topic}')

    def recv_loop(self):
        while self.running:
            try:
                data, _addr = self.sock.recvfrom(512)
            except OSError:
                return
            parts = data.decode('ascii', errors='ignore').strip().split()
            if len(parts) < 4:
                continue
            try:
                sent_wall = float(parts[0])
                linear_x = float(parts[1])
                angular_z = float(parts[2])
                ros_latency_ms = float(parts[3])
            except ValueError:
                continue

            udp_latency_ms = max((time.time() - sent_wall) * 1000.0, 0.0)
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.pub.publish(msg)
            self.count += 1
            self.last_receive_time = time.monotonic()
            self.stopped = False
            self.last_udp_latency_ms = udp_latency_ms
            self.last_ros_latency_ms = ros_latency_ms
            self.max_udp_latency_ms = max(self.max_udp_latency_ms, udp_latency_ms)
            self.max_ros_latency_ms = max(self.max_ros_latency_ms, ros_latency_ms)

    def check_timeout(self):
        if self.stopped:
            return
        if time.monotonic() - self.last_receive_time > 0.5:
            self.pub.publish(Twist())
            self.stopped = True
            self.get_logger().warn('No UDP command for 0.5s; published zero stop command')

    def log_status(self):
        delta = self.count - self.last_count
        self.last_count = self.count
        self.get_logger().info(
            f'received={self.count} rate={delta}/s '
            f'last_udp_latency_ms={self.last_udp_latency_ms:.1f} max_udp_latency_ms={self.max_udp_latency_ms:.1f} '
            f'last_ros_latency_ms={self.last_ros_latency_ms:.1f} max_ros_latency_ms={self.max_ros_latency_ms:.1f}'
        )

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = UdpCmdVelReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
