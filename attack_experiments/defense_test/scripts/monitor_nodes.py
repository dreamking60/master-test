#!/usr/bin/env python3
# Node monitoring tool - detects unexpected publishers on critical topics

import rclpy
from rclpy.node import Node
import time
from datetime import datetime
import sys

class NodeMonitor(Node):
    """Monitor nodes publishing to critical topics"""
    
    def __init__(self, topic_name='/cmd_vel', whitelist=None):
        super().__init__('node_monitor')
        self.topic_name = topic_name
        self.whitelist = whitelist or []
        self.known_publishers = set()
        
        self.get_logger().info(f'Monitoring topic: {topic_name}')
        self.get_logger().info(f'Whitelist: {self.whitelist}')
        
        # Monitor topic info periodically
        self.timer = self.create_timer(2.0, self.check_publishers)
    
    def check_publishers(self):
        """Check for new or unexpected publishers"""
        try:
            # Get topic info
            import subprocess
            result = subprocess.run(
                ['ros2', 'topic', 'info', self.topic_name],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                publishers_found = False
                current_publishers = set()
                
                for i, line in enumerate(lines):
                    if 'Publisher count:' in line:
                        count = int(line.split()[-1])
                        if count > 0:
                            publishers_found = True
                            # Get publisher details from next lines
                            for j in range(i+1, min(i+20, len(lines))):
                                if 'node name:' in lines[j]:
                                    node_name = lines[j].split()[-1]
                                    current_publishers.add(node_name)
                
                # Check for new publishers
                new_publishers = current_publishers - self.known_publishers
                if new_publishers:
                    timestamp = datetime.now().strftime('%H:%M:%S')
                    for pub in new_publishers:
                        if pub not in self.whitelist:
                            self.get_logger().warn(
                                f'[{timestamp}] ALERT: Unexpected publisher detected: {pub}'
                            )
                        else:
                            self.get_logger().info(
                                f'[{timestamp}] Known publisher: {pub}'
                            )
                    self.known_publishers.update(new_publishers)
                
                # Check for removed publishers
                removed = self.known_publishers - current_publishers
                if removed:
                    timestamp = datetime.now().strftime('%H:%M:%S')
                    for pub in removed:
                        self.get_logger().info(
                            f'[{timestamp}] Publisher stopped: {pub}'
                        )
                    self.known_publishers = current_publishers
                    
        except Exception as e:
            self.get_logger().error(f'Error checking publishers: {e}')


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Monitor ROS2 nodes for security')
    parser.add_argument('--topic', type=str, default='/cmd_vel',
                       help='Topic to monitor')
    parser.add_argument('--whitelist', type=str, nargs='+', default=[],
                       help='Whitelist of allowed node names')
    
    args = parser.parse_args()
    
    rclpy.init()
    monitor = NodeMonitor(topic_name=args.topic, whitelist=args.whitelist)
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

