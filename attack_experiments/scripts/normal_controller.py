#!/usr/bin/env python3
# Normal controller - simulates a regular user controlling the robot
# Just a simple script that sends commands at a fixed rate

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import argparse
import time
import math


class NormalController(Node):
    """Normal controller - what a regular user would do"""
    
    def __init__(self, node_name='normal_operator'):
        super().__init__(node_name)
        # Publishing to cmd_vel like any normal control node would
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info(f'Normal controller started: {node_name}')
    
    def _create_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """Helper to create a velocity command message"""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Standard frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        return msg
    
    def run_square_pattern(self, frequency=10, speed=0.2, duration=60):
        """Go in a square: forward, turn, forward, turn"""
        self.get_logger().info(f'Square pattern: {speed} m/s @ {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            current_time = time.time() - start_time
            cycle_time = current_time % 8  # 8s cycle: 2s forward, 2s turn, repeat
            
            if cycle_time < 2.0:
                msg = self._create_twist_stamped(linear_x=speed)
            elif cycle_time < 4.0:
                msg = self._create_twist_stamped(angular_z=-0.5)  # Turn right
            elif cycle_time < 6.0:
                msg = self._create_twist_stamped(linear_x=speed)
            else:
                msg = self._create_twist_stamped(angular_z=-0.5)
            
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().info('Square pattern done')
    
    def run_circle_pattern(self, frequency=10, speed=0.2, duration=60):
        """Go in circles"""
        self.get_logger().info(f'Circle pattern: {speed} m/s @ {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            # Forward + slight turn = circle
            msg = self._create_twist_stamped(linear_x=speed, angular_z=0.3)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().info('Circle pattern done')
    
    def run_forward_backward(self, frequency=10, speed=0.2, duration=60):
        """Go forward, then backward, repeat"""
        self.get_logger().info(f'Forward-backward: {speed} m/s @ {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            current_time = time.time() - start_time
            cycle_time = current_time % 4  # 4s cycle: 2s forward, 2s backward
            
            if cycle_time < 2.0:
                msg = self._create_twist_stamped(linear_x=speed)
            else:
                msg = self._create_twist_stamped(linear_x=-speed)
            
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().info('Forward-backward done')
    
    def run_continuous(self, frequency=10, speed=0.2, duration=60):
        """Just keep going straight - simple and predictable"""
        self.get_logger().info(f'Starting continuous forward: {speed} m/s @ {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=speed)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().info('Done - stopped the robot')
    
    def stop(self):
        """Send zero velocity to stop the robot"""
        msg = self._create_twist_stamped()
        self.publisher.publish(msg)
        self.get_logger().info('Stopped')


def main():
    parser = argparse.ArgumentParser(
        description='Normal controller - simulates a regular user',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Square pattern
  python3 normal_controller.py --pattern square --frequency 10 --duration 60
  
  # Circle
  python3 normal_controller.py --pattern circle --frequency 10 --duration 60
  
  # Forward and back
  python3 normal_controller.py --pattern forward_backward --frequency 10 --duration 60
  
  # Just go straight (what we use for experiments)
  python3 normal_controller.py --pattern continuous --frequency 10 --duration 60
        """
    )
    
    parser.add_argument('--pattern', type=str, default='square',
                       choices=['square', 'circle', 'forward_backward', 'continuous'],
                       help='What pattern to run')
    parser.add_argument('--frequency', type=float, default=10.0,
                       help='How often to send commands (Hz)')
    parser.add_argument('--speed', type=float, default=0.2,
                       help='How fast to go (m/s)')
    parser.add_argument('--duration', type=float, default=60.0,
                       help='How long to run (seconds)')
    parser.add_argument('--node-name', type=str, default='normal_operator',
                       help='ROS2 node name')
    
    args = parser.parse_args()
    
    rclpy.init()
    controller = NormalController(node_name=args.node_name)
    
    try:
        if args.pattern == 'square':
            controller.run_square_pattern(frequency=args.frequency, speed=args.speed, duration=args.duration)
        elif args.pattern == 'circle':
            controller.run_circle_pattern(frequency=args.frequency, speed=args.speed, duration=args.duration)
        elif args.pattern == 'forward_backward':
            controller.run_forward_backward(frequency=args.frequency, speed=args.speed, duration=args.duration)
        elif args.pattern == 'continuous':
            controller.run_continuous(frequency=args.frequency, speed=args.speed, duration=args.duration)
        
        time.sleep(0.5)  # Give it a moment to finish
    except KeyboardInterrupt:
        print("\nInterrupted - stopping...")
        controller.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

