#!/usr/bin/env python3
"""
TurtleBot3 control test script
Alternative to teleop_keyboard, directly control robot via command-line arguments
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import sys
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info('TurtleBot controller started')

    def _create_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """Create TwistStamped message"""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        return msg

    def move_forward(self, speed=0.2, duration=1.0):
        """Move forward"""
        self.get_logger().info(f'Moving forward for {duration} seconds at {speed} m/s')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def move_backward(self, speed=0.2, duration=1.0):
        """Move backward"""
        self.get_logger().info(f'Moving backward for {duration} seconds at {speed} m/s')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=-speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def turn_left(self, angular_speed=0.5, duration=1.0):
        """Turn left"""
        self.get_logger().info(f'Turning left for {duration} seconds at {angular_speed} rad/s')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=angular_speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def turn_right(self, angular_speed=0.5, duration=1.0):
        """Turn right"""
        self.get_logger().info(f'Turning right for {duration} seconds at {angular_speed} rad/s')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=-angular_speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def stop(self):
        """Stop"""
        self.get_logger().info('Stopped')
        msg = self._create_twist_stamped()
        self.publisher.publish(msg)

    def spin_attack(self, duration=3.0):
        """Spin attack (for testing)"""
        self.get_logger().info(f'Spin attack for {duration} seconds')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=2.0)  # Fast spin
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()


def main():
    rclpy.init()
    controller = TurtleBotController()

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 test_control.py forward [speed] [duration]  # Move forward")
        print("  python3 test_control.py backward [speed] [duration]  # Move backward")
        print("  python3 test_control.py left [angular_speed] [duration]    # Turn left")
        print("  python3 test_control.py right [angular_speed] [duration]   # Turn right")
        print("  python3 test_control.py stop                    # Stop")
        print("  python3 test_control.py spin [duration]             # Spin attack")
        print("")
        print("Examples:")
        print("  python3 test_control.py forward 0.2 2.0  # Move forward at 0.2 m/s for 2 seconds")
        print("  python3 test_control.py left 0.5 1.0     # Turn left at 0.5 rad/s for 1 second")
        print("  python3 test_control.py spin 3.0         # Spin for 3 seconds")
        rclpy.shutdown()
        return

    command = sys.argv[1].lower()

    try:
        if command == 'forward':
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.move_forward(speed, duration)
        elif command == 'backward':
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.move_backward(speed, duration)
        elif command == 'left':
            angular = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.turn_left(angular, duration)
        elif command == 'right':
            angular = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.turn_right(angular, duration)
        elif command == 'stop':
            controller.stop()
        elif command == 'spin':
            duration = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
            controller.spin_attack(duration)
        else:
            print(f"Unknown command: {command}")
            print("Available commands: forward, backward, left, right, stop, spin")
    except ValueError as e:
        print(f"Parameter error: {e}")
        print("Please ensure speed and duration parameters are numbers")
    except KeyboardInterrupt:
        print("\nInterrupted, stopping robot...")
        controller.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
