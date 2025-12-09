#!/usr/bin/env python3
"""
TurtleBot3 控制测试脚本
用于替代 teleop_keyboard，直接通过命令行参数控制机器人
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
        self.get_logger().info('TurtleBot 控制器已启动')

    def _create_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """创建 TwistStamped 消息"""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        return msg

    def move_forward(self, speed=0.2, duration=1.0):
        """前进"""
        self.get_logger().info(f'前进 {duration} 秒，速度 {speed}')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def move_backward(self, speed=0.2, duration=1.0):
        """后退"""
        self.get_logger().info(f'后退 {duration} 秒，速度 {speed}')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(linear_x=-speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def turn_left(self, angular_speed=0.5, duration=1.0):
        """左转"""
        self.get_logger().info(f'左转 {duration} 秒，角速度 {angular_speed}')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=angular_speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def turn_right(self, angular_speed=0.5, duration=1.0):
        """右转"""
        self.get_logger().info(f'右转 {duration} 秒，角速度 {angular_speed}')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=-angular_speed)
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def stop(self):
        """停止"""
        self.get_logger().info('停止')
        msg = self._create_twist_stamped()
        self.publisher.publish(msg)

    def spin_attack(self, duration=3.0):
        """旋转攻击（用于测试）"""
        self.get_logger().info(f'旋转攻击 {duration} 秒')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self._create_twist_stamped(angular_z=2.0)  # 快速旋转
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()


def main():
    rclpy.init()
    controller = TurtleBotController()

    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 test_control.py forward [速度] [时长]  # 前进")
        print("  python3 test_control.py backward [速度] [时长]  # 后退")
        print("  python3 test_control.py left [角速度] [时长]    # 左转")
        print("  python3 test_control.py right [角速度] [时长]   # 右转")
        print("  python3 test_control.py stop                    # 停止")
        print("  python3 test_control.py spin [时长]             # 旋转攻击")
        print("")
        print("示例:")
        print("  python3 test_control.py forward 0.2 2.0  # 以 0.2 m/s 前进 2 秒")
        print("  python3 test_control.py left 0.5 1.0     # 以 0.5 rad/s 左转 1 秒")
        print("  python3 test_control.py spin 3.0         # 旋转 3 秒")
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
            print(f"未知命令: {command}")
            print("可用命令: forward, backward, left, right, stop, spin")
    except ValueError as e:
        print(f"参数错误: {e}")
        print("请确保速度和时长参数是数字")
    except KeyboardInterrupt:
        print("\n中断，停止机器人...")
        controller.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

