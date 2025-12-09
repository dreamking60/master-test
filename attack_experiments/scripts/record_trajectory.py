#!/usr/bin/env python3
# Records where the robot goes - subscribes to /odom and saves everything to CSV
# Useful for seeing if the attack actually worked

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import argparse
import time
from datetime import datetime
import sys


class TrajectoryRecorder(Node):
    """Just listens to /odom and writes down where the robot is"""
    
    def __init__(self, output_file, plot=False):
        super().__init__('trajectory_recorder')
        self.output_file = output_file
        self.plot = plot
        self.trajectory = []  # Store all the positions
        self.start_time = None
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Trajectory recorder started. Saving to: {output_file}')
    
    def odom_callback(self, msg):
        """Callback when receiving odometry message"""
        if self.start_time is None:
            self.start_time = time.time()
            self.experiment_start_time = msg.header.stamp
        
        # Calculate relative time from start
        current_time = time.time()
        relative_time = current_time - self.start_time
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract orientation (quaternion to yaw)
        orientation = msg.pose.pose.orientation
        # Simple yaw calculation (for 2D)
        import math
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        # Extract linear and angular velocities
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        angular_z = msg.twist.twist.angular.z
        
        # Record data point
        data_point = {
            'timestamp': current_time,
            'relative_time': relative_time,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z
        }
        
        self.trajectory.append(data_point)
        self.get_logger().debug(f'Recorded: t={relative_time:.2f}s, x={x:.3f}, y={y:.3f}')
    
    def save_to_csv(self):
        """Save trajectory to CSV file"""
        if not self.trajectory:
            self.get_logger().warn('No trajectory data recorded!')
            return
        
        with open(self.output_file, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'relative_time', 'x', 'y', 'z', 
                'yaw', 'linear_x', 'linear_y', 'angular_z'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for point in self.trajectory:
                writer.writerow(point)
        
        self.get_logger().info(f'Trajectory saved to {self.output_file}')
        self.get_logger().info(f'Total data points: {len(self.trajectory)}')
        self.get_logger().info(f'Duration: {self.trajectory[-1]["relative_time"]:.2f} seconds')
    
    def plot_trajectory(self):
        """Plot trajectory if matplotlib is available"""
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            
            if not self.trajectory:
                self.get_logger().warn('No data to plot')
                return
            
            x_coords = [p['x'] for p in self.trajectory]
            y_coords = [p['y'] for p in self.trajectory]
            times = [p['relative_time'] for p in self.trajectory]
            
            # Create figure with subplots
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            
            # Plot 1: Trajectory (x-y plot)
            ax1.plot(x_coords, y_coords, 'b-', linewidth=2, label='Robot Trajectory')
            ax1.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')
            ax1.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')
            ax1.set_xlabel('X Position (m)')
            ax1.set_ylabel('Y Position (m)')
            ax1.set_title('Robot Trajectory (Top View)')
            ax1.grid(True)
            ax1.legend()
            ax1.axis('equal')
            
            # Plot 2: Position over time
            ax2.plot(times, x_coords, 'b-', label='X position', linewidth=2)
            ax2.plot(times, y_coords, 'r-', label='Y position', linewidth=2)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Position (m)')
            ax2.set_title('Position vs Time')
            ax2.grid(True)
            ax2.legend()
            
            plt.tight_layout()
            
            # Save plot
            plot_file = self.output_file.replace('.csv', '_plot.png')
            plt.savefig(plot_file, dpi=150)
            self.get_logger().info(f'Plot saved to {plot_file}')
            
            # Optionally show plot
            # plt.show()
            plt.close()
            
        except ImportError:
            self.get_logger().warn('matplotlib not available, skipping plot generation')
        except Exception as e:
            self.get_logger().error(f'Error plotting: {e}')


def main():
    parser = argparse.ArgumentParser(
        description='Record robot trajectory from /odom topic',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record for 60 seconds
  python3 record_trajectory.py --output trajectory.csv --duration 60
  
  # Record with automatic plotting
  python3 record_trajectory.py --output attack_result.csv --duration 30 --plot
  
  # Record until manually stopped (Ctrl+C)
  python3 record_trajectory.py --output trajectory.csv
        """
    )
    
    parser.add_argument('--output', type=str, default='trajectory.csv',
                       help='Output CSV file path')
    parser.add_argument('--duration', type=float, default=None,
                       help='Recording duration in seconds (None = until Ctrl+C)')
    parser.add_argument('--plot', action='store_true',
                       help='Generate trajectory plot (requires matplotlib)')
    
    args = parser.parse_args()
    
    rclpy.init()
    recorder = TrajectoryRecorder(args.output, plot=args.plot)
    
    try:
        if args.duration:
            # Record for specified duration
            print(f"Recording trajectory for {args.duration} seconds...")
            print("Press Ctrl+C to stop early")
            
            end_time = time.time() + args.duration
            while time.time() < end_time:
                rclpy.spin_once(recorder, timeout_sec=0.1)
        else:
            # Record until interrupted
            print("Recording trajectory... Press Ctrl+C to stop")
            rclpy.spin(recorder)
    
    except KeyboardInterrupt:
        print("\nRecording stopped by user")
    finally:
        # Save data
        recorder.save_to_csv()
        
        # Generate plot if requested
        if args.plot:
            recorder.plot_trajectory()
        
        rclpy.shutdown()
        print(f"\nTrajectory saved to: {args.output}")


if __name__ == '__main__':
    main()

