#!/usr/bin/env python3
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrajectoryTrailSpawner(Node):
    def __init__(self):
        super().__init__('trajectory_trail_spawner')
        self.world_name = os.environ.get('GZ_WORLD_NAME', 'empty')
        self.min_distance = float(os.environ.get('TRAIL_MIN_DISTANCE', '0.08'))
        self.marker_radius = float(os.environ.get('TRAIL_MARKER_RADIUS', '0.07'))
        self.marker_height = float(os.environ.get('TRAIL_MARKER_HEIGHT', '0.012'))
        self.max_markers = int(os.environ.get('TRAIL_MAX_MARKERS', '500'))
        self.color = os.environ.get('TRAIL_COLOR', '1.0 0.12 0.0 1.0')
        self.marker_dir = Path(os.environ.get('TRAIL_MARKER_DIR', '/tmp/tb3_trajectory_trail'))
        self.marker_dir.mkdir(parents=True, exist_ok=True)
        self.last_position = None
        self.marker_count = 0
        self.lock = threading.Lock()
        self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.get_logger().info(
            'Trajectory trail enabled: /odom -> Gazebo orange discs '
            f'min_distance={self.min_distance} radius={self.marker_radius} '
            f'max_markers={self.max_markers} dir={self.marker_dir}'
        )

    def on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        x = float(position.x)
        y = float(position.y)

        with self.lock:
            if self.marker_count >= self.max_markers:
                return

            if self.last_position is not None:
                distance = math.hypot(x - self.last_position[0], y - self.last_position[1])
                if distance < self.min_distance:
                    return

            self.last_position = (x, y)
            self.marker_count += 1
            marker_id = self.marker_count

        threading.Thread(target=self.spawn_marker, args=(marker_id, x, y), daemon=True).start()

    def marker_name(self, marker_id):
        return f'trajectory_trail_{marker_id:04d}'

    def marker_sdf(self, marker_id, x, y):
        name = f'trajectory_trail_{marker_id:04d}'
        z = self.marker_height / 2.0 + 0.01
        return f"""<sdf version='1.7'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{self.marker_radius:.4f}</radius>
            <length>{self.marker_height:.4f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{self.color}</ambient>
          <diffuse>{self.color}</diffuse>
          <emissive>{self.color}</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def spawn_marker(self, marker_id, x, y):
        name = self.marker_name(marker_id)
        sdf = self.marker_sdf(marker_id, x, y)
        sdf_path = self.marker_dir / f'{name}.sdf'
        sdf_path.write_text(sdf, encoding='utf-8')
        request = f'sdf_filename: "{sdf_path}" name: "{name}"'
        service = f'/world/{self.world_name}/create'
        started = time.monotonic()
        result = subprocess.run(
            [
                'gz', 'service',
                '-s', service,
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', request,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )

        if result.returncode == 0:
            if marker_id == 1 or marker_id % 25 == 0:
                self.get_logger().info(f'Spawned trajectory marker {marker_id} at x={x:.2f}, y={y:.2f}')
            return

        if marker_id <= 5:
            elapsed = time.monotonic() - started
            output = (result.stdout + result.stderr).strip().replace('\n', ' | ')
            self.get_logger().warn(
                f'Failed to spawn trajectory marker {marker_id} through {service} '
                f'after {elapsed:.2f}s returncode={result.returncode} output={output[:500]}'
            )


def main():
    rclpy.init()
    node = TrajectoryTrailSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
