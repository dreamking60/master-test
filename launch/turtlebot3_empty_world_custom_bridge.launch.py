#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(tb3_pkg, 'worlds', 'empty_world.world')
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_sdf = os.path.join(tb3_pkg, 'models', f'turtlebot3_{model}', 'model.sdf')
    bridge_yaml = os.path.join(project_root, 'config', 'turtlebot3_burger_bridge_custom.yaml')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_pkg, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', model,
            '-file', model_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
        output='screen',
    )

    relay_cmd = ExecuteProcess(
        cmd=['python3', os.path.join(project_root, 'scripts', 'setup', 'cmd_vel_relay.py')],
        output='screen',
    )

    trail_cmd = ExecuteProcess(
        cmd=['python3', os.path.join(project_root, 'scripts', 'setup', 'trajectory_trail_spawner.py')],
        output='screen',
    )

    # Force physics to run in case GUI starts in paused state.
    unpause_world_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/empty/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'pause: false'
                ],
                output='screen',
                shell=False,
            )
        ],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_pkg, 'models'),
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(relay_cmd)
    ld.add_action(trail_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(unpause_world_cmd)
    ld.add_action(set_env_vars_resources)
    return ld
